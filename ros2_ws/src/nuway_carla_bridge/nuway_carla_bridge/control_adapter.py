"""control_adapter node (M0): ControlCommand -> CarlaEgoVehicleControl.

Subscribes ``/nuway/control/command`` and ``/nuway/sim/vehicle_state`` (for the
current speed), publishes ``/carla/hero/vehicle_control_cmd``. ``accel`` is
mapped to throttle / brake through ``nuway_ml.common.longitudinal_map``
(parity-tested twin of ``nuway_control::LongitudinalMap``), ``steering_angle``
through ``carla_conv.steer_from_ros``; ``emergency_stop`` is brake 1.

Converts every received command immediately: the lockstep gate in
``world_manager`` guarantees exactly one command per tick, so there is no
repeat-last logic and no watchdog (``M0_bringup.md`` §2.3). The pedal split
uses the ``vehicle_state`` stamped with the command's own tick (docs/02 §2:
no node consumes "the latest" message); the few recent ones are kept by
tick, which needs no clearing on reset.

Why an adapter at all. The stack's ``ControlCommand`` is in physical units
(``accel`` in m/s^2, ``steering_angle`` as the front wheel angle in rad, ROS
sign convention), so controllers and planners stay vehicle-agnostic. CARLA
wants what a driver has: throttle and brake pedals in [0, 1] and a steer
in [-1, 1], in its own left-handed sign. Pedals are not accelerations: the
same throttle gives less acceleration at speed, and braking depends on
speed too. The sysid step responses (M0 §2.6) measured those tables for
this vehicle, and ``LongitudinalMap.inverse(speed, accel)`` is the inverse
vehicle model: the pedal pair that produces ``accel`` at ``speed``,
throttle when the wanted acceleration is above the coasting deceleration
and brake below it, saturating at the pedal limits. Steer is a proportion
of ``max_steer_angle``, the wheel angle at full lock.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from carla_msgs.msg import CarlaEgoVehicleControl
from nuway_msgs.msg import ControlCommand, NodeDiag, VehicleState
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from nuway_ml.common.carla_conv import steer_from_ros
from nuway_ml.common.frames import (
    TOPIC_CONTROL_COMMAND,
    TOPIC_DIAG_PREFIX,
    TOPIC_VEHICLE_STATE,
)
from nuway_ml.common.longitudinal_map import LongitudinalMap
from nuway_ml.common.tick import tick_index
from nuway_rclpy.ros_qos import qos

NODE_NAME = "control_adapter"
SPEED_HISTORY_TICKS = 20  # vehicle_state kept by tick (1 s)
# CARLA's native ROS 2 subscriber for the hero (ros_name "hero"); the server
# applies the last message it holds when it computes the next frame.
TOPIC_CARLA_CONTROL = "/carla/hero/vehicle_control_cmd"
# CARLA's native subscriber matched a reliable / volatile publisher on the dev
# box (docs/02 §3.1, control command finding).
CARLA_CONTROL_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


@dataclass(frozen=True, slots=True)
class HoldAtRest:
    """Brake instead of coasting when the car should stand still.

    The pedal map's zero-acceleration throttle at rest is the rolling
    resistance's (0.1-0.2 on the Lincoln), so a controller holding a stop
    with ``accel`` near zero would creep, and on a grade roll back: the task
    11 smoke drive rolled back at 5 cm/s for 300 s at a corner. Below
    ``speed_mps`` with ``accel`` at most ``accel_mps2`` the adapter brakes
    with ``brake`` instead; a positive command releases it.
    """

    speed_mps: float = 0.3
    accel_mps2: float = 0.05
    brake: float = 0.3

    def applies(self, speed_mps: float, accel_mps2: float) -> bool:
        """Whether the stand-still brake replaces the pedal map this tick."""
        return abs(speed_mps) < self.speed_mps and accel_mps2 <= self.accel_mps2


def carla_control_from_command(
    msg: ControlCommand,
    lon_map: LongitudinalMap,
    max_steer_rad: float,
    speed_mps: float,
    hold: HoldAtRest | None = None,
) -> CarlaEgoVehicleControl:
    """Map ``accel`` to pedals at ``speed_mps``, flip the steer sign; e-stop is brake 1.

    Pure function so the unit test can pin it. ``emergency_stop`` is the
    no-input output of the controller (docs/02 §2): full brake, wheels
    straight, whatever the other fields say. Gear 0 with manual shifting off
    leaves CARLA's automatic gearbox in charge; reverse is never used.
    ``hold`` applies the stand-still brake of :class:`HoldAtRest`.
    """
    out = CarlaEgoVehicleControl()
    out.header.stamp = msg.header.stamp
    if msg.emergency_stop:
        out.throttle = 0.0
        out.brake = 1.0
        out.steer = 0.0
    elif hold is not None and hold.applies(speed_mps, float(msg.accel)):
        out.throttle = 0.0
        out.brake = hold.brake
        out.steer = float(steer_from_ros(float(msg.steering_angle), max_steer_rad))
    else:
        throttle, brake = lon_map.inverse(speed_mps, float(msg.accel))
        out.throttle = float(throttle)
        out.brake = float(brake)
        out.steer = float(steer_from_ros(float(msg.steering_angle), max_steer_rad))
    out.hand_brake = False
    out.reverse = False
    out.manual_gear_shift = False
    out.gear = 0
    return out


class ControlAdapterNode(Node):  # type: ignore[misc]  # rclpy.Node has no stubs (03 §7.3)
    """Thin conversion node; the logic is LongitudinalMap and carla_conv.

    Not in the tick barrier: it answers every command as it comes and never
    waits, so it cannot cause a lockstep timeout; its ``NodeDiag`` carries
    the command's stamp for the diagnostics timeline.
    """

    def __init__(self) -> None:
        """Load the vehicle model and wire the topics."""
        super().__init__(NODE_NAME)
        self.declare_parameter("vehicle", "configs/vehicle/lincoln_mkz_2020.yaml")
        self.declare_parameter("hold_speed_mps", 0.3)
        self.declare_parameter("hold_accel_mps2", 0.05)
        self.declare_parameter("hold_brake", 0.3)
        self._hold = HoldAtRest(
            float(self.get_parameter("hold_speed_mps").value),
            float(self.get_parameter("hold_accel_mps2").value),
            float(self.get_parameter("hold_brake").value),
        )
        vehicle_path = Path(str(self.get_parameter("vehicle").value))
        with vehicle_path.open() as f:
            cfg = yaml.safe_load(f)
        self._max_steer_rad = float(cfg["max_steer_angle"])
        self._map = LongitudinalMap.from_dict(cfg["longitudinal_map"])
        # speed by tick; the command of tick k is answered with the speed of
        # tick k (world_manager publishes vehicle_state before it waits for
        # the command, so it is normally already here).
        self._speed_by_tick: dict[int, float] = {}
        self._latest_speed = 0.0
        self._pub_carla = self.create_publisher(
            CarlaEgoVehicleControl, TOPIC_CARLA_CONTROL, CARLA_CONTROL_QOS
        )
        self._pub_diag = self.create_publisher(
            NodeDiag, TOPIC_DIAG_PREFIX + NODE_NAME, qos("diag")
        )
        self._sub_vehicle_state = self.create_subscription(
            VehicleState, TOPIC_VEHICLE_STATE, self._on_vehicle_state, qos("stream")
        )
        self._sub_command = self.create_subscription(
            ControlCommand,
            TOPIC_CONTROL_COMMAND,
            self._on_control_command,
            qos("stream"),
        )
        self.get_logger().info(
            f"vehicle {vehicle_path}, max_steer {self._max_steer_rad:.3f} rad"
        )

    def _on_vehicle_state(self, msg: VehicleState) -> None:
        """Remember the speed of tick k; forget speeds older than 1 s."""
        k = tick_index(msg.header.stamp)
        self._speed_by_tick[k] = float(msg.speed)
        self._latest_speed = float(msg.speed)
        for old in [t for t in self._speed_by_tick if t < k - SPEED_HISTORY_TICKS]:
            del self._speed_by_tick[old]

    def _on_control_command(self, msg: ControlCommand) -> None:
        """Convert and forward one command, using the speed of its own tick.

        The latest speed is the fallback only for a command whose
        ``vehicle_state`` never arrived (a tick that timed out).
        """
        k = tick_index(msg.header.stamp)
        speed = self._speed_by_tick.get(k, self._latest_speed)
        out = carla_control_from_command(
            msg, self._map, self._max_steer_rad, speed, self._hold
        )
        self._pub_carla.publish(out)
        diag = NodeDiag()
        diag.header.stamp = msg.header.stamp
        diag.node = NODE_NAME
        diag.cycle_ms = 0.0
        diag.input_age_ms = 0.0
        diag.status = NodeDiag.STATUS_OK
        diag.message = "emergency_stop" if msg.emergency_stop else ""
        self._pub_diag.publish(diag)


def main(args: list[str] | None = None) -> int:
    """Entry point."""
    rclpy.init(args=args)
    node = ControlAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
