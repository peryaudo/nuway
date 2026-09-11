"""The nuway stack under the official CARLA Leaderboard evaluator (M1 §3.12).

The evaluator owns the CARLA client, the tick, the sensors and the scoring;
this module is the thin ``AutonomousAgent`` between it and the stack, not a
second stack. The pinned Leaderboard has no ROS track: its ``ROS2Agent`` is
a base class that launches the carla_ros_bridge and lets the stack spawn its
own sensors through bridge services, which would put a second client and a
second sensor rig between the runner and us. The agent therefore runs on the
``MAP`` track and does the ROS side itself (M1 Decisions log, task 19); MAP
rather than SENSORS because the stack consumes the HD map through the
``sensor.opendrive_map`` pseudo-sensor, which the runner allows on MAP only:

1. ``sensors()`` returns ``configs/sensors/rig_leaderboard.json`` (the dev
   rig's spawnable entries) plus the OpenDRIVE and speedometer
   pseudo-sensors.
2. ``run_step(input_data, timestamp)`` publishes ``/clock`` for the tick,
   then the sensor payloads on the CARLA-native topic names, types and
   frames (reliable QoS: no camera drops on this path), then
   ``/nuway/sim/vehicle_state`` from the speedometer, and **blocks** until a
   ``ControlCommand`` stamped with that tick arrives. The runner cannot tick
   until the agent has answered, which is the lockstep protocol of docs/02
   §2 with the runner in ``world_manager``'s seat.
3. The **watchdog**: no command within ``lockstep_timeout_s`` (the startup
   timeout on the first tick) publishes ``TickTimeout`` exactly as
   ``world_manager`` would, answers full brake and counts the tick; the
   count and the measured per-tick serialization cost go to a JSON sidecar
   next to the runner's own results.
4. The first tick of a route writes the OpenDRIVE pseudo-sensor's map to
   ``data/maps/<town>/map.xodr`` (``map_server`` waits for it), publishes
   the latched ``/tf_static`` and camera_info of the rig, the route as
   ``/nuway/route/waypoints`` and a ``ResetEvent``, in that order.

Under the runner there is no privileged access of any kind: the agent
refuses any ``use_gt.*: true``. Before M5 the stack has no localization
source here, the controller never fires and every tick is answered by the
watchdog (M1 §3.12): the M1 criterion is the mechanical integration.
"""

from __future__ import annotations

import json
import os
import threading
import time
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
import yaml

if TYPE_CHECKING:
    import carla
    from builtin_interfaces.msg import Time
    from nuway_msgs.msg import ControlCommand
    from nuway_msgs.srv import Reset, SetWeather

    from nuway_ml.common.geometry import Array

try:
    from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track
    from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
except ImportError:  # the pure helpers below are unit-tested without external/

    class AutonomousAgent:  # type: ignore[no-redef]  # stand-in outside the evaluator
        """Stand-in base class when the Leaderboard checkout is absent."""

        def __init__(self, *_args: object, **_kwargs: object) -> None:
            self._global_plan_world_coord: list[Any] = []

    Track = None
    CarlaDataProvider = None

from nuway_ml.common.carla_conv import (
    CarlaLocation,
    gyroscope_to_ros,
    lidar_points_to_ros,
    location_to_ros,
    steer_from_ros,
)
from nuway_ml.common.frames import (
    FRAME_BASE_LINK,
    FRAME_MAP,
    TOPIC_CAMERA_INFO_FMT,
    TOPIC_CLOCK,
    TOPIC_CONTROL_COMMAND,
    TOPIC_DIAG_PREFIX,
    TOPIC_RESET_EVENT,
    TOPIC_ROUTE_WAYPOINTS,
    TOPIC_TICK_TIMEOUT,
    TOPIC_VEHICLE_STATE,
)
from nuway_ml.common.geometry import yaw_to_quaternion
from nuway_ml.common.longitudinal_map import LongitudinalMap
from nuway_ml.common.rig import (
    Rig,
    SensorSpec,
    VehicleGeometry,
    load_rig,
    sensor_in_base_link,
)
from nuway_ml.common.routes import Waypoint, path_payload, waypoints_from_global_plan
from nuway_ml.common.tick import tick_index

NODE_NAME = "leaderboard_agent"
OPENDRIVE_ID = "opendrive"
SPEED_ID = "speed"
TOPIC_LIDAR_FMT = "/carla/hero/{id}/point_cloud"
TOPIC_IMAGE_FMT = "/carla/hero/{id}/image"
TOPIC_IMU = "/carla/hero/imu"
TOPIC_GNSS = "/carla/hero/gnss"
GT_TOGGLES = ("localization", "perception", "traffic_lights", "prediction", "planning")
POINT_STEP = 16  # x, y, z, intensity as float32


def get_entry_point() -> str:
    """Return the agent class name (the evaluator's hook)."""
    return "NuwayAgent"


@dataclass(frozen=True, slots=True)
class AgentConfig:
    """What the agent reads from the profile passed as ``--agent-config``."""

    rig: Rig
    vehicle: VehicleGeometry
    lon_map: LongitudinalMap
    lockstep_timeout_s: float = 2.0
    lockstep_startup_timeout_s: float = 120.0
    map_dir: Path = Path("data/maps")

    @classmethod
    def load(cls, profile_path: Path, repo_root: Path) -> AgentConfig:
        """Read the profile; refuse any ``use_gt.*: true`` (no privileged access under the runner)."""
        with profile_path.open() as f:
            prof = yaml.safe_load(f)
        gt = prof.get("use_gt", {})
        on = [k for k in GT_TOGGLES if bool(gt.get(k, False))]
        if on:
            msg = f"leaderboard_agent refuses use_gt.{on[0]}: true (M1 §3.12)"
            raise ValueError(msg)
        carla_cfg = prof.get("carla", {})
        vehicle_path = repo_root / str(prof["vehicle"])
        return cls(
            rig=load_rig(repo_root / str(prof["sensors"])),
            vehicle=VehicleGeometry.from_yaml(vehicle_path),
            lon_map=LongitudinalMap.from_yaml(vehicle_path),
            lockstep_timeout_s=float(carla_cfg.get("lockstep_timeout_s", 2.0)),
            lockstep_startup_timeout_s=float(
                carla_cfg.get("lockstep_startup_timeout_s", 120.0)
            ),
            map_dir=repo_root / "data/maps",
        )


def runner_sensors(rig: Rig) -> list[dict[str, Any]]:
    """Build the evaluator's sensor list: the rig's spawnable entries plus the two pseudo-sensors.

    Extrinsics are CARLA convention relative to the actor origin, which is
    what both the rig JSON and the evaluator use; the evaluator fixes every
    LiDAR/IMU/GNSS attribute itself and takes only size and FOV of a camera.
    """
    out: list[dict[str, Any]] = []
    for s in rig.select(label_only=False, viz_only=False):
        entry: dict[str, Any] = {
            "type": s.type,
            "id": s.id,
            "x": s.x,
            "y": s.y,
            "z": s.z,
            "roll": s.roll,
            "pitch": s.pitch,
            "yaw": s.yaw,
        }
        if s.is_camera:
            width, height = s.image_size
            entry.update({"width": width, "height": height, "fov": s.fov_deg})
        out.append(entry)
    out.append(
        {"type": "sensor.opendrive_map", "id": OPENDRIVE_ID, "reading_frequency": 1}
    )
    out.append({"type": "sensor.speedometer", "id": SPEED_ID, "reading_frequency": 20})
    return out


@dataclass(slots=True)
class LockstepGate:
    """The tick gate of docs/02 §2: block until the command stamped k arrives, or time out.

    ``offer(k)`` runs on the spin thread, ``wait(k, timeout)`` on the
    evaluator's; the condition variable joins them. A command for a newer
    tick also releases the gate (it can only exist after a timeout let the
    world move on, and then it is the one to honour), as in world_manager.
    """

    latest_k: int = -1
    cond: threading.Condition = field(default_factory=threading.Condition)

    def offer(self, k: int) -> None:
        """Record a command stamped k and wake the waiter."""
        with self.cond:
            self.latest_k = max(self.latest_k, k)
            self.cond.notify_all()

    def wait(self, k: int, timeout_s: float) -> tuple[float, bool]:
        """Return ``(waited_s, timed_out)`` once a command for tick k or later exists."""
        start = time.monotonic()
        deadline = start + timeout_s
        with self.cond:
            while self.latest_k < k:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return time.monotonic() - start, True
                self.cond.wait(timeout=min(remaining, 0.5))
        return time.monotonic() - start, False


@dataclass(frozen=True, slots=True)
class Pedals:
    """A CARLA control as plain numbers (``carla.VehicleControl`` needs the client library)."""

    throttle: float
    brake: float
    steer: float


def pedals_from_command(
    accel_mps2: float,
    steering_angle_rad: float,
    emergency_stop: bool,
    *,
    speed_mps: float,
    lon_map: LongitudinalMap,
    max_steer_rad: float,
) -> Pedals:
    """Map a command to pedals as ``control_adapter`` does (M0 §2.3): accel at speed, CARLA steer sign."""
    steer = float(steer_from_ros(steering_angle_rad, max_steer_rad))
    if emergency_stop:
        return Pedals(0.0, 1.0, steer)
    throttle, brake = lon_map.inverse(speed_mps, accel_mps2)
    return Pedals(float(throttle), float(brake), steer)


def brake_pedals(steer: float) -> Pedals:
    """Return the watchdog's answer: full brake, wheel held where the last command put it."""
    return Pedals(0.0, 1.0, steer)


def route_waypoints(
    global_plan_world_coord: Iterable[tuple[Any, object]],
) -> tuple[Waypoint, ...]:
    """Convert the evaluator's downsampled ``global_plan_world_coord`` to map-frame waypoints."""
    return waypoints_from_global_plan(global_plan_world_coord)


def town_name(map_name: str) -> str:
    """Strip ``Carla/Maps/Town03`` to ``Town03`` (the ``data/maps/<town>/`` key)."""
    return map_name.rsplit("/", maxsplit=1)[-1]


def write_opendrive(map_dir: Path, town: str, opendrive: str) -> Path:
    """Write ``data/maps/<town>/map.xodr`` once, as world_manager does (M0 §2.4)."""
    path = map_dir / town / "map.xodr"
    if not path.is_file():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(opendrive)
    return path


@dataclass(slots=True)
class AgentStats:
    """What the sidecar JSON reports next to the runner's results."""

    ticks: int = 0
    tick_timeouts: int = 0
    serialize_ms: list[float] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        """Summarize: the mean and max serialization cost per tick."""
        ms = np.array(self.serialize_ms, dtype=np.float64)
        return {
            "ticks": self.ticks,
            "tick_timeouts": self.tick_timeouts,
            "serialize_ms_mean": float(ms.mean()) if len(ms) else 0.0,
            "serialize_ms_max": float(ms.max()) if len(ms) else 0.0,
            "serialize_ms_p99": float(np.percentile(ms, 99)) if len(ms) else 0.0,
        }


class NuwayAgent(AutonomousAgent):  # type: ignore[misc]  # the base is Any outside the evaluator
    """The ``MAP``-track agent that fronts the nuway stack (module docstring)."""

    def setup(self, path_to_conf_file: str) -> None:
        """Set up the ROS side; called once per route by the evaluator, before ``sensors()``."""
        # Imported here: the module's pure helpers stay importable without ROS.
        import rclpy  # noqa: PLC0415 -- ROS only inside the evaluator
        from nuway_msgs.msg import (  # noqa: PLC0415
            ControlCommand,
            NodeDiag,
            ResetEvent,
            TickTimeout,
            VehicleState,
        )
        from nuway_msgs.srv import Reset, SetWeather  # noqa: PLC0415
        from rosgraph_msgs.msg import Clock  # noqa: PLC0415
        from sensor_msgs.msg import Image, Imu, NavSatFix, PointCloud2  # noqa: PLC0415
        from tf2_ros import StaticTransformBroadcaster  # noqa: PLC0415

        from nuway_rclpy.ros_qos import CLOCK_QOS, qos  # noqa: PLC0415

        if Track is not None:
            self.track = Track.MAP  # the OpenDRIVE pseudo-sensor is MAP-only
        self._repo_root = Path(
            os.environ.get("NUWAY_ROOT", Path(__file__).resolve().parents[4])
        )
        self._config = AgentConfig.load(Path(path_to_conf_file), self._repo_root)
        self._out_path = Path(
            os.environ.get(
                "NUWAY_LB_AGENT_JSON", "data/eval_runs/leaderboard_agent.json"
            )
        )
        self._gate = LockstepGate()
        self._stats = AgentStats()
        self._episode_id = 0
        self._started = False
        self._last_cmd: Any = None
        self._last_pedals = Pedals(0.0, 1.0, 0.0)
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node(NODE_NAME)
        n = self._node
        self._msg_types = {
            "Clock": Clock,
            "Image": Image,
            "Imu": Imu,
            "NavSatFix": NavSatFix,
            "PointCloud2": PointCloud2,
            "ResetEvent": ResetEvent,
            "TickTimeout": TickTimeout,
            "VehicleState": VehicleState,
            "NodeDiag": NodeDiag,
        }
        self._pub_clock = n.create_publisher(Clock, TOPIC_CLOCK, CLOCK_QOS)
        self._pub_reset = n.create_publisher(
            ResetEvent, TOPIC_RESET_EVENT, qos("event")
        )
        self._pub_timeout = n.create_publisher(
            TickTimeout, TOPIC_TICK_TIMEOUT, qos("event")
        )
        self._pub_state = n.create_publisher(
            VehicleState, TOPIC_VEHICLE_STATE, qos("stream")
        )
        self._pub_diag = n.create_publisher(
            NodeDiag, TOPIC_DIAG_PREFIX + NODE_NAME, qos("diag")
        )
        self._pub_sensors: dict[str, Any] = {}
        for spec in self._config.rig.select(label_only=False, viz_only=False):
            if spec.is_camera:
                self._pub_sensors[spec.id] = n.create_publisher(
                    Image, TOPIC_IMAGE_FMT.format(id=spec.id), qos("stream")
                )
            elif spec.type == "sensor.lidar.ray_cast":
                self._pub_sensors[spec.id] = n.create_publisher(
                    PointCloud2, TOPIC_LIDAR_FMT.format(id=spec.id), qos("stream")
                )
            elif spec.type == "sensor.other.imu":
                self._pub_sensors[spec.id] = n.create_publisher(
                    Imu, TOPIC_IMU, qos("stream")
                )
            elif spec.type == "sensor.other.gnss":
                self._pub_sensors[spec.id] = n.create_publisher(
                    NavSatFix, TOPIC_GNSS, qos("stream")
                )
        self._tf_static = StaticTransformBroadcaster(n)
        self._sub_command = n.create_subscription(
            ControlCommand,
            TOPIC_CONTROL_COMMAND,
            self._on_control_command,
            qos("stream"),
        )
        # The episode services of docs/02 §3.10: the runner owns the episode,
        # so a reset is acknowledged with the current id and weather is refused.
        self._srv_reset = n.create_service(Reset, "/nuway/sim/reset", self._on_reset)
        self._srv_weather = n.create_service(
            SetWeather, "/nuway/sim/set_weather", self._on_set_weather
        )
        self._spin_thread = threading.Thread(target=rclpy.spin, args=(n,), daemon=True)
        self._spin_thread.start()

    def sensors(self) -> list[dict[str, Any]]:
        """Return the rig plus the OpenDRIVE and speedometer pseudo-sensors."""
        return runner_sensors(self._config.rig)

    # ------------------------------------------------------------ callbacks
    def _on_control_command(self, msg: ControlCommand) -> None:
        """Record the command and release the gate for its tick (spin thread)."""
        self._last_cmd = msg
        self._gate.offer(
            tick_index(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        )

    def _on_reset(
        self, _request: Reset.Request, response: Reset.Response
    ) -> Reset.Response:
        """Acknowledge a reset with the current id: the runner owns the episode."""
        response.ok = True
        response.episode_id = self._episode_id
        response.message = "the Leaderboard runner owns the episode"
        return response

    def _on_set_weather(
        self, _request: SetWeather.Request, response: SetWeather.Response
    ) -> SetWeather.Response:
        """Refuse: weather is the route file's under the runner."""
        response.ok = False
        return response

    # ------------------------------------------------------------ the tick
    def run_step(
        self, input_data: dict[str, Any], timestamp: float
    ) -> carla.VehicleControl:
        """Run one lockstep tick: /clock, sensors, vehicle state, then wait for the command."""
        import carla  # noqa: PLC0415 -- only inside the evaluator

        wall0 = time.monotonic()
        k = tick_index(timestamp)
        stamp = self._stamp(timestamp)
        first = not self._started
        if first:
            self._start_episode(input_data, stamp)
        clock = self._msg_types["Clock"]()
        clock.clock = stamp
        self._pub_clock.publish(clock)
        speed = self._publish_sensors(input_data, stamp)
        self._stats.serialize_ms.append(1000.0 * (time.monotonic() - wall0))
        self._stats.ticks += 1

        timeout_s = (
            self._config.lockstep_startup_timeout_s
            if first
            else self._config.lockstep_timeout_s
        )
        waited_s, timed_out = self._gate.wait(k, timeout_s)
        if timed_out:
            self._stats.tick_timeouts += 1
            msg = self._msg_types["TickTimeout"]()
            msg.header.stamp = stamp
            msg.episode_id = self._episode_id
            msg.waited_s = float(waited_s)
            self._pub_timeout.publish(msg)
            self._node.get_logger().error(
                f"lockstep timeout: no ControlCommand for tick {k} within {waited_s:.1f}s"
            )
            pedals = brake_pedals(self._last_pedals.steer)
        else:
            cmd = self._last_cmd
            pedals = pedals_from_command(
                float(cmd.accel),
                float(cmd.steering_angle),
                bool(cmd.emergency_stop),
                speed_mps=speed,
                lon_map=self._config.lon_map,
                max_steer_rad=self._config.vehicle.max_steer_angle,
            )
        self._last_pedals = pedals
        self._publish_diag(stamp, timed_out, waited_s)
        control = carla.VehicleControl()
        control.throttle = pedals.throttle
        control.brake = pedals.brake
        control.steer = pedals.steer
        control.hand_brake = False
        control.reverse = False
        return control

    def _start_episode(self, input_data: dict[str, Any], stamp: Time) -> None:
        """Start the episode on a route's first tick: map file, static rig topics, route, ResetEvent."""
        self._started = True
        self._episode_id += 1
        town = "unknown"
        if CarlaDataProvider is not None:
            town = town_name(CarlaDataProvider.get_map().name)
        if OPENDRIVE_ID in input_data:
            write_opendrive(
                self._config.map_dir, town, input_data[OPENDRIVE_ID][1]["opendrive"]
            )
        self._publish_static_rig(stamp)
        waypoints = route_waypoints(self._global_plan_world_coord)
        self._publish_waypoints(waypoints)
        event = self._msg_types["ResetEvent"]()
        event.header.stamp = stamp
        event.episode_id = self._episode_id
        event.town = town
        if waypoints:
            from nuway_rclpy.ros_conv import pose_from_xyz_yaw  # noqa: PLC0415

            first_pose = path_payload(waypoints)["poses"][0]
            event.start_pose = pose_from_xyz_yaw(
                first_pose["x"], first_pose["y"], first_pose["z"], first_pose["yaw"]
            )
        self._pub_reset.publish(event)

    def _publish_static_rig(self, stamp: Time) -> None:
        """Publish the latched base_link -> sensor transforms and camera_info, as sensor_rig.py does."""
        from geometry_msgs.msg import TransformStamped  # noqa: PLC0415
        from sensor_msgs.msg import CameraInfo  # noqa: PLC0415

        from nuway_carla_bridge.sensor_rig import camera_info_msg  # noqa: PLC0415
        from nuway_rclpy.ros_conv import transform_from_se3  # noqa: PLC0415
        from nuway_rclpy.ros_qos import qos  # noqa: PLC0415

        transforms = []
        self._pub_camera_info: dict[str, Any] = {}
        for spec in self._config.rig.select(label_only=False, viz_only=False):
            msg = TransformStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = FRAME_BASE_LINK
            msg.child_frame_id = spec.id
            msg.transform = transform_from_se3(
                sensor_in_base_link(spec, self._config.vehicle)
            )
            transforms.append(msg)
            if spec.is_camera:
                pub = self._node.create_publisher(
                    CameraInfo,
                    TOPIC_CAMERA_INFO_FMT.format(cam=spec.id),
                    qos("latched"),
                )
                pub.publish(camera_info_msg(spec))
                self._pub_camera_info[spec.id] = pub
        if transforms:
            self._tf_static.sendTransform(transforms)

    def _publish_waypoints(self, waypoints: Sequence[Waypoint]) -> None:
        """Publish the route as a latched ``nav_msgs/Path`` (docs/02 §3.3)."""
        from geometry_msgs.msg import PoseStamped  # noqa: PLC0415
        from nav_msgs.msg import Path as PathMsg  # noqa: PLC0415

        from nuway_rclpy.ros_qos import qos  # noqa: PLC0415

        payload = path_payload(waypoints)
        msg = PathMsg()
        msg.header.frame_id = payload["frame_id"]
        for pose in payload["poses"]:
            ps = PoseStamped()
            ps.header.frame_id = FRAME_MAP
            ps.pose.position.x = pose["x"]
            ps.pose.position.y = pose["y"]
            ps.pose.position.z = pose["z"]
            q = yaw_to_quaternion(pose["yaw"])
            ps.pose.orientation.x = float(q[0])
            ps.pose.orientation.y = float(q[1])
            ps.pose.orientation.z = float(q[2])
            ps.pose.orientation.w = float(q[3])
            msg.poses.append(ps)
        self._pub_waypoints = self._node.create_publisher(
            PathMsg, TOPIC_ROUTE_WAYPOINTS, qos("latched")
        )
        self._pub_waypoints.publish(msg)

    def _publish_sensors(self, input_data: dict[str, Any], stamp: Time) -> float:
        """Republish every payload of this tick and return the speedometer's speed."""
        speed = 0.0
        for spec in self._config.rig.select(label_only=False, viz_only=False):
            if spec.id not in input_data:
                continue
            payload = input_data[spec.id][1]
            pub = self._pub_sensors.get(spec.id)
            if pub is None:
                continue
            if spec.is_camera:
                pub.publish(self._image_msg(spec, payload, stamp))
            elif spec.type == "sensor.lidar.ray_cast":
                pub.publish(self._cloud_msg(spec, payload, stamp))
            elif spec.type == "sensor.other.imu":
                pub.publish(self._imu_msg(spec, payload, stamp))
            elif spec.type == "sensor.other.gnss":
                pub.publish(self._gnss_msg(spec, payload, stamp))
        if SPEED_ID in input_data:
            raw = input_data[SPEED_ID][1].get("speed")
            speed = float(raw) if raw is not None else 0.0
        state = self._msg_types["VehicleState"]()
        state.header.stamp = stamp
        state.speed = speed
        state.steering_angle = 0.0
        state.valid_steering = False
        state.throttle = self._last_pedals.throttle
        state.brake = self._last_pedals.brake
        state.gear = 1
        self._pub_state.publish(state)
        return speed

    def _image_msg(self, spec: SensorSpec, bgra: Array, stamp: Time) -> object:
        """Wrap a CARLA BGRA frame as ``sensor_msgs/Image`` (``bgra8``), frame_id = the sensor id."""
        msg = self._msg_types["Image"]()
        msg.header.stamp = stamp
        msg.header.frame_id = spec.id
        array = np.asarray(bgra, dtype=np.uint8)
        msg.height = int(array.shape[0])
        msg.width = int(array.shape[1])
        msg.encoding = "bgra8"
        msg.is_bigendian = 0
        msg.step = int(array.shape[1]) * 4
        msg.data = array.tobytes()
        return msg

    def _cloud_msg(self, spec: SensorSpec, points: Array, stamp: Time) -> object:
        """Wrap CARLA LiDAR points as ``sensor_msgs/PointCloud2`` (x, y, z, intensity; ROS axes)."""
        from sensor_msgs.msg import PointField  # noqa: PLC0415

        msg = self._msg_types["PointCloud2"]()
        msg.header.stamp = stamp
        msg.header.frame_id = spec.id
        array = lidar_points_to_ros(np.asarray(points, dtype=np.float32).reshape(-1, 4))
        msg.height = 1
        msg.width = int(array.shape[0])
        msg.fields = [
            PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, name in enumerate(("x", "y", "z", "intensity"))
        ]
        msg.is_bigendian = False
        msg.point_step = POINT_STEP
        msg.row_step = POINT_STEP * int(array.shape[0])
        msg.is_dense = True
        msg.data = array.tobytes()
        return msg

    def _imu_msg(self, spec: SensorSpec, data: Array, stamp: Time) -> object:
        """Convert ``[ax, ay, az, gx, gy, gz, compass]`` to ``sensor_msgs/Imu`` (orientation unknown)."""
        msg = self._msg_types["Imu"]()
        msg.header.stamp = stamp
        msg.header.frame_id = spec.id
        arr = np.asarray(data, dtype=np.float64)
        acc = location_to_ros(
            CarlaLocation(float(arr[0]), float(arr[1]), float(arr[2]))
        )
        gyro = gyroscope_to_ros(
            CarlaLocation(float(arr[3]), float(arr[4]), float(arr[5]))
        )
        (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ) = (
            float(acc[0]),
            float(acc[1]),
            float(acc[2]),
        )
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
            float(gyro[0]),
            float(gyro[1]),
            float(gyro[2]),
        )
        msg.orientation_covariance[0] = -1.0
        return msg

    def _gnss_msg(self, spec: SensorSpec, data: Array, stamp: Time) -> object:
        """Convert ``[lat, lon, alt]`` to ``sensor_msgs/NavSatFix``."""
        msg = self._msg_types["NavSatFix"]()
        msg.header.stamp = stamp
        msg.header.frame_id = spec.id
        arr = np.asarray(data, dtype=np.float64)
        msg.latitude = float(arr[0])
        msg.longitude = float(arr[1])
        msg.altitude = float(arr[2])
        return msg

    def _publish_diag(self, stamp: Time, timed_out: bool, waited_s: float) -> None:
        """Publish the node's ``NodeDiag`` (docs/02 §7): the serialization cost and the gate's wait."""
        diag = self._msg_types["NodeDiag"]()
        diag.header.stamp = stamp
        diag.node = NODE_NAME
        diag.cycle_ms = float(self._stats.serialize_ms[-1])
        diag.input_age_ms = float(1000.0 * waited_s)
        diag.status = int(diag.STATUS_ERROR if timed_out else diag.STATUS_OK)
        diag.message = "lockstep timeout" if timed_out else ""
        self._pub_diag.publish(diag)

    @staticmethod
    def _stamp(timestamp: float) -> Time:
        """Return the tick's stamp (``builtin_interfaces/Time``) from the evaluator's game time."""
        from nuway_rclpy.ros_conv import stamp_from_seconds  # noqa: PLC0415

        return stamp_from_seconds(timestamp)

    def destroy(self) -> None:
        """Write the sidecar and take the node down at the end of the route."""
        import rclpy  # noqa: PLC0415 -- ROS only inside the evaluator

        self._out_path.parent.mkdir(parents=True, exist_ok=True)
        self._out_path.write_text(json.dumps(self._stats.as_dict(), indent=2) + "\n")
        self._node.get_logger().info(f"agent stats: {self._stats.as_dict()}")
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self._spin_thread.join(timeout=5.0)
