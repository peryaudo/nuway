"""System-identification sweeps (M0 §2.6): drive the hero with raw CARLA controls.

Runs against a stack launched with ``configs/profiles/sysid.yaml`` (``world_manager``
only: a generated 3 km straight, no sensors, no controller). This script is the
"controller" of the lockstep gate: it answers every ``/nuway/gt/ego_odom`` tick with a
``ControlCommand`` stamped with that tick **and** writes the raw pedal / steer values
straight to ``/carla/hero/vehicle_control_cmd`` (``control_adapter`` must not run: it
would map ``accel`` through the very table these sweeps are meant to fit).

Modes (each run starts with a ``/nuway/sim/reset`` to the start of the straight):

* ``throttle``: from standstill and from initial speeds, hold a throttle level;
* ``brake``: reach an initial speed, hold a brake level until stopped;
* ``coast``: reach an initial speed, throttle = brake = 0;
* ``steer``: reach a speed, hold it with a PI throttle, apply a steer step for 6 s.

Output: ``data/sysid/<mode>.csv`` (one row per tick) read by ``fit_models.py``.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import rclpy
from carla_msgs.msg import CarlaEgoVehicleControl
from nav_msgs.msg import Odometry
from nuway_msgs.msg import ControlCommand, ResetEvent, TickTimeout, VehicleState
from nuway_msgs.srv import Reset
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock

from nuway_ml.common.frames import (
    TOPIC_CLOCK,
    TOPIC_CONTROL_COMMAND,
    TOPIC_GT_EGO_ODOM,
    TOPIC_RESET_EVENT,
    TOPIC_TICK_TIMEOUT,
    TOPIC_VEHICLE_STATE,
)
from nuway_ml.common.geometry import quaternion_to_yaw, yaw_to_quaternion
from nuway_ml.common.qos import QOS
from nuway_ml.common.tick import TICK_DT_S, tick_index

TOPIC_CARLA_CONTROL = "/carla/hero/vehicle_control_cmd"
CSV_COLUMNS = [
    "mode",
    "run",
    "v0",
    "level",
    "phase",
    "k",
    "t",
    "throttle",
    "brake",
    "steer",
    "x",
    "y",
    "yaw",
    "vx",
    "vy",
    "yaw_rate",
    "wheel_angle",
    "wheel_tick",
]
SETTLE_TICKS = 10  # brake after the teleport while physics settles
CLOCK_SUB_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)
PI_KP = 0.25
PI_KI = 0.02


def _qos(name: str) -> QoSProfile:
    p = QOS[name]
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=p.depth,
        reliability=QoSReliabilityPolicy.RELIABLE
        if p.reliability == "reliable"
        else QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        if p.durability == "transient_local"
        else QoSDurabilityPolicy.VOLATILE,
    )


@dataclass(frozen=True, slots=True)
class RunSpec:
    """One sweep run."""

    mode: str
    run: int
    v0: float  # initial speed to reach before the step, m/s
    level: float  # throttle, brake or steer value held during the step
    hold_ticks: int  # max length of the hold phase
    stop_speed: float = -1.0  # end the hold below this speed (brake, coast)


@dataclass(slots=True)
class RunState:
    """Phase machine of the current run."""

    spec: RunSpec
    phase: str = "settle"
    phase_start_k: int = -1
    integral: float = 0.0
    rows: list[dict[str, float | int | str]] = field(default_factory=list)


class SweepNode(Node):  # type: ignore[misc]  # rclpy.Node has no stubs (03 §7.3)
    """Answers the lockstep gate and writes raw CARLA controls per run phase."""

    def __init__(self, start_xy: tuple[float, float]) -> None:
        """Wire the topics and the reset client."""
        super().__init__("sysid_sweeps")
        self._start_xy = start_xy
        self._run: RunState | None = None
        self._episode_stamp: tuple[int, int] | None = None
        self._episode_seen = False
        self._timeouts = 0
        # gt_publisher publishes ego_odom before vehicle_state, so at odom(k)
        # only wheel(k-1) has arrived; rows are joined by tick after the run.
        self._wheel_by_tick: dict[int, float] = {}
        self._pub_command = self.create_publisher(
            ControlCommand, TOPIC_CONTROL_COMMAND, _qos("stream")
        )
        self._pub_carla = self.create_publisher(
            CarlaEgoVehicleControl,
            TOPIC_CARLA_CONTROL,
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
            ),
        )
        self._sub_odom = self.create_subscription(
            Odometry, TOPIC_GT_EGO_ODOM, self._on_ego_odom, _qos("stream")
        )
        # world_manager waits for a /clock subscriber before its first tick.
        self._sub_clock = self.create_subscription(
            Clock, TOPIC_CLOCK, lambda _msg: None, CLOCK_SUB_QOS
        )
        self._sub_vehicle_state = self.create_subscription(
            VehicleState, TOPIC_VEHICLE_STATE, self._on_vehicle_state, _qos("stream")
        )
        self._sub_reset = self.create_subscription(
            ResetEvent, TOPIC_RESET_EVENT, self._on_reset_event, _qos("event")
        )
        self._sub_timeout = self.create_subscription(
            TickTimeout, TOPIC_TICK_TIMEOUT, self._on_tick_timeout, _qos("event")
        )
        self._reset_client = self.create_client(Reset, "/nuway/sim/reset")

    # ------------------------------------------------------------ callbacks
    def _on_reset_event(self, msg: ResetEvent) -> None:
        self._episode_stamp = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        self._episode_seen = False

    def _on_tick_timeout(self, msg: TickTimeout) -> None:
        self._timeouts += 1
        self.get_logger().warning(
            f"lockstep timeout at tick {tick_index(msg.header.stamp)}"
        )

    def _on_vehicle_state(self, msg: VehicleState) -> None:
        self._wheel_by_tick[tick_index(msg.header.stamp)] = float(msg.steering_angle)

    def _on_ego_odom(self, msg: Odometry) -> None:
        k = tick_index(msg.header.stamp)
        stamp = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        if self._episode_stamp is not None and stamp >= self._episode_stamp:
            self._episode_seen = True
        vx = float(msg.twist.twist.linear.x)
        throttle, brake, steer, phase = 0.0, 1.0, 0.0, "idle"
        run = self._run
        if run is not None and self._episode_seen:
            throttle, brake, steer, phase = self._control(run, k, vx)
            run.rows.append(
                {
                    "mode": run.spec.mode,
                    "run": run.spec.run,
                    "v0": run.spec.v0,
                    "level": run.spec.level,
                    "phase": phase,
                    "k": k,
                    "t": k * TICK_DT_S,
                    "throttle": throttle,
                    "brake": brake,
                    "steer": steer,
                    "x": float(msg.pose.pose.position.x),
                    "y": float(msg.pose.pose.position.y),
                    "yaw": quaternion_to_yaw(
                        np.array(
                            [
                                msg.pose.pose.orientation.x,
                                msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z,
                                msg.pose.pose.orientation.w,
                            ]
                        )
                    ),
                    "vx": vx,
                    "vy": float(msg.twist.twist.linear.y),
                    "yaw_rate": float(msg.twist.twist.angular.z),
                    "wheel_angle": math.nan,  # filled by _join_wheel_angles
                    "wheel_tick": k,
                }
            )
        carla_cmd = CarlaEgoVehicleControl()
        carla_cmd.header.stamp = msg.header.stamp
        carla_cmd.throttle = float(throttle)
        carla_cmd.brake = float(brake)
        carla_cmd.steer = float(steer)
        self._pub_carla.publish(carla_cmd)
        cmd = ControlCommand()
        cmd.header.stamp = msg.header.stamp
        cmd.emergency_stop = phase in ("idle", "settle")
        self._pub_command.publish(cmd)

    # --------------------------------------------------------- phase machine
    def _control(
        self, run: RunState, k: int, vx: float
    ) -> tuple[float, float, float, str]:
        """Advance the phase machine one tick; returns (throttle, brake, steer, phase)."""
        spec = run.spec
        if run.phase_start_k < 0:
            run.phase_start_k = k
        elapsed = k - run.phase_start_k
        if run.phase == "settle":
            if elapsed >= SETTLE_TICKS:
                self._enter(run, "reach" if spec.v0 > 0.0 else "hold", k)
            return 0.0, 1.0, 0.0, "settle"
        if run.phase == "reach":
            if vx < spec.v0:
                return 1.0, 0.0, 0.0, "reach"
            self._enter(run, "hold", k)
            elapsed = 0
        if run.phase == "hold":
            stopped = spec.stop_speed >= 0.0 and elapsed > 5 and vx < spec.stop_speed
            if elapsed >= spec.hold_ticks or stopped:
                run.phase = "done"
            else:
                return (*self._hold_control(run, vx), "hold")
        return 0.0, 1.0, 0.0, "done"

    @staticmethod
    def _enter(run: RunState, phase: str, k: int) -> None:
        run.phase = phase
        run.phase_start_k = k
        run.integral = 0.0

    @staticmethod
    def _hold_control(run: RunState, vx: float) -> tuple[float, float, float]:
        """Raw (throttle, brake, steer) during the hold phase of each mode."""
        spec = run.spec
        if spec.mode == "throttle":
            return spec.level, 0.0, 0.0
        if spec.mode == "brake":
            return 0.0, spec.level, 0.0
        if spec.mode == "coast":
            return 0.0, 0.0, 0.0
        # steer: a PI throttle holds v0 while the steer step is applied
        error = spec.v0 - vx
        run.integral = max(-2.0, min(2.0, run.integral + error))
        throttle = max(0.0, min(1.0, 0.1 + PI_KP * error + PI_KI * run.integral))
        return throttle, 0.0, spec.level

    # ---------------------------------------------------------------- driver
    def reset(self) -> bool:
        """Teleport the hero to the start of the straight (base_link pose)."""
        if not self._reset_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error("reset service unavailable")
            return False
        req = Reset.Request()
        req.spawn_index = -1
        req.clear_traffic = False
        req.traffic_seed = -1
        req.start_pose.position.x = float(self._start_xy[0])
        req.start_pose.position.y = float(self._start_xy[1])
        req.start_pose.position.z = 0.0
        q = yaw_to_quaternion(0.0)
        req.start_pose.orientation.x = float(q[0])
        req.start_pose.orientation.y = float(q[1])
        req.start_pose.orientation.z = float(q[2])
        req.start_pose.orientation.w = float(q[3])
        self._episode_seen = False
        future = self._reset_client.call_async(req)
        t0 = time.monotonic()
        while rclpy.ok() and not future.done() and time.monotonic() - t0 < 200.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        result = future.result() if future.done() else None
        if result is None or not result.ok:
            self.get_logger().error(
                f"reset failed: {result.message if result else 'timeout'}"
            )
            return False
        return True

    def execute(
        self, spec: RunSpec, wall_timeout_s: float
    ) -> list[dict[str, float | int | str]]:
        """Run one sweep to completion and return its tick rows."""
        if not self.reset():
            return []
        self._run = RunState(spec)
        t0 = time.monotonic()
        while rclpy.ok() and self._run.phase != "done":
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - t0 > wall_timeout_s:
                self.get_logger().error(f"run {spec} timed out on wall clock")
                break
        rows = self._run.rows
        self._run = None
        self._join_wheel_angles(rows)
        return rows

    def _join_wheel_angles(self, rows: list[dict[str, float | int | str]]) -> None:
        """Fill ``wheel_angle`` with the vehicle_state of the row's own tick."""
        last = 0.0
        for row in rows:
            k = int(row["k"])
            if k in self._wheel_by_tick:
                last = self._wheel_by_tick[k]
            row["wheel_angle"] = last
        self._wheel_by_tick.clear()

    @property
    def timeouts(self) -> int:
        """Lockstep timeouts observed so far (any means corrupted data)."""
        return self._timeouts


LEVELS = [round(0.1 * i, 1) for i in range(1, 11)]
THROTTLE_V0 = (0.0, 5.0, 10.0, 15.0, 20.0)
BRAKE_V0 = (5.0, 10.0, 20.0, 30.0)
COAST_V0 = (10.0, 20.0, 30.0)
STEER_V0 = (5.0, 10.0, 15.0)
# The M0 grid (0.1, 0.2, 0.4) saturates the tyres above 5 m/s (wheel angles of
# 7-28 deg); the small steps give the linear regime the wheelbase / understeer
# fit needs (decisions log, task 9).
STEER_LEVELS = (0.02, 0.05, 0.1, 0.2, 0.4)


def build_specs(mode: str, hold_s: float, coast_cap_s: float) -> list[RunSpec]:
    """Return the sweep grid of M0 §2.6 for one mode."""
    hold = round(hold_s / TICK_DT_S)
    grids: dict[str, list[tuple[float, float, int, float]]] = {
        "throttle": [(v0, u, hold, -1.0) for v0 in THROTTLE_V0 for u in LEVELS],
        "brake": [
            (v0, b, round(20.0 / TICK_DT_S), 0.2) for v0 in BRAKE_V0 for b in LEVELS
        ],
        "coast": [(v0, 0.0, round(coast_cap_s / TICK_DT_S), 0.5) for v0 in COAST_V0],
        "steer": [
            (v0, sign * level, round(6.0 / TICK_DT_S), -1.0)
            for v0 in STEER_V0
            for level in STEER_LEVELS
            for sign in (1.0, -1.0)
        ],
    }
    if mode not in grids:
        msg = f"unknown mode {mode}"
        raise ValueError(msg)
    return [
        RunSpec(mode, i, v0, level, hold_ticks, stop_speed)
        for i, (v0, level, hold_ticks, stop_speed) in enumerate(grids[mode])
    ]


def write_csv(path: Path, rows: list[dict[str, float | int | str]]) -> None:
    """Write the tick rows of one mode."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--mode", nargs="+", default=["throttle", "brake", "coast", "steer"]
    )
    parser.add_argument("--out", type=Path, default=Path("data/sysid"))
    parser.add_argument(
        "--hold-s", type=float, default=12.0, help="throttle hold length"
    )
    parser.add_argument("--coast-cap-s", type=float, default=40.0)
    parser.add_argument(
        "--start-x", type=float, default=30.0, help="base_link x on the straight"
    )
    parser.add_argument(
        "--lane-y", type=float, default=-1.75, help="lane -1 centre (ROS y)"
    )
    parser.add_argument(
        "--limit", type=int, default=0, help="run only the first N specs per mode"
    )
    args = parser.parse_args(argv)

    rclpy.init()
    node = SweepNode((args.start_x, args.lane_y))
    try:
        for mode in args.mode:
            specs = build_specs(mode, args.hold_s, args.coast_cap_s)
            if args.limit > 0:
                specs = specs[: args.limit]
            rows: list[dict[str, float | int | str]] = []
            t0 = time.monotonic()
            for spec in specs:
                run_rows = node.execute(spec, wall_timeout_s=300.0)
                hold = [r for r in run_rows if r["phase"] == "hold"]
                v_end = float(hold[-1]["vx"]) if hold else math.nan
                print(
                    f"{mode} run {spec.run:2d} v0 {spec.v0:4.1f} level {spec.level:+.1f}: "
                    f"{len(run_rows)} ticks, hold {len(hold)} ticks, v_end {v_end:.2f} m/s"
                )
                rows.extend(run_rows)
            out = args.out / f"{mode}.csv"
            write_csv(out, rows)
            print(
                f"wrote {out}: {len(rows)} rows in {time.monotonic() - t0:.0f} s wall"
            )
        if node.timeouts:
            print(f"WARNING: {node.timeouts} lockstep timeouts during the sweeps")
            return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
