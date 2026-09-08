"""Route runner v0 (M0 §2.11): drive route XMLs through a running or self-launched stack.

Per town: launch ``stack.launch.py`` with the profile (its ``carla.town``
overridden through a temporary profile copy), then for every route call
``/nuway/sim/reset`` at the first waypoint (heading along the route), publish
the whole route on ``/nuway/route/waypoints`` (latched, unstamped), and spin
until the ego is within ``goal_radius_m`` of the last waypoint after driving
``min_progress`` of the route's length, or a limit
hits. Lateral error comes from ``/nuway/control/debug`` (ticks with
``solver_ok`` only); every ``/nuway/gt/ego_odom`` and debug message of the
route is written to CSV so the lockstep criterion (bit-identical odometry
across two runs, M0 completion criteria) can be checked offline.

The runner is not part of the tick barrier: it observes and never answers a
tick, so its wall-clock pace cannot change results.
"""

from __future__ import annotations

import csv
import math
import os
import signal
import subprocess
import time
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from nuway_msgs.msg import (
    ControlDebug,
    NodeDiag,
    ReferenceLine,
    ResetEvent,
    TickTimeout,
)
from nuway_msgs.srv import Reset
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from nuway_ml.common.frames import (
    TOPIC_CONTROL_DEBUG,
    TOPIC_DIAG_PREFIX,
    TOPIC_GT_EGO_ODOM,
    TOPIC_REFERENCE_LINE,
    TOPIC_RESET_EVENT,
    TOPIC_ROUTE_WAYPOINTS,
    TOPIC_TICK_TIMEOUT,
)
from nuway_ml.common.geometry import quaternion_to_yaw, yaw_to_quaternion
from nuway_ml.common.qos import QOS
from nuway_ml.common.routes import RouteSpec, path_payload, route_length_m
from nuway_ml.common.tick import tick_index

NODE_NAME = "route_runner"
ODOM_COLUMNS = ("k", "t", "x", "y", "z", "yaw", "vx", "vy", "yaw_rate")
LINE_COLUMNS = ("s", "x", "y", "heading", "curvature", "speed_limit", "lane_id")
DEBUG_COLUMNS = (
    "k",
    "lateral_error",
    "heading_error",
    "speed_error",
    "lookahead",
    "solve_time_ms",
    "solver_ok",
)
RESULT_COLUMNS = (
    "route_id",
    "town",
    "status",
    "length_m",
    "driven_m",
    "sim_s",
    "wall_s",
    "ticks",
    "lat_mean_m",
    "lat_abs_mean_m",
    "lat_max_m",
    "heading_max_rad",
    "speed_mean_mps",
    "emergency_ticks",
    "timeouts",
    "reroutes",
)


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
class RunLimits:
    """Per-route limits."""

    goal_radius_m: float = 5.0
    # Reaching the goal counts only after this share of the route's chord
    # length has been driven: routes loop back past their own end.
    min_progress: float = 0.9
    wall_timeout_s: float = 900.0
    min_avg_speed_mps: float = 3.0  # sim limit = length / this + 60 s
    stall_s: float = 60.0  # sim seconds without 1 m of progress -> blocked

    def sim_timeout_s(self, length_m: float) -> float:
        """Sim-time budget of a route."""
        return length_m / self.min_avg_speed_mps + 60.0


@dataclass
class RouteResult:
    """One row of results.csv."""

    route_id: str
    town: str
    status: str = "not_run"
    length_m: float = 0.0
    driven_m: float = 0.0
    sim_s: float = 0.0
    wall_s: float = 0.0
    ticks: int = 0
    lat_mean_m: float = 0.0
    lat_abs_mean_m: float = 0.0
    lat_max_m: float = 0.0
    heading_max_rad: float = 0.0
    speed_mean_mps: float = 0.0
    emergency_ticks: int = 0
    timeouts: int = 0
    reroutes: int = 0

    def row(self) -> dict[str, Any]:
        """CSV row."""
        return {c: getattr(self, c) for c in RESULT_COLUMNS}


@dataclass
class _Trace:
    odom: list[tuple[Any, ...]] = field(default_factory=list)
    debug: list[tuple[Any, ...]] = field(default_factory=list)
    lines: list[list[tuple[Any, ...]]] = field(default_factory=list)  # one per (re)plan
    timeouts: int = 0
    reroutes: int = 0


class RouteRunnerNode(Node):  # type: ignore[misc]  # rclpy.Node has no stubs (03 §7.3)
    """Observer node: resets, publishes waypoints, records odometry and control debug."""

    def __init__(self) -> None:
        """Wire the topics and the reset client."""
        super().__init__(NODE_NAME)
        self._pub_waypoints = self.create_publisher(
            PathMsg, TOPIC_ROUTE_WAYPOINTS, _qos("latched")
        )
        self._sub_odom = self.create_subscription(
            Odometry, TOPIC_GT_EGO_ODOM, self._on_ego_odom, _qos("stream")
        )
        self._sub_debug = self.create_subscription(
            ControlDebug, TOPIC_CONTROL_DEBUG, self._on_control_debug, _qos("stream")
        )
        self._sub_reset = self.create_subscription(
            ResetEvent, TOPIC_RESET_EVENT, self._on_reset_event, _qos("event")
        )
        self._sub_timeout = self.create_subscription(
            TickTimeout, TOPIC_TICK_TIMEOUT, self._on_tick_timeout, _qos("event")
        )
        self._sub_reference_line = self.create_subscription(
            ReferenceLine,
            TOPIC_REFERENCE_LINE,
            self._on_reference_line,
            _qos("latched"),
        )
        self._sub_route_diag = self.create_subscription(
            NodeDiag,
            TOPIC_DIAG_PREFIX + "route_planner_node",
            self._on_route_diag,
            _qos("diag"),
        )
        self._reset_client = self.create_client(Reset, "/nuway/sim/reset")
        self._episode_k = -1
        self._episode_id = -1  # ResetEvent.episode_id of the current episode
        self._trace = _Trace()
        self._last_xy: np.ndarray | None = None

    # ------------------------------------------------------------ callbacks
    def _on_reset_event(self, msg: ResetEvent) -> None:
        # The event topic is transient_local (depth 10): a fresh subscription
        # replays earlier episodes' events, in no guaranteed order relative to
        # the new one. Episode ids only grow, so anything not newer is a replay.
        if int(msg.episode_id) <= self._episode_id:
            return
        self._episode_id = int(msg.episode_id)
        self._episode_k = tick_index(msg.header.stamp)
        self._trace = _Trace()
        self._last_xy = None

    def _on_ego_odom(self, msg: Odometry) -> None:
        k = tick_index(msg.header.stamp)
        if k < self._episode_k:
            return
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(np.array([q.x, q.y, q.z, q.w]))
        self._trace.odom.append(
            (
                k,
                float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
                p.x,
                p.y,
                p.z,
                yaw,
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.angular.z,
            )
        )
        self._last_xy = np.array([p.x, p.y])

    def _on_control_debug(self, msg: ControlDebug) -> None:
        k = tick_index(msg.header.stamp)
        if k < self._episode_k:
            return
        self._trace.debug.append(
            (
                k,
                float(msg.lateral_error),
                float(msg.heading_error),
                float(msg.speed_error),
                float(msg.lookahead),
                float(msg.solve_time_ms),
                int(msg.solver_ok),
            )
        )

    def _on_reference_line(self, msg: ReferenceLine) -> None:
        if tick_index(msg.header.stamp) < self._episode_k:
            return
        self._trace.lines.append(
            [
                (
                    float(msg.s[i]),
                    msg.points[i].x,
                    msg.points[i].y,
                    float(msg.heading[i]),
                    float(msg.curvature[i]),
                    float(msg.speed_limit[i]),
                    int(msg.lane_id[i]),
                )
                for i in range(len(msg.s))
            ]
        )

    def _on_tick_timeout(self, msg: TickTimeout) -> None:
        if tick_index(msg.header.stamp) >= self._episode_k:
            self._trace.timeouts += 1

    def _on_route_diag(self, msg: NodeDiag) -> None:
        if msg.message.startswith("reroute"):
            self._trace.reroutes += 1

    # ---------------------------------------------------------------- driver
    def reset_to(self, route: RouteSpec, wait_s: float) -> bool:
        """Teleport the hero to the route start (base_link pose) and wait for the ResetEvent."""
        if not self._reset_client.wait_for_service(timeout_sec=wait_s):
            self.get_logger().error("reset service unavailable")
            return False
        start, nxt = route.waypoints[0], route.waypoints[1]
        yaw = math.atan2(nxt.y - start.y, nxt.x - start.x)
        req = Reset.Request()
        req.spawn_index = -1
        req.clear_traffic = True
        req.traffic_seed = -1
        req.start_pose.position.x = float(start.x)
        req.start_pose.position.y = float(start.y)
        req.start_pose.position.z = float(start.z)
        q = yaw_to_quaternion(yaw)
        req.start_pose.orientation.x = float(q[0])
        req.start_pose.orientation.y = float(q[1])
        req.start_pose.orientation.z = float(q[2])
        req.start_pose.orientation.w = float(q[3])
        future = self._reset_client.call_async(req)
        t0 = time.monotonic()
        while rclpy.ok() and not future.done() and time.monotonic() - t0 < wait_s:
            rclpy.spin_once(self, timeout_sec=0.05)
        result = future.result() if future.done() else None
        if result is None or not result.ok:
            self.get_logger().error(
                f"reset failed: {result.message if result else 'timeout'}"
            )
            return False
        # Wait for that episode's ResetEvent (it may already have arrived).
        wanted = int(result.episode_id)
        while (
            rclpy.ok() and self._episode_id < wanted and time.monotonic() - t0 < wait_s
        ):
            rclpy.spin_once(self, timeout_sec=0.05)
        return self._episode_id >= wanted

    def publish_waypoints(self, route: RouteSpec) -> None:
        """Publish the whole route, unstamped (accepted for any episode, docs/02 §3.3)."""
        payload = path_payload(route.waypoints)
        msg = PathMsg()
        msg.header.frame_id = payload["frame_id"]
        for pose in payload["poses"]:
            ps = PoseStamped()
            ps.header.frame_id = payload["frame_id"]
            ps.pose.position.x = pose["x"]
            ps.pose.position.y = pose["y"]
            ps.pose.position.z = pose["z"]
            q = yaw_to_quaternion(pose["yaw"])
            ps.pose.orientation.x = float(q[0])
            ps.pose.orientation.y = float(q[1])
            ps.pose.orientation.z = float(q[2])
            ps.pose.orientation.w = float(q[3])
            msg.poses.append(ps)
        self._pub_waypoints.publish(msg)

    def drive(self, route: RouteSpec, limits: RunLimits, out_dir: Path) -> RouteResult:
        """Run one route to completion and return its result row."""
        result = RouteResult(route.route_id, route.town)
        result.length_m = route_length_m(route.waypoints)
        wall0 = time.monotonic()
        if not self.reset_to(route, wait_s=200.0):
            result.status = "reset_failed"
            return result
        self.publish_waypoints(route)
        goal = np.array([route.goal.x, route.goal.y])
        progress_xy: np.ndarray | None = None
        progress_t = 0.0
        driven_m = 0.0
        prev_xy: np.ndarray | None = None
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - wall0 > limits.wall_timeout_s:
                result.status = "wall_timeout"
                break
            if not self._trace.odom:
                continue
            sim_s = self._trace.odom[-1][1] - self._trace.odom[0][1]
            xy = self._last_xy
            assert xy is not None
            if prev_xy is not None:
                driven_m += float(np.linalg.norm(xy - prev_xy))
            prev_xy = xy
            if (
                np.linalg.norm(xy - goal) <= limits.goal_radius_m
                and driven_m >= limits.min_progress * result.length_m
            ):
                result.status = "completed"
                break
            if progress_xy is None or float(np.linalg.norm(xy - progress_xy)) >= 1.0:
                progress_xy, progress_t = xy, sim_s
            elif sim_s - progress_t > limits.stall_s:
                result.status = "blocked"
                break
            if sim_s > limits.sim_timeout_s(result.length_m):
                result.status = "sim_timeout"
                break
        self._fill(result, wall0)
        self._write_trace(out_dir / route.route_id)
        return result

    def _fill(self, result: RouteResult, wall0: float) -> None:
        odom = self._trace.odom
        debug = self._trace.debug
        result.wall_s = time.monotonic() - wall0
        result.ticks = len(odom)
        result.timeouts = self._trace.timeouts
        result.reroutes = self._trace.reroutes
        if odom:
            xy = np.array([[o[2], o[3]] for o in odom])
            result.driven_m = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
            result.sim_s = odom[-1][1] - odom[0][1]
            result.speed_mean_mps = float(np.mean([o[6] for o in odom]))
            expected_ticks = odom[-1][0] - odom[0][0] + 1
            if len(odom) < expected_ticks:
                # One ego_odom per tick is the lockstep contract; fewer means
                # the server advanced on its own (world_manager logs it) and
                # the lateral numbers are not the stack's.
                self.get_logger().error(
                    f"lockstep broken: {len(odom)} poses over {expected_ticks} "
                    "ticks; restart the CARLA server"
                )
        if debug:
            ok = np.array([d[6] for d in debug], dtype=bool)
            lat = np.array([d[1] for d in debug])[ok]
            head = np.array([d[2] for d in debug])[ok]
            result.emergency_ticks = int((~ok).sum())
            if lat.size:
                result.lat_mean_m = float(lat.mean())
                result.lat_abs_mean_m = float(np.abs(lat).mean())
                result.lat_max_m = float(np.abs(lat).max())
                result.heading_max_rad = float(np.abs(head).max())

    def _write_trace(self, route_dir: Path) -> None:
        route_dir.mkdir(parents=True, exist_ok=True)
        with (route_dir / "ego_odom.csv").open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(ODOM_COLUMNS)
            w.writerows(self._trace.odom)
        with (route_dir / "control_debug.csv").open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(DEBUG_COLUMNS)
            w.writerows(self._trace.debug)
        for i, line in enumerate(self._trace.lines):
            name = "reference_line.csv" if i == 0 else f"reference_line_{i}.csv"
            with (route_dir / name).open("w", newline="") as f:
                w = csv.writer(f)
                w.writerow(LINE_COLUMNS)
                w.writerows(line)


# ------------------------------------------------------------------ stacks
def town_profile(profile_path: Path, town: str, out_dir: Path) -> Path:
    """Copy of the profile with ``carla.town`` set, written into the run directory."""
    with profile_path.open() as f:
        prof = yaml.safe_load(f)
    prof.setdefault("carla", {})["town"] = town
    path = out_dir / f"profile_{town}.yaml"
    with path.open("w") as f:
        yaml.safe_dump(prof, f, sort_keys=False)
    return path


class StackProcess:
    """One ``ros2 launch nuway_bringup stack.launch.py`` per town."""

    def __init__(self, profile_path: Path, log_path: Path) -> None:
        """Start the launch in its own process group."""
        self._log = log_path.open("w")
        self._proc = subprocess.Popen(
            [
                "ros2",
                "launch",
                "nuway_bringup",
                "stack.launch.py",
                f"profile:={profile_path}",
                "foxglove:=false",
            ],
            stdout=self._log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def alive(self) -> bool:
        """Return False once `ros2 launch` has exited (e.g. world_manager died with CARLA)."""
        return self._proc.poll() is None

    def stop(self, grace_s: float = 20.0) -> None:
        """SIGINT the process group (launch forwards it), then SIGKILL."""
        if self._proc.poll() is None:
            os.killpg(self._proc.pid, signal.SIGINT)
            try:
                self._proc.wait(timeout=grace_s)
            except subprocess.TimeoutExpired:
                os.killpg(self._proc.pid, signal.SIGKILL)
                self._proc.wait()
        self._log.close()


def group_by_town(routes: Iterable[RouteSpec]) -> dict[str, list[RouteSpec]]:
    """Routes grouped by town, file order preserved."""
    out: dict[str, list[RouteSpec]] = {}
    for r in routes:
        out.setdefault(r.town, []).append(r)
    return out


def write_results(results: Sequence[RouteResult], out_dir: Path) -> None:
    """results.csv + report.md (v0: the table and the M0 criteria check)."""
    with (out_dir / "results.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=RESULT_COLUMNS)
        w.writeheader()
        for r in results:
            w.writerow(r.row())
    lines = ["# Route run report (v0)", "", format_table(results), ""]
    done = [r for r in results if r.status == "completed"]
    lines.append(f"Completed {len(done)} / {len(results)} routes.")
    if done:
        lines.append(
            f"Lateral error over completed routes: |mean| max {max(r.lat_abs_mean_m for r in done):.3f} m, "
            f"max {max(r.lat_max_m for r in done):.3f} m; "
            f"TickTimeouts {sum(r.timeouts for r in done)}."
        )
    (out_dir / "report.md").write_text("\n".join(lines) + "\n")


def format_table(results: Sequence[RouteResult]) -> str:
    """Markdown table of the result rows."""
    cols = (
        "route_id",
        "town",
        "status",
        "length_m",
        "driven_m",
        "sim_s",
        "lat_abs_mean_m",
        "lat_max_m",
        "speed_mean_mps",
        "emergency_ticks",
        "timeouts",
        "reroutes",
    )
    head = "| " + " | ".join(cols) + " |"
    sep = "|" + "|".join("---" for _ in cols) + "|"
    rows = []
    for r in results:
        cells = []
        for c in cols:
            v = getattr(r, c)
            cells.append(f"{v:.3f}" if isinstance(v, float) else str(v))
        rows.append("| " + " | ".join(cells) + " |")
    return "\n".join([head, sep, *rows])
