"""Route runner (M1 §3.10): drive a protocol through a running or self-launched stack.

Per town: one ``stack.launch.py`` (its ``carla.town`` overridden through a
temporary profile copy), then for every (route, weather, seed):
``/nuway/sim/reset`` at the first waypoint with the route's traffic seed
(the reset respawns the traffic and publishes ``ResetEvent`` so every node
clears itself), ``/nuway/sim/set_weather``, the whole route once on
``/nuway/route/waypoints``, an MCAP recorder around it, and a spin loop
that feeds every tick to the infraction detectors until the goal, a
route-ending infraction (blocked, timeout, deviation) or a crash.

The runner is not part of the tick barrier: it observes and never answers
a tick, so its wall-clock pace cannot change results. Its own CARLA client
(:mod:`nuway_eval.carla_probe`) only reads.

Per route it writes ``<route>_<weather>_<seed>/`` with ``run.mcap``,
``ego_odom.csv``, ``control_command.csv``, ``control_debug.csv`` and the
reference line(s); ``results.csv`` and ``report.md`` at the top come from
:mod:`nuway_eval.report`.
"""

from __future__ import annotations

import csv
import math
import os
import shutil
import signal
import subprocess
import time
from collections.abc import Callable, Iterable
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
    AgentArray,
    ControlCommand,
    ControlDebug,
    NodeDiag,
    ReferenceLine,
    ResetEvent,
    TickTimeout,
)
from nuway_msgs.srv import Reset, SetWeather
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from nuway_eval.carla_probe import CarlaProbe, vehicle_speeds_near
from nuway_eval.chase_writer import ChaseWriter
from nuway_eval.driving_score import ScoringConfig
from nuway_eval.infractions import InfractionTracker, TickObservation
from nuway_eval.report import RouteResult, percentile
from nuway_ml.common.frames import (
    TOPIC_CONTROL_COMMAND,
    TOPIC_CONTROL_DEBUG,
    TOPIC_DIAG_PREFIX,
    TOPIC_GT_AGENTS,
    TOPIC_GT_EGO_ODOM,
    TOPIC_REFERENCE_LINE,
    TOPIC_RESET_EVENT,
    TOPIC_ROUTE_WAYPOINTS,
    TOPIC_TICK_TIMEOUT,
)
from nuway_ml.common.frenet import ReferenceLine as PyReferenceLine
from nuway_ml.common.geometry import quaternion_to_yaw, yaw_to_quaternion
from nuway_ml.common.qos import QOS
from nuway_ml.common.routes import RouteSpec, path_payload
from nuway_ml.common.tick import tick_index

NODE_NAME = "route_runner"
ODOM_COLUMNS = ("k", "t", "x", "y", "z", "yaw", "vx", "vy", "yaw_rate")
COMMAND_COLUMNS = ("k", "accel", "steering_angle", "emergency_stop")
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
DIAG_NODES = (
    "planner_node",
    "safety_layer_node",
    "mpc_node",
    "behavior_fsm_node",
    "const_vel_node",
    "route_planner_node",
    "world_manager",
)
WEATHER_PRESETS = ("ClearNoon", "WetSunset", "HardRainNight")
# Agent classes that count as vehicles for the min-speed test (Agent.msg).
VEHICLE_CLASSES = (1, 2, 3, 4)
LIGHT_RADIUS_M = 40.0
RECORD_REGEX = (
    r"^/nuway/.*|^/clock$|^/tf$|^/tf_static$|^/carla/hero/vehicle_control_cmd$"
)
SENSOR_REGEX = r"^/carla/hero/.*"


# The input names a diag message may call degraded, for the `degraded_<source>`
# incident kind (docs/02 §8.2); a message naming none falls back to the node.
DEGRADABLE_SOURCES = (
    "fallback_samples",
    "samples",
    "agents",
    "occupancy",
    "behavior",
    "pose",
    "ego_odom",
    "vehicle_state",
    "safe_trajectory",
    "trajectory",
    "reference_line",
    "traffic_lights",
)


def degraded_source(node: str, message: str) -> str:
    """Return the degraded input a diag message names, else the node's name."""
    text = message.lower()
    for source in DEGRADABLE_SOURCES:
        if source in text:
            return source
    return node


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
class RouteRun:
    """One protocol entry: a route under a weather preset with a traffic seed."""

    route: RouteSpec
    weather: str
    seed: int

    @property
    def key(self) -> tuple[str, str, str, int]:
        """The ``results.csv`` key."""
        return (self.route.route_id, self.route.town, self.weather, self.seed)

    @property
    def dir_name(self) -> str:
        """``<route>_<weather>_<seed>``."""
        return f"{self.route.route_id}_{self.weather}_{self.seed}"


@dataclass(frozen=True, slots=True)
class RunLimits:
    """Per-route limits that are the harness's, not the scoring's."""

    goal_radius_m: float = 5.0
    wall_timeout_s: float = 1800.0
    record: bool = True
    record_sensors: bool = False
    chase_cam: bool = False  # write <route>/chase/*.jpg from /nuway/viz/chase_cam


@dataclass(frozen=True, slots=True)
class EvalConfig:
    """The profile's ``eval`` block that the runner needs, plus its CARLA endpoint."""

    host: str = "localhost"
    port: int = 2000
    record: bool = True
    record_sensors: bool = False
    scoring: str = "configs/eval/scoring_lb20.yaml"
    traffic_seed: int = 0
    chase_cam: bool = False
    render: str = "incidents"  # off | incidents | full (docs/02 §8)
    render_stride: int = 10
    incident_window: tuple[int, int] = (40, 20)

    @classmethod
    def from_profile(cls, profile: dict[str, Any]) -> EvalConfig:
        """Read the keys of docs/02 §5."""
        ev = profile.get("eval", {}) or {}
        carla_cfg = profile.get("carla", {}) or {}
        traffic = carla_cfg.get("traffic", {}) or {}
        window = list(ev.get("incident_window", (40, 20)) or (40, 20))
        if len(window) != 2:
            window = [40, 20]
        return cls(
            host=str(carla_cfg.get("host", "localhost")),
            port=int(carla_cfg.get("port", 2000)),
            record=bool(ev.get("record", True)),
            record_sensors=bool(ev.get("record_sensors", False)),
            scoring=str(ev.get("scoring", "configs/eval/scoring_lb20.yaml")),
            traffic_seed=int(traffic.get("seed", 0)),
            chase_cam=bool(ev.get("chase_cam", False)),
            render=str(ev.get("render", "incidents")),
            render_stride=int(ev.get("render_stride", 10)),
            incident_window=(int(window[0]), int(window[1])),
        )


@dataclass
class _Trace:
    """Everything recorded for one episode."""

    odom: list[tuple[Any, ...]] = field(default_factory=list)
    commands: list[tuple[Any, ...]] = field(default_factory=list)
    debug: list[tuple[Any, ...]] = field(default_factory=list)
    lines: list[list[tuple[Any, ...]]] = field(default_factory=list)
    planner_ms: list[float] = field(default_factory=list)
    mpc_ms: list[float] = field(default_factory=list)
    safety_interventions: int = 0
    mpc_failures: int = 0
    timeouts: int = 0
    first_timeout_tick: int = -1
    degraded: list[str] = field(default_factory=list)
    events: list[tuple[int, str]] = field(
        default_factory=list
    )  # (tick, kind) of docs/02 §8.2
    agents: dict[int, dict[str, float]] = field(default_factory=dict)
    visible_ids: set[int] = field(default_factory=set)


class RouteRunnerNode(Node):  # type: ignore[misc]  # rclpy.Node has no stubs (03 §7.3)
    """Observer node: resets, weather, waypoints, recording and per-tick scoring."""

    def __init__(self, probe: CarlaProbe | None = None) -> None:
        """Wire the topics and the service clients."""
        super().__init__(NODE_NAME)
        self._probe = probe
        self._pub_waypoints = self.create_publisher(
            PathMsg, TOPIC_ROUTE_WAYPOINTS, _qos("latched")
        )
        self._sub_odom = self.create_subscription(
            Odometry, TOPIC_GT_EGO_ODOM, self._on_ego_odom, _qos("stream")
        )
        self._sub_command = self.create_subscription(
            ControlCommand, TOPIC_CONTROL_COMMAND, self._on_command, _qos("stream")
        )
        self._sub_debug = self.create_subscription(
            ControlDebug, TOPIC_CONTROL_DEBUG, self._on_control_debug, _qos("stream")
        )
        self._sub_agents = self.create_subscription(
            AgentArray, TOPIC_GT_AGENTS, self._on_agents, _qos("stream")
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
        self._sub_diag = [
            self.create_subscription(
                NodeDiag,
                TOPIC_DIAG_PREFIX + name,
                lambda msg, name=name: self._on_diag(name, msg),
                _qos("diag"),
            )
            for name in DIAG_NODES
        ]
        self._reset_client = self.create_client(Reset, "/nuway/sim/reset")
        self._weather_client = self.create_client(SetWeather, "/nuway/sim/set_weather")
        self._episode_k = -1
        self._episode_id = -1
        self._trace = _Trace()
        self._line: PyReferenceLine | None = None
        self._line_left: np.ndarray | None = None
        self._line_right: np.ndarray | None = None
        self._pending: list[Odometry] = []

    # ------------------------------------------------------------ callbacks
    def _on_reset_event(self, msg: ResetEvent) -> None:
        # Transient-local replays of earlier episodes arrive in no order;
        # episode ids only grow, so anything not newer is a replay.
        if int(msg.episode_id) <= self._episode_id:
            return
        self._episode_id = int(msg.episode_id)
        self._episode_k = tick_index(msg.header.stamp)
        self._trace = _Trace()
        self._line = None
        self._pending = []

    def _on_ego_odom(self, msg: Odometry) -> None:
        if tick_index(msg.header.stamp) < self._episode_k:
            return
        self._pending.append(msg)

    def _on_command(self, msg: ControlCommand) -> None:
        k = tick_index(msg.header.stamp)
        if k < self._episode_k:
            return
        self._trace.commands.append(
            (k, float(msg.accel), float(msg.steering_angle), int(msg.emergency_stop))
        )

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
        self._trace.mpc_ms.append(float(msg.solve_time_ms))

    def _on_agents(self, msg: AgentArray) -> None:
        if tick_index(msg.header.stamp) < self._episode_k:
            return
        agents: dict[int, dict[str, float]] = {}
        visible: set[int] = set()
        for a in msg.agents:
            agents[int(a.id)] = {
                "x": float(a.pose.position.x),
                "y": float(a.pose.position.y),
                "vx": float(a.vx),
                "vy": float(a.vy),
                "is_vehicle": 1.0 if int(a.class_id) in VEHICLE_CLASSES else 0.0,
            }
            if a.visible:
                visible.add(int(a.id))
        self._trace.agents = agents
        self._trace.visible_ids = visible

    def _on_reference_line(self, msg: ReferenceLine) -> None:
        if tick_index(msg.header.stamp) < self._episode_k or len(msg.s) < 2:
            return
        rows = [
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
        self._trace.lines.append(rows)
        pts = np.array([[r[1], r[2]] for r in rows], dtype=np.float64)
        self._line = PyReferenceLine.from_points(pts)
        self._line_left = np.array(msg.left_bound, dtype=np.float64)
        self._line_right = np.array(msg.right_bound, dtype=np.float64)

    def _on_tick_timeout(self, msg: TickTimeout) -> None:
        k = tick_index(msg.header.stamp)
        if k < self._episode_k:
            return
        self._trace.timeouts += 1
        self._trace.events.append((k, "tick_timeout"))
        if self._trace.first_timeout_tick < 0:
            self._trace.first_timeout_tick = k - self._episode_k

    def _on_diag(self, name: str, msg: NodeDiag) -> None:
        if tick_index(msg.header.stamp) < self._episode_k:
            return
        warn = int(msg.status) != NodeDiag.STATUS_OK
        k = tick_index(msg.header.stamp)
        if name == "planner_node":
            self._trace.planner_ms.append(float(msg.cycle_ms))
        elif name == "safety_layer_node" and warn:
            self._trace.safety_interventions += 1
            if msg.message.startswith("intervened"):
                self._trace.events.append((k, "safety_intervention"))
        elif name == "mpc_node" and warn:
            self._trace.mpc_failures += 1
            self._trace.events.append((k, "mpc_failure"))
        # A degradation is latched by a tick timeout (docs/02 §2); the safety
        # layer's one-tick "missing" note at the episode start is not one.
        if self._trace.timeouts > 0 and "degraded" in msg.message:
            tag = f"{name}:{msg.message.split(';')[-1].strip()[:40]}"
            if tag not in self._trace.degraded:
                self._trace.degraded.append(tag)
                self._trace.events.append(
                    (k, f"degraded_{degraded_source(name, msg.message)}")
                )

    # ---------------------------------------------------------------- driver
    def _spin_until(self, done: Callable[[], bool], wait_s: float) -> bool:
        t0 = time.monotonic()
        while rclpy.ok() and not done() and time.monotonic() - t0 < wait_s:
            rclpy.spin_once(self, timeout_sec=0.05)
        return bool(done())

    def reset_to(self, run: RouteRun, wait_s: float) -> bool:
        """Teleport the hero to the route start with the run's seed; wait for the ResetEvent."""
        if not self._reset_client.wait_for_service(timeout_sec=wait_s):
            self.get_logger().error("reset service unavailable")
            return False
        start, nxt = run.route.waypoints[0], run.route.waypoints[1]
        yaw = math.atan2(nxt.y - start.y, nxt.x - start.x)
        req = Reset.Request()
        req.spawn_index = -1
        req.clear_traffic = True
        req.traffic_seed = int(run.seed)
        req.start_pose.position.x = float(start.x)
        req.start_pose.position.y = float(start.y)
        req.start_pose.position.z = float(start.z)
        q = yaw_to_quaternion(yaw)
        req.start_pose.orientation.x = float(q[0])
        req.start_pose.orientation.y = float(q[1])
        req.start_pose.orientation.z = float(q[2])
        req.start_pose.orientation.w = float(q[3])
        future = self._reset_client.call_async(req)
        self._spin_until(future.done, wait_s)
        result = future.result() if future.done() else None
        if result is None or not result.ok:
            self.get_logger().error(
                f"reset failed: {result.message if result else 'timeout'}"
            )
            return False
        wanted = int(result.episode_id)
        return self._spin_until(lambda: self._episode_id >= wanted, wait_s)

    def set_weather(self, preset: str, wait_s: float = 30.0) -> bool:
        """Apply a weather preset through world_manager."""
        if not self._weather_client.wait_for_service(timeout_sec=wait_s):
            self.get_logger().error("set_weather service unavailable")
            return False
        req = SetWeather.Request()
        req.preset = preset
        future = self._weather_client.call_async(req)
        self._spin_until(future.done, wait_s)
        result = future.result() if future.done() else None
        return bool(result is not None and result.ok)

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

    def drive(
        self,
        run: RouteRun,
        limits: RunLimits,
        out_dir: Path,
        config: ScoringConfig,
        result: RouteResult | None = None,
    ) -> RouteResult:
        """Run one protocol entry to its end and return its ``results.csv`` row."""
        result = result or RouteResult(*run.key)
        route_dir = out_dir / run.dir_name
        route_dir.mkdir(parents=True, exist_ok=True)
        wall0 = time.monotonic()
        recorder = _Recorder(route_dir, limits) if limits.record else None
        chase: ChaseWriter | None = None
        try:
            if recorder is not None:
                recorder.start()
            if not self.reset_to(run, wait_s=200.0):
                result.status = "reset_failed"
                return result
            if limits.chase_cam:
                chase = ChaseWriter(self, route_dir, self._episode_k)
            if not self.set_weather(run.weather):
                self.get_logger().warning(f"weather {run.weather} not applied")
            if self._probe is not None and not self._probe.attach():
                self.get_logger().warning("no hero to attach the collision sensor to")
            self.publish_waypoints(run.route)
            self._loop(run, limits, config, result, wall0)
        finally:
            result.wall_time_s = time.monotonic() - wall0
            if chase is not None:
                self.get_logger().info(f"chase camera: {chase.stop()} frames")
            if recorder is not None:
                result.bag_path = recorder.stop()
        self._fill(result)
        self._write_trace(route_dir)
        return result

    def _loop(
        self,
        run: RouteRun,
        limits: RunLimits,
        config: ScoringConfig,
        result: RouteResult,
        wall0: float,
    ) -> None:
        """Spin until the route ends, feeding one TickObservation per ego_odom message."""
        goal = np.array([run.route.goal.x, run.route.goal.y])
        tracker: InfractionTracker | None = None
        route_length = 0.0
        max_s = 0.0
        last_s: float | None = None
        line_seen: PyReferenceLine | None = None
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - wall0 > limits.wall_timeout_s:
                result.status = "wall_timeout"
                return
            if self._line is not line_seen and self._line is not None:
                # A new line (first plan or a reroute): the goal's arc length
                # is the route length; the 50 m extension past it is not.
                line_seen = self._line
                f = self._line.to_frenet(float(goal[0]), float(goal[1]), 60.0)
                route_length = f.s if f is not None else self._line.length
                last_s = None
                if tracker is None:
                    tracker = InfractionTracker(config, route_length)
                    result.route_length_m = route_length
            pending, self._pending = self._pending, []
            if tracker is None or self._line is None:
                continue
            for msg in pending:
                obs = self._observe(msg, last_s, config.min_speed_radius_m)
                if obs.s is not None:
                    last_s = obs.s
                    max_s = max(max_s, obs.s)
                self._trace.odom.append(self._odom_row(msg))
                fired = tracker.update(obs)
                if fired:
                    self.get_logger().info(f"tick {obs.k}: {', '.join(fired)}")
                result.completion = (
                    min(1.0, max_s / route_length) if route_length > 0 else 0.0
                )
                if tracker.ended:
                    result.status = tracker.ended
                    self._finish(tracker, obs.k, result, config)
                    return
                reached = (
                    max_s >= route_length - limits.goal_radius_m
                    and math.hypot(obs.x - goal[0], obs.y - goal[1])
                    <= limits.goal_radius_m
                )
                if reached:
                    result.status = "completed"
                    result.completion = 1.0
                    self._finish(tracker, obs.k, result, config)
                    return
        result.status = "interrupted"

    def _finish(
        self,
        tracker: InfractionTracker,
        k: int,
        result: RouteResult,
        config: ScoringConfig,
    ) -> None:
        """Close the tracker (the last min-speed checkpoint) and score the row."""
        fired = tracker.finish(k)
        if fired:
            self.get_logger().info(f"tick {k}: {', '.join(fired)}")
        result.apply_counts(tracker.counts, config)

    def _observe(
        self, msg: Odometry, last_s: float | None, traffic_radius_m: float
    ) -> TickObservation:
        assert self._line is not None
        p = msg.pose.pose.position
        k = tick_index(msg.header.stamp)
        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        limit = 30.0  # the route-deviation distance
        f = None
        if last_s is not None:
            f = self._line.to_frenet_near(
                p.x, p.y, limit, last_s, back_m=5.0, ahead_m=40.0
            )
        if f is None:
            f = self._line.to_frenet(p.x, p.y, limit)
        s = f.s if f is not None else None
        d = f.d if f is not None else None
        left = right = 1e9
        if (
            f is not None
            and self._line_left is not None
            and self._line_right is not None
        ):
            idx = min(len(self._line_left) - 1, max(0, round(f.s / 0.5)))
            left = float(self._line_left[idx])
            right = float(self._line_right[idx])
        red_lines: list[Any] = []
        signs: list[Any] = []
        hits: list[Any] = []
        if self._probe is not None:
            red_lines = self._probe.red_stop_lines((p.x, p.y), LIGHT_RADIUS_M)
            signs = self._probe.stop_signs_near((p.x, p.y), LIGHT_RADIUS_M)
            visible = self._trace.visible_ids
            hits = self._probe.drain_collisions(lambda i: i in visible)
        traffic = vehicle_speeds_near(
            list(self._trace.agents.values()), (p.x, p.y), traffic_radius_m
        )
        return TickObservation(
            k=k,
            t=t,
            x=float(p.x),
            y=float(p.y),
            speed_mps=speed,
            s=s,
            d=d,
            left_bound_m=left,
            right_bound_m=right,
            red_stop_lines=red_lines,
            stop_signs=signs,
            collisions=hits,
            traffic_speeds_mps=traffic,
        )

    @staticmethod
    def _odom_row(msg: Odometry) -> tuple[Any, ...]:
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        return (
            tick_index(msg.header.stamp),
            float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
            p.x,
            p.y,
            p.z,
            quaternion_to_yaw(np.array([q.x, q.y, q.z, q.w])),
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
        )

    def _fill(self, result: RouteResult) -> None:
        tr = self._trace
        if tr.odom:
            result.sim_time_s = tr.odom[-1][1] - tr.odom[0][1]
            expected = tr.odom[-1][0] - tr.odom[0][0] + 1
            if len(tr.odom) < expected:
                self.get_logger().error(
                    f"lockstep broken: {len(tr.odom)} poses over {expected} ticks; "
                    "restart the CARLA server"
                )
        result.n_safety_interventions = tr.safety_interventions
        result.n_mpc_failures = tr.mpc_failures
        result.planner_p50_ms = percentile(tr.planner_ms, 50)
        result.planner_p99_ms = percentile(tr.planner_ms, 99)
        result.mpc_p50_ms = percentile(tr.mpc_ms, 50)
        result.mpc_p99_ms = percentile(tr.mpc_ms, 99)
        result.n_tick_timeouts = tr.timeouts
        result.first_timeout_tick = tr.first_timeout_tick
        result.degraded_sources = ";".join(tr.degraded)
        result.non_deterministic = tr.timeouts > 0
        result.incidents = sorted({*result.incidents, *tr.events})

    def _write_trace(self, route_dir: Path) -> None:
        tr = self._trace
        for name, cols, rows in (
            ("ego_odom.csv", ODOM_COLUMNS, tr.odom),
            ("control_command.csv", COMMAND_COLUMNS, tr.commands),
            ("control_debug.csv", DEBUG_COLUMNS, tr.debug),
        ):
            with (route_dir / name).open("w", newline="") as f:
                w = csv.writer(f)
                w.writerow(cols)
                w.writerows(rows)
        for i, line in enumerate(tr.lines):
            name = "reference_line.csv" if i == 0 else f"reference_line_{i}.csv"
            with (route_dir / name).open("w", newline="") as f:
                w = csv.writer(f)
                w.writerow(LINE_COLUMNS)
                w.writerows(line)


class _Recorder:
    """``ros2 bag record`` around one route: ``run.mcap`` in the route directory."""

    def __init__(self, route_dir: Path, limits: RunLimits) -> None:
        self._route_dir = route_dir
        self._bag_dir = route_dir / "bag"
        self._limits = limits
        self._proc: subprocess.Popen[bytes] | None = None
        self._log = route_dir / "record.log"

    def start(self) -> None:
        """Start the recorder and give DDS discovery a moment."""
        shutil.rmtree(self._bag_dir, ignore_errors=True)
        regex = RECORD_REGEX
        if self._limits.record_sensors:
            regex += "|" + SENSOR_REGEX
        with self._log.open("w") as log:
            self._proc = subprocess.Popen(
                [
                    "ros2",
                    "bag",
                    "record",
                    "-s",
                    "mcap",
                    # zstd chunks: the GT occupancy grid alone is ~1 MB per
                    # planning tick uncompressed and shrinks ~400x.
                    "--storage-preset-profile",
                    "zstd_fast",
                    "-o",
                    str(self._bag_dir),
                    "-e",
                    regex,
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        time.sleep(2.0)

    def stop(self) -> str:
        """Stop the recorder; returns the relative ``run.mcap`` path (or "")."""
        if self._proc is None:
            return ""
        if self._proc.poll() is None:
            os.killpg(self._proc.pid, signal.SIGINT)
            try:
                self._proc.wait(timeout=30.0)
            except subprocess.TimeoutExpired:
                os.killpg(self._proc.pid, signal.SIGKILL)
                self._proc.wait()
        self._proc = None
        mcaps = sorted(self._bag_dir.glob("*.mcap"))
        if not mcaps:
            return ""
        target = self._route_dir / "run.mcap"
        shutil.move(str(mcaps[0]), target)
        shutil.rmtree(self._bag_dir, ignore_errors=True)
        return f"{self._route_dir.name}/run.mcap"


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
        self._log = log_path.open("a")
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
        """SIGINT `ros2 launch` alone, then SIGKILL the group.

        Launch forwards the SIGINT to every node and escalates by itself;
        signalling the whole group would hand each node a second SIGINT and
        cut world_manager's actor teardown short.
        """
        if self._proc.poll() is None:
            self._proc.send_signal(signal.SIGINT)
            try:
                self._proc.wait(timeout=grace_s)
            except subprocess.TimeoutExpired:
                os.killpg(self._proc.pid, signal.SIGKILL)
                self._proc.wait()
        self._log.close()


def group_by_town(runs: Iterable[RouteRun]) -> dict[str, list[RouteRun]]:
    """Group the runs by town, protocol order preserved."""
    out: dict[str, list[RouteRun]] = {}
    for r in runs:
        out.setdefault(r.route.town, []).append(r)
    return out


def protocol(
    routes: Iterable[RouteSpec], weathers: Iterable[str], seed: int
) -> list[RouteRun]:
    """Every (route, weather) with one traffic seed, routes outermost."""
    ws = list(weathers)
    return [RouteRun(route, w, seed) for route in routes for w in ws]
