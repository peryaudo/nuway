#!/usr/bin/env python3
"""Render a route's MCAP into PNG frames, contact sheets and incident sheets.

docs/02 §8: this runs anywhere the repo checks out — no display, no CARLA,
no ROS environment and no ``rclpy``. ``rosbags`` decodes the bag from the
message schemas embedded in the MCAP; ``nuway_ml.viz`` draws.

    render_bag.py --bag <route>/run.mcap [--out <dir>] [--stride N] [--ticks a:b]
                  [--layers L ...]                       # frames/ + sheets/
    render_bag.py --bag ... --incident TICK:KIND ... [--window B:A]   # incidents/

The bag is read twice: a first pass over the light per-tick topics (pose,
command, behavior, diag, lights, resets, the latched map and line) fixes the
tick range and the per-frame header facts; the second pass streams the heavy
topics (occupancy, candidates, agents, samples, trajectories) and renders a
frame as soon as every producer of its tick has been passed, so memory holds a
handful of ticks whatever the route length. Every recorded message starts with
a ``std_msgs/Header``, whose stamp is read straight from the CDR bytes, so a
1 MB occupancy grid of an unwanted tick is never deserialized.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
from collections.abc import Callable, Iterable, Iterator, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, TypeVar

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from nuway_ml.common.tick import TICK_DT_S, tick_index
from nuway_ml.viz import style
from nuway_ml.viz.contact_sheet import FRAMES_PER_SHEET, tile, write_png
from nuway_ml.viz.panel import render_frame
from nuway_ml.viz.scene import (
    AgentBox,
    Candidates,
    EgoPose,
    LaneMap,
    OccupancyRaster,
    Predictions,
    RefLine,
    Scene,
    StopLine,
)
from nuway_ml.viz.scene import (
    Path as VizPath,
)

Msg = Any  # a rosbags message object; the typestore builds them without stubs
T = TypeVar("T")
Frame = np.ndarray

TOPIC_POSE = "/nuway/loc/pose"
TOPIC_COMMAND = "/nuway/control/command"
TOPIC_BEHAVIOR = "/nuway/planning/behavior"
TOPIC_RESET = "/nuway/sim/reset_event"
TOPIC_TIMEOUT = "/nuway/sim/tick_timeout"
TOPIC_LIGHTS = "/nuway/perception/traffic_lights"
TOPIC_LANE_GRAPH = "/nuway/map/lane_graph"
TOPIC_REFERENCE_LINE = "/nuway/route/reference_line"
DIAG_PREFIX = "/nuway/diag/"
# Heavy per-tick topics -> the Scene layer they fill.
HEAVY_TOPICS: dict[str, str] = {
    "/nuway/perception/occupancy": "occupancy",
    "/nuway/perception/agents": "agents",
    "/nuway/gt/agents": "gt_agents",
    "/nuway/prediction/samples": "predictions",
    "/nuway/planning/candidates": "candidates",
    "/nuway/planning/trajectory": "trajectory",
    "/nuway/planning/safe_trajectory": "safe_trajectory",
    "/nuway/control/horizon": "mpc_horizon",
}
# A frame at tick k takes a layer's message from k, k-1 or k-2 (10 Hz producers
# publish on even ticks only).
LOOKBACK_TICKS = 2
# A tick's frame is rendered once the stream is this far past it: every
# producer of tick k has published by then (recorded in receive order).
FLUSH_LAG_TICKS = 6
LATERAL = {0: "keep", 1: "change_left", 2: "change_right"}
LONGITUDINAL = {0: "free", 1: "follow", 2: "yield", 3: "stop"}
LIGHT_STATES = {1: "red", 2: "yellow", 3: "green"}
DEFAULT_WINDOW = (40, 20)


def header_tick(raw: bytes) -> int:
    """Tick of a CDR message whose first field is a ``std_msgs/Header``.

    The 4-byte encapsulation header (0x0001 = little-endian CDR) precedes
    ``stamp.sec`` (int32) and ``stamp.nanosec`` (uint32).
    """
    little = raw[1] == 1
    sec, nanosec = struct.unpack_from("<iI" if little else ">iI", raw, 4)
    return tick_index(sec + nanosec * 1e-9)


def _starts_with_header(msgdef: str) -> bool:
    for line in msgdef.splitlines():
        text = line.split("#", 1)[0].strip()
        if not text or "=" in text:
            continue
        return text.startswith("std_msgs/Header ")
    return False


def _yaw(q: Msg) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def ego_pose(msg: Msg) -> EgoPose:
    """EgoState -> EgoPose."""
    p = msg.pose.position
    return EgoPose(
        float(p.x), float(p.y), _yaw(msg.pose.orientation), math.hypot(msg.vx, msg.vy)
    )


def agent_boxes(msg: Msg, ego: EgoPose | None) -> tuple[AgentBox, ...]:
    """AgentArray -> boxes in the map frame (a base_link array needs the tick's ego pose)."""
    to_map = msg.header.frame_id != "map"
    if to_map and ego is None:
        return ()
    c, s = (
        (math.cos(ego.yaw), math.sin(ego.yaw))
        if (to_map and ego is not None)
        else (1.0, 0.0)
    )
    out = []
    for a in msg.agents:
        x, y, yaw = (
            float(a.pose.position.x),
            float(a.pose.position.y),
            _yaw(a.pose.orientation),
        )
        if to_map and ego is not None:
            x, y, yaw = ego.x + c * x - s * y, ego.y + s * x + c * y, yaw + ego.yaw
        out.append(
            AgentBox(
                int(a.id),
                int(a.class_id),
                x,
                y,
                yaw,
                float(a.length),
                float(a.width),
                math.hypot(a.vx, a.vy),
                bool(a.visible),
            )
        )
    return tuple(out)


def predictions(msg: Msg) -> Predictions | None:
    """PredictionSamples -> [S, A, T, 2] samples."""
    s_n, a_n, t_n = int(msg.num_samples), len(msg.agent_ids), int(msg.num_timesteps)
    xy = np.asarray(msg.xy, dtype=np.float64)
    if s_n * a_n * t_n == 0 or xy.size < s_n * a_n * t_n * 2:
        return None
    weights = np.asarray(msg.sample_weight, dtype=np.float64)
    if weights.size != s_n:
        weights = np.full(s_n, 1.0 / s_n)
    return Predictions(xy[: s_n * a_n * t_n * 2].reshape(s_n, a_n, t_n, 2), weights)


def path_xy(traj: Msg) -> np.ndarray:
    """Trajectory -> [N, 2]."""
    return np.array(
        [[float(p.x), float(p.y)] for p in traj.points], dtype=np.float64
    ).reshape(-1, 2)


def candidates(msg: Msg) -> Candidates:
    """TrajectoryCandidates -> paths, costs, selected."""
    return Candidates(
        tuple(path_xy(c) for c in msg.candidates),
        np.asarray(msg.cost, dtype=np.float64),
        int(msg.selected_index),
    )


def occupancy(msg: Msg, anchor: EgoPose | None) -> OccupancyRaster | None:
    """OccupancyGridMC -> [6, H, W] raster anchored at its tick's base_link pose."""
    if anchor is None:
        return None
    h, w, c = int(msg.height), int(msg.width), int(msg.num_channels)
    data = np.asarray(msg.data, dtype=np.float64)
    if c < 6 or data.size < c * h * w:
        return None
    return OccupancyRaster(
        float(msg.resolution),
        float(msg.x_min),
        float(msg.y_min),
        data[: c * h * w].reshape(c, h, w)[:6],
        anchor,
    )


def lane_map(graph: Msg, states: Mapping[int, str]) -> LaneMap:
    """LaneGraph (+ the tick's light states) -> LaneMap."""
    centerlines = tuple(
        np.array([[p.x, p.y] for p in lane.centerline], dtype=np.float64).reshape(-1, 2)
        for lane in graph.lanes
    )
    drivable = tuple(
        int(lane.type) in (0, 3) for lane in graph.lanes
    )  # TYPE_DRIVING, TYPE_BIDIRECTIONAL
    stop_lines = tuple(
        StopLine(
            float(tl.stop_line.x),
            float(tl.stop_line.y),
            float(tl.heading),
            "light",
            states.get(int(tl.id), ""),
        )
        for tl in graph.traffic_lights
    )
    signs = tuple(
        np.array(
            [[p.x, p.y] for p in sign.trigger_volume.points], dtype=np.float64
        ).reshape(-1, 2)
        for sign in graph.stop_signs
    )
    crosswalks = tuple(
        np.array([[p.x, p.y] for p in cw.footprint.points], dtype=np.float64).reshape(
            -1, 2
        )
        for cw in graph.crosswalks
    )
    return LaneMap(
        centerlines,
        drivable,
        stop_lines,
        tuple(v for v in signs if len(v) >= 3),
        tuple(c for c in crosswalks if len(c) >= 3),
    )


def reference_line(msg: Msg) -> RefLine:
    """ReferenceLine -> RefLine."""
    xy = np.array([[p.x, p.y] for p in msg.points], dtype=np.float64).reshape(-1, 2)
    n = len(xy)
    return RefLine(
        xy,
        np.asarray(msg.heading, dtype=np.float64)[:n],
        np.asarray(msg.left_bound, dtype=np.float64)[:n],
        np.asarray(msg.right_bound, dtype=np.float64)[:n],
    )


def behavior_text(msg: Msg) -> str:
    """BehaviorDecision -> ``keep/follow:7 stop_s=812 v*=8.3 (reason)``."""
    text = f"{LATERAL.get(int(msg.lateral), '?')}/{LONGITUDINAL.get(int(msg.longitudinal), '?')}"
    if int(msg.lead_agent_id):
        text += f":{int(msg.lead_agent_id)}"
    if float(msg.stop_s) >= 0.0:
        text += f" stop_s={float(msg.stop_s):.0f}"
    text += f" v*={float(msg.target_speed):.1f}"
    if msg.reason:
        text += f" ({msg.reason})"
    return text


@dataclass
class BagIndex:
    """The light per-tick facts of a bag (pass 1) and the latched map and line."""

    ticks: list[int] = field(default_factory=list)  # every tick with a pose, sorted
    pose: dict[int, EgoPose] = field(default_factory=dict)
    command: dict[int, tuple[float, float, bool]] = field(default_factory=dict)
    behavior: dict[int, str] = field(default_factory=dict)
    diag: dict[int, dict[str, tuple[float, str]]] = field(default_factory=dict)
    lights: dict[int, dict[int, str]] = field(default_factory=dict)
    episodes: list[tuple[int, int]] = field(default_factory=list)  # (tick, episode_id)
    timeouts: set[int] = field(default_factory=set)
    lane_graph: Msg | None = None
    reference_lines: list[tuple[int, RefLine]] = field(default_factory=list)

    def episode_at(self, k: int) -> int:
        """Episode id in force at tick k (0 before any reset)."""
        episode = 0
        for tick, ep in self.episodes:
            if tick <= k:
                episode = ep
        return episode

    def reference_line_at(self, k: int) -> RefLine | None:
        """Return the latest reference line stamped at or before k (the first one otherwise)."""
        chosen: RefLine | None = None
        for tick, line in self.reference_lines:
            if tick <= k or chosen is None:
                chosen = line
        return chosen

    def latest(
        self, table: Mapping[int, T], k: int, lookback: int = LOOKBACK_TICKS
    ) -> T | None:
        """Return ``table[k]`` or the newest entry within ``lookback`` ticks before it."""
        for t in range(k, k - lookback - 1, -1):
            if t in table:
                return table[t]
        return None


class BagReader:
    """One MCAP with its embedded schemas registered for decoding."""

    def __init__(self, bag: Path) -> None:
        """Open the bag and register every schema it carries."""
        self.bag = bag
        self.typestore = get_typestore(Stores.ROS2_JAZZY)
        self.header_topics: set[str] = set()
        with Reader(bag) as reader:
            add: dict[str, Any] = {}
            for conn in reader.connections:
                if conn.msgdef and conn.msgtype not in self.typestore.types:
                    add.update(get_types_from_msg(conn.msgdef.data, conn.msgtype))
                if conn.msgdef and _starts_with_header(conn.msgdef.data):
                    self.header_topics.add(conn.topic)
            self.typestore.register(add)

    def messages(self, topics: Iterable[str]) -> Iterator[tuple[str, str, bytes]]:
        """Yield ``(topic, msgtype, raw)`` of the given topics in bag order."""
        wanted = set(topics)
        with Reader(self.bag) as reader:
            conns = [c for c in reader.connections if c.topic in wanted]
            for conn, _, raw in reader.messages(connections=conns):
                yield conn.topic, conn.msgtype, bytes(raw)

    def decode(self, msgtype: str, raw: bytes) -> Msg:
        """Deserialize one CDR message."""
        return self.typestore.deserialize_cdr(raw, msgtype)

    def tick_of(self, topic: str, msgtype: str, raw: bytes) -> int:
        """Return the message's tick, from the raw header when the type starts with one."""
        if topic in self.header_topics:
            return header_tick(raw)
        return tick_index(self.decode(msgtype, raw).header.stamp)

    def index(self) -> BagIndex:
        """Pass 1: the light topics."""
        index = BagIndex()
        light = {
            TOPIC_POSE,
            TOPIC_COMMAND,
            TOPIC_BEHAVIOR,
            TOPIC_RESET,
            TOPIC_TIMEOUT,
            TOPIC_LIGHTS,
            TOPIC_LANE_GRAPH,
            TOPIC_REFERENCE_LINE,
        }
        with Reader(self.bag) as reader:
            topics = {c.topic for c in reader.connections}
        light |= {t for t in topics if t.startswith(DIAG_PREFIX)}
        for topic, msgtype, raw in self.messages(light):
            msg = self.decode(msgtype, raw)
            k = tick_index(msg.header.stamp)
            if topic == TOPIC_POSE:
                if bool(msg.valid):
                    index.pose[k] = ego_pose(msg)
            elif topic == TOPIC_COMMAND:
                index.command[k] = (
                    float(msg.accel),
                    float(msg.steering_angle),
                    bool(msg.emergency_stop),
                )
            elif topic == TOPIC_BEHAVIOR:
                index.behavior[k] = behavior_text(msg)
            elif topic == TOPIC_RESET:
                index.episodes.append((k, int(msg.episode_id)))
            elif topic == TOPIC_TIMEOUT:
                index.timeouts.add(k)
            elif topic == TOPIC_LIGHTS:
                index.lights[k] = {
                    int(tl.id): LIGHT_STATES.get(int(tl.state), "") for tl in msg.lights
                }
            elif topic == TOPIC_LANE_GRAPH:
                index.lane_graph = msg
            elif topic == TOPIC_REFERENCE_LINE:
                index.reference_lines.append((k, reference_line(msg)))
            elif topic.startswith(DIAG_PREFIX):
                index.diag.setdefault(k, {})[topic[len(DIAG_PREFIX) :]] = (
                    float(msg.cycle_ms),
                    str(msg.message),
                )
        index.ticks = sorted(index.pose)
        index.episodes.sort()
        index.reference_lines.sort(key=lambda item: item[0])
        return index


def _heavy_layers(
    reader: BagReader,
    index: BagIndex,
    k: int,
    heavy: Mapping[str, Mapping[int, tuple[str, bytes]]],
) -> tuple[dict[str, Any], str]:
    """Decode the heavy messages of tick k into Scene layers; returns them and the plan's source."""
    layers: dict[str, Any] = {}
    source = ""
    for topic, name in HEAVY_TOPICS.items():
        entry = index.latest(heavy.get(topic, {}), k)
        if entry is None:
            continue
        msgtype, raw = entry
        msg = reader.decode(msgtype, raw)
        t = tick_index(msg.header.stamp)
        if name == "occupancy":
            layers[name] = occupancy(msg, index.pose.get(t))
        elif name in ("agents", "gt_agents"):
            layers[name] = agent_boxes(msg, index.pose.get(t))
        elif name == "predictions":
            layers[name] = predictions(msg)
        elif name == "candidates":
            layers[name] = candidates(msg)
        else:
            dots = 20 if name == "mpc_horizon" else 10
            layers[name] = VizPath(path_xy(msg), str(msg.source), dot_every=dots)
            if name == "trajectory":
                source = str(msg.source)
    return layers, source


def build_scene(
    reader: BagReader,
    index: BagIndex,
    k: int,
    heavy: Mapping[str, Mapping[int, tuple[str, bytes]]],
    *,
    marks: Sequence[str] = (),
    lane_cache: dict[int, LaneMap] | None = None,
) -> Scene:
    """Assemble the Scene of tick k from the index and the decoded heavy messages."""
    command = index.command.get(k)
    diag = index.diag.get(k, {})
    degraded = tuple(
        f"{node}: {message[:40]}"
        for node, (_, message) in sorted(diag.items())
        if "degraded" in message
    )
    infractions = list(marks)
    if k in index.timeouts:
        infractions.append("tick_timeout")
    layers, source = _heavy_layers(reader, index, k, heavy)
    if command is not None and command[2]:
        source = f"{source} (emergency_stop)".strip()
    lanes: LaneMap | None = None
    if index.lane_graph is not None:
        states = index.latest(index.lights, k) or {}
        key = hash(tuple(sorted(states.items())))
        if lane_cache is not None and key in lane_cache:
            lanes = lane_cache[key]
        else:
            lanes = lane_map(index.lane_graph, states)
            if lane_cache is not None:
                lane_cache[key] = lanes
    return Scene(
        tick=k,
        sim_time_s=k * TICK_DT_S,
        episode_id=index.episode_at(k),
        ego=index.pose.get(k),
        behavior=index.latest(index.behavior, k) or "",
        source=source,
        degraded=degraded,
        accel_mps2=command[0] if command else None,
        steer_rad=command[1] if command else None,
        infractions=tuple(infractions),
        diag_cycle_ms={node: cycle for node, (cycle, _) in diag.items()},
        lanes=lanes,
        reference_line=index.reference_line_at(k),
        **layers,
    )


def iter_frames(
    reader: BagReader,
    index: BagIndex,
    ticks: Iterable[int],
    layers: Sequence[str] | None = None,
    marks: Mapping[int, Sequence[str]] | None = None,
) -> Iterator[tuple[int, Frame]]:
    """Pass 2: stream the heavy topics and yield ``(tick, frame)`` in tick order."""
    wanted = sorted(set(ticks))
    if not wanted:
        return
    keep: set[int] = set()
    for k in wanted:
        keep.update(range(k - LOOKBACK_TICKS, k + 1))
    pending: dict[str, dict[int, tuple[str, bytes]]] = {t: {} for t in HEAVY_TOPICS}
    lane_cache: dict[int, LaneMap] = {}
    marks = marks or {}
    remaining = list(wanted)

    def flush(upto: int) -> Iterator[tuple[int, Frame]]:
        while remaining and remaining[0] <= upto:
            k = remaining.pop(0)
            scene = build_scene(
                reader, index, k, pending, marks=marks.get(k, ()), lane_cache=lane_cache
            )
            yield k, render_frame(scene, None, layers)
            for table in pending.values():
                for t in [t for t in table if t < k - LOOKBACK_TICKS]:
                    del table[t]

    for topic, msgtype, raw in reader.messages(HEAVY_TOPICS):
        t = reader.tick_of(topic, msgtype, raw)
        if t in keep:
            pending[topic][t] = (msgtype, raw)
        yield from flush(t - FLUSH_LAG_TICKS)
    yield from flush(max(wanted))


def render_route(
    reader: BagReader,
    index: BagIndex,
    out_dir: Path,
    ticks: Sequence[int],
    *,
    layers: Sequence[str] | None = None,
    write_frames: bool = True,
    progress: Callable[[str], None] | None = None,
) -> int:
    """Render ``ticks`` into ``out_dir/frames`` and ``out_dir/sheets``; returns the frame count."""
    sheet: list[tuple[int, Frame]] = []
    count = 0

    def close_sheet() -> None:
        if sheet:
            first, last = sheet[0][0], sheet[-1][0]
            write_png(
                tile([f for _, f in sheet]),
                out_dir / "sheets" / f"{first:06d}_{last:06d}.png",
            )
            sheet.clear()

    for k, frame in iter_frames(reader, index, ticks, layers):
        count += 1
        if write_frames:
            write_png(frame, out_dir / "frames" / f"{k:06d}.png")
        sheet.append((k, frame))
        if len(sheet) == FRAMES_PER_SHEET:
            close_sheet()
        if progress is not None and count % 100 == 0:
            progress(f"{count} frames, tick {k}")
    close_sheet()
    return count


def incident_ticks(
    tick: int, window: tuple[int, int], available: Sequence[int]
) -> list[int]:
    """Return the ticks of one incident sheet: the window sampled down to one sheet of frames."""
    before, after = window
    span = [k for k in available if tick - before <= k <= tick + after]
    if not span:
        return []
    stride = max(1, math.ceil(len(span) / FRAMES_PER_SHEET))
    chosen = span[::stride]
    if tick in span and tick not in chosen:
        chosen.append(tick)
    return sorted(chosen)[:FRAMES_PER_SHEET]


def render_incidents(
    reader: BagReader,
    index: BagIndex,
    out_dir: Path,
    incidents: Sequence[tuple[int, str]],
    *,
    window: tuple[int, int] = DEFAULT_WINDOW,
    layers: Sequence[str] | None = None,
) -> list[Path]:
    """One contact sheet per (tick, kind) into ``out_dir/incidents/{tick:06d}_{kind}.png``."""
    plan: dict[tuple[int, str], list[int]] = {}
    marks: dict[int, list[str]] = {}
    for tick, kind in incidents:
        ticks = incident_ticks(tick, window, index.ticks)
        if ticks:
            plan[(tick, kind)] = ticks
            marks.setdefault(tick, []).append(kind)
    frames: dict[int, Frame] = {}
    needed = {k for ticks in plan.values() for k in ticks}
    for k, frame in iter_frames(reader, index, needed, layers, marks):
        frames[k] = frame
    written = []
    for (tick, kind), ticks in plan.items():
        path = out_dir / "incidents" / f"{tick:06d}_{kind}.png"
        write_png(tile([frames[k] for k in ticks if k in frames]), path)
        written.append(path)
    return written


def parse_ticks(text: str | None, available: Sequence[int]) -> list[int]:
    """``a:b`` (inclusive, either side optional) over the ticks the bag holds."""
    if not available:
        return []
    lo, hi = available[0], available[-1]
    if text:
        a, _, b = text.partition(":")
        lo = int(a) if a else lo
        hi = int(b) if b else hi
    return [k for k in available if lo <= k <= hi]


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """CLI of docs/02 §8.1."""
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--bag", type=Path, required=True, help="run.mcap of a route")
    p.add_argument(
        "--out",
        type=Path,
        help="route directory to write into (default: the bag's directory)",
    )
    p.add_argument(
        "--stride",
        type=int,
        default=10,
        help="ticks between frames (default 10 = 2 Hz)",
    )
    p.add_argument("--ticks", help="a:b tick range, inclusive (default: the whole bag)")
    p.add_argument(
        "--layers", nargs="*", choices=style.LAYER_NAMES, help="draw only these layers"
    )
    p.add_argument(
        "--incident",
        action="append",
        default=[],
        metavar="TICK:KIND",
        help="render one incident sheet (repeatable); no frames/ or sheets/",
    )
    p.add_argument(
        "--window",
        default=f"{DEFAULT_WINDOW[0]}:{DEFAULT_WINDOW[1]}",
        help="ticks before:after an incident (default 40:20)",
    )
    p.add_argument(
        "--no-frames", action="store_true", help="write only the contact sheets"
    )
    return p.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Entry point."""
    args = parse_args(argv)
    out_dir = args.out or args.bag.parent
    reader = BagReader(args.bag)
    index = reader.index()
    if not index.ticks:
        print("no /nuway/loc/pose messages in the bag")
        return 1
    if args.incident:
        before, _, after = args.window.partition(":")
        incidents = []
        for item in args.incident:
            tick, _, kind = item.partition(":")
            incidents.append((int(tick), kind or "incident"))
        paths = render_incidents(
            reader,
            index,
            out_dir,
            incidents,
            window=(int(before), int(after)),
            layers=args.layers,
        )
        for path in paths:
            print(path)
        return 0
    ticks = parse_ticks(args.ticks, index.ticks)[:: max(1, args.stride)]
    n = render_route(
        reader,
        index,
        out_dir,
        ticks,
        layers=args.layers,
        write_frames=not args.no_frames,
        progress=print,
    )
    print(f"{n} frames -> {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
