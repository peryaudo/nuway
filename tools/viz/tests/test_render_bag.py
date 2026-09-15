"""render_bag: a synthetic MCAP renders frames, sheets and incident sheets without ROS."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Any

import numpy as np
import pytest

pytest.importorskip("matplotlib")
pytest.importorskip("rosbags")

import render_bag
from rosbags.rosbag2 import Writer  # after the importorskip
from rosbags.rosbag2.enums import StoragePlugin
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

ROOT = Path(__file__).resolve().parents[3]
MSG_DIR = ROOT / "ros2_ws/src/nuway_msgs/msg"
TICKS = 40


def _typestore() -> Any:  # rosbags' Typestore has no public stub
    ts = get_typestore(Stores.ROS2_JAZZY)
    add: dict[str, Any] = {}
    for f in sorted(MSG_DIR.glob("*.msg")):
        add.update(get_types_from_msg(f.read_text(), f"nuway_msgs/msg/{f.stem}"))
    ts.register(add)
    return ts


def _default(ts: Any, node: tuple[Any, Any]) -> Any:  # typestore field nodes
    """A zero value for one rosbags field definition node."""
    kind, arg = node
    name = kind.name
    if name == "BASE":
        base = arg[0]
        if base == "string":
            return ""
        if base == "bool":
            return False
        return 0
    if name == "NAME":
        return _msg(ts, arg)
    inner, count = arg
    if inner[0].name == "BASE" and inner[1][0] not in ("string", "bool"):
        return np.zeros(count, dtype=_dtype(inner[1][0]))
    return [_default(ts, inner) for _ in range(count)]


def _dtype(base: str) -> Any:  # numpy dtype
    return {
        "float32": np.float32,
        "float64": np.float64,
        "uint8": np.uint8,
        "int32": np.int32,
        "uint32": np.uint32,
    }[base]


def _msg(ts: Any, name: str, **fields: Any) -> Any:  # rosbags message objects
    """Build a rosbags message with zero defaults for the fields not given."""
    values = {field: _default(ts, node) for field, node in ts.fielddefs[name][1]}
    values.update(fields)
    return ts.types[name](**values)


def _header(ts: Any, k: int, frame: str = "map") -> Any:  # rosbags message
    nanos = k * 50_000_000
    stamp = ts.types["builtin_interfaces/msg/Time"](
        sec=nanos // 10**9, nanosec=nanos % 10**9
    )
    return ts.types["std_msgs/msg/Header"](stamp=stamp, frame_id=frame)


def _point(ts: Any, x: float, y: float, z: float = 0.0) -> Any:  # rosbags message
    return ts.types["geometry_msgs/msg/Point"](x=x, y=y, z=z)


def _pose(ts: Any, x: float, y: float, yaw: float) -> Any:  # rosbags message
    q = ts.types["geometry_msgs/msg/Quaternion"](
        x=0.0, y=0.0, z=float(np.sin(yaw / 2)), w=float(np.cos(yaw / 2))
    )
    return ts.types["geometry_msgs/msg/Pose"](position=_point(ts, x, y), orientation=q)


def _trajectory(ts: Any, k: int, x0: float, source: str, n: int = 81) -> Any:
    pts = [
        _msg(
            ts,
            "nuway_msgs/msg/TrajectoryPoint",
            t=0.1 * i,
            x=x0 + 0.5 * i,
            y=1.0,
            v=5.0,
        )
        for i in range(n)
    ]
    return _msg(
        ts,
        "nuway_msgs/msg/Trajectory",
        header=_header(ts, k),
        points=pts,
        source=source,
        candidate_id=3,
        sample_index=-1,
    )


def write_bag(path: Path) -> None:
    """A 40-tick episode with every layer's topic present."""
    ts = _typestore()
    m = "nuway_msgs/msg/"
    with Writer(path, version=9, storage_plugin=StoragePlugin.MCAP) as w:
        conns = {}

        def conn(topic: str, msgtype: str) -> None:
            conns[topic] = w.add_connection(topic, msgtype, typestore=ts)

        for topic, msgtype in [
            ("/nuway/loc/pose", m + "EgoState"),
            ("/nuway/control/command", m + "ControlCommand"),
            ("/nuway/planning/behavior", m + "BehaviorDecision"),
            ("/nuway/diag/planner_node", m + "NodeDiag"),
            ("/nuway/diag/mpc_node", m + "NodeDiag"),
            ("/nuway/sim/reset_event", m + "ResetEvent"),
            ("/nuway/sim/tick_timeout", m + "TickTimeout"),
            ("/nuway/perception/traffic_lights", m + "TrafficLightArray"),
            ("/nuway/map/lane_graph", m + "LaneGraph"),
            ("/nuway/route/reference_line", m + "ReferenceLine"),
            ("/nuway/perception/occupancy", m + "OccupancyGridMC"),
            ("/nuway/perception/agents", m + "AgentArray"),
            ("/nuway/gt/agents", m + "AgentArray"),
            ("/nuway/prediction/samples", m + "PredictionSamples"),
            ("/nuway/planning/candidates", m + "TrajectoryCandidates"),
            ("/nuway/planning/trajectory", m + "Trajectory"),
            ("/nuway/planning/safe_trajectory", m + "Trajectory"),
            ("/nuway/control/horizon", m + "Trajectory"),
        ]:
            conn(topic, msgtype)

        def put(topic: str, msg: Any, k: int) -> None:
            w.write(
                conns[topic],
                k * 50_000_000 + 1,
                ts.serialize_cdr(msg, conns[topic].msgtype),
            )

        lane = _msg(
            ts,
            m + "Lane",
            id=1,
            type=0,
            centerline=[_point(ts, float(x), 0.0) for x in range(0, 120, 2)],
            width=np.full(60, 3.5, np.float32),
        )
        light = _msg(
            ts,
            m + "TrafficLightMapping",
            id=5,
            stop_line=_point(ts, 60.0, 0.0),
            heading=0.0,
        )
        sign = _msg(
            ts,
            m + "StopSign",
            id=6,
            stop_line=_point(ts, 80.0, 0.0),
            trigger_volume=ts.types["geometry_msgs/msg/Polygon"](
                points=[
                    ts.types["geometry_msgs/msg/Point32"](x=float(x), y=float(y), z=0.0)
                    for x, y in ((78, -2), (82, -2), (82, 2), (78, 2))
                ]
            ),
        )
        graph = _msg(
            ts,
            m + "LaneGraph",
            header=_header(ts, 0),
            lanes=[lane],
            traffic_lights=[light],
            stop_signs=[sign],
        )
        put("/nuway/map/lane_graph", graph, 0)
        n = 200
        line = _msg(
            ts,
            m + "ReferenceLine",
            header=_header(ts, 0),
            s=np.arange(n, dtype=np.float32) * 0.5,
            points=[_point(ts, 0.5 * i, 1.0) for i in range(n)],
            heading=np.zeros(n, np.float32),
            curvature=np.zeros(n, np.float32),
            speed_limit=np.full(n, 8.0, np.float32),
            left_bound=np.full(n, 1.75, np.float32),
            right_bound=np.full(n, 1.75, np.float32),
            lane_id=np.ones(n, np.uint32),
        )
        put("/nuway/route/reference_line", line, 0)
        put(
            "/nuway/sim/reset_event",
            _msg(
                ts,
                m + "ResetEvent",
                header=_header(ts, 0),
                episode_id=1,
                town="Town03",
                start_pose=_pose(ts, 0.0, 1.0, 0.0),
            ),
            0,
        )
        put(
            "/nuway/sim/tick_timeout",
            _msg(
                ts,
                m + "TickTimeout",
                header=_header(ts, 25),
                episode_id=1,
                waited_s=0.5,
            ),
            25,
        )
        for k in range(TICKS):
            x = 0.25 * k
            put(
                "/nuway/loc/pose",
                _msg(
                    ts,
                    m + "EgoState",
                    header=_header(ts, k),
                    pose=_pose(ts, x, 1.0, 0.0),
                    vx=5.0,
                    valid=True,
                ),
                k,
            )
            put(
                "/nuway/control/command",
                _msg(
                    ts,
                    m + "ControlCommand",
                    header=_header(ts, k),
                    accel=0.3,
                    steering_angle=-0.01,
                    emergency_stop=k == 30,
                ),
                k,
            )
            put(
                "/nuway/diag/mpc_node",
                _msg(
                    ts,
                    m + "NodeDiag",
                    header=_header(ts, k),
                    node="mpc_node",
                    cycle_ms=0.4,
                    message="ok",
                ),
                k,
            )
            put(
                "/nuway/planning/safe_trajectory",
                _trajectory(ts, k, x, "lattice" if k < 30 else "fallback"),
                k,
            )
            put("/nuway/control/horizon", _trajectory(ts, k, x, "mpc", n=21), k)
            put(
                "/nuway/gt/agents",
                _msg(
                    ts,
                    m + "AgentArray",
                    header=_header(ts, k),
                    agents=[
                        _msg(
                            ts,
                            m + "Agent",
                            id=7,
                            class_id=1,
                            pose=_pose(ts, x + 15.0, 1.0, 0.0),
                            length=4.5,
                            width=2.0,
                            vx=4.0,
                            visible=True,
                        )
                    ],
                ),
                k,
            )
            if k % 2 == 0:
                put(
                    "/nuway/planning/behavior",
                    _msg(
                        ts,
                        m + "BehaviorDecision",
                        header=_header(ts, k),
                        longitudinal=1,
                        lead_agent_id=7,
                        stop_s=-1.0,
                        target_speed=6.0,
                        reason="lead",
                    ),
                    k,
                )
                put(
                    "/nuway/diag/planner_node",
                    _msg(
                        ts,
                        m + "NodeDiag",
                        header=_header(ts, k),
                        node="planner_node",
                        cycle_ms=5.0,
                        message="samples degraded" if k >= 26 else "ok",
                    ),
                    k,
                )
                put(
                    "/nuway/perception/traffic_lights",
                    _msg(
                        ts,
                        m + "TrafficLightArray",
                        header=_header(ts, k),
                        lights=[
                            _msg(
                                ts,
                                m + "TrafficLight",
                                id=5,
                                state=1 if k < 20 else 3,
                                stop_line=_point(ts, 60.0, 0.0),
                                confidence=1.0,
                            )
                        ],
                    ),
                    k,
                )
                grid = np.zeros((6, 40, 40), np.float32)
                grid[3, :, 15:25] = 1.0
                grid[0, 30:34, 18:22] = 1.0
                put(
                    "/nuway/perception/occupancy",
                    _msg(
                        ts,
                        m + "OccupancyGridMC",
                        header=_header(ts, k, "base_link"),
                        resolution=0.5,
                        x_min=-10.0,
                        y_min=-10.0,
                        height=40,
                        width=40,
                        num_channels=6,
                        channel_names=[
                            "occupied",
                            "free",
                            "unknown",
                            "drivable",
                            "height_max",
                            "dynamic",
                        ],
                        data=grid.ravel(),
                    ),
                    k,
                )
                put(
                    "/nuway/perception/agents",
                    _msg(
                        ts,
                        m + "AgentArray",
                        header=_header(ts, k, "base_link"),
                        agents=[
                            _msg(
                                ts,
                                m + "Agent",
                                id=7,
                                class_id=1,
                                pose=_pose(ts, 15.0, 0.0, 0.0),
                                length=4.5,
                                width=2.0,
                                vx=4.0,
                                visible=True,
                            )
                        ],
                    ),
                    k,
                )
                xy = np.zeros((2, 1, 16, 2), np.float32)
                xy[:, 0, :, 0] = x + 15.0 + np.arange(1, 17) * 2.0
                xy[:, 0, :, 1] = 1.0
                put(
                    "/nuway/prediction/samples",
                    _msg(
                        ts,
                        m + "PredictionSamples",
                        header=_header(ts, k),
                        agent_ids=np.array([7], np.uint32),
                        num_samples=2,
                        num_timesteps=16,
                        dt=0.5,
                        xy=xy.ravel(),
                        yaw=np.zeros(32, np.float32),
                        sample_weight=np.array([0.6, 0.4], np.float32),
                    ),
                    k,
                )
                cands = [
                    _trajectory(ts, k, x + 0.1 * i, "lattice", n=41) for i in range(5)
                ]
                put(
                    "/nuway/planning/candidates",
                    _msg(
                        ts,
                        m + "TrajectoryCandidates",
                        header=_header(ts, k),
                        candidates=cands,
                        cost=np.array([3.0, 1.0, 2.0, 5.0, 4.0], np.float32),
                        cost_breakdown_names=["a"],
                        cost_breakdown=np.ones(5, np.float32),
                        selected_index=1,
                    ),
                    k,
                )
                put("/nuway/planning/trajectory", _trajectory(ts, k, x, "lattice"), k)


@pytest.fixture(scope="module")
def bag(tmp_path_factory: pytest.TempPathFactory) -> Path:
    path = tmp_path_factory.mktemp("bag") / "run.mcap"
    write_bag(path.parent / "run")
    mcap = next((path.parent / "run").glob("*.mcap"))
    mcap.rename(path)
    return path


def test_header_tick_reads_the_cdr_stamp() -> None:
    ts = _typestore()
    msg = _msg(ts, "nuway_msgs/msg/ControlCommand", header=_header(ts, 1234))
    raw = ts.serialize_cdr(msg, "nuway_msgs/msg/ControlCommand")
    assert render_bag.header_tick(raw) == 1234


def test_index_collects_the_light_topics(bag: Path) -> None:
    reader = render_bag.BagReader(bag)
    index = reader.index()
    assert index.ticks == list(range(TICKS))
    assert index.episode_at(5) == 1
    assert index.timeouts == {25}
    assert index.behavior[10].startswith("keep/follow:7")
    assert index.lights[10][5] == "red"
    assert index.lights[20][5] == "green"
    assert index.lane_graph is not None
    assert index.reference_line_at(10) is not None
    assert index.latest(index.behavior, 11) == index.behavior[10]


def test_route_render_writes_frames_and_sheets(bag: Path, tmp_path: Path) -> None:
    reader = render_bag.BagReader(bag)
    index = reader.index()
    ticks = render_bag.parse_ticks("0:39", index.ticks)[::2]
    n = render_bag.render_route(reader, index, tmp_path, ticks)
    assert n == 20
    frames = sorted((tmp_path / "frames").glob("*.png"))
    assert [f.name for f in frames][:3] == ["000000.png", "000002.png", "000004.png"]
    assert sorted(p.name for p in (tmp_path / "sheets").glob("*.png")) == [
        "000000_000038.png"
    ]
    # Tick 30 carries the e-stop and, from 26 on, the degraded planner: the scene says so.
    heavy: dict[str, dict[int, tuple[str, bytes]]] = {}
    scene = render_bag.build_scene(reader, index, 30, heavy)
    assert "emergency_stop" in scene.source
    assert scene.degraded == ("planner_node: samples degraded",)
    assert scene.behavior.startswith("keep/follow:7")
    assert scene.lanes is not None
    assert scene.lanes.stop_lines[0].state == "green"
    assert len(scene.lanes.sign_volumes) == 1


def test_scene_layers_come_from_the_heavy_topics(bag: Path) -> None:
    reader = render_bag.BagReader(bag)
    index = reader.index()
    frames = dict(render_bag.iter_frames(reader, index, [11]))
    assert set(frames) == {11}
    assert frames[11].shape == (800, 1000, 3)
    # The odd tick 11 takes the 10 Hz layers from tick 10; assemble the scene to check.
    pending: dict[str, dict[int, tuple[str, bytes]]] = {
        t: {} for t in render_bag.HEAVY_TOPICS
    }
    for topic, msgtype, raw in reader.messages(render_bag.HEAVY_TOPICS):
        t = reader.tick_of(topic, msgtype, raw)
        if 9 <= t <= 11:
            pending[topic][t] = (msgtype, raw)
    scene = render_bag.build_scene(reader, index, 11, pending)
    assert scene.occupancy is not None
    assert scene.agents[0].x == pytest.approx(
        0.25 * 10 + 15.0
    )  # base_link -> map with tick 10's pose
    assert scene.gt_agents[0].x == pytest.approx(0.25 * 11 + 15.0)
    assert scene.predictions is not None
    assert scene.predictions.xy.shape == (2, 1, 16, 2)
    assert scene.candidates is not None
    assert len(scene.candidates.paths) == 5
    assert scene.trajectory is not None
    assert scene.trajectory.source == "lattice"
    assert scene.mpc_horizon is not None
    assert len(scene.mpc_horizon.xy) == 21


def test_incident_sheets_are_one_png_each(bag: Path, tmp_path: Path) -> None:
    reader = render_bag.BagReader(bag)
    index = reader.index()
    paths = render_bag.render_incidents(
        reader,
        index,
        tmp_path,
        [(25, "tick_timeout"), (30, "red_light")],
        window=(10, 5),
    )
    assert [p.name for p in paths] == [
        "000025_tick_timeout.png",
        "000030_red_light.png",
    ]
    assert all(p.exists() for p in paths)
    assert (
        render_bag.incident_ticks(30, (40, 20), index.ticks)
        == list(range(0, TICKS, 2))[:20]
    )


def test_cli_runs_without_ros_or_rclpy(bag: Path, tmp_path: Path) -> None:
    code = (
        "import sys, runpy\n"
        f"sys.argv = ['render_bag.py', '--bag', {str(bag)!r}, '--out', {str(tmp_path)!r}, '--ticks', '0:9', '--stride', '3']\n"
        "try:\n    runpy.run_path(sys.argv[0] if False else 'tools/viz/render_bag.py', run_name='__main__')\n"
        "except SystemExit as e:\n    assert e.code == 0, e.code\n"
        "assert 'rclpy' not in sys.modules\n"
        "assert not any(m.startswith('rosidl') for m in sys.modules)\n"
    )
    env = {"HOME": str(Path.home()), "PATH": "/usr/bin:/bin", "PYTHONPATH": "tools/viz"}
    subprocess.run([sys.executable, "-c", code], check=True, cwd=ROOT, env=env)
    assert (tmp_path / "sheets" / "000000_000009.png").exists()
