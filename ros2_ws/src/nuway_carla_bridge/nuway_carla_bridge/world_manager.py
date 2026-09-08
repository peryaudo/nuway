"""world_manager node (M0): the one process that owns the CARLA client.

Responsibilities (``M0_bringup.md`` §2.1, ``docs/02_interfaces.md`` §2):

1. connect, load the town if different (a ``carla.town`` ending in ``.xodr``
   is generated from that OpenDRIVE file and named by its stem; M0 sysid),
   export ``data/maps/<town>/map.xodr`` once, set synchronous mode and the
   Traffic Manager to sync;
2. spawn the hero and the rig (``sensor_rig``), publish ``/tf_static`` and
   ``camera_info``;
3. own the tick loop in **lockstep**: ``world.tick()``, publish ``/clock`` and
   the GT topics (``gt_publisher``), then block until a ``ControlCommand``
   whose tick index equals this tick arrives, or the wall-clock timeout
   elapses, in which case ``/nuway/sim/tick_timeout`` is published and the
   loop ticks anyway (the only wall-clock-triggered event in the stack);
4. serve ``/nuway/sim/reset`` and ``/nuway/sim/set_weather``; a reset moves the
   hero, publishes ``ResetEvent`` before the first tick of the new episode and
   then ticks once through the normal gate with the startup timeout;
5. destroy every spawned actor on shutdown.

State across ticks: the episode id, the last command tick and the modules'
own state; a reset clears ``gt_publisher``'s ring buffers.

CARLA in one paragraph. The server (``CarlaUE4``) simulates; clients talk to
it over RPC (port 2000) and see the world through snapshots. In
*asynchronous* mode the server steps physics on its own clock and clients
only observe. In *synchronous* mode (``synchronous_mode=True`` with
``fixed_delta_seconds=0.05``) the server freezes between frames and advances
exactly one 0.05 s frame per ``world.tick()`` from the client that owns the
tick; every actor transform, velocity and sensor image is then the state
after that frame. That is what makes a replayable stack possible: the
simulation advances only when this node says so, and only after the
controller has answered the previous frame.

One tick, with k the tick index ``round(elapsed_seconds / 0.05)``
(``nuway_ml.common.tick``; every stamp in the stack is compared by k)::

    world.tick()                          frame k is simulated
    /clock, /nuway/gt/**  stamped k       this node + gt_publisher
      -> gt_pose_node        -> /nuway/loc/pose k
      -> pure_pursuit_pid    -> /nuway/control/command k
      -> control_adapter     -> /carla/hero/vehicle_control_cmd
    block until a ControlCommand stamped k has arrived   (_wait_for_command)
    world.tick()                          frame k + 1 ...

The gate checks only that the ``ControlCommand`` was published; the
adapter's CARLA control follows on the native topic and the server applies
whatever it holds when the next frame is computed, so the stack has one
tick of actuator latency by design (``docs/02_interfaces.md`` §2).

Threads. The rclpy callbacks (command subscription, the two services) run
on a ``MultiThreadedExecutor`` in a background thread; the tick loop runs in
the main thread, because the CARLA client is not thread safe and every
``world`` / actor call must come from one thread. The two meet through
``_cmd_cond`` (a command arrived) and ``_pending_reset`` (a reset to apply
between two ticks).

Time. Sim time is CARLA's ``elapsed_seconds``, published on ``/clock``; every
node runs with ``use_sim_time`` so its clock stands still between ticks and
no node can time anything by the wall clock. The lockstep timeout is the
one wall-clock decision, and it is reported (``TickTimeout``) rather than
hidden.

Frames. CARLA's world is left-handed (x forward, y right, z up, yaw
clockwise, degrees); ROS is right-handed (REP-103), so y, pitch and yaw
flip sign, and every quantity read from or written to the API goes through
``nuway_ml.common.carla_conv`` (the only place with that arithmetic). A
vehicle's CARLA origin is its mesh origin, under the middle of the car and
``actor_origin_height`` above the road; the stack's ``base_link`` is the
rear axle on the ground (``VehicleGeometry.base_link_in_actor``), hence the
``_base_pose_of`` conversions.
"""

from __future__ import annotations

import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import carla
import numpy as np
import rclpy
from nuway_msgs.msg import ControlCommand, NodeDiag, ResetEvent, TickTimeout
from nuway_msgs.srv import Reset, SetWeather
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from nuway_carla_bridge.gt_publisher import GtParams, GtPublisher
from nuway_carla_bridge.sensor_rig import SensorRig
from nuway_ml.common.carla_conv import (
    location_from_ros,
    rotation_from_ros,
    transform_to_ros,
)
from nuway_ml.common.frames import (
    TOPIC_CLOCK,
    TOPIC_CONTROL_COMMAND,
    TOPIC_DIAG_PREFIX,
    TOPIC_GT_EGO_ODOM,
    TOPIC_RESET_EVENT,
    TOPIC_TICK_TIMEOUT,
)
from nuway_ml.common.geometry import SE3, compose, inverse, quaternion_to_rpy
from nuway_ml.common.rig import VehicleGeometry, load_rig
from nuway_ml.common.tick import TICK_DT_S, tick_index, tick_stamp
from nuway_rclpy.ros_conv import (
    pose_from_se3,
    se3_from_pose,
    stamp_from_seconds,
)
from nuway_rclpy.ros_qos import CLOCK_QOS, qos

NODE_NAME = "world_manager"
HERO_ROLE = "hero"  # role_name / ros_name of the ego: topics are /carla/hero/**
RESET_DROP_MARGIN_M = 0.15  # teleport slightly above the ground and let physics settle
GENERATED_MAP_NAME = (
    "OpenDriveMap"  # CARLA's name for every generate_opendrive_world() map
)
# Mesh generation for a world built from a bare OpenDRIVE file (no town assets):
# road triangles every 2 m, no side walls, 1 m of extra pavement so a car on
# the outer lane edge does not fall off, and no walker navmesh (no walkers).
OPENDRIVE_GENERATION_PARAMETERS = carla.OpendriveGenerationParameters(
    vertex_distance=2.0,
    max_road_length=500.0,
    wall_height=0.0,
    additional_width=1.0,
    smooth_junctions=True,
    enable_mesh_visibility=True,
    enable_pedestrian_navigation=False,
)
# The named presets of carla.WeatherParameters (ClearNoon, WetSunset, ...),
# selectable through /nuway/sim/set_weather. Weather only changes rendering.
WEATHER_PRESETS = {
    name: getattr(carla.WeatherParameters, name)
    for name in dir(carla.WeatherParameters)
    if name[0].isupper()
    and isinstance(getattr(carla.WeatherParameters, name), carla.WeatherParameters)
}


@dataclass
class _PendingReset:
    """A reset request handed from the service thread to the tick loop.

    The service callback fills ``request``, blocks on ``done`` and returns
    ``response`` once the tick thread has applied the reset (CARLA calls must
    stay on that thread).
    """

    request: Reset.Request
    done: threading.Event
    response: Reset.Response


class WorldManagerNode(Node):  # type: ignore[misc]  # rclpy.Node has no stubs (03 §7.3)
    """Owns the CARLA client, the lockstep tick loop, /clock and the GT topics.

    Construction does the whole set-up in order (parameters, connect, world,
    map export, sync mode, hero, rig, GT publishers, ROS endpoints);
    :meth:`run` is the tick loop; :meth:`shutdown` undoes the set-up.
    """

    def __init__(self) -> None:
        """Declare parameters, connect to CARLA, spawn the hero and the rig."""
        super().__init__(NODE_NAME)
        p = self._declare_params()
        self._dt = float(p["carla.fixed_delta_seconds"])
        self._lockstep_timeout_s = float(p["carla.lockstep_timeout_s"])
        self._startup_timeout_s = float(p["carla.lockstep_startup_timeout_s"])
        self._realtime_factor = float(p["carla.realtime_factor"])
        self._vehicle = VehicleGeometry.from_yaml(Path(str(p["vehicle"])))
        self._rig = load_rig(Path(str(p["sensors"])))
        self._map_dir = Path(str(p["map_dir"]))

        self._client = carla.Client(str(p["carla.host"]), int(p["carla.port"]))
        self._client.set_timeout(120.0)
        self._world = self._load_world(
            str(p["carla.town"]), bool(p["carla.no_rendering"])
        )
        town_param = str(p["carla.town"])
        self._town = (
            Path(town_param).stem
            if town_param.endswith(".xodr")
            else self._world.get_map().name.split("/")[-1]
        )
        self._export_opendrive()
        self._apply_sync_settings(
            bool(p["carla.sync"]),
            int(p["carla.traffic.tm_port"]),
            int(p["carla.traffic.seed"]),
        )

        self._hero, self._spawn_pose = self._spawn_hero(int(p["spawn_index"]))
        self._sensor_rig = SensorRig(
            self,
            self._world,
            self._hero,
            self._rig,
            self._vehicle,
            label_only=bool(p["spawn_label_only_sensors"]),
            viz_only=bool(p["spawn_viz_only_sensors"]),
        )
        try:
            self._sensor_rig.spawn()
            self._gt = GtPublisher(
                self,
                self._world,
                self._hero,
                self._vehicle,
                GtParams(
                    agent_radius_m=float(p["gt.agent_radius_m"]),
                    truck_blueprints=tuple(str(b) for b in p["gt.truck_blueprints"]),
                ),
            )
        except (RuntimeError, ValueError, KeyError):
            # CARLA does not garbage-collect actors when the client goes
            # away: a failed start-up would leave a second `hero` (and its
            # sensors) publishing on /carla/hero/** at the next launch.
            self._destroy_actors()
            raise

        self._pub_clock = self.create_publisher(Clock, TOPIC_CLOCK, CLOCK_QOS)
        self._pub_reset_event = self.create_publisher(
            ResetEvent, TOPIC_RESET_EVENT, qos("event")
        )
        self._pub_tick_timeout = self.create_publisher(
            TickTimeout, TOPIC_TICK_TIMEOUT, qos("event")
        )
        self._pub_diag = self.create_publisher(
            NodeDiag, TOPIC_DIAG_PREFIX + NODE_NAME, qos("diag")
        )
        sub_group = MutuallyExclusiveCallbackGroup()
        self._sub_command = self.create_subscription(
            ControlCommand,
            TOPIC_CONTROL_COMMAND,
            self._on_control_command,
            qos("stream"),
            callback_group=sub_group,
        )
        srv_group = ReentrantCallbackGroup()
        self._srv_reset = self.create_service(
            Reset, "/nuway/sim/reset", self._on_reset, callback_group=srv_group
        )
        self._srv_weather = self.create_service(
            SetWeather,
            "/nuway/sim/set_weather",
            self._on_set_weather,
            callback_group=srv_group,
        )

        self._cmd_cond = threading.Condition()
        self._latest_cmd_tick = -1
        self._episode_id = 0
        self._startup_pending = True
        self._pending_reset: _PendingReset | None = None
        self._pending_lock = threading.Lock()
        self._last_tick_wall = time.monotonic()
        self._timeouts = 0
        self._last_frame: int | None = None  # server frame of the last tick
        self.get_logger().info(
            f"ready: town {self._town}, dt {self._dt}, lockstep timeout {self._lockstep_timeout_s}s "
            f"(startup {self._startup_timeout_s}s), realtime_factor {self._realtime_factor}"
        )

    # -------------------------------------------------------------- set-up
    def _declare_params(self) -> dict[str, Any]:
        """Declare every parameter with its default and return the values.

        The ``carla.*`` keys mirror the profile's ``carla:`` section
        (``docs/02_interfaces.md`` §5); the launch file passes the profile
        through as parameter overrides, so a bare ``ros2 run`` also works.
        """
        defaults: dict[str, Any] = {
            "carla.host": "localhost",
            "carla.port": 2000,
            "carla.town": "Town03",
            "carla.fixed_delta_seconds": TICK_DT_S,
            "carla.sync": True,
            "carla.no_rendering": False,
            "carla.lockstep_timeout_s": 2.0,
            "carla.lockstep_startup_timeout_s": 120.0,
            "carla.realtime_factor": 0.0,
            "carla.traffic.tm_port": 8000,
            "carla.traffic.seed": 0,
            "sensors": "configs/sensors/rig_dev.json",
            "vehicle": "configs/vehicle/lincoln_mkz_2020.yaml",
            "spawn_index": 0,
            "map_dir": "data/maps",
            "spawn_label_only_sensors": False,
            "spawn_viz_only_sensors": False,
            "gt.agent_radius_m": 100.0,
            "gt.truck_blueprints": [""],
        }
        values: dict[str, Any] = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            values[name] = self.get_parameter(name).value
        return values

    def _load_world(self, town: str, no_rendering: bool) -> carla.World:
        """Return the world for ``town``, (re)loading it only when it differs.

        ``load_world`` restarts the level (seconds of wall time, and CARLA
        0.9.16 crashes loading Town01 over another town, ``docs/00`` §4), so
        the current world is reused whenever it already is the requested
        one. ``no_rendering_mode`` skips the camera and lidar rendering
        (physics only; sysid runs use it).
        """
        world = self._client.get_world()
        if town.endswith(".xodr"):
            # A generated OpenDRIVE world (M0 sysid: configs/maps/sysid_straight.xodr).
            # CARLA names every generated map "OpenDriveMap", so the current world
            # is reused only when its OpenDRIVE matches the file.
            xodr = Path(town).read_text()
            current = world.get_map()
            if (
                not current.name.endswith(GENERATED_MAP_NAME)
                or current.to_opendrive() != xodr
            ):
                self.get_logger().info(
                    f"generating world from {town} (current: {current.name})"
                )
                world = self._client.generate_opendrive_world(
                    xodr, OPENDRIVE_GENERATION_PARAMETERS
                )
        elif town not in world.get_map().name:
            self.get_logger().info(f"loading {town} (current: {world.get_map().name})")
            world = self._client.load_world(town)
        if no_rendering:
            settings = world.get_settings()
            settings.no_rendering_mode = True
            world.apply_settings(settings)
        return world

    def _export_opendrive(self) -> None:
        """Write the town's OpenDRIVE to ``data/maps/<town>/map.xodr`` once.

        ``map_server_node`` and ``route_gen`` parse that file, so every map
        consumer works from exactly the geometry the server simulates.
        """
        path = self._map_dir / self._town / "map.xodr"
        if path.is_file():
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self._world.get_map().to_opendrive())
        self.get_logger().info(f"exported {path}")

    def _apply_sync_settings(self, sync: bool, tm_port: int, seed: int) -> None:
        """Put the world and the Traffic Manager into synchronous mode.

        With ``synchronous_mode`` the server advances one ``fixed_delta_seconds``
        frame per ``world.tick()`` and freezes otherwise (module docstring).
        The Traffic Manager, CARLA's autopilot for NPC vehicles, runs its own
        thread inside this client process and must be told about sync mode
        too, or it would step the NPC controls on wall time; its random
        seed makes those decisions reproducible (M1 traffic).
        """
        settings = self._world.get_settings()
        settings.synchronous_mode = sync
        settings.fixed_delta_seconds = self._dt
        self._world.apply_settings(settings)
        self._traffic_manager = self._client.get_trafficmanager(tm_port)
        self._traffic_manager.set_synchronous_mode(sync)
        self._traffic_manager.set_random_device_seed(seed)

    def _spawn_hero(self, spawn_index: int) -> tuple[carla.Actor, SE3]:
        """Spawn the hero; returns it with its actor pose (ROS) at the spawn point.

        Blueprint attributes: ``role_name`` is how CARLA tooling finds the
        ego, ``ros_name`` is what the native ROS 2 layer names topics from
        (``/carla/hero/**``), and ``ros_publish_tf`` off keeps CARLA from
        broadcasting its own ``/tf`` for the car (``gt_pose_node`` owns TF).
        Spawn points are the map's predefined, collision-free poses.
        """
        lib = self._world.get_blueprint_library()
        bp = lib.find(self._rig.vehicle)
        bp.set_attribute("role_name", HERO_ROLE)
        bp.set_attribute("ros_name", HERO_ROLE)
        bp.set_attribute("ros_publish_tf", "false")
        spawn_points = self._world.get_map().get_spawn_points()
        transform = spawn_points[spawn_index % len(spawn_points)]
        hero = self._world.spawn_actor(bp, transform)
        self.get_logger().info(
            f"spawned {self._rig.vehicle} as hero at spawn point {spawn_index}"
        )
        return hero, transform_to_ros(transform.location, transform.rotation)

    # ------------------------------------------------------------ callbacks
    def _on_control_command(self, msg: ControlCommand) -> None:
        """Record the newest command tick and wake the tick loop.

        Executor thread. Only the tick index is kept: the command itself is
        consumed by ``control_adapter``; this node gates on its existence.
        """
        k = tick_index(msg.header.stamp)
        with self._cmd_cond:
            self._latest_cmd_tick = max(self._latest_cmd_tick, k)
            self._cmd_cond.notify_all()

    def _on_reset(
        self, request: Reset.Request, response: Reset.Response
    ) -> Reset.Response:
        """Queue the reset for the tick thread and block until it was applied.

        Executor thread, hence the hand-off: CARLA calls belong to the tick
        thread, and a teleport in the middle of a tick would put the episode
        boundary inside a frame. The service sits in a reentrant callback
        group so blocking here does not stall the command subscription.
        """
        pending = _PendingReset(request, threading.Event(), response)
        with self._pending_lock:
            if self._pending_reset is not None:
                response.ok = False
                response.message = "a reset is already pending"
                return response
            self._pending_reset = pending
        if not pending.done.wait(timeout=self._startup_timeout_s + 30.0):
            # Withdraw it: otherwise the tick loop would still apply a reset
            # the caller was told failed, refuse the caller's retry with "a
            # reset is already pending" until then, and write into a
            # response object that was already returned.
            with self._pending_lock:
                if self._pending_reset is pending:
                    self._pending_reset = None
            response.ok = False
            response.message = "reset did not complete in time"
        return pending.response

    def _on_set_weather(
        self, request: SetWeather.Request, response: SetWeather.Response
    ) -> SetWeather.Response:
        """Apply a named ``carla.WeatherParameters`` preset (rendering only)."""
        preset = WEATHER_PRESETS.get(request.preset)
        if preset is None:
            response.ok = False
            self.get_logger().warning(f"unknown weather preset {request.preset!r}")
            return response
        self._world.set_weather(preset)
        response.ok = True
        return response

    # ------------------------------------------------------------ tick loop
    def run(self) -> None:
        """Tick until shutdown (main thread; the executor spins elsewhere).

        Per iteration: apply a queued reset if any, pace to the requested
        real-time factor, then one gated tick (:meth:`_tick_once`). Episode
        0 starts with the ``ResetEvent`` every other node waits for before
        it accepts data.
        """
        self._wait_for_peers()
        # Before the first tick the client snapshot of a freshly spawned actor
        # is empty and get_transform() is the identity, so episode 0's start
        # pose comes from the spawn transform itself.
        self._publish_reset_event(self._base_pose_of(self._spawn_pose))
        while rclpy.ok():
            self._process_pending_reset()
            self._pace()
            self._tick_once()

    def _wait_for_peers(self) -> None:
        """Before the first tick only: wait until a controller is discovered.

        A node that joins after a tick was published never sees that tick and
        can never answer it, so the first tick waits for a publisher on the
        command topic and a subscriber on the GT ego odometry. (A /clock
        subscriber count would be vacuous: this node's own use_sim_time
        subscription counts.) This is wall clock before any tick exists, so
        it cannot change results; the bound is the startup timeout.
        """
        deadline = time.monotonic() + self._startup_timeout_s
        last_log = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            ready = (
                self.count_publishers(TOPIC_CONTROL_COMMAND) > 0
                and self.count_subscribers(TOPIC_GT_EGO_ODOM) > 0
            )
            if ready:
                time.sleep(0.5)  # let endpoint matching finish
                return
            if time.monotonic() - last_log > 5.0:
                last_log = time.monotonic()
                self.get_logger().info(
                    "waiting for peers before the first tick: "
                    f"command publishers {self.count_publishers(TOPIC_CONTROL_COMMAND)}, "
                    f"ego_odom subscribers {self.count_subscribers(TOPIC_GT_EGO_ODOM)}"
                )
            time.sleep(0.2)
        self.get_logger().warning(
            "no controller discovered before the first tick; ticking anyway"
        )

    def _pace(self) -> None:
        """Sleep so ticks come no faster than ``realtime_factor`` x real time.

        0 means as fast as the gate allows (evaluation); 1 is real time, for
        watching in Foxglove. Slower never changes results: nodes act on
        ticks, not on the wall clock.
        """
        if self._realtime_factor <= 0.0:
            return
        period = self._dt / self._realtime_factor
        elapsed = time.monotonic() - self._last_tick_wall
        if elapsed < period:
            time.sleep(period - elapsed)

    def _tick_once(self) -> None:
        """One lockstep tick: simulate, publish, gate, report.

        1. ``world.tick()`` advances the server one frame; the snapshot gives
           its sim time, from which the tick index k and the stamp every
           message of this tick carries are derived.
        2. Publish ``/clock`` first (so subscribers' clocks move before any
           data of the tick), then the GT topics.
        3. Block until a ``ControlCommand`` stamped k arrives, up to the
           lockstep timeout (the startup timeout on an episode's first tick,
           while the nodes plan). On a timeout publish ``TickTimeout`` so the
           nodes can fall back (docs/02 §2), and tick anyway.
        4. Publish this node's ``NodeDiag`` (``cycle_ms`` = the GT publish,
           ``input_age_ms`` = how long the gate waited).
        """
        wall_start = time.monotonic()
        self._last_tick_wall = wall_start
        self._world.tick()
        snapshot = self._world.get_snapshot()
        elapsed_s = float(snapshot.timestamp.elapsed_seconds)
        frame = int(snapshot.frame)
        if self._last_frame is not None and frame != self._last_frame + 1:
            # A server that advances frames on its own (seen after clients were
            # SIGKILLed mid-episode) breaks lockstep silently: this node ticks,
            # publishes and gates once per several frames and the stack drives
            # on a 0.35 s control period. Only a server restart fixes it.
            self.get_logger().error(
                f"world advanced {frame - self._last_frame} frames during one "
                "tick: the server is not in lockstep; restart it "
                "(tools/carla/start_carla.sh)",
                throttle_duration_sec=5.0,
            )
        self._last_frame = frame
        stamp = stamp_from_seconds(elapsed_s)
        k = tick_index(elapsed_s)
        clock = Clock()
        clock.clock = stamp
        self._pub_clock.publish(clock)
        self._gt.publish(stamp)
        publish_ms = 1000.0 * (time.monotonic() - wall_start)

        timeout_s = (
            self._startup_timeout_s
            if self._startup_pending
            else self._lockstep_timeout_s
        )
        if self._startup_pending:
            self.get_logger().info(
                f"first tick of episode {self._episode_id}: k={k} "
                f"(sim {elapsed_s:.3f}s); waiting up to {timeout_s:.0f}s for its ControlCommand"
            )
        waited_s, timed_out = self._wait_for_command(k, timeout_s)
        self._startup_pending = False
        status = NodeDiag.STATUS_OK
        message = ""
        if timed_out:
            self._timeouts += 1
            status = NodeDiag.STATUS_ERROR
            message = f"lockstep timeout: no ControlCommand for tick {k} within {waited_s:.1f}s"
            self.get_logger().error(message)
            timeout_msg = TickTimeout()
            timeout_msg.header.stamp = stamp
            timeout_msg.episode_id = self._episode_id
            timeout_msg.waited_s = float(waited_s)
            self._pub_tick_timeout.publish(timeout_msg)
        elif self._gt.warnings:
            status = NodeDiag.STATUS_WARN
            message = "; ".join(self._gt.warnings)
            self.get_logger().warning(message)
        diag = NodeDiag()
        diag.header.stamp = stamp
        diag.node = NODE_NAME
        diag.cycle_ms = float(publish_ms)
        diag.input_age_ms = float(1000.0 * waited_s)
        diag.status = int(status)
        diag.message = message
        self._pub_diag.publish(diag)

    def _wait_for_command(self, k: int, timeout_s: float) -> tuple[float, bool]:
        """Block until a ControlCommand stamped tick k (or later) arrived.

        Returns ``(waited_s, timed_out)``. "Or later" because a command for
        a newer tick can only exist after a timeout let the world move on,
        and then it is the one to honour. The condition variable is notified
        by :meth:`_on_control_command` on the executor thread.
        """
        start = time.monotonic()
        deadline = start + timeout_s
        with self._cmd_cond:
            while self._latest_cmd_tick < k and rclpy.ok():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return time.monotonic() - start, True
                self._cmd_cond.wait(timeout=min(remaining, 0.5))
        return time.monotonic() - start, False

    # ---------------------------------------------------------------- reset
    def _base_pose_of(self, actor_pose: SE3) -> SE3:
        """base_link pose (ROS) of an actor pose.

        CARLA poses a vehicle at its mesh origin, slightly above the road
        under the middle of the car; the stack's ``base_link`` is the rear
        axle on the ground, ``base_link_in_actor`` away from it
        (``VehicleGeometry``), so the two poses differ by that offset rotated
        into the world.
        """
        base = compose(
            actor_pose,
            SE3(self._vehicle.base_link_in_actor, np.array([0.0, 0.0, 0.0, 1.0])),
        )
        assert isinstance(base, SE3)
        return base

    def _process_pending_reset(self) -> None:
        """Apply a queued reset on the tick thread and release the service."""
        with self._pending_lock:
            pending = self._pending_reset
        if pending is None:
            return
        try:
            self._apply_reset(pending.request, pending.response)
        finally:
            with self._pending_lock:
                self._pending_reset = None
            pending.done.set()

    def _apply_reset(self, request: Reset.Request, response: Reset.Response) -> None:
        """Start a new episode: teleport the hero, then one tick through the gate.

        The target is a spawn point (``spawn_index >= 0``) or a ``base_link``
        pose, converted to the actor origin and lifted a little so the car
        settles onto the road instead of intersecting it. The physics body is
        recreated (see the comment below), the car is brought to rest with
        the brake on, the episode id is incremented, the GT ring buffers and
        the command gate are cleared, and ``ResetEvent`` goes out *before*
        the first tick so every node has reset its state by the time that
        tick's data arrives. The first tick then waits with the startup
        timeout, since the planner has to build a route first.
        """
        if request.spawn_index >= 0:
            spawn_points = self._world.get_map().get_spawn_points()
            transform = spawn_points[request.spawn_index % len(spawn_points)]
            actor_pose = transform_to_ros(transform.location, transform.rotation)
        else:
            base_pose = se3_from_pose(request.start_pose)
            actor_in_base = inverse(
                SE3(self._vehicle.base_link_in_actor, np.array([0.0, 0.0, 0.0, 1.0]))
            )
            assert isinstance(actor_in_base, SE3)
            raw_pose = compose(base_pose, actor_in_base)
            assert isinstance(raw_pose, SE3)
            actor_pose = SE3(
                raw_pose.translation + np.array([0.0, 0.0, RESET_DROP_MARGIN_M]),
                raw_pose.rotation,
            )
        rpy = quaternion_to_rpy(actor_pose.rotation)
        loc = location_from_ros(actor_pose.translation)
        rot = rotation_from_ros(rpy)
        # Toggling physics recreates the PhysX vehicle, so suspension travel,
        # wheel spin and the rest of its internal state do not carry over
        # from the previous episode (they did: the first tick after a
        # teleport from a moving hero differed by 1.7 cm and 0.1 m/s from
        # one after a standing hero, and the bit-identical criterion needs
        # every episode of a route to start from the same state).
        self._hero.set_simulate_physics(False)
        self._hero.set_transform(
            carla.Transform(
                carla.Location(x=loc.x, y=loc.y, z=loc.z),
                carla.Rotation(pitch=rot.pitch, yaw=rot.yaw, roll=rot.roll),
            )
        )
        self._hero.set_simulate_physics(True)
        self._hero.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
        self._hero.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
        self._hero.apply_control(
            carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)
        )
        if request.traffic_seed >= 0:
            self._traffic_manager.set_random_device_seed(int(request.traffic_seed))
        # TODO(M1): clear_traffic and respawn traffic with the seed.
        self._episode_id += 1
        self._gt.reset()
        with self._cmd_cond:
            self._latest_cmd_tick = -1
        self._startup_pending = True
        self._publish_reset_event(self._base_pose_of(actor_pose))
        self._tick_once()
        response.ok = True
        response.episode_id = self._episode_id
        response.message = f"episode {self._episode_id} at spawn {request.spawn_index}"
        self.get_logger().info(response.message)

    def _publish_reset_event(self, start_pose: SE3) -> None:
        """ResetEvent stamped with the first tick of the new episode.

        Published on the transient_local ``event`` QoS so a node that starts
        late still receives it; consumers ignore stamps before it and
        episode ids not newer than the one they know (docs/02 §7).
        """
        # The exact tick stamp, not elapsed + dt in float: consumers compare
        # tick indices, and the first pose of the episode carries CARLA's own
        # float accumulation of the same tick.
        next_k = tick_index(self._world.get_snapshot().timestamp.elapsed_seconds) + 1
        sec, nanosec = tick_stamp(next_k)
        msg = ResetEvent()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nanosec
        msg.episode_id = self._episode_id
        msg.town = self._town
        msg.start_pose = pose_from_se3(start_pose)
        self._pub_reset_event.publish(msg)

    # ------------------------------------------------------------- shutdown
    def shutdown(self) -> None:
        """Destroy every spawned actor and leave the server in async mode."""
        self.get_logger().info(
            f"shutting down after {self._timeouts} lockstep timeouts"
        )
        self._destroy_actors()
        try:
            settings = self._world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self._world.apply_settings(settings)
            self._traffic_manager.set_synchronous_mode(False)
        except RuntimeError as err:
            self.get_logger().warning(f"restoring async mode failed: {err}")

    def _destroy_actors(self) -> None:
        """Destroy the rig (if spawned) and the hero."""
        self._sensor_rig.destroy()
        try:
            if self._hero.is_alive:
                self._hero.destroy()
        except RuntimeError as err:
            self.get_logger().warning(f"destroying hero failed: {err}")


def main(args: list[str] | None = None) -> int:
    """Entry point: spin the executor in a thread, tick in the main thread.

    The reverse arrangement (tick in a timer callback) would put CARLA calls
    on executor threads and let a slow service callback delay a tick; with
    the loop in the main thread the executor only ever touches the small
    shared state under its locks. Shutdown always destroys the actors and
    restores asynchronous mode, or the next launch finds a second hero and a
    frozen world.
    """
    rclpy.init(args=args)
    node: WorldManagerNode | None = None
    try:
        node = WorldManagerNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        spinner = threading.Thread(target=executor.spin, daemon=True)
        spinner.start()
        node.run()
    except KeyboardInterrupt:
        pass
    except RuntimeError as err:
        rclpy.logging.get_logger(NODE_NAME).error(f"fatal: {err}")
        return 1
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
