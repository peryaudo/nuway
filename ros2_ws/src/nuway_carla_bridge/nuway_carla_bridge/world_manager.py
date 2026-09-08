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
from nuway_ml.common.tick import TICK_DT_S, tick_index
from nuway_rclpy.ros_conv import (
    pose_from_se3,
    se3_from_pose,
    stamp_from_seconds,
)
from nuway_rclpy.ros_qos import CLOCK_QOS, qos

NODE_NAME = "world_manager"
HERO_ROLE = "hero"
RESET_DROP_MARGIN_M = 0.15  # teleport slightly above the ground and let physics settle
GENERATED_MAP_NAME = (
    "OpenDriveMap"  # CARLA's name for every generate_opendrive_world() map
)
OPENDRIVE_GENERATION_PARAMETERS = carla.OpendriveGenerationParameters(
    vertex_distance=2.0,
    max_road_length=500.0,
    wall_height=0.0,
    additional_width=1.0,
    smooth_junctions=True,
    enable_mesh_visibility=True,
    enable_pedestrian_navigation=False,
)
WEATHER_PRESETS = {
    name: getattr(carla.WeatherParameters, name)
    for name in dir(carla.WeatherParameters)
    if name[0].isupper()
    and isinstance(getattr(carla.WeatherParameters, name), carla.WeatherParameters)
}


@dataclass
class _PendingReset:
    """A reset request handed from the service thread to the tick loop."""

    request: Reset.Request
    done: threading.Event
    response: Reset.Response


class WorldManagerNode(Node):  # type: ignore[misc]  # rclpy.Node has no stubs (03 §7.3)
    """Owns the CARLA client, the lockstep tick loop, /clock and the GT topics."""

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

        self._hero = self._spawn_hero(int(p["spawn_index"]))
        self._sensor_rig = SensorRig(
            self,
            self._world,
            self._hero,
            self._rig,
            self._vehicle,
            label_only=bool(p["spawn_label_only_sensors"]),
            viz_only=bool(p["spawn_viz_only_sensors"]),
        )
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
        self.get_logger().info(
            f"ready: town {self._town}, dt {self._dt}, lockstep timeout {self._lockstep_timeout_s}s "
            f"(startup {self._startup_timeout_s}s), realtime_factor {self._realtime_factor}"
        )

    # -------------------------------------------------------------- set-up
    def _declare_params(self) -> dict[str, Any]:
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
        path = self._map_dir / self._town / "map.xodr"
        if path.is_file():
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self._world.get_map().to_opendrive())
        self.get_logger().info(f"exported {path}")

    def _apply_sync_settings(self, sync: bool, tm_port: int, seed: int) -> None:
        settings = self._world.get_settings()
        settings.synchronous_mode = sync
        settings.fixed_delta_seconds = self._dt
        self._world.apply_settings(settings)
        self._traffic_manager = self._client.get_trafficmanager(tm_port)
        self._traffic_manager.set_synchronous_mode(sync)
        self._traffic_manager.set_random_device_seed(seed)

    def _spawn_hero(self, spawn_index: int) -> carla.Actor:
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
        return hero

    # ------------------------------------------------------------ callbacks
    def _on_control_command(self, msg: ControlCommand) -> None:
        k = tick_index(msg.header.stamp)
        with self._cmd_cond:
            self._latest_cmd_tick = max(self._latest_cmd_tick, k)
            self._cmd_cond.notify_all()

    def _on_reset(
        self, request: Reset.Request, response: Reset.Response
    ) -> Reset.Response:
        pending = _PendingReset(request, threading.Event(), response)
        with self._pending_lock:
            if self._pending_reset is not None:
                response.ok = False
                response.message = "a reset is already pending"
                return response
            self._pending_reset = pending
        if not pending.done.wait(timeout=self._startup_timeout_s + 30.0):
            response.ok = False
            response.message = "reset did not complete in time"
        return pending.response

    def _on_set_weather(
        self, request: SetWeather.Request, response: SetWeather.Response
    ) -> SetWeather.Response:
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
        """Tick until shutdown (main thread; the executor spins elsewhere)."""
        self._wait_for_peers()
        self._publish_reset_event(self._hero_base_pose())
        while rclpy.ok():
            self._process_pending_reset()
            self._pace()
            self._tick_once()

    def _wait_for_peers(self) -> None:
        """Before the first tick only: wait until a controller is discovered.

        A node that joins after a tick was published never sees that tick and
        can never answer it, so the first tick waits for a publisher on the
        command topic and subscribers on /clock and the GT ego odometry. This
        is wall clock before any tick exists, so it cannot change results;
        the bound is the startup timeout.
        """
        deadline = time.monotonic() + self._startup_timeout_s
        last_log = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            ready = (
                self.count_publishers(TOPIC_CONTROL_COMMAND) > 0
                and self.count_subscribers(TOPIC_CLOCK) > 0
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
                    f"clock subscribers {self.count_subscribers(TOPIC_CLOCK)}, "
                    f"ego_odom subscribers {self.count_subscribers(TOPIC_GT_EGO_ODOM)}"
                )
            time.sleep(0.2)
        self.get_logger().warning(
            "no controller discovered before the first tick; ticking anyway"
        )

    def _pace(self) -> None:
        if self._realtime_factor <= 0.0:
            return
        period = self._dt / self._realtime_factor
        elapsed = time.monotonic() - self._last_tick_wall
        if elapsed < period:
            time.sleep(period - elapsed)

    def _tick_once(self) -> None:
        wall_start = time.monotonic()
        self._last_tick_wall = wall_start
        self._world.tick()
        snapshot = self._world.get_snapshot()
        elapsed_s = float(snapshot.timestamp.elapsed_seconds)
        stamp = stamp_from_seconds(elapsed_s)
        k = tick_index(elapsed_s)
        clock = Clock()
        clock.clock = stamp
        self._pub_clock.publish(clock)
        self._gt.publish(stamp, k)
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
        """Block until a ControlCommand stamped tick k (or later) arrived."""
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
    def _hero_base_pose(self) -> SE3:
        tf = self._hero.get_transform()
        actor = transform_to_ros(tf.location, tf.rotation)
        base = compose(
            actor, SE3(self._vehicle.base_link_in_actor, np.array([0.0, 0.0, 0.0, 1.0]))
        )
        assert isinstance(base, SE3)
        return base

    def _process_pending_reset(self) -> None:
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
        base_after = compose(
            actor_pose,
            SE3(self._vehicle.base_link_in_actor, np.array([0.0, 0.0, 0.0, 1.0])),
        )
        assert isinstance(base_after, SE3)
        self._publish_reset_event(base_after)
        self._tick_once()
        response.ok = True
        response.episode_id = self._episode_id
        response.message = f"episode {self._episode_id} at spawn {request.spawn_index}"
        self.get_logger().info(response.message)

    def _publish_reset_event(self, start_pose: SE3) -> None:
        """ResetEvent stamped with the first tick of the new episode."""
        next_s = float(self._world.get_snapshot().timestamp.elapsed_seconds) + self._dt
        msg = ResetEvent()
        msg.header.stamp = stamp_from_seconds(next_s)
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
        self._sensor_rig.destroy()
        try:
            if self._hero.is_alive:
                self._hero.destroy()
        except RuntimeError as err:
            self.get_logger().warning(f"destroying hero failed: {err}")
        try:
            settings = self._world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self._world.apply_settings(settings)
            self._traffic_manager.set_synchronous_mode(False)
        except RuntimeError as err:
            self.get_logger().warning(f"restoring async mode failed: {err}")


def main(args: list[str] | None = None) -> int:
    """Entry point: spin the executor in a thread, tick in the main thread."""
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
