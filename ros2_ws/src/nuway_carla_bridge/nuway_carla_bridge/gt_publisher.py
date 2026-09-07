"""Ground-truth publishers read from the CARLA Python API (M0). Module, not a node.

Called by ``world_manager`` right after every ``world.tick()`` in the same
process (one CARLA client per process). Publishes ``/nuway/gt/ego_odom``,
``/nuway/sim/vehicle_state``, ``/nuway/gt/agents`` and
``/nuway/gt/traffic_lights`` (``docs/02_interfaces.md`` §3.2). Every CARLA
quantity passes through ``nuway_ml.common.carla_conv``; no conversion
arithmetic lives here.

State across ticks: the per-actor history ring buffers (0.1 s spacing, sampled
on planning ticks) and the traffic-light id cache; :meth:`GtPublisher.reset`
clears the ring buffers (``docs/02_interfaces.md`` §7).
"""

from __future__ import annotations

import collections
from dataclasses import dataclass, field

import carla
import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from numpy.typing import NDArray
from nuway_msgs.msg import (
    Agent,
    AgentArray,
    LaneGraph,
    TrafficLight,
    TrafficLightArray,
    VehicleState,
)
from rclpy.node import Node

from nuway_carla_bridge.ros_conv import pose_from_se3
from nuway_carla_bridge.ros_qos import qos
from nuway_ml.common.carla_conv import (
    angular_velocity_to_ros,
    location_to_ros,
    transform_to_ros,
    yaw_to_ros,
)
from nuway_ml.common.frames import (
    FRAME_BASE_LINK,
    FRAME_MAP,
    TOPIC_GT_AGENTS,
    TOPIC_GT_EGO_ODOM,
    TOPIC_GT_TRAFFIC_LIGHTS,
    TOPIC_LANE_GRAPH,
    TOPIC_VEHICLE_STATE,
)
from nuway_ml.common.geometry import SE3, apply, compose, inverse, quaternion_to_yaw
from nuway_ml.common.rig import VehicleGeometry
from nuway_ml.common.tick import is_planning_tick

Array = NDArray[np.float64]

HISTORY_LEN = int(Agent.HISTORY_LEN)
TL_ID_UNMAPPED_FLAG = 0x80000000
TL_STOP_LINE_MATCH_M = 2.0
BICYCLE_HINTS = (
    "bike",
    "bicycle",
    "omafiets",
    "century",
    "diamondback",
    "gazelle",
    "crossbike",
)
TL_STATE = {
    carla.TrafficLightState.Red: TrafficLight.STATE_RED,
    carla.TrafficLightState.Yellow: TrafficLight.STATE_YELLOW,
    carla.TrafficLightState.Green: TrafficLight.STATE_GREEN,
    carla.TrafficLightState.Off: TrafficLight.STATE_OFF,
    carla.TrafficLightState.Unknown: TrafficLight.STATE_UNKNOWN,
}


@dataclass(frozen=True, slots=True)
class GtParams:
    """Tunables of the GT publisher (node parameters ``gt.*``)."""

    agent_radius_m: float = 100.0
    truck_blueprints: tuple[str, ...] = ()


@dataclass
class _LaneIndex:
    """Lane-graph lookup: (road_id, lane_id_odr) -> [(lane_id, centerline (N, 2))]."""

    by_odr: dict[tuple[int, int], list[tuple[int, Array]]] = field(default_factory=dict)
    tl_stop_lines: list[tuple[int, Array]] = field(default_factory=list)

    @classmethod
    def from_msg(cls, msg: LaneGraph) -> _LaneIndex:
        """Index a LaneGraph message."""
        index = cls()
        for lane in msg.lanes:
            pts = np.array([[p.x, p.y] for p in lane.centerline], dtype=np.float64)
            index.by_odr.setdefault(
                (int(lane.road_id), int(lane.lane_id_odr)), []
            ).append((int(lane.id), pts))
        for tl in msg.traffic_lights:
            index.tl_stop_lines.append(
                (int(tl.id), np.array([tl.stop_line.x, tl.stop_line.y]))
            )
        return index

    def lane_id_at(self, road_id: int, lane_id_odr: int, xy: Array) -> int | None:
        """Our lane id nearest to ``xy`` among lanes with this OpenDRIVE (road, lane)."""
        best: tuple[float, int] | None = None
        for lane_id, pts in self.by_odr.get((road_id, lane_id_odr), []):
            d = float(np.min(np.linalg.norm(pts - xy, axis=1)))
            if best is None or d < best[0]:
                best = (d, lane_id)
        return None if best is None else best[1]

    def tl_id_near(self, xy: Array) -> int | None:
        """TrafficLightMapping id whose stop line is within 2 m of ``xy``."""
        best: tuple[float, int] | None = None
        for tl_id, stop in self.tl_stop_lines:
            d = float(np.linalg.norm(stop - xy))
            if d < TL_STOP_LINE_MATCH_M and (best is None or d < best[0]):
                best = (d, tl_id)
        return None if best is None else best[1]


@dataclass
class _TlStatic:
    """Cached static part of one CARLA traffic light."""

    tl_id: int
    stop_line: Array
    affected_lane_ids: list[int]


class GtPublisher:
    """Publishes the GT topics from the CARLA API after every tick."""

    def __init__(
        self,
        node: Node,
        world: carla.World,
        hero: carla.Actor,
        vehicle: VehicleGeometry,
        params: GtParams,
    ) -> None:
        """Create the publishers and subscribe to the latched lane graph."""
        self._node = node
        self._world = world
        self._hero = hero
        self._vehicle = vehicle
        self._params = params
        self._pub_ego_odom = node.create_publisher(
            Odometry, TOPIC_GT_EGO_ODOM, qos("stream")
        )
        self._pub_vehicle_state = node.create_publisher(
            VehicleState, TOPIC_VEHICLE_STATE, qos("stream")
        )
        self._pub_agents = node.create_publisher(
            AgentArray, TOPIC_GT_AGENTS, qos("stream")
        )
        self._pub_traffic_lights = node.create_publisher(
            TrafficLightArray, TOPIC_GT_TRAFFIC_LIGHTS, qos("stream")
        )
        self._sub_lane_graph = node.create_subscription(
            LaneGraph, TOPIC_LANE_GRAPH, self._on_lane_graph, qos("latched")
        )
        self._lanes: _LaneIndex | None = None
        self._tl_static: dict[int, _TlStatic] = {}
        self._history: dict[int, collections.deque[tuple[float, float, float]]] = {}
        self._prev_body_velocity: Array | None = None
        self._unmapped_warned: set[int] = set()
        self.warnings: list[str] = []

    # ----------------------------------------------------------------- reset
    def reset(self) -> None:
        """Drop the history ring buffers (ResetEvent, docs/02 §7)."""
        self._history.clear()
        self._prev_body_velocity = None

    # ------------------------------------------------------------ lane graph
    def _on_lane_graph(self, msg: LaneGraph) -> None:
        self._lanes = _LaneIndex.from_msg(msg)
        self._tl_static.clear()
        self._node.get_logger().info(
            f"lane graph received: {len(msg.lanes)} lanes, {len(msg.traffic_lights)} lights"
        )

    # ------------------------------------------------------------- per tick
    def publish(self, stamp: Time, k: int) -> None:
        """Publish every GT topic for tick ``k`` stamped ``stamp``."""
        self.warnings = []
        hero_tf = self._hero.get_transform()
        actor_pose = transform_to_ros(hero_tf.location, hero_tf.rotation)
        self._publish_ego(stamp, actor_pose)
        self._publish_vehicle_state(stamp)
        self._publish_agents(stamp, k, actor_pose)
        self._publish_traffic_lights(stamp)

    def _publish_ego(self, stamp: Time, actor_pose: SE3) -> None:
        base_in_actor = SE3(
            self._vehicle.base_link_in_actor, np.array([0.0, 0.0, 0.0, 1.0])
        )
        base_pose = compose(actor_pose, base_in_actor)
        assert isinstance(base_pose, SE3)
        world_velocity = location_to_ros(self._hero.get_velocity())
        omega_world = angular_velocity_to_ros(self._hero.get_angular_velocity())
        # Velocity of the rear axle: v_origin + omega x r (all map frame).
        r_world = (
            apply(actor_pose, self._vehicle.base_link_in_actor) - actor_pose.translation
        )
        axle_velocity = world_velocity + np.cross(omega_world, r_world)
        body = inverse(actor_pose)
        assert isinstance(body, SE3)
        body_velocity = apply(SE3(np.zeros(3), body.rotation), axle_velocity)
        body_omega = apply(SE3(np.zeros(3), body.rotation), omega_world)
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_MAP
        msg.child_frame_id = FRAME_BASE_LINK
        msg.pose.pose = pose_from_se3(base_pose)
        msg.twist.twist.linear.x = float(body_velocity[0])
        msg.twist.twist.linear.y = float(body_velocity[1])
        msg.twist.twist.linear.z = float(body_velocity[2])
        msg.twist.twist.angular.x = float(body_omega[0])
        msg.twist.twist.angular.y = float(body_omega[1])
        msg.twist.twist.angular.z = float(body_omega[2])
        self._pub_ego_odom.publish(msg)

    def _publish_vehicle_state(self, stamp: Time) -> None:
        control = self._hero.get_control()
        fl = self._hero.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        fr = self._hero.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
        msg = VehicleState()
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_BASE_LINK
        msg.speed = float(self._hero.get_velocity().length())
        # Wheel angles are CARLA yaw-like (degrees, clockwise positive).
        msg.steering_angle = yaw_to_ros(0.5 * (float(fl) + float(fr)))
        msg.valid_steering = True
        msg.throttle = float(control.throttle)
        msg.brake = float(control.brake)
        msg.gear = int(control.gear)
        self._pub_vehicle_state.publish(msg)

    def _class_id(self, actor: carla.Actor) -> int:
        type_id = str(actor.type_id)
        if type_id.startswith("walker."):
            return int(Agent.CLASS_PEDESTRIAN)
        if type_id.startswith("static.prop"):
            return int(Agent.CLASS_STATIC_OBSTACLE)
        if not type_id.startswith("vehicle."):
            return int(Agent.CLASS_UNKNOWN)
        if actor.attributes.get("number_of_wheels", "4") == "2":
            is_bicycle = any(h in type_id for h in BICYCLE_HINTS)
            return int(Agent.CLASS_BICYCLE if is_bicycle else Agent.CLASS_MOTORCYCLE)
        is_truck = type_id in self._params.truck_blueprints
        return int(Agent.CLASS_TRUCK if is_truck else Agent.CLASS_CAR)

    def _publish_agents(self, stamp: Time, k: int, hero_pose: SE3) -> None:
        msg = AgentArray()
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_MAP
        hero_xy = hero_pose.translation[:2]
        seen: set[int] = set()
        sample = is_planning_tick(k)
        for actor in self._world.get_actors():
            type_id = str(actor.type_id)
            if actor.id == self._hero.id or not (
                type_id.startswith(("vehicle.", "walker.", "static.prop"))
            ):
                continue
            tf = actor.get_transform()
            pose = transform_to_ros(tf.location, tf.rotation)
            if (
                float(np.linalg.norm(pose.translation[:2] - hero_xy))
                > self._params.agent_radius_m
            ):
                continue
            bbox = actor.bounding_box
            center = apply(pose, location_to_ros(bbox.location))
            yaw = quaternion_to_yaw(pose.rotation)
            velocity = location_to_ros(actor.get_velocity())
            omega = angular_velocity_to_ros(actor.get_angular_velocity())
            agent = Agent()
            agent.id = int(actor.id)
            agent.class_id = self._class_id(actor)
            agent.score = 1.0
            agent.pose = pose_from_se3(SE3(center, pose.rotation))
            agent.length = float(2.0 * bbox.extent.x)
            agent.width = float(2.0 * bbox.extent.y)
            agent.height = float(2.0 * bbox.extent.z)
            agent.vx = float(velocity[0])
            agent.vy = float(velocity[1])
            agent.yaw_rate = float(omega[2])
            agent.visible = True
            history = self._history.setdefault(
                int(actor.id), collections.deque(maxlen=HISTORY_LEN)
            )
            flat = [0.0] * (3 * HISTORY_LEN)
            for i, (hx, hy, hyaw) in enumerate(reversed(history)):
                flat[3 * i : 3 * i + 3] = [hx, hy, hyaw]
            agent.history_len = len(history)
            agent.history = flat
            if sample:
                history.append((float(center[0]), float(center[1]), float(yaw)))
            seen.add(int(actor.id))
            msg.agents.append(agent)
        for actor_id in [a for a in self._history if a not in seen]:
            del self._history[actor_id]
        self._pub_agents.publish(msg)

    def _tl_static_for(self, tl: carla.Actor) -> _TlStatic:
        cached = self._tl_static.get(int(tl.id))
        if cached is not None:
            return cached
        stop_wps = tl.get_stop_waypoints()
        if stop_wps:
            loc = stop_wps[0].transform.location
            stop_line = location_to_ros(loc)
        else:
            stop_line = location_to_ros(tl.get_transform().location)
        affected: list[int] = []
        tl_id: int | None = None
        if self._lanes is not None:
            tl_id = self._lanes.tl_id_near(stop_line[:2])
            for wp in tl.get_affected_lane_waypoints():
                xy = location_to_ros(wp.transform.location)[:2]
                lane_id = self._lanes.lane_id_at(int(wp.road_id), int(wp.lane_id), xy)
                if lane_id is not None and lane_id not in affected:
                    affected.append(lane_id)
        if tl_id is None:
            tl_id = int(tl.id) | TL_ID_UNMAPPED_FLAG
            if self._lanes is not None and int(tl.id) not in self._unmapped_warned:
                self._unmapped_warned.add(int(tl.id))
                self.warnings.append(
                    f"traffic light actor {tl.id} has no LaneGraph mapping within 2 m"
                )
        entry = _TlStatic(tl_id, stop_line, affected)
        if self._lanes is not None:
            self._tl_static[int(tl.id)] = entry
        return entry

    def _publish_traffic_lights(self, stamp: Time) -> None:
        msg = TrafficLightArray()
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_MAP
        for tl in self._world.get_actors().filter("traffic.traffic_light"):
            static = self._tl_static_for(tl)
            light = TrafficLight()
            light.id = int(static.tl_id)
            light.state = int(TL_STATE.get(tl.get_state(), TrafficLight.STATE_UNKNOWN))
            light.stop_line = Point(
                x=float(static.stop_line[0]),
                y=float(static.stop_line[1]),
                z=float(static.stop_line[2]),
            )
            light.affected_lane_ids = list(static.affected_lane_ids)
            light.confidence = 1.0
            light.time_in_state = float(tl.get_elapsed_time())
            light.yellow_duration = float(tl.get_yellow_time())
            light.latched = False
            msg.lights.append(light)
        self._pub_traffic_lights.publish(msg)
