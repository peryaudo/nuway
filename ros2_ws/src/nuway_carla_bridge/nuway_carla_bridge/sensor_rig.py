"""Sensor rig spawning, /tf_static and camera_info (M0). Module, not a node.

Hosted by the ``world_manager`` node (one CARLA client per process,
``docs/01_directory_structure.md`` Rules). Every sensor gets ``ros_name`` (the
attribute CARLA names topics from) and ``ros_publish_tf=false`` (CARLA would
otherwise re-send the extrinsics on ``/tf`` every tick, parented to ``hero``);
``enable_for_ros()`` is called on every sensor except ``viz_only`` ones. All
geometry comes from ``nuway_ml.common.rig``: this file holds none.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import carla
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from tf2_ros import StaticTransformBroadcaster

from nuway_ml.common.frames import FRAME_BASE_LINK, TOPIC_CAMERA_INFO_FMT
from nuway_ml.common.rig import (
    Rig,
    SensorSpec,
    VehicleGeometry,
    intrinsics,
    sensor_in_base_link,
)
from nuway_rclpy.ros_conv import transform_from_se3
from nuway_rclpy.ros_qos import qos

if TYPE_CHECKING:
    from rclpy.publisher import Publisher


class SensorRig:
    """Spawns the rig on the hero and publishes its static side (TF, camera_info)."""

    def __init__(
        self,
        node: Node,
        world: carla.World,
        hero: carla.Actor,
        rig: Rig,
        vehicle: VehicleGeometry,
        *,
        label_only: bool,
        viz_only: bool,
    ) -> None:
        """Select the entries to spawn; nothing is spawned until :meth:`spawn`."""
        self._node = node
        self._world = world
        self._hero = hero
        self._vehicle = vehicle
        self._specs = rig.select(label_only=label_only, viz_only=viz_only)
        self._actors: list[carla.Actor] = []
        self._tf_broadcaster = StaticTransformBroadcaster(node)
        self._pub_camera_info: dict[str, Publisher] = {}

    @property
    def actors(self) -> list[carla.Actor]:
        """The spawned sensor actors."""
        return list(self._actors)

    def spawn(self) -> None:
        """Spawn every selected sensor attached to the hero."""
        lib = self._world.get_blueprint_library()
        for spec in self._specs:
            bp = lib.find(spec.type)
            bp.set_attribute("ros_name", spec.id)
            bp.set_attribute("role_name", spec.id)
            bp.set_attribute("ros_publish_tf", "false")
            for key, value in spec.attributes.items():
                bp.set_attribute(key, value)
            transform = carla.Transform(
                carla.Location(x=spec.x, y=spec.y, z=spec.z),
                carla.Rotation(pitch=spec.pitch, yaw=spec.yaw, roll=spec.roll),
            )
            actor = self._world.spawn_actor(bp, transform, attach_to=self._hero)
            if not spec.viz_only:
                actor.enable_for_ros()
            self._actors.append(actor)
            self._node.get_logger().info(f"spawned {spec.type} as {spec.id}")
        self._publish_static_tf()
        self._publish_camera_info()

    def _publish_static_tf(self) -> None:
        """Latched base_link -> <sensor> for every spawned, non-viz sensor."""
        transforms = []
        for spec in self._specs:
            if spec.viz_only:
                continue
            msg = TransformStamped()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = FRAME_BASE_LINK
            msg.child_frame_id = spec.id
            msg.transform = transform_from_se3(sensor_in_base_link(spec, self._vehicle))
            transforms.append(msg)
        if transforms:
            self._tf_broadcaster.sendTransform(transforms)

    def _publish_camera_info(self) -> None:
        """Latched /nuway/sensors/<cam>/camera_info from size + FOV (CARLA's is broken)."""
        for spec in self._specs:
            if not spec.is_camera or spec.viz_only:
                continue
            topic = TOPIC_CAMERA_INFO_FMT.format(cam=spec.id)
            pub = self._node.create_publisher(CameraInfo, topic, qos("latched"))
            pub.publish(camera_info_msg(spec))
            self._pub_camera_info[spec.id] = pub

    def destroy(self) -> None:
        """Destroy every spawned sensor."""
        for actor in reversed(self._actors):
            try:
                if actor.is_alive:
                    actor.stop()
                    actor.destroy()
            except RuntimeError as err:
                self._node.get_logger().warning(f"destroying sensor failed: {err}")
        self._actors.clear()


def camera_info_msg(spec: SensorSpec) -> CameraInfo:
    """Pinhole CameraInfo for a rig camera, frame_id = its ros_name."""
    width, height = spec.image_size
    k = intrinsics(spec)
    msg = CameraInfo()
    msg.header.frame_id = spec.id
    msg.width = width
    msg.height = height
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [float(v) for v in k.ravel()]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [
        float(v)
        for v in [
            k[0, 0],
            0.0,
            k[0, 2],
            0.0,
            0.0,
            k[1, 1],
            k[1, 2],
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
    ]
    return msg
