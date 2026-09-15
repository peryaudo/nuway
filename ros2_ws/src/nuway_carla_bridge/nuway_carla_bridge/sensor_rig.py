"""Sensor rig spawning, /tf_static and camera_info (M0). Module, not a node.

Hosted by the ``world_manager`` node (one CARLA client per process,
``docs/01_directory_structure.md`` Rules). Every sensor gets ``ros_name`` (the
attribute CARLA names topics from) and ``ros_publish_tf=false`` (CARLA would
otherwise re-send the extrinsics on ``/tf`` every tick, parented to ``hero``);
``enable_for_ros()`` is called on every sensor except ``viz_only`` ones. All
geometry comes from ``nuway_ml.common.rig``: this file holds none.

How sensors work in CARLA. A sensor is an actor attached to the hero with a
fixed relative transform (CARLA frame: x forward, y right, z up, degrees).
It renders or samples once per simulated frame, so in synchronous mode
every sensor output belongs to exactly one tick. With the server started
``--ros2``, each sensor that had ``enable_for_ros()`` publishes its data
natively over DDS on ``/carla/<ros_name>/<sensor ros_name>/...`` (images,
point clouds, IMU, GNSS); no sensor data passes through this process, which
is why the bridge stays a Python node without a throughput problem. What
this module adds is the static side that CARLA's own output lacks or gets
wrong: the extrinsics on ``/tf_static`` (latched, sent once; sensor frames
are named by ``ros_name`` under ``base_link``) and a correct pinhole
``CameraInfo`` per camera (CARLA's has broken intrinsics, ``docs/00`` §4).

Rig selection: ``label_only`` sensors (the semantic lidar that labels
training data) and ``viz_only`` sensors (the chase camera for Foxglove)
are only spawned when the profile asks for them, so a plain driving run
spawns the sensors the driving stack consumes and nothing more.
"""

from __future__ import annotations

import io
from typing import TYPE_CHECKING

import carla
import numpy as np
from geometry_msgs.msg import TransformStamped
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage
from tf2_ros import StaticTransformBroadcaster

from nuway_ml.common.frames import (
    CHASE_CAM_ID,
    FRAME_BASE_LINK,
    TOPIC_CAMERA_INFO_FMT,
    TOPIC_VIZ_CHASE_CAM,
)
from nuway_ml.common.rig import (
    Rig,
    SensorSpec,
    VehicleGeometry,
    intrinsics,
    sensor_in_base_link,
)
from nuway_ml.common.tick import tick_index, tick_stamp
from nuway_rclpy.ros_conv import transform_from_se3
from nuway_rclpy.ros_qos import qos

if TYPE_CHECKING:
    from rclpy.publisher import Publisher

CHASE_JPEG_QUALITY = 80


def encode_jpeg(raw_bgra: bytes, width: int, height: int) -> bytes:
    """JPEG-encode a CARLA RGB camera frame (its ``raw_data`` is BGRA, row-major)."""
    pixels = np.frombuffer(raw_bgra, dtype=np.uint8).reshape(height, width, 4)
    rgb = pixels[:, :, 2::-1]  # BGRA -> RGB
    buf = io.BytesIO()
    Image.fromarray(np.ascontiguousarray(rgb)).save(
        buf, format="JPEG", quality=CHASE_JPEG_QUALITY
    )
    return buf.getvalue()


class SensorRig:
    """Spawns the rig on the hero and publishes its static side (TF, camera_info).

    Lifecycle: constructed with the rig selection, :meth:`spawn` creates the
    actors (all or nothing is the caller's job: ``world_manager`` destroys
    what was spawned when a later step fails), :meth:`destroy` removes them;
    CARLA keeps actors alive across client disconnects, so every spawn needs
    its destroy.
    """

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
        self._pub_chase: Publisher | None = None

    @property
    def actors(self) -> list[carla.Actor]:
        """The spawned sensor actors."""
        return list(self._actors)

    def spawn(self) -> None:
        """Spawn every selected sensor attached to the hero.

        ``ros_name`` (the attribute CARLA names topics from) and
        ``role_name`` (what generic CARLA tooling reads) are both set to the
        rig id; ``ros_publish_tf`` is off because CARLA would otherwise
        re-send the extrinsics on ``/tf`` every frame, parented to the hero
        actor rather than ``base_link``. Attaching to the hero makes the
        transform relative and moves the sensor with the car.
        """
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
            elif spec.id == CHASE_CAM_ID:
                self._listen_chase(actor)
            self._actors.append(actor)
            self._node.get_logger().info(f"spawned {spec.type} as {spec.id}")
        self._publish_static_tf()
        self._publish_camera_info()

    def _listen_chase(self, actor: carla.Actor) -> None:
        """Publish the chase camera through a ``listen()`` callback (docs/02 §8.3).

        The one sensor read through the Python API rather than
        ``enable_for_ros()``: it is a witness for the eval renders, not an
        input, so it goes out JPEG-compressed on the viz QoS at the rig's
        ``sensor_tick`` (every 5th tick) instead of as a raw native image.
        The callback runs on the CARLA client thread; publishing from there
        is safe because the executor never touches this publisher.
        """
        self._pub_chase = self._node.create_publisher(
            CompressedImage, TOPIC_VIZ_CHASE_CAM, qos("viz")
        )

        def on_image(image: carla.Image) -> None:
            if self._pub_chase is None:
                return
            msg = CompressedImage()
            sec, nanosec = tick_stamp(tick_index(float(image.timestamp)))
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = nanosec
            msg.header.frame_id = CHASE_CAM_ID
            msg.format = "jpeg"
            msg.data = encode_jpeg(
                bytes(image.raw_data), int(image.width), int(image.height)
            )
            self._pub_chase.publish(msg)

        actor.listen(on_image)

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
    """Pinhole CameraInfo for a rig camera, frame_id = its ros_name.

    ``K`` (3x3 intrinsics from the image size and horizontal field of view,
    ``nuway_ml.common.rig.intrinsics``) maps a camera-frame point to pixels;
    ``P`` is the same matrix as the 3x4 projection of a monocular camera
    (zero translation), ``R`` the identity (no stereo rectification) and the
    distortion zero: CARLA renders an ideal pinhole.
    """
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
