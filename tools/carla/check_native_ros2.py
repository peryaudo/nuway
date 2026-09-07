"""Smoke test of CARLA's native ROS 2 interface against a running --ros2 server (M0 task 5).

Spawns a hero with LiDAR, cameras, IMU and GNSS through the CARLA Python API
with ``ros_name`` set and ``enable_for_ros()`` called, ticks in synchronous
mode, and re-asserts the facts recorded in ``docs/02_interfaces.md`` §1 and §3.1:

- topic names ``/carla/hero/<ros_name>/{point_cloud,image,camera_info}``,
  ``/carla/hero/imu``, ``/carla/hero/gnss`` appear;
- every native message carries ``header.frame_id == ros_name``;
- exactly one LiDAR sweep, one IMU and one GNSS sample per tick, and the camera
  drop rate is measured (best-effort QoS);
- CARLA's ``camera_info`` focal length is unusable (reported, not asserted);
- native topics are ROS-convention: a sensor mounted on the CARLA right comes
  back on ``/tf`` with negative y, and while the hero steers left the IMU yaw
  rate is positive while CARLA's own yaw decreases.

Usage: ``uv run tools/carla/check_native_ros2.py [--host H] [--port P] [--town T] [--ticks N]``.
Exit status is 1 if any assertion fails. Run it again on every CARLA upgrade.
"""

from __future__ import annotations

import argparse
import contextlib
import itertools
import math
import sys
import time
from dataclasses import dataclass, field

import carla
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, NavSatFix, PointCloud2
from tf2_msgs.msg import TFMessage

HERO = "hero"
WARMUP_TICKS = 40
TICK_S = 0.05
EXPECTED_FX = 400.0 / math.tan(math.radians(45.0))


@dataclass(frozen=True, slots=True)
class SensorEntry:
    """One sensor of the verification rig (CARLA convention, meters)."""

    id: str
    type: str
    x: float
    y: float
    z: float
    attributes: dict[str, str]


LIDAR_ATTRS = {
    "channels": "32",
    "range": "75",
    "points_per_second": "600000",
    "rotation_frequency": "20",
    "upper_fov": "10",
    "lower_fov": "-30",
    "sensor_tick": "0.05",
}
RIG = [
    SensorEntry("lidar_top", "sensor.lidar.ray_cast", 0.0, 0.0, 2.4, LIDAR_ATTRS),
    SensorEntry(
        "cam_front",
        "sensor.camera.rgb",
        1.5,
        0.0,
        2.0,
        {
            "image_size_x": "800",
            "image_size_y": "450",
            "fov": "90",
            "sensor_tick": "0.05",
        },
    ),
    # Mounted 1.5 m to the CARLA right: must come back on /tf with y = -1.5.
    SensorEntry(
        "cam_asym",
        "sensor.camera.rgb",
        1.0,
        1.5,
        2.0,
        {
            "image_size_x": "320",
            "image_size_y": "180",
            "fov": "90",
            "sensor_tick": "0.05",
        },
    ),
    SensorEntry("imu", "sensor.other.imu", 0.0, 0.0, 0.0, {"sensor_tick": "0.05"}),
    SensorEntry("gnss", "sensor.other.gnss", 0.0, 0.0, 0.0, {"sensor_tick": "0.05"}),
]
EXPECTED_TOPICS = {
    "lidar_top": "/carla/hero/lidar_top/point_cloud",
    "cam_front": "/carla/hero/cam_front/image",
    "cam_front_info": "/carla/hero/cam_front/camera_info",
    "imu": "/carla/hero/imu",
    "gnss": "/carla/hero/gnss",
}
ONE_PER_TICK = ("lidar_top", "imu", "gnss", "cam_front_info")
BEST_EFFORT = QoSProfile(
    depth=50,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


@dataclass
class TopicStats:
    """Per-topic message count, distinct stamps and observed frame ids."""

    count: int = 0
    stamps: set[tuple[int, int]] = field(default_factory=set)
    frame_ids: set[str] = field(default_factory=set)

    def clear(self) -> None:
        """Forget everything seen so far (end of warm-up)."""
        self.count = 0
        self.stamps.clear()


class ListenerNode(Node):  # type: ignore[misc]  # rclpy.Node has no stubs (03 §7.3)
    """Subscribes to the native topics and records stats, IMU yaw rate and /tf."""

    def __init__(self) -> None:
        """Create every subscription up front so discovery overlaps the spawn."""
        super().__init__("check_native_ros2")
        self.stats: dict[str, TopicStats] = {k: TopicStats() for k in EXPECTED_TOPICS}
        self.imu_yaw_rates: list[float] = []
        self.imu_accel_y: list[float] = []
        self.camera_info: CameraInfo | None = None
        self.tf_asym: tuple[float, float, float] | None = None
        self.tf_count = 0
        self.record_imu = False
        self._sub_lidar = self.create_subscription(
            PointCloud2, EXPECTED_TOPICS["lidar_top"], self._on_lidar, BEST_EFFORT
        )
        self._sub_image = self.create_subscription(
            Image, EXPECTED_TOPICS["cam_front"], self._on_image, BEST_EFFORT
        )
        self._sub_info = self.create_subscription(
            CameraInfo,
            EXPECTED_TOPICS["cam_front_info"],
            self._on_camera_info,
            BEST_EFFORT,
        )
        self._sub_imu = self.create_subscription(
            Imu, EXPECTED_TOPICS["imu"], self._on_imu, BEST_EFFORT
        )
        self._sub_gnss = self.create_subscription(
            NavSatFix, EXPECTED_TOPICS["gnss"], self._on_gnss, BEST_EFFORT
        )
        self._sub_tf = self.create_subscription(
            TFMessage, "/tf", self._on_tf, BEST_EFFORT
        )

    def _record(
        self, key: str, msg: PointCloud2 | Image | CameraInfo | Imu | NavSatFix
    ) -> None:
        stats = self.stats[key]
        stats.count += 1
        stats.stamps.add((msg.header.stamp.sec, msg.header.stamp.nanosec))
        stats.frame_ids.add(msg.header.frame_id)

    def _on_lidar(self, msg: PointCloud2) -> None:
        self._record("lidar_top", msg)

    def _on_image(self, msg: Image) -> None:
        self._record("cam_front", msg)

    def _on_gnss(self, msg: NavSatFix) -> None:
        self._record("gnss", msg)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._record("cam_front_info", msg)
        self.camera_info = msg

    def _on_imu(self, msg: Imu) -> None:
        self._record("imu", msg)
        if self.record_imu:
            self.imu_yaw_rates.append(float(msg.angular_velocity.z))
            self.imu_accel_y.append(float(msg.linear_acceleration.y))

    def _on_tf(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self.tf_count += 1
            if t.child_frame_id == "cam_asym":
                tr = t.transform.translation
                self.tf_asym = (float(tr.x), float(tr.y), float(tr.z))


def spawn_rig(world: carla.World, hero: carla.Actor) -> list[carla.Actor]:
    """Spawn RIG on the hero with ros_name set and ROS publishing enabled."""
    lib = world.get_blueprint_library()
    sensors = []
    for entry in RIG:
        bp = lib.find(entry.type)
        bp.set_attribute("ros_name", entry.id)
        bp.set_attribute("role_name", entry.id)
        for k, v in entry.attributes.items():
            bp.set_attribute(k, v)
        tf = carla.Transform(carla.Location(x=entry.x, y=entry.y, z=entry.z))
        sensor = world.spawn_actor(bp, tf, attach_to=hero)
        sensor.enable_for_ros()
        sensors.append(sensor)
    return sensors


@dataclass
class DriveLog:
    """What the tick loop measured on the CARLA side."""

    counted_ticks: int = 0
    carla_yaws_deg: list[float] = field(default_factory=list)


def drive_and_collect(
    world: carla.World, hero: carla.Actor, node: ListenerNode, ticks: int
) -> DriveLog:
    """Tick `ticks` times: warm up, drive straight, then turn left; spin rclpy in between."""
    log = DriveLog()
    for i in range(ticks):
        if i < WARMUP_TICKS:
            hero.apply_control(carla.VehicleControl(throttle=0.0))
        elif i < ticks // 2:
            hero.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0))
        else:
            hero.apply_control(carla.VehicleControl(throttle=0.5, steer=-0.5))
        world.tick()
        if i == WARMUP_TICKS:
            for stats in node.stats.values():
                stats.clear()
            node.tf_count = 0
        if i >= WARMUP_TICKS:
            log.counted_ticks += 1
        if i == (3 * ticks) // 4:
            node.record_imu = True
        if node.record_imu:
            log.carla_yaws_deg.append(float(hero.get_transform().rotation.yaw))
        deadline = time.monotonic() + 0.04
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.005)
    rclpy.spin_once(node, timeout_sec=0.2)
    return log


def check_topics(node: ListenerNode, failures: list[str]) -> None:
    """Topics advertised, messages received, frame_id == ros_name."""
    names = {name for name, _ in node.get_topic_names_and_types()}
    carla_names = sorted(n for n in names if n.startswith("/carla"))
    for key, topic in EXPECTED_TOPICS.items():
        if topic not in names:
            failures.append(f"topic {topic} not advertised (got {carla_names})")
        stats = node.stats[key]
        ros_name = key.replace("_info", "")
        if stats.count == 0:
            failures.append(f"no messages on {topic}")
        elif stats.frame_ids != {ros_name}:
            failures.append(
                f"{topic}: frame_id {stats.frame_ids} != ros_name {ros_name!r}"
            )


def check_rates(node: ListenerNode, counted_ticks: int, failures: list[str]) -> None:
    """One message per tick on LiDAR / IMU / GNSS / camera_info; camera drop rate."""
    print(f"counted {counted_ticks} ticks")
    for key, topic in EXPECTED_TOPICS.items():
        stats = node.stats[key]
        rate = stats.count / max(1, counted_ticks)
        print(
            f"  {topic}: {stats.count} msgs, {len(stats.stamps)} distinct stamps"
            f" -> {rate:.3f}/tick"
        )
        if key in ONE_PER_TICK and abs(rate - 1.0) > 0.02:
            failures.append(f"{topic}: {rate:.3f} msg/tick, expected 1.00")
        if key == "cam_front":
            print(f"  camera drop rate: {100.0 * (1.0 - rate):.2f} %")
            if rate < 0.9:
                failures.append(f"{topic}: {rate:.3f} msg/tick, expected ~0.99")
    print(f"  /tf: {node.tf_count / max(1, counted_ticks):.2f} transforms/tick")


def report_camera_info(node: ListenerNode) -> None:
    """Report the (known broken) native focal length."""
    if node.camera_info is None:
        return
    fx = float(node.camera_info.k[0])
    verdict = (
        "BROKEN as documented"
        if abs(fx - EXPECTED_FX) > 1.0
        else "correct now: revisit docs/02 §3.1"
    )
    print(
        f"  camera_info fx = {fx:.3f} (pinhole would be {EXPECTED_FX:.1f}); {verdict}"
    )


def check_convention(node: ListenerNode, log: DriveLog, failures: list[str]) -> None:
    """Native topics are ROS-convention: /tf y sign and IMU yaw-rate sign."""
    if node.tf_asym is None:
        failures.append("no /tf for cam_asym (ros_publish_tf default changed?)")
    else:
        print(
            f"  /tf hero->cam_asym translation = {node.tf_asym} (spawned at CARLA y=+1.5)"
        )
        if abs(node.tf_asym[1] + 1.5) > 0.05:
            failures.append(f"/tf cam_asym y = {node.tf_asym[1]:.3f}, expected -1.5")
    if len(log.carla_yaws_deg) <= 10 or not node.imu_yaw_rates:
        failures.append("not enough IMU samples for the yaw-rate check")
        return
    deltas = [
        (b - a + 180.0) % 360.0 - 180.0
        for a, b in itertools.pairwise(log.carla_yaws_deg)
    ]
    carla_rate = math.radians(sum(deltas) / len(deltas)) / TICK_S
    imu_rate = sum(node.imu_yaw_rates) / len(node.imu_yaw_rates)
    acc_y = sum(node.imu_accel_y) / len(node.imu_accel_y)
    print(
        f"  CARLA yaw rate {carla_rate:+.3f} rad/s (cw+), IMU angular_velocity.z"
        f" {imu_rate:+.3f} rad/s, IMU accel.y {acc_y:+.2f}"
    )
    if not (carla_rate < -0.05 and imu_rate > 0.05):
        failures.append("left turn: expected CARLA yaw rate < 0 and IMU yaw rate > 0")
    elif abs(abs(carla_rate) - abs(imu_rate)) > 0.15 * abs(carla_rate):
        failures.append(
            f"IMU yaw rate magnitude {abs(imu_rate):.3f} != CARLA {abs(carla_rate):.3f}"
        )


def main() -> int:
    """Run the smoke test; return 1 on any failed assertion."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument(
        "--town", default=None, help="load this town first (default: keep current)"
    )
    parser.add_argument(
        "--ticks", type=int, default=400, help="ticks to run (default 400 = 20 s)"
    )
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(120.0)
    world = client.get_world()
    if args.town and args.town not in world.get_map().name:
        world = client.load_world(args.town)
    print(f"server {client.get_server_version()} map {world.get_map().name}")

    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = TICK_S
    world.apply_settings(settings)
    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)

    rclpy.init()
    node = ListenerNode()
    failures: list[str] = []
    actors: list[carla.Actor] = []
    try:
        hero_bp = world.get_blueprint_library().find("vehicle.lincoln.mkz_2020")
        hero_bp.set_attribute("role_name", HERO)
        hero_bp.set_attribute("ros_name", HERO)
        hero = world.spawn_actor(hero_bp, world.get_map().get_spawn_points()[0])
        actors.append(hero)
        actors.extend(spawn_rig(world, hero))
        world.tick()
        log = drive_and_collect(world, hero, node, args.ticks)
        check_topics(node, failures)
        check_rates(node, log.counted_ticks, failures)
        report_camera_info(node)
        check_convention(node, log, failures)
    finally:
        for actor in reversed(actors):
            with contextlib.suppress(RuntimeError):
                actor.destroy()
        settings.synchronous_mode = False
        world.apply_settings(settings)
        traffic_manager.set_synchronous_mode(False)
        node.destroy_node()
        rclpy.shutdown()

    if failures:
        print("FAILED:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print(
        "OK: native ROS 2 interface behaves as documented in docs/02_interfaces.md §1/§3.1"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
