"""``/nuway/viz/chase_cam`` -> ``<route>/chase/{tick:06d}.jpg`` (docs/02 §8.3).

The chase camera is a witness for humans and post-mortems, never an input:
its only subscriber is this writer, which the route runner starts for the
duration of a run when ``eval.chase_cam`` is on. The JPEG bytes are written
as they arrive; nothing is decoded.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from nuway_ml.common.tick import tick_index

if TYPE_CHECKING:
    from pathlib import Path

    from rclpy.node import Node
    from rclpy.subscription import Subscription


def chase_frame_path(route_dir: Path, tick: int) -> Path:
    """``<route>/chase/{tick:06d}.jpg``."""
    return route_dir / "chase" / f"{tick:06d}.jpg"


def write_chase_frame(route_dir: Path, tick: int, data: bytes) -> Path:
    """Write one JPEG frame; returns its path."""
    path = chase_frame_path(route_dir, tick)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    return path


class ChaseWriter:
    """Subscribes for one run and writes every frame stamped inside the episode."""

    def __init__(self, node: Node, route_dir: Path, first_tick: int = 0) -> None:
        """Create the subscription on ``node``; frames before ``first_tick`` are dropped."""
        # The ROS-side imports stay inside the class so the pure helpers above
        # (and their test) load without a sourced ROS environment.
        from sensor_msgs.msg import CompressedImage  # noqa: PLC0415  # see above

        from nuway_ml.common import frames  # noqa: PLC0415  # see above
        from nuway_rclpy.ros_qos import qos  # noqa: PLC0415  # see above

        self._route_dir = route_dir
        self._first_tick = first_tick
        self.frames = 0
        self._sub: Subscription | None = node.create_subscription(
            CompressedImage, frames.TOPIC_VIZ_CHASE_CAM, self._on_image, qos("viz")
        )
        self._node = node

    def _on_image(self, msg: object) -> None:
        stamp = getattr(msg, "header").stamp  # noqa: B009  # CompressedImage typed as object for the pure helpers
        tick = tick_index(stamp)
        if tick < self._first_tick:
            return
        write_chase_frame(self._route_dir, tick, bytes(getattr(msg, "data")))  # noqa: B009
        self.frames += 1

    def stop(self) -> int:
        """Destroy the subscription; returns the number of frames written."""
        if self._sub is not None:
            self._node.destroy_subscription(self._sub)
            self._sub = None
        return self.frames
