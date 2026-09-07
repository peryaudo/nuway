"""Shared scaffolding of the GT cheat twins (M0 §2.9): tick barrier, reset, timeout, diag.

A twin acts on planning ticks only (even ``k``, docs/02 §2), waits for every
per-tick input stamped ``k`` (``TickBarrier``), publishes exactly once per
tick it acts on, and emits its no-input output when the pose is invalid.
``TickTimeout(k)`` degrades the inputs still missing for ``k`` until the next
``ResetEvent``; messages stamped before the reset belong to the previous
episode and are dropped (docs/02 §7).
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from collections.abc import Callable
from typing import Any

from builtin_interfaces.msg import Time
from nuway_msgs.msg import EgoState, NodeDiag, ResetEvent, TickTimeout
from rclpy.node import Node

from nuway_ml.common.frames import (
    TOPIC_DIAG_PREFIX,
    TOPIC_POSE,
    TOPIC_RESET_EVENT,
    TOPIC_TICK_TIMEOUT,
)
from nuway_ml.common.tick import TickBarrier, is_planning_tick, tick_index, tick_stamp
from nuway_rclpy.ros_qos import qos

POSE_INPUT = "pose"
# Messages older than this many ticks behind the newest are dropped from the
# per-tick buffers (a tick that never completed is never published).
BUFFER_TICKS = 4


class GtTwinNode(Node, ABC):  # type: ignore[misc]  # rclpy.Node has no stubs (03 §7.3)
    """Base of gt_perception_node and gt_traffic_light_node."""

    def __init__(self, name: str, inputs: dict[str, tuple[type, str]]) -> None:
        """Wire the pose plus ``inputs`` (name -> (msg type, topic)), reset and timeout."""
        super().__init__(name)
        self._node_name = name
        self._barrier = TickBarrier([POSE_INPUT, *inputs])
        self._buffers: dict[str, dict[int, Any]] = {
            key: {} for key in (POSE_INPUT, *inputs)
        }
        self._episode_start_k = 0
        self._last_published_k = -1
        self._pub_diag = self.create_publisher(
            NodeDiag, TOPIC_DIAG_PREFIX + name, qos("diag")
        )
        self._sub_pose = self.create_subscription(
            EgoState, TOPIC_POSE, lambda m: self._on_input(POSE_INPUT, m), qos("stream")
        )
        self._subs = [
            self.create_subscription(
                msg_type, topic, self._input_callback(key), qos("stream")
            )
            for key, (msg_type, topic) in inputs.items()
        ]
        self._sub_reset = self.create_subscription(
            ResetEvent, TOPIC_RESET_EVENT, self._on_reset_event, qos("event")
        )
        self._sub_tick_timeout = self.create_subscription(
            TickTimeout, TOPIC_TICK_TIMEOUT, self._on_tick_timeout, qos("event")
        )

    def _input_callback(self, key: str) -> Callable[[object], None]:
        return lambda msg: self._on_input(key, msg)

    # ------------------------------------------------------------ overrides
    @abstractmethod
    def _process(self, stamp: Time, pose: EgoState, inputs: dict[str, Any]) -> str:
        """Publish the outputs for tick ``k``; ``inputs`` lacks degraded/missing sources.

        Returns the diag message (empty when nominal).
        """

    @abstractmethod
    def _publish_no_input(self, stamp: Time) -> None:
        """Publish the no-input outputs of docs/02 §2 stamped ``stamp``."""

    def _on_reset(self) -> None:
        """Drop subclass cross-tick state (called after the barrier reset)."""

    # ------------------------------------------------------------- callbacks
    def _on_reset_event(self, msg: ResetEvent) -> None:
        self._episode_start_k = tick_index(msg.header.stamp)
        self._barrier.reset()
        for buffer in self._buffers.values():
            buffer.clear()
        self._last_published_k = self._episode_start_k - 1
        self._on_reset()
        self.get_logger().info(f"reset: episode {msg.episode_id}")

    def _on_input(self, key: str, msg: object) -> None:
        k = tick_index(msg.header.stamp)  # type: ignore[attr-defined]  # every input is a stamped ROS message
        if k < self._episode_start_k or not is_planning_tick(k):
            return
        self._barrier.arrive(key, k)
        buffer = self._buffers[key]
        buffer[k] = msg
        for old in [old_k for old_k in buffer if old_k < k - BUFFER_TICKS]:
            del buffer[old]
        if self._barrier.is_complete(k):
            self._run(k)

    def _on_tick_timeout(self, msg: TickTimeout) -> None:
        k = tick_index(msg.header.stamp)
        if k < self._episode_start_k or not is_planning_tick(k):
            return
        missing = self._barrier.missing(k)
        for name in missing:
            self._barrier.degrade(name)
        if missing:
            self.get_logger().warning(
                f"tick {k} timed out; degraded {missing} until the next reset"
            )
        self._run(k)

    # -------------------------------------------------------------- per tick
    def _run(self, k: int) -> None:
        if k <= self._last_published_k:
            return
        self._last_published_k = k
        start = time.perf_counter()
        pose = self._buffers[POSE_INPUT].get(k)
        status = NodeDiag.STATUS_OK
        if pose is None or not pose.valid:
            stamp = pose.header.stamp if pose is not None else _tick_stamp(k)
            self._publish_no_input(stamp)
            message = "pose invalid" if pose is not None else "no pose (degraded)"
            if pose is None:
                status = NodeDiag.STATUS_WARN
        else:
            inputs = {
                key: buffer[k]
                for key, buffer in self._buffers.items()
                if key != POSE_INPUT and k in buffer
            }
            message = self._process(pose.header.stamp, pose, inputs)
            stamp = pose.header.stamp
            if any(self._barrier.is_degraded(name) for name in self._barrier.inputs):
                status = NodeDiag.STATUS_WARN
                degraded = [
                    n for n in self._barrier.inputs if self._barrier.is_degraded(n)
                ]
                message = f"degraded {degraded}; {message}".rstrip("; ")
        diag = NodeDiag()
        diag.header.stamp = stamp
        diag.node = self._node_name
        diag.cycle_ms = float((time.perf_counter() - start) * 1e3)
        diag.input_age_ms = 0.0
        diag.status = int(status)
        diag.message = message
        self._pub_diag.publish(diag)


def _tick_stamp(k: int) -> Time:
    sec, nanosec = tick_stamp(k)
    return Time(sec=sec, nanosec=nanosec)
