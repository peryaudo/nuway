"""rclpy QoSProfile objects built from the plain-data table in nuway_ml (M0).

``nuway_ml.common.qos.QOS`` holds the profiles of ``docs/02_interfaces.md``
§3.11 without importing rclpy; this is the rclpy side of it.
"""

from __future__ import annotations

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from nuway_ml.common.qos import QOS


def qos(name: str) -> QoSProfile:
    """Return the rclpy profile for one of the named profiles (sensor, stream, ...)."""
    spec = QOS[name]
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=spec.depth,
        reliability=(
            QoSReliabilityPolicy.RELIABLE
            if spec.reliability == "reliable"
            else QoSReliabilityPolicy.BEST_EFFORT
        ),
        durability=(
            QoSDurabilityPolicy.TRANSIENT_LOCAL
            if spec.durability == "transient_local"
            else QoSDurabilityPolicy.VOLATILE
        ),
    )


# /clock: reliable so it matches both reliable and best-effort time sources
# (a reliable publisher is compatible with every subscription reliability).
CLOCK_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)
