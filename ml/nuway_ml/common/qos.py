"""The QoS table of ``docs/02_interfaces.md`` §3.11 as plain data (M0).

No ``rclpy`` import: node packages build ``rclpy.qos.QoSProfile`` objects from
these records, which keeps ``nuway_ml`` importable in ROS-free environments.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


@dataclass(frozen=True, slots=True)
class QosProfile:
    """One named QoS profile: reliability, durability and keep_last depth."""

    reliability: Literal["reliable", "best_effort"]
    durability: Literal["volatile", "transient_local"]
    depth: int


QOS: dict[str, QosProfile] = {
    "sensor": QosProfile("best_effort", "volatile", 1),
    "stream": QosProfile("reliable", "volatile", 2),
    "latched": QosProfile("reliable", "transient_local", 1),
    "event": QosProfile("reliable", "transient_local", 10),
    "diag": QosProfile("reliable", "volatile", 10),
    "viz": QosProfile("best_effort", "volatile", 1),
}
