"""Tick index and phase of ``docs/02_interfaces.md`` §2 (M0).

Mirrors ``nuway_common/tick.hpp``: every stamp is nominally ``k * 0.05`` s,
comparisons go through :func:`tick_index`, 10 Hz nodes act on even ticks, and
:class:`TickBarrier` is the current-tick barrier. No ``rclpy`` import: stamps
are duck typed (``sec`` / ``nanosec`` attributes) or plain seconds.
"""

from __future__ import annotations

from typing import Protocol

TICK_DT_S = 0.05
NANOS_PER_SECOND = 1_000_000_000
_TICK_NANOS = 50_000_000


class StampLike(Protocol):
    """A ``builtin_interfaces/Time``: integer ``sec`` and ``nanosec``."""

    @property
    def sec(self) -> int: ...  # noqa: D102  -- protocol member
    @property
    def nanosec(self) -> int: ...  # noqa: D102  -- protocol member


def tick_index(stamp: float | StampLike) -> int:
    """Tick index ``k = round(stamp / 0.05)`` of a stamp in seconds or a ROS Time."""
    if isinstance(stamp, int | float):
        seconds = float(stamp)
    else:
        seconds = float(stamp.sec) + float(stamp.nanosec) / 1e9
    return round(seconds / TICK_DT_S)


def tick_time_s(k: int) -> float:
    """Stamp in seconds of tick k."""
    return float(k) * TICK_DT_S


def tick_stamp(k: int) -> tuple[int, int]:
    """``(sec, nanosec)`` of tick k (exact: 50 ms multiples are integer nanoseconds)."""
    nanos = k * _TICK_NANOS
    return nanos // NANOS_PER_SECOND, nanos % NANOS_PER_SECOND


def is_planning_tick(k: int) -> bool:
    """10 Hz nodes act on even ticks only; odd ticks are control-only."""
    return k % 2 == 0


class TickBarrier:
    """Current-tick barrier over a fixed set of named per-tick inputs.

    :meth:`arrive` records each input's latest tick; :meth:`is_complete` is true
    when every non-degraded input has arrived for the tick. :meth:`degrade`
    removes an input from the wait set until :meth:`reset` (docs/02 §2
    Degradation); ``reset`` also drops every recorded arrival (a ResetEvent).
    """

    def __init__(self, inputs: list[str]) -> None:
        """Create a barrier over ``inputs``."""
        self._inputs = list(inputs)
        self._latest: dict[str, int] = {}
        self._degraded: dict[str, bool] = {}
        self.reset()

    @property
    def inputs(self) -> list[str]:
        """The input names, in declaration order."""
        return list(self._inputs)

    def arrive(self, name: str, k: int) -> None:
        """Record that ``name`` has a message stamped tick k."""
        self._latest[name] = k

    def _has_arrived(self, name: str, k: int) -> bool:
        return self._latest.get(name, -1) >= k

    def is_complete(self, k: int) -> bool:
        """Return True when every non-degraded input has arrived for tick k."""
        return all(
            self._degraded[name] or self._has_arrived(name, k) for name in self._inputs
        )

    def missing(self, k: int) -> list[str]:
        """Names of the non-degraded inputs that have not arrived for tick k."""
        return [
            name
            for name in self._inputs
            if not self._degraded[name] and not self._has_arrived(name, k)
        ]

    def degrade(self, name: str) -> None:
        """Stop waiting for ``name`` until the next reset."""
        self._degraded[name] = True

    def is_degraded(self, name: str) -> bool:
        """Return True if ``name`` is currently degraded."""
        return self._degraded[name]

    def reset(self) -> None:
        """Drop every arrival and every degraded flag."""
        self._latest.clear()
        self._degraded = dict.fromkeys(self._inputs, False)
