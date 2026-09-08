"""Reference line and Cartesian <-> Frenet conversion (M0).

Mirrors ``nuway_common/frenet.h``: a polyline sampled along arc length ``s``
(meters, map frame) with per-sample heading (rad) and curvature (1/m). Frenet
coordinates are ``(s, d)`` with ``d`` positive to the left (ROS convention).
Cartesian points are ``p(s) + d * n(theta(s))`` with the heading interpolated on
the circle; the inverse solves the tangent condition with Newton steps, so the
round trip is exact while ``|d| * curvature < 1``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from nuway_ml.common.geometry import wrap_angle

Array = NDArray[np.float64]

NEWTON_ITERATIONS = 12
NEWTON_TOLERANCE = 1e-10


@dataclass(frozen=True, slots=True)
class FrenetPoint:
    """A point in Frenet coordinates: arc length ``s`` and signed offset ``d`` (+left)."""

    s: float = 0.0
    d: float = 0.0


@dataclass(frozen=True, slots=True)
class CartesianPoint:
    """A Cartesian point with the line heading (rad) at its projection."""

    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0


def menger_curvature(first: Array, mid: Array, last: Array) -> float:
    """Signed Menger curvature of three points: positive for a left turn."""
    first_mid = np.asarray(mid, dtype=np.float64) - first
    mid_last = np.asarray(last, dtype=np.float64) - mid
    first_last = np.asarray(last, dtype=np.float64) - first
    cross = first_mid[0] * mid_last[1] - first_mid[1] * mid_last[0]
    denom = (
        np.linalg.norm(first_mid)
        * np.linalg.norm(mid_last)
        * np.linalg.norm(first_last)
    )
    if denom < 1e-12:
        return 0.0
    return float(2.0 * cross / denom)


class ReferenceLine:
    """Polyline reference line with per-sample s, heading and curvature."""

    def __init__(
        self, points: Array, s: Array, heading: Array, curvature: Array
    ) -> None:
        """Build from explicit samples; prefer :meth:`from_points`."""
        self._points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        self._s = np.asarray(s, dtype=np.float64)
        self._heading = np.asarray(heading, dtype=np.float64)
        self._curvature = np.asarray(curvature, dtype=np.float64)

    @classmethod
    def from_points(cls, points: Array) -> ReferenceLine:
        """Build from ``(N, 2)`` sample points.

        ``s`` is the cumulative chord length, heading the central-difference
        direction (one-sided at the ends), curvature the signed Menger curvature
        of consecutive triples (zero at the ends).
        """
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        n = pts.shape[0]
        s = np.zeros(n)
        heading = np.zeros(n)
        curvature = np.zeros(n)
        if n >= 2:
            seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
            s[1:] = np.cumsum(seg)
        for i in range(n):
            prev = max(0, i - 1)
            nxt = min(n - 1, i + 1)
            if prev == nxt:
                continue
            delta = pts[nxt] - pts[prev]
            heading[i] = math.atan2(delta[1], delta[0])
        for i in range(1, n - 1):
            curvature[i] = menger_curvature(pts[i - 1], pts[i], pts[i + 1])
        return cls(pts, s, heading, curvature)

    @classmethod
    def from_samples(
        cls, points: Array, s: Array, heading: Array, curvature: Array
    ) -> ReferenceLine:
        """Build from explicit samples (e.g. a ReferenceLine message)."""
        return cls(points, s, heading, curvature)

    @property
    def size(self) -> int:
        """Number of samples."""
        return int(self._points.shape[0])

    @property
    def empty(self) -> bool:
        """True if the line has no samples."""
        return self.size == 0

    @property
    def length(self) -> float:
        """Total arc length in meters."""
        return float(self._s[-1]) if self.size else 0.0

    @property
    def points(self) -> Array:
        """Sample points ``(N, 2)``."""
        return self._points

    @property
    def s(self) -> Array:
        """Arc length per sample ``(N,)``."""
        return self._s

    @property
    def heading(self) -> Array:
        """Heading per sample ``(N,)`` in radians."""
        return self._heading

    @property
    def curvature(self) -> Array:
        """Curvature per sample ``(N,)`` in 1/m."""
        return self._curvature

    def segment_index(self, s: float) -> int:
        """Index of the segment ``[i, i+1]`` containing s (clamped to the line)."""
        if self.size < 2:
            return 0
        idx = int(np.searchsorted(self._s, s, side="right")) - 1
        return max(0, min(idx, self.size - 2))

    def _segment_fraction(self, idx: int, s: float) -> float:
        seg_len = float(self._s[idx + 1] - self._s[idx])
        if seg_len <= 0.0:
            return 0.0
        return max(0.0, min(1.0, (s - float(self._s[idx])) / seg_len))

    def _interpolate(self, s: float) -> Array:
        idx = self.segment_index(s)
        alpha = self._segment_fraction(idx, s)
        return np.asarray(
            self._points[idx] + alpha * (self._points[idx + 1] - self._points[idx]),
            dtype=np.float64,
        )

    def point_at(self, s: float) -> CartesianPoint:
        """Line point at arc length s (linear interpolation, clamped)."""
        return self.to_cartesian(FrenetPoint(s, 0.0))

    def heading_at(self, s: float) -> float:
        """Heading of the line at arc length s, interpolated on the circle."""
        if self.size < 2:
            return float(self._heading[0]) if self.size else 0.0
        idx = self.segment_index(s)
        alpha = self._segment_fraction(idx, s)
        h0 = float(self._heading[idx])
        h1 = float(self._heading[idx + 1])
        return wrap_angle(h0 + alpha * wrap_angle(h1 - h0))

    def curvature_at(self, s: float) -> float:
        """Curvature of the line at arc length s (linear interpolation)."""
        if self.size < 2:
            return float(self._curvature[0]) if self.size else 0.0
        idx = self.segment_index(s)
        alpha = self._segment_fraction(idx, s)
        c0 = float(self._curvature[idx])
        c1 = float(self._curvature[idx + 1])
        return c0 + alpha * (c1 - c0)

    def to_cartesian(self, frenet: FrenetPoint) -> CartesianPoint:
        """Frenet -> Cartesian: ``p(s) + d * n(s)``, s clamped to ``[0, length]``."""
        if self.empty:
            return CartesianPoint()
        if self.size == 1:
            return CartesianPoint(
                float(self._points[0, 0]), float(self._points[0, 1]) + frenet.d, 0.0
            )
        base = self._interpolate(frenet.s)
        heading = self.heading_at(frenet.s)
        out = base + frenet.d * np.array([-math.sin(heading), math.cos(heading)])
        return CartesianPoint(float(out[0]), float(out[1]), heading)

    def _nearest_segment_s(self, query: Array, max_dist: float) -> float | None:
        best_dist2 = max_dist * max_dist
        best: float | None = None
        p0 = self._points[:-1]
        seg = self._points[1:] - p0
        seg_len2 = np.sum(seg * seg, axis=1)
        alpha = np.zeros_like(seg_len2)
        nonzero = seg_len2 > 0.0
        alpha[nonzero] = (
            np.sum((query - p0[nonzero]) * seg[nonzero], axis=1) / (seg_len2[nonzero])
        )
        alpha = np.clip(alpha, 0.0, 1.0)
        proj = p0 + alpha[:, None] * seg
        dist2 = np.sum((query - proj) ** 2, axis=1)
        i = int(np.argmin(dist2))
        if dist2[i] < best_dist2:
            best = float(self._s[i] + alpha[i] * math.sqrt(seg_len2[i]))
        return best

    def to_frenet(
        self, x: float, y: float, max_dist: float = 1e9
    ) -> FrenetPoint | None:
        """Cartesian -> Frenet by nearest-segment projection refined with Newton steps.

        Returns None if the line has fewer than two points or the point is
        farther than ``max_dist`` from every segment.
        """
        if self.size < 2:
            return None
        query = np.array([x, y], dtype=np.float64)
        coarse = self._nearest_segment_s(query, max_dist)
        if coarse is None:
            return None
        s = coarse
        for _ in range(NEWTON_ITERATIONS):
            diff = query - self._interpolate(s)
            heading = self.heading_at(s)
            tangent = np.array([math.cos(heading), math.sin(heading)])
            normal = np.array([-tangent[1], tangent[0]])
            along = float(diff @ tangent)
            lateral = float(diff @ normal)
            slope = 1.0 - lateral * self.curvature_at(s)
            step = along / slope if abs(slope) > 0.1 else along
            nxt = max(0.0, min(self.length, s + step))
            converged = abs(nxt - s) < NEWTON_TOLERANCE
            s = nxt
            if converged:
                break
        diff = query - self._interpolate(s)
        heading = self.heading_at(s)
        normal = np.array([-math.sin(heading), math.cos(heading)])
        return FrenetPoint(s, float(diff @ normal))
