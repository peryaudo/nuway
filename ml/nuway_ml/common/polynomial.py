"""Quintic and quartic polynomials for lattice sampling in Frenet coordinates (M1).

Mirrors ``nuway_common/polynomial.h``. A quintic has six coefficients, exactly
enough to meet a start state ``(x, x', x'')`` and an end state over a horizon:
it is the jerk-optimal connection of two states (the minimiser of the
integral of squared jerk, Werling 2010 section III), used for the lateral
path ``d(s)`` and for stopping profiles ``s(t)``. With the end position free
(velocity keeping) one condition drops and a quartic is the jerk-optimal
answer. Both are evaluated by Horner's rule with derivatives up to the third.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Polynomial5:
    """``x(t) = sum c_i t^i`` for ``i`` in ``[0, 5]``, ``t`` from the polynomial's own start."""

    c: tuple[float, float, float, float, float, float] = (0.0,) * 6

    @classmethod
    def quintic(  # noqa: PLR0917  -- mirrors Polynomial5::Quintic's positional boundary conditions (03 §9.2)
        cls,
        x0: float,
        dx0: float,
        ddx0: float,
        x1: float,
        dx1: float,
        ddx1: float,
        length: float,
    ) -> Polynomial5:
        """Build the quintic through ``(x0, dx0, ddx0)`` at 0 and ``(x1, dx1, ddx1)`` at ``length``."""
        if length <= 0.0:
            return cls((x0, 0.0, 0.0, 0.0, 0.0, 0.0))
        c1 = dx0
        c2 = ddx0 / 2.0
        t = length
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        r0 = x1 - x0 - c1 * t - c2 * t2
        r1 = dx1 - c1 - 2.0 * c2 * t
        r2 = ddx1 - 2.0 * c2
        c3 = (10.0 * r0 - 4.0 * r1 * t + 0.5 * r2 * t2) / t3
        c4 = (-15.0 * r0 + 7.0 * r1 * t - r2 * t2) / t4
        c5 = (6.0 * r0 - 3.0 * r1 * t + 0.5 * r2 * t2) / t5
        return cls((x0, c1, c2, c3, c4, c5))

    @classmethod
    def quartic(  # noqa: PLR0917  -- mirrors Polynomial5::Quartic's positional boundary conditions (03 §9.2)
        cls, x0: float, dx0: float, ddx0: float, dx1: float, ddx1: float, length: float
    ) -> Polynomial5:
        """Build the quartic through ``(x0, dx0, ddx0)`` at 0 reaching ``(dx1, ddx1)`` at ``length``."""
        if length <= 0.0:
            return cls((x0, 0.0, 0.0, 0.0, 0.0, 0.0))
        c1 = dx0
        c2 = ddx0 / 2.0
        t = length
        t2 = t * t
        t3 = t2 * t
        r1 = dx1 - c1 - 2.0 * c2 * t
        r2 = ddx1 - 2.0 * c2
        c3 = (r1 - r2 * t / 3.0) / t2
        c4 = (r2 * t - 2.0 * r1) / (4.0 * t3)
        return cls((x0, c1, c2, c3, c4, 0.0))

    def eval(self, t: float) -> float:
        """Return ``x(t)``."""
        c = self.c
        return c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * (c[4] + t * c[5]))))

    def eval_first(self, t: float) -> float:
        """Return the first derivative at ``t``."""
        c = self.c
        return c[1] + t * (
            2.0 * c[2] + t * (3.0 * c[3] + t * (4.0 * c[4] + t * 5.0 * c[5]))
        )

    def eval_second(self, t: float) -> float:
        """Return the second derivative at ``t``."""
        c = self.c
        return 2.0 * c[2] + t * (6.0 * c[3] + t * (12.0 * c[4] + t * 20.0 * c[5]))

    def eval_third(self, t: float) -> float:
        """Return the third derivative at ``t``."""
        c = self.c
        return 6.0 * c[3] + t * (24.0 * c[4] + t * 60.0 * c[5])
