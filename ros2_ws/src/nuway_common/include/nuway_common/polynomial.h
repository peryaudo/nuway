// Quintic and quartic polynomials for lattice sampling in Frenet
// coordinates (M1 §3.3). Mirrored by nuway_ml/common/polynomial.py
// (parity-tested). Introduced in M1.
//
// Why these two. A quintic x(t) = c0 + c1 t + ... + c5 t^5 has six
// coefficients, exactly enough to meet a start state (x, x', x'') and an end
// state (x, x', x'') over a horizon T: it is the jerk-optimal connection of
// two states (the minimiser of the integral of x'''^2 is a quintic, Werling
// 2010 §III), which is why lattice planners use it for the lateral path
// d(s) and for stopping profiles s(t) whose final position is fixed. When
// the end position is free (velocity keeping: "reach speed v_T with zero
// acceleration, wherever that puts us") one condition drops and a quartic
// (five coefficients) is the jerk-optimal answer. Both are evaluated by
// Horner's rule with derivatives up to the third.
#ifndef NUWAY_COMMON_POLYNOMIAL_H_
#define NUWAY_COMMON_POLYNOMIAL_H_

#include <array>

namespace nuway_common {

// x(t) = sum c_i t^i for i in [0, 5], with t counted from the polynomial's
// own start (the caller shifts). Also represents a quartic (c5 = 0).
class Polynomial5 {
 public:
  Polynomial5() = default;
  explicit Polynomial5(const std::array<double, 6>& coefficients)
      : c_(coefficients) {}

  // Quintic through (x0, dx0, ddx0) at 0 and (x1, dx1, ddx1) at `length`
  // (the horizon: T seconds for s(t), a distance for d(s)). A non-positive
  // length yields the constant x0.
  static Polynomial5 Quintic(double x0, double dx0, double ddx0, double x1,
                             double dx1, double ddx1, double length);

  // Quartic through (x0, dx0, ddx0) at 0 reaching (dx1, ddx1) at `length`
  // with the end position free (velocity keeping).
  static Polynomial5 Quartic(double x0, double dx0, double ddx0, double dx1,
                             double ddx1, double length);

  double Eval(double t) const;        // x(t)
  double EvalFirst(double t) const;   // x'(t)
  double EvalSecond(double t) const;  // x''(t)
  double EvalThird(double t) const;   // x'''(t)
  const std::array<double, 6>& coefficients() const { return c_; }

 private:
  std::array<double, 6> c_{};
};

// The six boundary conditions give a linear system for c3..c5 once c0..c2
// are read off the start state; these are its closed-form solutions.
inline Polynomial5 Polynomial5::Quintic(double x0, double dx0, double ddx0,
                                        double x1, double dx1, double ddx1,
                                        double length) {
  std::array<double, 6> c{};
  c[0] = x0;
  if (length <= 0.0) {
    return Polynomial5(c);
  }
  c[1] = dx0;
  c[2] = ddx0 / 2.0;
  const double t = length;
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;
  // Residuals of the end state after the start-state terms.
  const double r0 = x1 - c[0] - (c[1] * t) - (c[2] * t2);
  const double r1 = dx1 - c[1] - (2.0 * c[2] * t);
  const double r2 = ddx1 - (2.0 * c[2]);
  c[3] = ((10.0 * r0) - (4.0 * r1 * t) + (0.5 * r2 * t2)) / t3;
  c[4] = ((-15.0 * r0) + (7.0 * r1 * t) - (r2 * t2)) / t4;
  c[5] = ((6.0 * r0) - (3.0 * r1 * t) + (0.5 * r2 * t2)) / t5;
  return Polynomial5(c);
}

// Five boundary conditions (no end position): c3, c4 from the end velocity
// and acceleration.
inline Polynomial5 Polynomial5::Quartic(double x0, double dx0, double ddx0,
                                        double dx1, double ddx1,
                                        double length) {
  std::array<double, 6> c{};
  c[0] = x0;
  if (length <= 0.0) {
    return Polynomial5(c);
  }
  c[1] = dx0;
  c[2] = ddx0 / 2.0;
  const double t = length;
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double r1 = dx1 - c[1] - (2.0 * c[2] * t);
  const double r2 = ddx1 - (2.0 * c[2]);
  c[3] = (r1 - (r2 * t / 3.0)) / t2;
  c[4] = ((r2 * t) - (2.0 * r1)) / (4.0 * t3);
  c[5] = 0.0;
  return Polynomial5(c);
}

inline double Polynomial5::Eval(double t) const {
  return c_[0] +
         (t * (c_[1] +
               (t * (c_[2] + (t * (c_[3] + (t * (c_[4] + (t * c_[5])))))))));
}

inline double Polynomial5::EvalFirst(double t) const {
  return c_[1] +
         (t *
          ((2.0 * c_[2]) +
           (t * ((3.0 * c_[3]) + (t * ((4.0 * c_[4]) + (t * 5.0 * c_[5])))))));
}

inline double Polynomial5::EvalSecond(double t) const {
  return (2.0 * c_[2]) +
         (t * ((6.0 * c_[3]) + (t * ((12.0 * c_[4]) + (t * 20.0 * c_[5])))));
}

inline double Polynomial5::EvalThird(double t) const {
  return (6.0 * c_[3]) + (t * ((24.0 * c_[4]) + (t * 60.0 * c_[5])));
}

}  // namespace nuway_common

#endif  // NUWAY_COMMON_POLYNOMIAL_H_
