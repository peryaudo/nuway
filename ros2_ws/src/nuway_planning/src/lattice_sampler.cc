#include "nuway_planning/lattice_sampler.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nuway_common/agents.h>
#include <nuway_common/geometry.h>
#include <nuway_common/polynomial.h>
#include <nuway_common/trajectory.h>

namespace nuway_planning {
namespace {

using nuway_common::CartesianState;
using nuway_common::FrenetState;
using nuway_common::Polynomial5;

constexpr double kEps = 1e-6;

// A lateral candidate d(s): the quintic between s0 and s_f, held at d_f
// beyond s_f; or (`fitted` false) the current d held to the line end.
struct PathSpec {
  double s0 = 0.0;
  double s_f = 0.0;
  double d0 = 0.0;
  double d_f = 0.0;
  bool fitted = true;
  Polynomial5 poly;

  // (d, d', d'') at arc length s.
  void At(double s, double* d, double* d_prime, double* d_dprime) const {
    if (!fitted) {
      *d = d0;
      *d_prime = 0.0;
      *d_dprime = 0.0;
      return;
    }
    if (s >= s_f) {
      *d = d_f;
      *d_prime = 0.0;
      *d_dprime = 0.0;
      return;
    }
    const double u = std::max(0.0, s - s0);
    *d = poly.Eval(u);
    *d_prime = poly.EvalFirst(u);
    *d_dprime = poly.EvalSecond(u);
  }
};

// A longitudinal candidate s(t): a polynomial over [0, T] extended at its
// terminal state (at rest when `stops`, else at constant speed), or the
// constant-deceleration stop at a_min.
struct SpeedSpec {
  SpeedKind kind = SpeedKind::kKeep;
  double v_target = 0.0;
  double horizon = 8.0;
  bool stops = false;
  bool constant_decel = false;
  double s0 = 0.0;
  double v0 = 0.0;
  double decel = 0.0;  // positive magnitude, constant_decel only
  Polynomial5 poly;

  // (s, s_dot, s_ddot) at time t.
  void At(double t, double* s, double* s_dot, double* s_ddot) const {
    if (constant_decel) {
      const double t_stop = decel > kEps ? v0 / decel : 0.0;
      if (t >= t_stop) {
        *s = s0 + (0.5 * v0 * t_stop);
        *s_dot = 0.0;
        *s_ddot = 0.0;
      } else {
        *s = s0 + (v0 * t) - (0.5 * decel * t * t);
        *s_dot = v0 - (decel * t);
        *s_ddot = -decel;
      }
      return;
    }
    if (t <= horizon) {
      *s = poly.Eval(t);
      *s_dot = poly.EvalFirst(t);
      *s_ddot = poly.EvalSecond(t);
      return;
    }
    const double s_end = poly.Eval(horizon);
    const double v_end = stops ? 0.0 : poly.EvalFirst(horizon);
    *s = s_end + (v_end * (t - horizon));
    *s_dot = v_end;
    *s_ddot = 0.0;
  }
};

// The velocity-keeping quartic (end position free, Werling 2010 §V.B).
SpeedSpec KeepSpeed(const FrenetState& ego, double v_target, double horizon) {
  SpeedSpec spec;
  spec.kind = SpeedKind::kKeep;
  spec.v_target = v_target;
  spec.horizon = horizon;
  spec.poly = Polynomial5::Quartic(ego.s, ego.s_dot, ego.s_ddot, v_target, 0.0,
                                   horizon);
  return spec;
}

// A quintic to rest at s_stop over `horizon` (Werling 2010 §V.A stopping).
SpeedSpec StopAt(const FrenetState& ego, double s_stop, double horizon,
                 SpeedKind kind) {
  SpeedSpec spec;
  spec.kind = kind;
  spec.v_target = 0.0;
  spec.horizon = horizon;
  spec.stops = true;
  spec.poly = Polynomial5::Quintic(ego.s, ego.s_dot, ego.s_ddot, s_stop, 0.0,
                                   0.0, horizon);
  return spec;
}

// Constant deceleration `decel` (> 0) to rest, then holding.
SpeedSpec ConstantDecel(const FrenetState& ego, double decel, SpeedKind kind) {
  SpeedSpec spec;
  spec.kind = kind;
  spec.constant_decel = true;
  spec.stops = true;
  spec.s0 = ego.s;
  spec.v0 = std::max(0.0, ego.s_dot);
  spec.decel = decel;
  spec.horizon = decel > kEps ? spec.v0 / decel : 0.0;
  return spec;
}

// The lead's rear bumper on the line and its speed along it, for the
// gap-keeping candidate; nullopt without a projectable lead.
struct LeadOnLine {
  double s_rear = 0.0;
  double v = 0.0;
};

std::optional<LeadOnLine> ProjectLead(const SceneInput& in,
                                      const FrenetState& ego,
                                      std::uint32_t lead_id) {
  for (const nuway_common::AgentState& agent : in.agents) {
    if (agent.id != lead_id) {
      continue;
    }
    const std::optional<nuway_common::FrenetPoint> f =
        in.route->Project(agent.pose.x, agent.pose.y, 10.0, ego.s, 10.0, 200.0);
    if (!f.has_value()) {
      return std::nullopt;
    }
    const double heading = in.route->line().HeadingAt(f->s);
    const double v =
        (agent.vx_mps * std::cos(heading)) + (agent.vy_mps * std::sin(heading));
    return LeadOnLine{f->s - (0.5 * agent.length_m), std::max(0.0, v)};
  }
  return std::nullopt;
}

}  // namespace

LatticeLimits LatticeLimits::FromVehicleModel(
    const nuway_control::VehicleModel& m) {
  LatticeLimits limits;
  limits.kappa_phys = m.PhysicalCurvatureMax();
  limits.a_min_mps2 = m.limits.a_min_mps2;
  limits.a_max_mps2 = m.limits.a_max_mps2;
  limits.width_m = m.width_m;
  limits.wheelbase_m = m.wheelbase_m;
  // Rear axle to the front bumper: half the length past the actor origin,
  // which sits rear_axle_offset_x behind... i.e. ahead of the axle.
  limits.ego_front_m = (0.5 * m.length_m) - m.rear_axle_offset_x_m;
  return limits;
}

std::optional<FrenetState> EgoFrenetState(const EgoObs& ego,
                                          const RouteLine& route,
                                          const LatticeOptions& options,
                                          double wheelbase_m,
                                          std::optional<double> s_hint) {
  CartesianState c;
  c.x = ego.pose.x;
  c.y = ego.pose.y;
  c.yaw = ego.pose.yaw;
  c.v = ego.vx_mps;
  c.a = ego.ax_mps2;
  c.kappa =
      wheelbase_m > kEps ? std::tan(ego.steering_angle_rad) / wheelbase_m : 0.0;
  if (s_hint.has_value()) {
    const std::optional<FrenetState> near = route.line().ToFrenetStateNear(
        c, options.projection_max_dist_m, *s_hint, options.projection_back_m,
        options.projection_ahead_m);
    if (near.has_value()) {
      return near;
    }
  }
  return route.line().ToFrenetState(c, options.projection_max_dist_m);
}

LatticeSampler::LatticeSampler(LatticeOptions options, LatticeLimits limits)
    : options_(std::move(options)), limits_(limits) {}

namespace {

// Builds the 81-point trajectory of one path x speed pair. s past the line
// end is clamped there at rest (the line ends 50 m past the goal, M0 §2.5,
// so a normal candidate never gets there).
Candidate Combine(const RouteLine& route, const PathSpec& path,
                  const SpeedSpec& speed, std::uint32_t id) {
  Candidate c;
  c.id = id;
  c.d_f_m = path.d_f;
  c.s_f_m = path.s_f;
  c.v_target_mps = speed.v_target;
  c.horizon_s = speed.horizon;
  c.speed_kind = speed.kind;
  c.trajectory.reserve(nuway_common::kTrajectoryPoints);
  c.frenet.reserve(nuway_common::kTrajectoryPoints);
  const double s_end = route.length();
  for (int i = 0; i < nuway_common::kTrajectoryPoints; ++i) {
    const double t = i * nuway_common::kTrajectoryDtS;
    FrenetState f;
    speed.At(t, &f.s, &f.s_dot, &f.s_ddot);
    if (f.s >= s_end) {
      f.s = s_end;
      f.s_dot = 0.0;
      f.s_ddot = 0.0;
    }
    path.At(f.s, &f.d, &f.d_prime, &f.d_dprime);
    const CartesianState cart = route.line().ToCartesianState(f);
    nuway_common::TrajectoryPoint p;
    p.t = t;
    p.x = cart.x;
    p.y = cart.y;
    p.yaw = cart.yaw;
    p.v = cart.v;
    p.a = cart.a;
    p.kappa = cart.kappa;
    c.trajectory.push_back(p);
    c.frenet.push_back(f);
  }
  return c;
}

}  // namespace

std::vector<Candidate> LatticeSampler::Sample(
    const SceneInput& in, const FrenetState& ego,
    const BehaviorOutput& decision) const {
  std::vector<Candidate> out;
  const RouteLine& route = *in.route;
  const double s_end = route.length();
  const double v = std::max(0.0, ego.s_dot);

  // Lateral set. The reference line is the target lane's centre in every
  // state (it already follows the route's lane changes, M0 §3.3; Decisions
  // log 2026-09-09), so d_f is offset from 0; a CHANGE_* adds the current
  // offset so the planner can hold its lane while it waits for a gap.
  std::vector<PathSpec> paths;
  if (s_end - ego.s < options_.min_lateral_fit_m) {
    PathSpec hold;
    hold.s0 = ego.s;
    hold.s_f = s_end;
    hold.d0 = ego.d;
    hold.d_f = ego.d;
    hold.fitted = false;
    paths.push_back(hold);
  } else {
    std::vector<double> targets = options_.d_offsets_m;
    if (decision.lateral != Lateral::kKeep) {
      targets.push_back(ego.d);
    }
    std::vector<double> ends;
    for (const double ds : options_.ds_set_m) {
      const double s_f =
          std::min(s_end, ego.s + std::max(ds, options_.ds_speed_factor_s * v));
      bool duplicate = false;
      for (const double e : ends) {
        duplicate = duplicate || std::abs(e - s_f) < kEps;
      }
      if (!duplicate) {
        ends.push_back(s_f);
      }
    }
    for (const double d_f : targets) {
      for (const double s_f : ends) {
        PathSpec path;
        path.s0 = ego.s;
        path.s_f = s_f;
        path.d0 = ego.d;
        path.d_f = d_f;
        path.poly = Polynomial5::Quintic(ego.d, ego.d_prime, ego.d_dprime, d_f,
                                         0.0, 0.0, s_f - ego.s);
        paths.push_back(path);
      }
    }
  }

  // Longitudinal set by behavior state.
  std::vector<SpeedSpec> speeds;
  const double v_limit = route.SpeedLimitAt(ego.s);
  const auto add_keeping = [&](double v_center) {
    std::vector<double> targets;
    for (const double offset : options_.speed_offsets_mps) {
      const double v_t = std::clamp(v_center + offset, 0.0, v_limit);
      bool duplicate = false;
      for (const double e : targets) {
        duplicate = duplicate || std::abs(e - v_t) < kEps;
      }
      if (!duplicate) {
        targets.push_back(v_t);
      }
    }
    for (const double v_t : targets) {
      for (const double horizon : options_.keep_horizons_s) {
        speeds.push_back(KeepSpeed(ego, v_t, horizon));
      }
    }
  };
  const auto add_stops = [&](double s_stop) {
    if (s_stop - ego.s > 0.5) {
      for (const double horizon : options_.stop_horizons_s) {
        speeds.push_back(StopAt(ego, s_stop, horizon, SpeedKind::kStop));
      }
    }
  };
  switch (decision.longitudinal) {
    case Longitudinal::kFree:
      add_keeping(decision.target_speed_mps);
      break;
    case Longitudinal::kFollow: {
      add_keeping(decision.target_speed_mps);
      const std::optional<LeadOnLine> lead =
          ProjectLead(in, ego, decision.lead_agent_id);
      if (lead.has_value()) {
        for (const double horizon : options_.keep_horizons_s) {
          // Where the ego's rear axle sits when its front keeps the IDM
          // desired gap s0 + T v behind the lead's rear at time `horizon`.
          const double s_gap =
              lead->s_rear + (lead->v * horizon) -
              (options_.idm_s0_m + (options_.idm_t_s * lead->v)) -
              limits_.ego_front_m;
          if (s_gap > ego.s) {
            SpeedSpec spec;
            spec.kind = SpeedKind::kGap;
            spec.v_target = lead->v;
            spec.horizon = horizon;
            spec.poly = Polynomial5::Quintic(ego.s, ego.s_dot, ego.s_ddot,
                                             s_gap, lead->v, 0.0, horizon);
            speeds.push_back(spec);
          }
        }
      }
      break;
    }
    case Longitudinal::kYield:
      add_stops(decision.stop_s);
      // The "go" candidate: keep the free-road bound.
      speeds.push_back(KeepSpeed(ego, route.SpeedBoundAt(ego.s, {}),
                                 options_.keep_horizons_s.back()));
      break;
    case Longitudinal::kStop:
      add_stops(decision.stop_s);
      speeds.push_back(
          ConstantDecel(ego, -limits_.a_min_mps2, SpeedKind::kHardStop));
      break;
  }

  std::uint32_t id = 0;
  for (const PathSpec& path : paths) {
    for (const SpeedSpec& speed : speeds) {
      out.push_back(Combine(route, path, speed, id++));
    }
  }
  std::vector<Candidate> injected = SampleInjected(in, ego, id);
  out.insert(out.end(), std::make_move_iterator(injected.begin()),
             std::make_move_iterator(injected.end()));
  return out;
}

std::vector<Candidate> LatticeSampler::SampleInjected(
    const SceneInput& in, const FrenetState& ego,
    std::uint32_t first_id) const {
  const RouteLine& route = *in.route;
  const double v = std::max(0.0, ego.s_dot);
  // Both hold the current lane's centre: the line (Sample()'s convention),
  // reached with the same lateral quintic as a normal candidate over the
  // gentle stopping distance, or held where the ego is when nearly stopped.
  PathSpec path;
  path.s0 = ego.s;
  path.d0 = ego.d;
  path.d_f = 0.0;
  const double d_gentle = (v * v) / (2.0 * options_.a_gentle_mps2);
  path.s_f = std::min(route.length(), ego.s + std::max(d_gentle, 5.0));
  if (v < options_.stopped_speed_mps ||
      path.s_f - ego.s < options_.min_lateral_fit_m) {
    path.fitted = false;
    path.d_f = ego.d;
  } else {
    path.poly = Polynomial5::Quintic(ego.d, ego.d_prime, ego.d_dprime, 0.0, 0.0,
                                     0.0, path.s_f - ego.s);
  }
  SpeedSpec gentle;
  if (v < options_.stopped_speed_mps) {
    gentle = ConstantDecel(ego, options_.a_gentle_mps2, SpeedKind::kGentleStop);
  } else {
    const double horizon = std::clamp(
        v / options_.a_gentle_mps2, options_.min_horizon_s,
        nuway_common::kTrajectoryDtS * (nuway_common::kTrajectoryPoints - 1));
    gentle = StopAt(ego, ego.s + d_gentle, horizon, SpeedKind::kGentleStop);
  }
  const SpeedSpec hard =
      ConstantDecel(ego, -limits_.a_min_mps2, SpeedKind::kHardStop);
  std::vector<Candidate> out;
  for (const SpeedSpec& speed : {gentle, hard}) {
    Candidate c = Combine(route, path, speed, first_id++);
    c.source = "stop";
    c.injected = true;
    out.push_back(std::move(c));
  }
  return out;
}

void LatticeSampler::Filter(const RouteLine& route,
                            std::vector<Candidate>* candidates) const {
  const double half_width = 0.5 * limits_.width_m;
  for (Candidate& c : *candidates) {
    if (c.injected) {
      continue;
    }
    c.reject.clear();
    if (c.trajectory.size() !=
        static_cast<std::size_t>(nuway_common::kTrajectoryPoints)) {
      c.reject = "length";
      continue;
    }
    for (std::size_t i = 0; i < c.trajectory.size() && c.reject.empty(); ++i) {
      const nuway_common::TrajectoryPoint& p = c.trajectory[i];
      const FrenetState& f = c.frenet[i];
      const LateralBounds bounds = route.BoundsAt(f.s);
      if (p.v < -0.05) {
        c.reject = "reverse";
      } else if (std::abs(p.kappa) > limits_.kappa_phys + kEps) {
        c.reject = "kappa";
      } else if (p.a < limits_.a_min_mps2 - kEps ||
                 p.a > limits_.a_max_mps2 + kEps) {
        c.reject = "accel";
      } else if (p.v * p.v * std::abs(p.kappa) >
                 limits_.a_lat_max_mps2 + kEps) {
        c.reject = "a_lat";
      } else if (f.d > bounds.left_m - half_width + kEps ||
                 f.d < -bounds.right_m + half_width - kEps) {
        c.reject = "bounds";
      }
    }
  }
}

const char* SpeedKindName(SpeedKind kind) {
  switch (kind) {
    case SpeedKind::kKeep:
      return "keep";
    case SpeedKind::kGap:
      return "gap";
    case SpeedKind::kStop:
      return "stop";
    case SpeedKind::kHardStop:
      return "hard_stop";
    case SpeedKind::kGentleStop:
      return "gentle_stop";
  }
  return "unknown";
}

}  // namespace nuway_planning
