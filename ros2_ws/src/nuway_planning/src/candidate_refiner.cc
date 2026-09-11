#include "nuway_planning/candidate_refiner.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nuway_common/agents.h>
#include <nuway_common/geometry.h>
#include <nuway_common/trajectory.h>

namespace nuway_planning {
namespace {

using nuway_common::AgentState;
using nuway_common::CartesianState;
using nuway_common::FrenetState;
using nuway_common::PredictionSet;

// A refined lateral profile d(s) at uniform knots, evaluated by linear
// interpolation between them (the knots are 1 m apart and the candidate
// is resampled at 0.1 s, so the interpolation error is far below the
// tracking tolerance of the controller).
struct LateralProfile {
  double s0 = 0.0;
  double ds = 1.0;
  std::vector<double> d;
  std::vector<double> d_prime;
  std::vector<double> d_dprime;

  void At(double s, double* dd, double* dp, double* dpp) const {
    if (d.size() < 2) {
      *dd = d.empty() ? 0.0 : d.front();
      *dp = 0.0;
      *dpp = 0.0;
      return;
    }
    const double u =
        std::clamp((s - s0) / ds, 0.0, static_cast<double>(d.size() - 1));
    const auto i = static_cast<std::size_t>(std::floor(u));
    const std::size_t j = std::min(i + 1, d.size() - 1);
    const double a = u - static_cast<double>(i);
    *dd = d[i] + (a * (d[j] - d[i]));
    *dp = d_prime[i] + (a * (d_prime[j] - d_prime[i]));
    *dpp = d_dprime[i] + (a * (d_dprime[j] - d_dprime[i]));
  }
};

// The candidate's lattice d at arc length s, from its Frenet samples
// (monotone in s while moving; held where the candidate has stopped).
double LatticeDAt(const Candidate& c, double s) {
  const std::vector<FrenetState>& f = c.frenet;
  if (f.empty()) {
    return 0.0;
  }
  if (s <= f.front().s) {
    return f.front().d;
  }
  for (std::size_t i = 1; i < f.size(); ++i) {
    if (f[i].s >= s) {
      const double span = f[i].s - f[i - 1].s;
      const double a = span > 1e-9 ? (s - f[i - 1].s) / span : 1.0;
      return f[i - 1].d + (a * (f[i].d - f[i - 1].d));
    }
  }
  return f.back().d;
}

// Rebuilds the candidate's Cartesian samples from its Frenet samples.
void RebuildTrajectory(const RouteLine& route, Candidate* c) {
  for (std::size_t i = 0; i < c->frenet.size(); ++i) {
    const CartesianState cart = route.line().ToCartesianState(c->frenet[i]);
    nuway_common::TrajectoryPoint& p = c->trajectory[i];
    p.x = cart.x;
    p.y = cart.y;
    p.yaw = cart.yaw;
    p.v = cart.v;
    p.a = cart.a;
    p.kappa = cart.kappa;
  }
}

// The agent's pose at time t from its sample (linear between steps, the
// observed pose before the first step, the last step held after).
nuway_common::SE2 AgentPoseAt(const AgentState& agent, const PredictionSet& set,
                              int index, int sample, double t) {
  if (index < 0 || set.num_timesteps <= 0 || set.dt_s <= 0.0 || t <= 0.0) {
    return agent.pose;
  }
  const double u = t / set.dt_s;  // step k is at time (k + 1) dt
  const int k1 =
      std::min(static_cast<int>(std::ceil(u)) - 1, set.num_timesteps - 1);
  if (k1 < 0) {
    return agent.pose;
  }
  const nuway_common::SE2 p1 = set.PoseAt(sample, index, k1);
  const nuway_common::SE2 p0 =
      k1 == 0 ? agent.pose : set.PoseAt(sample, index, k1 - 1);
  const double t0 = k1 == 0 ? 0.0 : set.TimeAt(k1 - 1);
  const double t1 = set.TimeAt(k1);
  const double a = std::clamp((t - t0) / (t1 - t0), 0.0, 1.0);
  return nuway_common::SE2{
      p0.x + (a * (p1.x - p0.x)), p0.y + (a * (p1.y - p0.y)),
      nuway_common::WrapAngle(p0.yaw +
                              (a * nuway_common::WrapAngle(p1.yaw - p0.yaw)))};
}

}  // namespace

CandidateRefiner::CandidateRefiner(RefinerOptions options, LatticeLimits limits,
                                   CollisionOptions collision)
    : options_(options),
      limits_(limits),
      collision_(collision),
      path_qp_(options.qp),
      speed_qp_(options.qp) {}

void CandidateRefiner::OnPlanningTick() {
  speed_qp_.ShiftWarmStart();
  speed_qp_.ShiftWarmStart();
}

void CandidateRefiner::Reset() {
  path_qp_.Reset();
  speed_qp_.Reset();
}

RefineOutcome CandidateRefiner::Refine(const SceneInput& in,
                                       const FrenetState& ego, Candidate* c) {
  RefineOutcome out;
  const RouteLine& route = *in.route;
  const double half_width = 0.5 * limits_.width_m;
  const double ego_rear = collision_.ego_length_m - limits_.ego_front_m;
  if (c->frenet.size() != c->trajectory.size() || c->frenet.empty()) {
    out.message = "malformed candidate";
    return out;
  }

  // ---- Path QP: d over s at path_ds_m knots up to the candidate's end.
  const double s_final = c->frenet.back().s;
  const int n_path = static_cast<int>(std::floor(
                         ((s_final - ego.s) / options_.path_ds_m) + 1e-9)) +
                     1;
  LateralProfile profile;
  profile.s0 = ego.s;
  profile.ds = options_.path_ds_m;
  if (n_path < 4) {  // under 3 m of path: nothing to reshape
    out.path_skipped = true;
    out.path = QpOutcome::kSolved;
  } else {
    PiecewiseJerkProblem p;
    p.step = options_.path_ds_m;
    p.n = n_path;
    p.x0 = {ego.d, ego.d_prime, ego.d_dprime};
    p.w_x = options_.w_d;
    p.w_x_end = options_.w_end;
    p.w_ddx = options_.w_dd;
    p.w_dddx = options_.w_ddd;
    p.x_ref.resize(static_cast<std::size_t>(n_path));
    p.x_lower.resize(static_cast<std::size_t>(n_path));
    p.x_upper.resize(static_cast<std::size_t>(n_path));
    p.ddx_lower.resize(static_cast<std::size_t>(n_path));
    p.ddx_upper.resize(static_cast<std::size_t>(n_path));
    for (int i = 0; i < n_path; ++i) {
      const auto k = static_cast<std::size_t>(i);
      const double s = ego.s + (i * options_.path_ds_m);
      const LateralBounds b = route.BoundsAt(s);
      p.x_ref[k] = LatticeDAt(*c, s);
      p.x_lower[k] = -b.right_m + half_width;
      p.x_upper[k] = b.left_m - half_width;
      const double budget =
          std::max(options_.min_curvature_budget,
                   limits_.kappa_phys - std::abs(route.line().CurvatureAt(s)));
      p.ddx_lower[k] = -budget;
      p.ddx_upper[k] = budget;
    }
    p.ddx_lower[0] = std::min(p.ddx_lower[0], ego.d_dprime);
    p.ddx_upper[0] = std::max(p.ddx_upper[0], ego.d_dprime);
    // Static and slow agents tighten the bound on the side the lattice
    // path passes them, over the s interval their footprint (plus the
    // ego's own length and the margins) occupies.
    for (const AgentState& agent : in.agents) {
      if (agent.speed_mps() >= options_.static_speed_mps) {
        continue;
      }
      const std::optional<nuway_common::FrenetPoint> f =
          route.Project(agent.pose.x, agent.pose.y, 20.0, ego.s, 20.0,
                        s_final - ego.s + 20.0);
      if (!f.has_value()) {
        continue;
      }
      const double heading = route.line().HeadingAt(f->s);
      const double rel =
          std::abs(nuway_common::WrapAngle(agent.pose.yaw - heading));
      // Footprint extent along and across the line for a yawed box.
      const double along = (0.5 * agent.length_m * std::abs(std::cos(rel))) +
                           (0.5 * agent.width_m * std::abs(std::sin(rel)));
      const double across = (0.5 * agent.length_m * std::abs(std::sin(rel))) +
                            (0.5 * agent.width_m * std::abs(std::cos(rel)));
      const double s_lo = f->s - along - collision_.agent_margin_m -
                          limits_.ego_front_m - collision_.margin_lon_m;
      const double s_hi = f->s + along + collision_.agent_margin_m + ego_rear +
                          collision_.margin_lon_m;
      const bool pass_left = LatticeDAt(*c, f->s) >= f->d;
      const double clearance = across + collision_.agent_margin_m + half_width +
                               collision_.margin_lat_m;
      for (int i = 0; i < n_path; ++i) {
        const double s = ego.s + (i * options_.path_ds_m);
        if (s < s_lo || s > s_hi) {
          continue;
        }
        const auto k = static_cast<std::size_t>(i);
        if (pass_left) {
          p.x_lower[k] = std::max(p.x_lower[k], f->d + clearance);
        } else {
          p.x_upper[k] = std::min(p.x_upper[k], f->d - clearance);
        }
      }
    }
    const PiecewiseJerkSolution sol = path_qp_.Solve(p);
    out.path = sol.outcome;
    out.path_iterations = sol.iterations;
    if (sol.outcome == QpOutcome::kFailed) {
      out.message = "path qp " + sol.status;
      c->qp_relaxed = true;
      return out;
    }
    profile.d = sol.x;
    profile.d_prime = sol.dx;
    profile.d_dprime = sol.ddx;
  }
  // The refined path under the lattice speed profile, for the curvature
  // speed bound and the S-T boxes.
  std::vector<FrenetState> frenet = c->frenet;
  if (!out.path_skipped) {
    for (FrenetState& f : frenet) {
      profile.At(f.s, &f.d, &f.d_prime, &f.d_dprime);
    }
  }
  std::vector<double> kappa_path(frenet.size(), 0.0);
  for (std::size_t i = 0; i < frenet.size(); ++i) {
    kappa_path[i] = route.line().ToCartesianState(frenet[i]).kappa;
  }

  // ---- Speed QP: s over t at the trajectory's own 0.1 s knots.
  const int n = static_cast<int>(frenet.size());
  PiecewiseJerkProblem q;
  q.step = nuway_common::kTrajectoryDtS;
  q.n = n;
  q.x0 = {ego.s, ego.s_dot, ego.s_ddot};
  q.w_x = options_.w_s;
  q.w_dx = options_.w_v;
  q.dx_ref = c->v_target_mps;
  q.w_ddx = options_.w_a;
  q.w_dddx = options_.w_j;
  q.dddx_max = options_.jerk_max_mps3;
  q.x_ref.resize(frenet.size());
  q.x_lower.assign(frenet.size(), ego.s);
  q.x_upper.assign(frenet.size(), route.length());
  q.dx_lower.assign(frenet.size(), 0.0);
  q.dx_upper.resize(frenet.size());
  q.ddx_lower.assign(frenet.size(), limits_.a_min_mps2);
  q.ddx_upper.assign(frenet.size(), limits_.a_max_mps2);
  for (std::size_t i = 0; i < frenet.size(); ++i) {
    q.x_ref[i] = frenet[i].s;
    double v_max = route.SpeedLimitAt(frenet[i].s);
    if (std::abs(kappa_path[i]) > 1e-6) {
      v_max = std::min(
          v_max, std::sqrt(limits_.a_lat_max_mps2 / std::abs(kappa_path[i])));
    }
    q.dx_upper[i] = std::max(v_max, 0.0);
  }
  // S-T boxes of the dynamic agents over every sample.
  std::vector<int> index;
  index.reserve(in.agents.size());
  for (const AgentState& agent : in.agents) {
    index.push_back(in.predictions.IndexOf(agent.id));
  }
  const int samples = std::max(1, in.predictions.num_samples);
  for (std::size_t a = 0; a < in.agents.size(); ++a) {
    const AgentState& agent = in.agents[a];
    if (agent.speed_mps() < options_.static_speed_mps) {
      continue;  // bounded laterally by the path QP
    }
    for (int s = 0; s < samples; ++s) {
      const int idx = s < in.predictions.num_samples ? index[a] : -1;
      // The homotopy is fixed where the box first touches the path: the
      // lattice profile behind it then stays behind (upper bounds), ahead
      // stays ahead (lower bounds). Deciding per knot would let a profile
      // that grazes a slower agent late in the horizon ask for both.
      int side = 0;
      for (std::size_t i = 0; i < frenet.size(); ++i) {
        const double t = c->trajectory[i].t;
        const nuway_common::SE2 pose =
            AgentPoseAt(agent, in.predictions, idx, s, t);
        const std::optional<nuway_common::FrenetPoint> f =
            route.Project(pose.x, pose.y, 20.0, frenet[i].s, 40.0, 60.0);
        if (!f.has_value()) {
          continue;
        }
        double d_path = 0.0;
        double unused_p = 0.0;
        double unused_pp = 0.0;
        if (out.path_skipped) {
          d_path = LatticeDAt(*c, f->s);
        } else {
          profile.At(f->s, &d_path, &unused_p, &unused_pp);
        }
        const double heading = route.line().HeadingAt(f->s);
        const double rel =
            std::abs(nuway_common::WrapAngle(pose.yaw - heading));
        const double along = (0.5 * agent.length_m * std::abs(std::cos(rel))) +
                             (0.5 * agent.width_m * std::abs(std::sin(rel)));
        const double across = (0.5 * agent.length_m * std::abs(std::sin(rel))) +
                              (0.5 * agent.width_m * std::abs(std::cos(rel)));
        if (std::abs(f->d - d_path) >= across + collision_.agent_margin_m +
                                           half_width +
                                           collision_.margin_lat_m) {
          continue;  // beside the path at this time
        }
        const double s_lo = f->s - along - collision_.agent_margin_m -
                            limits_.ego_front_m - collision_.margin_lon_m;
        const double s_hi = f->s + along + collision_.agent_margin_m +
                            ego_rear + collision_.margin_lon_m;
        if (side == 0) {
          side = frenet[i].s < s_hi ? -1 : 1;
        }
        if (side < 0) {
          q.x_upper[i] = std::min(q.x_upper[i], s_lo);
        } else {
          q.x_lower[i] = std::max(q.x_lower[i], s_hi);
        }
      }
    }
  }
  // The fixed initial state must sit inside its own boxes (a speeding or
  // hard-braking ego would otherwise make the QP infeasible outright); a
  // box that closes entirely is left to the slack.
  q.dx_upper[0] = std::max(q.dx_upper[0], ego.s_dot);
  q.dx_lower[0] = std::min(q.dx_lower[0], ego.s_dot);
  q.ddx_upper[0] = std::max(q.ddx_upper[0], ego.s_ddot);
  q.ddx_lower[0] = std::min(q.ddx_lower[0], ego.s_ddot);
  // A box closed behind the ego (an agent's inflated rear already at or
  // behind ego.s, e.g. a lead that stopped inside the margin) admits no
  // profile: the slack would have to carry metres, which costs tens of
  // thousands of iterations at the fixed rho. The candidate keeps its
  // lattice shape and the collision check decides its fate.
  for (std::size_t i = 0; i < frenet.size(); ++i) {
    if (q.x_upper[i] < q.x_lower[i]) {
      out.message =
          "speed box closed at " + std::to_string(c->trajectory[i].t) + " s";
      c->qp_relaxed = true;
      return out;
    }
  }
  const PiecewiseJerkSolution sol = speed_qp_.Solve(q);
  out.speed = sol.outcome;
  out.speed_iterations = sol.iterations;
  if (sol.outcome == QpOutcome::kFailed) {
    out.message = "speed qp " + sol.status;
    c->qp_relaxed = true;
    return out;
  }
  for (std::size_t i = 0; i < frenet.size(); ++i) {
    FrenetState& f = frenet[i];
    f.s = std::clamp(sol.x[i], ego.s, route.length());
    f.s_dot = std::max(0.0, sol.dx[i]);
    f.s_ddot = sol.ddx[i];
    if (out.path_skipped) {
      f.d = LatticeDAt(*c, f.s);
    } else {
      profile.At(f.s, &f.d, &f.d_prime, &f.d_dprime);
    }
  }
  c->frenet = std::move(frenet);
  RebuildTrajectory(route, c);
  c->refined = true;
  c->qp_relaxed =
      out.path == QpOutcome::kRelaxed || out.speed == QpOutcome::kRelaxed;
  out.message = std::string("path ") + QpOutcomeName(out.path) + ", speed " +
                QpOutcomeName(out.speed);
  return out;
}

}  // namespace nuway_planning
