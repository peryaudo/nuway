// BehaviorFsm: the rules of M1 §3.2, one function per rule. Frames: every
// position is map frame; s and d are along the route line (route_line.h).
#include "nuway_planning/behavior_fsm.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace nuway_planning {

using nuway_common::AgentClass;
using nuway_common::AgentState;
using nuway_common::PredictionSet;
using nuway_common::SE2;

BehaviorOutput NoInputDecision() {
  BehaviorOutput out;
  out.longitudinal = Longitudinal::kStop;
  out.reason = "no_input";
  return out;
}

BehaviorFsm::BehaviorFsm(BehaviorFsmOptions options) : options_(options) {}

void BehaviorFsm::Reset() {
  last_s_.reset();
  committed_ = Lateral::kKeep;
  committed_lane_ = 0;
  commit_age_s_ = 0.0;
  slow_lead_age_s_ = 0.0;
  yellow_latch_.clear();
  last_color_.clear();
  honoured_signs_.clear();
  dwelling_sign_ = 0;
  dwell_age_s_ = 0.0;
  yield_age_s_ = 0.0;
  yield_stop_s_.reset();
  yield_reason_.clear();
}

std::uint32_t BehaviorFsm::EgoLane(const SceneInput& in, double s) const {
  if (in.graph != nullptr) {
    const std::optional<nuway_map::LaneQuery> q =
        in.graph->NearestLane(in.ego.pose.x, in.ego.pose.y, in.ego.pose.yaw,
                              options_.lane_search_dist_m);
    if (q.has_value()) {
      return q->lane_id;
    }
  }
  return in.route->LaneIdAt(s);
}

double BehaviorFsm::LaneHalfWidth(const SceneInput& in, std::uint32_t lane_id,
                                  double x, double y) const {
  if (in.graph != nullptr && lane_id != 0) {
    const nuway_map::Lane* lane = in.graph->lane(lane_id);
    const nuway_common::ReferenceLine* line = in.graph->reference_line(lane_id);
    if (lane != nullptr && line != nullptr && !lane->width.empty()) {
      std::size_t idx = 0;
      const std::optional<nuway_common::FrenetPoint> f =
          line->ToFrenet(x, y, 10.0);
      if (f.has_value()) {
        idx = static_cast<std::size_t>(line->SegmentIndex(f->s));
      }
      idx = std::min(idx, lane->width.size() - 1);
      return 0.5 * lane->width[idx];
    }
  }
  return options_.default_lane_half_width_m;
}

// The lead of M1 §3.2: nearest agent with s > s_ego within follow_range
// whose Frenet d lies within half width + slack of the lane band centre.
const AgentState* BehaviorFsm::FindLead(const SceneInput& in, double s_ego,
                                        double d_lane,
                                        double half_width) const {
  const AgentState* best = nullptr;
  double best_s = std::numeric_limits<double>::infinity();
  for (const AgentState& agent : in.agents) {
    const std::optional<nuway_common::FrenetPoint> f = in.route->ProjectNear(
        agent.pose.x, agent.pose.y, options_.follow_range_m, s_ego,
        options_.projection_back_m, options_.follow_range_m + 10.0);
    if (!f.has_value()) {
      continue;
    }
    if (f->s <= s_ego || f->s - s_ego > options_.follow_range_m) {
      continue;
    }
    if (std::abs(f->d - d_lane) > half_width + options_.lane_slack_m) {
      continue;
    }
    if (f->s < best_s) {
      best_s = f->s;
      best = &agent;
    }
  }
  return best;
}

bool BehaviorFsm::GapClear(const SceneInput& in, double s_ego, double d_lane,
                           double half_width, double back_m,
                           double ahead_m) const {
  const PredictionSet& pred = in.predictions;
  for (const AgentState& agent : in.agents) {
    std::vector<SE2> poses = {agent.pose};
    const int a = pred.IndexOf(agent.id);
    if (a >= 0) {
      for (int t = 0; t < pred.num_timesteps; ++t) {
        if (pred.TimeAt(t) > options_.gap_horizon_s) {
          break;
        }
        for (int s = 0; s < pred.num_samples; ++s) {
          poses.push_back(pred.PoseAt(s, a, t));
        }
      }
    }
    for (const SE2& pose : poses) {
      const std::optional<nuway_common::FrenetPoint> f = in.route->ProjectNear(
          pose.x, pose.y, half_width + options_.lane_slack_m + 5.0, s_ego,
          back_m + 10.0, ahead_m + 10.0);
      if (!f.has_value()) {
        continue;
      }
      const double ds = f->s - s_ego;
      if (ds < -back_m || ds > ahead_m) {
        continue;
      }
      if (std::abs(f->d - d_lane) <= half_width + options_.lane_slack_m) {
        return false;
      }
    }
  }
  return true;
}

// Lateral intent (M1 §3.2). The route requirement is read off the lane ids:
// the line already carries the route's lane changes (M0 §2.5), so the
// decision names the lane the line is on and the sampler keeps aiming at
// the line. An overtake is the one change the line does not carry.
void BehaviorFsm::StepLateral(const SceneInput& in, double s_ego, double d_ego,
                              std::uint32_t ego_lane, const AgentState* lead,
                              double v_limit, BehaviorOutput* out) {
  const std::uint32_t route_lane = in.route->LaneIdAt(s_ego);
  Lateral desired = Lateral::kKeep;
  std::uint32_t desired_lane = route_lane != 0 ? route_lane : ego_lane;
  const nuway_map::Lane* lane = (in.graph != nullptr && ego_lane != 0)
                                    ? in.graph->lane(ego_lane)
                                    : nullptr;
  if (lane != nullptr && route_lane != 0 && route_lane != ego_lane) {
    if (lane->left_neighbor == route_lane) {
      desired = Lateral::kChangeLeft;
    } else if (lane->right_neighbor == route_lane) {
      desired = Lateral::kChangeRight;
    }
  }
  // Overtaking: a lead well below the limit for slow_lead_s, and a clear
  // gap in a neighbour lane we may change into.
  if (lead != nullptr &&
      lead->speed_mps() < v_limit - options_.slow_lead_margin_mps) {
    slow_lead_age_s_ += in.dt_s;
  } else {
    slow_lead_age_s_ = 0.0;
  }
  const double half_width =
      LaneHalfWidth(in, ego_lane, in.ego.pose.x, in.ego.pose.y);
  if (options_.overtake_enabled && desired == Lateral::kKeep &&
      committed_ == Lateral::kKeep && lane != nullptr &&
      slow_lead_age_s_ > options_.slow_lead_s) {
    const double lane_width = 2.0 * half_width;
    if (lane->left_neighbor != 0 && lane->left_change_allowed &&
        GapClear(in, s_ego, d_ego + lane_width, half_width, options_.gap_back_m,
                 options_.gap_ahead_m)) {
      desired = Lateral::kChangeLeft;
      desired_lane = lane->left_neighbor;
    } else if (lane->right_neighbor != 0 && lane->right_change_allowed &&
               GapClear(in, s_ego, d_ego - lane_width, half_width,
                        options_.gap_back_m, options_.gap_ahead_m)) {
      desired = Lateral::kChangeRight;
      desired_lane = lane->right_neighbor;
    }
  }
  // Commitment with hysteresis: a change is held for change_commit_s and
  // until the ego is on the target lane, unless the gap closes.
  if (committed_ == Lateral::kKeep && desired != Lateral::kKeep) {
    committed_ = desired;
    committed_lane_ = desired_lane;
    commit_age_s_ = 0.0;
  } else if (committed_ != Lateral::kKeep) {
    commit_age_s_ += in.dt_s;
    const double d_target =
        d_ego + ((committed_ == Lateral::kChangeLeft ? 1.0 : -1.0) *
                 (2.0 * half_width));
    const bool route_change = committed_lane_ == route_lane;
    const bool arrived = ego_lane == committed_lane_;
    const bool aborted =
        !route_change && !GapClear(in, s_ego, d_target, half_width,
                                   options_.abort_gap_m, options_.abort_gap_m);
    if (aborted || (arrived && commit_age_s_ >= options_.change_commit_s)) {
      committed_ = Lateral::kKeep;
      committed_lane_ = 0;
      commit_age_s_ = 0.0;
      if (aborted) {
        out->reason += "change_aborted;";
      }
    }
  }
  out->lateral = committed_;
  out->target_lane_id =
      committed_ != Lateral::kKeep ? committed_lane_ : desired_lane;
}

// The dilemma-zone rule of M1 §3.2, evaluated on the first yellow tick and
// latched: with d_brake = v t_react + v^2 / (2 a_comf) the car cannot stop
// comfortably when d_brake > d_stop (proceed); otherwise it stops unless it
// clears the line before the red (d_stop / v <= remaining yellow).
bool BehaviorFsm::YellowMeansStop(const TrafficLightObs& light, double d_stop,
                                  double v) {
  const auto latched = yellow_latch_.find(light.id);
  if (latched != yellow_latch_.end()) {
    return latched->second;
  }
  const double d_brake =
      (v * options_.t_react_s) + ((v * v) / (2.0 * options_.a_comf_mps2));
  bool stop = false;
  if (d_brake > d_stop) {
    stop = false;
  } else {
    const double t_yellow =
        light.time_in_state_s >= 0.0
            ? light.yellow_duration_s - light.time_in_state_s
            : light.yellow_duration_s;
    stop = d_stop / std::max(v, 0.1) > t_yellow;
  }
  yellow_latch_[light.id] = stop;
  return stop;
}

std::optional<double> BehaviorFsm::StopLineAhead(const SceneInput& in,
                                                 double s_ego, double v,
                                                 std::string* reason) {
  std::optional<double> best;
  std::string best_reason;
  const auto consider = [&](double s_line, const std::string& why) {
    if (!best.has_value() || s_line < *best) {
      best = s_line;
      best_reason = why;
    }
  };
  const double d_brake =
      (v * options_.t_react_s) + ((v * v) / (2.0 * options_.a_comf_mps2));
  // Traffic lights.
  for (const TrafficLightObs& light : in.lights) {
    bool on_route = false;
    for (const std::uint32_t lane : light.affected_lane_ids) {
      if (in.route->IsRouteLaneAhead(lane, s_ego)) {
        on_route = true;
        break;
      }
    }
    if (!on_route) {
      continue;
    }
    const std::optional<nuway_common::FrenetPoint> f =
        in.route->ProjectNear(light.stop_line.x(), light.stop_line.y(), 6.0,
                              s_ego, options_.projection_back_m, 300.0);
    if (!f.has_value() || f->s < s_ego - options_.passed_line_m ||
        f->s - s_ego > 300.0) {
      yellow_latch_.erase(light.id);
      last_color_.erase(light.id);
      continue;
    }
    TrafficLightColor color = light.state;
    if (light.confidence < options_.unknown_confidence) {
      color = TrafficLightColor::kUnknown;
    }
    const auto last = last_color_.find(light.id);
    if (last == last_color_.end() || last->second != color) {
      yellow_latch_.erase(light.id);  // the light changed: re-evaluate
    }
    last_color_[light.id] = color;
    const double d_stop = f->s - s_ego;
    switch (color) {
      case TrafficLightColor::kRed:
        consider(f->s, "red_light");
        break;
      case TrafficLightColor::kYellow:
        if (YellowMeansStop(light, d_stop, v)) {
          consider(f->s, "yellow_light");
        }
        break;
      case TrafficLightColor::kUnknown:
        if (d_stop > d_brake) {
          consider(f->s, "unknown_light");
        }
        break;
      default:
        break;
    }
  }
  // Stop signs (from the lane graph; honoured once per episode).
  if (in.graph != nullptr) {
    for (const nuway_map::StopSign& sign : in.graph->stop_signs()) {
      if (honoured_signs_.count(sign.id) > 0) {
        continue;
      }
      bool on_route = false;
      for (const std::uint32_t lane : sign.affected_lane_ids) {
        if (in.route->IsRouteLaneAhead(lane, s_ego)) {
          on_route = true;
          break;
        }
      }
      if (!on_route) {
        continue;
      }
      const std::optional<nuway_common::FrenetPoint> f =
          in.route->ProjectNear(sign.stop_line.x(), sign.stop_line.y(), 6.0,
                                s_ego, options_.projection_back_m, 300.0);
      if (!f.has_value() || f->s < s_ego - options_.passed_line_m) {
        continue;
      }
      const double d_stop = f->s - s_ego;
      if (std::abs(d_stop) <= options_.stop_sign_dist_m &&
          v < options_.stop_sign_speed_mps) {
        if (dwelling_sign_ != sign.id) {
          dwelling_sign_ = sign.id;
          dwell_age_s_ = 0.0;
        }
        dwell_age_s_ += in.dt_s;
        if (dwell_age_s_ >= options_.stop_sign_hold_s) {
          honoured_signs_.insert(sign.id);
          dwelling_sign_ = 0;
          continue;
        }
      } else if (dwelling_sign_ == sign.id) {
        dwelling_sign_ = 0;
        dwell_age_s_ = 0.0;
      }
      consider(f->s, "stop_sign");
    }
  }
  // The route goal.
  if (in.route->goal_s() >= s_ego - options_.passed_line_m) {
    consider(in.route->goal_s(), "goal");
  }
  if (best.has_value()) {
    *reason = best_reason;
  }
  return best;
}

std::optional<double> BehaviorFsm::ConflictAhead(const SceneInput& in,
                                                 double s_ego, double v,
                                                 double half_width,
                                                 std::string* reason) const {
  const PredictionSet& pred = in.predictions;
  std::optional<double> best;
  for (const AgentState& agent : in.agents) {
    // A standing agent is not crossing anything: a walker parked on the
    // corner held a yield stop forever (task 17 protocol, dev03_01). If it
    // is in the path, the collision check stops the car before it.
    if (agent.speed_mps() < options_.yield_min_speed_mps) {
      continue;
    }
    const bool pedestrian = agent.class_id == AgentClass::kPedestrian;
    std::vector<std::pair<double, SE2>> poses = {{0.0, agent.pose}};
    const int a = pred.IndexOf(agent.id);
    if (a >= 0) {
      for (int t = 0; t < pred.num_timesteps; ++t) {
        if (pred.TimeAt(t) > options_.yield_horizon_s) {
          break;
        }
        for (int s = 0; s < pred.num_samples; ++s) {
          poses.emplace_back(pred.TimeAt(t), pred.PoseAt(s, a, t));
        }
      }
    }
    for (const auto& [t_agent, pose] : poses) {
      const std::optional<nuway_common::FrenetPoint> f = in.route->ProjectNear(
          pose.x, pose.y, half_width + options_.yield_margin_m + 3.0, s_ego,
          options_.projection_back_m, options_.yield_lookahead_m + 10.0);
      if (!f.has_value()) {
        continue;
      }
      const double ds = f->s - s_ego;
      if (ds < 1.0 || ds > options_.yield_lookahead_m) {
        continue;
      }
      if (std::abs(f->d) > half_width + options_.yield_margin_m) {
        continue;
      }
      const double heading = in.route->line().HeadingAt(f->s);
      const bool crossing =
          pedestrian || std::abs(nuway_common::WrapAngle(pose.yaw - heading)) >
                            options_.crossing_angle_rad;
      if (!crossing) {
        continue;
      }
      const double t_ego = ds / std::max(v, 1.0);
      if (t_agent > t_ego + options_.yield_time_margin_s) {
        continue;  // we clear the region before the agent gets there
      }
      if (t_agent < t_ego - options_.yield_time_margin_s) {
        continue;  // the agent is through the region before we arrive
      }
      if (!best.has_value() || f->s < *best) {
        best = f->s;
        *reason = "yield:agent" + std::to_string(agent.id);
      }
    }
  }
  return best;
}

// The Intelligent Driver Model (Treiber, Hennecke, Helbing 2000):
//   a = a_max (1 - (v / v0)^4 - (s* / gap)^2),
//   s* = s0 + max(0, v T + v dv / (2 sqrt(a_max b))),
// the free-road term drives v toward v0, the interaction term keeps the
// desired gap s* (jam distance plus time headway plus a braking term
// against the closing speed dv). The target speed handed to the sampler is
// the speed IDM reaches in idm_horizon_s, clamped to [0, v_limit].
double BehaviorFsm::IdmTargetSpeed(double v, double gap_m, double v_lead,
                                   double v_limit) const {
  const double v0 = std::max(v_limit, 0.1);
  const double dv = v - v_lead;
  const double s_star =
      options_.idm_s0_m +
      std::max(
          0.0,
          (v * options_.idm_t_s) +
              (v * dv /
               (2.0 * std::sqrt(options_.idm_a_mps2 * options_.idm_b_mps2))));
  const double gap = std::max(gap_m, 0.1);
  const double ratio = v / v0;
  const double a_idm =
      options_.idm_a_mps2 * (1.0 - (ratio * ratio * ratio * ratio) -
                             ((s_star / gap) * (s_star / gap)));
  return std::clamp(v + (a_idm * options_.idm_horizon_s), 0.0, v_limit);
}

BehaviorOutput BehaviorFsm::Step(const SceneInput& in) {
  if (in.route == nullptr || in.route->empty()) {
    last_s_.reset();
    return NoInputDecision();
  }
  const std::optional<nuway_common::FrenetPoint> ego_f = in.route->Project(
      in.ego.pose.x, in.ego.pose.y, options_.projection_max_dist_m, last_s_,
      options_.projection_back_m, options_.projection_ahead_m);
  if (!ego_f.has_value()) {
    last_s_.reset();
    BehaviorOutput out = NoInputDecision();
    out.reason = "off_route";
    return out;
  }
  const double s = ego_f->s;
  const double d = ego_f->d;
  last_s_ = s;
  const double v = std::max(0.0, in.ego.vx_mps);
  BehaviorOutput out;
  out.ego_s = s;
  out.ego_d = d;
  out.reason.clear();

  const std::uint32_t ego_lane = EgoLane(in, s);
  const double half_width =
      LaneHalfWidth(in, ego_lane, in.ego.pose.x, in.ego.pose.y);
  const double v_limit = in.route->SpeedLimitAt(s);
  const double v_bound = in.route->SpeedBoundAt(s, options_.speed);

  // Lead in the route lane band (d = 0) or in the ego's own band when the
  // ego is off the line (mid lane change): the nearer of the two.
  const AgentState* lead = FindLead(in, s, 0.0, half_width);
  if (std::abs(d) > half_width) {
    const AgentState* own = FindLead(in, s, d, half_width);
    if (own != nullptr) {
      if (lead == nullptr) {
        lead = own;
      } else {
        const std::optional<nuway_common::FrenetPoint> f_lead =
            in.route->ProjectNear(lead->pose.x, lead->pose.y, 100.0, s, 10.0,
                                  100.0);
        const std::optional<nuway_common::FrenetPoint> f_own =
            in.route->ProjectNear(own->pose.x, own->pose.y, 100.0, s, 10.0,
                                  100.0);
        if (f_lead.has_value() && f_own.has_value() && f_own->s < f_lead->s) {
          lead = own;
        }
      }
    }
  }

  StepLateral(in, s, d, ego_lane, lead, v_limit, &out);

  // Longitudinal, in priority order.
  std::string why;
  const std::optional<double> stop_line = StopLineAhead(in, s, v, &why);
  const double stopping_distance =
      ((v * v) / (2.0 * options_.a_comf_mps2)) + options_.stop_lookahead_m;
  if (stop_line.has_value() && *stop_line - s <= stopping_distance) {
    out.longitudinal = Longitudinal::kStop;
    out.stop_s = *stop_line - options_.stop_margin_m;
    const double remaining = std::max(0.0, out.stop_s - s);
    out.target_speed_mps =
        std::min(v_bound, std::sqrt(2.0 * options_.a_comf_mps2 * remaining));
    out.reason += why;
    return out;
  }
  std::string yield_why;
  std::optional<double> conflict =
      ConflictAhead(in, s, v, half_width, &yield_why);
  if (conflict.has_value()) {
    yield_age_s_ = 0.0;
    yield_stop_s_ = std::max(s, *conflict - options_.yield_stop_back_m);
    yield_reason_ = yield_why;
  } else if (yield_stop_s_.has_value()) {
    // Hysteresis: hold the last yield until it has aged out or is behind.
    yield_age_s_ += in.dt_s;
    // The epsilon keeps ten 0.1 s ticks from summing to 0.999... s.
    if (yield_age_s_ + 1e-9 < options_.yield_hold_s &&
        *yield_stop_s_ >= s - 1.0) {
      conflict = *yield_stop_s_ + options_.yield_stop_back_m;
      yield_why = yield_reason_ + "(held)";
    } else {
      yield_stop_s_.reset();
    }
  }
  if (conflict.has_value()) {
    out.longitudinal = Longitudinal::kYield;
    out.stop_s = std::max(s, *conflict - options_.yield_stop_back_m);
    const double remaining = std::max(0.0, out.stop_s - s);
    out.target_speed_mps =
        std::min(v_bound, std::sqrt(2.0 * options_.a_comf_mps2 * remaining));
    out.reason += yield_why;
    return out;
  }
  if (lead != nullptr) {
    out.longitudinal = Longitudinal::kFollow;
    out.lead_agent_id = lead->id;
    const std::optional<nuway_common::FrenetPoint> lf = in.route->ProjectNear(
        lead->pose.x, lead->pose.y, 100.0, s, 10.0, 100.0);
    const double gap = lf.has_value() ? (lf->s - (0.5 * lead->length_m)) -
                                            (s + options_.ego_front_m)
                                      : options_.follow_range_m;
    const double heading =
        in.route->line().HeadingAt(lf.has_value() ? lf->s : s);
    const double v_lead = std::max(0.0, (lead->vx_mps * std::cos(heading)) +
                                            (lead->vy_mps * std::sin(heading)));
    out.target_speed_mps =
        std::min(v_bound, IdmTargetSpeed(v, gap, v_lead, v_limit));
    out.reason += "follow:agent" + std::to_string(lead->id);
    return out;
  }
  out.longitudinal = Longitudinal::kFree;
  out.target_speed_mps = v_bound;
  out.reason += "free";
  return out;
}

}  // namespace nuway_planning
