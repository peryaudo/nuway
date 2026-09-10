// Lattice sampler (M1 §3.3): the Werling 2010 Frenet lattice. Lateral
// candidates are quintic polynomials d(s) from the ego's (d, d', d'') to
// (d_f, 0, 0) at s_f = s + ds; longitudinal candidates are quartic
// (velocity-keeping) or quintic (stopping, gap-keeping) polynomials s(t)
// chosen by the behavior state; every path x speed pair is converted to a
// map-frame trajectory through ReferenceLine::ToCartesianState and
// resampled at 0.1 s over 8 s (81 points), the shorter profiles extended at
// their terminal state. Two in-lane stop candidates (gentle, hard) are
// injected in every state as the planner's floor; the feasibility filter
// (curvature, acceleration, lateral acceleration, drivable bounds) applies
// to the sampled candidates only. Collision is the checker's rule (§3.4),
// applied by the planner after this filter. No rclcpp.
#ifndef NUWAY_PLANNING_LATTICE_SAMPLER_H_
#define NUWAY_PLANNING_LATTICE_SAMPLER_H_

#include <optional>
#include <vector>

#include <nuway_common/frenet.h>
#include <nuway_control/vehicle_model.h>

#include "nuway_planning/behavior_fsm.h"
#include "nuway_planning/candidate.h"
#include "nuway_planning/route_line.h"
#include "nuway_planning/scene.h"

namespace nuway_planning {

// The vehicle-derived bounds of the feasibility filter. kappa_phys is the
// physical steering limit tan(max_steer) / wheelbase (about 0.96 rad/m for
// the Lincoln), not the YAML's comfort `limits.kappa_max`: the Town03
// junction corners have R = 2.4 m (kappa = 0.42) and a 0.18 bound would
// reject every normal candidate there (§3.3).
struct LatticeLimits {
  double kappa_phys = 0.96;  // 1/m
  double a_min_mps2 = -6.0;  // braking cap (negative)
  double a_max_mps2 = 3.0;   // acceleration cap
  double a_lat_max_mps2 = 4.0;
  double width_m = 1.837;     // ego footprint width (bounds check)
  double wheelbase_m = 2.86;  // ego curvature from the steering angle
  double ego_front_m = 3.9;   // rear axle to front bumper (gap keeping)

  static LatticeLimits FromVehicleModel(const nuway_control::VehicleModel& m);
};

struct LatticeOptions {
  std::vector<double> d_offsets_m = {-1.0, -0.5, 0.0, 0.5, 1.0};
  std::vector<double> ds_set_m = {20.0, 35.0, 50.0};
  double ds_speed_factor_s = 3.0;  // ds = max(ds, factor * v)
  double min_lateral_fit_m = 5.0;  // closer to the line end: hold d
  std::vector<double> speed_offsets_mps = {-3.0, -1.5, 0.0, 1.5};
  std::vector<double> keep_horizons_s = {4.0, 6.0, 8.0};
  std::vector<double> stop_horizons_s = {3.0, 5.0, 7.0};
  double idm_s0_m = 2.0;  // gap-keeping candidate: s0 + T v_lead
  double idm_t_s = 1.5;
  double a_gentle_mps2 = 1.5;          // injected gentle stop
  double gentle_time_factor = 1.5;     // its horizon: factor * v / a_gentle
  double min_horizon_s = 1.0;          // floor of any stop horizon
  double stopped_speed_mps = 0.3;      // below: the stops hold the pose
  double projection_max_dist_m = 5.0;  // ego projection (EgoFrenetState)
  double projection_back_m = 10.0;
  double projection_ahead_m = 50.0;
};

// The ego's Frenet state on the route line: base_link pose, body speed and
// acceleration, curvature tan(steer) / wheelbase. nullopt when the ego is
// off the line (farther than projection_max_dist_m).
std::optional<nuway_common::FrenetState> EgoFrenetState(
    const EgoObs& ego, const RouteLine& route, const LatticeOptions& options,
    double wheelbase_m, std::optional<double> s_hint);

class LatticeSampler {
 public:
  LatticeSampler(LatticeOptions options, LatticeLimits limits);

  // The normal candidates of the decision's state followed by the injected
  // pair, ids 0.. in that order. `in.route` must be set. The feasibility
  // filter is not applied here (Filter()).
  std::vector<Candidate> Sample(const SceneInput& in,
                                const nuway_common::FrenetState& ego,
                                const BehaviorOutput& decision) const;

  // The injected pair alone (case 2 of §3.6: no usable decision).
  std::vector<Candidate> SampleInjected(const SceneInput& in,
                                        const nuway_common::FrenetState& ego,
                                        std::uint32_t first_id) const;

  // The kinematic and bounds rules of §3.3 on every non-injected candidate:
  // sets `reject` to the first failing rule. Collision is the planner's job.
  void Filter(const RouteLine& route, std::vector<Candidate>* candidates) const;

  const LatticeOptions& options() const { return options_; }
  const LatticeLimits& limits() const { return limits_; }

 private:
  LatticeOptions options_;
  LatticeLimits limits_;
};

}  // namespace nuway_planning

#endif  // NUWAY_PLANNING_LATTICE_SAMPLER_H_
