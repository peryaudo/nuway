// Topic name constants of nuway_viz (docs/02_interfaces.md §3.9).
#ifndef NUWAY_VIZ_NAMES_H_
#define NUWAY_VIZ_NAMES_H_

namespace nuway_viz {

constexpr const char* kNodeName = "marker_node";
constexpr const char* kTopicVizPrefix = "/nuway/viz/";
// The marker layers (`/nuway/viz/<layer>`, MarkerArray). One vocabulary with
// the headless renderer: every name here is a draw_<layer>() in
// ml/nuway_ml/viz/bev_draw.py. The layers whose producers arrive later
// (localization M5, sim_rollout M9, tl_crops M4) are drawn headlessly from
// fixture messages until then.
constexpr const char* kLayerLanes = "lanes";
constexpr const char* kLayerReferenceLine = "reference_line";
constexpr const char* kLayerAgents = "agents";
constexpr const char* kLayerGtAgents = "gt_agents";
constexpr const char* kLayerPredictions = "predictions";
constexpr const char* kLayerCandidates = "candidates";
constexpr const char* kLayerTrajectory = "trajectory";
constexpr const char* kLayerSafeTrajectory = "safe_trajectory";
constexpr const char* kLayerMpcHorizon = "mpc_horizon";
// The one raster layer (sensor_msgs/Image): the OccupancyGridMC colorized.
constexpr const char* kLayerOccupancy = "occupancy";

}  // namespace nuway_viz

#endif  // NUWAY_VIZ_NAMES_H_
