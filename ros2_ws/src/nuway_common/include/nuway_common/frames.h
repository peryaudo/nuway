// Frame ids and the topic names shared by more than one package
// (docs/02_interfaces.md §1, §3). Package-specific topics live in that
// package's own names header. Mirrored by nuway_ml/common/frames.py.
// Introduced in M0.
#ifndef NUWAY_COMMON_FRAMES_H_
#define NUWAY_COMMON_FRAMES_H_

namespace nuway_common {

constexpr const char* kFrameMap = "map";
constexpr const char* kFrameOdom = "odom";
constexpr const char* kFrameBaseLink = "base_link";
constexpr const char* kFrameLidarTop = "lidar_top";
constexpr const char* kFrameImu = "imu";
constexpr const char* kFrameGnss = "gnss";

constexpr const char* kTopicClock = "/clock";
constexpr const char* kTopicResetEvent = "/nuway/sim/reset_event";
constexpr const char* kTopicTickTimeout = "/nuway/sim/tick_timeout";
constexpr const char* kTopicVehicleState = "/nuway/sim/vehicle_state";
constexpr const char* kTopicGtEgoOdom = "/nuway/gt/ego_odom";
constexpr const char* kTopicGtAgents = "/nuway/gt/agents";
constexpr const char* kTopicGtTrafficLights = "/nuway/gt/traffic_lights";
constexpr const char* kTopicLaneGraph = "/nuway/map/lane_graph";
constexpr const char* kTopicRoutePlan = "/nuway/route/plan";
constexpr const char* kTopicReferenceLine = "/nuway/route/reference_line";
constexpr const char* kTopicRouteWaypoints = "/nuway/route/waypoints";
constexpr const char* kTopicPose = "/nuway/loc/pose";
constexpr const char* kTopicPerceptionAgents = "/nuway/perception/agents";
constexpr const char* kTopicPerceptionOccupancy = "/nuway/perception/occupancy";
constexpr const char* kTopicPerceptionTrafficLights =
    "/nuway/perception/traffic_lights";
constexpr const char* kTopicControlCommand = "/nuway/control/command";
constexpr const char* kTopicControlDebug = "/nuway/control/debug";
constexpr const char* kTopicDiagPrefix = "/nuway/diag/";

}  // namespace nuway_common

#endif  // NUWAY_COMMON_FRAMES_H_
