# Interfaces: frames, time, topics, messages, configuration

This document is the contract between modules. Change it before changing code.

---

## 1. Coordinate frames

ROS REP-103 throughout: right-handed, x forward, y left, z up, yaw counter-clockwise positive.

| Frame | Parent | Published by | Rate | Meaning |
|-------|--------|--------------|------|---------|
| `map` | — | — | — | Global frame. Identical to CARLA world frame after handedness conversion. Prior maps, lane graph, routes live here. |
| `odom` | `map` | localization (`gt_pose_node` or `smoother_node`) | 10 Hz | Drift-corrected frame. `map→odom` jumps are allowed. |
| `base_link` | `odom` | localization (`gt_pose_node` or `pose_extrapolator_node`) | 100 Hz | Ego rear-axle center, on the ground plane (z = 0 at road). |
| `lidar_top`, `cam_*`, `imu`, `gnss` | `base_link` | `world_manager` (static TF) | latched | Sensor extrinsics from `configs/sensors/*.json`. |

**CARLA handedness.** CARLA (UE4) is left-handed: x forward, y right, z up, yaw clockwise. Conversion (done only in `nuway_common/carla_conv.hpp` and `nuway_carla_bridge`):
```
x_ros = x_c;  y_ros = -y_c;  z_ros = z_c
roll_ros = roll_c;  pitch_ros = -pitch_c;  yaw_ros = -yaw_c   (degrees→radians)
```
Whether the CARLA native ROS 2 sensor topics already apply this conversion **must be verified empirically in M0** and the result recorded here:

> M0 finding: _(fill in: "native topics are ROS-convention" / "native topics are CARLA-convention; conversion applied in X")_

**Ego origin.** `base_link` is at the rear axle projected to ground, not CARLA's actor origin (vehicle bbox center). The offset is in `configs/vehicle/<vehicle>.yaml` (`rear_axle_offset_x`, `bbox_center_z`).

**BEV / occupancy grid frame.** Grids are expressed in `base_link` at the message stamp. Cell (row `i`, col `j`) center in `base_link`:
```
x = x_min + (i + 0.5) * resolution
y = y_min + (j + 0.5) * resolution
```
Rows index x (forward), columns index y (left). Default: `resolution = 0.5`, `x_min = -50`, `y_min = -50`, `H = W = 200`. All producers and consumers use `nuway_common/occupancy.hpp::GridSpec` / `nuway_ml/common/occupancy.py::GridSpec`.

## 2. Time

- `use_sim_time: true` on every node. `/clock` is published by `world_manager` after each `world.tick()`.
- `fixed_delta_seconds = 0.05`. Sensor stamps come from CARLA's snapshot timestamp.
- Message `header.stamp` is the time of the *observation* the message is about, not publish time.
- Planning outputs carry `header.stamp` = the ego state time they were planned from; trajectory points carry `t` relative to that stamp.

## 3. Topics

Prefix everything with `/nuway/` except CARLA-native topics.

### 3.1 CARLA-native (produced by CARLA server with `--ros2`)
| Topic | Type |
|-------|------|
| `/carla/hero/lidar_top` | `sensor_msgs/PointCloud2` |
| `/carla/hero/cam_front`, `cam_left`, `cam_right`, `cam_rear` | `sensor_msgs/Image` (+ `/camera_info`) |
| `/carla/hero/imu` | `sensor_msgs/Imu` |
| `/carla/hero/gnss` | `sensor_msgs/NavSatFix` |
| `/carla/hero/vehicle_control_cmd` (subscribed by CARLA) | `carla_msgs/CarlaEgoVehicleControl` |
| `/carla/hero/status` (Leaderboard handshake) | `std_msgs/Bool` |

`carla_msgs` is vendored into `ros2_ws/src/carla_msgs` from the CARLA `ros-carla-msgs` repo (leaderboard-2.0 branch) — the only third-party ROS package, and only for message definitions.

### 3.2 Simulation / ground truth
| Topic | Type | Producer | Notes |
|-------|------|----------|-------|
| `/clock` | `rosgraph_msgs/Clock` | world_manager | |
| `/nuway/gt/ego_odom` | `nav_msgs/Odometry` | gt_publisher | `map`→`base_link`, 20 Hz |
| `/nuway/gt/agents` | `nuway_msgs/AgentArray` | gt_publisher | all actors within 100 m, with visibility flags |
| `/nuway/gt/traffic_lights` | `nuway_msgs/TrafficLightArray` | gt_publisher | |
| `/nuway/sim/vehicle_state` | `nuway_msgs/VehicleState` | gt_publisher | speed, accel, steering angle, gear, from CARLA (allowed as "CAN bus" signals even in no-GT mode) |

### 3.3 Map & route
| Topic | Type | Producer | Notes |
|-------|------|----------|-------|
| `/nuway/map/lane_graph` | `nuway_msgs/LaneGraph` | map_server | transient_local (latched) |
| `/nuway/route/plan` | `nuway_msgs/Route` | route_planner | transient_local; republished on reroute |
| `/nuway/route/reference_line` | `nuway_msgs/ReferenceLine` | route_planner | 0.5 m spacing, curvature, speed limit, 1 Hz + on change |
| `/nuway/route/goal` (sub) | `geometry_msgs/PoseStamped` | eval harness / user | |

### 3.4 Localization
| Topic | Type | Producer |
|-------|------|----------|
| `/tf`, `/tf_static` | | see §1 |
| `/nuway/loc/pose` | `nuway_msgs/EgoState` | gt_pose_node **or** pose_extrapolator_node (100 Hz) |
| `/nuway/loc/pose_lowrate` | `nuway_msgs/EgoState` | smoother_node (10 Hz, with covariance) |
| `/nuway/loc/odometry_delta` | `geometry_msgs/TransformStamped` | lidar_odometry_node |

### 3.5 Perception
| Topic | Type | Producer |
|-------|------|----------|
| `/nuway/perception/agents` | `nuway_msgs/AgentArray` | gt_perception_node **or** bevfusion_node (+tracker) |
| `/nuway/perception/occupancy` | `nuway_msgs/OccupancyGridMC` | gt_perception_node **or** bevfusion_node |
| `/nuway/perception/traffic_lights` | `nuway_msgs/TrafficLightArray` | gt_perception_node **or** traffic_light_node |

### 3.6 Prediction
| Topic | Type | Producer |
|-------|------|----------|
| `/nuway/prediction/samples` | `nuway_msgs/PredictionSamples` | const_vel_node **or** flow_matching_node |

### 3.7 Planning
| Topic | Type | Producer |
|-------|------|----------|
| `/nuway/planning/behavior` | `nuway_msgs/BehaviorDecision` | behavior_fsm_node |
| `/nuway/planning/candidates` | `nuway_msgs/TrajectoryCandidates` | planner_node (all candidates with scores, for viz) |
| `/nuway/planning/trajectory` | `nuway_msgs/Trajectory` | planner_node (selected, refined) |
| `/nuway/planning/safe_trajectory` | `nuway_msgs/Trajectory` | safety_layer_node (what control actually follows) |
| `/nuway/planning/learned_candidates` | `nuway_msgs/TrajectoryCandidates` | learned_planner_node (M7) |

### 3.8 Control
| Topic | Type | Producer |
|-------|------|----------|
| `/nuway/control/command` | `nuway_msgs/ControlCommand` | pure_pursuit_pid_node **or** mpc_node |
| `/nuway/control/debug` | `nuway_msgs/ControlDebug` | same |

### 3.9 Diagnostics
| Topic | Type |
|-------|------|
| `/nuway/diag/<node_name>` | `nuway_msgs/NodeDiag` |

## 4. Messages (`nuway_msgs`)

All angles radians, all lengths meters, all times seconds. `float32` for arrays, `float64` for poses.

```
# EgoState.msg
std_msgs/Header header            # frame_id: "map"
geometry_msgs/Pose pose           # base_link in map
float64 vx                        # body-frame longitudinal velocity
float64 vy                        # body-frame lateral velocity
float64 yaw_rate
float64 ax                        # body-frame longitudinal accel
float64 ay
float64 steering_angle            # front wheel angle, rad
float64[36] covariance            # 6x6 pose covariance (row-major), zeros if GT
```
```
# VehicleState.msg   (signals a real car's CAN bus would provide)
std_msgs/Header header
float64 speed
float64 steering_angle
float64 throttle
float64 brake
int8 gear
```
```
# Agent.msg
uint32 id
uint8 class_id                    # 0 unknown, 1 car, 2 truck, 3 bicycle, 4 motorcycle, 5 pedestrian, 6 static_obstacle
float32 score
geometry_msgs/Pose pose           # box center; frame per AgentArray.header.frame_id
float32 length
float32 width
float32 height
float32 vx                        # frame-aligned velocity (same frame as pose)
float32 vy
float32 yaw_rate
bool visible                      # GT only: passed visibility filter
uint8 history_len                 # number of valid entries in history
geometry_msgs/Pose2D[20] history  # past poses at 0.1 s spacing, history[0] = 0.1 s ago, frame = AgentArray frame
```
```
# AgentArray.msg
std_msgs/Header header            # frame_id: "base_link" (perception) or "map" (gt)
Agent[] agents
```
```
# OccupancyGridMC.msg
std_msgs/Header header            # frame_id: "base_link"
float32 resolution
float32 x_min
float32 y_min
uint16 height                     # rows (x)
uint16 width                      # cols (y)
uint8 num_channels
string[] channel_names            # see channel spec below
float32[] data                    # [channel][row][col], row-major, values in [0,1] or meters for height
```
Channel spec (fixed order):
| idx | name | meaning |
|-----|------|---------|
| 0 | `occupied` | P(cell contains a static or unclassified obstacle) |
| 1 | `free` | P(cell observed free) |
| 2 | `unknown` | 1 − occupied − free (visibility) |
| 3 | `drivable` | P(cell is drivable road surface) |
| 4 | `height_max` | max obstacle height above ground in cell [m], 0 if none |
| 5 | `dynamic` | P(cell occupied by a detected dynamic agent) — informational; planners use agents list instead |

```
# TrafficLight.msg / TrafficLightArray.msg
uint32 id
uint8 state                       # 0 unknown, 1 red, 2 yellow, 3 green, 4 off
geometry_msgs/Point stop_line     # map frame
uint32[] affected_lane_ids
float32 confidence
```
```
# LaneGraph.msg
std_msgs/Header header
Lane[] lanes
TrafficLightMapping[] traffic_lights
StopSign[] stop_signs
Crosswalk[] crosswalks

# Lane.msg
uint32 id
uint32 road_id
int32 lane_id_odr
uint8 type                        # 0 driving, 1 shoulder, 2 parking, 3 bidirectional, 4 other
geometry_msgs/Point[] centerline  # ≥2 points, ≤ 2 m spacing
float32[] width                   # per point
uint32[] successors
uint32[] predecessors
uint32 left_neighbor              # 0 = none
uint32 right_neighbor
bool left_change_allowed
bool right_change_allowed
float32 speed_limit               # m/s, 0 if unknown
```
```
# Route.msg
std_msgs/Header header
uint32[] lane_ids                 # ordered
geometry_msgs/Pose goal

# ReferenceLine.msg
std_msgs/Header header            # map frame
float32[] s                       # arc length, 0.5 m spacing
geometry_msgs/Point[] points
float32[] heading
float32[] curvature
float32[] speed_limit
float32[] left_bound              # lateral distance to drivable edge, positive
float32[] right_bound
uint32[] lane_id                  # lane at each point
```
```
# TrajectoryPoint.msg
float32 t                         # relative to Trajectory.header.stamp
float32 x
float32 y
float32 yaw
float32 v
float32 a
float32 kappa

# Trajectory.msg
std_msgs/Header header            # frame_id: "map" (all planning in map frame)
TrajectoryPoint[] points          # 0.1 s spacing, horizon 8 s (81 points) unless noted
string source                     # "lattice", "learned", "fallback", ...
uint32 candidate_id

# TrajectoryCandidates.msg
std_msgs/Header header
Trajectory[] candidates
float32[] cost
string[] cost_breakdown_names
float32[] cost_breakdown          # [candidate][name] flattened
int32 selected_index
```
```
# PredictionSamples.msg
std_msgs/Header header            # frame_id: "map"
uint32[] agent_ids                # A agents (ego excluded here; ego samples in M7 go to learned_candidates)
uint8 num_samples                 # S
uint8 num_timesteps               # T
float32 dt                        # 0.5 s
float32[] xy                      # [S][A][T][2]
float32[] yaw                     # [S][A][T]
float32[] sample_weight           # [S], sums to 1
```
```
# BehaviorDecision.msg
std_msgs/Header header
uint8 lateral                     # 0 keep, 1 change_left, 2 change_right
uint32 target_lane_id
uint8 longitudinal                # 0 free, 1 follow, 2 yield, 3 stop
uint32 lead_agent_id              # 0 none
float32 stop_s                    # arc length on reference line, -1 if none
float32 target_speed
string reason
```
```
# ControlCommand.msg
std_msgs/Header header
float32 accel                     # desired longitudinal accel, m/s^2
float32 steering_angle            # desired front wheel angle, rad
bool emergency_stop

# ControlDebug.msg
std_msgs/Header header
float32 lateral_error
float32 heading_error
float32 speed_error
float32 lookahead                 # pure pursuit
float32 solve_time_ms             # mpc
bool solver_ok
```
```
# NodeDiag.msg
std_msgs/Header header
string node
float32 cycle_ms
float32 input_age_ms              # stamp of newest input minus now
uint8 status                      # 0 ok, 1 warn, 2 error
string message
```

## 5. Configuration

`configs/profiles/<name>.yaml` is passed to `stack.launch.py` and merged over package defaults. Keys:

```yaml
profile: m1_classical
carla:
  host: localhost
  port: 2000
  town: Town03
  fixed_delta_seconds: 0.05
  sync: true
  quality: Low
  render_offscreen: true
sensors: configs/sensors/rig_dev.json
vehicle: configs/vehicle/lincoln_mkz_2020.yaml
use_gt:
  localization: true
  perception: true
  traffic_lights: true
  prediction: false          # const-vel is not GT; GT prediction only in offline eval
planning:
  candidate_sources: [lattice]        # + learned (M7)
  refiner: qp                          # qp | ilqr (M9)
  selector: rule                       # rule | forward_sim (M8)
control:
  controller: mpc                      # pure_pursuit | mpc
```

Nodes read their own namespace (e.g. `planner_node:` block). Node parameter files live next to each package as `config/defaults.yaml`.

## 6. GT toggle semantics

Each toggled module has exactly two launch alternatives producing identical topics. The launch file selects based on `use_gt.*`. Consumers never know which producer is running. GT producers must populate every field including `score = 1.0` and realistic `history`.

## 7. Diagnostics & logging

- Every node wraps its callback in `nuway_common::ScopedTimer` and publishes `NodeDiag` every cycle.
- `nuway_eval` records all `/nuway/**` topics plus `/carla/hero/vehicle_control_cmd` to an MCAP bag per route when `record: true`. Sensor topics recorded only if `record_sensors: true`.
- Foxglove layouts in `ros2_ws/src/nuway_viz/foxglove/` show: BEV panel (agents, occupancy, candidates, selected trajectory, prediction samples), map panel, control panel, diag table.
