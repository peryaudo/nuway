# Interfaces: frames, time, topics, messages, configuration

This document is the contract between modules. Change it before changing code.

---

## 1. Coordinate frames

ROS REP-103 throughout: right-handed, x forward, y left, z up, yaw counter-clockwise positive.

| Frame | Parent | Published by | Rate | Meaning |
|-------|--------|--------------|------|---------|
| `map` | — | — | — | Global frame. Identical to CARLA world frame after handedness conversion. Prior maps, lane graph, routes live here. |
| `odom` | `map` | localization (`gt_pose_node` or `smoother_node`) | 10 Hz (every 2nd tick) | Drift-corrected frame. `map→odom` jumps are allowed. |
| `base_link` | `odom` | localization (`gt_pose_node` or `pose_extrapolator_node`) | 20 Hz (every tick) | Ego rear-axle center, on the ground plane (z = 0 at road). |
| `lidar_top`, `cam_*`, `imu`, `gnss` | `base_link` | `world_manager` (static TF) | latched | Sensor extrinsics from `configs/sensors/*.json`. |

**CARLA handedness.** CARLA (UE4) is left-handed: x forward, y right, z up, yaw clockwise. Conversion (implemented only in `nuway_common/carla_conv.hpp` and `nuway_ml/common/carla_conv.py`; `nuway_carla_bridge` and the collectors call them and contain no conversion arithmetic of their own — `01_directory_structure.md` Rules):
```
x_ros = x_c;  y_ros = -y_c;  z_ros = z_c
roll_ros = roll_c;  pitch_ros = -pitch_c;  yaw_ros = -yaw_c   (degrees→radians)
```
Whether the CARLA native ROS 2 sensor topics already apply this conversion was verified empirically on the dev box (CARLA 0.9.16, `--ros2`):

> **M0 finding (2026-09-06): native topics are ROS-convention. CARLA applies the conversion itself; do not apply it again to anything arriving on a `/carla/**` topic.**
>
> Two independent checks, both on a hero steering left (`VehicleControl.steer = -0.5`):
> - **Extrinsics.** A camera spawned at CARLA `(x=+1.0, y=+1.5, z=2.0)` — 1.5 m to the *right* — is published on `/tf` as `hero->cam_asym` with `y = -1.500`. The y-sign flip is already applied.
> - **Rates of turn.** While the CARLA-side yaw *decreased* 3.89°/tick (= 1.358 rad/s clockwise, CARLA convention), `/carla/hero/imu` reported `angular_velocity.z = +1.3553` rad/s — same magnitude, opposite sign, i.e. counter-clockwise-positive as ROS expects. `linear_acceleration` agrees: in a left turn it reads `y = +11.5` (centripetal, pointing left) and `z = +9.72` (gravity).
>
> `carla_conv` is therefore **not** applied to native sensor topics. It is still needed for everything read through the CARLA **Python API** (`tools/collect/`, `gt_publisher`, `sensor_rig`'s own JSON extrinsics), which is in CARLA convention.

**Ego origin.** `base_link` is at the rear axle projected to ground, not CARLA's actor origin (vehicle bbox center). The offset is in `configs/vehicle/<vehicle>.yaml` (`rear_axle_offset_x`, `bbox_center_z`).

**BEV / occupancy grid frame.** Grids are expressed in `base_link` at the message stamp. Cell (row `i`, col `j`) center in `base_link`:
```
x = x_min + (i + 0.5) * resolution
y = y_min + (j + 0.5) * resolution
```
Rows index x (forward), columns index y (left). Default: `resolution = 0.5`, `x_min = -50`, `y_min = -50`, `H = W = 200`. All producers and consumers use `nuway_common/occupancy.hpp::GridSpec` / `nuway_ml/common/occupancy.py::GridSpec`.

## 2. Time

- `use_sim_time: true` on every node. `/clock` is published by `world_manager` after each `world.tick()`. **`world_manager` owns `/clock`.** CARLA's native ROS 2 interface publishes its own `/clock` with identical stamps (§3.1); `world_manager` disables it if the API allows, and otherwise the duplicate is benign (same values from the same snapshot) — the authoritative clock is `world_manager`'s, and under the Leaderboard it is `leaderboard_agent`'s (M1 §3.12).
- `fixed_delta_seconds = 0.05`. Sensor stamps come from CARLA's snapshot timestamp.
- **Rates are multiples of the tick.** 1 tick = 0.05 s. "20 Hz" means one callback per tick, "10 Hz" one per two ticks. CARLA sensors in synchronous mode deliver at most one sample per tick regardless of `sensor_tick`, so the IMU and GNSS are 20 Hz. No node claims or relies on a rate above 20 Hz; estimators that "propagate between updates" propagate to the next tick, not to a faster clock.
- **Lockstep protocol.** `world_manager` ticks, publishes `/clock` and the GT topics, and then blocks until it has received `/nuway/control/command` whose `header.stamp` equals the tick it just published (the controller runs every tick and always stamps the tick it consumed). Only then does it call `world.tick()` again. If no matching command arrives within `lockstep_timeout_s` (default 2 s of wall clock), it raises `NodeDiag` status 2 (`error`) with the missing stamp and ticks anyway — CARLA simply holds the last applied control (`control_adapter` has no repeat-last logic, `M0_bringup.md` §2.3); the evaluation harness marks such a route `non_deterministic` in `results.csv`. Nodes that run every 2nd tick (planner, perception, prediction) are triggered by their inputs, not by timers, so they also run exactly once per two ticks. `realtime_factor` inserts a wall-clock sleep *before* the tick and therefore never changes results.
- Message `header.stamp` is the time of the *observation* the message is about, not publish time.
- Planning outputs carry `header.stamp` = the ego state time they were planned from; trajectory points carry `t` relative to that stamp.

## 3. Topics

Prefix everything with `/nuway/` except CARLA-native topics.

### 3.1 CARLA-native (produced by CARLA server with `--ros2`)
Topic names come from the sensor blueprint's **`ros_name`** attribute (and the hero vehicle's), *not* `role_name`. If `ros_name` is unset CARLA falls back to actor ids — `/carla/actor109/actor110/point_cloud` — which is unusable, so `sensor_rig.py` must set `ros_name` on every sensor it spawns. Verified names (CARLA 0.9.16):

| Topic | Type |
|-------|------|
| `/carla/hero/lidar_top/point_cloud` | `sensor_msgs/PointCloud2` |
| `/carla/hero/<cam>/image` for `cam_front`, `cam_left`, `cam_right`, `cam_rear` | `sensor_msgs/Image` |
| `/carla/hero/<cam>/camera_info` | `sensor_msgs/CameraInfo` — **published, but the intrinsics are wrong; see below** |
| `/carla/hero/imu` | `sensor_msgs/Imu` |
| `/carla/hero/gnss` | `sensor_msgs/NavSatFix` |
| `/carla/hero/vehicle_control_cmd` (subscribed by CARLA) | `carla_msgs/CarlaEgoVehicleControl` |
| `/carla/hero/ackermann_control_cmd` (subscribed by CARLA) | `ackermann_msgs/AckermannDriveStamped` — not used; we command through `vehicle_control_cmd` |
| `/clock` | `rosgraph_msgs/Clock` — CARLA publishes this natively too; `world_manager` stays the owner (§2) and disables or ignores the identical-stamp duplicate |
| `/tf` | `tf2_msgs/TFMessage` — see "Native TF" below |
| `/carla/hero/status` (Leaderboard handshake) | `std_msgs/Bool` — not present in our harness; Leaderboard-only |

Note the **`/point_cloud` and `/image` suffixes**: the sensor's `ros_name` is a namespace, not the topic leaf. Only IMU and GNSS publish directly at the sensor name.

**Rates are exactly one message per tick.** Measured over 1179 ticks in synchronous mode with `fixed_delta_seconds = 0.05` and `rotation_frequency = 20`: LiDAR, IMU, GNSS, `camera_info` and `/clock` all landed at 1.00 msg/tick with stamp deltas of exactly 0.0500 s — so **one full LiDAR sweep per tick, no partial sweeps, no accumulation needed**. `image` came in at 0.99/tick with an occasional 0.1000 s gap: under `best_effort` sensor QoS the camera drops a frame now and then. Nodes must key off the stamp, never assume an unbroken image sequence.

**Broken `camera_info`.** CARLA publishes `camera_info` every tick, and `width`/`height`/`cx`/`cy`/`distortion_model` are correct, but the focal length is garbage: an 800×450 camera at FOV 90 (fx should be `400 / tan(45°) = 400.0`) reports **`fx = fy = -22973.444`** — negative and three orders of magnitude off. `D` is all zeros, which is right for a pinhole. Therefore `sensor_rig.py` **must publish its own `CameraInfo`** computed from the rig JSON's size and FOV, on `/nuway/sensors/<cam>/camera_info`, and no node may subscribe to CARLA's. Re-check on any CARLA upgrade.

**Native TF.** With `ros_publish_tf` (default true) CARLA publishes one `/tf` transform per sensor per tick — measured 4.00/tick for a 4-sensor rig — as `hero -> <ros_frame_id>`, dynamic, not `/tf_static`. These are the sensor extrinsics and they are already ROS-convention (§1). Two consequences: the parent frame is `hero`, not our `base_link` (CARLA's actor origin is the bbox center, ours is the rear axle — §1 "Ego origin"), and it re-sends unchanging extrinsics at 20 Hz. `world_manager` therefore sets `ros_publish_tf=false` on every sensor and publishes the rig as proper latched `/tf_static` against `base_link` instead.

`carla_msgs` is vendored into `ros2_ws/src/carla_msgs` from the CARLA `ros-carla-msgs` repo (leaderboard-2.0 branch) — the only third-party ROS package, and only for message definitions.

**Optional and label-only sensors** (same producer, only present when the profile's rig enables them):
| Topic | Type | Enabled by | Consumers |
|-------|------|------------|-----------|
| `/carla/hero/cam_tl` | `sensor_msgs/Image` (+ `/camera_info`) | `rig_dev.json` entry `cam_tl` (M4, decided by the blind-distance study) | `traffic_light_node` only |
| `/carla/hero/lidar_top_semantic` | `sensor_msgs/PointCloud2` (x, y, z, cos_angle, object_idx, tag) | profiles with `use_gt.perception: true` from M2 on, and the M5 mapping profile | `gt_perception_node`, `map_builder` |

Semantic LiDAR and depth cameras are **never** subscribed by a learned node. In the offline collectors (M2) they are read through the CARLA Python API, not ROS.

### 3.2 Simulation / ground truth
| Topic | Type | Producer | Notes |
|-------|------|----------|-------|
| `/clock` | `rosgraph_msgs/Clock` | world_manager | |
| `/nuway/sim/reset_event` | `nuway_msgs/ResetEvent` | world_manager | published once per `/nuway/sim/reset` call, *before* the first tick of the new episode; see §7 |
| `/nuway/gt/ego_odom` | `nav_msgs/Odometry` | gt_publisher | `map`→`base_link`, 20 Hz |
| `/nuway/gt/agents` | `nuway_msgs/AgentArray` | gt_publisher | all actors within 100 m, with visibility flags |
| `/nuway/gt/traffic_lights` | `nuway_msgs/TrafficLightArray` | gt_publisher | includes `time_in_state` and `yellow_duration` from the CARLA API |
| `/nuway/sim/vehicle_state` | `nuway_msgs/VehicleState` | gt_publisher **or** leaderboard_agent | speed, steering angle, throttle, brake, gear. From the CARLA API in our harness; under the Leaderboard only `speed` comes from the speedometer pseudo-sensor and `steering_angle` is the last commanded angle passed through the steering-lag model (`valid_steering: false`). Allowed as "CAN bus" signals even in no-GT mode. |
| `/nuway/sensors/<cam>/camera_info` | `sensor_msgs/CameraInfo` | sensor_rig | QoS `latched`; computed from the rig JSON's size and FOV. Replaces CARLA's broken native `camera_info` (§3.1), which no node may subscribe to. |

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
| `/nuway/loc/pose` | `nuway_msgs/EgoState` | gt_pose_node **or** pose_extrapolator_node (20 Hz, every tick) |
| `/nuway/loc/pose_lowrate` | `nuway_msgs/EgoState` | smoother_node (10 Hz, with covariance) |
| `/nuway/loc/odometry_delta` | `nuway_msgs/OdometryDelta` | lidar_odometry_node (10 Hz) |

### 3.5 Perception
| Topic | Type | Producer |
|-------|------|----------|
| `/nuway/perception/agents` | `nuway_msgs/AgentArray` | gt_perception_node **or** perception_node (+tracker) |
| `/nuway/perception/occupancy` | `nuway_msgs/OccupancyGridMC` | gt_perception_node **or** perception_node |
| `/nuway/perception/traffic_lights` | `nuway_msgs/TrafficLightArray` | gt_traffic_light_node **or** traffic_light_node |
| `/nuway/perception/tl_debug` | `sensor_msgs/Image` | traffic_light_node, only when `traffic_light_node.debug: true` (M4) |

### 3.6 Prediction
| Topic | Type | Producer |
|-------|------|----------|
| `/nuway/prediction/samples` | `nuway_msgs/PredictionSamples` | const_vel_node **or** prediction_node **or** gt_prediction_node (log replay only, M6) |

### 3.7 Planning
| Topic | Type | Producer |
|-------|------|----------|
| `/nuway/planning/behavior` | `nuway_msgs/BehaviorDecision` | behavior_fsm_node **or** gt_planning_node (M6) |
| `/nuway/planning/candidates` | `nuway_msgs/TrajectoryCandidates` | planner_node **or** gt_planning_node (all candidates with scores, for viz) |
| `/nuway/planning/trajectory` | `nuway_msgs/Trajectory` | planner_node **or** gt_planning_node (selected, refined) |
| `/nuway/planning/safe_trajectory` | `nuway_msgs/Trajectory` | safety_layer_node (what control actually follows) |
| `/nuway/planning/learned_candidates` | `nuway_msgs/TrajectoryCandidates` | prediction_node (M8: the ego planning head lives in the prediction process) |

### 3.8 Control
| Topic | Type | Producer |
|-------|------|----------|
| `/nuway/control/command` | `nuway_msgs/ControlCommand` | pure_pursuit_pid_node **or** mpc_node (every tick; `header.stamp` = the tick consumed, see §2) |
| `/nuway/control/debug` | `nuway_msgs/ControlDebug` | same |

### 3.9 Diagnostics and visualization
| Topic | Type | Producer |
|-------|------|----------|
| `/nuway/diag/<node_name>` | `nuway_msgs/NodeDiag` | every node |
| `/nuway/viz/<layer>` | `visualization_msgs/MarkerArray` | `nuway_viz/marker_node` only; layers: `lanes`, `reference_line`, `agents`, `predictions`, `candidates`, `trajectory`, `mpc_horizon`, `sim_rollout`, `tl_crops` |
| `/nuway/viz/chase_cam` | `sensor_msgs/CompressedImage` | `nuway_carla_bridge/sensor_rig` only when `eval.chase_cam: true` (M1); a third-person view of the hero, never subscribed by any node in the stack (§8.3) |

### 3.10 Services (`nuway_msgs/srv`)
| Service | Type | Server | Notes |
|---------|------|--------|-------|
| `/nuway/sim/reset` | `nuway_msgs/Reset` | world_manager | respawn hero at a pose (or spawn index), clear traffic, publish `ResetEvent`, then tick once |
| `/nuway/sim/set_weather` | `nuway_msgs/SetWeather` | world_manager | preset name |
| `/nuway/map/nearest_lane` | `nuway_msgs/NearestLane` | map_server | for tools and tests; runtime nodes use the `LaneGraph` library directly |

Under the Leaderboard runner (M1) the first two services are served by `leaderboard_agent`, which forwards them to the runner's own episode control; the interface seen by the stack is identical.

### 3.11 QoS profiles

Every publisher and subscription names one of these (`nuway_common/qos.hpp`, `nuway_ml/common/frames.py::QOS`); no implicit defaults. `frames.py::QOS` is plain data (profile name → reliability/durability/depth) so importing it pulls in no `rclpy`; the rclpy `QoSProfile` objects are constructed from it inside the node packages, which keeps `nuway_ml` importable in ROS-free environments (the collectors, `render_bag.py`).

| Profile | Reliability | Durability | History | Used for |
|---------|-------------|------------|---------|----------|
| `sensor` | best_effort | volatile | keep_last 1 | CARLA-native sensor topics (subscriptions must match CARLA's publisher) |
| `stream` | reliable | volatile | keep_last 2 | every per-tick or per-2-tick `/nuway/**` data topic (`loc`, `perception`, `prediction`, `planning`, `control`, `gt`) |
| `latched` | reliable | transient_local | keep_last 1 | `/nuway/map/lane_graph`, `/nuway/route/plan`, `/nuway/route/reference_line`, `/nuway/sensors/*/camera_info`, `/tf_static` |
| `event` | reliable | transient_local | keep_last 10 | `/nuway/sim/reset_event`, `/nuway/route/goal` |
| `diag` | best_effort | volatile | keep_last 1 | `/nuway/diag/**`, `/nuway/viz/**`, `/nuway/perception/tl_debug` |

`/clock` uses the ROS default clock QoS (best_effort, keep_last 1).

## 4. Messages (`nuway_msgs`)

All angles radians, all lengths meters, all times seconds. `float32` for arrays, `float64` for poses.

**Enumerated fields carry message constants.** Every `uint8` field with a fixed meaning declares its values as constants in the `.msg` file (shown inline below). C++ mirrors them as `enum class` with `kCamelCase` enumerators and Python uses the generated constants; no integer literal for one of these values appears in code (`03_style_and_conventions.md` §2.1).

```
# ResetEvent.msg
std_msgs/Header header            # stamp: the sim time the new episode starts at
uint32 episode_id                 # increments per reset
string town
geometry_msgs/Pose start_pose     # hero spawn pose, map frame
```
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
# OdometryDelta.msg               (M5)
std_msgs/Header header            # stamp: LiDAR keyframe k; frame_id: "base_link"
builtin_interfaces/Time prev_stamp # keyframe k-1 the delta is measured from
geometry_msgs/Transform delta     # T_{k-1 -> k}: base_link at k-1 -> base_link at k
float64[36] covariance            # 6x6 (x, y, z, roll, pitch, yaw), from the ICP Hessian (M5 §2.1)
```
```
# VehicleState.msg   (signals a real car's CAN bus would provide)
std_msgs/Header header
float64 speed
float64 steering_angle
bool valid_steering               # false under the Leaderboard (angle is model-propagated from the command)
float64 throttle
float64 brake
int8 gear
```
```
# Agent.msg
uint8 CLASS_UNKNOWN=0
uint8 CLASS_CAR=1
uint8 CLASS_TRUCK=2
uint8 CLASS_BICYCLE=3
uint8 CLASS_MOTORCYCLE=4
uint8 CLASS_PEDESTRIAN=5
uint8 CLASS_STATIC_OBSTACLE=6
uint8 HISTORY_LEN=20
uint32 id
uint8 class_id                    # one of CLASS_*
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
float32[60] history               # [20][3] (x, y, yaw) at 0.1 s spacing, history[0] = 0.1 s ago, frame = AgentArray frame.
                                  # Flat float32 array rather than geometry_msgs/Pose2D (deprecated since Foxy).
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
# TrafficLight.msg
uint8 STATE_UNKNOWN=0
uint8 STATE_RED=1
uint8 STATE_YELLOW=2
uint8 STATE_GREEN=3
uint8 STATE_OFF=4
uint32 id                         # LaneGraph.traffic_lights[].id
uint8 state                       # one of STATE_*
geometry_msgs/Point stop_line     # map frame
uint32[] affected_lane_ids
float32 confidence                # 1.0 for GT
float32 time_in_state             # seconds since the light entered `state` as observed by the producer.
                                  # GT: CARLA get_elapsed_time(). Learned: seconds since this state was first
                                  # confirmed (a lower bound on the true elapsed time). -1 if unknown.
float32 yellow_duration           # GT: CARLA get_yellow_time(). Learned: the configured conservative default
                                  # (3 s). The planner computes remaining yellow = yellow_duration - time_in_state.
bool latched                      # learned only: state is being held by the visibility latch, not observed now

# TrafficLightArray.msg
std_msgs/Header header            # stamp: observation time (GT: tick; learned: cam_front stamp)
TrafficLight[] lights
```
```
# LaneGraph.msg
std_msgs/Header header
Lane[] lanes
TrafficLightMapping[] traffic_lights
StopSign[] stop_signs
Crosswalk[] crosswalks

# TrafficLightMapping.msg           (static; one per OpenDRIVE signal type 1000001)
uint32 id                         # stable per town: hash of (road_id, signal_id), also used by TrafficLight.id
uint32[] affected_lane_ids        # lanes governed by this light, after data/maps/<town>/tl_overrides.yaml
geometry_msgs/Point stop_line     # map frame, on the first affected lane's centerline
float32 heading                   # signal facing direction (OpenDRIVE hOffset resolved to map yaw)
geometry_msgs/Point bulb_position # map frame, from data/maps/<town>/tl_bulbs.json; zero if unknown
bool from_override                # true if the association came from tl_overrides.yaml, not <validity>

# StopSign.msg
uint32 id
uint32[] affected_lane_ids
geometry_msgs/Point stop_line     # map frame
geometry_msgs/Polygon trigger_volume  # map frame, footprint used by the M1 infraction checker

# Crosswalk.msg
uint32 id
geometry_msgs/Polygon footprint   # map frame
uint32[] crossing_lane_ids        # lanes the crosswalk spans

# Lane.msg
uint8 TYPE_DRIVING=0
uint8 TYPE_SHOULDER=1
uint8 TYPE_PARKING=2
uint8 TYPE_BIDIRECTIONAL=3
uint8 TYPE_OTHER=4
uint32 id
uint32 road_id
int32 lane_id_odr
uint8 type                        # one of TYPE_*
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
TrajectoryPoint[] points          # 0.1 s spacing, horizon 8 s (81 points). Every producer, including iLQR (M10), emits 81 points.
string source                     # "lattice", "learned", "fallback", "gt", ...
uint32 candidate_id
int32 sample_index                # M8: index into PredictionSamples of the joint sample this ego trajectory is consistent with
                                  # (Source A); -1 for lattice, learned-head (Source B) and fallback candidates

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
uint32[] agent_ids                # A agents (ego excluded here; ego samples in M8 go to learned_candidates)
uint8 num_samples                 # S
uint8 num_timesteps               # T
float32 dt                        # 0.5 s
float32[] xy                      # [S][A][T][2]
float32[] yaw                     # [S][A][T]
float32[] sample_weight           # [S], sums to 1
```
```
# BehaviorDecision.msg
uint8 LATERAL_KEEP=0
uint8 LATERAL_CHANGE_LEFT=1
uint8 LATERAL_CHANGE_RIGHT=2
uint8 LONGITUDINAL_FREE=0
uint8 LONGITUDINAL_FOLLOW=1
uint8 LONGITUDINAL_YIELD=2
uint8 LONGITUDINAL_STOP=3
std_msgs/Header header
uint8 lateral                     # one of LATERAL_*
uint32 target_lane_id
uint8 longitudinal                # one of LONGITUDINAL_*
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
uint8 STATUS_OK=0
uint8 STATUS_WARN=1
uint8 STATUS_ERROR=2
std_msgs/Header header
string node
float32 cycle_ms
float32 input_age_ms              # stamp of newest input minus now
uint8 status                      # one of STATUS_*
string message
```

Services (`nuway_msgs/srv`):
```
# Reset.srv
geometry_msgs/Pose start_pose     # map frame; ignored if spawn_index >= 0
int32 spawn_index                 # -1 = use start_pose
string town                       # empty = keep current
bool clear_traffic
int64 traffic_seed                # -1 = keep the profile's carla.traffic.seed; the harness sets it per route (M1 §3.9)
---
bool ok
uint32 episode_id
string message

# SetWeather.srv
string preset                     # CARLA WeatherParameters preset name, e.g. "ClearNoon"
---
bool ok

# NearestLane.srv
float64 x
float64 y
float64 yaw
float64 max_dist
---
bool found
uint32 lane_id
float64 s
float64 d
```

## 5. Configuration

`configs/profiles/<name>.yaml` is passed to `stack.launch.py` and merged over package defaults. The profile-level keys are the closed set below; a milestone that needs a new profile-level key adds it here first. Keys marked with a milestone are absent (and default to the value shown) until that milestone lands.

```yaml
profile: m1_classical
carla:
  host: localhost
  port: 2000
  town: Town03
  fixed_delta_seconds: 0.05
  sync: true
  quality: Epic                        # -quality-level=Low segfaults load_world (00_overview.md §4); never Low
  render_offscreen: true
  lockstep_timeout_s: 2.0              # §2; wall-clock wait for the tick's ControlCommand
  realtime_factor: 0.0                 # 0 = as fast as lockstep allows; 1 = wall-clock pace
  traffic:                             # M1
    n_vehicles: 50
    n_walkers: 30
    seed: 0                            # per-route override via Reset.srv traffic_seed (M1 §3.9)
    tm_port: 8000
    hybrid_physics: false
sensors: configs/sensors/rig_dev.json
vehicle: configs/vehicle/lincoln_mkz_2020.yaml
use_gt:
  localization: true
  perception: true
  traffic_lights: true
  prediction: false                    # gt_prediction_node needs a recorded log; true only in replay eval (M6)
  planning: false                      # gt_planning_node = M6 expert as a node (M6)
gt_perception:
  apply_visibility: true               # M2; false = omniscient agents
  source: sensor                       # M8; sensor | geometry — geometry builds occupancy and visibility from the
                                       #   M6 static map + raycast (no semantic LiDAR spawn), for render-free DAgger
perception:
  tl_in_bev_process: true              # M4; traffic_light_node hosted by perception_node
prediction:
  source: const_vel                    # M7; const_vel | learned — selects the non-GT PredictionSamples producer (§6)
  guidance:
    enabled: false                     # M10
planning:
  candidate_sources: [lattice]         # + learned (M8)
  refiner: qp                          # refiner for learned candidates: qp | ilqr (M10)
  lattice_refiner: qp                  # fixed
  selector: rule                       # rule | forward_sim (M9)
  expert_extensions: false             # M6; scenario heuristics, on for the expert, optional for the fallback
  forward_sim:                         # M9
    agent_mode: sample                 # sample | reactive | mix
control:
  controller: mpc                      # pure_pursuit | mpc
eval:
  record: true
  record_sensors: false
  render: incidents                    # M1; off | incidents | full — headless renders, §8
  render_stride: 10                    # ticks between frames when render: full
  incident_window: [40, 20]            # ticks rendered before / after each infraction
  chase_cam: false                     # M1; spawn cam_chase and publish /nuway/viz/chase_cam, §8.3
  scoring: configs/eval/scoring_lb20.yaml
```

Nodes read their own namespace (e.g. `planner_node:` block). Node parameter files live next to each package as `config/defaults.yaml`. `configs/gt_toggles/*.yaml` are fragments containing only a `use_gt:` block that a profile includes.

## 6. GT toggle semantics

There are five toggles: `use_gt.localization`, `use_gt.perception`, `use_gt.traffic_lights`, `use_gt.prediction`, `use_gt.planning`. Each toggled module has exactly two launch alternatives producing identical topics, and **each toggle selects exactly one node pair**:

| toggle | GT node | learned / classical node | topics |
|--------|---------|--------------------------|--------|
| `localization` | `gt_pose_node` | `lidar_odometry_node` + `scan_to_map_node` + `smoother_node` + `pose_extrapolator_node` | `/nuway/loc/*`, TF |
| `perception` | `gt_perception_node` | `perception_node` | `/nuway/perception/agents`, `/nuway/perception/occupancy` |
| `traffic_lights` | `gt_traffic_light_node` | `traffic_light_node` | `/nuway/perception/traffic_lights` |
| `prediction` | `gt_prediction_node` (log replay only) | `const_vel_node` or `prediction_node` (by `prediction.source`, §5) | `/nuway/prediction/samples` |
| `planning` | `gt_planning_node` | `behavior_fsm_node` + `planner_node` | `/nuway/planning/behavior`, `candidates`, `trajectory` |

Consumers never know which producer is running. GT producers must populate every field including `score = 1.0`, `confidence = 1.0` and realistic `history`. Perception and traffic lights are separate toggles served by separate nodes precisely so that any combination is a plain launch choice.

## 7. Diagnostics, logging, reset

- Every node wraps its callback in `nuway_common::ScopedTimer` and publishes `NodeDiag` every cycle.
- **Reset convention.** Every node that keeps state across cycles subscribes to `/nuway/sim/reset_event` (QoS `event`) and, on receipt, drops that state before processing any message stamped at or after `ResetEvent.header.stamp`: tracker tracks and histories, temporal BEV queue and EMA memory, traffic-light latches and vote buffers, smoother graph and initialization, QP/iLQR warm starts, FSM timers and latched yellow decisions, selector consistency memory, MPC warm start, `gt_publisher` history ring buffers. A node that has no cross-cycle state documents that in its header comment. `world_manager` publishes the event before the first tick of the new episode, so the ordering is unambiguous. Nodes ignore data from a previous `episode_id`.
- `nuway_eval` records all `/nuway/**` topics plus `/carla/hero/vehicle_control_cmd` to an MCAP bag per route when `eval.record: true`. Sensor topics recorded only if `eval.record_sensors: true`.
- Foxglove layouts in `ros2_ws/src/nuway_viz/foxglove/` show: BEV panel (agents, occupancy, candidates, selected trajectory, prediction samples), map panel, control panel, diag table. The same layers must also be renderable headlessly to image files; that contract is §8.

---

## 8. Offline visualization (headless render contract)

Foxglove (§3.9, `nuway_viz`) is a live GUI attached to a running stack: it serves a human sitting at the devbox. It cannot be read by CI, by a post-hoc debugging session, or by a coding agent working in this repo. So:

**Every visualization has a headless twin that writes image files under `data/`.** A layer that exists only as a Foxglove panel is incomplete. Renders are PNG and camera frames are JPEG, because those are the formats every consumer — a browser, a notebook, CI, an LLM agent's file reader — can open directly. MP4 is a derived convenience artifact only (`tools/viz/make_video.sh`); nothing in the criteria of any milestone may depend on it.

### 8.1 The renderer

All drawing lives in `ml/nuway_ml/viz/` and is imported by everything that renders: `tools/viz/render_bag.py`, the M2 spot-check notebook, and the training-time `val/viz` frames (`03_style_and_conventions.md` §9.7). One BEV drawing implementation, one style, so a training render and an eval render of the same scene are directly comparable.

- matplotlib with the `Agg` backend. No display, no GPU, no CARLA connection, no ROS node, and no `rclpy` import: bags are decoded with `rosbags` from the message schemas embedded in the MCAP, so `render_bag.py` runs anywhere the repo checks out (`viz` dependency group, `03_style_and_conventions.md` §6.1). `nuway_ml.viz` is never imported by a runtime node, exactly as `hydra` and `wandb` are not.
- Deterministic: the same bag produces byte-identical PNGs. Fixed figure size and DPI, no wall-clock text, no random jitter in colors — a render diff between two runs is therefore meaningful.
- The layer names drawn are exactly the `/nuway/viz/<layer>` names of §3.9. One vocabulary for both back-ends; adding a Foxglove layer without the matching `draw_<layer>()` is a defect.

`tools/viz/render_bag.py --bag <path> [--out <dir>] [--stride N] [--ticks a:b] [--layers ...]` replays one route's MCAP (§7) and writes into the route's directory:

```
data/eval_runs/<run_id>/
├── report.md
├── results.csv
└── <route>_<weather>_<seed>/
    ├── run.mcap
    ├── frames/{tick:06d}.png            # render: full only
    ├── sheets/{first:06d}_{last:06d}.png
    ├── incidents/{tick:06d}_<kind>.png
    └── chase/{tick:06d}.jpg             # eval.chase_cam: true only
```

One frame is the standard panel: a BEV of the ego neighborhood (`lanes`, `reference_line`, `agents`, `predictions`, `candidates`, `trajectory`, `mpc_horizon`, `sim_rollout`) plus a header line — tick, sim time, `episode_id`, behavior state, speed and commanded acceleration/steer, any infraction firing on that tick — and a one-line diag strip with the per-node cycle times of that tick. Everything an infraction post-mortem needs is in the image; no cross-referencing another file.

A route at 20 Hz is 6k–12k ticks, so `eval.render: full` (stride 10 → 2 Hz) is opt-in and `incidents` is the default. **Contact sheets** are always written: 20 frames tiled into one PNG, so a whole route is 30–60 images to skim rather than a thousand, and one `open` shows a minute of driving at a glance.

### 8.2 Incident frames

`report.py` already knows the tick of every infraction, safety-layer intervention, MPC failure and lockstep timeout. With `eval.render: incidents` (the default) it renders `eval.incident_window` ticks around each one — by default 40 before and 20 after, i.e. 2 s of approach and 1 s of aftermath — into `incidents/` plus one contact sheet per incident, and links them from `report.md` by relative path next to the row that reports the infraction. This is what turns "driving score dropped 8 points" into something diagnosable without re-running the route.

### 8.3 Chase camera

With `eval.chase_cam: true` the rig JSON gains a `cam_chase` entry (`sensor.camera.rgb`, 640×360, behind and above the hero) that `sensor_rig.py` spawns and publishes as `sensor_msgs/CompressedImage` on `/nuway/viz/chase_cam` (`diag` QoS, every 5th tick). `nuway_eval/chase_writer.py` is its only subscriber and writes `chase/{tick:06d}.jpg`. No node in the stack may subscribe to it — it is not a sensor, it is a witness.

`-RenderOffScreen` (`00_overview.md` §4) does not prevent this: it suppresses the game window, not sensor rendering. The cost is one extra 640×360 render per 5 ticks, which is why it is off by default and why `record_sensors` and `chase_cam` are separate keys. Under the Leaderboard runner (M1 §3.12) it is unavailable — the agent's sensor list is fixed by the runner's limits — and `leaderboard.yaml` must set it `false`.

### 8.4 Training renders

Training-time visualizations follow the same rule from the other direction: `run_logger.log_images()` writes PNGs into the Hydra run directory's `viz/` **and** logs them to W&B, so a run with `logging.wandb.mode: disabled` still leaves the fixed validation scenes on disk (`03_style_and_conventions.md` §9.7). W&B is where a human compares runs; `data/checkpoints/<experiment>/<timestamp>/viz/` is where anything without a browser looks.
