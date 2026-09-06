# M0 — Bring-up

**Goal:** a route is driven end-to-end in an empty town using CARLA's native ROS 2 interface, our own map/route stack, a system-identified vehicle model, and a pure-pursuit + PID controller. All perception and localization are ground truth.

**Completion criteria**
- [ ] `ros2 launch nuway_bringup stack.launch.py profile:=m0_gt_all` completes 10 routes (Town01, Town03, Town05, ≥ 1.5 km each) with no traffic.
- [ ] Mean lateral error < 0.3 m, max < 0.8 m, on straight and curved segments; no route deviation.
- [ ] `configs/vehicle/lincoln_mkz_2020.yaml` contains fitted longitudinal and steering models with reported fit residuals.
- [ ] `02_interfaces.md` §1 "M0 finding" is filled in.
- [ ] Unit tests: `nuway_common`, `nuway_map`, `nuway_route`, `nuway_control` green; `test_geometry_parity.py` green. Integration test `tests/integration/test_m0_route.py` green.
- [ ] Lockstep verified: two runs of the same route produce bit-identical `/nuway/gt/ego_odom` sequences.

---

## 1. Scope

In: CARLA connection, world manager, sensor rig spawning, GT publishers, static TF, OpenDRIVE parsing, lane graph, route A*, reference line, system identification, pure pursuit + PID, control adapter, launch infrastructure, diag infrastructure, Foxglove layout v0.

Out: any planning beyond "follow the reference line at the speed limit", any traffic, MPC (M1), evaluation harness (M1; M0 uses a minimal route runner script).

## 2. Components

### 2.1 `nuway_carla_bridge/world_manager.py` (rclpy)

Responsibilities:
1. Connect to CARLA, load the town from config (`world = client.load_world(town)` only if different from current).
2. Set synchronous mode + `fixed_delta_seconds`. Set Traffic Manager to synchronous mode too (`tm.set_synchronous_mode(True)`), even with no traffic, so later milestones don't change behavior.
3. Spawn the hero vehicle (`vehicle.lincoln.mkz_2020`, `role_name=hero`) at the route start pose (received via `/nuway/route/goal` handshake or a launch parameter `spawn_index`).
4. Spawn sensors from `configs/sensors/rig_dev.json` with `ros_name` attributes set and `sensor.enable_for_ros()` called, so the native ROS 2 interface publishes them. Publish `/tf_static` for each sensor from the JSON extrinsics (ROS convention).
5. Own the tick loop in **lockstep** (`02_interfaces.md` §2): `world.tick()`, publish `/clock` from `snapshot.timestamp.elapsed_seconds`, run `gt_publisher`, then block until `/nuway/control/command` with `header.stamp == this tick` arrives (or `lockstep_timeout_s` elapses, which is logged as a diag error and, in eval, marks the route non-deterministic). Only then sleep for `realtime_factor` pacing and tick again. `realtime_factor = 0` means no sleep. The subscription to the command is for the gate only; `control_adapter` does the conversion.
6. Expose services `/nuway/sim/reset` and `/nuway/sim/set_weather` (`02_interfaces.md` §3.10). `reset` respawns the hero, clears traffic, publishes `/nuway/sim/reset_event`, then performs one tick.
7. On shutdown, destroy all spawned actors.

`configs/sensors/rig_dev.json` schema:
```json
{
  "vehicle": "vehicle.lincoln.mkz_2020",
  "sensors": [
    {"id": "lidar_top", "type": "sensor.lidar.ray_cast",
     "x": 0.0, "y": 0.0, "z": 2.4, "roll": 0, "pitch": 0, "yaw": 0,
     "attributes": {"channels": 32, "range": 75, "points_per_second": 600000,
                    "rotation_frequency": 20, "upper_fov": 10, "lower_fov": -30,
                    "dropoff_general_rate": 0.0, "dropoff_intensity_limit": 1.0,
                    "dropoff_zero_intensity": 0.0, "noise_stddev": 0.0}},
    {"id": "cam_front", "type": "sensor.camera.rgb", "x": 1.5, "y": 0, "z": 2.0, "yaw": 0,
     "attributes": {"image_size_x": 704, "image_size_y": 256, "fov": 90}},
    {"id": "cam_left",  "type": "sensor.camera.rgb", "x": 0.5, "y": -0.9, "z": 2.0, "yaw": -90, "attributes": {...}},
    {"id": "cam_right", "type": "sensor.camera.rgb", "x": 0.5, "y":  0.9, "z": 2.0, "yaw":  90, "attributes": {...}},
    {"id": "cam_rear",  "type": "sensor.camera.rgb", "x": -1.5, "y": 0, "z": 2.0, "yaw": 180, "attributes": {...}},
    {"id": "imu",  "type": "sensor.other.imu",  "x": 0, "y": 0, "z": 0, "attributes": {"sensor_tick": 0.05}},
    {"id": "gnss", "type": "sensor.other.gnss", "x": 0, "y": 0, "z": 0, "attributes": {"sensor_tick": 0.05}},
    {"id": "cam_chase", "type": "sensor.camera.rgb", "viz_only": true,
     "x": -6.0, "y": 0, "z": 3.0, "pitch": -15, "yaw": 0,
     "attributes": {"image_size_x": 640, "image_size_y": 360, "fov": 90, "sensor_tick": 0.25}},
    {"id": "lidar_top_semantic", "type": "sensor.lidar.ray_cast_semantic", "label_only": true,
     "x": 0.0, "y": 0.0, "z": 2.4, "roll": 0, "pitch": 0, "yaw": 0,
     "attributes": {"channels": 32, "range": 75, "points_per_second": 600000, "rotation_frequency": 20,
                    "upper_fov": 10, "lower_fov": -30}}
  ]
}
```
Positions in this JSON are **CARLA convention relative to the CARLA actor origin** (that is what `carla.Transform` wants). `sensor_rig.py` converts to ROS `base_link` (rear-axle) through `nuway_ml.common.carla_conv` when publishing `/tf_static`. Note y-sign flip. `rotation_frequency` must equal the sim rate (20) so one LiDAR sweep = one tick (no partial sweeps). `sensor_tick` is 0.05 for every sensor: in synchronous mode CARLA delivers at most one sample per tick, so a smaller value only wastes render time. Entries with `label_only: true` are spawned only when the profile asks for them (`use_gt.perception: true` from M2 on, and the M5 mapping profile) and are never subscribed by a learned node. `viz_only: true` is the same idea for visualization: `cam_chase` (M1) is spawned only when `eval.chase_cam: true`, publishes `sensor_msgs/CompressedImage` on `/nuway/viz/chase_cam` rather than under `/carla/hero/`, is excluded from `/tf_static` and from `rig_leaderboard.json`, and no node in the stack may subscribe to it (`02_interfaces.md` §8.3). It is the one sensor whose `sensor_tick` is not 0.05.

### 2.2 `nuway_carla_bridge/gt_publisher.py` (rclpy)

Subscribed to nothing; runs after each tick (called by `world_manager` in the same process — a single CARLA client per process; do **not** open two clients that both tick).

Publishes:
- `/nuway/gt/ego_odom`: hero transform → ROS, rear-axle correction, velocity from `actor.get_velocity()` rotated into body frame.
- `/nuway/sim/vehicle_state`: `get_control()` and `get_physics_control()` derived speed/steer. Steering angle = `control.steer * max_steer_angle` where `max_steer_angle` is read from `physics_control.wheels[0].max_steer_angle` (degrees → rad).
- `/nuway/gt/agents`: every `vehicle.*`, `walker.*`, and `static.prop.*` actor within 100 m. Fields per `Agent.msg`. `visible` is set by the visibility filter (M2; in M0 always `true`). `history` is maintained by a per-actor ring buffer inside the publisher (0.1 s spacing → sample every 2 ticks); the buffer is cleared on `/nuway/sim/reset`.
- `/nuway/gt/traffic_lights`: all `traffic.traffic_light` actors with state, `time_in_state = tl.get_elapsed_time()`, `yellow_duration = tl.get_yellow_time()`, `confidence = 1.0`, stop line from `tl.get_stop_waypoints()`, and `affected_lane_ids` from `tl.get_affected_lane_waypoints()`. The publisher subscribes to the latched `/nuway/map/lane_graph` and resolves `(road_id, lane_id_odr, s)` to our lane ids itself; `TrafficLight.id` is the `TrafficLightMapping.id` whose stop line is nearest (< 2 m), else a diag warning and the actor id with the high bit set.

Class mapping: `vehicle.*` with `number_of_wheels==2` → bicycle/motorcycle by blueprint id; `walker.*` → pedestrian; trucks/buses by blueprint id list in config; else car. `static.prop.*` → static_obstacle.

### 2.3 `nuway_carla_bridge/control_adapter.py` (rclpy)

Subscribes `/nuway/control/command`, publishes `/carla/hero/vehicle_control_cmd`. Uses `nuway_control::LongitudinalMap` logic (pure-Python copy in `nuway_ml/common/longitudinal_map.py` with a parity test) to convert `accel` → throttle/brake using the sysid table, and `steering_angle / max_steer_angle` → `steer`. `emergency_stop` → brake=1, throttle=0. Converts every received command immediately; the lockstep gate in `world_manager` guarantees exactly one command per tick, so there is no repeat-last logic. A watchdog remains for the non-lockstep Leaderboard case (M1): if command age > 0.5 s, brake.

Rationale for separate adapter node: keeps the CARLA-specific control message out of `nuway_control`, and matches the Leaderboard ROS interface which wants `CarlaEgoVehicleControl`.

### 2.4 `nuway_map` (C++)

**`OpenDriveParser`**: input OpenDRIVE XML string (from `world.get_map().to_opendrive()`, saved by `world_manager` to `data/maps/<town>.xodr` once and loaded by the map server thereafter). Parse:
- `road` → `planView` geometries (line, arc, spiral, poly3, paramPoly3) → sample the reference line at 0.5 m.
- `lanes/laneSection/left|right/lane` with `width` records (cubic polynomials in `s`) → lane centerlines by offsetting the road reference line by cumulative widths.
- `link` predecessor/successor for roads and lanes; `junction` connections.
- `objects`/`signals` for stop lines and traffic light signals (CARLA encodes traffic lights as signals with `type="1000001"`; stop signs `type="206"`).
- Lane `type`; only `driving`, `bidirectional`, `parking`, `shoulder` kept.

Use `pugixml`. Implement spiral via Fresnel integral series (or copy the odrSpiral algorithm). Test against CARLA's own waypoint API: sample `world.get_map().generate_waypoints(2.0)` for Town03 into a fixture JSON; parser centerlines must be within 0.15 m of the nearest CARLA waypoint of the same `(road_id, lane_id)`.

**`LaneGraph`**: assigns a `uint32` id per `(road_id, section_idx, lane_id_odr)`, computes successors/predecessors (through junction connections), left/right neighbors (adjacent lane ids with same direction), change-allowed flags from `roadMark` type (`broken`/`solid`), and speed limit from `<speed>` records (fallback: config default 8.33 m/s). Also fills `TrafficLightMapping`, `StopSign` and `Crosswalk` from the `signals`/`objects` records (`02_interfaces.md` §4). Provides (Google-style names, `03_style_and_conventions.md` §2.1):
- `NearestLane(x, y, yaw, max_dist)` → `std::optional<LaneQuery>` (lane id + `s` + lateral offset; KD-tree over centerline samples, heading-consistency check).
- `lane(id)` (accessor), `Successors(id)`, `Neighbors(id)`.
- `TrafficLightsForLane(id)`, `StopLineForLane(id)`.

**`map_server_node`**: loads `.xodr`, builds graph, publishes `/nuway/map/lane_graph` (QoS `latched`). Service `/nuway/map/nearest_lane` for tools.

### 2.5 `nuway_route` (C++)

**`RoutePlanner`**: A* over lane graph. Node = lane id; edge cost = lane length (+ `lane_change_penalty` = 20 m equivalent for lateral neighbor edges). Start = `NearestLane(ego)`, goal = `NearestLane(goal_pose)`. Output ordered lane ids.

**`ReferenceLineBuilder`**: concatenates lane centerlines along the route; at lane-change edges, blends laterally over `blend_length` (30 m) with a quintic; resamples at 0.5 m; computes heading (finite difference) and curvature (Menger / three-point); smooths curvature with a 5-point moving average; fills `left_bound/right_bound` from lane widths and neighbor lanes (drivable extent, not lane extent); fills `speed_limit`. Extends 50 m beyond the goal by continuing the last lane (so planners don't run out of line).

**`route_planner_node`**: subscribes lane graph, `/nuway/loc/pose`, `/nuway/route/goal`. Publishes route and reference line. Reroute when ego is > 5 m laterally or > 30° heading-off from the reference line for 2 s (rare in M0; needed later).

Also implements the Leaderboard route format loader (`nuway_eval/routes/*.xml`: list of waypoints in map frame) → converts to a sequence of goals, publishing the next goal when within 20 m of the current one.

### 2.6 System identification (`tools/sysid/`)

Run in an empty straight stretch (Town04 highway, or Town06). All runs in synchronous mode with the world_manager, logging `/nuway/gt/ego_odom`, `/nuway/sim/vehicle_state`, and the command sent.

**Longitudinal**
- `run_sweeps.py --mode throttle`: for throttle ∈ {0.1, 0.2, …, 1.0}, from standstill hold for 12 s; record `(v, a)` per tick. Repeat from initial speeds {5, 10, 15, 20} m/s.
- `--mode brake`: from {5, 10, 20, 30} m/s apply brake ∈ {0.1, …, 1.0} until stopped.
- `--mode coast`: throttle = brake = 0 from {10, 20, 30} m/s.
- `fit_models.py`: fit `a = f(v, throttle)` as a 2-D lookup table (v bins 1 m/s, throttle bins 0.1) with linear interpolation, and `a = g(v, brake)` similarly, plus coast drag `a_coast(v)`. Produce the inverse map `u = h(v, a_des)` by 1-D root finding per v bin. Also fit a first-order actuator lag `τ_throttle` by step response (time to 63% of steady-state accel). Write all to `configs/vehicle/lincoln_mkz_2020.yaml` and residual plots to `data/sysid/`.

**Lateral**
- `--mode steer`: at v ∈ {5, 10, 15} m/s apply steer ∈ {±0.1, ±0.2, ±0.4} for 4 s; measure yaw rate steady state.
- Fit wheelbase `L` from `yaw_rate = v · tan(δ) / L` given `δ = steer · max_steer_angle`; compare to geometric wheelbase from `physics_control.wheels` positions. Fit steering actuator lag `τ_steer` from step response. Record `max_steer_angle`, `L`, `τ_steer`, and (for M1 MPC) an estimated understeer gradient from residuals at 15 m/s.

Vehicle YAML:
```yaml
name: lincoln_mkz_2020
wheelbase: 2.85
rear_axle_offset_x: -1.4          # base_link relative to CARLA actor origin (ROS x)
bbox_center_z: 0.75
max_steer_angle: 1.22             # rad, from physics_control
tau_steer: 0.12
tau_throttle: 0.20
limits: {a_max: 3.0, a_min: -6.0, jerk_max: 5.0, steer_rate_max: 0.8, kappa_max: 0.18}
longitudinal_map:
  v_bins: [...]
  throttle_bins: [...]
  accel_table: [[...]]            # a[v][throttle]
  brake_bins: [...]
  decel_table: [[...]]
  coast_accel: [...]
```

### 2.7 `nuway_control/pure_pursuit_pid_node` (C++)

- Lateral: pure pursuit on the reference line. Lookahead `L_d = clamp(k_v · v + L_0, 3, 20)` with `k_v=0.6, L_0=2.0`. Target point = reference line point at arc length `s_ego + L_d`. `δ = atan(2 L sin(α) / L_d)`. Clamp to `max_steer_angle`. Rate-limit by `steer_rate_max`.
- Longitudinal: target speed = `min(speed_limit(s), v_curvature(s..s+30))` where `v_curvature = sqrt(a_lat_max / |κ|)`, `a_lat_max = 2.0`. PID on speed error → `accel`, gains in config, anti-windup, output clamped to limits. Feed-forward drag from `coast_accel(v)`.
- Runs once per tick, triggered by `/nuway/loc/pose`; publishes `ControlCommand` stamped with that tick (the lockstep gate depends on this) and `ControlDebug`.

This controller stays in the repo permanently as the simplest possible fallback and as a sanity-check tool.

### 2.8 `nuway_localization/gt_pose_node` (C++)

Subscribes `/nuway/gt/ego_odom` (every tick). Publishes `/nuway/loc/pose` every tick (same rate and message layout as the M5 `pose_extrapolator_node`, so downstream never notices the swap), `map→odom` (identity) and `odom→base_link` TF. Optional `noise:` params (translation σ, yaw σ, latency in ticks) to stress downstream before M5 exists.

### 2.9 `nuway_perception/gt_perception_node.py` and `gt_traffic_light_node.py` (rclpy) — minimal in M0

Both are Python, against the C++-runtime default, per `00_overview.md` §2.6: they are cheat twins, and the grid logic they will grow in M2 must stay single-sourced with `nuway_ml/data/gt_occupancy.py`. They import `nuway_ml.common` (`occupancy.GridSpec`, `geometry`) the way `sensor_rig.py` already imports `nuway_ml.common.carla_conv`.

**Neither may pull in torch.** The GT-only stack (M0–M2) must run on a machine with no `ml/` runtime deps installed beyond numpy, and an M0 profile that fails to launch without CUDA would be a bad trade for a cheat twin. Concretely: `GridSpec` and the world↔grid arithmetic in `nuway_ml/common/occupancy.py` are numpy and stay numpy; `bilinear_sample()` is the only torch function there and imports torch inside the function body, not at module scope. Same rule for `gt_occupancy.py` (M2 §3.3). A test imports both modules in an env without torch and asserts success.

Lockstep makes the runtime cost bounded: `world_manager` blocks until the tick's `ControlCommand` arrives, so a slower node lengthens wall clock and never changes results (`00_overview.md` §2.5).

`gt_perception_node.py`: republishes `/nuway/gt/agents` transformed into `base_link` as `/nuway/perception/agents`. Occupancy grid: M0 publishes a grid with only `drivable` filled from the lane graph (rasterize lane polygons) and `free = drivable`, others zero. Full GT occupancy comes in M2, from `generate_gt_occupancy` directly.

`gt_traffic_light_node.py`: republishes `/nuway/gt/traffic_lights` as `/nuway/perception/traffic_lights` unchanged. It is a separate node so that `use_gt.perception` and `use_gt.traffic_lights` are independent launch choices (`02_interfaces.md` §6).

### 2.10 Launch & profiles

`stack.launch.py` reads the profile YAML, then includes:
`sim.launch.py` (world_manager + gt_publisher + control_adapter) → `map.launch.py` → `localization.launch.py` (selects by `use_gt.localization`) → `perception.launch.py` (selects by `use_gt.perception` and `use_gt.traffic_lights` independently) → `prediction.launch.py` (none in M0) → `planning.launch.py` (none in M0; reference-line follow is inside the controller) → `control.launch.py` → `viz.launch.py`.

Pass `--ros-args --params-file` merged from package defaults + profile. Set `use_sim_time` globally.

### 2.11 Minimal route runner (`tools/eval/run_routes.py`, v0)

Loads a route XML, calls `/nuway/sim/reset` at the start pose, publishes goals, waits for completion (ego within 5 m of final goal) or timeout, records lateral error stats from `/nuway/control/debug`, prints a table. M1 replaces the metrics part with the real harness; keep the CLI.

## 3. Task list

1. [ ] Toolchain and style tooling, exactly as specified in `03_style_and_conventions.md` §6–§7: `setup_env.sh`, root `pyproject.toml` as uv workspace (+ `ml/pyproject.toml` with hatchling, `uv.lock`, `.python-version`), `ros2_ws/colcon_defaults.yaml`, `nuway_cmake` package, `.clang-format`, `.clang-tidy`, `.clangd`, `.pre-commit-config.yaml`, `tools/lint/` (`format_cpp.sh`, `tidy_cpp.sh`, `lint_py.sh`, `merge_compile_commands.py`, header guard check), and the CI jobs (format, tidy, sanitizer, ruff, mypy, pytest). Record every pin in §6.4 (uv, LLVM wheels, ruff, mypy, torch cu12x index). Lands first: every later task's build and lint run through it.
2. [ ] Vendor `carla_msgs` (leaderboard-2.0 branch) and `nanoflann_vendor` into `ros2_ws/src/`. Build.
3. [ ] `nuway_msgs`: all messages and services in `02_interfaces.md`, with the declared constants. Build.
4. [ ] `nuway_common`: `geometry.hpp`, `carla_conv.hpp` (+ tests with known transforms), `frenet.hpp` (+ tests: round trip cartesian→frenet→cartesian on an arc within 1e-6), `trajectory.hpp`, `occupancy.hpp` (GridSpec + bilinear sample + tests), `qos.hpp`, `diag.hpp`, `params.hpp`. Python twins `nuway_ml/common/{geometry,frenet,carla_conv,occupancy}.py` + `tests/integration/test_geometry_parity.py`.
5. [ ] `tools/carla/check_native_ros2.py`: start CARLA with `--ros2`, spawn a vehicle + lidar + camera with `enable_for_ros()`, verify topics appear, print frame conventions by driving forward and comparing `/carla/hero/imu` and native pose vs Python API; confirm one LiDAR sweep per tick and one IMU/GNSS sample per tick. **Fill in `02_interfaces.md` §1.**
6. [ ] `world_manager.py` (lockstep gate, reset event), `sensor_rig.py`, `gt_publisher.py`, `control_adapter.py`. Verify `/clock` never advances without a matching `ControlCommand`; verify static TF in Foxglove.
7. [ ] `nuway_map`: parser (fixture test vs CARLA waypoints for Town03 and Town05), lane graph incl. `TrafficLightMapping`/`StopSign`/`Crosswalk`, map server. Publish the `lanes` marker layer in `nuway_viz`.
8. [ ] `nuway_route`: A*, reference line builder (tests: curvature of a circular lane matches 1/R; bounds are positive; extension beyond goal), node, Leaderboard route loader.
9. [ ] `tools/sysid/`: sweeps and fits. Produce `lincoln_mkz_2020.yaml`. Plot residuals; document residual RMS in this file's Decisions log.
10. [ ] `nuway_control`: `LongitudinalMap` (C++ + Python parity test), pure pursuit + PID node.
11. [ ] `gt_pose_node` (C++), minimal `gt_perception_node.py`, `gt_traffic_light_node.py`; all three handle `reset_event`.
12. [ ] Launch files, profile `m0_gt_all.yaml`, Foxglove layout v0 (map, ego, reference line, lookahead point).
13. [ ] `run_routes.py` v0; 10 routes; record lateral error; iterate gains until criteria met.
14. [ ] `tests/integration/test_m0_route.py`: launches stack on Town03 short route in CI-ish mode (`realtime_factor=0`), asserts completion and lateral error bound. Mark as `slow`.

## 4. Testing notes

- All C++ libraries are tested without ROS (pure gtest). Nodes are tested via integration only.
- Fixture generation scripts go in `tests/fixtures/gen_*.py` and are committed; fixtures themselves (small JSON) are committed too.
- Frame convention test: drive straight +x in CARLA for 2 s; `/nuway/gt/ego_odom` must show +x increase, yaw ≈ 0; turn left (CARLA steer < 0) → yaw increases (positive) in ROS.

## 5. Decisions log

- (2026-09-02) Vehicle: `vehicle.lincoln.mkz_2020` — same as Leaderboard default.
- (2026-09-02) Camera rig: 4 × 704×256, FOV 90. Leaderboard allows more; we stay light for GPU budget.
- (2026-09-02) LiDAR: 32 channels, 600k pts/s at 20 Hz ≈ 30k pts/sweep. Increase to 64 ch only if M3 detection recall on pedestrians is insufficient.
- (2026-09-06) **Cheat twins are Python; `gt_pose_node` is the exception.** `gt_perception_node` and `gt_traffic_light_node` move to rclpy (and `gt_prediction_node` in M6). The deciding argument is not "fixtures should be simple" — these twins are the production path for M0–M4 and the measured baseline for M3/M4/M5 — but **single-sourcing**: the M2 occupancy generator would otherwise be implemented twice and held together by a parity test forever (M2 §3.3). Lockstep means the wall-clock cost cannot change results, and the closed-loop cost is bounded (~1.9k grids per route, occupancy every 2nd tick). `gt_pose_node` stays C++: it runs every tick, its publication is what triggers the controller (§2.7), it must stay swap-identical with the M5 `pose_extrapolator_node`, and it has no duplicated implementation to remove — there is no simplicity to win, only per-tick overhead on the tightest path in the stack.

## 6. Open questions

- Does the native ROS 2 LiDAR topic deliver one full sweep per tick when `rotation_frequency == 1/fixed_delta`? Verify in task 5; if partial sweeps appear, accumulate 2 ticks.
- Does native ROS 2 publish `camera_info`? If not, `sensor_rig.py` must publish it from the FOV/size.
