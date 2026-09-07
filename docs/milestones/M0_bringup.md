# M0 — Bring-up

**Goal:** a route is driven end-to-end in an empty town using CARLA's native ROS 2 interface, our own map/route stack, a system-identified vehicle model, and a pure-pursuit + PID controller. All perception and localization are ground truth.

**Completion criteria**
- [ ] `ros2 launch nuway_bringup stack.launch.py profile:=m0_gt_all` completes 10 routes (Town01, Town03, Town05, ≥ 1.5 km each) with no traffic.
- [ ] Mean lateral error < 0.3 m, max < 0.8 m, on straight and curved segments; no route deviation.
- [ ] `configs/vehicle/lincoln_mkz_2020.yaml` contains fitted longitudinal and steering models with reported fit residuals.
- [x] `02_interfaces.md` §1 "M0 finding" is filled in (2026-09-06, by hand against a live server; task 5 turns the measurement into a script that re-asserts it).
- [ ] Unit tests: `nuway_common`, `nuway_map`, `nuway_route`, `nuway_control` green; `test_geometry_parity.py` green. Integration test `tests/integration/test_m0_route.py` green.
- [ ] Lockstep verified: two runs of the same route produce bit-identical `/nuway/gt/ego_odom` sequences, and no `/nuway/sim/tick_timeout` is published in either run — including at episode start, which the no-input convention (`02_interfaces.md` §2) must cover.

---

## 1. Scope

In: CARLA connection, world manager, sensor rig spawning, GT publishers, static TF, OpenDRIVE parsing, lane graph, route A*, reference line, system identification, pure pursuit + PID, control adapter, launch infrastructure, diag infrastructure, Foxglove layout v0.

Out: any planning beyond "follow the reference line at the speed limit", any traffic, MPC (M1), evaluation harness (M1; M0 uses a minimal route runner script).

## 2. Components

### 2.1 `nuway_carla_bridge/world_manager.py` (rclpy)

The one node that owns the CARLA client. `gt_publisher.py` and `sensor_rig.py` are modules it calls from the same process (`01_directory_structure.md` rules); `control_adapter.py` (§2.3) is the package's other node.

Responsibilities:
1. Connect to CARLA, load the town from config (`world = client.load_world(town)` only if different from current). Save `world.get_map().to_opendrive()` to `data/maps/<town>/map.xodr` if the file is absent, before spawning anything; `map_server_node` waits for that file (§2.4).
2. Set synchronous mode + `fixed_delta_seconds`. Set Traffic Manager to synchronous mode too (`tm.set_synchronous_mode(True)`), even with no traffic, so later milestones don't change behavior.
3. Spawn the hero vehicle (`vehicle.lincoln.mkz_2020`, `role_name=hero`) at launch at the map spawn point `spawn_index` (launch parameter, default 0), so that sensors and topics exist before any route does. Every route then begins with a `/nuway/sim/reset` call from the harness, which moves the hero to the route's start pose (`02_interfaces.md` §3.10).
4. Spawn sensors from `configs/sensors/rig_dev.json` with `ros_name` attributes set and `sensor.enable_for_ros()` called, so the native ROS 2 interface publishes them. **`ros_name` is mandatory, and it is not `role_name`** — without it CARLA names topics by actor id (`/carla/actor109/actor110/point_cloud`). Also set `ros_publish_tf=false` on every sensor: CARLA otherwise re-publishes the extrinsics on `/tf` every tick, parented to `hero` rather than `base_link`. Publish `/tf_static` for each sensor from the JSON extrinsics (ROS convention) and `/nuway/sensors/<cam>/camera_info` from size + FOV; both come from `nuway_ml.common.rig`, the one parser of the rig JSON (`01_directory_structure.md`), so `sensor_rig.py` itself holds no geometry. Note `enable_for_ros()` exists on sensors only, not on `Vehicle`. With `carla.no_rendering: true` (M8) the profile's rig is `rig_none.json` and `world.settings.no_rendering_mode` is set before `load_world`.
5. Own the tick loop in **lockstep** (`02_interfaces.md` §2): `world.tick()`, publish `/clock` from `snapshot.timestamp.elapsed_seconds` (`world_manager` owns `/clock`; CARLA's native interface publishes an identical-stamp duplicate — disable it if the API allows, otherwise it is benign, `02_interfaces.md` §2), run `gt_publisher`, then block until a `/nuway/control/command` whose `TickIndex(header.stamp)` equals this tick arrives. If `lockstep_timeout_s` elapses first (`lockstep_startup_timeout_s` on the first tick of the stack and on the first tick after every reset), publish `/nuway/sim/tick_timeout` for this tick, log a diag error, and tick anyway; in eval that marks the route non-deterministic. Only then sleep for `realtime_factor` pacing and tick again. `realtime_factor = 0` means no sleep. The subscription to the command is for the gate only; `control_adapter` does the conversion. Because every node follows the no-input convention, the first ticks after a reset — before the route is published and planned — are answered by `emergency_stop` commands, not by timeouts.
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
Positions in this JSON are **CARLA convention relative to the CARLA actor origin** (that is what `carla.Transform` wants). `sensor_rig.py` converts to ROS `base_link` (rear-axle) through `nuway_ml.common.carla_conv` when publishing `/tf_static`. Note y-sign flip. `rotation_frequency` must equal the sim rate (20) so one LiDAR sweep = one tick (no partial sweeps). `sensor_tick` is 0.05 for every sensor: in synchronous mode CARLA delivers at most one sample per tick, so a smaller value only wastes render time. Entries with `label_only: true` are spawned only when the profile asks for them (`use_gt.perception: true` from M2 on, and the M5 mapping profile) and are never subscribed by a learned node. `viz_only: true` is the same idea for visualization: `cam_chase` (M1) is spawned only when `eval.chase_cam: true`, is **not** enabled for ROS but read through a Python-API `listen()` callback, JPEG-encoded and published as `sensor_msgs/CompressedImage` on `/nuway/viz/chase_cam` rather than under `/carla/hero/`, is excluded from `/tf_static` and from `rig_leaderboard.json`, and no node in the stack may subscribe to it (`02_interfaces.md` §8.3). It is the one sensor whose `sensor_tick` is not 0.05.

### 2.2 `nuway_carla_bridge/gt_publisher.py` (rclpy)

Subscribed to nothing; runs after each tick (called by `world_manager` in the same process — a single CARLA client per process; do **not** open two clients that both tick).

Publishes:
- `/nuway/gt/ego_odom`: hero transform → ROS, rear-axle correction, velocity from `actor.get_velocity()` rotated into body frame.
- `/nuway/sim/vehicle_state`: `speed` from `get_velocity()`, throttle/brake/gear from `get_control()`, and `steering_angle` = the mean of `vehicle.get_wheel_steer_angle(FL_Wheel)` and `(FR_Wheel)` (degrees → rad) — the *actual* wheel angle after actuator lag, not the command, so `valid_steering: true` in our harness. `max_steer_angle` (from `physics_control.wheels[0].max_steer_angle`) is what `control_adapter` divides by.
- `/nuway/gt/agents`: every `vehicle.*`, `walker.*`, and `static.prop.*` actor within 100 m. Fields per `Agent.msg`. `visible` is set by the visibility filter (M2; in M0 always `true`). `history` is maintained by a per-actor ring buffer inside the publisher (0.1 s spacing → sample every 2 ticks); the buffer is cleared on `/nuway/sim/reset`.
- `/nuway/gt/traffic_lights`: all `traffic.traffic_light` actors with state, `time_in_state = tl.get_elapsed_time()`, `yellow_duration = tl.get_yellow_time()`, `confidence = 1.0`, stop line from `tl.get_stop_waypoints()`, and `affected_lane_ids` from `tl.get_affected_lane_waypoints()`. The publisher subscribes to the latched `/nuway/map/lane_graph` and resolves `(road_id, lane_id_odr, s)` to our lane ids itself; `TrafficLight.id` is the `TrafficLightMapping.id` whose stop line is nearest (< 2 m), else a diag warning and the actor id with the high bit set.

Class mapping: `vehicle.*` with `number_of_wheels==2` → bicycle/motorcycle by blueprint id; `walker.*` → pedestrian; trucks/buses by blueprint id list in config; else car. `static.prop.*` → static_obstacle.

### 2.3 `nuway_carla_bridge/control_adapter.py` (rclpy)

Subscribes `/nuway/control/command`, publishes `/carla/hero/vehicle_control_cmd`. Uses `nuway_control::LongitudinalMap` logic (pure-Python copy in `nuway_ml/common/longitudinal_map.py`, parity-tested through `nuway_py`) to convert `accel` → throttle/brake using the sysid table, and `steering_angle / max_steer_angle` → `steer`. `emergency_stop` → brake=1, throttle=0. Converts every received command immediately; the lockstep gate in `world_manager` guarantees exactly one command per tick, so there is no repeat-last logic and no watchdog. Under the Leaderboard (M1 §3.12) this node does not run: `leaderboard_agent` performs the identical conversion itself through the same Python `LongitudinalMap`, and its own per-tick watchdog is the only place a command can be synthesized.

Rationale for separate adapter node: keeps the CARLA-specific control message out of `nuway_control`, and keeps the conversion in one Python function that the Leaderboard agent can share.

### 2.4 `nuway_map` (C++)

**`OpenDriveParser`**: input OpenDRIVE XML string (from `world.get_map().to_opendrive()`, saved by `world_manager` to `data/maps/<town>/map.xodr` once and loaded by the map server thereafter). Parse:
- `road` → `planView` geometries (line, arc, spiral, poly3, paramPoly3) → sample the reference line at 0.5 m.
- `lanes/laneSection/left|right/lane` with `width` records (cubic polynomials in `s`) → lane centerlines by offsetting the road reference line by cumulative widths.
- `link` predecessor/successor for roads and lanes; `junction` connections.
- `objects`/`signals` for stop lines and traffic light signals (CARLA encodes traffic lights as signals with `type="1000001"`; stop signs `type="206"`).
- Lane `type`; only `driving`, `bidirectional`, `parking`, `shoulder` kept.

Use `pugixml` (`libpugixml-dev` via rosdep, `03_style_and_conventions.md` §6.4). Implement spiral via Fresnel integral series (or copy the odrSpiral algorithm). Test against CARLA's own waypoint API: sample `world.get_map().generate_waypoints(2.0)` for Town03 into a fixture JSON; parser centerlines must be within 0.15 m of the nearest CARLA waypoint of the same `(road_id, lane_id)`.

**`LaneGraph`**: assigns a `uint32` id per `(road_id, section_idx, lane_id_odr)`, computes successors/predecessors (through junction connections), left/right neighbors (adjacent lane ids with same direction), change-allowed flags from `roadMark` type (`broken`/`solid`), and speed limit from `<speed>` records (fallback: config default 8.33 m/s). Also fills `TrafficLightMapping`, `StopSign` and `Crosswalk` from the `signals`/`objects` records (`02_interfaces.md` §4). Provides (Google-style names, `03_style_and_conventions.md` §2.1):
- `NearestLane(x, y, yaw, max_dist)` → `std::optional<LaneQuery>` (lane id + `s` + lateral offset; KD-tree over centerline samples, heading-consistency check).
- `lane(id)` (accessor), `Successors(id)`, `Neighbors(id)`.
- `TrafficLightsForLane(id)`, `StopLineForLane(id)`.

**`map_server_node`**: loads `.xodr`, builds graph, publishes `/nuway/map/lane_graph` (QoS `latched`), including the `<geoReference>` fields (`02_interfaces.md` §4, used by M5). Service `/nuway/map/nearest_lane` for tools. The file is written by whoever owns the CARLA connection — `world_manager` in our harness (§2.1), `leaderboard_agent` on its first step under the runner (M1 §3.12) — and both start concurrently with the map server, so `map_server_node` **polls for the file** (1 s wall-clock interval; this is start-up, before any tick, so it cannot affect results) and publishes the graph when it appears. Loads `configs/maps/<town>/tl_overrides.yaml` when present (M4).

### 2.5 `nuway_route` (C++)

**`RoutePlanner`**: A* over lane graph. Node = lane id; edge cost = lane length (+ `lane_change_penalty` = 20 m equivalent for lateral neighbor edges). Input is the whole `/nuway/route/waypoints` list: A* from `NearestLane(ego)` through `NearestLane(w_i)` for every waypoint in order, concatenated into one ordered lane-id list, planned **once** per episode. Output ordered lane ids; `Route.goal` is the last waypoint.

**`ReferenceLineBuilder`**: concatenates lane centerlines along the route; at lane-change edges, blends laterally over `blend_length` (30 m) with a quintic; resamples at 0.5 m; computes heading (finite difference) and curvature (Menger / three-point); smooths curvature with a 5-point moving average; fills `left_bound/right_bound` from lane widths and neighbor lanes (drivable extent, not lane extent); fills `speed_limit`. Extends 50 m beyond the goal by continuing the last lane (so planners don't run out of line).

**`route_planner_node`**: subscribes lane graph, `/nuway/loc/pose`, `/nuway/route/waypoints`. Publishes route and reference line once per episode, and again only on reroute: when ego is > 5 m laterally or > 30° heading-off from the reference line for 2 s (rare in M0; needed later), it replans from the current lane through the remaining waypoints. The reference line is the whole route, so downstream warm starts and the consistency cost are never disturbed by a goal hand-over.

The Leaderboard route XML (`tools/eval/routes/*.xml`: waypoints in map frame) and the Leaderboard's `global_plan` are loaded by `nuway_ml/common/routes.py` (Python; its two callers, `run_routes.py` and `leaderboard_agent.py`, are Python), which produces the `nav_msgs/Path` payload of `/nuway/route/waypoints`. The C++ route package never reads XML.

### 2.6 System identification (`tools/sysid/`)

Run in an empty straight stretch (Town04 highway, or Town06) with the `sysid.yaml` profile (`rig_none.json`, no route; the sweep script publishes `/nuway/control/command` itself, so it is the "controller" that satisfies the lockstep gate). All runs in synchronous mode with the world_manager, logging `/nuway/gt/ego_odom`, `/nuway/sim/vehicle_state`, and the command sent.

**Longitudinal**
- `run_sweeps.py --mode throttle`: for throttle ∈ {0.1, 0.2, …, 1.0}, from standstill hold for 12 s; record `(v, a)` per tick. Repeat from initial speeds {5, 10, 15, 20} m/s.
- `--mode brake`: from {5, 10, 20, 30} m/s apply brake ∈ {0.1, …, 1.0} until stopped.
- `--mode coast`: throttle = brake = 0 from {10, 20, 30} m/s.
- `fit_models.py`: fit `a = f(v, throttle)` as a 2-D lookup table (v bins 1 m/s, throttle bins 0.1) with linear interpolation, and `a = g(v, brake)` similarly, plus coast drag `a_coast(v)`. Produce the inverse map `u = h(v, a_des)` by 1-D root finding per v bin. Also fit a first-order actuator lag `τ_throttle` by step response (time to 63% of steady-state accel). Write all to `configs/vehicle/lincoln_mkz_2020.yaml` and residual plots to `data/sysid/`.

**Lateral**
- `--mode steer`: at v ∈ {5, 10, 15} m/s apply steer ∈ {±0.1, ±0.2, ±0.4} for 4 s; measure yaw rate steady state.
- Fit wheelbase `L` from `yaw_rate = v · tan(δ) / L` given `δ = steer · max_steer_angle`; compare to geometric wheelbase from `physics_control.wheels` positions. Fit steering actuator lag `τ_steer` from step response. Record `max_steer_angle`, `L`, `τ_steer`, and (for M1 MPC) an estimated understeer gradient from residuals at 15 m/s.
- Record the footprint: `length` and `width` = 2 × `actor.bounding_box.extent.{x,y}`, and the bounding-box centre offset. The collision checker (M1 §3.4), the safety layer and the M9 forward simulator read the ego footprint from this file.

Vehicle YAML:
```yaml
name: lincoln_mkz_2020
wheelbase: 2.85
length: 4.93                      # 2 * bounding_box.extent.x, from the CARLA actor
width: 1.86                       # 2 * bounding_box.extent.y
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
- Runs once per tick, triggered by `/nuway/loc/pose`; publishes `ControlCommand` stamped with that tick (the lockstep gate depends on this) and `ControlDebug`. **No-input convention** (`02_interfaces.md` §2): when the pose is `valid: false` or no reference line has been published for this episode yet, it still publishes, with `emergency_stop: true`, so the first ticks after a reset never starve the gate.

This controller stays in the repo permanently as the simplest possible fallback and as a sanity-check tool.

### 2.8 `nuway_localization/gt_pose_node` (C++)

Subscribes `/nuway/gt/ego_odom` and `/nuway/sim/vehicle_state` (both every tick; `steering_angle` comes from the latter, `ax`/`ay` from the one-tick finite difference of the body-frame velocity, `valid` is always `true`). Publishes `/nuway/loc/pose` every tick (same rate and message layout as the M5 `pose_extrapolator_node`, so downstream never notices the swap), `map→odom` (identity, every 2nd tick — the 10 Hz rate the M5 `smoother_node` publishes at, `02_interfaces.md` §1) and `odom→base_link` TF every tick. Optional `noise:` params (translation σ, yaw σ, latency in ticks) to stress downstream before M5 exists.

### 2.9 `nuway_perception/gt_perception_node.py` and `gt_traffic_light_node.py` (rclpy) — minimal in M0

Both are Python, against the C++-runtime default, per `00_overview.md` §2.6: they are cheat twins, and the grid logic they will grow in M2 must stay single-sourced with `nuway_ml/data/gt_occupancy.py`. They import `nuway_ml.common` (`occupancy.GridSpec`, `geometry`) the way `sensor_rig.py` already imports `nuway_ml.common.carla_conv`.

**Neither may pull in torch.** The GT-only stack (M0–M2) must run on a machine with no `ml/` runtime deps installed beyond numpy, and an M0 profile that fails to launch without CUDA would be a bad trade for a cheat twin. Concretely: `GridSpec` and the world↔grid arithmetic in `nuway_ml/common/occupancy.py` are numpy and stay numpy; `bilinear_sample()` is the only torch function there and imports torch inside the function body, not at module scope. Same rule for `gt_occupancy.py` (M2 §3.3). This is what makes torch an *extra* of `nuway-ml` rather than a base dependency (`03_style_and_conventions.md` §6.1): a plain `uv sync` has no torch, and the `import_light` pytest marker names the tests that CI runs in exactly that environment to assert both modules import.

Lockstep makes the runtime cost bounded: `world_manager` blocks until the tick's `ControlCommand` arrives, so a slower node lengthens wall clock and never changes results (`00_overview.md` §2.5).

`gt_perception_node.py`: republishes `/nuway/gt/agents` transformed into `base_link` as `/nuway/perception/agents`, on even ticks only (`02_interfaces.md` §2). Occupancy grid: M0 publishes a grid with `drivable` filled from the lane graph (rasterize lane polygons), `free = drivable`, `unknown = 1 − drivable` (the channel invariant `unknown = 1 − occupied − free` holds), `occupied`, `height_max` and `dynamic` zero. Full GT occupancy comes in M2, from `generate_gt_occupancy` directly.

`gt_traffic_light_node.py`: republishes `/nuway/gt/traffic_lights` as `/nuway/perception/traffic_lights` unchanged. It is a separate node so that `use_gt.perception` and `use_gt.traffic_lights` are independent launch choices (`02_interfaces.md` §6).

### 2.10 Launch & profiles

`stack.launch.py` reads the profile YAML, then includes:
`sim.launch.py` (the `world_manager` node, which hosts `gt_publisher` and `sensor_rig`, plus the `control_adapter` node) → `map.launch.py` → `localization.launch.py` (selects by `use_gt.localization`) → `perception.launch.py` (selects by `use_gt.perception` and `use_gt.traffic_lights` independently) → `prediction.launch.py` (none in M0) → `planning.launch.py` (none in M0; reference-line follow is inside the controller) → `control.launch.py` → `viz.launch.py`.

Pass `--ros-args --params-file` merged from package defaults + profile. Set `use_sim_time` globally.

### 2.11 Minimal route runner (`tools/eval/run_routes.py`, v0)

Loads a route XML through `nuway_ml/common/routes.py`, calls `/nuway/sim/reset` at the start pose, publishes the whole route once on `/nuway/route/waypoints`, waits for completion (ego within 5 m of the last waypoint) or timeout, records lateral error stats from `/nuway/control/debug`, prints a table. One stack launch per town; routes of the same town are separated by `/nuway/sim/reset` only (sim time stays monotonic, `02_interfaces.md` §2). M1 replaces the metrics part with the real harness; keep the CLI.

## 3. Task list

1. [x] Toolchain and style tooling, exactly as specified in `03_style_and_conventions.md` §6–§7: `setup_env.sh`, root `pyproject.toml` as uv workspace (+ `ml/pyproject.toml` with hatchling, `uv.lock`, `.python-version`), `ros2_ws/colcon_defaults.yaml`, `nuway_cmake` package, `.clang-format`, `.clang-tidy`, `.clangd`, `.pre-commit-config.yaml`, `tools/lint/` (`format_cpp.sh`, `tidy_cpp.sh`, `lint_py.sh`, `merge_compile_commands.py`, header guard check), and the CI jobs (format, tidy, sanitizer, ruff, mypy, pytest). Record every pin in §6.4 (uv, LLVM wheels, ruff, mypy; the torch index is already fixed at cu126). Lands first: every later task's build and lint run through it. Also confirm `.claude/hooks/format-on-edit.sh` starts formatting on edit (it keys on `uv.lock` and `.clang-format` existing and needs no change).
2. [x] Vendor `carla_msgs` (leaderboard-2.0 branch) into `ros2_ws/src/`. nanoflann comes from apt (`libnanoflann-dev`) via rosdep on Jazzy/noble, so it needs no vendor package. Build.
3. [x] `nuway_msgs`: all messages and services in `02_interfaces.md`, with the declared constants. Build.
4. [x] `nuway_common`: `geometry.hpp`, `carla_conv.hpp` (+ tests with known transforms), `frenet.hpp` (+ tests: round trip cartesian→frenet→cartesian on an arc within 1e-6), `trajectory.hpp`, `occupancy.hpp` (GridSpec + bilinear sample + tests), `tick.hpp` (tick index, planning-tick predicate, barrier helper), `qos.hpp`, `diag.hpp`, `params.hpp`. Python twins `nuway_ml/common/{geometry,frenet,carla_conv,occupancy,tick,qos,frames}.py`. The `nuway_py` pybind package (`01_directory_structure.md`) binding `nuway_common` so that `tests/integration/test_geometry_parity.py` can compare both sides; it grows in M6.
5. [ ] `tools/carla/start_carla.sh` (the one place the server flags live; reads `carla.port` and `carla.quality` from a profile, refuses `Low`) and `tools/carla/check_native_ros2.py` (against an already running `--ros2` server, `04_setup.md` §7): spawn a vehicle + lidar + camera with `enable_for_ros()`, verify topics appear, assert `header.frame_id == ros_name` on every native message, re-assert the frame-convention finding of `02_interfaces.md` §1 by driving forward and comparing `/carla/hero/imu` and native pose vs Python API; confirm one LiDAR sweep per tick and one IMU/GNSS sample per tick, and measure the camera drop rate. The §1 finding is already written; this task makes it re-checkable on every CARLA upgrade.
6. [ ] `nuway_ml/common/rig.py` (parse, extrinsics to `base_link`, intrinsics; tests against hand-computed K and a right-mounted sensor), `world_manager.py` (lockstep gate with `TickIndex` comparison, startup timeout per reset, `TickTimeout` publication, reset event, `map.xodr` export, no-rendering option), `sensor_rig.py`, `gt_publisher.py`, `control_adapter.py`. Verify `/clock` never advances without a matching `ControlCommand`; verify static TF in Foxglove.
7. [ ] `nuway_map`: parser (fixture test vs CARLA waypoints for Town03 and Town05), lane graph incl. `TrafficLightMapping`/`StopSign`/`Crosswalk` and the `<geoReference>` fields, map server with the file-wait. Publish the `lanes` marker layer from a minimal `nuway_viz/marker_node` (M1 task 12 completes it).
8. [ ] `nuway_route`: multi-waypoint A*, reference line builder (tests: curvature of a circular lane matches 1/R; bounds are positive; extension beyond goal; a 3-waypoint route yields one continuous line), node with reroute; `nuway_ml/common/routes.py` (route XML → waypoints) + test on a committed route file.
9. [ ] `tools/sysid/`: sweeps and fits. Produce `lincoln_mkz_2020.yaml`. Plot residuals; document residual RMS in this file's Decisions log.
10. [ ] `nuway_control`: `LongitudinalMap` (C++, bound in `nuway_py`; Python twin + parity test), pure pursuit + PID node with the no-input `emergency_stop` output.
11. [ ] `gt_pose_node` (C++, `valid: true`), minimal `gt_perception_node.py`, `gt_traffic_light_node.py`; all three handle `reset_event` and publish their no-input outputs when the pose is invalid.
12. [ ] Launch files, profiles `m0_gt_all.yaml` and `sysid.yaml`, Foxglove layout v0 (map, ego, reference line, lookahead point). This layout is the one knowing exception to the headless-twin rule of `01_directory_structure.md`: `nuway_ml/viz/` and `draw_lanes` / `draw_reference_line` arrive with M1 task 13, which must cover every layer this layout shows.
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
- (2026-09-06) **Environment is Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12**, not 22.04 / Humble / 3.10 as originally specified: the dev box is 24.04 and Humble is not packaged for noble. Verified on the box before the bump — CARLA 0.9.16 publishes a `cp312` client wheel (so the client matches Jazzy's interpreter), the 0.9.16 server binary resolves cleanly against glibc 2.39 and renders offscreen through Vulkan on the RTX 3090 Ti, and every ROS package the stack needs exists for Jazzy/noble (`rmw_cyclonedds_cpp`, `iceoryx`, `rosbag2_storage_mcap`, `foxglove_bridge`, `pcl_ros`, `tf2_ros`). See `00_overview.md` §4 and `03_style_and_conventions.md` §6.
- (2026-09-06) **CARLA runs at default (Epic) quality; `-quality-level=Low` is dropped.** At Low, the first `client.load_world()` segfaults the server in UE4.26's Vulkan RHI (`FVulkanVertexInputStateInfo::Generate` receives a null `VertexDeclaration` from the async pipeline-compile task). Reproducible on every map change, unaffected by `--ros2`, `map_layers` or `r.AsyncPipelineCompile`; `reload_world()` of the *same* map is fine, so it is specific to loading a new map's shaders. CARLA 0.9.16 also ignores the map name on the command line (always boots `Town10HD_Opt`), so `load_world` cannot be avoided. Cost: ~3× slower rendering (see the table in `00_overview.md` §4). This is a wall-clock cost only — the stack is lockstep and nothing is timed by wall-clock. The M6 collection farm is unaffected in practice because it runs `no_rendering_mode` (M6 §1).
- (2026-09-06) **Native ROS 2 verified end-to-end on the dev box** (ROS 2 Jazzy client against CARLA 0.9.16 `--ros2`), which closes `02_interfaces.md` §1 and two open questions. Native topics are **ROS-convention** — confirmed twice, by a right-mounted sensor coming back with negative `y` on `/tf`, and by IMU `angular_velocity.z` having opposite sign to CARLA's yaw rate at equal magnitude. Topic names come from the `ros_name` attribute and carry `/point_cloud` and `/image` suffixes. `/clock` and `/tf` are published by CARLA itself. Everything is exactly 1 msg/tick except the camera, which occasionally drops one under best-effort QoS. `camera_info` is published with a broken focal length and must be replaced by ours. CycloneDDS interoperates with CARLA's embedded FastDDS (both discovery and data), so the §4 rmw choice stands; it logs a benign `Failed to parse type hash ... from USER_DATA '(null)'` warning per topic, because CARLA does not advertise Jazzy type hashes.
- (2026-09-06) **`world_manager` owns `/clock`** even though CARLA publishes one natively with `--ros2`: ownership keeps clock and GT topics ordered (both emitted together right after the tick) and the topology identical under the Leaderboard, where `leaderboard_agent` publishes it. The native duplicate carries identical stamps; disable it if the API allows, otherwise it is benign (`02_interfaces.md` §2).
- (2026-09-06) **No-input convention and `TickTimeout` (design review).** The lockstep gate as first written deadlocked at every episode start: no route → no reference line → no command, and the barrier made the safety layer wait forever for a planner output. Rather than special-casing start-up, every node now publishes on every tick it acts on, with a fixed "nothing to say" message per type (`02_interfaces.md` §2), and `lockstep_startup_timeout_s` applies after every reset. The same review removed every wall-clock timer from the nodes: a fallback (constant-velocity prediction, lattice-only planning, stop profile) engages only when `world_manager` publishes `/nuway/sim/tick_timeout`, so a fallback can never fire on a route that is not flagged. Also from that review: `gt_publisher.py` and `sensor_rig.py` are modules of the `world_manager` node, not nodes; `world_manager` exports `map.xodr` and `map_server` waits for it; the vehicle YAML carries `length`/`width`; `gt_pose_node` reads `vehicle_state` for the steering angle; the sysid runs get their own `sysid.yaml` profile.

## 6. Open questions

- **CARLA-side determinism is assumed, not yet verified.** The bit-identical criteria (this file, M1 §5) require that CARLA's PhysX stepping, the Traffic Manager and the walker controllers are themselves reproducible under synchronous mode with a fixed seed. CARLA documents this as "deterministic" only with `set_synchronous_mode` + `set_random_device_seed` + `hybrid_physics` off, and it is known to break with `no_rendering_mode` walkers and with unseeded blueprint choices. Task 14 measures it for the hero alone; M1 task 10 measures it with traffic. If either fails, the criteria degrade to "identical up to CARLA's own spread", which must then be measured and stated.
- ~~Does the native ROS 2 LiDAR topic deliver one full sweep per tick when `rotation_frequency == 1/fixed_delta`?~~ **Answered 2026-09-06: yes.** Measured 1.00 msg/tick over 1179 ticks, stamp delta exactly 0.0500 s. No partial sweeps, no accumulation needed. (`02_interfaces.md` §3.1)
- ~~Does native ROS 2 publish `camera_info`?~~ **Answered 2026-09-06: it publishes it, but the intrinsics are unusable** — `fx = fy = -22973.444` where 800×450 @ FOV 90 should give 400.0 (`cx`/`cy`/size/`D` are correct). So the original fallback stands for a different reason than expected: `sensor_rig.py` must publish `CameraInfo` from the rig JSON's FOV and size, and no node may subscribe to CARLA's. (`02_interfaces.md` §3.1)
