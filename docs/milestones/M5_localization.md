# M5 — Localization (LiDAR odometry + prior map + fixed-lag factor graph)

**Goal:** replace GT pose with a factor-graph localization pipeline: KISS-ICP-style LiDAR odometry as front-end, an offline-built point-cloud map per town, online scan-to-map registration, and a GTSAM fixed-lag smoother fusing odometry, scan-to-map, IMU preintegration, and GNSS. High-rate `odom→base_link` via IMU/wheel-speed forward propagation. **After this milestone the stack runs with `use_gt.* = false` everywhere.**

**Completion criteria**
- [ ] ATE (translation) < 0.20 m RMS and yaw RMS < 0.5° against GT over ≥ 10 min of driving per town (Town03, Town05, Town10HD), including two loops.
- [ ] Lateral position error at 100 Hz output < 0.15 m p95 (this is what lane keeping needs).
- [ ] `map→odom` corrections are smooth: no jump > 0.3 m / 2° between consecutive 10 Hz updates in steady state.
- [ ] Initialization from GNSS + heading search succeeds within 3 s from a standstill anywhere on the road.
- [ ] Closed-loop: profile `m5_no_gt.yaml` (learned perception + learned traffic lights + learned localization) scores ≥ 90% of the `m4_learned_tl` score on the M1 protocol.
- [ ] Localization node chain CPU time ≤ 20 ms per 10 Hz cycle.

---

## 1. Architecture

```
/carla/hero/lidar_top ─► lidar_odometry_node ──► odometry_delta (T_{k-1→k}, Σ)  ─┐
                                                                                   │
prior map (data/maps/<town>.ply + voxel index)                                     ▼
/carla/hero/lidar_top ─► scan_to_map_node ─► absolute pose factor (T_map_base, Σ) ─► smoother_node ─► /nuway/loc/pose_lowrate (10 Hz)
/carla/hero/imu       ─► IMU preintegration ─────────────────────────────────────┘        │ map→odom TF
/carla/hero/gnss      ─► GNSS prior (weak) ───────────────────────────────────────┘        ▼
/carla/hero/imu + /nuway/sim/vehicle_state ─► pose_extrapolator_node ─► /nuway/loc/pose (100 Hz), odom→base_link TF
```

Two modes: **mapping** (offline tool, builds the prior map) and **localization** (online, above). Both share the odometry front-end.

## 2. Components

### 2.1 LiDAR odometry (`nuway_localization/src/lidar_odometry.cpp`, node wrapper)

KISS-ICP recipe, implemented in-repo (no external KISS-ICP dependency; ~600 lines):
1. Preprocess: transform sweep to `base_link`; drop points with range < 2 m or > 75 m; voxel downsample at 0.5 m (keep one point per voxel); further subsample to ≤ 8k points for registration ("source"), keep the 0.5 m cloud for map insertion.
2. Deskew: CARLA sweeps are rendered instantaneously per tick → **no deskew needed**. Keep a config flag `deskew: false` and a stub for real sensors.
3. Motion prediction: constant velocity: `T_pred = T_{k-1} · (T_{k-2}^{-1} T_{k-1})`.
4. Local map: voxel hash map (voxel 1.0 m, ≤ 20 points/voxel), points within 100 m of the current pose; insert the downsampled scan after registration; remove far voxels.
5. Registration: point-to-point ICP with adaptive threshold (KISS-ICP's `σ` from the model deviation of previous predictions), robust Cauchy kernel, Gauss-Newton on SE(3) with Eigen, max 50 iterations, convergence on `‖Δ‖ < 1e-4`. Nearest neighbors from the voxel hash (search 27 neighboring voxels).
6. Output `T_{k-1→k}` and an information estimate: use the GN Hessian at convergence scaled by residual variance (cap the resulting σ at 0.05 m / 0.2° min, 0.5 m / 2° max).

Tests: synthetic cube-room point clouds with known motion → recovered transform within 1 cm / 0.1°; timing test ≤ 12 ms for 30k input points.

### 2.2 Offline mapping (`tools/mapping/build_map.py` + C++ `map_builder` binary)

1. Drive the town with the M1 stack in GT-localization mode, recording LiDAR + GT pose (GT pose is allowed for mapping — a real system would use survey-grade equipment). Routes: `nuway_eval/routes/mapping_<town>.xml` covering every driving lane at least once.
2. `map_builder`: accumulate sweeps in `map` frame using GT poses, voxel downsample at 0.2 m, remove dynamic points by class: use the semantic LiDAR channel of the recording to drop points on vehicles/walkers. Save `data/maps/<town>/map.ply` (xyz + normal estimated with 10-NN) and a serialized voxel index.
3. Optional: run `pose_graph_refine` (GTSAM pose graph with odometry BetweenFactors from 2.1 + loop closures + GT priors) to validate that our SLAM back-end reconstructs GT within 0.1 m. This exercises the graph-SLAM code path even though the deployed map uses GT poses. Report in `data/maps/<town>/REPORT.md`.

Loop closure for the validation run: Scan Context descriptor (20 rings × 60 sectors, max height per bin), candidate search over descriptor distance < 0.2, verify with ICP inlier ratio > 0.6; robust `BetweenFactor` with Huber (k = 1.0).

### 2.3 Scan-to-map (`scan_to_map.cpp`, node)

- Loads the prior map; builds a voxel hash (1.0 m) with normals.
- Input: current downsampled scan + initial guess (from the smoother's latest pose propagated by the odometry delta).
- Point-to-plane ICP (normals from map), 20 iterations, robust kernel, ≤ 6k points. Outputs `T_map_base` and covariance from the Hessian; rejects the result if inlier ratio < 0.4 or if the pose moved > 2 m / 10° from the initial guess (publishes a diag warn).
- Runs at 10 Hz (every 2nd sweep) to leave CPU for odometry.

### 2.4 Fixed-lag smoother (`smoother.cpp`, node, GTSAM 4.2)

Variables: at each LiDAR keyframe `k` (10 Hz): pose `X_k` (Pose3), velocity `V_k`, IMU bias `B_k` (constant-bias factor between consecutive nodes with small noise).

Factors:
- `BetweenFactor<Pose3>(X_{k-1}, X_k, T_odom, Σ_odom)` from 2.1.
- `PriorFactor<Pose3>(X_k, T_scan2map, Σ_s2m)` from 2.3 (when accepted). Robust (Huber) to survive bad registrations.
- `ImuFactor(X_{k-1}, V_{k-1}, X_k, V_k, B_{k-1})` from `PreintegratedImuMeasurements` over the 200 Hz IMU samples between keyframes. CARLA IMU noise params from the sensor attributes (set explicit `noise_accel_stddev_*`, `noise_gyro_stddev_*` in the rig so the model matches).
- `GPSFactor(X_k, p_gnss, σ=2 m)` from GNSS (converted from lat/lon via CARLA's `map.transform_to_geolocation` inverse — implement the equirectangular inverse in `carla_conv.hpp`; CARLA's GNSS is exact + configured noise).
- Wheel-speed factor: a unary factor on `V_k` body-x component from `/nuway/sim/vehicle_state.speed` (σ = 0.2 m/s), body-y ≈ 0 (σ = 0.3, non-holonomic soft constraint).

`IncrementalFixedLagSmoother` with lag 2.0 s, iSAM2 params relinearize threshold 0.01. Output the latest `X_k` with marginal covariance as `/nuway/loc/pose_lowrate`, and publish `map→odom = X_k · (odom_pose_k)^{-1}` where `odom_pose_k` is the odometry-integrated pose of keyframe `k` (so `odom` is continuous and `map→odom` absorbs corrections).

### 2.5 Pose extrapolator (`pose_extrapolator.cpp`, node, 100 Hz)

Maintains `odom→base_link` by integrating: yaw rate from IMU gyro z, longitudinal speed from `vehicle_state.speed` (or the smoother's velocity estimate when available), with a planar non-holonomic model. Resets its integration origin to the latest odometry keyframe pose when a new `odometry_delta` arrives (so drift between keyframes is bounded to 0.1 s of integration). Publishes `/nuway/loc/pose` (100 Hz) = `map→odom (latest) · odom→base_link (now)`, with velocities/yaw rate/accel from IMU + wheel speed.

This is the only "high-rate estimator" and it is explicitly a model-based propagator, not a filter.

### 2.6 Initialization

On start or on `/nuway/sim/reset`: wait for GNSS → position prior; heading: sample 36 yaw hypotheses, run scan-to-map from each (coarse: 3 iterations, 2k points), take the best inlier ratio, then refine. If the best ratio < 0.5, keep trying with new sweeps; publish diag error after 5 s.

## 3. Evaluation

`tools/eval/eval_localization.py`: replays MCAP (sensor topics + GT) through the localization nodes offline (ROS launch in replay mode) or reads a live run; computes ATE/RPE (evo-style, implemented in-repo), 100 Hz lateral error against GT projected onto the reference line, correction jump stats, init time. Produces plots into `data/eval_runs/<run>/loc/`.

## 4. Task list

1. [ ] `voxel_hash_map.hpp` (+ tests), `icp.hpp` (point-to-point & point-to-plane, robust, SE3 GN; tests vs synthetic).
2. [ ] `lidar_odometry.cpp` + node; test on a recorded MCAP; plot drift vs GT.
3. [ ] Scan Context descriptor + loop detection (+ tests on repeated synthetic scenes).
4. [ ] `map_builder`, `build_map.py`, mapping routes for Town03/05/10HD; `pose_graph_refine` validation; REPORT per town.
5. [ ] `scan_to_map.cpp` + node; rejection logic; timing.
6. [ ] `smoother.cpp` + node (GTSAM); IMU preintegration wiring; GNSS conversion in `carla_conv.hpp` with test against CARLA API on 100 points.
7. [ ] `pose_extrapolator.cpp` + node; TF publishing; parity with `gt_pose_node` output rate/format.
8. [ ] Initialization; test from 20 random spawn points.
9. [ ] `eval_localization.py`; achieve ATE criteria; tune noise models; record in Decisions log.
10. [ ] Profile `m5_no_gt.yaml`; closed-loop eval; report.
11. [ ] `tests/integration/test_m4_no_gt.py` (short route, learned perception + localization, asserts completion).

## 5. Decisions log

- (2026-09-02) Deployed localization is *map-relative* (scan-to-map + smoother), not online SLAM. Online SLAM (odometry + loop closure + pose graph) is implemented and validated in the mapping tool. This mirrors production AV practice and gives lane-level accuracy without drift.
- (2026-09-02) Wheel speed from `/nuway/sim/vehicle_state` is treated as a legitimate sensor (CAN bus equivalent), not GT.

## 6. Open questions

- CARLA GNSS noise defaults are zero; set `noise_lat_stddev`/`noise_lon_stddev` to ≈ 1e-5 deg (~1 m) so GNSS is realistically weak and the smoother is actually exercised. Decide and put in the rig JSON.
- IMU factor may be over-engineering for a planar sim; if it causes instability, replace with a 2-D constant-yaw-rate BetweenFactor from gyro integration and document.
