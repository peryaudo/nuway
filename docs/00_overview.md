# nuway — Toy Autonomous Driving Stack on CARLA

**Status:** planning document, v1 (2026-09-02)
**Audience:** Claude Code and human contributors. Read this file first, then `01_directory_structure.md`, then `02_interfaces.md`, then `03_style_and_conventions.md`, then the milestone you are working on.

---

## 1. What we are building

A modular, "modern" autonomous driving stack that drives in the CARLA simulator, built from scratch on plain ROS 2 (no Autoware, no third-party AD packages) with all learned components in PyTorch. The end state is a stack that:

- localizes with a factor-graph (Graph SLAM) pipeline against a prior point-cloud map,
- perceives with a LiDAR+camera BEV fusion model (CenterPoint-style heads, temporal fusion, no Kalman filter),
- predicts other agents' futures **jointly** with a flow-matching generative model,
- plans with a learned planning head whose candidates are refined by QP/iLQR and chosen by a forward-simulation selector,
- controls with linear time-varying MPC,
- has a fully classical fallback path (FSM + Frenet lattice + QP) that never depends on learned components,
- is scored by an automated evaluation harness compatible with the CARLA Leaderboard 2.x conventions.

It is a *toy* system: small models, one workstation (RTX 3090 Ti 24 GB), CARLA 0.9.16. Correctness, debuggability and clean module boundaries are valued over peak performance.

## 2. Non-negotiable design principles

These apply to every milestone. Violating them requires updating this document first.

1. **Every module has a ground-truth (cheat) twin.** Localization, perception, traffic lights, prediction and planning each have a node that publishes the same message type from CARLA ground truth. A single config switch (`use_gt.<module>: true|false`) selects which one runs. This is how we ablate and debug. The localization, perception and traffic-light twins exist from M0; the prediction twin (`gt_prediction_node`, futures read ahead from a recorded CARLA log, so it only runs in log-replay evaluation) and the planning twin (`gt_planning_node`, the M6 privileged expert wrapped as a node) are delivered in M6.
2. **Explicit interfaces only.** No learnable feature maps cross a ROS topic boundary. Perception → planning carries an agent list and a multi-channel occupancy grid, both human-readable and visualizable. The prediction model and its ego planning head (M7/M8) are *one* module sharing one encoder in one process; what leaves that process is `PredictionSamples` and `TrajectoryCandidates`, nothing else. See `02_interfaces.md`.
3. **The classical path always works.** Whatever learned component is added, the FSM + lattice + QP + MPC path from Milestone 1 stays runnable and is the fallback at runtime.
4. **No Kalman filters.** State estimation is done with fixed-lag factor-graph smoothing (localization) and learned velocity attributes + nearest-neighbor association (perception). Where a "filter" is truly needed for high-rate control, use model-based forward propagation (extrapolation), not an EKF.
5. **Deterministic, lockstep simulation.** CARLA runs in synchronous mode, `fixed_delta_seconds = 0.05`. One process (the world manager) owns `world.tick()` and does **not** tick again until it has received the `ControlCommand` stamped for the current tick (see `02_interfaces.md` §2). Every rate in the stack is a multiple of the 20 Hz tick; nothing runs faster than one callback per tick, and nothing is timed by wall-clock inside the stack. `realtime_factor` only slows the loop down for humans watching; it never changes results.
6. **Runtime nodes in C++ (rclcpp); ML inference nodes and GT cheat twins in Python (rclpy); training in Python.** The C++ rule is about the *learned closed-loop path*, not about ground truth. A cheat twin is measurement scaffolding: it is never the thing a learned component ships behind, it is the thing the learned component is measured against, and it is written in whichever language lets its logic be **single-sourced with the offline code that generates the labels**. Point clouds and grids therefore cross into Python in two places, both deliberate: the ML inference nodes that consume or produce them (`bevfusion_node.py`, `flow_matching_node.py`), and `gt_perception_node.py`. No other Python node subscribes to a point cloud or a grid, and no C++ node receives one from Python except through `nuway_msgs` topics. The one twin that stays C++ is `gt_pose_node`: it publishes every tick, its publication is what triggers the controller, it must stay swap-identical with the C++ `pose_extrapolator_node` (M5), and at its size there is no duplicated implementation to remove (`M0_bringup.md` §5, 2026-09-06).
7. **Evaluation harness first.** Milestone 1 delivers the harness; every later milestone reports the same metrics from the same harness.
8. **Each milestone ends with a runnable stack.** Never leave `main` in a state where `ros2 launch nuway_bringup stack.launch.py` cannot complete a route with some combination of GT toggles.

## 3. Milestones

| # | Name | Deliverable | Completion criterion |
|---|------|-------------|----------------------|
| M0 | Bring-up | CARLA native ROS 2 connection, world manager, OpenDRIVE map server, route planner, vehicle system ID, pure-pursuit + PID controller. All perception/localization from GT. | 10 routes completed in empty towns, lateral error < 0.3 m |
| M1 | Classical planning | LTV-MPC, behavior FSM, Frenet lattice sampler, piecewise-jerk QP refinement, rule-based selector, safety layer, constant-velocity prediction, evaluation harness, **Leaderboard 2.x integration** (ROS agent wrapper, external tick owner, no CARLA client in the stack). | Driving score > 40 on Town03/05, 10 routes × 3 weathers, reproducible; same stack completes a route under the official Leaderboard runner |
| M2 | Perception data pipeline | Sensor-rich data collection, auto-labeling with visibility filtering, GT occupancy generator, WebDataset shards. | ≥ 150k labeled frames across ≥ 6 towns; dataset loader test passes |
| M3 | BEV perception | BEVFusion (LiDAR + 4 cameras), temporal fusion, CenterPoint heads with velocity, occupancy head, NN tracker. Replaces GT agents/occupancy. | Vehicle mAP > 0.6; driving score drop vs GT perception < 20% |
| M4 | Traffic light perception | Map-projected crop classifier, lane–traffic-light association verified against CARLA on every town, visibility latch near the stop line. Replaces GT traffic lights. | State accuracy > 0.97 within 40 m; red-light infractions ≤ 1.5× GT |
| M5 | Localization | KISS-ICP-style LiDAR odometry, offline mapping, scan-to-map registration, fixed-lag smoother (GTSAM), TF publisher. Replaces GT pose. **Milestone: the stack drives with zero GT.** | ATE < 0.2 m; driving score drop vs GT pose < 10% |
| M6 | Planning data pipeline | Privileged expert planner, render-free high-throughput collection, prediction/planning labels, noise injection, open-loop eval, GT prediction and GT planning cheat nodes. | ≥ 3M frames; expert score ≥ 85; data sanity report |
| M7 | Flow-matching prediction | Scene encoder, flow-matching velocity net, joint multi-agent sampling. Planner remains lattice; prediction becomes learned. | Driving score ≥ M5 (no-GT, lattice) + 10 |
| M8 | Learned planning head + DAgger | Ego planning head on the shared encoder, QP refinement of learned output, DAgger loop, runtime fallback logic. | Beats lattice baseline on driving score and intervention rate |
| M9 | Forward-sim selector | Batched kinematic rollout scorer over all candidates; two-stage selection. | Measurable score gain over rule-based selector |
| M10 | iLQR refinement | iLQR replaces QP on the ML-planner path; optional guidance in sampling. | No regression vs QP; smoother control effort |

Dependencies: M0 → M1 → M2 → M3 → M4 → M5 → M6 → M7 → M8 → M9 → M10. M2 and M6 share code; M6 extends M2. M4's model, association and latch work depend only on M2 (labels) and M0 (map) and can proceed in parallel with M3; its closed-loop criterion and the in-process node option need M3. M9 depends on M8 (it scores lattice + learned candidates and is measured against M8 checkpoints). M10 is optional.

**Baseline and the chain of relative criteria.** "The M1 score" that later milestones compare against is the M1 classical stack with GT localization and GT perception *with the M2 visibility filter on* (`gt_perception.apply_visibility: true`), re-measured at the end of M2. M3 to M5 each allow a bounded drop relative to the previous milestone, and M7/M8 must improve on their predecessor; there is deliberately **no absolute floor** tying the final no-GT learned stack back to the M1 GT baseline. It is therefore possible for the M8 stack to pass every criterion while scoring below M1-with-GT. That is accepted for a toy system whose purpose is the pipeline, not the leaderboard rank; the M8 report must state the gap explicitly.

## 4. Environment

| Item | Choice | Note |
|------|--------|------|
| OS | Ubuntu 22.04 | |
| ROS 2 | Humble | rmw: CycloneDDS with shared memory (iceoryx) enabled |
| CARLA | 0.9.16 (UE4) | Not 0.10.x (UE5: heavier GPU, native ROS 2 less stable) |
| Python | 3.10 (system, Humble's) | `uv`-managed venv at repo root (`uv sync`, `uv.lock` committed, system site packages for rclpy), `torch>=2.4`, CUDA 12.x |
| GPU | RTX 3090 Ti 24 GB | shared between CARLA and inference |
| CPU | ≥ 12 cores recommended | CARLA + Traffic Manager alone use 4–6 |
| C++ | C++17, GCC 11 (Clang 17+ locally), CMake ≥ 3.22 via `ament_cmake`, Ninja + ccache + mold | Eigen, GTSAM 4.2, OSQP + osqp-eigen, nanoflann, PCL (I/O only); non-apt libs as `*_vendor` packages |
| Build | `source setup_env.sh && colcon build` (defaults from `ros2_ws/colcon_defaults.yaml`) | `uv sync` for Python; no `pip` anywhere |
| Tooling | clang-format/clang-tidy (LLVM ≥ 17 PyPI wheels), ruff, mypy, gersemi, pre-commit | all pinned in `uv.lock`; see `03_style_and_conventions.md` §6 |
| Training tooling | Hydra (config management, `configs/training/`), Weights & Biases (losses, metrics, run configs; project `nuway`) | `train` dependency group; see `03_style_and_conventions.md` §9.7. Runtime nodes depend on neither |
| Visualization | Foxglove (live) + matplotlib/`Agg` renders to PNG (headless) | `viz` dependency group; `02_interfaces.md` §8. Every layer exists in both back-ends; the headless one needs neither CARLA nor ROS |

CARLA launch (development):
```
./CarlaUE4.sh -RenderOffScreen -quality-level=Low --ros2 -carla-rpc-port=2000
```

## 5. Compute budget (target, wall clock, single GPU shared with CARLA)

| Stage | Rate | Budget |
|-------|------|--------|
| CARLA tick incl. 4 cams (704×256) + 32-ch LiDAR | 20 Hz sim | ≤ 35 ms |
| LiDAR preprocessing + BEVFusion (fp16) | 10 Hz | ≤ 40 ms |
| Traffic light crop + classifier | 10 Hz | ≤ 5 ms |
| LiDAR odometry + smoother | 10 Hz | ≤ 20 ms (CPU) |
| Prediction (encoder + 6 Euler steps × 16 samples) | 10 Hz | ≤ 25 ms |
| Planner cycle, total (all of the rows below that are enabled by the profile) | 10 Hz | ≤ 35 ms |
| ├ Lattice + collision check + QP (top-8) + rule selector (M1) | 10 Hz | ≤ 15 ms |
| ├ QP refinement of learned candidates (M8, replaced by iLQR in M10) | 10 Hz | ≤ 5 ms |
| ├ Forward-simulation selector, CPU (M9) | 10 Hz | ≤ 8 ms |
| └ iLQR refinement of learned candidates, 8 s horizon (M10) | 10 Hz | ≤ 12 ms |
| MPC | 20 Hz | ≤ 3 ms |

Because CARLA is in lockstep, exceeding these budgets slows the simulation but does not change results. Budgets exist to keep interactive development pleasant and Leaderboard timeouts safe.

## 6. Conventions summary (full detail in `02_interfaces.md`)

- Frames: `map`, `odom`, `base_link`, `lidar_top`, `cam_front`, `cam_left`, `cam_right`, `cam_rear`, `imu`, `gnss`. ROS REP-103 (x forward, y left, z up, right-handed). CARLA's left-handed frame is converted at the boundary and nowhere else.
- Time: all stamps are simulation time from `/clock`. `use_sim_time: true` everywhere. Rates are stated in ticks (1 tick = 0.05 s): "20 Hz" means every tick, "10 Hz" every second tick. No node claims a rate above 20 Hz.
- Ego vehicle role name: `hero`.
- Config: one YAML per launch profile under `configs/`, overriding package defaults. GT toggles live in `configs/gt_toggles/*.yaml` (fragments that a profile's `use_gt:` block includes or overrides).
- Logging: every node publishes `/nuway/diag/<node>` (`nuway_msgs/NodeDiag`) with processing time per cycle.
- Reset: every node that carries state across cycles subscribes to `/nuway/sim/reset_event` and clears that state (`02_interfaces.md` §7). Route-to-route results must not depend on run order.
- Tests: C++ (gtest) and Python (pytest) unit tests must pass in `colcon test` / `uv run pytest ml tools`. Every milestone adds an integration test runnable with `tools/eval/run_routes.py --profile <milestone>`.

## 7. How Claude Code should work in this repo

- Read the milestone document fully before starting. Each has a **Task list** section; work through it in order and tick items in the doc as they land.
- Prefer small, compilable increments. Run `colcon build --packages-select <pkg>` and the package tests after every change.
- Style is specified in `03_style_and_conventions.md`: Google C++ Style Guide for C++, PEP 8 for Python. Run `clang-format` + `clang-tidy` after every C++ change and `ruff format` + `ruff check` + `mypy` after every Python change (root configs); all must be clean before a commit.
- **Look at the renders.** Foxglove is for a human at the devbox; you cannot see it. After an eval run, read `data/eval_runs/<run_id>/report.md` and open the incident sheets it links (`02_interfaces.md` §8) — they are PNGs on disk, and they are how you debug a route without a human describing it to you. `tools/viz/render_bag.py` renders any recorded bag on demand. The same holds for training: `data/checkpoints/<experiment>/<timestamp>/viz/`, not the W&B web UI.
- When a spec in a milestone doc is ambiguous, choose the simplest option that satisfies the completion criterion, and record the decision in the milestone doc under **Decisions log**.
- Do not introduce new message types, topics, services or profile-level config keys outside `02_interfaces.md`; propose the change there first.
- Never commit data. `data/` is gitignored.
- Keep `docs/` in sync with code: if you rename a node or topic, update the docs in the same commit.
