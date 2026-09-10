# Directory structure

Monorepo. Two build systems coexist: `colcon` for `ros2_ws/` and `uv` (workspace root `pyproject.toml`, member `ml/`) for everything Python. Tools under `tools/` are plain Python scripts run through `uv run` that import from `nuway_ml` and the CARLA Python API; the one package under `tools/` is the evaluation harness `tools/eval/nuway_eval/`, which uses `rclpy` and therefore cannot live in `ml/`.

```
nuway/
├── README.md
├── docs/
│   ├── 00_overview.md
│   ├── 01_directory_structure.md          # this file
│   ├── 02_interfaces.md                   # frames, topics, messages, configs
│   ├── 03_style_and_conventions.md        # Google C++ style, clang-format/clang-tidy, python rules
│   ├── 04_setup.md                        # fresh-machine setup procedure (ROS, CARLA, toolchain)
│   └── milestones/
│       ├── M0_bringup.md
│       ├── M1_classical_planning.md
│       ├── M2_perception_data_pipeline.md
│       ├── M3_bev_perception.md
│       ├── M4_traffic_light_perception.md
│       ├── M5_localization.md
│       ├── M6_planning_data_pipeline.md
│       ├── M7_flow_matching_prediction.md
│       ├── M8_learned_planner_dagger.md
│       ├── M9_forward_sim_selector.md
│       └── M10_ilqr.md
│
├── configs/                               # runtime configuration (YAML)
│   ├── profiles/                          # one per launch profile
│   │   ├── m0_gt_all.yaml
│   │   ├── m0_gt_all.yaml                 # M0: Town03, rig_dev, every use_gt toggle true, pure pursuit + PID
│   │   ├── sysid.yaml                     # M0 system identification: generated straight (configs/maps/sysid_straight.xodr), rig_none, no route
│   │   ├── m1_classical.yaml
│   │   ├── m3_learned_perception.yaml
│   │   ├── m4_learned_tl.yaml
│   │   ├── m5_no_gt.yaml
│   │   ├── mapping.yaml                   # M5 offline mapping runs (GT pose, label-only semantic LiDAR)
│   │   ├── m6_gt_planning.yaml            # M6: m1_classical + use_gt.planning: true + gt_perception.source: geometry
│   │   ├── m7_learned_prediction.yaml
│   │   ├── m8_learned_planner.yaml
│   │   ├── m8_dagger_cheap.yaml           # M8: GT loc/perception/TL, carla.no_rendering, rig_none, geometry occupancy, shadow_expert
│   │   ├── m9_forward_sim.yaml            # M9: includes m8_learned_planner, overrides planning.selector: forward_sim
│   │   ├── m10_ilqr.yaml                  # M10: includes m9_forward_sim, overrides planning.refiner: ilqr
│   │   └── leaderboard.yaml               # one stack per route under the official runner (M1 §3.12)
│   ├── gt_toggles/                        # small YAMLs setting use_gt.* flags
│   ├── sensors/                           # sensor rigs (CARLA blueprint attrs + extrinsics)
│   │   ├── rig_dev.json
│   │   ├── rig_leaderboard.json
│   │   └── rig_none.json                  # no sensors: M0 sysid, and M8 render-free GT-only runs (carla.no_rendering: true)
│   ├── vehicle/
│   │   └── lincoln_mkz_2020.yaml          # sysid results (M0)
│   ├── maps/
│   │   ├── sysid_straight.xodr            # M0: generated 3 km straight for tools/sysid (carla.town: <path>.xodr)
│   │   └── <town>/                        # curated per-town files: tl_overrides.yaml (M4), tl_bulbs.json
│   │                                      # (no planning/ or control/ directory: runtime node parameters live in each package's
│   │                                      #   config/defaults.yaml and are overridden from a profile's <node>: block, 02 §5, M0 task 12)
│   ├── perception/
│   ├── localization/
│   ├── eval/
│   │   └── scoring_lb20.yaml              # Leaderboard 2.0 penalty coefficients (M1)
│   ├── collect/
│   │   ├── perception_v1.yaml             # M2 collection protocol
│   │   └── planning_v1.yaml               # M6 collection protocol
│   ├── maps/<town>/                       # committed, small, partly hand-curated per-town map artefacts (M4);
│   │   ├── tl_bulbs.json                  #   bulk map data stays in data/maps. Bulb positions from CARLA get_light_boxes()
│   │   ├── tl_overrides.yaml              #   lane->light association fallbacks decided by verify_tl_association.py
│   │   └── tl_association_report.md
│   └── training/                          # Hydra config root for ml/ training (03 §9.7)
│       ├── bevfusion.yaml                 # M3 experiment: defaults list + overrides
│       ├── traffic_light.yaml             # M4
│       ├── fm_prediction.yaml             # M7
│       ├── planner.yaml                   # M8 (multi-task + DAgger rounds)
│       ├── model/                         # config groups; schemas live in nuway_ml/common/config.py
│       ├── data/
│       ├── optim/
│       ├── stage/                         # M3 stage A/B/C, M8 DAgger rounds
│       └── logging/                       # wandb project/group/mode
│
├── ros2_ws/
│   ├── colcon_defaults.yaml               # Ninja, ccache, mold, compile_commands, RelWithDebInfo
│   └── src/
│       ├── nuway_cmake/                   # shared CMake: warnings, -Werror, NUWAY_CLANG_TIDY/SANITIZE/LTO
│       ├── carla_msgs/                    # vendored from ros-carla-msgs (leaderboard-2.0 branch); messages only (M0)
│       ├── osqp_eigen_vendor/             # ExternalProject, pinned tag+hash (M1) — the only vendored
│       │                                  #   dep; OSQP, GTSAM and nanoflann come from apt on
│       │                                  #   Jazzy/noble (03_style_and_conventions.md §6.2)
│       ├── nuway_msgs/                    # ALL custom messages and services. No msgs/srvs elsewhere.
│       │   ├── msg/
│       │   ├── srv/
│       │   └── CMakeLists.txt
│       ├── nuway_common/                  # C++ header-mostly library
│       │   ├── include/nuway_common/
│       │   │   ├── geometry.h           # SE2, SE3 helpers, angle wrap
│       │   │   ├── agents.h             # AgentClass, AgentState, PredictionSet: plain images of Agent / PredictionSamples
│       │   │   │                        #   for the rclcpp-free planning and prediction libraries (M1)
│       │   │   ├── frenet.h             # reference line, cartesian<->frenet (+ state conversions, M1)
│       │   │   ├── polynomial.h         # quintic / quartic polynomials for the lattice (M1; mirrors polynomial.py)
│       │   │   ├── bicycle_model.h      # kinematic bicycle, discretization, jacobians
│       │   │   ├── trajectory.h         # Trajectory struct, resampling, interpolation
│       │   │   ├── carla_conv.h         # left-handed <-> ROS conversion, GNSS <-> map (C++ side; see Rules)
│       │   │   ├── occupancy.h          # multi-channel grid accessor, bilinear sample
│       │   │   ├── diag.h               # NodeDiag publisher helper, scoped timer
│       │   │   ├── qos.h                # the QoS profiles from 02_interfaces.md §3.11
│       │   │   ├── tick.h               # TickIndex(stamp), IsPlanningTick(k); current-tick barrier helper (02 §2)
│       │   │   ├── frames.h             # frame ids and the topic names shared by more than one package (mirrors frames.py)
│       │   │   ├── params.h             # declare/get param helpers
│       │   │   ├── ros_conv.h           # geometry_msgs <-> Eigen / SE2 / SE3 field copies for nodes (mirrors ros_conv.py);
│       │   │   │                        #   AgentsToMap(AgentArray, EgoState): the one base_link -> map agent transform (M1 §3.1)
│       │   │   └── node_main.h          # RunNode<T>(): the one main() body of every C++ node (init, spin, shutdown, node-boundary catch)
│       │   └── test/
│       ├── nuway_py/                      # THE pybind11 package (one per repo). M0: nuway_common + nuway_control::LongitudinalMap
│       │                                  #   for the parity tests; M6: FSM, lattice, QP, selector, collision, const-vel predictor,
│       │                                  #   safety layer, MPC for the expert and gt_planning_node. Deliberate exception to the
│       │                                  #   "ml/ owns everything tools/ imports" rule (see Rules)
│       ├── nuway_rclpy/                   # python: the rclpy-side helpers every Python node package shares (M0):
│       │   └── nuway_rclpy/               #   ros_qos.py (QoSProfile from the 02 §3.11 table), ros_conv.py (SE3 <-> Pose,
│       │                                  #   stamps). nuway_ml never imports rclpy, so these cannot live there
│       ├── nuway_carla_bridge/            # python (rclpy) — talks to CARLA Python API. Two nodes: world_manager and
│       │   ├── nuway_carla_bridge/        #   control_adapter. gt_publisher.py and sensor_rig.py are MODULES hosted by the
│       │   │   ├── world_manager.py       #   world_manager node (one process, one CARLA client). Node: owns world.tick() in
│       │   │   │                          #   lockstep, /clock, /nuway/sim/tick_timeout, the reset/weather services
│       │   │   ├── gt_publisher.py        # module: GT agents, GT ego pose/odom, traffic lights, vehicle_state
│       │   │   ├── sensor_rig.py          # module: spawns the rig, publishes /tf_static + camera_info + chase_cam; all
│       │   │   │                          #   parsing/arithmetic comes from nuway_ml.common.rig (shared with the collectors)
│       │   │   ├── control_adapter.py     # node: ControlCommand -> CarlaEgoVehicleControl
│       │   │   └── leaderboard_agent.py   # Leaderboard 2.x ROS agent entry point (M1)
│       │   └── launch/
│       ├── nuway_map/                     # C++
│       │   ├── src/
│       │   │   ├── opendrive_parser.cc   # OpenDRIVE -> LaneGraph
│       │   │   ├── lane_graph.cc         # lanes, successors, neighbors, TL/stop associations
│       │   │   ├── tl_overrides.cc       # loads configs/maps/<town>/tl_overrides.yaml (M4)
│       │   │   └── map_server_node.cc    # publishes nuway_msgs/LaneGraph (latched), query srv
│       │   └── include/nuway_map/
│       ├── nuway_route/                   # C++: A* on lane graph, reference line builder
│       │   └── tools/route_gen.cc        # `ros2 run nuway_route route_gen`: seeded successor walks -> tools/eval/routes/*.xml,
│       │                                  #   every route validated with the stack's RoutePlanner (M0 task 13)
│       ├── nuway_localization/            # C++
│       │   ├── config/defaults.yaml
│       │   ├── include/nuway_localization/
│       │   ├── src/
│       │   │   ├── gt_pose_node.cc       # cheat twin (M0): GT odom + vehicle_state -> /nuway/loc/pose + TF, noise.* params
│       │   │   ├── voxel_hash_map.cc / icp.cc / scan_context.cc   # M5 libraries
│       │   │   ├── lidar_odometry.cc / lidar_odometry_node.cc      # M5
│       │   │   ├── scan_to_map.cc / scan_to_map_node.cc            # M5
│       │   │   ├── smoother.cc / smoother_node.cc                  # M5 (GTSAM fixed-lag)
│       │   │   └── pose_extrapolator.cc / pose_extrapolator_node.cc  # M5 (per-tick odom->base_link)
│       │   └── tools/                     # map_builder, pose_graph_refine binaries (M5)
│       ├── nuway_perception/               # python except lidar_preproc_node
│       │   ├── src/lidar_preproc_node.cc    # escalation path only: C++ pillarization if Python preproc > 8 ms (M3)
│       │   ├── config/defaults.yaml
│       │   ├── nuway_perception/          # python
│       │   │   ├── gt_twin_node.py           # base of the GT twins: tick barrier, reset, TickTimeout degradation, diag (M0)
│       │   │   ├── gt_perception_node.py     # cheat twin: AgentArray + OccupancyGridMC from GT (M0/M2)
│       │   │   ├── gt_traffic_light_node.py  # cheat twin: TrafficLightArray from GT (M0)
│       │   │   ├── perception_node.py        # inference (M3); optionally hosts traffic_light_node (M4); imports the
│       │   │   │                             #   tracker from nuway_ml.perception.tracker
│       │   │   └── traffic_light_node.py     # crop, classify, latch (M4)
│       │   └── launch/
│       ├── nuway_prediction/
│       │   ├── src/const_vel.cc / const_vel_node.cc   # M1; library (bound by nuway_py for the M6 expert) + node
│       │   ├── include/nuway_prediction/
│       │   └── nuway_prediction/
│       │       ├── gt_prediction_node.py     # cheat twin: futures from a recorded CARLA log, replay eval only (M6)
│       │       └── prediction_node.py        # M7; also hosts the M8 ego planning head
│       ├── nuway_planning/                # C++ except gt_planning_node.py
│       │   ├── src/
│       │   │   ├── route_line.cc          # reference line + lane ids, limits, bounds, goal; the M0 speed profile as SpeedBoundAt (M1)
│       │   │   ├── behavior_fsm.cc / behavior_fsm_node.cc      # M1
│       │   │   ├── lattice_sampler.cc     # Frenet lattice + feasibility filter + the injected stop pair; candidate.h is the Candidate struct (M1)
│       │   │   ├── collision_checker.cc   # SAT box overlap vs predictions, min_ttc, 3-disc min distance (M1)
│       │   │   ├── piecewise_jerk_qp.cc   # the OSQP piecewise-jerk QP both refinements reduce to (M1)
│       │   │   ├── candidate_refiner.cc   # path QP + speed QP (S-T boxes) on one candidate (M1)
│       │   │   ├── rule_selector.cc                             # M1
│       │   │   ├── safety_layer_node.cc                         # M1
│       │   │   ├── planner_node.cc        # orchestrates: candidates -> refine -> select
│       │   │   ├── forward_sim_scorer.cc  # M9
│       │   │   └── ilqr.cc                # M10
│       │   ├── nuway_planning/gt_planning_node.py   # cheat twin: M6 expert (via nuway_py) as a node (M6)
│       │   └── include/nuway_planning/
│       ├── nuway_control/                 # C++
│       │   ├── src/
│       │   │   ├── longitudinal_map.cc        # a_des -> throttle/brake from sysid table (M0)
│       │   │   ├── vehicle_model.cc           # configs/vehicle/<vehicle>.yaml -> VehicleModel (geometry, limits, fit, map) (M0)
│       │   │   ├── pure_pursuit_pid.cc        # the controller as a library (gtest without rclcpp) (M0)
│       │   │   ├── pure_pursuit_pid_node.cc   # M0
│       │   │   └── mpc_node.cc                # M1
│       │   ├── include/nuway_control/
│       │   ├── config/defaults.yaml
│       │   └── test/
│       ├── nuway_viz/                     # marker_node.cc, foxglove layouts, rviz configs
│       └── nuway_bringup/
│           ├── nuway_bringup/
│           │   ├── profile.py             # profile YAML -> per-node parameter dicts (defaults < profile keys < <node>: block);
│           │   │                          #   no rclpy, tested by tests/integration/test_bringup_profile.py
│           │   └── launch_util.py         # profile_launch()/stack_node(): the launch-file side of profile.py
│           ├── launch/stack.launch.py     # THE launch file; reads a profile YAML
│           └── launch/*.launch.py         # per-subsystem launches used by stack.launch.py (sim, map, localization,
│                                          #   perception, prediction, planning, control, viz)
│
├── ml/                                    # uv workspace member `nuway-ml` (hatchling)
│   ├── pyproject.toml                     # package metadata + runtime deps only; no [tool.ruff]/[tool.mypy] here
│   ├── nuway_ml/
│   │   ├── common/
│   │   │   ├── geometry.py                # numpy/torch SE2 utils, must mirror nuway_common
│   │   │   ├── frenet.py                  # mirrors nuway_common/frenet.h
│   │   │   ├── polynomial.py              # mirrors nuway_common/polynomial.h (M1)
│   │   │   ├── trajectory.py              # mirrors nuway_common/trajectory.h resampling (M8 spline resample)
│   │   │   ├── longitudinal_map.py        # mirrors nuway_control's LongitudinalMap; parity-tested (M0)
│   │   │   ├── carla_conv.py              # left-handed <-> ROS conversion (Python side; see Rules)
│   │   │   ├── occupancy.py               # GridSpec + world<->grid: numpy, no torch (gt_perception_node imports it)
│   │   │   │                               # bilinear_sample(): torch, imported lazily inside the function
│   │   │   ├── frames.py                  # frame name constants shared with the ROS side
│   │   │   ├── qos.py                     # the QoS table of 02 §3.11 as plain data (no rclpy import)
│   │   │   ├── tick.py                    # tick_index(stamp), is_planning_tick(k); mirrors tick.h (02 §2)
│   │   │   ├── rig.py                     # the one rig-JSON parser: entries, CARLA->base_link extrinsics, intrinsics K from
│   │   │   │                               # size+FOV. Used by sensor_rig.py (tf_static, camera_info), the collectors and
│   │   │   │                               # CameraCalib (M2), so every K and every extrinsic has one source
│   │   │   ├── routes.py                  # route XML / Leaderboard global_plan -> ordered waypoints (nav_msgs/Path payload)
│   │   │   ├── seeding.py                 # the one random-seed helper
│   │   │   ├── config.py                  # structured-config dataclasses + Hydra ConfigStore
│   │   │   ├── run_logger.py              # the one wandb wrapper; training loops log through it
│   │   │   └── schema.py                  # dataset record schema (dataclasses + validation)
│   │   ├── viz/                           # M1; the one BEV drawing implementation (docs/02 §8)
│   │   │   ├── style.py                   # colors, widths, figure geometry; Agg backend only
│   │   │   ├── bev_draw.py                # one draw_<layer>() per §3.9 marker layer, onto an Axes
│   │   │   ├── panel.py                   # the standard frame: BEV + header text + diag strip
│   │   │   └── contact_sheet.py           # tile N frames into one PNG
│   │   ├── data/
│   │   │   ├── webdataset_io.py           # shard writer/reader
│   │   │   ├── labeling/
│   │   │   │   ├── visibility.py          # M2
│   │   │   │   ├── boxes.py               # M2
│   │   │   │   └── traffic_lights.py      # M2
│   │   │   ├── postprocess_run.py         # history/future fill (M2)
│   │   │   ├── perception_dataset.py      # M2
│   │   │   ├── planning_dataset.py        # M6
│   │   │   ├── augment.py                 # occupancy noise, agent dropout (M6/M8)
│   │   │   └── gt_occupancy.py            # GT occupancy generator; the only implementation (collector + gt_perception_node)
│   │   ├── perception/                    # M3
│   │   │   ├── model/
│   │   │   │   ├── camera_branch.py
│   │   │   │   ├── lidar_branch.py
│   │   │   │   ├── fusion.py
│   │   │   │   ├── temporal.py
│   │   │   │   ├── centerpoint_head.py
│   │   │   │   ├── occupancy_head.py
│   │   │   │   └── bevfusion.py
│   │   │   ├── losses.py
│   │   │   ├── targets.py                 # heatmap/regression target generation
│   │   │   ├── decode.py                  # peak extraction, box decoding
│   │   │   ├── tracker.py                 # velocity-projected NN association (M3); numpy/scipy, no ROS; used by
│   │   │   │                               #   perception_node.py and ml/scripts/eval_tracking.py
│   │   │   ├── metrics.py                 # mAP, occupancy IoU
│   │   │   └── train.py
│   │   ├── traffic_light/                 # M4
│   │   │   ├── model.py                   # 4-layer crop CNN
│   │   │   ├── projection.py              # map TL -> camera crop (shared with the node)
│   │   │   ├── latch.py                   # visibility latch state machine (shared with the node)
│   │   │   ├── metrics.py
│   │   │   └── train.py
│   │   ├── prediction/                    # M7
│   │   │   ├── tokenizer.py               # scene -> tokens
│   │   │   ├── scene_encoder.py
│   │   │   ├── relpe.py                   # pairwise relative positional encoding
│   │   │   ├── deform_attn.py
│   │   │   ├── velocity_net.py
│   │   │   ├── flow_matching.py           # train step, sampler, guidance
│   │   │   ├── aux_losses.py
│   │   │   ├── metrics.py                 # ADE/FDE/minADE, collision rate
│   │   │   └── train.py
│   │   ├── planning/                      # M8
│   │   │   ├── planning_head.py
│   │   │   ├── forward_sim.py             # torch reference of the M9 forward simulator (tests only)
│   │   │   ├── dagger.py
│   │   │   └── train.py
│   │   └── export/                        # torchscript / tensorrt export helpers
│   ├── scripts/                           # eval_tracking.py (M3), eval_openloop.py (M6), compute_norm_stats.py (M7)
│   ├── notebooks/                         # inspect_perception.ipynb (M2), committed without outputs
│   └── tests/
│
├── tools/
│   ├── carla/
│   │   ├── start_carla.sh                 # M0: the one place the server flags live (-RenderOffScreen --ros2, port, never -quality-level=Low)
│   │   └── check_native_ros2.py           # M0 smoke test
│   ├── sysid/                             # M0
│   │   ├── run_sweeps.py                  # throttle / brake / coast / steer sweeps -> data/sysid/<mode>.csv (acts as the controller)
│   │   ├── fit_models.py                  # CSVs -> configs/vehicle/lincoln_mkz_2020.yaml + residual plots in data/sysid/
│   │   └── tests/
│   ├── collect/                           # data collection clients
│   │   ├── collect_perception.py          # M2 (rendered)
│   │   ├── collect_planning.py            # M6 (no rendering)
│   │   ├── collect_dagger.py              # M8
│   │   ├── launch_farm.sh                 # M6: N CARLA servers + collectors
│   │   ├── expert/                        # privileged expert planner (M6)
│   │   └── scenarios/                     # scenario/route generation helpers
│   ├── mapping/                           # M4/M5 offline map tooling
│   │   ├── build_map.py                   # M5
│   │   ├── build_static_occ.py            # M6: static occupancy raster from the M5 map
│   │   └── verify_tl_association.py       # M4
│   ├── eval/
│   │   ├── nuway_eval/                    # THE harness package (rclpy; imported by the scripts below, never by ml/ or ros2_ws)
│   │   │   ├── route_runner.py            # one stack per town, second non-ticking CARLA client, resume. M0 v0: reset +
│   │   │   │                              #   waypoints per route, lateral error from ControlDebug, odom/debug CSVs per route
│   │   │   ├── infractions.py
│   │   │   ├── driving_score.py
│   │   │   ├── report.py                  # results.csv (schema in M1 §3.10), report.md, incident renders
│   │   │   └── chase_writer.py            # M1; /nuway/viz/chase_cam -> chase/*.jpg (docs/02 §8.3)
│   │   ├── routes/                        # route XMLs (leaderboard format): dev_* (M0: dev_town01/03/05, 10 routes >= 1.5 km
│   │   │                                  #   from route_gen), mapping_*, collect_long_*
│   │   ├── run_routes.py                  # entry point for evaluation harness (also --replay, M6)
│   │   ├── setup_leaderboard.sh           # M1: clones leaderboard + scenario_runner into external/ at the pinned commits
│   │   ├── run_leaderboard.sh             # M1: one stack + one evaluator invocation per route (M1 §3.12)
│   │   ├── eval_localization.py           # M5
│   │   └── compare_runs.py
│   ├── lint/                              # format_cpp.sh, tidy_cpp.sh, lint_py.sh, merge_compile_commands.py, header guard check (M0)
│   └── viz/                               # M1; headless renderers (docs/02 §8); no ROS env needed
│       ├── render_bag.py                  # MCAP -> frames/, sheets/, incidents/ PNGs
│       └── make_video.sh                  # optional ffmpeg wrapper; MP4 is never the primary artifact
│
├── external/                              # gitignored; leaderboard/ and scenario_runner/ checkouts (tools/eval/setup_leaderboard.sh)
├── data/                                  # gitignored
│   ├── raw/
│   ├── sysid/                             # M0 system-identification logs + residual plots
│   ├── shards/
│   ├── maps/<town>/                       # one directory per town: map.xodr, map.ply, map_tags.npy, static_occ.npz, REPORT.md
│   │                                      #   (the small curated files live in configs/maps/<town>/)
│   ├── checkpoints/                       # <experiment>/<timestamp>/: ckpts, viz/, .hydra/config.yaml
│   └── eval_runs/                         # <run_id>/: report.md, results.csv, per-route mcap + renders (docs/02 §8)
│
├── tests/                                 # cross-cutting integration tests
│   ├── fixtures/                          # gen_*.py generators (CLI, may print) + small committed CSV/JSON fixtures
│   └── integration/
├── .github/workflows/                     # CI: format, tidy, sanitizer, ruff, mypy, pytest (see docs/03 §7.4)
├── .clang-format                          # BasedOnStyle: Google (see docs/03)
├── .clang-tidy                            # google-* + naming checks, warnings are errors (see docs/03)
├── .pre-commit-config.yaml                # local hooks via `uv run`: clang-format, clang-tidy, gersemi, ruff, mypy
├── .clangd                                # points at merged ros2_ws/build/compile_commands.json
├── pyproject.toml                         # uv workspace root: dependency groups, ruff, mypy, pytest config
├── uv.lock                                # committed; the only place versions are pinned
├── .python-version                        # 3.12
├── .gitignore
└── setup_env.sh                           # source in every shell: ROS, colcon defaults, uv venv, pre-commit
```

## Rules

- **`nuway_msgs` is the only package that defines messages and services.** Any new message or service requires an entry in `02_interfaces.md`.
- **`nuway_common/carla_conv.h` (C++) and `nuway_ml/common/carla_conv.py` (Python) are the only code that knows CARLA's coordinate convention.** `nuway_carla_bridge` and the collectors under `tools/collect/` call `carla_conv.py` (through `rig.py` for everything rig-related); they contain no conversion arithmetic of their own. The two files are parity-tested against each other, including the GNSS ↔ map conversion added in M5. Everything downstream is ROS-convention.
- **`nuway_ml/common/*` must mirror `nuway_common/*`** for geometry/frenet/occupancy/carla_conv/tick conventions. There is a cross-language test (`tests/integration/test_geometry_parity.py`) that runs the C++ versions through the `nuway_py` pybind package and compares outputs. Keep it green.
- **`nuway_py` is the one pybind package and the one sanctioned import from `ros2_ws` into `tools/`.** It exists so that the M6 expert, `gt_planning_node.py` and the parity tests run the *same* C++ planner/controller code as the runtime, which is worth more than the "everything `tools/` imports lives in `ml/`" rule it bends. It is available only after `colcon build` with `ros2_ws/install/setup.bash` sourced; `setup_env.sh` does that. It ships no stubs, so it is listed in the mypy `ignore_missing_imports` override (`03_style_and_conventions.md` §7.3). No *node package* under `ros2_ws` may be imported by `tools/` or `ml/`; the rclpy-side harness code (`tools/eval/nuway_eval/`, `tools/sysid/run_sweeps.py`, `tools/carla/check_native_ros2.py`) may import `rclpy` and the message packages (`nuway_msgs`, `carla_msgs`, the standard `*_msgs`), nothing else. This is why the evaluation harness (`nuway_eval`) lives under `tools/eval/` and the tracker under `ml/nuway_ml/perception/`: anything a script or a training-side tool imports is outside `ros2_ws`.
- **`ml/` never imports `rclpy`.** `nuway_ml` must import in an environment with no ROS (the collectors, `render_bag.py`, training). Code that needs `rclpy` and is not a node lives in `tools/eval/nuway_eval/`.
- **Mixed C++/Python packages** (`nuway_perception`, `nuway_prediction`, `nuway_planning`) are `ament_cmake` packages that install their Python subpackage with `ament_cmake_python` (`ament_python_install_package(${PROJECT_NAME})`) and register the Python nodes as `install(PROGRAMS ...)` entry points. `mypy` finds them through `mypy_path` (`03_style_and_conventions.md` §7.3).
- **In `nuway_carla_bridge` only `world_manager.py` and `control_adapter.py` are nodes.** `gt_publisher.py` and `sensor_rig.py` are modules called by the `world_manager` node, because there is exactly one CARLA client per process and the GT topics must be emitted in the same call that publishes `/clock`. "One node per file" applies to the node files; module files say `# module, not a node` in their header comment.
- **GT occupancy has exactly one implementation** (`nuway_ml/data/gt_occupancy.py`, both the sensor-based `generate_gt_occupancy` and the geometry-based `generate_from_geometry`), called directly by both the offline collectors and the runtime cheat node `gt_perception_node.py`. There is no C++ port and therefore no parity test to keep green. This is why the GT perception twin is Python (`00_overview.md` §2.6): a second implementation of the DDA ray casting and lane rasterization, maintained forever for one consumer, costs more than the node saves. No CLI bridge, no pybind for this function.
- One node per file, one file per node. Nodes are thin: they parse params, subscribe/publish, and call a library class that is unit-tested without ROS.
- Python packages inside `ros2_ws` only contain nodes and thin glue. Model code and anything shared with `tools/` lives in `ml/nuway_ml` and is imported.
- Config YAML keys are namespaced by node name. Do not read environment variables in nodes.
- **`configs/training/` is for Hydra only and `configs/*` elsewhere is for ROS nodes only.** Training configs are composed by Hydra from the groups above and validated against the dataclasses in `nuway_ml/common/config.py`; runtime YAMLs are loaded by ROS and never by `ml/`. A value both sides need (grid spec, class list, history length) is declared in `02_interfaces.md` and duplicated deliberately, with a parity test, not shared by importing one config from the other.
- **Every visualization has a headless twin that writes image files under `data/`.** Foxglove layouts in `nuway_viz` are for a human at the devbox; a layer that exists only as a Foxglove panel is incomplete. The drawing code is `ml/nuway_ml/viz/` (matplotlib, `Agg`, no ROS, no GPU, no CARLA), imported by `tools/viz/render_bag.py`, by the M2 spot-check notebook and by the training-time `val/viz` renders, so a training render and an eval render of the same scene look identical. Contract in `02_interfaces.md` §8.
- **Training metrics live in Weights & Biases, artifacts live in `data/`.** No metric CSVs, no tensorboard event files, no plots committed to the repo; a milestone's reported numbers cite a W&B run and the checkpoint directory that produced them (`03_style_and_conventions.md` §9.7).
- Python dependencies enter only through `uv add` (lock updated in the same commit). C++ dependencies enter only through `package.xml` + rosdep, or a `*_vendor` package. No `pip`, no submodules, no system-wide installs. See `03_style_and_conventions.md` §6.
- All C++ is Google style and must pass `clang-format` and `clang-tidy`; all Python is PEP 8 and must pass `ruff format`, `ruff check` and `mypy`, with the root configs. See `03_style_and_conventions.md`.
