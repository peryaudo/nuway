# Directory structure

Monorepo. Two build systems coexist: `colcon` for `ros2_ws/` and `pip` for `ml/`. Tools under `tools/` are plain Python scripts that import from `nuway_ml` and the CARLA Python API.

```
nuway/
├── README.md
├── docs/
│   ├── 00_overview.md
│   ├── 01_directory_structure.md          # this file
│   ├── 02_interfaces.md                   # frames, topics, messages, configs
│   ├── 03_style_and_conventions.md        # Google C++ style, clang-format/clang-tidy, python rules
│   └── milestones/
│       ├── M0_bringup.md
│       ├── M1_classical_planning.md
│       ├── M2_perception_data_pipeline.md
│       ├── M3_bev_perception.md
│       ├── M4_localization.md
│       ├── M5_planning_data_pipeline.md
│       ├── M6_flow_matching_prediction.md
│       ├── M7_learned_planner_dagger.md
│       ├── M8_forward_sim_selector.md
│       └── M9_ilqr.md
│
├── configs/                               # runtime configuration (YAML)
│   ├── profiles/                          # one per launch profile
│   │   ├── m0_gt_all.yaml
│   │   ├── m1_classical.yaml
│   │   ├── m3_learned_perception.yaml
│   │   ├── m4_no_gt.yaml
│   │   ├── m6_learned_prediction.yaml
│   │   ├── m7_learned_planner.yaml
│   │   └── leaderboard.yaml
│   ├── gt_toggles/                        # small YAMLs setting use_gt.* flags
│   ├── sensors/                           # sensor rigs (CARLA blueprint attrs + extrinsics)
│   │   ├── rig_dev.json
│   │   └── rig_leaderboard.json
│   ├── vehicle/
│   │   └── lincoln_mkz_2020.yaml          # sysid results (M0)
│   ├── planning/
│   ├── control/
│   ├── perception/
│   ├── localization/
│   └── training/                          # hydra/omegaconf configs for ml/
│
├── ros2_ws/
│   ├── colcon_defaults.yaml               # Ninja, ccache, mold, compile_commands, RelWithDebInfo
│   └── src/
│       ├── nuway_cmake/                   # shared CMake: warnings, -Werror, NUWAY_CLANG_TIDY/SANITIZE/LTO
│       ├── gtsam_vendor/                  # FetchContent, pinned tag+hash (M4); same pattern for osqp_vendor, nanoflann_vendor
│       ├── nuway_msgs/                    # ALL custom messages. No msgs elsewhere.
│       │   ├── msg/
│       │   └── CMakeLists.txt
│       ├── nuway_common/                  # C++ header-mostly library
│       │   ├── include/nuway_common/
│       │   │   ├── geometry.hpp           # SE2, SE3 helpers, angle wrap
│       │   │   ├── frenet.hpp             # reference line, cartesian<->frenet
│       │   │   ├── bicycle_model.hpp      # kinematic bicycle, discretization, jacobians
│       │   │   ├── trajectory.hpp         # Trajectory struct, resampling, interpolation
│       │   │   ├── carla_conv.hpp         # left-handed <-> ROS conversion (the ONLY place)
│       │   │   ├── occupancy.hpp          # multi-channel grid accessor, bilinear sample
│       │   │   ├── diag.hpp               # NodeDiag publisher helper, scoped timer
│       │   │   └── params.hpp             # declare/get param helpers
│       │   └── test/
│       ├── nuway_carla_bridge/            # python (rclpy) — talks to CARLA Python API
│       │   ├── nuway_carla_bridge/
│       │   │   ├── world_manager.py       # spawns hero+sensors, owns world.tick(), /clock
│       │   │   ├── gt_publisher.py        # GT agents, GT ego pose/odom, traffic lights
│       │   │   ├── control_adapter.py     # ControlCommand -> CarlaEgoVehicleControl
│       │   │   └── sensor_rig.py          # loads configs/sensors/*.json
│       │   └── launch/
│       ├── nuway_map/                     # C++
│       │   ├── src/
│       │   │   ├── opendrive_parser.cpp   # OpenDRIVE -> LaneGraph
│       │   │   ├── lane_graph.cpp         # lanes, successors, neighbors, TL/stop associations
│       │   │   └── map_server_node.cpp    # publishes nuway_msgs/LaneGraph (latched), query srv
│       │   └── include/nuway_map/
│       ├── nuway_route/                   # C++: A* on lane graph, reference line builder
│       ├── nuway_localization/            # C++
│       │   ├── src/
│       │   │   ├── gt_pose_node.cpp       # cheat twin (M0)
│       │   │   ├── lidar_odometry_node.cpp        # M4
│       │   │   ├── scan_to_map_node.cpp           # M4
│       │   │   ├── smoother_node.cpp              # M4 (GTSAM fixed-lag)
│       │   │   └── pose_extrapolator_node.cpp     # M4 (high-rate odom->base_link)
│       │   └── tools/                     # offline mapping binaries
│       ├── nuway_perception/
│       │   ├── src/gt_perception_node.cpp # cheat twin: AgentArray + OccupancyGridMC from GT (M0/M2)
│       │   ├── src/lidar_preproc_node.cpp # voxelize on GPU, publishes shared tensor handle (M3)
│       │   ├── nuway_perception/          # python
│       │   │   ├── bevfusion_node.py      # inference (M3)
│       │   │   ├── tracker.py             # velocity-projected NN association (M3)
│       │   │   └── traffic_light_node.py  # (M3)
│       │   └── launch/
│       ├── nuway_prediction/
│       │   ├── src/const_vel_node.cpp     # M1
│       │   ├── src/gt_prediction_node.cpp # cheat twin: future from CARLA log replay (eval only)
│       │   └── nuway_prediction/flow_matching_node.py   # M6
│       ├── nuway_planning/                # C++ except learned_planner_node.py
│       │   ├── src/
│       │   │   ├── behavior_fsm.cpp / behavior_fsm_node.cpp      # M1
│       │   │   ├── lattice_sampler.cpp                           # M1
│       │   │   ├── piecewise_jerk_qp.cpp                         # M1
│       │   │   ├── rule_selector.cpp                             # M1
│       │   │   ├── safety_layer_node.cpp                         # M1
│       │   │   ├── planner_node.cpp        # orchestrates: candidates -> refine -> select
│       │   │   ├── forward_sim_scorer.cpp  # M8
│       │   │   └── ilqr.cpp                # M9
│       │   ├── nuway_planning/learned_planner_node.py            # M7
│       │   └── include/nuway_planning/
│       ├── nuway_control/                 # C++
│       │   ├── src/
│       │   │   ├── pure_pursuit_pid_node.cpp   # M0
│       │   │   ├── mpc_node.cpp                # M1
│       │   │   └── longitudinal_map.cpp        # a_des -> throttle/brake from sysid table
│       ├── nuway_eval/                    # python: route runner, metrics, infraction detection
│       │   ├── nuway_eval/
│       │   │   ├── route_runner.py
│       │   │   ├── infractions.py
│       │   │   ├── driving_score.py
│       │   │   └── report.py
│       │   └── routes/                    # route XMLs (leaderboard format)
│       ├── nuway_viz/                     # marker publishers, foxglove layouts, rviz configs
│       └── nuway_bringup/
│           ├── launch/stack.launch.py     # THE launch file; reads a profile YAML
│           └── launch/*.launch.py         # per-subsystem launches used by stack.launch.py
│
├── ml/                                    # uv workspace member `nuway-ml` (hatchling)
│   ├── pyproject.toml                     # package metadata + runtime deps only; no [tool.ruff]/[tool.mypy] here
│   ├── nuway_ml/
│   │   ├── common/
│   │   │   ├── geometry.py                # numpy/torch SE2 utils, must mirror nuway_common
│   │   │   ├── occupancy.py               # grid spec, world<->grid, bilinear sample (torch)
│   │   │   ├── frames.py                  # frame conventions shared with ROS side
│   │   │   └── schema.py                  # dataset record schema (dataclasses + validation)
│   │   ├── data/
│   │   │   ├── webdataset_io.py           # shard writer/reader
│   │   │   ├── perception_dataset.py      # M2
│   │   │   ├── planning_dataset.py        # M5
│   │   │   ├── augment.py                 # occupancy noise, agent dropout (M5/M7)
│   │   │   └── gt_occupancy.py            # GT occupancy generator (shared with ROS cheat node via CLI)
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
│   │   │   ├── metrics.py                 # mAP, occupancy IoU
│   │   │   └── train.py
│   │   ├── prediction/                    # M6
│   │   │   ├── tokenizer.py               # scene -> tokens
│   │   │   ├── scene_encoder.py
│   │   │   ├── relpe.py                   # pairwise relative positional encoding
│   │   │   ├── deform_attn.py
│   │   │   ├── velocity_net.py
│   │   │   ├── flow_matching.py           # train step, sampler, guidance
│   │   │   ├── aux_losses.py
│   │   │   ├── metrics.py                 # ADE/FDE/minADE, collision rate
│   │   │   └── train.py
│   │   ├── planning/                      # M7
│   │   │   ├── planning_head.py
│   │   │   ├── dagger.py
│   │   │   └── train.py
│   │   └── export/                        # torchscript / tensorrt export helpers
│   └── tests/
│
├── tools/
│   ├── carla/
│   │   ├── start_carla.sh
│   │   └── check_native_ros2.py           # M0 smoke test
│   ├── sysid/                             # M0
│   │   ├── run_sweeps.py
│   │   └── fit_models.py
│   ├── collect/                           # data collection clients
│   │   ├── collect_perception.py          # M2 (rendered)
│   │   ├── collect_planning.py            # M5 (no rendering)
│   │   ├── expert/                        # privileged expert planner (M5)
│   │   └── scenarios/                     # scenario/route generation helpers
│   ├── mapping/                           # M4 offline map building
│   ├── eval/
│   │   ├── run_routes.py                  # entry point for evaluation harness
│   │   └── compare_runs.py
│   ├── lint/                              # format_cpp.sh, tidy_cpp.sh, lint_py.sh, merge_compile_commands.py, header guard check (M0)
│   └── viz/
│
├── data/                                  # gitignored
│   ├── raw/
│   ├── shards/
│   ├── maps/                              # point cloud maps per town (M4)
│   ├── checkpoints/
│   └── eval_runs/
│
├── tests/                                 # cross-cutting integration tests
│   └── integration/
├── .clang-format                          # BasedOnStyle: Google (see docs/03)
├── .clang-tidy                            # google-* + naming checks, warnings are errors (see docs/03)
├── .pre-commit-config.yaml                # local hooks via `uv run`: clang-format, clang-tidy, gersemi, ruff, mypy
├── .clangd                                # points at merged ros2_ws/build/compile_commands.json
├── pyproject.toml                         # uv workspace root: dependency groups, ruff, mypy, pytest config
├── uv.lock                                # committed; the only place versions are pinned
├── .python-version                        # 3.10
├── .gitignore
└── setup_env.sh                           # source in every shell: ROS, colcon defaults, uv venv, pre-commit
```

## Rules

- **`nuway_msgs` is the only package that defines messages.** Any new message requires an entry in `02_interfaces.md`.
- **`nuway_common/carla_conv.hpp` and `nuway_carla_bridge` are the only places that know CARLA's coordinate convention.** Everything downstream is ROS-convention.
- **`nuway_ml/common/*` must mirror `nuway_common/*`** for geometry/occupancy conventions. There is a cross-language test (`tests/integration/test_geometry_parity.py`) that runs the C++ versions through a small pybind shim and compares outputs. Keep it green.
- One node per file, one file per node. Nodes are thin: they parse params, subscribe/publish, and call a library class that is unit-tested without ROS.
- Python packages inside `ros2_ws` only contain nodes and thin glue. Model code lives in `ml/nuway_ml` and is imported.
- Config YAML keys are namespaced by node name. Do not read environment variables in nodes.
- Python dependencies enter only through `uv add` (lock updated in the same commit). C++ dependencies enter only through `package.xml` + rosdep, or a `*_vendor` package. No `pip`, no submodules, no system-wide installs. See `03_style_and_conventions.md` §6.
- All C++ is Google style and must pass `clang-format` and `clang-tidy`; all Python is PEP 8 and must pass `ruff format`, `ruff check` and `mypy`, with the root configs. See `03_style_and_conventions.md`.
