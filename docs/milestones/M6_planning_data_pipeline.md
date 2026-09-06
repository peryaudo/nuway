# M6 — Planning data pipeline (privileged expert + render-free collection)

**Goal:** produce a large prediction/planning dataset (millions of frames) with a high-quality expert as planning teacher, collected without rendering at high throughput; add planner-side augmentation and an open-loop evaluation to validate the data before M7/M8 depend on it; deliver the prediction and planning ground-truth twins promised by `00_overview.md` principle 1.

**Completion criteria**
- [ ] ≥ 3M frames (10 Hz) across ≥ 8 towns, all weathers (weather doesn't matter for state-only data but is logged), 3 traffic densities, scenario-runner-style scenarios (see §4).
- [ ] Expert (privileged, GT state) achieves driving score ≥ 85 on the M1 protocol (10 routes × 3 weathers, Town03/05) and ≥ 70 on the Leaderboard-2.0-style scenario routes in §4.
- [ ] Open-loop baselines computed on the held-out split: constant-velocity and lane-follow constant-velocity minADE/minFDE at 3 s / 8 s per class, reported in `openloop.md`. (The learned data-validation model that earlier drafts required here is dropped: it needed the M7 encoder before M7 exists. M7's first criterion now covers it.)
- [ ] `PlanningDataset` throughput ≥ 2000 frames/s with 8 workers, including on-the-fly occupancy generation (§3.1).
- [ ] Data sanity report: label coverage (fraction of agents with full 8 s future), expert intent histogram, ego speed/accel distributions, collision-free fraction of expert runs, schema validation of every shard.
- [ ] `gt_prediction_node` and `gt_planning_node` exist, are selected by `use_gt.prediction` / `use_gt.planning`, and the M1 protocol with `use_gt.planning: true` reproduces the expert's driving score (±2) through the ROS stack.

---

## 1. Collector (`tools/collect/collect_planning.py`)

- CARLA client, sync mode, `world.settings.no_rendering_mode = True`, no sensors: **occupancy from GT geometry** (see §3), so nothing is rendered at all.
- Hero driven by the expert (§2). Other agents by Traffic Manager (varied aggressiveness per run: `vehicle_percentage_speed_difference` ∈ [−20, 30], `distance_to_leading_vehicle` ∈ [1.5, 4]).
- Per tick (20 Hz) log to an in-memory buffer; every 2 ticks emit a `PlanningFrame`; flush shards every 1000 frames. Throughput target ≥ 150 ticks/s per server with 60 traffic vehicles (to be measured in task 6 and recorded here; the Traffic Manager, not rendering, is the bottleneck in no-rendering mode).
- `N` parallel collector processes against `N` CARLA server instances (ports 2000, 2002, …) via `tools/collect/launch_farm.sh`, each server started with `-RenderOffScreen -quality-level=Low` and `no_rendering_mode` switched on by the client. CARLA still needs a working RHI to start, so `-nullrhi` is **not** used. `N = floor(cores / 6)`, i.e. **2** on the reference 12-core workstation; the script refuses to oversubscribe. At 2 servers × 150 ticks/s the 3M-frame target (6M ticks) takes ≈ 6 h.

## 2. Privileged expert (`tools/collect/expert/`)

A rule-based planner with GT access, in the spirit of PDM-Lite, reusing M1 components where possible (import the C++ lattice/QP via a pybind module `nuway_planning_py`, built in M6; or reimplement a simplified version in numpy — **decision: pybind the M1 planner**, so the expert and the runtime fallback share behavior).

Expert = M1 stack (FSM + lattice + QP + rule selector) with these privileged replacements:
- Perception: GT agents (omniscient, no visibility filter), GT traffic lights.
- Prediction: the expert uses M1's constant-velocity + lane-follow prediction from the *current* GT state only. It does **not** read other agents' GT futures and it does **not** read Traffic Manager intent: either would make it clairvoyant in ways a student cannot imitate.
- Additional privileged heuristics (tuned for Leaderboard 2.0 scenarios): handling of `ConstructionObstacle`, `Accident`, `ParkedObstacle` (lane change into opposing lane when clear via oncoming-traffic gap check), `HazardAtSideLane`, door-opening, emergency vehicle yielding, pedestrians crossing (stop if predicted to enter lane), and `EnterActorFlow`/`MergerIntoSlowTraffic`. Each is a behavior FSM extension gated by the profile key `planning.expert_extensions` and written against `AgentArray`/`OccupancyGridMC`/`TrafficLightArray` only, so the same code runs on perceived inputs in the runtime fallback later (principle 3).
- Expert intent is exported per frame: `BehaviorDecision` fields + which extension fired.

Expert must be deterministic given the seed.

### 2.1 GT twins for prediction and planning

- `nuway_planning/gt_planning_node.py` wraps the expert (through `nuway_planning_py`) as a ROS node: subscribes `/nuway/gt/agents`, `/nuway/gt/traffic_lights`, `/nuway/loc/pose`, the lane graph and reference line; publishes `/nuway/planning/behavior`, `/nuway/planning/candidates` and `/nuway/planning/trajectory` with `source="gt"`. Selected by `use_gt.planning: true`; the safety layer and MPC downstream are unchanged. Its purpose is ablation (how much of a closed-loop loss is planning versus perception) and DAgger labelling in-stack (M8).
- `nuway_prediction/gt_prediction_node.cpp` publishes `PredictionSamples` (S = 1, weight 1) by reading each agent's *actual* future from a recorded run. It can only run in **log replay** (`tools/eval/run_routes.py --replay <mcap>`, which feeds the recorded `/nuway/gt/agents` stream with an 8 s lookahead); in a live simulation the future does not exist yet, and the launch file refuses `use_gt.prediction: true` without `--replay`. Its purpose is the upper bound for prediction-dependent metrics in open-loop planner evaluation.

## 3. Record schema (`schema.py` additions)

```python
@dataclass(frozen=True, slots=True)
class PlanningFrame:
    key, timestamp, town, weather, run_id, frame_idx
    ego: EgoRecord             # pose_map [4,4], v, a, yaw_rate, steering, history [20,3] (map), future [80,3] (map), future_v [80]
    expert: ExpertRecord       # decision fields, selected trajectory [80,3], candidate count, cost breakdown of selected
    agents: list[AgentLabel]   # as M2. `visible` is the 2-D raycast result of §3.1 (same meaning as M2's sensor-based
                               # flag: "the hero could plausibly perceive this agent"), so M7 reads one flag for both datasets
    map_local: MapLocal        # lanes within 100 m: ids only (resolved from the per-town LaneGraph cache by the loader)
    route: RouteRecord         # reference line window [-20, +150] m: s, xy, heading, curvature, speed_limit, bounds
    traffic_lights: list[TLLabel]   # states + stop lines + affected lanes (no crops)
    perturbed: bool            # §5
    # No occupancy array is stored: it is regenerated in the loader (§3.1). 3M × [6,200,200] float16 would be ≈ 1.4 TB.
```

### 3.1 Occupancy without sensors
`gt_occupancy.generate_from_geometry(spec, static_occ, agents, lane_polygons, ego_pose)`:
- `occupied`: rasterized footprints of static props/parked vehicles (static agents) + building/wall/fence footprints from a **per-town static occupancy map** `data/maps/<town>/static_occ.npz` (0.25 m, map frame), built by `tools/mapping/build_static_occ.py` from the M5 semantic point-cloud map (`map.ply` + `map_tags.npy`), which covers every driving lane by construction. Cropped and rotated into `base_link`.
- `free/unknown`: a 2-D raycast from the LiDAR origin over the `occupied` raster at 1° steps (visibility emulation). Cheap (numpy) and gives realistic unknown regions behind obstacles/agents.
- `drivable`, `dynamic`, `height_max` as in M2 (height from the static map).
- `visible` per agent: the same 2-D raycast test hitting the agent box before any occluder.

The function is deterministic in `(ego_pose, agents, town)`, so `PlanningDataset` calls it in the worker instead of reading a stored array; the 2000 frames/s criterion includes this cost (vectorised numpy raycast, ≈ 1 ms/frame). A `--materialize` flag writes `{key}.occ.npy` for a subset when a training run wants to trade disk for CPU.

This keeps M6 frames consistent with what the M3 perception would plausibly output, without rendering.

## 4. Scenario coverage

Beyond Traffic Manager traffic, generate Leaderboard-2.0-style scenarios with in-repo lightweight scripting (`tools/collect/scenarios/`) triggered along routes: construction zones (cone rows), parked/accident vehicles blocking the lane, pedestrians crossing from behind parked cars, cut-ins, hard braking lead vehicle, opposite-direction overtakes, yield to emergency vehicle. Each scenario is a small class with `spawn(world, ego_wp)` and `tick()`. Use scenario_runner's route XMLs only as a source of trigger positions if convenient; do not depend on scenario_runner at runtime.

Route sets: reuse `nuway_eval/routes/*` plus `collect_long_<town>.xml` (10–20 km random routes).

## 5. Noise injection for planning targets (recorded, not synthetic)

ChauffeurNet-style perturbation is *recorded from the expert*, not synthesized:
- With probability 0.1 per 10 s window, the collector teleports the hero laterally by ±[0.3, 1.2] m and/or yaw ±[3°, 12°] (small enough that CARLA physics tolerates it: use `set_transform` at low speed only, or apply a steering disturbance for 0.5 s at any speed) and records the expert's recovery. Frames within the disturbance window are tagged `perturbed=true` so training can weight them.

## 6. Dataset loader & augmentation

`PlanningDataset` yields token-ready tensors (see M7 §2 for the tokenizer interface): agent histories `[A,20,C]`, map polylines `[M,20,C]` (resolved from per-town LaneGraph cache by id), route `[R,C]`, TL tokens, occupancy `[6,200,200]` (generated per §3.1), ego/agent futures, masks. Normalization into ego frame at `t0` happens in the loader (`frames.py`). The tokenizer itself is M7 code; M6 defines the record → tensor contract and ships the loader with a stub tokenizer that emits the raw padded arrays.

`augment.py` (planning part):
- Occupancy noise: cell dropout 0–15%, Gaussian blur σ ∈ [0, 1] cell, probability squash, random "unknown" wedges.
- Agent dropout 0–10% (visible agents), position jitter σ = 0.15 m, velocity jitter σ = 0.3 m/s, history truncation (random `history_len` ≥ 5).
- Random SE(2) of the whole scene (only for models without built-in invariance tests; kept as an option).
Level of noise is scheduled by the training config (the `data/augment` Hydra group, `03_style_and_conventions.md` §9.7) so M8's DAgger can turn it down per round with an override as real perception data enters the mix.

## 7. Open-loop evaluation (`nuway_ml/prediction/metrics.py`, `ml/scripts/eval_openloop.py`)

minADE/minFDE (K = 6) at 3 s / 8 s per class, miss rate @2 m, collision rate between predicted samples of different agents, offroad rate; for ego: L2 to expert at 1/2/3 s (nuScenes-style) and comfort stats. Baselines: constant velocity, lane-follow constant velocity. Report to `data/eval_runs/<run>/openloop.md`.

## 8. Task list

1. [ ] `nuway_planning_py` pybind module exposing FSM, lattice, QP, selector, collision checker, Frenet utils.
2. [ ] Expert with extensions; deterministic; eval on M1 protocol (≥ 85) and scenario routes (≥ 70). Fix M1 planner bugs found here (they are shared).
3. [ ] `gt_planning_node.py` (+ `use_gt.planning` launch wiring) and `gt_prediction_node.cpp` (+ `--replay` mode in `run_routes.py`, launch refusal outside replay); M1 protocol with `use_gt.planning: true` reproduces the expert score.
4. [ ] `tools/mapping/build_static_occ.py`: per-town `static_occ.npz` from the M5 semantic map.
5. [ ] `generate_from_geometry` + raycast visibility + tests; consistency test vs M2 sensor-derived occupancy on 100 aligned frames (IoU of `occupied` > 0.7); per-frame generation time ≤ 1 ms.
6. [ ] Scenario library (≥ 8 scenario types) + unit smoke tests (each spawns and ticks 100 steps).
7. [ ] `collect_planning.py`, `launch_farm.sh` (core-count guard), resume/manifest; perturbation recording; measure and record ticks/s per server.
8. [ ] `PlanningDataset` (on-the-fly occupancy, `--materialize`), `augment.py` (planning), throughput test.
9. [ ] `metrics.py`, `eval_openloop.py`, constant-velocity and lane-follow baselines; `openloop.md`.
10. [ ] Sanity report; fix schema gaps **before** M7 starts.

## 9. Decisions log

- (2026-09-02) Expert = M1 planner + privileged inputs + scenario extensions, exposed via pybind. One planner codebase.
- (2026-09-02) No rendering for planning data; occupancy emulated from a static map + 2-D raycast.
- (2026-09-05) The learned data-validation model was dropped from the completion criteria: it depended on M7's encoder, which made M6 circular. M7's open-loop criterion validates the data instead.
- (2026-09-05) Occupancy is not stored in planning shards; it is regenerated deterministically in the loader from the static map and the agent list. Saves ≈ 1.4 TB.
- (2026-09-05) Static occupancy comes from the M5 semantic map, not from M2 shards: M5 now precedes M6, its mapping routes cover every lane, and the Traffic-Manager-driven M2 hero did not.
- (2026-09-05) The prediction and planning GT twins live here because both need the expert and the recorded futures that M6 produces.
