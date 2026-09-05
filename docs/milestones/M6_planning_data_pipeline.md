# M6 — Planning data pipeline (privileged expert + render-free collection)

**Goal:** produce a large prediction/planning dataset (millions of frames) with a high-quality expert as planning teacher, collected without rendering at high throughput; add planner-side augmentation and an open-loop evaluation to validate the data before M7/M8 depend on it.

**Completion criteria**
- [ ] ≥ 3M frames (10 Hz) across ≥ 8 towns, all weathers (weather doesn't matter for state-only data but is logged), 3 traffic densities, scenario-runner-style scenarios (see §4).
- [ ] Expert (privileged, GT state) achieves driving score ≥ 85 on the M1 protocol (10 routes × 3 weathers, Town03/05) and ≥ 70 on the Leaderboard-2.0-style scenario routes in §4.
- [ ] Open-loop check: a small model (the M7 encoder with a regression head, trained for 1 day) reaches minADE@3s (K=6) < 0.6 m for vehicles, and beats constant-velocity ADE@3s by ≥ 40%.
- [ ] `PlanningDataset` throughput ≥ 2000 frames/s with 8 workers.
- [ ] Data sanity report: label coverage (fraction of agents with full 8 s future), expert intent histogram, ego speed/accel distributions, collision-free fraction of expert runs.

---

## 1. Collector (`tools/collect/collect_planning.py`)

- CARLA client, sync mode, `world.settings.no_rendering_mode = True`, no sensors except a semantic LiDAR **only if** occupancy labels are needed from sensing; default: **occupancy from GT geometry** (see §3) so no sensors are needed at all.
- Hero driven by the expert (§2). Other agents by Traffic Manager (varied aggressiveness per run: `vehicle_percentage_speed_difference` ∈ [−20, 30], `distance_to_leading_vehicle` ∈ [1.5, 4]).
- Per tick (20 Hz) log to an in-memory buffer; every 2 ticks emit a `PlanningFrame`; flush shards every 1000 frames. Throughput target ≥ 300 ticks/s → a 300 s run in ~20 s wall clock.
- 8 parallel collector processes against 8 CARLA server instances (ports 2000, 2002, …) via `tools/collect/launch_farm.sh`; each server at `-RenderOffScreen -nullrhi` (no GPU use at all in no-rendering mode).

## 2. Privileged expert (`tools/collect/expert/`)

A rule-based planner with GT access, in the spirit of PDM-Lite, reusing M1 components where possible (import the C++ lattice/QP via a pybind module `nuway_planning_py`, built in M6; or reimplement a simplified version in numpy — **decision: pybind the M1 planner**, so the expert and the runtime fallback share behavior).

Expert = M1 stack (FSM + lattice + QP + rule selector) with these privileged replacements:
- Perception: GT agents (omniscient, no visibility filter), GT traffic lights.
- Prediction: **GT future** for other agents is *not* used (it would make the expert clairvoyant in ways a student can't imitate). Instead: constant-velocity + lane-follow (M1's `const_vel` logic) **plus** knowledge of Traffic Manager intent is *not* used. Keep the expert honest: only current GT state.
- Additional privileged heuristics (tuned for Leaderboard 2.0 scenarios): handling of `ConstructionObstacle`, `Accident`, `ParkedObstacle` (lane change into opposing lane when clear via oncoming-traffic gap check), `HazardAtSideLane`, door-opening, emergency vehicle yielding, pedestrians crossing (stop if predicted to enter lane), and `EnterActorFlow`/`MergerIntoSlowTraffic`. Each is a behavior FSM extension gated by a config `expert_extensions: true` — those extensions are also available to the runtime fallback later.
- Expert intent is exported per frame: `BehaviorDecision` fields + which extension fired.

Expert must be deterministic given the seed.

## 3. Record schema (`schema.py` additions)

```python
@dataclass
class PlanningFrame:
    key, timestamp, town, weather, run_id, frame_idx
    ego: EgoRecord             # pose_map [4,4], v, a, yaw_rate, steering, history [20,3] (map), future [80,3] (map), future_v [80]
    expert: ExpertRecord       # decision fields, selected trajectory [80,3], candidate count, cost breakdown of selected
    agents: list[AgentLabel]   # as M2 (visible=True always; a separate `visible_hero_view` flag computed by a cheap geometric raycast against agent boxes and map buildings — see below)
    map_local: MapLocal        # lanes within 100 m: ids, polylines in map frame (the encoder tokenizes on the fly; storing ids avoids storing polylines per frame → store lane ids + use the per-town LaneGraph cache)
    route: RouteRecord         # reference line window [-20, +150] m: s, xy, heading, curvature, speed_limit, bounds
    traffic_lights: list[TLLabel]   # states + stop lines + affected lanes (no crops)
    occupancy_static: np.ndarray    # [6,200,200] float16 generated from GT geometry (§3.1)
```

### 3.1 Occupancy without sensors
`gt_occupancy.generate_from_geometry(spec, static_meshes, agents, lane_polygons, ego_pose)`:
- `occupied`: rasterized footprints of static props/parked vehicles (static agents) + building/wall/fence footprints from a **per-town static occupancy map** built once from CARLA's semantic LiDAR during M2 collection (`data/maps/<town>/static_occ.npz`, 0.25 m, map frame), cropped and rotated into `base_link`.
- `free/unknown`: a 2-D raycast from the LiDAR origin over the `occupied` raster at 1° steps (visibility emulation). Cheap (numpy) and gives realistic unknown regions behind obstacles/agents.
- `drivable`, `dynamic`, `height_max` as in M2 (height from the static map).
- `visible_hero_view` per agent: the same 2-D raycast test hitting the agent box before any occluder.

This keeps M6 frames consistent with what the M3 perception would plausibly output, without rendering.

## 4. Scenario coverage

Beyond Traffic Manager traffic, generate Leaderboard-2.0-style scenarios with in-repo lightweight scripting (`tools/collect/scenarios/`) triggered along routes: construction zones (cone rows), parked/accident vehicles blocking the lane, pedestrians crossing from behind parked cars, cut-ins, hard braking lead vehicle, opposite-direction overtakes, yield to emergency vehicle. Each scenario is a small class with `spawn(world, ego_wp)` and `tick()`. Use scenario_runner's route XMLs only as a source of trigger positions if convenient; do not depend on scenario_runner at runtime.

Route sets: reuse `nuway_eval/routes/*` plus `collect_long_<town>.xml` (10–20 km random routes).

## 5. Noise injection for planning targets (recorded, not synthetic)

ChauffeurNet-style perturbation is *recorded from the expert*, not synthesized:
- With probability 0.1 per 10 s window, the collector teleports the hero laterally by ±[0.3, 1.2] m and/or yaw ±[3°, 12°] (small enough that CARLA physics tolerates it: use `set_transform` at low speed only, or apply a steering disturbance for 0.5 s at any speed) and records the expert's recovery. Frames within the disturbance window are tagged `perturbed=true` so training can weight them.

## 6. Dataset loader & augmentation

`PlanningDataset` yields token-ready tensors (see M7 §2 for the tokenizer interface): agent histories `[A,20,C]`, map polylines `[M,20,C]` (resolved from per-town LaneGraph cache by id), route `[R,C]`, TL tokens, occupancy `[6,200,200]`, ego/agent futures, masks. Normalization into ego frame at `t0` happens in the loader (`frames.py`).

`augment.py` (planning part):
- Occupancy noise: cell dropout 0–15%, Gaussian blur σ ∈ [0, 1] cell, probability squash, random "unknown" wedges.
- Agent dropout 0–10% (visible agents), position jitter σ = 0.15 m, velocity jitter σ = 0.3 m/s, history truncation (random `history_len` ≥ 5).
- Random SE(2) of the whole scene (only for models without built-in invariance tests; kept as an option).
Level of noise is scheduled by the training config so M8's DAgger can turn it down as real perception data enters the mix.

## 7. Open-loop evaluation (`nuway_ml/prediction/metrics.py`, `ml/scripts/eval_openloop.py`)

minADE/minFDE (K = 6) at 3 s / 8 s per class, miss rate @2 m, collision rate between predicted samples of different agents, offroad rate; for ego: L2 to expert at 1/2/3 s (nuScenes-style) and comfort stats. Baselines: constant velocity, lane-follow constant velocity. Report to `data/eval_runs/<run>/openloop.md`.

## 8. Task list

1. [ ] `nuway_planning_py` pybind module exposing FSM, lattice, QP, selector, collision checker, Frenet utils.
2. [ ] Expert with extensions; deterministic; eval on M1 protocol (≥ 85) and scenario routes (≥ 70). Fix M1 planner bugs found here (they are shared).
3. [ ] Per-town `static_occ.npz` builder (from M2 semantic LiDAR shards or a dedicated mapping run).
4. [ ] `generate_from_geometry` + raycast visibility + tests; consistency test vs M2 sensor-derived occupancy on 100 aligned frames (IoU of `occupied` > 0.7).
5. [ ] Scenario library (≥ 8 scenario types) + unit smoke tests (each spawns and ticks 100 steps).
6. [ ] `collect_planning.py`, `launch_farm.sh`, resume/manifest; perturbation recording.
7. [ ] `PlanningDataset`, `augment.py` (planning), throughput test.
8. [ ] `metrics.py`, `eval_openloop.py`, constant-velocity baselines.
9. [ ] Train the M7 encoder + regression head (K=6 modes, WTA) as the data-validation model; report.
10. [ ] Sanity report; fix schema gaps **before** M7 starts.

## 9. Decisions log

- (2026-09-02) Expert = M1 planner + privileged inputs + scenario extensions, exposed via pybind. One planner codebase.
- (2026-09-02) No rendering for planning data; occupancy emulated from a static map + 2-D raycast.
