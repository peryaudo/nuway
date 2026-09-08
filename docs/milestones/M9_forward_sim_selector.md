# M9 — Forward-simulation selector

**Goal:** replace the purely rule-based scoring of candidates with a closed-loop forward simulation of each candidate: the ego tracks the candidate with a simple controller in a lightweight simulator while other agents follow their predicted samples (or a reactive IDM fallback), and the outcome (collisions, progress, comfort, rule compliance) is scored. Two-stage selection: cheap rule scoring → top-K → forward-sim → final choice.

**Completion criteria**
- [ ] Driving score improves over M8 (same protocol, same checkpoints) by a measurable margin (≥ 3 points) **or** the collision rate drops by ≥ 25% at equal or better route completion.
- [ ] Forward simulation of K=16 candidates × S=16 prediction samples × 50 steps ≤ 8 ms p50 on CPU (C++/Eigen in `planner_node`, batched over candidates, 4 threads). The torch reference implementation is for tests and is not a runtime path (`00_overview.md` principle 6).
- [ ] Selector is deterministic; no chattering regression (mode switch rate ≤ M8).
- [ ] Ablation table in the report: rule-only vs forward-sim, with predicted-sample agents vs reactive-IDM agents.

---

## 1. Design

```
candidates (lattice + refined learned) ─► rule_selector (M1) → costs
                                          top-K (K=16) by rule cost
                                             │
                                             ▼
                                   ForwardSimScorer
                                     for each candidate c, for each prediction sample s:
                                        roll out 5 s @ 0.1 s:
                                          ego: kinematic bicycle + pure-pursuit/PID tracking candidate c
                                          others: follow sample s trajectories (position lookup)
                                                  OR reactive IDM along lane (mode B)
                                        metrics: collision (first t), min distance, progress s(5s), offroad, red-light crossing, |a|, |jerk|, |a_lat|
                                     aggregate over s with sample weights (mean cost; collision = any-sample weighted)
                                             │
                                             ▼
                                   final cost = w_rule · rule_cost + w_sim · sim_cost
                                   argmin → selected
```

## 2. `ForwardSimScorer` (`nuway_planning/src/forward_sim_scorer.cc`, with a torch reference impl in `nuway_ml/planning/forward_sim.py` for tests and for possible RL later)

### 2.1 Ego closed-loop model
- Kinematic bicycle (`bicycle_model.h`), `dt = 0.1`, steering lag `τ_steer`, accel lag `τ_throttle` from the vehicle YAML.
- Tracking controller: pure pursuit (lookahead `max(3, 0.6·v)`) for steering + P-controller on speed error to the candidate's `v(t)`, clipped by limits. Deliberately simple: the point is to measure what a *feasible* execution of the candidate does, not to replicate MPC exactly.

### 2.2 Other agents
- **Mode A (default):** positions/yaws from the prediction sample `s` at each `t` (interpolate 0.5 s → 0.1 s).
- **Mode B (reactive):** IDM along the agent's lane (from `LaneGraph::NearestLane` + `Successors`) with the ego treated as a potential leader (so agents brake for the ego); walkers constant velocity. Used when `prediction` is missing or when configured for ablations. Mode B implements the "others react to me" assumption; Mode A the "others follow their forecast" assumption. Config `forward_sim.agent_mode: sample | reactive | mix` (mix = average of both costs).

### 2.3 Metrics per rollout
| metric | definition |
|--------|-----------|
| collision | first `t` with ego–agent OBB overlap (inflated 0.3 m); `inf` if none |
| min_dist | min over t of center distance minus radii (disc approx) |
| progress | `s(5 s) − s(0)` along the reference line |
| offroad | Σ_t 𝟙[ego footprint corner outside drivable bounds] (uses reference-line bounds; occupancy `occupied` sampled at footprint corners ⇒ also counted as collision) |
| rule | red-light stop-line crossing while red/yellow, stop-sign non-compliance |
| comfort | mean |a|, mean |jerk|, max |a_lat| |
| tracking | mean distance between simulated ego and the candidate (large ⇒ candidate infeasible for the tracker) |

### 2.4 Aggregation
```
sim_cost(c) = Σ_s w_s · [ w_col · 𝟙[collision_s < ∞] · (1 + (5 − t_col)/5)
                       + w_ttc · max(0, 1 − t_col/4)
                       + w_prox · max(0, 1 − min_dist/3)
                       + w_prog · (1 − progress / (v_limit · 5))
                       + w_off · offroad + w_rule · rule + w_comf · comfort + w_track · tracking ]
```
Defaults: `w_col=1000, w_ttc=50, w_prox=10, w_prog=30, w_off=200, w_rule=500, w_comf=5, w_track=20`. Combine with rule cost: `w_rule_total = 0.3`, `w_sim = 1.0` (rule cost still carries lateral-deviation, consistency, source bias).

### 2.5 Implementation
- C++: batched over candidates in a struct-of-arrays layout; agents' sample trajectories pre-interpolated once per cycle; OBB checks via SAT with early-out on distance. The ego rollout depends only on the candidate, never on the sample (Mode A agents do not react, and Mode B agents react to the ego, not to a sample), so there are **K ego rollouts, not K × S**; only the metric pass runs per (candidate, sample). That is what makes the 8 ms budget credible: 16 rollouts of 50 steps plus 16 × 16 × 50 × ≤ 32 distance-gated OBB checks. Multi-threaded over candidates (`std::thread` pool, 4 threads): each candidate is scored entirely by one thread and the final aggregation runs in candidate order on the calling thread, so the result is independent of the thread count and of scheduling (M1 §5).
- Torch reference (`forward_sim.py`): fully vectorized `[K, S, T]` rollout; used in `tests/` for parity (max metric deviation < 1e-3) and as the base for any future learned/RL selector.

## 3. Two-stage selection in `planner_node`

1. Rule cost for all candidates (as before).
2. Drop candidates with hard infeasibility (collision at t < 1 s with the marginal prediction, bounds violation).
3. Top-K by rule cost (K = 16, config), ensuring at least 2 lattice and 2 learned candidates are included if available (diversity guard).
4. Forward-sim scoring; final cost; argmin.
5. Publish `TrajectoryCandidates` with the sim metrics added to `cost_breakdown` (names `sim_col`, `sim_prog`, …) for the Foxglove table; publish the rollout of the selected candidate as markers (`nuway_viz`: "sim rollout" layer).

## 4. Evaluation

- M1 protocol + M6 scenario routes with M8 checkpoints, four configs: `{rule, forward_sim} × {sample, reactive}`; plus `mix`. Report table with driving score, collision counts by type, completion, comfort, selector cycle time.
- Sanity on synthetic scenes (`tests/test_forward_sim_scenarios.py`): cut-in vehicle → the "brake" candidate wins; slow lead + clear left lane → the lane-change candidate wins; pedestrian stepping in → stop candidate wins.

## 5. Task list

1. [ ] `forward_sim.py` (torch reference) + scenario tests.
2. [ ] `forward_sim_scorer.cc` + parity test vs torch; timing test.
3. [ ] Agent Mode B (IDM along lane) in C++; tests (agent brakes for ego).
4. [ ] `planner_node` two-stage selection, diversity guard, breakdown publishing; config `configs/planning/forward_sim.yaml`; profile `m9_forward_sim.yaml` (includes `m8_learned_planner.yaml`, sets `planning.selector: forward_sim`).
5. [ ] Foxglove "sim rollout" layer.
6. [ ] Ablation runs and report `data/eval_runs/m9_report.md`.

## 6. Decisions log

- (2026-09-02) Forward-sim (PDM-Closed style) chosen over a learned scorer first: cheap, interpretable, strong. A learned ranking head (InfoNCE against expert) remains a documented option if the ablation shows headroom; RL is out of scope.
- (2026-09-05) M9 depends on M8: the selector is designed and measured over lattice + learned candidates (diversity guard, M8 checkpoints). It is not reordered before M8.

## 7. Open questions

- Horizon 5 s vs 8 s for the rollout: 5 s is cheaper and less prediction-error-dominated; 8 s catches slow conflicts. Start at 5 s; test 8 s in the ablation.
