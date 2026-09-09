# M1 — Classical planning stack + MPC + evaluation harness

**Goal:** drive in traffic with a fully classical, learned-component-free stack: behavior FSM → Frenet lattice sampler → piecewise-jerk QP refinement → rule-based selector → safety layer → LTV-MPC. Prediction is constant-velocity. This is the permanent fallback path and the baseline every later milestone is measured against. M1 also delivers the **Leaderboard 2.x integration** (§3.12): the same stack runs unchanged under the official Leaderboard runner, which is what makes the overview's "Leaderboard-compatible" goal true rather than aspirational.

**Completion criteria**
- [ ] Driving score > 40 (Leaderboard 2.0 formula) averaged over the **M1 protocol**: 10 routes (5 per town from `dev_town03.xml` and `dev_town05.xml`, marked `protocol="m1"`) × 3 weather presets = 30 runs, with Traffic Manager traffic (50 vehicles, 30 walkers), GT perception + GT localization. Every later milestone's "M1 protocol" means these 30 runs; the full 20-route set is `run_routes.py --routes full`. (This M1 number is provisional; the baseline used by M3+ is re-measured at the end of M2 with the visibility filter on, see `00_overview.md` §3.)
- [ ] Two runs with the same seed produce bit-identical `results.csv` rows and identical per-tick `/nuway/control/command` sequences (determinism through the lockstep protocol, `02_interfaces.md` §2), with no `TickTimeout` in either run.
- [ ] No collisions on any route where all agents are visible and moving ≤ 15 m/s (i.e. failures must be attributable to prediction limits, not planner bugs).
- [ ] MPC solve time p99 < 3 ms; planner cycle p99 < 15 ms.
- [ ] `tools/eval/run_routes.py --profile m1_classical` produces `data/eval_runs/<run_id>/report.md` + `results.csv` + per-route MCAP.
- [ ] Every infraction row in `report.md` links a rendered incident sheet under `<route>/incidents/`, produced by the run itself with `eval.render: incidents` (`02_interfaces.md` §8.2). Rendering the same bag twice produces byte-identical PNGs.
- [ ] `tools/eval/run_leaderboard.sh --routes dev_town03.xml` runs the stack under the official Leaderboard 2.x runner (`leaderboard_evaluator.py`, ROS agent), one stack and one evaluator invocation per route (§3.12): the agent registers, sensor payloads and `/clock` flow, the lockstep gate holds (the runner blocks each tick until our `ControlCommand`), controls are applied, and the runner's JSON results land next to ours. Route completions and score parity are **not** M1 criteria: the runner provides no GT of any kind, and before M3/M5 the stack has no perception or localization source under it, so those criteria are deferred to M5 (`M5_localization.md`; §3.12).

---

## 1. Scope

In: LTV-MPC, delay compensation, behavior FSM, lattice sampler, QP refinement (path + speed), rule-based selector, safety layer, const-vel prediction, Traffic Manager integration, infraction detection, driving score, run/compare tooling, Foxglove BEV layout and its headless twin, Leaderboard 2.x agent wrapper and runner script.

Out: learned anything; forward-sim selector (M9); iLQR (M10).

## 2. Data flow

```
ref line ─┐
agents ───┼─► behavior_fsm_node ─► BehaviorDecision
pred ─────┘            │
                       ▼
            planner_node (C++, 10 Hz)
              ├─ LatticeSampler(decision, ref line)      → N candidates (path×speed)
              ├─ CollisionChecker(candidates, pred)      → feasibility mask
              ├─ PiecewiseJerkQP(candidate top-K)        → refined
              ├─ RuleSelector(costs)                     → selected
              └─ publish candidates + trajectory
                       ▼
            safety_layer_node (C++, 20 Hz)  → safe_trajectory
                       ▼
            mpc_node (C++, 20 Hz)           → ControlCommand
```

## 3. Components

### 3.1 `nuway_prediction/const_vel_node` (C++)

For each agent in `/nuway/perception/agents`: S = 1 sample, T = 16 (8 s @ 0.5 s), position = `p + v·t`, yaw constant, `sample_weight = [1.0]`. Pedestrians: same but velocity clamped to 2 m/s. Static obstacles: constant. Publishes `PredictionSamples` in `map` frame (transform from `base_link` via TF at the agents' stamp). Library `const_vel.cc` (bound by `nuway_py` for the M6 expert) + thin node.

Option flag `lane_follow: true`: for vehicles, project velocity onto the lane direction and follow the lane centerline (via `LaneGraph::NearestLane` + `Successors`) instead of a straight line. Default on; it materially reduces false collision predictions at curves.

**The node runs in every profile.** It always publishes `/nuway/prediction/fallback_samples`; when it is the primary producer (`prediction.source: const_vel`) it publishes `/nuway/prediction/samples` as well, with identical content. From M7 on, the consumers below switch to `fallback_samples` once a `TickTimeout` has marked `samples` degraded (`02_interfaces.md` §2 *Degradation*), which is what lets the lattice path survive a dead prediction process (M8 §2) without any consumer owning a timer.

### 3.2 `nuway_planning/behavior_fsm` (C++)

Inputs: ego state, reference line, agents (map frame), predictions (`samples`, or `fallback_samples` once `samples` is degraded, `02_interfaces.md` §2), traffic lights, lane graph. Runs on even ticks after all inputs stamped with that tick have arrived (`02_interfaces.md` §2 barrier). With an invalid pose or no reference line it emits the no-input decision (`LONGITUDINAL_STOP`, `reason: "no_input"`).

States and transitions (evaluated every cycle, hysteresis via timers):
- **Lateral**: `KEEP` | `CHANGE_LEFT` | `CHANGE_RIGHT`. Change is requested when (a) the route requires it (route lane id ≠ current lane id and neighbor is on route) or (b) `overtake_enabled` and a lead vehicle is slower than `speed_limit − 3` for > 4 s and the target lane gap is clear (no agent within `[−15, +25]` m longitudinally in the target lane, using predictions at t = 0..3 s). A change commits for ≥ 3 s and aborts if the gap closes (target-lane agent predicted within 8 m).
- **Longitudinal**: `FREE` | `FOLLOW` | `YIELD` | `STOP`.
  - `FOLLOW`: lead agent exists in current (or target) lane within 60 m ahead. Lead agent = nearest agent whose Frenet `d` is within lane half-width + 0.5 m and `s > s_ego`.
  - `STOP`: red traffic light on the route with stop line ahead within stopping distance `v²/(2·2.5) + 5`; yellow light per the dilemma-zone rule below; or stop sign not yet honored (honored = ego speed < 0.2 m/s within 3 m of stop line for 1 s); or goal reached.
  - `YIELD`: at an unsignalized junction, another agent predicted to occupy the conflict region before ego (conflict region = intersection of ego route polygon and agent predicted polygon within next 4 s). Also for pedestrians on/near crosswalks in ego path.
  - `FREE` otherwise.
- `target_speed` = min(speed limit, curvature speed, FOLLOW-derived IDM desired speed).
- `stop_s` for STOP: stop line `s` minus 1.0 m.
- **Yellow (dilemma zone)**: evaluated once on the green→yellow edge and then latched until the light changes or the stop line is passed, so the decision never flips mid-approach. With `d_stop` = distance to stop line, `t_yellow = yellow_duration − time_in_state` read from `TrafficLight.msg` (GT fills both from the CARLA API; M4 fills `yellow_duration` with a conservative default and `time_in_state` with the time since first confirmation, so `t_yellow` is a conservative estimate), `a_comf = 2.5`, `t_react = 0.3`:
  - `d_brake = v·t_react + v²/(2·a_comf)`. If `d_brake > d_stop` → cannot stop comfortably → **proceed** (treat as green).
  - else if `d_stop / max(v, 0.1) > t_yellow` → will not clear the line before red → **STOP**.
  - else → **proceed**.
  - Unknown TL state (`state == 0`, or `confidence < 0.5` once M4 replaces GT): treat as red if `d_stop > d_brake`, otherwise proceed. Log the reason.

Emit `BehaviorDecision` with `reason` string for logs.

IDM parameters (config): `s0=2.0, T=1.5, a=1.5, b=2.5`.

### 3.3 `nuway_planning/lattice_sampler` (C++)

Frenet frame from the reference line (`nuway_common/frenet.h`). Ego Frenet state `(s, ṡ, s̈, d, d', d'')` from `EgoState` (use `frenet.h::ToFrenet` with velocity/accel projection).

**Path candidates** (lateral, over `s`): quintic polynomials `d(s)` from current `(d, d', d'')` to `(d_f, 0, 0)` at `s_f = s + Δs`.
- `d_f` set: for KEEP: `{−1.0, −0.5, 0, 0.5, 1.0}` relative to lane center of the *target lane* (for CHANGE_*: target lane center is the neighbor lane; the `d` offset of the neighbor centerline is read from the reference line's lane geometry).
- `Δs` set: `{20, 35, 50}` m (scaled by speed: `max(Δs, 3·v)`).

**Speed candidates** (longitudinal, over `t`): quartic (velocity-keeping) or quintic (stopping) polynomials `s(t)`.
- FREE/FOLLOW: target speeds `{v_t − 3, v_t − 1.5, v_t, v_t + 1.5}` clipped to `[0, limit]`, horizon `{4, 6, 8}` s. FOLLOW additionally adds a gap-keeping candidate using the lead's constant-velocity position minus `s0 + T·v`.
- STOP: quintic to `(stop_s, 0, 0)` with horizon `{3, 5, 7}` s, plus a "hard stop" at max decel.
- YIELD: quintic to `(s_conflict − 3, 0, 0)` plus a "go" candidate.

**Injected stop candidates (always, in every behavior state).** Two candidates are built from the ego state and the reference line alone — no `BehaviorDecision` is consulted — and appended to the set on every planning tick:
- **gentle**: quintic to `(s + v²/(2·a_gentle), 0, 0)` with `a_gentle = 1.5`, holding the current lane's centerline (`d_f` = current lane center).
- **hard**: quintic to rest at `a_min` (max decel) on the same path.

Both carry `source: "stop"`. They are the planner's floor: they are **exempt from the feasibility filter of this section** (they are never rejected, including for collision) and **bypass the QP of §3.5** (a QP infeasibility can never remove them), so the selector's input is never empty (§3.6). They are otherwise ordinary candidates — scored by the full cost table, so a colliding gentle stop loses to the hard one, and any feasible normal candidate beats both on `progress` and `speed_dev` by ~25 cost units, which is why injecting them does not perturb normal driving. Normal candidates keep the hard feasibility filter unchanged: a colliding normal candidate is still rejected outright.

Combine path × speed → Cartesian trajectories via `frenet.h::ToCartesian`, resampled at 0.1 s, 8 s horizon (81 points), with `yaw`, `v`, `a`, `kappa`. A speed profile whose horizon is shorter than 8 s (the STOP and YIELD quintics, and the 4 s / 6 s keeping profiles) is **extended at its terminal state to 8 s**: at rest for stops, at constant speed along the path otherwise, so every candidate has 81 points regardless of the polynomial's own horizon. Total candidates ≈ 5·3·4·3 = 180 max; prune before QP.

**Feasibility filter**: `|kappa| ≤ kappa_max`, `a ∈ [a_min, a_max]`, `|a_lat| ≤ 4`, path stays within `[−right_bound + w/2, left_bound − w/2]`, no collision with predictions (see 3.4).

### 3.4 `nuway_planning/collision_checker` (C++)

Ego footprint: rectangle `length × width` (`configs/vehicle/<vehicle>.yaml`, recorded by sysid, M0 §2.6) inflated by `margin_lon = 1.0`, `margin_lat = 0.4`. For each candidate and each prediction sample, check oriented-box overlap at t ∈ {0, 0.5, …, 8} using the separating axis theorem (t = 0 uses the agent's current observed pose; t = 0.5 … 8 come from the sample, which carries T = 16 steps starting at 0.5 s); agent boxes inflated by `0.2`. Cost = `Σ_samples weight · 𝟙[collision]` and `min_ttc` = first colliding t. Also compute `min_distance(t)` (approximate by circle-disc decomposition: 3 discs per vehicle) for the selector's proximity cost.

### 3.5 `nuway_planning/piecewise_jerk_qp` (C++, OSQP)

Apollo-style path–speed decomposition, applied to the top-K (K = 8) candidates by pre-QP rule cost.

**Path QP** (variables `d_i, d'_i, d''_i` at `Δs = 1.0` m over the candidate's `s` range):
```
min  w_d Σ(d_i − d_ref_i)² + w_dd Σ d''_i² + w_ddd Σ ((d''_{i+1} − d''_i)/Δs)² + w_end (d_N − d_ref_N)²
s.t. d_{i+1} = d_i + d'_i Δs + ½ d''_i Δs² + ⅙ d'''_i Δs³   (piecewise-constant jerk)
     d'_{i+1} = d'_i + d''_i Δs + ½ d'''_i Δs²
     l_min_i ≤ d_i ≤ l_max_i           (drivable bounds minus half width, further tightened by static obstacle footprints projected into Frenet)
     |d''_i| ≤ kappa_max − |κ_ref|     (curvature budget)
     d_0, d'_0, d''_0 fixed to ego state
```
`d_ref` = the lattice path. Static/slow agents (|v| < 0.5) become lateral bound constraints over their `s` interval (pass-left or pass-right decided by the lattice candidate's side).

**Speed QP** (variables `s_i, ṡ_i, s̈_i` at `Δt = 0.1` s over 8 s):
```
min  w_v Σ(ṡ_i − v_ref)² + w_a Σ s̈_i² + w_j Σ jerk_i² + w_s Σ(s_i − s_ref_i)²
s.t. piecewise-jerk kinematics
     s_lower_i ≤ s_i ≤ s_upper_i       (from S–T obstacle boxes of dynamic agents along the refined path; yield ⇒ upper bound, follow ⇒ upper bound at lead − gap, overtake ⇒ lower bound)
     0 ≤ ṡ_i ≤ v_limit_i (path curvature + speed limit)
     a_min ≤ s̈_i ≤ a_max
     |jerk_i| ≤ jerk_max
```
S–T boxes: for each prediction sample, project the agent box onto the refined path; occupied `s` interval at each `t` (with margins). The candidate's original speed profile determines which side of each box we are on; keep that homotopy.

Warm start OSQP from the previous cycle's solution shifted by `Δt`. Fallback if infeasible: relax bounds with slack variables (weight 1e4), and mark `qp_relaxed=true` in the cost breakdown.

Output: refined `Trajectory` with `source="lattice"`.

### 3.6 `nuway_planning/rule_selector` (C++)

Cost per candidate (all terms normalized to roughly [0, 1] before weighting; weights in `configs/planning/selector.yaml`):
| term | definition | default w |
|------|-----------|-----------|
| collision | from collision checker (weighted fraction of samples) | 1000 |
| ttc | `max(0, 1 − min_ttc/4)` | 50 |
| proximity | `Σ_t max(0, 1 − min_dist(t)/3)` | 10 |
| progress | `−(s_end − s_0) / (v_limit · 8)` | 20 |
| speed_dev | mean `|v − target_speed| / v_limit` | 10 |
| lateral_dev | mean `|d − d_lane_center| / lane_half_width` | 5 |
| comfort | mean `a_lat² + jerk²` normalized | 5 |
| rule | red-light/stop-line crossing indicator, off-route indicator | 500 |
| consistency | distance to previous selected trajectory over first 2 s | 3 |
| qp_relaxed | 1 if slack used | 30 |

Select argmin. Publish `TrajectoryCandidates` with full breakdown (for the Foxglove table) and `Trajectory`.

**The planner publishes on every planning tick, and the selector's input is never empty.** Three cases, in order:
1. Valid pose, a reference line, and a usable `BehaviorDecision`: normal candidates (hard feasibility filter, §3.3) **plus** the two injected stop candidates. Argmin over all of them.
2. Valid pose and a reference line, but no usable decision — a no-input `BehaviorDecision` (`LONGITUDINAL_STOP` / `reason: "no_input"`, which is also what a degraded perception source produces) or `/nuway/planning/behavior` itself degraded (`02_interfaces.md` §2): no normal candidate can be sampled without a decision, so the set is the two injected stop candidates alone and the argmin is the gentle one unless it collides. An upstream failure with no collision imminent therefore ends in the gentlest in-lane stop the situation allows, not a hard brake and not a stop at the current pose.
3. No reference line or an invalid pose: the no-input pair — an empty `TrajectoryCandidates` and a `source: "none"` stop trajectory at the current pose (`02_interfaces.md` §2). This is the only case that publishes no candidate, because without a pose or a line there is no lane to stop in.

In every case the safety layer's barrier is satisfied, so an empty candidate set can never stall the tick and can never reach the `TickTimeout` path (§3.7, `02_interfaces.md` §2), which is reserved for a genuinely dead upstream process. All candidates from every source are 81 points over 8 s (`02_interfaces.md` §4), so the horizon-normalised terms above compare like with like; a candidate with fewer points is rejected by the feasibility filter, not scored.

### 3.7 `nuway_planning/safety_layer_node` (C++, 20 Hz)

Independent last check on `/nuway/planning/trajectory`, using agents and predictions (`samples`, or `fallback_samples` once degraded) directly (not the planner's internal state). Runs every tick, after the pose of that tick and — on planning ticks — after the planner's output stamped with that tick (`02_interfaces.md` §2 barrier); on control-only ticks it re-checks the last planning tick's trajectory against the current pose:
1. Under the barrier the input trajectory is never stale, so there is no staleness timer. If the planner is *degraded* (a `TickTimeout` while its output for that tick was missing) → publish the **gentlest** stop profile along the last safe trajectory's path that this node's own collision check clears — `a_gentle = 1.5` first, max decel only if the gentle one collides — and hold it for the rest of the episode. A dead planner with nothing bearing down on the car is not a reason to brake hard. If no safe trajectory was ever published in this episode (the planner died before its first output), the car is not yet moving and the fallback is the no-input stop at the current pose. A `source: "none"` input passes through as a stop at the current pose.
2. Re-run collision check with a larger margin over the first 3 s. If collision → replace with a max-decel stop profile along the same path.
3. Enforce limits: clip `a`, `kappa`, and re-integrate if clipped.
4. Occupancy footprint check against `occupied` channel (threshold 0.6) over the first 3 s → stop profile.
Publish `/nuway/planning/safe_trajectory` and a `NodeDiag` warning whenever it intervenes (intervention rate is a tracked metric).

### 3.8 `nuway_control/mpc_node` (C++, OSQP via osqp-eigen, 20 Hz)

**M1 retires the M0 controller.** From M1 on, `mpc_node` is the only producer of `/nuway/control/command` in every profile: `control.controller: pure_pursuit` stays valid for the M0 profiles alone, and no M1+ profile sets it. There is no controller-level redundancy — a failure of any kind stops the car (below) rather than handing it to a cruder controller, because the guaranteed-safe input is the safety layer's job (§3.7), not the control layer's.

**Model** (kinematic bicycle with steering lag), state `x = [X, Y, ψ, v, δ]`, input `u = [a, δ_cmd]`:
```
Ẋ = v cos ψ,  Ẏ = v sin ψ,  ψ̇ = v tan δ / L,  v̇ = a,  δ̇ = (δ_cmd − δ)/τ_steer
```
Discretize with RK2 at `dt = 0.05`, horizon `N = 20` (1 s). Linearize about the reference trajectory (`safe_trajectory` resampled at `dt`, using its `a` and `kappa`→`δ_ref = atan(L κ)`), giving `A_k, B_k, c_k`.

**Delay compensation**: before solving, propagate the measured state forward by `t_delay` (config, default 0.10 s = 2 ticks: perception-to-actuation) using the last commanded inputs; use the propagated state as `x_0`, and shift the reference accordingly. The measured `δ` comes from `VehicleState.steering_angle` when `valid_steering` is true (our harness: the actual wheel angle, M0 §2.2); otherwise (Leaderboard, §3.12) from the steering-lag model driven by our own commands.

**Timing**: runs once per tick, after `/nuway/loc/pose` *and* `/nuway/planning/safe_trajectory` stamped with that tick have both arrived (`02_interfaces.md` §2 barrier), and stamps the `ControlCommand` with that tick; the world manager's lockstep gate waits for it. A solve that exceeds the tick has no effect on results, only on wall-clock speed.

**QP**:
```
min Σ_k (x_k − x_ref_k)ᵀ Q (x_k − x_ref_k) + u_kᵀ R u_k + Δu_kᵀ R_d Δu_k + terminal
Q = diag(1.0, 1.0, 2.0, 0.5, 0.0) (position error in a rotated frame: use lateral/heading error weights, see below)
R = diag(0.1, 1.0),  R_d = diag(1.0, 10.0)
s.t. a ∈ [a_min, a_max], δ_cmd ∈ [−δ_max, δ_max], |Δa| ≤ jerk_max·dt, |Δδ_cmd| ≤ steer_rate_max·dt
```
Implementation detail: position error expressed in the reference point's local frame (longitudinal/lateral) so `Q` can weight lateral more than longitudinal (`Q_lon=0.5, Q_lat=4.0`). Condense to a dense QP over `u` (N·2 = 40 variables) — small enough that dense is fastest.

Output the first input `u_0` as `ControlCommand`. Warm start from the previous solution shifted.

**Solver outcomes.** The QP constrains inputs only — box bounds on `a` and `δ_cmd`, rate bounds against the last command — and never the state, so holding the previous input is always feasible and the problem cannot be primal infeasible; with `R` positive definite it cannot be unbounded either. "Solver failure" therefore means one thing in practice: the iteration budget ran out. Three outcomes:
- **Solved**, *including a failed polish step*: `polish: true` (§5) solves a reduced KKT system after convergence, and that step failing leaves the unpolished solution valid and optimal to tolerance. **A polish failure is not a solver failure** and must not be counted or reported as one — treating it as one would brake the car for a non-event.
- **Iteration budget exhausted** (`max_iter` reached / solution inaccurate): use the returned iterate. It is near-optimal, but an ADMM iterate satisfies the constraints only in the limit, so **clamp `u_0` to the input bounds and to the rate limits against the last command** before publishing it. Raise `NodeDiag` warn and increment the consecutive-failure counter.
- **No usable iterate** (factorization failure, non-finite values in the solution): straight to `emergency_stop: true` for that tick, without waiting for the counter.

After `mpc.max_consecutive_solver_failures` (default 5, i.e. 0.25 s) *consecutive* counted failures, publish `emergency_stop: true` instead of the iterate, and keep doing so while the failures continue. Any solved tick resets the counter. **This is deliberately not the degradation mechanism of `02_interfaces.md` §2**: it is node-local, counted in ticks rather than wall clock (so it is deterministic), and it recovers within the episode, because a structurally always-feasible QP has no failure that a later tick cannot undo. The counter clears on `ResetEvent` like every other piece of node state (§5). Each counted failure is one `n_mpc_failures` in `results.csv` (§3.10) and an incident sheet (§3.11).

**Steering while stopping.** On any tick this node raises `emergency_stop`, it sets `steering_angle` to the `δ_ref` of the reference point it was tracking (the last valid one when there is no usable reference), never 0: `control_adapter` converts `steering_angle` independently of `emergency_stop` (M0 §2.3 overrides throttle and brake only), so the wheel keeps the lane geometry while the brake is at the floor. Returning the wheel to centre at speed in a curve is a hazard of its own — the car brakes at the tyre limit while leaving the lane.

With a `valid: false` pose or a degraded `safe_trajectory`, publish `emergency_stop: true` stamped with the tick (no-input convention). A `source: "none"` safe trajectory is simply tracked, which holds the car still.

Debug: publish predicted MPC trajectory as markers.

**Failure response ladder (index; the rules themselves are in the sections cited).** `emergency_stop` is the hardest stop in the stack — brake 1.0 at the adapter, below the planning limit `a_min`, no trajectory followed — so it is reserved for the cases where *"stop in the lane" cannot be expressed at all*: no pose (no lane frame), no trajectory (nothing to track), or no computable input. Everywhere else the stack stops by **planning** a stop and tracking it through the MPC and the pedal table.

| failure | response | where |
|---|---|---|
| `/nuway/loc/pose` invalid or degraded | `emergency_stop` | §3.8, `02_interfaces.md` §2 |
| `safe_trajectory` degraded (safety layer dead) | `emergency_stop` | §3.8, `02_interfaces.md` §2 |
| MPC QP returns no usable iterate | `emergency_stop`, immediately | §3.8 |
| MPC QP fails `max_consecutive_solver_failures` ticks in a row | `emergency_stop`, after the clamped iterate was used for those ticks | §3.8 |
| `behavior` degraded (FSM dead) | gentlest in-lane stop the collision check clears | §3.6 case 2, `02_interfaces.md` §2 |
| `trajectory` degraded (planner dead) | gentlest in-lane stop along the last safe path, max-decel if that collides | §3.7 |
| perception degraded (agents / occupancy / traffic lights) | no-input decision → gentlest in-lane stop | §3.6 case 2 |
| `samples` degraded (prediction dead) | `fallback_samples`; driving continues, no stop | §3.1, `02_interfaces.md` §2 |
| every normal candidate rejected by the feasibility filter | injected stop candidates carry the tick: gentle, or hard if the gentle one collides | §3.3, §3.6 case 1 |
| safety layer intervenes (collision, occupancy, limits) | max-decel stop *profile*, tracked normally; no `emergency_stop` | §3.7 |
| no reference line, or pose invalid, at the planner | `source: "none"` stop at the current pose, tracked | §3.6 case 3 |

Under the Leaderboard profile the agent's watchdog answers a tick that got no command with a full-brake `VehicleControl` (§3.12); in M1 that is *every* tick, since no localization source runs under the runner before M5.

### 3.9 Traffic in the world manager

`world_manager` reads the profile's `carla.traffic:` block (`02_interfaces.md` §5): `n_vehicles`, `n_walkers`, `seed`, `tm_port`, `hybrid_physics: false`. Spawn via Traffic Manager with `tm.set_random_device_seed(seed)`, `set_global_distance_to_leading_vehicle(2.5)`, `ignore_lights_percentage(0)`. The seed is the profile's `carla.traffic.seed` unless the `/nuway/sim/reset` call that started the route carried `traffic_seed >= 0` (`Reset.srv`, `02_interfaces.md` §4), which is how the harness sets it per route. Walkers via `WalkerAIController`. All in sync mode. Traffic is despawned and respawned on every `/nuway/sim/reset` so that route N+1 never sees route N's actors.

### 3.10 Evaluation harness (`nuway_eval`, `tools/eval/`)

**Route runner** (`tools/eval/nuway_eval/route_runner.py`): groups the protocol by town and **launches one stack per town** (`02_interfaces.md` §2: a stack never reloads a town, so sim time stays monotonic). For each (route, weather, seed) in that town: `/nuway/sim/reset` at the start pose (which publishes `ResetEvent` so every stateful node clears itself, `02_interfaces.md` §7), traffic is respawned by the reset, set weather, publish the whole route on `/nuway/route/waypoints`, wait until goal/timeout/blocked, collect metrics, record MCAP. A route whose lockstep gate timed out at least once (any `/nuway/sim/tick_timeout`) is flagged `non_deterministic` in `results.csv` and excluded from the determinism criterion; the first timed-out tick and every source degradation that followed are listed in the report and rendered as incidents.

**Resume and crash recovery.** A protocol run is hours long, so `run_routes.py --resume <run_id>` skips every (route, weather, seed) that already has a complete row in `results.csv`. If the CARLA client times out mid-route (the server crashed), the runner marks the row `crashed`, restarts the server with `tools/carla/start_carla.sh`, relaunches the stack, and reruns that row once before moving on; a second crash leaves the row `crashed` and continues.

**`results.csv` schema** (one row per run; columns are stable across milestones and `compare_runs.py` joins on the first four):

| column | meaning |
|---|---|
| `route_id`, `town`, `weather`, `seed` | the protocol key |
| `profile`, `git_sha`, `run_id` | provenance |
| `route_length_m`, `completion`, `driving_score` | Leaderboard 2.0 quantities |
| `n_collision_pedestrian`, `n_collision_vehicle`, `n_collision_layout`, `n_red_light`, `n_stop_sign`, `outside_lanes_frac`, `n_min_speed`, `route_deviation`, `blocked`, `timeout` | infraction counts and flags |
| `n_safety_interventions`, `n_mpc_failures`, `planner_p50_ms`, `planner_p99_ms`, `mpc_p50_ms`, `mpc_p99_ms` | diag summaries |
| `n_tick_timeouts`, `first_timeout_tick`, `degraded_sources`, `non_deterministic`, `crashed` | lockstep health; `degraded_sources` is a `;`-joined list |
| `n_camera_drops` | learned profiles only (M3 §4.2); 0 otherwise |
| `sim_time_s`, `wall_time_s` | durations |
| `bag_path` | relative path of `run.mcap` |

The harness opens its **own CARLA client** (host/port from the profile), a second, *non-ticking* client alongside `world_manager`'s — M0 §2.2 forbids two ticking clients, not two clients. It uses it for GT reads (infractions below), for attaching the hero's `sensor.other.collision`, and for nothing that changes the world; every world change goes through the `/nuway/sim/*` services so that the Leaderboard path (§3.12), which has no client at all, sees the same interface.

**Infractions** (`infractions.py`), computed in the harness process from its client's GT reads each tick:
- collisions (layout / vehicle / pedestrian) via `sensor.other.collision` attached to hero, deduplicated with 2 s cooldown per other actor
- red light: ego crosses a stop line while the light is red (use CARLA's `traffic_light.get_stop_waypoints()` and hero transform)
- stop sign: ego enters a stop-sign trigger volume and never falls below 0.2 m/s inside it
- outside route lanes: fraction of route driven with hero center off the route lanes (via `LaneGraph::NearestLane` + route lane set)
- route deviation: > 30 m from the reference line
- agent blocked: speed < 0.1 m/s for 90 s
- route timeout: per-route time budget = route length / 5 m/s × 2 + 60 s
- min speed infractions: ported from the Leaderboard 2.0 `MinSpeedTest` criterion (ego slower than a fraction of the surrounding traffic's speed for a sustained window), so our harness and the official runner (§3.12) count the same things.

**Score** (`driving_score.py`): Leaderboard 2.0 formula: `route_completion × Π penalty_i^{n_i}` with penalties collision_pedestrian 0.50, collision_vehicle 0.60, collision_layout 0.65, red_light 0.70, stop_sign 0.80, outside_lanes scales by fraction, route_dev/blocked/timeout → completion truncated. Keep the coefficients in `configs/eval/scoring_lb20.yaml` so that a `lb21` variant can be added later.

**Report** (`report.py`): `results.csv` (one row per route run), `report.md` with per-town and overall driving score, infraction histogram, planner/controller diag summaries (p50/p99 cycle time, safety-layer intervention count, MPC failures), and links to MCAP files **and to the rendered incident sheets of §3.11** (relative paths, so the run directory can be copied or archived whole). `compare_runs.py A B` prints per-route deltas and a paired summary.

Route sets: `tools/eval/routes/dev_town03.xml`, `dev_town05.xml` (10 routes each, 1.5–3 km, mix of junctions, roundabout on Town03, highway on Town05). Five routes per file carry `protocol="m1"`; those 10 form the M1 protocol (30 runs with the three weathers), which every later milestone's closed-loop criterion refers to. `--routes full` runs all 20. Weather presets: `ClearNoon`, `WetSunset`, `HardRainNight` (the rain/night ones only matter after M3; M1 uses them to keep the protocol fixed).

### 3.11 Visualization

Two back-ends over one set of layers (`02_interfaces.md` §3.9 and §8). Live, for a human at the devbox; headless, for everyone and everything else.

**Live.** Foxglove layout `bev_planning.json`: the `occupancy` raster, agents (boxes by class), predictions (polylines faded by time), lattice candidates (thin, colored by cost quantile), refined selected (thick), `safe_trajectory` (if different), MPC predicted horizon, reference line & bounds, behavior state text, cost-breakdown table (from `TrajectoryCandidates`), diag table. Every one of these that is a drawing is a `/nuway/viz/<layer>` of `02_interfaces.md` §3.9 and has a `draw_<layer>()` twin.

`nuway_viz/marker_node.cc` converts each of the above topics to the `/nuway/viz/<layer>` `MarkerArray` topics. One node, many subscriptions.

**Headless.** `ml/nuway_ml/viz/` draws the same layers with matplotlib (`Agg`), and `tools/viz/render_bag.py` replays a route's MCAP into PNGs without CARLA, without a display and without `rclpy` — MCAP carries the schemas, `rosbags` decodes them. This is the only way an eval result can be inspected after the fact, on CI, or by a coding agent; the contract, the output layout and the frame composition are `02_interfaces.md` §8.

M1 owns this because M1 owns the eval harness: from here on, "the score dropped" is always answerable from `data/eval_runs/<run_id>/` alone.

**Incident frames.** `report.py` renders `eval.incident_window` ticks around every infraction, safety-layer intervention, MPC failure and lockstep timeout, and links the resulting sheet from the row in `report.md` that reports it (§8.2). Default `eval.render: incidents`; `full` renders the whole route at `render_stride`. Adding a Foxglove layer without the matching `draw_<layer>()` in `nuway_ml/viz/bev_draw.py` is a defect, not a follow-up.

**Chase camera.** `eval.chase_cam: true` adds the `cam_chase` rig entry, published on `/nuway/viz/chase_cam` and written to `<route>/chase/*.jpg` by `nuway_eval/chase_writer.py` (§8.3). Off by default, off under the Leaderboard profile. It answers "is the car doing something visibly insane" faster than any BEV.

### 3.12 Leaderboard 2.x integration (`nuway_carla_bridge/leaderboard_agent.py`, `tools/eval/run_leaderboard.sh`)

The official Leaderboard runner owns the CARLA client, the tick, the sensors and the scoring. Our stack must therefore run without `world_manager`, `gt_publisher` and the CARLA-native `--ros2` topics. The integration is a thin ROS agent, not a second stack:

- The Leaderboard and scenario_runner repositories are cloned into `external/` at the commit hashes pinned in `tools/eval/setup_leaderboard.sh` (`03_style_and_conventions.md` §6.4; the pair must target CARLA 0.9.16 and the pinned Leaderboard must expose the ROS 2 agent track, which task 19 verifies before pinning). Their Python dependencies are the `leaderboard` uv group. Nothing in the stack imports from `external/` except `leaderboard_agent.py`.
- **One stack per route.** The evaluator restarts game time at zero for every route and a route file may span towns, both of which break the stack's "one town, monotonic sim time" contract (`02_interfaces.md` §2). `run_leaderboard.sh` therefore splits the route file into single-route files and, for each, launches a fresh stack with `leaderboard.yaml`, runs `leaderboard_evaluator.py` on that one route, collects its JSON, and tears the stack down. The agent never has to offset clocks or reload maps; the price is one stack start-up per route.
- `leaderboard_agent.py` is a Leaderboard `AutonomousAgent` (`ROS2` track). `sensors()` returns the rig from `configs/sensors/rig_leaderboard.json` (same extrinsics as `rig_dev.json`, checked by a test; stays within the Leaderboard's sensor-count limits, so `cam_tl` from M4 is excluded there). `run_step(input_data, timestamp)` publishes the sensor payloads on the same `/carla/hero/*` topic names, types and `frame_id`s as the native interface (with *reliable* QoS, so there are no camera drops on this path), publishes `/clock` for that tick, publishes `/nuway/sim/vehicle_state` from the speedometer pseudo-sensor (`valid_steering: false`), then **blocks** until `/nuway/control/command` stamped with that tick arrives and returns it converted through the same Python `LongitudinalMap` that `control_adapter` uses (M0 §2.3). This preserves the lockstep protocol: the runner cannot tick until we have answered. If no command arrives within `lockstep_timeout_s`, the agent's **watchdog** publishes `/nuway/sim/tick_timeout` exactly as `world_manager` would, returns a full-brake, zero-throttle `VehicleControl`, logs a diag error and counts the tick; that count is reported next to the Leaderboard's results. Serializing four `Image`s and a `PointCloud2` through `rclpy` every tick costs real wall clock (expect 10–30 ms); task 19 measures it and the report states it. Under lockstep it is a wall-clock cost only.
- The map comes from the Leaderboard's OpenDRIVE pseudo-sensor on the first step, written to `data/maps/<town>/map.xodr` and served by `map_server`, which waits for the file (M0 §2.4), exactly as in our harness. The route is the Leaderboard's `global_plan`, converted to `/nuway/route/waypoints` by `nuway_ml/common/routes.py` (M0 §2.5).
- `/nuway/sim/reset` and `/nuway/sim/set_weather` are served by the agent (`setup()`/`destroy()` per route) and publish `ResetEvent` like `world_manager` does.
- No CARLA client is opened by the stack. GT toggles are meaningless under the Leaderboard: the agent refuses to start with **any** `use_gt.*: true` — the runner provides no privileged access of any kind, GT pose included. Before M5 the stack therefore has no localization source under the runner (and before M3, no perception source): no `/nuway/loc/pose` is ever published, the controller never fires, and **every tick is answered by the watchdog's brake command**. That is what "controls are applied" means in the M1 criterion: the runner ticks in lockstep against our watchdog, sensor payloads, `/clock` and results plumbing all flow, and the hero sits still until the runner's blocked-agent criterion ends the route. M1 validates the integration mechanically; the route-completion and score-parity criteria are deferred to M5, where `m5_no_gt.yaml` is the first profile that legitimately drives there.
- `run_leaderboard.sh` loops over the routes as above, each time launching the stack with `configs/profiles/leaderboard.yaml` and the Leaderboard evaluator with `--agent leaderboard_agent.py --track ROS2`, and merges the per-route JSON results next to our `results.csv`. Scores from the two are compared in the report.

## 4. Task list

1. [ ] `const_vel_node` (+ lane-follow option, unit test on a curved lane).
2. [ ] `frenet.h` extensions: velocity/accel projection, `ToCartesian` with `d(s)` polynomials; tests; mirror in `nuway_ml/common/frenet.py` + parity test.
3. [ ] `behavior_fsm` library + node + tests (scripted scenarios: lead vehicle, red light, yield at junction, route lane change).
4. [ ] `lattice_sampler` + feasibility filter + the two injected `source: "stop"` candidates (§3.3) + tests (candidate count, limits respected, both stop candidates present in every behavior state and never rejected by the filter).
5. [ ] `collision_checker` + tests (SAT correctness vs brute force on random boxes).
6. [ ] `piecewise_jerk_qp` path + speed + tests (feasibility on synthetic bounds; warm start speeds up second solve).
7. [ ] `rule_selector` + `planner_node` orchestrator; publish candidates/breakdown. Tests for the three cases of §3.6: a feasible normal candidate beats both stop candidates, every normal candidate colliding leaves the gentle stop selected (the hard one when the gentle collides too), and a missing/no-input decision yields the gentle stop rather than `source: "none"`.
8. [ ] `safety_layer_node` + tests (stale input, collision injection, planner degraded with a clear road → gentle profile, with an obstacle → max decel).
9. [ ] `mpc_node` + `bicycle_model.h` jacobians (tests: finite-difference check) + delay compensation + the solver-outcome policy of §3.8 (tests: a failed polish is not counted as a failure; an exhausted iteration budget publishes the clamped iterate; `max_consecutive_solver_failures` consecutive failures escalate to `emergency_stop` and a solved tick resets the counter; an `emergency_stop` tick carries `δ_ref`, not steer 0). Record the chosen `max_iter` and the observed iteration distribution in the task 17 tuning note — a tight `max_iter` turns a timing problem into a braking event. Flip the `control.controller` default in `nuway_bringup/profile.py` from `pure_pursuit` to `mpc` so that an M1+ profile which omits the key cannot silently launch the M0 controller (§3.8).
10. [ ] Traffic spawning in world_manager; seed determinism test (two runs → identical agent trajectories for 30 s); traffic respawn on reset.
11. [ ] `tools/eval/nuway_eval/`: `infractions.py` (incl. the ported min-speed criterion), `driving_score.py`, `route_runner.py` (one stack per town, second non-ticking client, waypoint publishing, reset event, non-deterministic flag, `--resume`, crash recovery), `report.py` (the `results.csv` schema of §3.10), `compare_runs.py`; route XMLs with the `protocol="m1"` marks; `configs/eval/scoring_lb20.yaml`.
12. [ ] Foxglove layout + marker node.
13. [ ] `nuway_ml/viz/` (style, `draw_<layer>()` per §3.9 layer, panel, contact sheet) + tests: a fixture scene renders to a byte-identical PNG twice, and every `/nuway/viz/<layer>` name has a `draw_` function.
14. [ ] `tools/viz/render_bag.py` (MCAP decode via `rosbags`, `--stride`/`--ticks`/`--layers`, frames + contact sheets); run it on a dev-route bag in an environment with ROS *not* sourced to prove the dependency claim.
15. [ ] Incident rendering in `report.py`: incident tick list → `incidents/` sheets → relative links in `report.md`. `eval.render` / `incident_window` config keys.
16. [ ] `cam_chase` rig entry, `/nuway/viz/chase_cam` publisher in `sensor_rig.py`, `chase_writer.py`; `eval.chase_cam` off in `leaderboard.yaml`.
17. [ ] Tune: lattice sets, selector weights, MPC weights on dev routes until criteria met. Record final weights in configs and a short tuning note in the Decisions log.
18. [ ] `tests/integration/test_m1_traffic.py` (short route, 20 vehicles, asserts no collision and completion), `test_m1_determinism.py` (same route twice, asserts identical command sequences) and `test_m1_degradation.py` (kill `const_vel_node` mid-route: exactly one `TickTimeout`, the FSM/planner/safety layer report `samples` degraded and stand the car still, the route is flagged).
19. [ ] `tools/eval/setup_leaderboard.sh` with the Leaderboard + scenario_runner commits pinned (verify the ROS 2 track exists in that Leaderboard commit and that the pair targets CARLA 0.9.16; record both hashes in `03_style_and_conventions.md` §6.4), the `leaderboard` uv group; `leaderboard_agent.py` (watchdog with `TickTimeout`, shared `LongitudinalMap`, reliable sensor republish, measured serialization cost), `rig_leaderboard.json` (+ parity test vs `rig_dev.json`), `configs/profiles/leaderboard.yaml`, `run_leaderboard.sh` (one stack per route); run a dev route under the official runner as the mechanical integration check (route completions and score parity are M5 criteria, §3.12).

## 5. Determinism checklist

- Single `world.tick()` owner; all nodes consume `/clock`; the owner ticks only after the current tick's `ControlCommand` arrived (lockstep, `02_interfaces.md` §2). No node uses a wall-clock timer; 10 Hz nodes act on even ticks only.
- **Current-tick barrier** (`02_interfaces.md` §2): every node waits for all of its inputs stamped with the current tick before running, so intra-tick DDS delivery order cannot change results. `test_m1_determinism.py` runs the same route with an artificial 20 ms delay injected into one node's callback (launch argument) and still expects identical command sequences; that is the test that catches a node consuming "latest" instead of "stamped `k`".
- Every stateful node clears on `ResetEvent`, including its degraded flags; the harness flags routes with lockstep timeouts. No node owns a wall-clock timer: the only wall-clock event is `world_manager`'s `TickTimeout`, and every fallback keys off it (`02_interfaces.md` §2).
- OSQP settings: `adaptive_rho: false`, fixed `max_iter`, `polish: true` for reproducibility; single-threaded BLAS in every node (`OMP_NUM_THREADS=1` set by the launch file, not read by nodes). Where a node uses its own thread pool (M9 forward-sim, M10 iLQR), work is partitioned per candidate and every reduction happens in candidate order on the calling thread, so the thread count never changes a result.
- CARLA's own reproducibility (PhysX, Traffic Manager, walkers) is a precondition, not something the stack can enforce; M0 §6 lists what is assumed and task 10 measures it with traffic.
- Random seeds: Traffic Manager, walker spawn, weather selection — all derived from the route seed.
- **Scope.** The bit-identical criteria (identical `results.csv` rows, identical command sequences) apply to classical/GT profiles. Learned profiles (M3 on) inherit deterministic *inputs* from lockstep, but CUDA kernels (scatter atomics, cudnn autotuning, `torch.compile`) are not bit-deterministic; milestone reports for learned profiles run the protocol at least twice and report the score spread instead of asserting bit-identity (`00_overview.md` §2.5).

## 6. Decisions log

- (2026-09-02) QP over iLQR for the classical path: convexity, feasibility detection, deterministic runtime. iLQR arrives in M10 for the learned path only.
- (2026-09-02) Path–speed decomposition rather than joint spatiotemporal optimization: simpler, matches Apollo, adequate for CARLA urban speeds.
- (2026-09-05) Yellow-light handling is an explicit dilemma-zone rule with a latched decision, not "always stop" (rear-end risk, hard braking) nor "always go" (red-light infractions). The rule only consumes `TrafficLightArray`, so M4 swaps the source without touching the planner.
- (2026-09-05) Determinism by lockstep rather than by wall-clock pacing: pacing made results depend on machine load, which made the "identical scores" criterion unmeetable. The cost is that a slow node slows the whole simulation, which is acceptable for a toy system.
- (2026-09-05) Visualization is specified as a rendering contract with two back-ends (`02_interfaces.md` §8), not as "a Foxglove layout". A live GUI is unreadable to CI, to a post-hoc session on a bag, and to the LLM agent doing most of the work in this repo, so a Foxglove-only layer leaves the project with no way to debug a bad route except re-running it in front of a human. Renders on disk cost some MB per route in a gitignored directory; that is the whole price.
- (2026-09-05) `eval.render` defaults to `incidents`, not `full`: a route is 6k–12k ticks, and the frames worth looking at are the ones the scorer already flagged.
- (2026-09-05) Leaderboard integration lives in M1, not in a later milestone, so that every subsequent milestone is measured under both our harness and the official runner and no design decision can silently break Leaderboard compatibility.
- (2026-09-06) The M1 Leaderboard criterion is mechanical integration only. The official runner provides no GT of any kind (perception included), and before M3/M5 the stack has nothing to fill those roles under it, so route completions and score parity against our harness are deferred to M5 — the first milestone whose `m5_no_gt.yaml` legitimately drives there.
- (2026-09-06) **Design review.** (a) The M1 protocol is 10 routes × 3 weathers = 30 runs, five marked routes per town, so every later closed-loop criterion costs a few hours rather than a day; the 20-route set stays available. (b) Fallbacks are keyed off `TickTimeout`, never off a consumer's own wall-clock timer, which retired the safety layer's 0.3 s / 0.6 s staleness rule (unreachable under the barrier anyway). (c) The harness package moved to `tools/eval/nuway_eval/` so `tools/` never imports from `ros2_ws` except `nuway_py`. (d) Under the Leaderboard the harness runs one stack per route; the alternative (clock offsets and map reloads inside the agent) would have made the agent a second world manager. (e) `results.csv` got a schema and the runner a `--resume`, because a protocol run is hours long and the CARLA server does crash.
- (2026-09-08) **No controller-level redundancy from M1 on.** `mpc_node`'s QP solver failure triggers `emergency_stop: true` directly instead of falling back to pure pursuit for that cycle, and `pure_pursuit_pid_node` is retired with M0: it stays launchable only by the M0 profiles, no M1+ profile sets `control.controller: pure_pursuit`, and the launch default flips to `mpc` (task 9) so the M0 controller cannot be reached by omission. Rationale: every other failure the node handles (invalid pose, degraded `safe_trajectory`) already stops rather than degrading to a cruder controller, and a second control path is a second thing to keep correct and tuned for a failure mode the safety layer already covers by construction (§3.7 guarantees a trackable `safe_trajectory`, so the control layer never needs its own opinion about safety).
- (2026-09-09) **The planner always has a floor, and an upstream failure stops gently.** Two in-lane stop candidates (`source: "stop"`, one at `a_gentle = 1.5` and one at max decel) are injected in every behavior state, exempt from the feasibility filter and from the QP, so the selector's input is never empty (§3.3, §3.6). Reason: collision was a *hard* filter (§3.3) while the FSM only samples a max-decel profile in the STOP state, so a cut-in during FREE could reject all ~180 candidates; the planner would then publish nothing, stall the barrier, and be indistinguishable from a dead process — `TickTimeout`, planner degraded, stopped for the rest of the episode, route flagged `non_deterministic`. That is the right answer for a dead process and the wrong one for a planner correctly reporting that no collision-free option exists. The filter is unchanged for normal candidates: a colliding normal candidate is still rejected, and the injected pair is what keeps the set non-empty. The same principle applies to the fallbacks themselves — a degraded `behavior` source (§2 table, new row) and a degraded planner (§3.7) both stop as gently as their own collision check allows, since an upstream process dying is not evidence that anything is bearing down on the car.
- (2026-09-09) **An MPC solver failure uses the iterate and escalates, rather than braking on the spot.** The (2026-09-08) entry above routed a solver failure straight to `emergency_stop`; that is too heavy once the QP is looked at closely. It constrains inputs only, so holding the previous input is always feasible: it cannot be primal infeasible, and with `R ≻ 0` it cannot be unbounded. The only realistic failure is the iteration budget running out — made likelier by §5's own `adaptive_rho: false` and fixed `max_iter`, and expected only at trajectory discontinuities where the warm start is poor (reset, a safety-layer intervention switching homotopy, a lane-change commit). Meanwhile `emergency_stop` is the *hardest* stop in the stack: brake 1.0 at the adapter, below the planning limit `a_min`, with no trajectory followed. Jumping there for a near-optimal-but-untightened iterate is disproportionate, so the node now publishes the clamped iterate, counts it, and escalates only after `max_consecutive_solver_failures` consecutive ones — node-local and recoverable, unlike the latching degradation of `02_interfaces.md` §2. Two traps recorded with it: a failed `polish` step leaves a valid solution and must not be counted as a failure, and an `emergency_stop` tick must carry `δ_ref` rather than steer 0, since `control_adapter` overrides only throttle and brake and a centred wheel under full braking leaves the lane on a curve.

## 7. Open questions

- Whether `lane_follow` const-vel prediction is enough at unsignalized junctions, or whether M1 needs a simple "route-following" heuristic for other agents (assume they continue on their most likely successor lane). Decide by measuring collision infractions at junctions after tuning.
