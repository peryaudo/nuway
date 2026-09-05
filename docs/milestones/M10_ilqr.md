# M10 — iLQR refinement on the ML-planner path (+ optional guidance)

**Goal:** replace the piecewise-jerk QP refinement of *learned* candidates with an iLQR refinement that optimizes the full nonlinear kinematic model jointly in space and time (no path–speed decomposition), with smooth non-convex obstacle costs derived from prediction samples and occupancy. The classical (lattice) path keeps the QP. Optionally, use the same differentiable costs as guidance inside flow-matching sampling.

**Completion criteria**
- [ ] No regression in driving score vs M9 (QP refinement) on the M1 protocol and scenario routes; improvement in comfort metrics (mean |jerk| ↓ ≥ 10%, max |a_lat| ↓) and in tracking error of MPC (mean lateral tracking error ↓).
- [ ] iLQR refinement of K=8 candidates over the full 8 s horizon (N=80) ≤ 12 ms p50 (C++, threaded), converged (cost decrease < 1e-3 relative) in ≤ 8 iterations from a learned-trajectory warm start in ≥ 95% of cycles.
- [ ] Unit tests: gradient/Hessian checks vs finite differences; regularization/line-search behavior on a contrived non-convex obstacle case; warm-start speedup measured.
- [ ] Guidance (if enabled): sample collision rate ↓ ≥ 30% with ≤ 1.6× sampling latency; documented on/off comparison.

---

## 1. Formulation (`nuway_planning/src/ilqr.cpp`, header `ilqr.hpp`)

State `x = [X, Y, ψ, v, δ]`, input `u = [a, δ̇]`, `dt = 0.1`, `N = 80` (the full 8 s horizon, so the refined output is an 81-point `Trajectory` like every other candidate and the selector's horizon-normalised terms stay comparable; `02_interfaces.md` §4). Prediction uncertainty in the far tail is handled by the time-decayed tracking weight and by the sample weighting, not by truncating the horizon. Dynamics from `bicycle_model.hpp` (RK2), analytic Jacobians `A_t, B_t` (tests vs finite differences).

Cost:
```
l_t(x,u) = w_ref(t) · ‖p(x_t) − p_ref(t)‖²_(lon/lat weighted)          # track the learned candidate
         + w_ψ · (ψ_t − ψ_ref(t))² + w_v · (v_t − v_ref(t))²
         + w_a · a² + w_δ̇ · δ̇² + w_jerk · (a_t − a_{t−1})²
         + Σ_agents Σ_samples w_s · w_obs · exp(−d²_{agent,t}/σ²)       # d = signed distance ego disc-set ↔ agent OBB (smooth approx via disc–disc)
         + w_occ · occ(p_t ± corners)                                   # bilinear sample of `occupied` + softplus barrier
         + w_bound · softplus(−(left_bound − d_lat)) + softplus(−(d_lat + right_bound))   # Frenet lateral bounds
         + w_lim · [softplus(|δ|−δ_max) + softplus(a − a_max) + softplus(a_min − a)]      # soft input limits
l_N(x)   = terminal versions of the tracking terms with 5× weights
```
`w_ref(t) = w_0 · exp(−t / 4 s)` as in M8. Agent costs use the same per-candidate sample weighting as M8 (own sample 0.6, marginal 0.4).

## 2. Algorithm

Standard iLQR with:
- Backward pass computing `Q_x, Q_u, Q_xx, Q_ux, Q_uu` with Gauss-Newton (drop second-order dynamics terms), Levenberg–Marquardt regularization `μ` on `Q_uu` (start 1e-3, ×10 on failure, ÷3 on success, cap 1e6).
- Forward pass with backtracking line search `α ∈ {1, 0.5, 0.25, 0.125, 0.0625}`; accept if actual cost decrease > 1e-4 × expected.
- Convergence: relative cost change < 1e-3 or `max_iter = 15`.
- Input clamping after the forward pass as a hard guard (soft limits in cost do the real work).
- Warm start: `u` sequence inverted from the learned candidate (smooth first with a 5-point moving average on `d(s)`, then invert kinematics: `δ = atan(L·κ)`, `a` from `Δv/dt`); on subsequent cycles, warm start from the previous refined solution of the matched mode, shifted by one step.
- Output `Trajectory` at 0.1 s, 81 points (states already on that grid), `source="learned"`, `sample_index` carried over from the input candidate, plus diag: iterations, final cost, regularization hits, line-search failures.
- Warm-start memory (previous refined solution per mode) is cleared on `ResetEvent`.

Threaded over candidates (thread pool). Costs are evaluated on preinterpolated agent samples (shared read-only per cycle).

## 3. Wiring

`configs/profiles/*.yaml: planning.refiner: ilqr` selects iLQR for `source=="learned"` candidates; lattice candidates remain QP-refined (config `planning.lattice_refiner: qp`, fixed). `planner_node` chooses the refiner per candidate source.

## 4. Guidance in flow-matching sampling (optional, `flow_matching.py: sample(..., guidance=...)`)

```
for each Euler step:
    v = velocity_net(x, t, c)
    x1_hat = x + v (1 − t)
    J = w_col · collision(x1_hat) + w_road · offroad(x1_hat, occ, drivable) + w_kin · kinematic(x1_hat)   # reuse aux_losses
    g = ∇_x J  (autograd, batched over samples)
    x = x + (v − s(t) · g) · dt         # s(t) = s_0 · t (weaker early, stronger late)
```
Implemented in the runtime node behind `prediction.guidance.enabled`; `s_0` tuned on open-loop sample collision rate vs minADE (guidance should not increase minADE by > 5%). Latency budget check against the 25 ms target; if exceeded, apply guidance only on the last 2 steps.

## 5. Tests

- `test_ilqr_jacobians.cpp`: analytic vs finite-difference `A_t, B_t`, and cost gradients/Hessians for each term.
- `test_ilqr_convergence.cpp`: straight road, one static obstacle in lane, warm start from a straight line → converged solution deviates laterally and returns; cost monotone non-increasing across accepted iterations.
- `test_ilqr_warmstart.cpp`: second solve from shifted solution converges in ≤ 3 iterations.
- `test_ilqr_vs_qp.py` (integration, offline on recorded scenes): distribution of curvature/jerk of refined outputs; iLQR ≤ QP on both.
- `test_guidance.py`: on 200 validation scenes, collision rate with vs without guidance; minADE change.

## 6. Task list

1. [ ] `ilqr.hpp/cpp` core (backward/forward passes, regularization, line search), Jacobian tests.
2. [ ] Cost terms (tracking, comfort, agent samples, occupancy, bounds, limits) with analytic derivatives; tests.
3. [ ] Warm-start inversion from trajectories; tests.
4. [ ] `planner_node` integration (per-source refiner), config, diag; timing.
5. [ ] Closed-loop comparison QP vs iLQR (same checkpoints); comfort/tracking metrics; report `data/eval_runs/m10_report.md`.
6. [ ] Guidance in sampler + node flag; open-loop and closed-loop on/off comparison.
7. [ ] Update Foxglove candidate table with iLQR diag columns.

## 7. Decisions log

- (2026-09-02) iLQR only on the learned path (good warm start, non-convex costs wanted); QP stays on the classical path (convexity, feasibility detection, determinism).
- (2026-09-02) Gauss-Newton iLQR (no second-order dynamics terms); DDP not needed for a kinematic bicycle.
- (2026-09-05) Full 8 s horizon (N=80) instead of 5 s: a shorter refined trajectory would win the selector's progress term and lose collision exposure for structural rather than behavioural reasons. Timing budget raised from 8 to 12 ms accordingly (`00_overview.md` §5).

## 8. Open questions

- Whether to extend the state with `a` (input = jerk) for smoother longitudinal profiles. Try only if jerk metrics remain worse than the QP's.
- Whether the occupancy term needs a distance transform (smoother gradients than bilinear-sampled probabilities). Add a precomputed EDT channel in `planner_node` if line search frequently fails near obstacles.
