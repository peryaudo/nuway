# M8 — Learned planning head + QP refinement + DAgger

**Goal:** add an ego planning head to the M7 model that emits M candidate ego trajectories with scores; refine them with the M1 piecewise-jerk QP; select with the rule-based selector (M9 upgrades this); keep the lattice path as a live fallback; close the imitation-learning loop with DAgger against the M6 expert.

**Completion criteria**
- [ ] Closed-loop, no GT anywhere, profile `m8_learned_planner.yaml`: driving score > M7 (lattice) score on the M1 protocol **and** on the M6 scenario routes; safety-layer intervention rate < 50% of the lattice path's; comfort (mean |jerk|) not worse. The report also states the gap to the M1 GT baseline and to the M6 expert; there is no pass/fail threshold on that gap (`00_overview.md` §3), but it must be printed.
- [ ] Fallback engages correctly in an injected-failure test (prediction node killed, so learned candidates vanish → lattice path continues within 0.3 s; learned candidates all infeasible → lattice selected).
- [ ] After ≥ 3 DAgger rounds: closed-loop score improves monotonically or plateaus; report shows it.
- [ ] Learned candidates are QP-refined in ≤ 5 ms for M=8 on CPU.

---

## 1. Planning head (`nuway_ml/planning/planning_head.py`)

Two candidate sources from the same model; both are used.

**Source A — joint samples.** The ego slot of the M7 flow-matching samples: S=16 ego trajectories, each *consistent with the other agents' futures in the same sample*. Free (already computed).

**Source B — dedicated planning head.** On the shared `context`:
```
q = learnable queries [M=8, D] + goal token + ego context token
q = 2 × (SelfAttn(q) + CrossAttn(q, context) + MLP)
traj = MLP(q) → [M, T=16, 3]  (normalized like M7)
score = MLP(q) → [M]
```
Trained with winner-takes-all imitation: `L_wta = min_m ‖traj_m − expert‖` + `CE(score, argmin)` + aux losses from M7 (kinematic/offroad/collision against the **M7 predicted samples**, so the head learns to avoid predicted agents) + a diversity term (pairwise repulsion between modes, small weight).

Source B gives a fast, deterministic, expert-like default; Source A gives interaction-consistent alternatives. Both feed the same refinement + selection.

Training is joint with M7 (multi-task, `w_plan = 1.0`) from the M7 checkpoint, 10 epochs, then DAgger rounds (§4). Config `configs/training/planner.yaml` composes the M7 groups plus `model/planning_head` and `stage/joint`, so the multi-task run and the M7 run differ by a defaults list rather than a forked config (`03_style_and_conventions.md` §9.7). W&B group `m8`, `job_type=train`: `train/loss_wta`, `train/loss_score`, `train/loss_div` and the inherited M7 terms are logged separately, so a regression in prediction quality caused by the joint loss is visible at the term level.

## 2. Runtime (`nuway_prediction/prediction_node.py` + `planner_node` changes)

There is no separate learned-planner node. The ego planning head runs inside the M7 node `nuway_prediction/prediction_node.py`, so the encoder runs once per cycle; the node publishes both `/nuway/prediction/samples` and `/nuway/planning/learned_candidates` (`TrajectoryCandidates`, `source="learned"`). This is the "one module, one process" case named in `00_overview.md` principle 2: the shared `context` tensor never leaves the process.
- Ego candidates (S + M = 24) at 0.5 s → interpolated to 0.1 s by a C² spline (the Python twin of `nuway_common/trajectory.hpp::Resample`, `nuway_ml/common/trajectory.py`) with speed/accel from derivatives; 81 points each.
- Each candidate carries `Trajectory.sample_index` = the index of the prediction sample it is consistent with (Source A) or `-1` (Source B) (`02_interfaces.md` §4).

`planner_node` (C++) now:
1. Collects candidates from all `planning.candidate_sources` (`lattice`, `learned`).
2. Feasibility filter + collision check (M1) — for Source A candidates, collision is checked against **their own sample** with weight 0.6 and against the marginal over all samples with weight 0.4 (config).
3. **QP refinement of learned candidates**: the learned trajectory becomes `d_ref(s)` / `s_ref(t)` in the M1 path/speed QPs (project to Frenet on the reference line; `w_d`/`w_s` raised so the QP tracks the learned shape, with time-decayed weight: `w(t) = w_0 · exp(−t/4 s)`), with the same bounds and S–T constraints. This yields kinematically feasible, bounded, smooth versions. Top-K by pre-QP cost (K=8) to bound runtime.
4. Selector (rule-based in M8; forward-sim in M9) over lattice + refined learned candidates jointly. A `source_bias` cost term (config, default −5 for learned) breaks ties in favor of the learned path.
5. Fallback logic: if `learned` candidates are absent for > 0.3 s, or all are infeasible, or the selected learned candidate's cost exceeds the best lattice candidate's cost by > `fallback_margin`, select from lattice only and set `Trajectory.source="fallback"`. Publish a diag warn with the reason. Hysteresis: after a fallback, require 1 s of valid learned candidates before returning.

Safety layer and MPC unchanged.

## 3. Consistency & smoothing

Time consistency term in the selector (`consistency` cost, M1) is raised for learned candidates to suppress mode switching between cycles. Additionally, the QP is warm-started from the previous cycle's refined solution of the same mode (matched by nearest end point).

## 4. DAgger loop (`nuway_ml/planning/dagger.py`, `tools/collect/collect_dagger.py`)

Round r:
1. Run the current student (full stack, no GT, profile m7) on the collection route set with the M6 scenario library, `n_routes` ≈ 200, render-free is **not** possible here (perception needs sensors) → run with rendering at default (Epic) quality — Low is unusable (`00_overview.md` §4): at the measured 15–26 ticks/s (`00_overview.md` §4 table) and ≈ 3 min of sim time per route this is ≈ 8–14 h per round on one GPU, so rounds are run overnight. Alternative "cheap DAgger": `use_gt.perception=true`, `use_gt.traffic_lights=true`, `use_gt.localization=true`, render-free, with `gt_perception.source: geometry` (`02_interfaces.md` §5): occupancy and agent visibility come from the M6 static map + raycast (M6 §3.1) instead of the semantic LiDAR the sensor mode needs — use this for rounds 1–2, then one rendered round.
2. At every frame, also run the expert (`gt_planning_node` from M6 alongside the student, both subscribed to the same GT inputs) and log its trajectory + decision as the label; the student's *own* trajectory is logged too. Frames are `PlanningFrame` with `expert.*` filled by the expert and `student_traj` added.
3. Aggregate: dataset_r = dataset_{r−1} ∪ new frames (weight new frames ×2 for the first epoch).
4. Fine-tune planning head (+ encoder at 0.1× lr) for 3 epochs: `uv run python -m nuway_ml.planning.train stage=dagger dagger.round=<r>`, one W&B run per round (`job_type=dagger`, tagged `round_<r>`).
5. Evaluate closed-loop on the dev protocol; log to `data/eval_runs/dagger_round_<r>/` and log the driving score, infraction counts and fallback rate to the same W&B run as `eval/` scalars, so the round-over-round curve that decides the stopping rule below is one chart rather than a set of directories to diff.

Stopping: 3 rounds minimum; stop when the dev score improvement < 1 point for 2 rounds.

Perception-noise curriculum: DAgger rounds decrease the synthetic occupancy/agent noise of M6 §6 (since real perception noise is now in the data).

## 5. Tests

- `test_fallback.py` (integration): kill `prediction_node` mid-route → completion still succeeds, `source` transitions logged.
- `test_qp_refine_learned.py`: learned candidate violating curvature and bounds → refined output satisfies both; deviation from the reference shape within 0.5 m at 2 s.
- `test_candidate_provenance.py`: Source A candidates carry `sample_index`; collision check uses the matched sample.

## 6. Task list

1. [ ] `planning_head.py` + losses; multi-task config; train from M7 checkpoint (10 epochs); open-loop ego L2 report.
2. [ ] Ego head in `prediction_node.py`: publish `TrajectoryCandidates(source="learned")`; spline resampling to 81 points; `sample_index` provenance.
3. [ ] `planner_node`: multi-source collection, per-sample collision weighting, QP refinement of learned candidates with decayed tracking weight, source bias, fallback + hysteresis.
4. [ ] Profile `m8_learned_planner.yaml`; closed-loop eval vs M7.
5. [ ] `collect_dagger.py` (both cheap and rendered modes), `dagger.py`; run ≥ 3 rounds; report.
6. [ ] Integration tests above.
7. [ ] Update Foxglove: color candidates by source; show fallback state.

## 7. Decisions log

- (2026-09-05) Training configuration is Hydra (structured configs in `nuway_ml/common/config.py`, groups under `configs/training/`) and run logging is Weights & Biases; tensorboard is dropped. One config system and one metrics store for every milestone, so runs are launched by override and compared on one chart. See `03_style_and_conventions.md` §9.7.
- (2026-09-02) Refinement of learned output via the existing M1 QP (not iLQR) in M8. iLQR is M10.
- (2026-09-02) Two candidate sources (joint samples + dedicated head). If the dedicated head is not pulling its weight after DAgger (never selected), drop it and document.
- (2026-09-05) No `learned_planner_node.py`: the head lives in the prediction node. One encoder pass, one process, one ROS boundary.
