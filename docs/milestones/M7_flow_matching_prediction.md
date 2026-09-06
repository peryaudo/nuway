# M7 — Flow-matching joint behavior prediction

**Goal:** replace constant-velocity prediction with a learned, joint, multi-modal prediction model: a scene encoder over agent/map/route/traffic-light/occupancy tokens and a flow-matching velocity network that generates S joint samples of all agents' futures. The ego is included in the generated tensor (for joint consistency) but the ego channel is **not** used for planning in this milestone; the planner remains M1's lattice.

**Completion criteria**
- [ ] Open-loop (held-out Town07 + 10% of other towns): vehicles minADE@8s (S=16) < 1.2 m, minFDE@8s < 2.8 m; pedestrians minADE@8s < 0.8 m; sample collision rate (agent–agent overlap within a sample) < 2%; offroad rate < 3%. Beats the M6 lane-follow constant-velocity baseline on minADE@3s by ≥ 40% for vehicles (this also serves as the validation of the M6 data).
- [ ] Invariance tests pass: rotating/translating the scene rotates/translates outputs (max deviation < 5 cm); occupancy perturbation test changes predictions near the inserted obstacle.
- [ ] Runtime: encoder + 6 Euler steps × 16 samples ≤ 25 ms p50 on the 3090 Ti in fp16, 10 Hz.
- [ ] Closed-loop: profile `m7_learned_prediction.yaml` (no GT anywhere, lattice planner) driving score ≥ M5 score + 10 on the M1 protocol; collisions with vehicles at junctions reduced by ≥ 30% vs M5.

---

## 1. Problem definition

Generate `x ∈ R^{A×T×3}`: for A ≤ 32 agents (slot 0 = ego), T = 16 timesteps at 0.5 s (8 s), channels `(Δx, Δy, Δψ)` relative to each agent's **current** pose, expressed in the ego frame at `t0`, divided by per-channel scales.

Normalization scales (`configs/training/fm_prediction.yaml: norm`), computed from the training set by `ml/scripts/compute_norm_stats.py` (std of each channel at each t, then a single scalar per channel = mean over t of the std): expected ≈ `scale_xy ≈ 20 m`, `scale_yaw ≈ 1.0 rad`. Assert at load that `x_1` has std within [0.5, 2.0] per channel.

## 2. Tokenizer (`nuway_ml/prediction/tokenizer.py`)

All in ego frame at `t0` (ROS convention). Produces:

| token set | count | features per token (before MLP) | position for PE |
|-----------|-------|--------------------------------|-----------------|
| agent | A ≤ 32 | history `[20,(x,y,cosψ,sinψ,vx,vy,valid)]` flattened → MLP; class one-hot; dims (l,w,h); `is_ego`; `visible` | current (x, y, ψ) |
| map | M ≤ 256 | polyline segment of 20 points at 1 m: `(x,y,cosθ,sinθ, width, type onehot, speed_limit/30)` → PointNet (MLP + max-pool); lane-change-allowed flags | segment center (x, y, θ) |
| route | R ≤ 32 | reference-line segments of 10 m: `(x,y,cosθ,sinθ,κ,left_bound,right_bound,speed_limit/30)` → PointNet | segment center |
| traffic light | L ≤ 8 | state one-hot, stop line `(x,y)`, distance along route | stop line (x, y, θ_lane) |
| occ patch | 64 | `f_coarse` from OccEncoder pooled to 8×8 | patch center (x, y), θ = 0 |
| goal | 1 | `(x,y)` of route point at +150 m, distance-to-goal/500 | that point |

Padding: fixed sizes with boolean `valid` masks. Agents sorted by distance to ego (ego first). Map lanes selected by distance to ego ≤ 100 m, prioritizing lanes reachable from the route.

Positions are fed to the relative PE module; no absolute PE is added to token features except `is_ego`.

## 3. Scene encoder (`scene_encoder.py`)

```
OccEncoder: [6,200,200] → stem(s2) → ResBlock(s2) → ResBlock(s2)  ⇒ f_fine [D,50,50] (1 m/cell), f_coarse [D,25,25] (2 m/cell) → pool 8×8 → 64 patch tokens
tokens = concat(agent, map, route, tl, occ_patch, goal)      # N ≤ 32+256+32+8+64+1
for l in 1..6:
    tokens = SelfAttnBlock(tokens, rel_pe(positions), mask, neighbor_mask(radius=60 m))
    tokens = DeformCrossAttn(tokens, f_fine, ref=positions)     # only on layers 2,4,6
context = tokens                                                 # [N,D], D=256
```
- `relpe.py`: pairwise `(Δx,Δy,cosΔθ,sinΔθ,log(1+dist))` in the query token's local frame → Fourier features (8 freqs) → MLP(64→64→heads) as attention bias, plus a value-side projection `W_r·r_ij` added to values. Computed once per layer; memory `[N,N,heads]`. Neighbor mask: tokens further than 60 m don't attend (except ego/goal tokens attend all).
- `deform_attn.py`: per token, K=4 sample points = `ref + R(θ)·offset_k`, offsets from a linear layer initialized to a fixed star pattern (radii 2, 5, 10 m, forward-biased), weights softmax over K, bilinear `grid_sample` on `f_fine`. Sampling coordinates through `GridSpec` (tests: world→grid parity with C++).
- 6 layers, D=256, 8 heads, pre-LN, GELU MLP ratio 4. ~12M parameters.

## 4. Velocity network (`velocity_net.py`)

DiT-style, operating on agent tokens only (A tokens), cross-attending to `context`:
```
h = Linear(T*3 → D)(x_t.flatten) + agent_context_token           # agent_context_token = context[agent slots]
for l in 1..4:
    h = AdaLN(h, t_emb)                       # t_emb = MLP(Fourier(t))
    h = h + SelfAttn(h, rel_pe over agents)   # joint interaction
    h = h + CrossAttn(h, context)
    h = h + MLP(h)
v = Linear(D → T*3)(h)  reshaped [A,T,3]     # zero-init
```
D=192, 4 heads, ~4M parameters. Padded agent slots are masked in attention and loss.

## 5. Training (`flow_matching.py`, `train.py`)

```python
x_1 = normalize(future)                        # [B,A,T,3]
x_0 = randn_like(x_1)
t   = sigmoid(randn(B) * t_sigma)              # logit-normal, t_sigma=1.0
x_t = (1-t) x_0 + t x_1
v   = velocity_net(x_t, t, context)
loss_fm = masked_mse(v, x_1 - x_0)
x1_hat = x_t + v * (1 - t)
loss_aux = t * ( w_kin * kinematic(x1_hat) + w_col * collision(x1_hat) + w_road * offroad(x1_hat, occ, drivable) )
loss = loss_fm + loss_aux
```
- `aux_losses.py`: `kinematic` = penalty on |curvature| > κ_max and |accel| > 4 m/s² from finite differences (vehicles only); `collision` = soft overlap between agent discs across pairs at each t; `offroad` = `1 − drivable` sampled along each vehicle's trajectory (bilinear, differentiable) + `occupied` sampled likewise. Weights start at 0 for the first 2 epochs, ramp to `w_kin=0.1, w_col=0.5, w_road=0.5`.
- Invisible agents (`visible=false`) are context but not loss targets. `visible` has the same meaning in M2 (sensor-derived) and M6 (raycast-derived) records: "the hero could plausibly perceive this agent"; the tokenizer feeds it as a feature and the loss mask reads the same flag.
- Hydra application, config `configs/training/fm_prediction.yaml` (`03_style_and_conventions.md` §9.7); the normalization scales above are its `data/norm` group, written by `compute_norm_stats.py` and composed in, never pasted into the model config.
- AdamW lr 3e-4, cosine, batch 64, 30 epochs on 3M frames (≈ 1.5 days on the 3090 Ti in fp16). EMA weights 0.999 used for eval/export.
- Validation each epoch: sample S=16 with 6 Euler steps, compute metrics from M6 §7, render 16 fixed scenes with samples.
- W&B group `m7`. Per-step `train/loss_total`, `train/loss_fm` and each aux term separately (`train/loss_kin`, `train/loss_col`, `train/loss_road`) plus their current weights (`train/w_kin`, …) — the ramp in the first 2 epochs is the thing most likely to destabilize this run, and it is only visible if the terms and their weights are logged apart. Also `train/lr`, `train/grad_norm`, and `train/loss_fm_by_t` (the FM loss bucketed into 4 `t` bins), which is how a bad `t` distribution shows itself.
- Per-epoch `val/minade_16`, `val/fde`, `val/collision_rate`, `val/offroad_rate` under `val/`, the 16 fixed scenes as `val/viz` images, and the step-count table (1/2/4/8/16 Euler steps) as a W&B table at the end of training. Best checkpoint uploaded as `m7_best`.
- A 1k-frame overfit run is the same entry point with `data=overfit_1k logging.wandb.mode=disabled`; it must reach ~0 loss before a full run is launched.

Sampler (`flow_matching.py: sample`): Euler, `n_steps` configurable (train-time eval uses 6; also report 1, 2, 4, 8, 16 in a table). Optional cost guidance hook (used in M10).

## 6. Runtime node (`nuway_prediction/prediction_node.py`)

- Subscribes agents (base_link), lane graph (latched; tokenizer caches per-lane polylines), reference line, traffic lights, occupancy, pose. Runs once per two ticks, triggered by the agents message. The occupancy subscription is the second sanctioned grid-into-Python case of `00_overview.md` principle 6.
- Builds tokens on GPU (numpy→torch, pinned), runs encoder once, samples S=16, denormalizes, transforms to `map` frame, publishes `PredictionSamples` with `sample_weight = 1/S`. Ego channel dropped from the message (M8 adds the ego planning head to this same node and publishes `/nuway/planning/learned_candidates` from it).
- Stateless across cycles except the per-lane polyline cache, which is keyed by town and survives `ResetEvent`; nothing else needs clearing.
- Agent slot limit: nearest 31 + ego; others beyond are given constant-velocity futures by the node (published in the same message) so downstream never loses an agent.
- `torch.compile` fp16; CUDA graph for the velocity net loop if needed. Diag breakdown: tokenize / encoder / sampling / postprocess.

## 7. Tests

- `test_invariance.py`: random scene, rotate+translate all inputs, compare denormalized outputs after inverse transform (tolerance 5 cm). Runs on an untrained model too (structural invariance) — must pass before training starts.
- `test_occ_sensitivity.py`: trained model; insert an `occupied` block 15 m ahead of a moving vehicle; assert its samples slow down or deviate (mean speed at 3 s decreases by > 1 m/s or lateral deviation > 1 m).
- `test_agent_identity.py`: two agents with different histories in identical positions must get different outputs (guards against dropped identity embedding).
- `test_masks.py`: padded agents don't influence real agents' outputs.
- Tokenizer parity: Python tokenizer vs a stored fixture generated by the runtime node from a recorded MCAP (ensures training/serving skew is zero).

## 8. Task list

1. [ ] `tokenizer.py` + fixtures + parity test.
2. [ ] `relpe.py`, `deform_attn.py` (+ star init), `scene_encoder.py`; invariance test on untrained model.
3. [ ] `velocity_net.py`; `flow_matching.py` (train step, sampler, guidance hook); `aux_losses.py` + tests (differentiability, finite-difference gradient check on offroad sampling).
4. [ ] `train.py`, config, `compute_norm_stats.py`; short overfit run on 1k frames (loss → ~0, samples reproduce futures).
5. [ ] Full training; open-loop report; step-count table.
6. [ ] `prediction_node.py`; export; latency benchmark; Foxglove prediction layer (samples as faded polylines, per-agent).
7. [ ] Closed-loop eval with lattice planner; compare vs M5; junction collision analysis.
8. [ ] `tests/integration/test_m7_learned_prediction.py` (short route, `m7_learned_prediction.yaml`, asserts completion and that `/nuway/prediction/samples` carries S=16).

## 9. Decisions log

- (2026-09-05) Training configuration is Hydra (structured configs in `nuway_ml/common/config.py`, groups under `configs/training/`) and run logging is Weights & Biases; tensorboard is dropped. One config system and one metrics store for every milestone, so runs are launched by override and compared on one chart. See `03_style_and_conventions.md` §9.7.
- (2026-09-02) Flow matching (rectified-flow parametrization, x-independent linear path, logit-normal t) instead of DDPM/DDIM: fewer sampling steps, simpler code; same guidance capability.
- (2026-09-02) One token per agent for the whole trajectory in the velocity net (T folded into the feature dim). Time-tokenized variant deferred.
- (2026-09-02) Ego is generated jointly but unused in M7 planning.

## 10. Open questions

- S=16 vs 32 samples: latency vs coverage. Measure minADE vs S on validation; pick the knee.
- Whether route tokens leak "GT-like" intent for the ego channel that other agents shouldn't see. They should only attend to the ego's slots… leave as is (all tokens visible) unless ego-conditioning artifacts appear in M8.
