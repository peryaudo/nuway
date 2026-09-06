# M4 — Traffic light perception (map-projected crop classifier + association + latch)

**Goal:** replace GT traffic light state with a learned classifier on camera crops, without changing what the planner consumes: `traffic_light_node.py` publishes the same `/nuway/perception/traffic_lights` (`TrafficLightArray`) as `gt_traffic_light_node`. The milestone owns everything between "a traffic light exists in the map" and "the planner knows its state": which light governs ego's lane, where it is in which camera, what state it shows, and what to report when it is no longer visible. **Prerequisites:** M0 map + GT publisher, M2 TL labels. The model, association and latch work (§1–§5) is independent of M3 and can proceed in parallel; the closed-loop criterion and the in-process hosting option (§6) need M3. The planner-side rule for yellow lights is fixed in M1 and is not touched here.

**Completion criteria**
- [ ] Offline (held-out Town07, all weathers): state accuracy > 0.97 on crops within 40 m, > 0.93 within 60 m; `visible` AUROC > 0.98. Report per state and per weather; night and rain separately.
- [ ] Association: for every town in the M2 collection set, the lane→traffic-light mapping from the OpenDRIVE parser agrees with CARLA's `get_affected_lane_waypoints()` on 100% of signalized lanes, **or** each disagreement is resolved by the geometric fallback (§2) and listed in `data/maps/<town>/tl_association_report.md`.
- [ ] Latch: in a scripted approach (Town03, 4 junctions × {stop, pass}), the state published while the light is out of view within 8 m of the stop line equals the last confidently observed state; no `unknown` is published inside 8 m unless the light was never seen.
- [ ] Runtime: crop + classify + latch ≤ 5 ms p50 for up to 4 lights, 10 Hz, fp16, in the same process as `perception_node.py` or standalone.
- [ ] Closed-loop: profile `m4_learned_tl.yaml` (`use_gt.perception=false`, `use_gt.traffic_lights=false`, `use_gt.localization=true`) on the M1 protocol: red-light infraction count ≤ 1.5× the `m3_learned_perception` run, driving score ≥ 95% of it. Also run with `use_gt.perception=true` to isolate the TL contribution.

---

## 1. Model (`nuway_ml/traffic_light/model.py`)

Tiny CNN: 4 × (3×3 conv + BN + ReLU, channels 32/64/128/128, stride 2 on the last three) → global average pool → 2 heads: `state` (4-class softmax: red/yellow/green/off) and `visible` (1 logit). Input `[3,64,64]` RGB crop, normalized with ImageNet statistics. ≈ 0.3M parameters; fp16 batch of 4 crops ≤ 1 ms.

`unknown` is never predicted directly: it is the output when `visible < 0.5` or max softmax `< 0.6` and no latched state exists (§4).

## 2. Lane → traffic light association (`nuway_map` + `tools/mapping/verify_tl_association.py`)

A junction shows 4–8 lights; exactly one governs ego's lane. Getting this wrong fails in one of two ways, both fatal: run a red (wrong light is green) or wait forever (wrong light is red). Two sources, cross-checked per town:

1. **OpenDRIVE** (M0 parser): `<signal type="1000001">` on a road, with `<validity fromLane toLane>` giving the governed lane range; the signal's `s`/`t` gives the stop line. This is the runtime source (`LaneGraph.traffic_lights`, `TrafficLightMapping`).
2. **CARLA API** (GT publisher, M0): `tl.get_affected_lane_waypoints()` + `tl.get_stop_waypoints()`. Not available at runtime without a CARLA client, but correct by construction.

`verify_tl_association.py --town <T>` builds both mappings, matches signals to actors by stop-line position (< 2 m), and reports every lane whose governing light differs. Known CARLA quirk: some towns have `<validity>` ranges that do not match the actor's affected lanes (typically lanes added by hand in the junction). **Fallback** when the OpenDRIVE mapping for a lane is missing or was flagged by the report: geometric matching — the light whose stop line lies on the lane's successor path within 30 m *and* whose orientation faces the lane (angle between lane direction at the stop line and the signal's `hOffset` < 45°). Fallback decisions are stored in `data/maps/<town>/tl_overrides.yaml` and loaded by the map server, so the runtime never recomputes them.

Task for M0 retroactively: none; the parser already emits signals. M4 adds the verification script, the override file, and the loader.

## 3. Projection and cropping (`nuway_ml/traffic_light/projection.py`, shared by node and labeler)

For each light on the route with stop line within 60 m ahead (from `/nuway/route/plan` + `LaneGraph`): bulb box center in map frame (from M2 labels: CARLA `get_light_boxes()`, stored once per town in `data/maps/<town>/tl_bulbs.json`) → `T_cam_from_map` (pose + static extrinsics) → pixel. Choose the camera with the largest projected bulb area among `cam_front`, `cam_left`, `cam_right`; crop a square of side `max(32 px, 2.5 × projected bulb height)` padded 25%, resize to 64×64. If no camera sees it (behind, above FOV, projected area < 4 px), emit no crop and let the latch (§4) answer.

Camera FOV limit: with the 90° rig the light leaves `cam_front`'s upper edge at roughly 5–8 m before the stop line for a light mounted on the far side, and much earlier for a near-side pole. Two mitigations, both cheap:
- `cam_front` pitch +10° is **not** adopted (breaks M3's BEV depth assumptions). Instead, `rig_dev.json` gains an optional fifth camera `cam_tl` (704×256, FOV 100, pitch +15°, topic `/carla/hero/cam_tl`, `02_interfaces.md` §3.1) that only this node subscribes to. Decide in §7 by measuring the "blind distance" per junction type with and without it; the completion criterion must be met without it, because `rig_leaderboard.json` cannot carry it (the Leaderboard caps the number of RGB cameras; M1 §3.12) and the latch must therefore suffice on its own.
- The latch, which is required regardless.

Same code path is used by the M2 labeler (`crop_cam`, `crop_bbox`) so train and inference crops are identical by construction; add a test that re-cropping an M2 frame reproduces the stored bbox within 2 px.

## 4. Visibility latch (`nuway_ml/traffic_light/latch.py`)

Per light id, a small state machine that turns per-frame classifier outputs into a published state:

- **Per-frame vote:** majority over the last 5 classifier outputs with `visible > 0.5` (a filter, not a Kalman filter; documented). A light state becomes *confirmed* after 3 agreeing votes; `confidence` = fraction of agreeing votes × mean softmax.
- **Latch:** when the light leaves the view (no crop, or `visible < 0.5` for 3 consecutive frames) and `d_stop < 15 m`, keep publishing the last *confirmed* state with `confidence` decaying linearly to 0.5 over 4 s. Beyond 15 m, publish `unknown` (the planner treats unknown as red when it can still stop; see M1 §3.2).
- **Release:** the latch clears when the stop line is passed (`s_ego > s_stop + 2 m`), when the light comes back into view and a new state is confirmed, or after 10 s.
- **Message fields:** every published light carries `time_in_state` = seconds since the current state was first *confirmed* (a lower bound on the true elapsed time), `yellow_duration` = the configured conservative default (3 s), `latched` = whether the latch is holding the state, and `confidence` as above. M1's dilemma-zone rule reads `yellow_duration − time_in_state` and therefore errs towards "will not clear in time" when the yellow was observed late.
- **Latched-yellow rule:** if the latch engages on `yellow`, keep publishing `yellow` with `time_in_state` still counting so that the planner's remaining-yellow estimate keeps decreasing; do not upgrade a latched yellow to red on a timer (the planner already decided).
- **Reset:** all vote buffers and latches are cleared on `ResetEvent`.

The risk that the light changes while latched (green → red after ego lost sight of it within 15 m) is accepted: at that distance a light that was green at latch time can only turn yellow, and a yellow of ≥ 3 s is enough to clear the line at ≥ 5 m/s. Document this in the node.

## 5. Training (`nuway_ml/traffic_light/train.py`)

- Data: M2 `tl` crops and states, all towns except Town07. Class balance: `off` and `yellow` are rare — oversample to ≥ 10% each per epoch. `visible` negatives: random 64×64 crops around the projected bulb with offset > 50% (label `visible=0`), and crops of M2 lights whose `crop_cam=None` re-projected anyway.
- Augmentation: color jitter (brightness ±0.4, contrast ±0.3, hue ±0.05 — must not flip red/green; assert on the augmented hue), random crop offset ±20%, random scale 0.8–1.25, motion blur (rain/night), no horizontal flip.
- Hydra application, config `configs/training/traffic_light.yaml` (`03_style_and_conventions.md` §9.7); the augmentation and class-balance knobs above are its `data/` group, so a re-run with different jitter is a command-line override, not an edit.
- AdamW 1e-3, cosine, 20 epochs, batch 256, fp16; 10 minutes on the 3090 Ti. EMA weights.
- Metrics (`metrics.py`): accuracy per state / distance bin (0–20/20–40/40–60 m) / weather; visible AUROC; confusion matrix into the report.
- W&B group `m4`. Per-step `train/loss_total`, `train/loss_state`, `train/loss_visible`, `train/lr`; per-epoch `val/acc_<state>`, `val/acc_dist_<bin>`, `val/auroc_visible` and the confusion matrix as a `wandb.plot` panel. Because the run is 10 minutes, the sweep over jitter and oversampling ratio is a Hydra `--multirun` and is read off one W&B sweep chart; best checkpoint uploaded as `m4_best`.

## 6. Runtime node (`nuway_perception/traffic_light_node.py`)

Subscribes `cam_front`, `cam_left`, `cam_right` (+ `cam_tl` if enabled), `/nuway/loc/pose`, `LaneGraph` (latched), `/nuway/route/plan`. Per cycle (10 Hz, triggered by `cam_front`):
1. Select lights on the route within 60 m (§2 mapping, overrides applied).
2. Project + crop (§3) for those visible in some camera; batch through the model.
3. Latch (§4) per light; publish `TrafficLightArray` with `stop_line`, `affected_lane_ids`, `state`, `confidence`, `time_in_state`, `yellow_duration`, `latched`. Stamp = `cam_front` stamp.
4. `NodeDiag` with breakdown: project, crop, model, latch; plus `n_lights`, `n_latched`.

Runs inside the `perception_node.py` process when the profile sets `perception.tl_in_bev_process: true` (saves an image subscription and a CUDA context); standalone otherwise. Either way the topic and message are identical, and either way the launch alternative is selected by `use_gt.traffic_lights` alone (`02_interfaces.md` §6).

Foxglove: overlay of crop boxes on `cam_front` with predicted state and latch status on `/nuway/perception/tl_debug` (only when `traffic_light_node.debug: true`), plus the `tl_crops` marker layer.

## 7. Integration & evaluation

- Profile `m4_learned_tl.yaml`. Run the M1 protocol twice (`use_gt.perception` true/false); compare with `compare_runs.py` against the `m3_learned_perception` run.
- Blind-distance study: for each junction on the dev routes, record the distance to the stop line at which the governing light leaves every camera, with and without `cam_tl`. Decide whether `cam_tl` becomes part of `rig_dev.json` and record the decision below.
- Failure triage on every red-light infraction: association (wrong light), classification (wrong state), latch (stale state) — tag in `data/eval_runs/m4_report.md`.

## 8. Task list

1. [ ] `verify_tl_association.py` + `tl_overrides.yaml` loader in the map server; run on all M2 towns; commit reports.
2. [ ] `projection.py` + reproduction test against M2 crop bboxes; `tl_bulbs.json` export per town.
3. [ ] `model.py`, `metrics.py`, `train.py`; train; offline report.
4. [ ] `latch.py` + unit tests (scripted sequences: lose sight at 6 m holding green; lose sight at 20 m → unknown; latched yellow with elapsed time; release on stop line).
5. [ ] `traffic_light_node.py` (+ in-process option in `perception_node.py`), launch wiring, profile `m4_learned_tl.yaml`.
6. [ ] Scripted latch integration test `tests/integration/test_m4_latch.py` (4 junctions × {stop, pass}).
7. [ ] Blind-distance study; `cam_tl` decision.
8. [ ] Closed-loop eval; `data/eval_runs/m4_report.md` with infraction triage.
9. [ ] Foxglove overlay.

## 9. Decisions log

- (2026-09-05) Training configuration is Hydra (structured configs in `nuway_ml/common/config.py`, groups under `configs/training/`) and run logging is Weights & Biases; tensorboard is dropped. One config system and one metrics store for every milestone, so runs are launched by override and compared on one chart. See `03_style_and_conventions.md` §9.7.
- (2026-09-05) Separate milestone from M3: no shared model, data, metric or loop, and a larger driving-score lever (red-light penalty 0.70) than a few mAP points.
- (2026-09-05) Runtime association comes from the map (OpenDRIVE + committed overrides), never from the CARLA client, so the node has no privileged dependency.
- (2026-09-05) Latch semantics are decided here, dilemma-zone semantics in M1: the node reports what it knows and how confident it is; the planner decides what to do with it.

## 10. Open questions

- Whether `cam_tl` is worth an extra rendered camera (~3 ms tick cost) for the dev rig only. Resolved by the blind-distance study; it can never be part of the Leaderboard rig.
- Whether to add a small detector instead of map projection for robustness to localization error. Not now: M5 localization is < 0.2 m ATE, and the crop padding of 25% absorbs it.
