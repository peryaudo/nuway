# M3 — BEV perception (BEVFusion + temporal fusion + CenterPoint heads + occupancy)

**Goal:** replace GT perception with a learned LiDAR+camera BEV model that outputs the same `AgentArray` (with velocities, IDs, and history) and `OccupancyGridMC`. Traffic lights stay on GT in this milestone (learned in M4). No Kalman filter: velocities are regressed by the network; IDs come from velocity-projected nearest-neighbor association.

**Completion criteria**
- [ ] Offline: vehicle mAP@BEV-IoU0.5 > 0.6, pedestrian AP@0.3 > 0.4 on a held-out town (Town07), velocity error (AVE) < 0.6 m/s for vehicles; occupancy IoU (`occupied`) > 0.6, (`drivable`) > 0.85.
- [ ] Runtime: end-to-end perception node ≤ 40 ms p50 / 60 ms p99 (fp16, CUDA graphs or TensorRT), 10 Hz, including preprocessing and tracking.
- [ ] Closed-loop: M1 stack with `use_gt.perception=false`, `use_gt.localization=true` scores ≥ 80% of the M1 GT baseline (visibility-on, recorded at the end of M2; `00_overview.md` §3) on the M1 protocol; no infraction category increases by > 2× (esp. collisions with static obstacles).
- [ ] Tracker: ID switches < 0.1 per agent-minute on GT-eval sequences.

---

## 1. Model (`nuway_ml/perception/model/`)

Design targets: ≤ 25M parameters, fp16-friendly, no custom CUDA kernels except an optional sparse-conv library (`spconv` or `torchsparse`); default path is **pillars + dense 2-D conv**, which needs nothing custom.

### 1.1 LiDAR branch (`lidar_branch.py`)
- Input: current sweep in `base_link` `[N,4]` (x, y, z, intensity), i.e. exactly the M2 `lidar` array and exactly what `/carla/hero/lidar_top` carries. Crop to grid extent, z ∈ [−3, 5].
- PointPillars: pillar size 0.25 m (400×400 pillars over ±50 m), max 32 points/pillar, 10-dim point features (x,y,z, Δ to pillar center, Δ to pillar mean, |r|), PillarFeatureNet 64-dim, scatter to `[64,400,400]`.
- 2-D backbone: 3 stages with strides {1,2,2} (channels 64,128,256), FPN-style up-fuse to `[128,200,200]` (0.5 m).
- Hook for `spconv` VoxelNet variant behind a config flag (not required for completion).

### 1.2 Camera branch (`camera_branch.py`)
- 4 images, `ResNet-18` (ImageNet init) → stride-16 features `[256,16,44]` per camera.
- Lift-Splat: depth distribution over 48 bins (2–50 m, uniform in log-depth), context 64-dim; splat to BEV `[64,200,200]` via the standard "cumsum trick" (pure torch). Depth supervised by projecting the LiDAR sweep into each image (BEVDepth-style) with a cross-entropy loss on the bin.
- Camera BEV is a **helper**, not the primary source; if it underperforms it may be dropped by config.

### 1.3 Fusion (`fusion.py`)
Concat `[128+64,200,200]` → 2 × (3×3 conv + BN + ReLU) → `[128,200,200]` = **current BEV feature** `F_t`.

### 1.4 Temporal fusion (`temporal.py`)
Concat-and-conv, no recurrence:
- Keep a queue of raw `F_{t-k}` for k = 1..4 (0.1 s spacing) with their poses.
- Warp each to the current `base_link` via `F.grid_sample` with an affine grid from `T_cur_from_prev` (rotation + translation on the BEV grid; use `GridSpec` helpers; test that a static wall aligns).
- Long-term: an EMA memory `M_t = α · warp(M_{t−1}) + (1−α) · F_t`, α = 0.8 (learnable scalar via sigmoid), at 1/2 resolution `[128,100,100]`, upsampled for fusion.
- `cat([F_t, F_{t-1..t-4} warped, up(M_t)])` → 1×1 conv → 2 × 3×3 conv → `[128,200,200]` = `F_t^fused`.
- Training: `K_prev=4` real previous frames from the dataset; EMA memory approximated during training by a 2-frame unroll (compute `M` from t−2, t−1; stop-grad on the older one) — document the approximation in code.

### 1.5 Heads
**CenterPoint head** (`centerpoint_head.py`) on `F_t^fused` after a shared 3×3 conv (64 ch):
- `heatmap [6,200,200]` (one channel per class incl. static_obstacle), Gaussian targets with CenterNet radius (min radius 2 cells), focal loss.
- `offset [2]`, `z [1]`, `dim [3]` (log l,w,h), `rot [2]` (sin,cos), `vel [2]` (m/s in base_link). L1 losses at positive centers, weights: offset 1, z 1, dim 1, rot 1, vel 2.
- Decode (`decode.py`): 3×3 max-pool peak extraction, top-200, score threshold 0.3 (class-specific later), circle-NMS (radius 1 m vehicles / 0.5 m pedestrians).

**Occupancy head** (`occupancy_head.py`): 2 × 3×3 conv → `[6,200,200]`; sigmoid for probability channels (BCE + Dice for `occupied`, `free`, `drivable`, `dynamic`), softmax constraint for `occupied/free/unknown` triple (train as 3-way CE, derive `unknown`), L1 for `height_max` on occupied cells. Loss weights: occupied 2, free 1, drivable 1, dynamic 0.5, height 0.5.

## 2. Training (`nuway_ml/perception/train.py`)

- Config `configs/training/bevfusion.yaml` (OmegaConf). AMP fp16, AdamW lr 2e-4 (backbone 1e-4), cosine, 24 epochs, batch 4 with grad accumulation 2, EMA weights 0.999.
- Stage A (8 epochs): LiDAR-only, no temporal (`K_prev=0`). Stage B (8 epochs): add camera branch and depth loss. Stage C (8 epochs): temporal on. Each stage starts from the previous checkpoint. This staged schedule is what makes 24 GB workable.
- Validation every epoch: mAP (BEV IoU, per class), AVE, occupancy IoU per channel, plus a fixed set of 16 frames rendered to `data/checkpoints/<run>/viz/`.
- Held-out: Town07 (all weathers). Report separately for day/night and rain.

## 3. Tracker (`nuway_perception/tracker.py`, Python but vectorized; port to C++ later if needed)

Per frame, detections `D` (with `vel`), tracks `T` (each: last box, vel, id, age, hits, misses, history ring buffer of 20 poses at 0.1 s in **map frame**):
1. Transform detections to map frame (using `/nuway/loc/pose` at the LiDAR stamp).
2. Predict each detection backwards: `p_det − v_det · Δt`; predict each track forwards: `p_trk + v_trk · Δt`. Use the average of both as the match position (symmetric).
3. Cost matrix: L2 distance, `+inf` if class mismatch or distance > `gate[class]` (car 3 m, ped 1 m, bicycle 1.5 m at Δt=0.1 s), plus `0.5·|log(l1/l2)| + 0.5·|Δyaw|` term.
4. Hungarian (`scipy.optimize.linear_sum_assignment`).
5. Matched: update box/vel from detection (no filtering), `hits += 1`, `misses = 0`, push pose to history. Unmatched track: `misses += 1`, propagate pose by its velocity; delete when `misses > 5` (0.5 s). Unmatched detection: new track (`hits = 1`).
6. Output agents with `hits ≥ 2` (suppresses one-frame false positives), transformed back to `base_link`, with `history` filled from the ring buffer (transformed to current base_link).
7. On `ResetEvent`: drop all tracks and reset the id counter.

`scipy` enters `ml/pyproject.toml` runtime dependencies here (`uv add scipy`).

Evaluation script `ml/scripts/eval_tracking.py`: run detector + tracker on held-out sequences; report ID switches, fragmentation, MOTA-lite, using GT ids.

## 4. Runtime nodes

### 4.1 LiDAR preprocessing: in the Python inference node

`bevfusion_node.py` subscribes `/carla/hero/lidar_top` directly and builds pillars with `torch` ops (`scatter_reduce`) on the GPU. This is the one sanctioned place where a point cloud crosses into Python (`00_overview.md` principle 6). Two facts to keep straight:
- `rclpy` in Humble does **not** support loaned (zero-copy) messages, so the `PointCloud2` is deserialized: ≈ 30k points × 16 bytes = 0.5 MB per sweep. Expected cost 1–3 ms; it is measured by the diag `preproc` breakdown and reported in the M3 report.
- If deserialization + pillarization exceeds **8 ms p50**, the escalation path is `lidar_preproc_node.cpp` (C++, LibTorch pillarization, tensor handed over through CUDA IPC). It is not built unless the threshold is crossed; the decision is recorded here.

The occupancy grid is likewise published from Python (it is this node's output); consumers are C++ and read it through the ordinary `OccupancyGridMC` topic.

### 4.2 `bevfusion_node.py`
- Subscribes LiDAR + 4 images (message_filters approximate sync, slop 0.03 s) + pose. Runs once per two ticks, triggered by the LiDAR message.
- Maintains the temporal queue and EMA memory (on GPU); both are cleared on `ResetEvent`, together with the tracker.
- Runs the model (torch.compile or TensorRT export via `nuway_ml/export/`; TensorRT is optional, target met with `torch.compile(mode="reduce-overhead")` + fp16 first).
- Decodes, tracks, publishes `AgentArray` (base_link, stamp = LiDAR stamp) and `OccupancyGridMC` (post-processed: `unknown = 1 − occupied − free` clamped; `occupied` thresholded softly).
- Publishes `NodeDiag` with breakdown: preproc, model, decode, track.

## 5. Integration & evaluation

- Profile `m3_learned_perception.yaml`: `use_gt.perception=false`, `use_gt.traffic_lights=true` (so `gt_traffic_light_node` runs while `bevfusion_node` replaces `gt_perception_node`), `use_gt.localization=true`.
- Run M1 protocol; compare with `compare_runs.py` against the M1 GT baseline run (visibility-on). Investigate any route where the score drops > 30%: replay MCAP in Foxglove with GT agents overlaid on perceived ones (`nuway_viz` has a "GT vs perceived" layout).
- Planner robustness pass (if needed): if the score drop is dominated by flicker (agents appearing/disappearing), raise `hits` threshold to 3 or extend `misses`; if by static-obstacle collisions, lower the safety layer's occupancy threshold; record changes in the M1 configs with a comment.

## 6. Task list

1. [ ] `targets.py` (heatmap/regression targets with don't-care mask), `losses.py`, `decode.py` + tests (encode→decode round trip within tolerance).
2. [ ] `lidar_branch.py` (pillars, scatter, backbone) + shape tests; `camera_branch.py` (LSS + depth supervision) + tests; `fusion.py`.
3. [ ] `temporal.py` + warp alignment test (synthetic feature with a "wall", ego motion, assert alignment).
4. [ ] `occupancy_head.py`, `centerpoint_head.py`, `bevfusion.py` (assembly, config-driven).
5. [ ] `metrics.py`: mAP (BEV IoU), AVE, occupancy IoU; test against toy cases.
6. [ ] `train.py` with staged schedule; run stage A; inspect; B; C.
7. [ ] `tracker.py` + `eval_tracking.py`.
8. [ ] `export/` (torch.compile config; optional TensorRT via `torch_tensorrt`), latency benchmark script.
9. [ ] `bevfusion_node.py`, launch wiring, profile YAML.
10. [ ] Closed-loop eval; write `data/eval_runs/m3_report.md` comparing to M1.
11. [ ] Foxglove layout "GT vs perceived".

## 7. Decisions log

- (2026-09-02) PointPillars over sparse voxels: no custom CUDA dependency; 0.25 m pillars are sufficient at CARLA scale.
- (2026-09-02) Temporal fusion by warp+concat (BEVDet4D style) with an EMA long-term memory; no GRU/attention.
- (2026-09-02) Tracking by velocity-projected Hungarian association (CenterPoint style). Histories are stored in map frame to survive ego motion.
- (2026-09-05) Traffic light perception moved out to its own milestone (M4): it shares no model, data, metric or training loop with BEVFusion, and its driving-score impact (red-light penalty 0.70) warrants a full design of its own.
- (2026-09-05) Point-cloud deserialization in Python is accepted as the one exception to principle 6 (`00_overview.md`), with a measured 8 ms escalation threshold, instead of a C++/CUDA-IPC preprocessing node from day one.

## 8. Open questions

- Whether camera branch is worth its ~10 ms at 704×256. Decide by ablation on night/rain: if LiDAR-only within 3 mAP, drop the camera branch here (M4 uses the cameras independently).
