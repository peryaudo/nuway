# M2 — Perception data pipeline

**Goal:** collect a labeled multi-sensor dataset from CARLA sufficient to train M3 and M4, with automatic labels that reflect what the sensors can actually see, and a ground-truth occupancy generator that is shared between the training labels and the runtime GT perception node.

**Completion criteria**
- [ ] ≥ 150k frames across Town01/02/03/04/05/06/07/10, ≥ 6 weather presets, day and night, traffic densities {low, medium, high}.
- [ ] Every frame has: 4 images, LiDAR sweep, ego pose, calibration, visible-filtered 3D boxes with velocities, 6-channel GT occupancy, traffic light crops/labels.
- [ ] `PerceptionDataset` loads a shard at ≥ 200 frames/s with 8 workers (no decoding bottleneck for training).
- [ ] Label sanity report: histogram of boxes per frame, class distribution, visibility rejection rate, per-town counts. Visual spot-check notebook renders 20 random frames with labels overlaid on camera and BEV.
- [ ] `gt_perception_node.py` now publishes the full 6-channel occupancy from the same generator the collector uses, and with `gt_perception.apply_visibility: false` M1's driving score is unchanged (±2). The one legitimate source of change is that the safety layer now sees a real `occupied` channel (M0 published zeros); if the score moves, the report attributes the difference to static-obstacle interventions before anything else is tuned.
- [ ] `generate_gt_occupancy` sustains the closed-loop budget: p99 < 100 ms per call at 10 Hz on the dev workstation, measured from `/nuway/diag/gt_perception_node`. Above that, vectorize the DDA before considering a C++ port (`00_overview.md` §2.6 would have to change first).
- [ ] **The baseline is re-measured**: the M1 protocol is run with `gt_perception.apply_visibility: true` and the result is recorded in the M1 Decisions log as *the* M1 GT baseline that M3, M4 and M5 compare against (`00_overview.md` §3).

---

## 1. Design

Two collection modes exist in this repo; M2 builds the rendered one. M6 builds the render-free one and reuses the same schema for everything except sensors.

```
collect_perception.py (CARLA client, sync mode, no ROS)
   ├─ spawn hero (autopilot via Traffic Manager for driving diversity; OR replay expert routes from M6 later)
   ├─ spawn sensor rig (same rig_dev.json as runtime) + semantic LiDAR + depth cams (label-only sensors)
   ├─ per tick: gather sensor data + GT actor states + TL states
   ├─ every N ticks (N=2 → 10 Hz): build a FrameRecord, run labelers, write to shard
   └─ ShardWriter (WebDataset tar, 1000 frames/shard)
```

No ROS in the collector. It talks to CARLA directly; it is faster and avoids serializing images through DDS. It **must** use the same rig loader and the same conversion code as the runtime to guarantee identical extrinsics and frame conventions: the rig JSON parsing, the CARLA→`base_link` extrinsics and the intrinsics `K` live in `nuway_ml/common/rig.py` (on top of `carla_conv.py`), which both `nuway_carla_bridge/sensor_rig.py` and the collector import (`01_directory_structure.md` rules). The collector imports nothing from `ros2_ws`.

## 2. Record schema (`nuway_ml/common/schema.py`)

All records are frozen dataclasses (`03_style_and_conventions.md` §9.3); no bare `dict` fields.

```python
@dataclass(frozen=True, slots=True)
class EgoControl:
    throttle: float
    brake: float
    steer: float                  # CARLA [-1, 1]

@dataclass(frozen=True, slots=True)
class CameraCalib:
    name: str
    K: np.ndarray                 # [3,3], from rig.py — the same function that fills /nuway/sensors/<cam>/camera_info
    T_cam_from_base: np.ndarray   # [4,4], from rig.py — the same extrinsic that sensor_rig.py publishes on /tf_static
    width: int
    height: int

@dataclass(frozen=True, slots=True)
class FrameRecord:
    schema_version: int           # SCHEMA_VERSION in schema.py; bumped on every field change, checked by the loaders,
                                  #   also written to every manifest.json. M6 and M8 extend the schema in place.
    key: str                      # f"{town}_{run_id}_{frame_idx:06d}"
    timestamp: float              # sim seconds
    town: str
    weather: str
    ego_pose_map: np.ndarray      # [4,4] base_link -> map (ROS convention)
    ego_vel_body: np.ndarray      # [3]
    ego_yaw_rate: float
    ego_control: EgoControl
    cameras: tuple[CameraCalib, ...]
    T_lidar_from_base: np.ndarray # [4,4]
    images: dict[str, bytes]      # cam name -> jpeg bytes (quality 92); the one dict, keyed by camera name only
    lidar: np.ndarray             # [N,4] float32: x,y,z,intensity in lidar frame — exactly what the runtime
                                  # sensor.lidar.ray_cast publishes; this is the model input and must not
                                  # contain label information
    lidar_semantic: np.ndarray    # [N,5] float32: x,y,z,object_idx,tag — the label-only semantic LiDAR in the LiDAR
                                  # frame, in the column order of the native ROS topic minus cos_angle (02 §3.1);
                                  # gt_perception_node builds the identical array from the topic. Used by the labelers
                                  # and kept for re-labeling, never fed to a model
    agents: list[AgentLabel]
    occupancy: np.ndarray         # [6,200,200] float16, channels per 02_interfaces
    traffic_lights: list[TLLabel]
    prev_ego_pose_map: np.ndarray # [4,4] at t-0.1 (for temporal fusion training) — also lidar of previous K frames are found by key arithmetic within the same run

@dataclass(frozen=True, slots=True)
class AgentLabel:
    id: int
    class_id: int
    box_base: np.ndarray          # [7]: x,y,z,l,w,h,yaw in base_link, z = box center
    vel_base: np.ndarray          # [2]: vx, vy in base_link
    visible: bool
    n_lidar_pts: int
    cam_visible: list[str]        # cameras in which the box projects with >= min_px area and depth-visible
    history_base: np.ndarray      # [20,3] x,y,yaw at 0.1 s spacing in current base_link, history[0] = 0.1 s ago (NaN if unavailable)
    future_map: np.ndarray        # [80,3] at 0.1 s in map frame, future[0] = +0.1 s: t = 0 is the box itself, so a
                                  # future is 80 points where a Trajectory (t = 0 … 8 s) is 81 (02 §4). Filled by the
                                  # post-pass, NaN beyond run end — used by M6/M7; cheap to store now

@dataclass(frozen=True, slots=True)
class TLLabel:
    id: int
    state: int
    stop_line_map: np.ndarray     # [3]
    crop_cam: str | None          # camera where the TL bulb projects
    crop_bbox: np.ndarray | None  # [4] xyxy pixels
```

Shard layout (WebDataset): `{key}.json` (everything scalar/small, lists), `{key}.cam_front.jpg` …, `{key}.lidar.npy`, `{key}.lidar_sem.npy`, `{key}.occ.npy`. Shard files: `data/shards/perception/{town}/{run_id}-{shard_idx:04d}.tar`. A `manifest.json` per run lists shards, frame count, weather, traffic config, seeds. LiDAR coordinates are float32 on purpose: float16 has a spacing of about 6 cm beyond 64 m, which is a quarter of a pillar. Budget ≈ 1.7 MB per frame (two clouds ≈ 1.1 MB: 30k × 4 × 4 B runtime + 30k × 5 × 4 B semantic; occupancy 0.48 MB; images 0.15 MB), so the ≥ 150k-frame target is ≈ 260 GB and the full protocol below ≈ 1 TB; check disk before the full run.

## 3. Labelers

### 3.1 Visibility filter (`nuway_ml/data/labeling/visibility.py`)

An actor is `visible` if **either**:
- semantic LiDAR returns ≥ `min_pts[class]` points with that actor's `object_idx` (car 5, truck 5, pedestrian 3, bicycle 3, motorcycle 3, static 5), **or**
- in any camera, the actor's 3D box projects to ≥ `min_px_area` (400 px²) inside the image **and** the depth camera at the box's projected center is within `box_depth ± (box_diag/2 + 1 m)` (i.e. not occluded).

Label-only sensors: `sensor.lidar.ray_cast_semantic` co-located with `lidar_top` (identical attributes; the `lidar_top_semantic` entry with `label_only: true` in `rig_dev.json`), `sensor.camera.depth` co-located with each RGB camera (collector only, not in the rig JSON). These are never subscribed by a learned node; the semantic LiDAR is spawned at runtime only in GT-perception and mapping profiles (`02_interfaces.md` §3.1).

Invisible agents are kept in the record with `visible=false` for label completeness and re-labeling; the M7 tokenizer drops them from the model input, because at serving time an agent that perception did not detect does not exist, and feeding it in training would be a training/serving skew (M7 §2). Perception training masks them out of all losses (not just positives — they are also excluded from the negative heatmap region by a "don't care" mask of radius 1.5 × box size).

### 3.2 Box & velocity labels

Box from `actor.bounding_box` (extent, local offset) + transform, converted to ROS convention and `base_link`. `z` is box center in base_link (ground at z≈0). Velocity from `actor.get_velocity()` rotated into base_link. Pedestrian boxes: CARLA walkers' bounding box is fine as is. Bicycles/motorcycles: the rider is a separate walker in some CARLA versions — merge by proximity (rider within 0.5 m of a 2-wheeler center → drop rider label).

Class ids per `02_interfaces.md`. Traffic-cone/barrier/props tagged `static_obstacle`.

### 3.3 GT occupancy generator (`nuway_ml/data/gt_occupancy.py`)

**One implementation, two callers**: the offline collector and the runtime `gt_perception_node.py` both import this function directly. There is no C++ port and no parity test — that duplication is exactly what made the GT perception twin Python (`00_overview.md` §2.6, `M0_bringup.md` §5). Because it is single-sourced, the occupancy the model trains on and the occupancy the GT stack drives on cannot drift:

```python
def generate_gt_occupancy(spec: GridSpec,
                          semantic_lidar_pts: np.ndarray,   # [N,5] x,y,z,object_idx,tag in base_link (schema §2 order;
                                                            #   callers apply T_base_from_lidar from rig.py first)
                          lidar_origin_base: np.ndarray,    # [3] the LiDAR origin in base_link: the start of every free-space ray
                          agents: list[AgentLabel],
                          lane_polygons_base: list[np.ndarray],
                          ego_footprint: np.ndarray) -> np.ndarray:  # [6,H,W]
```
Because a runtime node now imports it, `gt_occupancy.py` must stay **import-light**: `numpy` and the numpy half of `nuway_ml.common` only. No `torch`, no `webdataset`, no CARLA client, no `hydra`/`wandb`, no `nuway_ml.viz` — the same rule those already live under (`03_style_and_conventions.md` §6.1). The no-torch part matters most: it is what keeps the GT-only profiles runnable in a plain `uv sync` environment, where torch is not installed because it is an extra of `nuway-ml` (`03_style_and_conventions.md` §6.1, `M0_bringup.md` §2.9). Enforced by an `import_light`-marked test that CI runs in exactly that environment.

Channels:
- `occupied`: cells containing ≥ 1 semantic-LiDAR return with tag in {Building, Fence, Wall, Pole, Static, Dynamic-not-agent, Vegetation (z>0.3), TrafficSign, GuardRail, Other} **or** inside a `static_obstacle` agent box. Rasterize by point binning; then apply 3×3 max to close gaps. Exclude points with z > 3.5 m (overhangs).
- `free`: cells traversed by the ray from the LiDAR origin to each return (2-D DDA on the BEV grid, only for returns with tag Road/Sidewalk/Ground and z < 0.5 m), minus `occupied`. Also everything inside the ego footprint.
- `unknown`: `1 − max(occupied, free)`.
- `drivable`: rasterized lane polygons of type driving/bidirectional (from `LaneGraph`) transformed into base_link. Soft-edge with a 1-cell linear ramp.
- `height_max`: per-cell max z of occupied points, 0 elsewhere.
- `dynamic`: cells inside boxes of visible agents with class ∈ {car, truck, bicycle, motorcycle, pedestrian}, regardless of speed. Cells inside dynamic agent boxes are **removed from `occupied`** (so `occupied` is static-only), per the planner interface convention.

Test: synthetic scene with a wall, a car, and a lane polygon → expected channels checked cell-wise.

### 3.4 Traffic light labels

For every traffic light whose stop line is within 60 m ahead on lanes reachable from ego's lane: project the light-bulb location (CARLA `get_light_boxes()`) into cameras; take the camera with the largest projected area; store crop bbox (padded 25%) and state. If no camera sees it, `crop_cam=None`.

### 3.5 History/future post-pass

The collector writes each run to a *staging* directory (`data/raw/perception/{town}/{run_id}/`: per-frame `.npz`/`.jpg` plus the run-level `trajectories.parquet` sidecar `[frame_idx, agent_id, x, y, yaw, vx, vy]` in map frame, written with `pyarrow`, a base dependency of `nuway-ml`). After the run finishes, `postprocess_run.py` walks the frames in order, fills `history_base` and `future_map` for every agent id from the sidecar (records are frozen, so it builds new ones with `dataclasses.replace`), validates every record, and only then writes the final WebDataset shards; the staging directory is deleted on success. Shards are therefore never rewritten, and every shard on disk is complete. This makes the perception shards immediately reusable for M6 without re-collection.

## 4. Collection protocol

`configs/collect/perception_v1.yaml`:
```yaml
towns: [Town01, Town02, Town03, Town04, Town05, Town06, Town07, Town10HD]
weathers: [ClearNoon, CloudyNoon, WetNoon, HardRainNoon, ClearSunset, WetSunset, ClearNight, HardRainNight]
traffic: [{vehicles: 20, walkers: 10}, {vehicles: 60, walkers: 30}, {vehicles: 120, walkers: 60}]
runs_per_combo: 1
run_length_s: 300
frame_stride: 2        # 10 Hz from 20 Hz sim
hero_driver: traffic_manager      # replaced by expert in M6
seed_base: 1000
```
Hero driven by Traffic Manager autopilot with randomized `distance_to_leading_vehicle` and `vehicle_percentage_speed_difference` per run, so viewpoints vary. Ego collisions do not stop a run (they're fine for perception data; they're flagged in the manifest).

Throughput: rendering runs at default (Epic) quality — Low segfaults `load_world` (`00_overview.md` §4). With 4 RGB + 4 depth + 2 LiDAR expect roughly 8–20 ticks/s on the 3090 Ti (the `00_overview.md` §4 table measured 15–41 FPS at Epic with a lighter rig; the four depth cameras add load) → one 300 s run ≈ 5–13 min wall; 8 × 8 × 3 = 192 runs ≈ 25–40 h. Measure on the first batch and record the number here; run overnight in batches with `--resume`.

## 5. Dataset loader (`nuway_ml/data/perception_dataset.py`)

`PerceptionDataset(shards, K_prev=4)` yields:
```python
{
 'images': uint8 [4,3,256,704], 'K': [4,3,3], 'T_cam_from_base': [4,4,4],
 'lidar': list of K_prev+1 tensors [N_i,4] (current first), each in the base_link of ITS OWN frame,
 'T_prev_from_cur': [K_prev,4,4],          # ego motion for temporal warp
 'boxes': [M,7], 'vels': [M,2], 'classes': [M], 'dontcare_mask': [200,200],
 'occupancy': [6,200,200],
 'tl': {'crops': [n,3,64,64], 'states': [n]},
 'meta': {...}
}
```
Previous-frame LiDAR is resolved by key arithmetic inside the same run (frame_idx − 1·stride ...). Shards are written so a run's frames are contiguous; the loader keeps an LRU cache of the last 8 decoded frames per worker. If a previous frame does not exist (run start), it is replaced by the current one with identity motion and a `valid_prev` flag.

Augmentation (`augment.py`, perception part): random image scale ±10%, random BEV rotation ±15° and flip (applied consistently to LiDAR, boxes, occupancy), LiDAR point dropout 0–20%, per-image color jitter, and **camera dropout**: with probability 0.05 per camera the image is replaced by zeros and its entry in the `cam_valid` mask is cleared, which is exactly what `perception_node` does when the native topic loses a frame (`02_interfaces.md` §3.1), so masked inference is in-distribution.

## 6. Runtime GT perception (`gt_perception_node.py`, full version)

Subscribes GT agents, `/carla/hero/lidar_top_semantic/point_cloud` (the `label_only` rig entry, spawned because the profile has `use_gt.perception: true` and `gt_perception.source: sensor`; the six-field cloud is turned into the schema's `[N,5]` `x,y,z,object_idx,tag` array by `nuway_ml/common/rig.py`, dropping `cos_angle`, and transformed to `base_link` with the same rig extrinsics that give the LiDAR origin), lane graph. Calls `generate_gt_occupancy` (§3.3) directly — same function object the collector calls, no port, no bridge. Runs on even ticks once the agents and the semantic cloud of that tick have both arrived (`02_interfaces.md` §2 barrier). Publishes `/nuway/perception/occupancy` and agents with `visible` filtering applied (so GT perception ≈ "perfect but physically plausible" perception). Profile key `gt_perception.apply_visibility: true|false` toggles omniscient mode (`02_interfaces.md` §5). M6 adds a second key, `gt_perception.source: sensor | geometry` — `geometry` replaces the semantic-LiDAR inputs with the M6 static map + raycast (M6 §3.1) so the node runs without a sensor, for `m6_gt_planning`, the M8 shadow expert and cheap DAgger; `sensor`, specified here, is the default. With an invalid pose the node publishes the no-input pair (empty agents, all-`unknown` grid, `02_interfaces.md` §2). Traffic lights are not this node's business (`gt_traffic_light_node`, M0).

## 7. Task list

1. [ ] `schema.py` with validation (`FrameRecord.validate()` checks shapes/dtypes/NaN policy).
2. [ ] `webdataset_io.py`: `ShardWriter`, `ShardReader`, manifest handling, resume.
3. [ ] `sensor_rig.py` extension: `label_only` entries and the profile rule for spawning them; the collector's depth cameras.
4. [ ] `visibility.py` + tests on synthetic projections.
5. [ ] `gt_occupancy.py` + tests. No C++ port: `gt_perception_node.py` imports it.
6. [ ] TL labeler + tests.
7. [ ] `collect_perception.py` with `--config`, `--resume`, `--dry-run 50` (writes 50 frames and renders a debug PNG per frame).
8. [ ] `postprocess_run.py` (history/future fill), `trajectories.parquet` sidecar.
9. [ ] `perception_dataset.py` + `augment.py` + throughput test + spot-check notebook `ml/notebooks/inspect_perception.ipynb` (committed without outputs).
10. [ ] Run full collection; write label sanity report to `data/shards/perception/REPORT.md`.
11. [ ] Full `gt_perception_node.py` (+ diag timing check per the completion criteria); rerun M1 eval with `apply_visibility: false` (parity ±2) and with `apply_visibility: true` (record as the M1 GT baseline in the M1 Decisions log).

## 8. Decisions log

- (2026-09-02) Store `future_map` in perception records even though M2 doesn't use it: avoids re-collection for M6.
- (2026-09-02) Hero driven by Traffic Manager in M2 (viewpoint diversity, no expert needed yet). M6 switches to the expert and re-collects a planning-focused set without rendering.
- (2026-09-05) The stored `lidar` array is the runtime sensor's output (x, y, z, intensity, float32); the semantic cloud is stored separately. Earlier drafts stored the semantic tag as the fourth channel, which would have trained the model on a channel it never sees at runtime.
- (2026-09-06) **No C++ port of `generate_gt_occupancy`.** The earlier plan had `gt_occupancy.py` as reference and `gt_occupancy.cc` as a runtime port held together by a 50-frame parity test. Dropped: point binning, a 3×3 close, 2-D DDA ray casting and soft-edged lane rasterization is a lot of code to maintain twice for one consumer, and a parity test is an admission that the two *can* drift. `gt_perception_node.py` imports the reference (`M0_bringup.md` §5). The Python cost is bounded — collection already runs this function 150k times offline, and closed loop it is ~1.9k calls per route under lockstep, which cannot change results.

## 9. Open questions

- Semantic LiDAR `object_idx` reliability for walkers in 0.9.16 — verify in dry run; if unreliable, fall back to box-containment of raw LiDAR points.
