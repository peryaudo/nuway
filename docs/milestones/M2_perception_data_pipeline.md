# M2 — Perception data pipeline

**Goal:** collect a labeled multi-sensor dataset from CARLA sufficient to train M3 and M4, with automatic labels that reflect what the sensors can actually see, and a ground-truth occupancy generator that is shared between the training labels and the runtime GT perception node.

**Completion criteria**
- [ ] ≥ 150k frames across Town01/02/03/04/05/06/07/10, ≥ 6 weather presets, day and night, traffic densities {low, medium, high}.
- [ ] Every frame has: 4 images, LiDAR sweep, ego pose, calibration, visible-filtered 3D boxes with velocities, 6-channel GT occupancy, traffic light crops/labels.
- [ ] `PerceptionDataset` loads a shard at ≥ 200 frames/s with 8 workers (no decoding bottleneck for training).
- [ ] Label sanity report: histogram of boxes per frame, class distribution, visibility rejection rate, per-town counts. Visual spot-check notebook renders 20 random frames with labels overlaid on camera and BEV.
- [ ] `gt_perception_node` now publishes the full 6-channel occupancy from the same generator, and M1's driving score is unchanged (±2) with it.

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

No ROS in the collector. It talks to CARLA directly; it is faster and avoids serializing images through DDS. It **must** use the same `sensor_rig.py` and `carla_conv` logic as the runtime to guarantee identical extrinsics and frame conventions (import from `nuway_carla_bridge` as a plain Python module; it has no ROS dependency in its geometry helpers).

## 2. Record schema (`nuway_ml/common/schema.py`)

```python
@dataclass
class FrameRecord:
    key: str                      # f"{town}_{run_id}_{frame_idx:06d}"
    timestamp: float              # sim seconds
    town: str
    weather: str
    ego_pose_map: np.ndarray      # [4,4] base_link -> map (ROS convention)
    ego_vel_body: np.ndarray      # [3]
    ego_yaw_rate: float
    ego_control: dict             # throttle, brake, steer
    calib: dict                   # per camera: K [3,3], T_cam_from_base [4,4]; lidar: T_lidar_from_base
    images: dict                  # cam name -> jpeg bytes (quality 92)
    lidar: np.ndarray             # [N,4] float16: x,y,z in lidar frame, tag (semantic id) as float
    agents: list[AgentLabel]
    occupancy: np.ndarray         # [6,200,200] float16, channels per 02_interfaces
    traffic_lights: list[TLLabel]
    prev_ego_pose_map: np.ndarray # [4,4] at t-0.1 (for temporal fusion training) — also lidar of previous K frames are found by key arithmetic within the same run

@dataclass
class AgentLabel:
    id: int
    class_id: int
    box_base: np.ndarray          # [7]: x,y,z,l,w,h,yaw in base_link, z = box center
    vel_base: np.ndarray          # [2]: vx, vy in base_link
    visible: bool
    n_lidar_pts: int
    cam_visible: list[str]        # cameras in which the box projects with >= min_px area and depth-visible
    history_base: np.ndarray      # [20,3] x,y,yaw at 0.1 s spacing in current base_link (NaN if unavailable)
    future_map: np.ndarray        # [80,3] at 0.1 s in map frame (filled by post-pass, NaN beyond run end) — used by M6/M7; cheap to store now

@dataclass
class TLLabel:
    id: int
    state: int
    stop_line_map: np.ndarray     # [3]
    crop_cam: str | None          # camera where the TL bulb projects
    crop_bbox: np.ndarray | None  # [4] xyxy pixels
```

Shard layout (WebDataset): `{key}.json` (everything scalar/small, lists), `{key}.cam_front.jpg` …, `{key}.lidar.npy`, `{key}.occ.npy`. Shard files: `data/shards/perception/{town}/{run_id}-{shard_idx:04d}.tar`. A `manifest.json` per run lists shards, frame count, weather, traffic config, seeds.

## 3. Labelers

### 3.1 Visibility filter (`nuway_ml/data/labeling/visibility.py`)

An actor is `visible` if **either**:
- semantic LiDAR returns ≥ `min_pts[class]` points with that actor's `object_idx` (car 5, pedestrian 3, bicycle 3, static 5), **or**
- in any camera, the actor's 3D box projects to ≥ `min_px_area` (400 px²) inside the image **and** the depth camera at the box's projected center is within `box_depth ± (box_diag/2 + 1 m)` (i.e. not occluded).

Label-only sensors: `sensor.lidar.ray_cast_semantic` co-located with `lidar_top` (identical attributes), `sensor.camera.depth` co-located with each RGB camera. These are never part of the runtime rig.

Invisible agents are kept in the record with `visible=false` so prediction training (M6/M7) can still use them as context if desired; perception training masks them out of all losses (not just positives — they are also excluded from the negative heatmap region by a "don't care" mask of radius 1.5 × box size).

### 3.2 Box & velocity labels

Box from `actor.bounding_box` (extent, local offset) + transform, converted to ROS convention and `base_link`. `z` is box center in base_link (ground at z≈0). Velocity from `actor.get_velocity()` rotated into base_link. Pedestrian boxes: CARLA walkers' bounding box is fine as is. Bicycles/motorcycles: the rider is a separate walker in some CARLA versions — merge by proximity (rider within 0.5 m of a 2-wheeler center → drop rider label).

Class ids per `02_interfaces.md`. Traffic-cone/barrier/props tagged `static_obstacle`.

### 3.3 GT occupancy generator (`nuway_ml/data/gt_occupancy.py`)

One function used by both the collector and (through a pybind or a small C++ port validated by parity test) the runtime `gt_perception_node`:

```python
def generate_gt_occupancy(spec: GridSpec,
                          semantic_lidar_pts: np.ndarray,   # [N,5] x,y,z,tag,object_idx in base_link
                          agents: list[AgentLabel],
                          lane_polygons_base: list[np.ndarray],
                          ego_footprint: np.ndarray) -> np.ndarray:  # [6,H,W]
```
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

After a run finishes, `postprocess_run.py` walks the run's frames in order and fills `history_base` and `future_map` for every agent id from the recorded per-frame poses (kept in a run-level `trajectories.parquet` sidecar written during collection: `[frame_idx, agent_id, x, y, yaw, vx, vy]` in map frame). This makes the perception shards immediately reusable for M6 without re-collection.

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

Throughput: rendering 4 RGB + 4 depth + 2 LiDAR at Low quality is ~12–18 ticks/s on the 3090 Ti → one 300 s run ≈ 6 min wall; 8 × 8 × 3 = 192 runs ≈ 20 h. Acceptable; run overnight in batches with `--resume`.

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

Augmentation (`augment.py`, perception part): random image scale ±10%, random BEV rotation ±15° and flip (applied consistently to LiDAR, boxes, occupancy), LiDAR point dropout 0–20%, per-image color jitter.

## 6. Runtime GT perception (`gt_perception_node`, full version)

Subscribes GT agents, semantic LiDAR (a label-only sensor is allowed in the GT profile), lane graph. Calls the C++ port of `generate_gt_occupancy` (parity test against Python on 50 stored frames: max abs diff < 1e-3 per cell). Publishes `/nuway/perception/occupancy` at 10 Hz and agents with `visible` filtering applied (so GT perception ≈ "perfect but physically plausible" perception). Config `gt_perception.apply_visibility: true|false` to toggle omniscient mode.

## 7. Task list

1. [ ] `schema.py` with validation (`FrameRecord.validate()` checks shapes/dtypes/NaN policy).
2. [ ] `webdataset_io.py`: `ShardWriter`, `ShardReader`, manifest handling, resume.
3. [ ] `sensor_rig.py` extension: label-only sensors; make geometry helpers ROS-free.
4. [ ] `visibility.py` + tests on synthetic projections.
5. [ ] `gt_occupancy.py` + tests; C++ port in `nuway_perception` + parity test.
6. [ ] TL labeler + tests.
7. [ ] `collect_perception.py` with `--config`, `--resume`, `--dry-run 50` (writes 50 frames and renders a debug PNG per frame).
8. [ ] `postprocess_run.py` (history/future fill), `trajectories.parquet` sidecar.
9. [ ] `perception_dataset.py` + `augment.py` + throughput test + spot-check notebook `ml/notebooks/inspect_perception.ipynb` (committed without outputs).
10. [ ] Run full collection; write label sanity report to `data/shards/perception/REPORT.md`.
11. [ ] Full `gt_perception_node`; rerun M1 eval; confirm score parity.

## 8. Decisions log

- (2026-09-02) Store `future_map` in perception records even though M2 doesn't use it: avoids re-collection for M6.
- (2026-09-02) Hero driven by Traffic Manager in M2 (viewpoint diversity, no expert needed yet). M6 switches to the expert and re-collects a planning-focused set without rendering.

## 9. Open questions

- Semantic LiDAR `object_idx` reliability for walkers in 0.9.16 — verify in dry run; if unreliable, fall back to box-containment of raw LiDAR points.
