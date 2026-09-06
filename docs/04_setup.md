# Machine setup

How to take a brand-new machine to a working `nuway` development environment.

**Status.** Sections 0–3 and 5 were executed end-to-end on the reference dev box on 2026-09-06;
the versions and outputs shown are what that run actually produced. Sections 4 and 6 describe
the intended flow but **cannot be run yet** — `setup_env.sh`, the `ros2_ws/` packages and
`tools/carla/check_native_ros2.py` are M0 task 1 and task 5 deliverables and the repository is
still docs-only. The facts those sections rely on *were* verified directly (the CARLA client
wheel imports on system Python 3.12, torch cu126 reaches the GPU, and the topic list in §6 is
a real capture from a live server); only the wrapper scripts are missing. Update this note when
M0 lands them.

The authority on *what* the environment is remains `00_overview.md` §4 and
`03_style_and_conventions.md` §6 — this document is only the procedure. If the two disagree,
those two win and this file is stale.

**Target:** Ubuntu 24.04 LTS (noble), NVIDIA GPU, ≥ 60 GB free disk, ≥ 12 physical cores.

---

## 0. Check the machine

```bash
lsb_release -ds                 # must be Ubuntu 24.04.x LTS
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv
nproc && df -h /home | tail -1
```

Ubuntu 24.04 is not optional. The whole toolchain (ROS 2 Jazzy, system Python 3.12, GCC 13) is
the distro's, and ROS Humble — which earlier revisions of these docs targeted — is not packaged
for noble at all. Reference box: RTX 3090 Ti 24 GB, driver 595.84, Ryzen 9 5950X (16c/32t).

Disk: CARLA alone is 8.4 GB downloaded + 17 GB extracted, and ROS 2 desktop adds ~3 GB.

---

## 1. ROS 2 Jazzy

```bash
sudo apt-get update && sudo apt-get install -y curl gnupg lsb-release
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo tee /etc/apt/keyrings/ros-archive-keyring.gpg >/dev/null
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu noble main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt-get update

sudo apt-get install -y \
  ros-jazzy-desktop \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-foxglove-bridge \
  python3-colcon-common-extensions python3-rosdep
```

Verify — `rclpy` must resolve to the **system** Python 3.12, since that ABI match is what the
whole `uv` setup in §4 is built around:

```bash
source /opt/ros/jazzy/setup.bash
python3 -c "import rclpy; print(rclpy.__file__)"
# /opt/ros/jazzy/lib/python3.12/site-packages/rclpy/__init__.py
```

Initialise rosdep once:

```bash
sudo rosdep init && rosdep update
```

## 2. Build toolchain and C++ dependencies

```bash
sudo apt-get install -y \
  build-essential gcc-13 g++-13 clang-18 clang-tidy-18 \
  cmake ninja-build ccache mold git \
  libeigen3-dev libnanoflann-dev ros-jazzy-gtsam ros-jazzy-osqp-vendor
```

On Jazzy/noble the math stack comes from apt — GTSAM 4.2.0, OSQP 0.2.0 and nanoflann 1.5.4 are
all packaged, so `osqp-eigen` is the only vendored dependency in the workspace
(`03_style_and_conventions.md` §6.2). Confirm CMake can actually see them — `cmake --find-package`
is unreliable here, so use a real configure step:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p /tmp/findtest && cd /tmp/findtest
cat > CMakeLists.txt <<'EOF'
cmake_minimum_required(VERSION 3.28)
project(findtest CXX)
find_package(GTSAM REQUIRED)
find_package(osqp REQUIRED)
find_package(nanoflann REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
message(STATUS "GTSAM ${GTSAM_VERSION} / nanoflann ${nanoflann_VERSION} / Eigen ${Eigen3_VERSION} / PCL ${PCL_VERSION}")
EOF
cmake -S . -B b -G Ninja | grep -E "GTSAM|Could NOT"
# -- GTSAM 4.2.0 / nanoflann 1.5.4 / Eigen 3.4.0 / PCL 1.14.0
```

`Could NOT find Pcap` / `Could NOT find MPI` in PCL's output are optional components and
harmless.

Expected apt versions: cmake 3.28.3, ccache 4.9.1, mold 2.30.0, gcc-13 13.3.0, clang 18.1.3.

## 3. CARLA 0.9.16

```bash
mkdir -p ~/carla && cd ~/carla
curl -L -o CARLA_0.9.16.tar.gz https://downloads.carlasim.com/Linux/CARLA_0.9.16.tar.gz
tar xzf CARLA_0.9.16.tar.gz
```

8.35 GB (`downloads.carlasim.com`; the `carla-releases.b-cdn.net` host that `tiny.carla.org`
redirects to returns 403). Launch:

```bash
cd ~/carla && ./CarlaUE4.sh -RenderOffScreen --ros2 -carla-rpc-port=2000
```

> **Never pass `-quality-level=Low`.** It segfaults the server on the first `load_world()`
> (null `VertexDeclaration` in UE4.26's Vulkan RHI). Since CARLA 0.9.16 also ignores the map
> name on the command line — it always boots `Town10HD_Opt` — `load_world` is the only way to
> pick a town, so low quality cannot be worked around. Full analysis in `00_overview.md` §4 and
> the M0 decisions log.

Smoke test from a second shell:

```bash
# once §4 has run; before that, any venv with `pip install carla==0.9.16` on Python 3.12 works
cd ~/nuway && source setup_env.sh
python3 -c "
import carla
c = carla.Client('127.0.0.1', 2000); c.set_timeout(120.0)
print(c.get_client_version(), c.get_server_version())
w = c.load_world('Town03')
print('loaded', w.get_map().name)
"
```

Both versions must print `0.9.16`, and `load_world` must return rather than time out. A timeout
here means the server crashed — check that `-quality-level=Low` is absent.

## 4. Repository and Python environment

> Blocked on M0 task 1: `setup_env.sh`, the root `pyproject.toml`/`uv.lock` and `ros2_ws/` do
> not exist yet. The steps below are the target flow.

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh      # uv, if not present
git clone https://github.com/peryaudo/nuway.git ~/nuway
cd ~/nuway && source setup_env.sh
```

`setup_env.sh` is the single definition of the environment (`03_style_and_conventions.md` §6.3):
it checks prerequisites, sources ROS, creates `.venv` from the **system** interpreter with
system site-packages visible (`uv venv --python /usr/bin/python3.12 --system-site-packages`),
runs `uv sync`, and sources the colcon workspace. Never create the venv by hand — a
uv-downloaded interpreter breaks the ABI match with `rclpy`.

```bash
uv sync --group carla --group train --group viz     # full workstation
colcon build
```

PyTorch comes from the pinned **cu126** index. Verify CUDA reaches the GPU:

```bash
python3 -c "
import torch
print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_device_name(0))
"
# 2.14.0+cu126 True NVIDIA GeForce RTX 3090 Ti
```

The 595 driver reports CUDA 13.2 and runs cu126 wheels fine; cu126 is chosen over cu130 for
third-party CUDA extension availability (`03_style_and_conventions.md` §6.1).

## 5. Middleware

The stack runs on CycloneDDS. Add to your shell (`setup_env.sh` exports it too):

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

CycloneDDS interoperates with CARLA's embedded FastDDS for both discovery and data — verified.
It logs one benign warning per CARLA topic, which can be ignored:

```
[WARN] [rmw_cyclonedds_cpp]: Failed to parse type hash for topic 'rt/...' from USER_DATA '(null)'
```

CARLA does not advertise the ROS 2 type hashes that Jazzy expects. It is a warning only; data
flows normally.

**Shared memory (optional, off by default).** Jazzy's CycloneDDS 0.10.5 is built with iceoryx
support. To enable it, write a config and run the RouDi daemon *before* any node:

```bash
cat > ~/.cyclonedds.xml <<'XML'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any"><SharedMemory><Enable>true</Enable></SharedMemory></Domain>
</CycloneDDS>
XML
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml
/opt/ros/jazzy/bin/iox-roudi &      # must be running, or nodes fall back to the network path
```

Zero-copy only engages for fixed-size messages, so it does nothing for `PointCloud2` or `Image`.
Treat it as an optimisation to measure, not a default.

## 6. End-to-end check

> Blocked on M0 task 5: `tools/carla/check_native_ros2.py` does not exist yet. The `ros2 topic
> list` output below is a real capture and is valid to check against today, by spawning a rig
> manually through the CARLA Python API.

With CARLA running (§3) and the environment sourced (§4):

```bash
python3 tools/carla/check_native_ros2.py
```

This is M0 task 5. It spawns a hero and rig, and asserts the properties recorded in
`02_interfaces.md` §3.1. What a correct run looks like:

```bash
ros2 topic list
# /carla/hero/lidar_top/point_cloud
# /carla/hero/cam_front/image
# /carla/hero/cam_front/camera_info
# /carla/hero/imu
# /carla/hero/gnss
# /carla/hero/vehicle_control_cmd
# /clock
# /tf
```

If you instead see `/carla/actor109/actor110/point_cloud`, the sensor blueprints are missing
their **`ros_name`** attribute — that is the attribute CARLA names topics from, *not*
`role_name`.

---

## Troubleshooting

| Symptom | Cause |
|---|---|
| `load_world()` times out, server gone, `Signal 11` in the log | `-quality-level=Low` was passed. Remove it (§3). |
| Topics named `/carla/actorNNN/actorMMM/...` | `ros_name` not set on the blueprint before spawn. |
| `AttributeError: 'Vehicle' object has no attribute 'enable_for_ros'` | `enable_for_ros()` exists on sensors only. |
| Camera intrinsics absurd (`fx ≈ -22973`) | Known CARLA 0.9.16 bug. Use our own `CameraInfo`; never subscribe to CARLA's. |
| `Spawn failed because of collision at spawn position` | A previous run's actors survive. Reconnect and destroy them, or restart the server. |
| `rclpy` not importable in `.venv` | Venv built from a uv-downloaded interpreter. Delete `.venv` and re-run `setup_env.sh`. |
| `libiceoryx_binding_c.so => not found` in `ldd` | ROS not sourced. `source /opt/ros/jazzy/setup.bash`. |
| CARLA server survives `pkill -f CarlaUE4` | The pattern matches your own shell. Match on `/proc/<pid>/exe` instead. |
