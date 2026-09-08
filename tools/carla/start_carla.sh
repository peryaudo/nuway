#!/usr/bin/env bash
# tools/carla/start_carla.sh [--profile configs/profiles/<name>.yaml] [--port N] [--wait] [--foreground]
#
# The one place the CARLA server flags live (docs/01_directory_structure.md).
# Runs  $CARLA_ROOT/CarlaUE4.sh -RenderOffScreen --ros2 -carla-rpc-port=<port> -quality-level=<quality>
# reading carla.port and carla.quality from the profile (default: 2000 / Epic).
# Refuses quality Low: it segfaults load_world() on CARLA 0.9.16
# (docs/00_overview.md §4). CARLA_ROOT defaults to ~/carla.
#
# By default the server is started detached (log under data/logs/) and the
# script returns; --wait blocks until a client can connect (up to 180 s);
# --foreground runs it in this shell instead.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$root"

profile=""
port=""
wait_ready=0
foreground=0
while [ $# -gt 0 ]; do
  case "$1" in
    --profile) profile="$2"; shift 2 ;;
    --port) port="$2"; shift 2 ;;
    --wait) wait_ready=1; shift ;;
    --foreground) foreground=1; shift ;;
    -h|--help) sed -n '2,13p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "start_carla.sh: unknown argument $1" >&2; exit 2 ;;
  esac
done

quality="Epic"
if [ -n "$profile" ]; then
  [ -f "$profile" ] || { echo "start_carla.sh: no such profile $profile" >&2; exit 2; }
  read -r prof_port prof_quality < <(uv run --frozen python - "$profile" <<'PY'
import sys, yaml
with open(sys.argv[1]) as f:
    cfg = yaml.safe_load(f) or {}
carla = cfg.get("carla", {})
print(carla.get("port", 2000), carla.get("quality", "Epic"))
PY
)
  [ -n "$port" ] || port="$prof_port"
  quality="$prof_quality"
fi
[ -n "$port" ] || port=2000

if [ "${quality,,}" = "low" ]; then
  echo "start_carla.sh: refusing -quality-level=Low: it segfaults load_world() (docs/00_overview.md §4)" >&2
  exit 2
fi

carla_root="${CARLA_ROOT:-$HOME/carla}"
[ -x "$carla_root/CarlaUE4.sh" ] || { echo "start_carla.sh: $carla_root/CarlaUE4.sh not found (set CARLA_ROOT)" >&2; exit 2; }

cmd=("$carla_root/CarlaUE4.sh" -RenderOffScreen --ros2 "-carla-rpc-port=$port" "-quality-level=$quality")
echo "start_carla.sh: ${cmd[*]}"
if [ "$foreground" -eq 1 ]; then
  exec "${cmd[@]}"
fi

mkdir -p data/logs
log="data/logs/carla_${port}.log"
nohup "${cmd[@]}" > "$log" 2>&1 &
echo "start_carla.sh: pid $! log $log"

if [ "$wait_ready" -eq 1 ]; then
  uv run --frozen python - "$port" <<'PY'
import sys, time
try:
    import carla
except ImportError:
    print("start_carla.sh: --wait needs the carla wheel (uv sync --group carla)", file=sys.stderr)
    sys.exit(1)
port = int(sys.argv[1])
deadline = time.time() + 180.0
while time.time() < deadline:
    try:
        client = carla.Client("127.0.0.1", port)
        client.set_timeout(5.0)
        version = client.get_server_version()
        print(f"start_carla.sh: server {version} ready on port {port}")
        sys.exit(0)
    except RuntimeError:
        time.sleep(2.0)
print("start_carla.sh: server did not come up within 180 s", file=sys.stderr)
sys.exit(1)
PY
fi
