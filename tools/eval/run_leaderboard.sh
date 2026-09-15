#!/usr/bin/env bash
# Run the stack under the official Leaderboard evaluator, one stack per route (M1 §3.12).
#
#   tools/eval/run_leaderboard.sh --routes tools/eval/routes/dev_town03.xml [--run-id ID]
#       [--route-ids dev03_00 ...] [--host localhost] [--port 2000] [--tm-port 8000]
#
# For every <route> of the file: a single-route XML and a copy of configs/profiles/leaderboard.yaml
# with carla.town set are written under data/eval_runs/<run_id>/leaderboard/, a fresh stack is
# launched on that profile, leaderboard_evaluator.py drives the route with leaderboard_agent.py on
# the MAP track, its JSON checkpoint and the agent's sidecar are collected, and the stack is
# torn down. Finally nuway_eval.leaderboard_results merges the JSONs into
# data/eval_runs/<run_id>/leaderboard_results.csv next to our own results.csv (if any).
# Needs: `source setup_env.sh`, a CARLA server (tools/carla/start_carla.sh), external/ from
# tools/eval/setup_leaderboard.sh, `uv sync --group carla --group leaderboard` and CARLA_ROOT
# (default ~/carla) for the PythonAPI `agents` package.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

ROUTES=""; RUN_ID="lb_$(date +%Y%m%d_%H%M%S)"; HOST=localhost; PORT=2000; TM_PORT=8000
ROUTE_IDS=()
STACK_WAIT_S=${STACK_WAIT_S:-30}
EVAL_TIMEOUT_S=${EVAL_TIMEOUT_S:-120}
while [ $# -gt 0 ]; do
  case "$1" in
    --routes) ROUTES=$2; shift 2 ;;
    --run-id) RUN_ID=$2; shift 2 ;;
    --host) HOST=$2; shift 2 ;;
    --port) PORT=$2; shift 2 ;;
    --tm-port) TM_PORT=$2; shift 2 ;;
    --route-ids) shift; while [ $# -gt 0 ] && [[ "$1" != --* ]]; do ROUTE_IDS+=("$1"); shift; done ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[ -n "$ROUTES" ] || { echo "usage: $0 --routes <xml> [--run-id ID] [--route-ids ...]" >&2; exit 2; }
[ -d external/leaderboard ] && [ -d external/scenario_runner ] || { echo "run tools/eval/setup_leaderboard.sh first" >&2; exit 3; }

RUN_DIR="data/eval_runs/$RUN_ID"
LB_DIR="$RUN_DIR/leaderboard"
mkdir -p "$LB_DIR"
# nuway_eval (tools/eval) is imported by module name below; scenario_runner imports CARLA's PythonAPI `agents` package (GlobalRoutePlanner), which the
# carla wheel does not ship; it lives under $CARLA_ROOT/PythonAPI/carla (default ~/carla).
CARLA_ROOT="${CARLA_ROOT:-$HOME/carla}"
[ -d "$CARLA_ROOT/PythonAPI/carla/agents" ] || { echo "no $CARLA_ROOT/PythonAPI/carla/agents (set CARLA_ROOT)" >&2; exit 3; }
export PYTHONPATH="$ROOT/tools/eval:$ROOT/external/leaderboard:$ROOT/external/scenario_runner:$CARLA_ROOT/PythonAPI/carla${PYTHONPATH:+:$PYTHONPATH}"
export NUWAY_ROOT="$ROOT"
AGENT="$ROOT/ros2_ws/src/nuway_carla_bridge/nuway_carla_bridge/leaderboard_agent.py"

# Split the route file: one XML per route (the evaluator restarts game time per route and a file
# may span towns; the stack needs one town and monotonic sim time, docs/02 §2).
mapfile -t ROUTE_LIST < <(uv run python -m nuway_eval.leaderboard_results split "$ROUTES" "$LB_DIR" "${ROUTE_IDS[@]}")

stop_stack() {
  local pid=$1
  kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 20); do kill -0 "$pid" 2>/dev/null || break; sleep 1; done
  kill -9 "$pid" 2>/dev/null || true
  for p in nuway_planning nuway_control nuway_localization nuway_perception nuway_prediction nuway_route nuway_map nuway_viz nuway_carla_bridge; do
    pkill -9 -f "[i]nstall/$p/lib/" 2>/dev/null || true
  done
}

for entry in "${ROUTE_LIST[@]}"; do
  route_id=${entry%%,*}; rest=${entry#*,}; town=${rest%%,*}; xml=${rest#*,}
  profile="$LB_DIR/profile_${route_id}.yaml"
  uv run python - "$ROOT/configs/profiles/leaderboard.yaml" "$profile" "$town" "$HOST" "$PORT" <<'PY'
import sys, yaml
src, dst, town, host, port = sys.argv[1:]
prof = yaml.safe_load(open(src))
prof["carla"]["town"] = town
prof["carla"]["host"] = host
prof["carla"]["port"] = int(port)
yaml.safe_dump(prof, open(dst, "w"), sort_keys=False)
PY
  echo "== $route_id ($town)"
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 launch nuway_bringup stack.launch.py "profile:=$profile" foxglove:=false > "$LB_DIR/stack_${route_id}.log" 2>&1 &
  stack_pid=$!
  sleep "$STACK_WAIT_S"
  export NUWAY_LB_AGENT_JSON="$LB_DIR/${route_id}_agent.json"
  set +e
  uv run python external/leaderboard/leaderboard/leaderboard_evaluator.py \
    --routes "$xml" --agent "$AGENT" --agent-config "$profile" --track MAP \
    --checkpoint "$LB_DIR/${route_id}.json" --timeout "$EVAL_TIMEOUT_S" \
    --host "$HOST" --port "$PORT" --traffic-manager-port "$TM_PORT" --repetitions 1 \
    > "$LB_DIR/evaluator_${route_id}.log" 2>&1
  status=$?
  set -e
  echo "   evaluator exit $status (log: $LB_DIR/evaluator_${route_id}.log)"
  stop_stack "$stack_pid"
done

uv run python -m nuway_eval.leaderboard_results merge "$RUN_DIR"
