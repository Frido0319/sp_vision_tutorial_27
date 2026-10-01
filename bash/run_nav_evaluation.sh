#!/usr/bin/env bash
set -Eeuo pipefail

readonly repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
timeout=180
startup_timeout=45
output="$repo/artifacts/nav-evaluation.json"

usage() {
  cat <<'EOF'
Usage: bash/run_nav_evaluation.sh [--timeout SECONDS] [--startup-timeout SECONDS] [--output FILE]
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --timeout) timeout="$2"; shift 2 ;;
    --startup-timeout) startup_timeout="$2"; shift 2 ;;
    --output) output="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

readonly setup="$repo/install/setup.bash"
if [[ ! -f "$setup" ]]; then
  echo "Missing $setup; run colcon build --symlink-install first." >&2
  exit 1
fi

mkdir -p "$repo/artifacts"
source "$setup"
export SDL_VIDEODRIVER=dummy
export PYGAME_HIDE_SUPPORT_PROMPT=1
export ROS_LOG_DIR="$repo/artifacts/ros-log"

pids=()
stop_process_group() {
  local signal="$1"
  local pid
  for pid in "${pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "-$signal" -- "-$pid" 2>/dev/null || true
    fi
  done
}

cleanup() {
  trap - EXIT INT TERM
  stop_process_group INT
  local deadline=$((SECONDS + 5))
  local pid
  while (( SECONDS < deadline )); do
    local alive=0
    for pid in "${pids[@]}"; do
      kill -0 "$pid" 2>/dev/null && alive=1
    done
    (( alive == 0 )) && break
    sleep 0.1
  done
  stop_process_group TERM
  for pid in "${pids[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

start_group() {
  local log_file="$1"
  shift
  setsid "$@" >"$repo/artifacts/$log_file" 2>&1 &
  pids+=("$!")
}

start_group tf.log ros2 run tf2_ros static_transform_publisher \
  0 0.15 0 0 0 0 base_link livox_frame
start_group navigation.log ros2 launch sp_nav_bringup sp_nav.launch.py use_rviz:=false
start_group simulator.log ros2 launch sp_nav_sim sim_robot.launch.py

cd "$repo"
python3 tools/evaluate_nav.py \
  --timeout "$timeout" \
  --startup-timeout "$startup_timeout" \
  --output "$output"
