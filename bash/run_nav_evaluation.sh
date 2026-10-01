#!/usr/bin/env bash
set -Eeuo pipefail

readonly repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
timeout=180
startup_timeout=45
sim_pub_hz=10.0
output="$repo/artifacts/nav-evaluation.json"

usage() {
  cat <<'EOF'
Usage: bash/run_nav_evaluation.sh [--timeout SECONDS] [--startup-timeout SECONDS]
                                  [--sim-pub-hz HZ] [--output FILE]
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --timeout) timeout="$2"; shift 2 ;;
    --startup-timeout) startup_timeout="$2"; shift 2 ;;
    --sim-pub-hz) sim_pub_hz="$2"; shift 2 ;;
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
set +u
source "$setup"
set -u
export SDL_VIDEODRIVER=dummy
export PYGAME_HIDE_SUPPORT_PROMPT=1
export QT_QPA_PLATFORM=offscreen
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

wait_for_odometry() {
  local deadline=$((SECONDS + 20))
  while ! ros2 topic list 2>/dev/null | grep -qx '/Odometry'; do
    (( SECONDS >= deadline )) && return 1
    sleep 0.2
  done
  timeout 5 ros2 topic echo /Odometry --once >/dev/null 2>&1
}

start_group tf.log ros2 run tf2_ros static_transform_publisher \
  0 0.15 0 0 0 0 base_link livox_frame
sim_params="$(ros2 pkg prefix sp_nav_sim)/share/sp_nav_sim/config/sim_robot.yaml"
start_group simulator.log ros2 run sp_nav_sim sim_robot --ros-args \
  --params-file "$sim_params" -p "pub_hz:=$sim_pub_hz"
if ! wait_for_odometry; then
  echo "Simulator did not publish /Odometry within 20 seconds." >&2
  exit 1
fi
start_group navigation.log ros2 launch sp_nav_bringup sp_nav.launch.py use_rviz:=false

cd "$repo"
python3 -m tools.evaluate_nav \
  --timeout "$timeout" \
  --startup-timeout "$startup_timeout" \
  --sim-pub-hz "$sim_pub_hz" \
  --output "$output"
