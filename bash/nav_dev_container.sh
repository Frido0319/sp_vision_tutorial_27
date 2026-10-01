#!/usr/bin/env bash
set -Eeuo pipefail

readonly image="sp-nav-humble:2026-10-01"
readonly repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  bash/nav_dev_container.sh build -- <command> [args...]
  bash/nav_dev_container.sh shell
  bash/nav_dev_container.sh gui -- <command> [args...]
EOF
}

if [[ $# -lt 1 ]]; then
  usage >&2
  exit 2
fi

mode="$1"
shift
if [[ "${1:-}" == "--" ]]; then shift; fi

case "$mode" in
  build|shell|gui) ;;
  *) usage >&2; exit 2 ;;
esac

if ! docker image inspect "$image" >/dev/null 2>&1; then
  printf 'Missing image %s. Build it with:\n' "$image" >&2
  printf '  docker build -f docker/Dockerfile.humble -t %s .\n' "$image" >&2
  exit 1
fi

tty_args=(-i)
if [[ -t 0 && -t 1 ]]; then tty_args+=(-t); fi

docker_args=(
  run --rm "${tty_args[@]}"
  --network host
  --user "$(id -u):$(id -g)"
  --workdir /workspace
  --volume "$repo:/workspace"
  --env HOME=/tmp
  --env ROS_HOME=/tmp/.ros
)

if [[ "$mode" == "gui" ]]; then
  [[ -n "${DISPLAY:-}" ]] || { echo 'DISPLAY is not set' >&2; exit 1; }
  docker_args+=(
    --env DISPLAY
    --env QT_X11_NO_MITSHM=1
    --env SDL_VIDEODRIVER=x11
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
  )
fi

if [[ "$mode" == "shell" && $# -eq 0 ]]; then
  set -- bash
elif [[ $# -eq 0 ]]; then
  usage >&2
  exit 2
fi

exec docker "${docker_args[@]}" "$image" bash -lc \
  'source /opt/ros/humble/setup.bash; exec "$@"' bash "$@"
