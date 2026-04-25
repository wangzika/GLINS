#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

mkdir -p docker/build docker/devel docker/logs docker/output

export DOCKER_PLATFORM="${DOCKER_PLATFORM:-linux/amd64}"
DEFAULT_DISPLAY="${XQUARTZ_DISPLAY:-host.docker.internal:0}"
case "${DISPLAY:-}" in
  ""|/private/tmp/*|localhost:*|:*)
    export DISPLAY="${DEFAULT_DISPLAY}"
    ;;
  *)
    export DISPLAY="${DISPLAY}"
    ;;
esac
export GLINS_AUTOBUILD="${GLINS_AUTOBUILD:-1}"
export GLINS_BUILD_JOBS="${GLINS_BUILD_JOBS:-2}"
export GLINS_BUILD_LOAD="${GLINS_BUILD_LOAD:-2}"
export GTSAM_BUILD_JOBS="${GTSAM_BUILD_JOBS:-2}"

usage() {
  cat <<'EOF'
Usage:
  ./docker/macos.sh build
  ./docker/macos.sh shell
  ./docker/macos.sh bag [roslaunch args...]
  ./docker/macos.sh gui [roslaunch args...]
  ./docker/macos.sh desktop
  ./docker/macos.sh gui-web [roslaunch args...]

Examples:
  ./docker/macos.sh build
  ./docker/macos.sh shell
  ./docker/macos.sh bag bagpath:=/data/demo.bag imu_topic:=/imu/data lidar_topic:=/velodyne_points
  ./docker/macos.sh gui
  ./docker/macos.sh desktop
  ./docker/macos.sh gui-web bagpath:=/data/lidar_imu.bag
EOF
}

cmd="${1:-shell}"
if [ $# -gt 0 ]; then
  shift
fi

case "${cmd}" in
  build)
    docker compose build
    ;;
  shell)
    docker compose run --rm glins bash "$@"
    ;;
  bag)
    docker compose run --rm glins roslaunch glins run_bag.launch "$@"
    ;;
  gui)
    docker compose run --rm glins roslaunch glins run_bag.launch \
      rviz:=true robot_state_publisher:=true "$@"
    ;;
  desktop)
    echo "Open http://localhost:${NOVNC_PORT:-6080}/vnc.html in your browser."
    docker compose run --service-ports --rm glins /usr/local/bin/start-desktop.sh "$@"
    ;;
  gui-web)
    echo "Open http://localhost:${NOVNC_PORT:-6080}/vnc.html in your browser."
    docker compose run --service-ports --rm glins \
      /usr/local/bin/start-desktop.sh \
      roslaunch glins run_bag.launch rviz:=true robot_state_publisher:=true "$@"
    ;;
  *)
    usage
    exit 1
    ;;
esac
