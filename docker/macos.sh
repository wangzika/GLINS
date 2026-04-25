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
  ./docker/macos.sh bag-advanced [roslaunch args...]
  ./docker/macos.sh bag-full [roslaunch args...]
  ./docker/macos.sh bag-carrier-float [roslaunch args...]
  ./docker/macos.sh bag-carrier-no-nhc [roslaunch args...]
  ./docker/macos.sh bag-full --no-build [roslaunch args...]
  ./docker/macos.sh gui [roslaunch args...]
  ./docker/macos.sh desktop
  ./docker/macos.sh gui-web [roslaunch args...]

Examples:
  ./docker/macos.sh build
  ./docker/macos.sh shell
  ./docker/macos.sh bag bagpath:=/data/demo.bag imu_topic:=/imu/data lidar_topic:=/velodyne_points
  ./docker/macos.sh bag-advanced bagpath:=/data/lidar_imu.bag
  ./docker/macos.sh bag-full bagpath:=/data/lidar_imu.bag
  ./docker/macos.sh bag-carrier-float bagpath:=/data/lidar_imu.bag
  ./docker/macos.sh bag-carrier-no-nhc bagpath:=/data/lidar_imu.bag
  ./docker/macos.sh bag-full --no-build bagpath:=/data/lidar_imu.bag
  ./docker/macos.sh gui
  ./docker/macos.sh desktop
  ./docker/macos.sh gui-web bagpath:=/data/lidar_imu.bag
EOF
}

cmd="${1:-shell}"
if [ $# -gt 0 ]; then
  shift
fi

passthrough_args=()
while [ $# -gt 0 ]; do
  case "$1" in
    --no-build)
      export GLINS_AUTOBUILD=0
      shift
      ;;
    --build)
      export GLINS_AUTOBUILD=1
      shift
      ;;
    *)
      passthrough_args+=("$1")
      shift
      ;;
  esac
done

case "${cmd}" in
  build)
    docker compose build
    ;;
  shell)
    if [ ${#passthrough_args[@]} -gt 0 ]; then
      docker compose run --rm glins bash "${passthrough_args[@]}"
    else
      docker compose run --rm glins bash
    fi
    ;;
  bag)
    if [ ${#passthrough_args[@]} -gt 0 ]; then
      docker compose run --rm glins roslaunch glins run_bag.launch "${passthrough_args[@]}"
    else
      docker compose run --rm glins roslaunch glins run_bag.launch
    fi
    ;;
  bag-advanced)
    if [ ${#passthrough_args[@]} -gt 0 ]; then
      docker compose run --rm glins roslaunch glins run_bag_advanced.launch "${passthrough_args[@]}"
    else
      docker compose run --rm glins roslaunch glins run_bag_advanced.launch
    fi
    ;;
  bag-full)
    if [ ${#passthrough_args[@]} -gt 0 ]; then
      docker compose run --rm glins roslaunch glins run_bag_full.launch "${passthrough_args[@]}"
    else
      docker compose run --rm glins roslaunch glins run_bag_full.launch
    fi
    ;;
  bag-carrier-float)
    if [ ${#passthrough_args[@]} -gt 0 ]; then
      docker compose run --rm glins roslaunch glins run_bag_carrier_float.launch "${passthrough_args[@]}"
    else
      docker compose run --rm glins roslaunch glins run_bag_carrier_float.launch
    fi
    ;;
  bag-carrier-no-nhc)
    if [ ${#passthrough_args[@]} -gt 0 ]; then
      docker compose run --rm glins roslaunch glins run_bag_carrier_no_nhc.launch "${passthrough_args[@]}"
    else
      docker compose run --rm glins roslaunch glins run_bag_carrier_no_nhc.launch
    fi
    ;;
  gui)
    if [ ${#passthrough_args[@]} -gt 0 ]; then
      docker compose run --rm glins roslaunch glins run_bag.launch \
        rviz:=true robot_state_publisher:=true "${passthrough_args[@]}"
    else
      docker compose run --rm glins roslaunch glins run_bag.launch \
        rviz:=true robot_state_publisher:=true
    fi
    ;;
  desktop)
    echo "Open http://localhost:${NOVNC_PORT:-6080}/vnc.html in your browser."
    if [ ${#passthrough_args[@]} -gt 0 ]; then
      docker compose run --service-ports --rm glins /usr/local/bin/start-desktop.sh "${passthrough_args[@]}"
    else
      docker compose run --service-ports --rm glins /usr/local/bin/start-desktop.sh
    fi
    ;;
  gui-web)
    echo "Open http://localhost:${NOVNC_PORT:-6080}/vnc.html in your browser."
    if [ ${#passthrough_args[@]} -gt 0 ]; then
      docker compose run --service-ports --rm glins \
        /usr/local/bin/start-desktop.sh \
        roslaunch glins run_bag.launch rviz:=true robot_state_publisher:=true "${passthrough_args[@]}"
    else
      docker compose run --service-ports --rm glins \
        /usr/local/bin/start-desktop.sh \
        roslaunch glins run_bag.launch rviz:=true robot_state_publisher:=true
    fi
    ;;
  *)
    usage
    exit 1
    ;;
esac
