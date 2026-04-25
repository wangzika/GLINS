#!/usr/bin/env bash
set -euo pipefail

WORKSPACE=/catkin_ws
BUILD_JOBS="${GLINS_BUILD_JOBS:-2}"
BUILD_LOAD="${GLINS_BUILD_LOAD:-2}"

export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

source /opt/ros/noetic/setup.bash

mkdir -p "${WORKSPACE}/build" "${WORKSPACE}/devel" "${WORKSPACE}/logs"

if [ "${GLINS_AUTOBUILD:-0}" = "1" ] || [ ! -f "${WORKSPACE}/devel/setup.bash" ]; then
  echo "[entrypoint] building catkin workspace with -j${BUILD_JOBS} -l${BUILD_LOAD}..."
  pushd "${WORKSPACE}" >/dev/null
  catkin_make -j"${BUILD_JOBS}" -l"${BUILD_LOAD}"
  popd >/dev/null
fi

if [ -f "${WORKSPACE}/devel/setup.bash" ]; then
  source "${WORKSPACE}/devel/setup.bash"
fi

exec "$@"
