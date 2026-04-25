#!/usr/bin/env bash
set -euo pipefail

DISPLAY_NUM="${NOVNC_DISPLAY:-:1}"
VNC_PORT="${VNC_PORT:-5900}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
VNC_RESOLUTION="${VNC_RESOLUTION:-1600x900x24}"

export DISPLAY="${DISPLAY_NUM}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"

mkdir -p "${XDG_RUNTIME_DIR}"
chmod 700 "${XDG_RUNTIME_DIR}"

cleanup() {
  jobs -pr | xargs -r kill >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

rm -f /tmp/.X1-lock

Xvfb "${DISPLAY_NUM}" -screen 0 "${VNC_RESOLUTION}" +extension GLX +render -noreset &
XVFB_PID=$!
sleep 1

fluxbox >/tmp/fluxbox.log 2>&1 &
FLUXBOX_PID=$!

xterm -geometry 140x40+20+20 -fa Monospace -fs 10 -title "GLINS Desktop Shell" &
XTERM_PID=$!

x11vnc -display "${DISPLAY_NUM}" -forever -shared -nopw -rfbport "${VNC_PORT}" >/tmp/x11vnc.log 2>&1 &
X11VNC_PID=$!

websockify --web=/usr/share/novnc/ "${NOVNC_PORT}" "localhost:${VNC_PORT}" >/tmp/novnc.log 2>&1 &
WEBSOCKIFY_PID=$!

echo "[desktop] noVNC available at http://localhost:${NOVNC_PORT}/vnc.html"

if [ "$#" -gt 0 ]; then
  "$@" >/tmp/desktop-command.log 2>&1 &
  APP_PID=$!
else
  APP_PID=""
fi

wait "${XVFB_PID}" "${FLUXBOX_PID}" "${XTERM_PID}" "${X11VNC_PID}" "${WEBSOCKIFY_PID}" ${APP_PID:+"${APP_PID}"}
