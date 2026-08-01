#!/usr/bin/env bash

set -Eeuo pipefail

WS="${WS:-/home/b1/panthera_workcell_ws}"
HMI_PORT="${HMI_PORT:-8080}"
CAMERA_RECOVERY_ATTEMPTS="${CAMERA_RECOVERY_ATTEMPTS:-3}"
CAMERA_READY_TIMEOUT_SEC="${CAMERA_READY_TIMEOUT_SEC:-20}"
CAMERA_WATCH_INTERVAL_SEC="${CAMERA_WATCH_INTERVAL_SEC:-5}"
CAMERA_FAILURE_THRESHOLD="${CAMERA_FAILURE_THRESHOLD:-3}"
CAMERA_RETRY_BACKOFF_SEC="${CAMERA_RETRY_BACKOFF_SEC:-300}"
RUNTIME_DIR="$WS/.runtime"

mkdir -p "$RUNTIME_DIR"
exec 9>"$RUNTIME_DIR/camera_recovery.lock"
flock -n 9 || exit 0

camera_ready()
{
  curl -fsS --max-time 2 "http://127.0.0.1:${HMI_PORT}/api/status" 2>/dev/null | python3 -c '
import json, sys
camera=json.load(sys.stdin).get("camera", {}).get("rgb", {})
age=camera.get("age_sec")
ok=camera.get("available", False) and isinstance(age, (int, float)) and age < 2.0
raise SystemExit(0 if ok else 1)
'
}

recover_camera()
{
  local attempt second
  for ((attempt = 1; attempt <= CAMERA_RECOVERY_ATTEMPTS; ++attempt)); do
    if camera_ready; then
      return 0
    fi
    echo "[camera-auto] recovery attempt $attempt/$CAMERA_RECOVERY_ATTEMPTS"
    WS="$WS" bash "$WS/scripts/restart_camera.sh"
    for ((second = 0; second < CAMERA_READY_TIMEOUT_SEC; ++second)); do
      if camera_ready; then
        echo "[camera-auto] RGB stream recovered"
        return 0
      fi
      sleep 1
    done
  done
  return 1
}

if camera_ready; then
  echo "[camera-auto] RGB stream already healthy; watchdog active"
elif ! recover_camera; then
  echo "[camera-auto] camera unavailable; workcell remains operational, retry in ${CAMERA_RETRY_BACKOFF_SEC}s"
  sleep "$CAMERA_RETRY_BACKOFF_SEC"
fi

failures=0
while true; do
  sleep "$CAMERA_WATCH_INTERVAL_SEC"
  if camera_ready; then
    failures=0
    continue
  fi
  ((failures += 1))
  if ((failures < CAMERA_FAILURE_THRESHOLD)); then
    continue
  fi
  echo "[camera-auto] RGB stream stale for $failures checks; starting recovery"
  if recover_camera; then
    failures=0
  else
    echo "[camera-auto] recovery exhausted; workcell remains operational, retry in ${CAMERA_RETRY_BACKOFF_SEC}s"
    failures=0
    sleep "$CAMERA_RETRY_BACKOFF_SEC"
  fi
done
