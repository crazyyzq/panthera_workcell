#!/usr/bin/env bash
set -Eeuo pipefail

WS="${WS:-/home/b1/panthera_workcell_ws}"
RUNTIME_DIR="$WS/.runtime"
LOG_ROOT="$WS/validation_logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LOG_ROOT/${STAMP}_stop_workcell"
PID_FILE="$RUNTIME_DIR/workcell.pid"
STOP_WAIT_SEC="${STOP_WAIT_SEC:-180}"
HMI_PORT="${HMI_PORT:-8080}"
FORCE=0
FOR_RESTART=0
HOME_STATUS=not_checked

for arg in "$@"; do
  case "$arg" in
    --force|--no-home) FORCE=1 ;;
    --for-restart) FOR_RESTART=1 ;;
    *) echo "usage: $0 [--for-restart] [--force]"; exit 2 ;;
  esac
done

mkdir -p "$LOG_DIR" "$RUNTIME_DIR"
cd "$WS"
if [[ "${WORKCELL_LOCK_HELD:-0}" != "1" ]]; then
  exec 9>"$RUNTIME_DIR/workcell.lock"
  if ! flock -n 9; then
    echo "[stop] ERROR: another start/stop operation is running"
    exit 10
  fi
fi
exec > >(tee -a "$LOG_DIR/stop.log") 2>&1
ln -sfn "$LOG_DIR" "$RUNTIME_DIR/latest_stop_log"

source_workspace()
{
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  if [[ -r "$WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash"
  fi
  set -u
  export RCUTILS_COLORIZED_OUTPUT=0
  export ROS_LOCALHOST_ONLY=1
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
}

reset_ros2_daemon()
{
  timeout 3 ros2 daemon stop >/dev/null 2>&1 || true
  pkill -TERM -f '[f]rom ros2cli.daemon.daemonize import main' 2>/dev/null || true
  local cli_pattern='/opt/ros/humble/bin/ros2 (action|bag|control|doctor|interface|node|param|pkg|run|service|topic)( |$)'
  pkill -TERM -f "$cli_pattern" 2>/dev/null || true
  for _ in 1 2 3 4 5; do
    pgrep -f "$cli_pattern" >/dev/null 2>&1 || break
    sleep 0.1
  done
  pkill -KILL -f "$cli_pattern" 2>/dev/null || true
}

PROCESS_PATTERNS=(
  'fixed_spectrometer_cell\.launch\.py'
  'fixed_motion_bringup\.launch\.py'
  'motion_server_node'
  'spectrometer_cell_node'
  'web_hmi_node'
  'controller_manager/ros2_control_node'
  'robot_state_publisher'
  'static_transform_publisher'
  'controller_manager/spawner'
  'panthera_task_framework.*application_bringup'
  'workflow_executor_node'
  'pose_tuner_node'
  'laser_distance_node'
  'rs485_state_signal_node'
  'move_group'
  'gemini305_grasp_max\.launch\.py'
  'orbbec_camera'
)

related_pids()
{
  local pattern pid
  for pattern in "${PROCESS_PATTERNS[@]}"; do
    while read -r pid; do
      [[ -n "$pid" ]] && printf '%s\n' "$pid"
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
  done | sort -un
}

signal_related()
{
  local signal="$1"
  mapfile -t pids < <(related_pids)
  if [[ "${#pids[@]}" -gt 0 ]]; then
    kill "-$signal" "${pids[@]}" 2>/dev/null || true
  fi
}

cell_status()
{
  curl -fsS --max-time 2 "http://127.0.0.1:${HMI_PORT}/api/status" 2>/dev/null | python3 -c '
import json, sys
d=json.load(sys.stdin).get("spectrometer_cell", {})
c=d.get("context", {}) if isinstance(d.get("context", {}), dict) else {}
print("|".join((
    str(d.get("state", "UNKNOWN")),
    "1" if c.get("has_active_task", False) else "0",
    "1" if c.get("cup_in_gripper", False) else "0",
    "1" if c.get("spectrometer_occupied", False) else "0",
)))
'
}

motion_idle_and_settled()
{
  local reply
  reply="$(timeout 5 ros2 service call /motion/health std_srvs/srv/Trigger '{}' 2>/dev/null)" || return 1
  grep -q 'success=True' <<<"$reply" && grep -q 'busy=false' <<<"$reply" && grep -q 'state=settled' <<<"$reply"
}

wait_until_safe_state()
{
  local elapsed=0 status state active cup occupied
  local error_reset_attempted=0
  while (( elapsed <= STOP_WAIT_SEC )); do
    status="$(cell_status || true)"
    if [[ -z "$status" ]] && motion_idle_and_settled &&
      python3 "$WS/scripts/check_home.py" --timeout 3 --tolerance 0.05 \
        >>"$LOG_DIR/home_check_fallback.log" 2>&1; then
      echo "[stop] HMI unavailable, but Motion Server is idle and original Home is verified"
      return 0
    fi
    IFS='|' read -r state active cup occupied <<<"${status:-UNKNOWN|1|1|1}"
    echo "[stop] safety wait ${elapsed}/${STOP_WAIT_SEC}s state=$state active=$active cup=$cup occupied=$occupied"
    if [[ "$state" == "ERROR" && "$active" == "1" && "$cup" == "0" &&
      "$occupied" == "0" && "$error_reset_attempted" == "0" ]]
    then
      error_reset_attempted=1
      if python3 "$WS/scripts/check_home.py" --timeout 3 --tolerance 0.05 \
        >"$LOG_DIR/error_home_check.log" 2>&1; then
        echo "[stop] failed task is empty and Home-verified; clearing stale ERROR context"
        timeout 5 ros2 service call /spectrometer_cell/request_reset std_srvs/srv/Trigger '{}' \
          >"$LOG_DIR/error_reset.log" 2>&1 || true
        sleep 1
        continue
      fi
    fi
    if [[ "$active" == "0" && "$cup" == "0" && "$occupied" == "0" ]] &&
      [[ "$state" == "IDLE" || "$state" == "WAIT_DISCHARGE" || "$state" == "PAUSED" || "$state" == "ERROR" ]]; then
      return 0
    fi
    (( STOP_WAIT_SEC == 0 )) && return 1
    sleep 1
    ((elapsed+=1))
  done
  return 1
}

source_workspace
reset_ros2_daemon
mapfile -t initial_pids < <(related_pids)
if [[ "${#initial_pids[@]}" -eq 0 ]]; then
  rm -f "$PID_FILE"
  reset_ros2_daemon
  echo "[stop] workcell already stopped; no related processes found"
  exit 0
fi

echo "[stop] detected related pids: ${initial_pids[*]}"
hardware_running=0
pgrep -f 'controller_manager/ros2_control_node' >/dev/null 2>&1 && hardware_running=1
if [[ "$FORCE" -eq 0 && "$hardware_running" -eq 1 ]]; then
  if ! wait_until_safe_state; then
    echo "[stop] ERROR: active/incomplete cycle did not reach a safe empty state"
    echo "[stop] finish/recover the cycle first; processes remain running"
    echo "[stop] --force is only for a manually verified emergency recovery"
    exit 3
  fi

  timeout 5 ros2 service call /spectrometer_cell/manual_mode std_srvs/srv/Trigger '{}' \
    >"$LOG_DIR/manual_mode.log" 2>&1 || true
  sleep 1
  if ! motion_idle_and_settled; then
    echo "[stop] ERROR: Motion Server is busy or arm is not settled; processes remain running"
    exit 4
  fi
  if ! timeout 45 ros2 service call /spectrometer_cell/recover_home std_srvs/srv/Trigger '{}' \
    >"$LOG_DIR/recover_home.log" 2>&1 || ! grep -q 'success=True' "$LOG_DIR/recover_home.log"; then
    if motion_idle_and_settled &&
      python3 "$WS/scripts/check_home.py" --timeout 5 --tolerance 0.05 \
        >"$LOG_DIR/recover_home_fallback.log" 2>&1; then
      echo "[stop] recovery service unavailable, but Motion Server is settled and original Home is encoder-verified"
    else
      echo "[stop] ERROR: explicit Home recovery failed and original Home was not verified; processes remain running"
      exit 5
    fi
  fi
  if ! python3 "$WS/scripts/check_home.py" --timeout 5 --tolerance 0.05 | tee "$LOG_DIR/home_check.log"; then
    echo "[stop] ERROR: commissioned Home was not verified; processes remain running"
    exit 6
  fi
  HOME_STATUS=verified
elif [[ "$FORCE" -eq 1 ]]; then
  echo "[stop] WARNING: --force skips cycle/Home verification"
  HOME_STATUS=force_skipped
else
  echo "[stop] hardware process is not active; cleaning non-hardware residuals"
  HOME_STATUS=not_required_hardware_inactive
fi

# At Home this also sends an explicit brush stop and cancels any stale action goal.
timeout 8 ros2 service call /spectrometer_cell/stop_motion std_srvs/srv/Trigger '{}' \
  >"$LOG_DIR/stop_motion.log" 2>&1 || true

if [[ -s "$PID_FILE" ]]; then
  launcher_pid="$(cat "$PID_FILE")"
  if [[ "$launcher_pid" =~ ^[0-9]+$ ]] && kill -0 "$launcher_pid" 2>/dev/null; then
    echo "[stop] TERM launch process group -$launcher_pid"
    kill -TERM -- "-$launcher_pid" 2>/dev/null || kill -TERM "$launcher_pid" 2>/dev/null || true
  fi
fi
signal_related TERM

for _ in $(seq 1 10); do
  [[ -z "$(related_pids)" ]] && break
  sleep 1
done

mapfile -t remaining < <(related_pids)
if [[ "${#remaining[@]}" -gt 0 ]]; then
  echo "[stop] escalating KILL for remaining pids: ${remaining[*]}"
  signal_related KILL
  sleep 1
fi

mapfile -t remaining < <(related_pids)
if [[ "${#remaining[@]}" -gt 0 ]]; then
  echo "[stop] ERROR: related processes remain: ${remaining[*]}"
  exit 7
fi

rm -f "$PID_FILE" "$RUNTIME_DIR"/*.pid
rm -f "$RUNTIME_DIR/active_log"
reset_ros2_daemon
if ss -ltn 2>/dev/null | grep -qE "[:.]${HMI_PORT}[[:space:]]"; then
  echo "[stop] ERROR: port ${HMI_PORT} is still listening"
  exit 8
fi

echo "[stop] STOPPED Home=$HOME_STATUS brush=stop processes=clean"
[[ "$FOR_RESTART" -eq 1 ]] || echo "[stop] logs: $LOG_DIR"
