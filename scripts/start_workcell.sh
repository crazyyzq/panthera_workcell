#!/usr/bin/env bash

# Keep traps, file descriptors, and `exit` inside this script even when an
# operator starts it with `. start_workcell.sh` or `source start_workcell.sh`.
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  if bash "${BASH_SOURCE[0]}" "$@"; then
    return 0
  else
    return $?
  fi
fi

set -Eeuo pipefail

WS="${WS:-/home/b1/panthera_workcell_ws}"
RUNTIME_DIR="$WS/.runtime"
LOG_ROOT="$WS/validation_logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LOG_ROOT/${STAMP}_start_workcell"
PID_FILE="$RUNTIME_DIR/workcell.pid"
MODE_FILE="$RUNTIME_DIR/workcell.control_mode"
START_TIMEOUT_SEC="${START_TIMEOUT_SEC:-45}"
START_ATTEMPTS="${START_ATTEMPTS:-2}"
START_RETRY_DELAY_SEC="${START_RETRY_DELAY_SEC:-5}"
SPEED_SCALE="${SPEED_SCALE:-1.0}"
CONTROL_MODE="${CONTROL_MODE:-position_velocity}"
HMI_PORT="${HMI_PORT:-8080}"
HMI_ADVERTISED_HOST="${HMI_ADVERTISED_HOST:-100.95.35.71}"
CAMERA_AUTO_RECOVER="${CAMERA_AUTO_RECOVER:-1}"
MIT_KP="${MIT_KP:-75.0,105.0,135.0,135.0,75.0,75.0}"
MIT_KD="${MIT_KD:-5.5,5.5,5.5,5.5,5.5,5.5}"
MIT_GRAVITY_SCALE="${MIT_GRAVITY_SCALE:-0.0,1.04,1.10,1.52,0.0,0.0}"
PAYLOAD_MASS_KG="${PAYLOAD_MASS_KG:-0.0}"
PAYLOAD_COM_XYZ_M="${PAYLOAD_COM_XYZ_M:-0.0,0.0,0.0}"
PAYLOAD_FRAME="${PAYLOAD_FRAME:-gripper_center}"
ROBOT_CONFIG="${ROBOT_CONFIG:-$WS/install/panthera_ht_config/share/panthera_ht_config/robot_param/Follower_absolute.yaml}"
CELL_CONFIG="${CELL_CONFIG:-$WS/install/panthera_spectrometer_cell/share/panthera_spectrometer_cell/config/spectrometer_cell.yaml}"
KEEP_RUNNING_ON_FAILURE=0

delegate_start_to_hmi()
{
  local snapshot state
  echo "[start] terminal lacks realtime limits; delegating startup to persistent HMI"
  curl -fsS --max-time 5 -X POST \
    -H 'Content-Type: application/json' \
    -d "{\"control_mode\":\"${CONTROL_MODE}\"}" \
    "http://127.0.0.1:${HMI_PORT}/api/workcell/start" >/dev/null 2>&1 || true
  for _ in $(seq 1 60); do
    snapshot="$(curl -fsS --max-time 3 \
      "http://127.0.0.1:${HMI_PORT}/api/status" 2>/dev/null || true)"
    state="$(printf '%s' "$snapshot" | python3 -c '
import json, sys
try:
    workcell = json.load(sys.stdin).get("workcell_control", {})
except Exception:
    print("waiting")
    raise SystemExit
if workcell.get("busy"):
    print("waiting")
elif workcell.get("running") and workcell.get("last_exit_code") in (None, 0):
    print("ready")
else:
    print("failed")
' 2>/dev/null || echo waiting)"
    case "$state" in
      ready)
        echo "[start] READY via HMI: http://${HMI_ADVERTISED_HOST}:${HMI_PORT}"
        return 0
        ;;
      failed)
        echo "[start] ERROR: HMI-managed startup failed; inspect HMI operation log"
        return 1
        ;;
    esac
    sleep 2
  done
  echo "[start] ERROR: timed out waiting for HMI-managed startup"
  return 1
}

case "$CONTROL_MODE" in
  position_velocity|mit_gravity_compensation) ;;
  *)
    echo "[start] ERROR: CONTROL_MODE must be position_velocity or mit_gravity_compensation"
    exit 1
    ;;
esac

rt_priority_limit="$(ulimit -r)"
locked_memory_limit="$(ulimit -l)"
if { ! [[ "$rt_priority_limit" =~ ^[0-9]+$ ]] || (( rt_priority_limit < 50 )) ||
  [[ "$locked_memory_limit" != "unlimited" ]]; } &&
  curl -fsS --max-time 2 "http://127.0.0.1:${HMI_PORT}/api/status" >/dev/null 2>&1
then
  delegate_start_to_hmi
  exit $?
fi

mkdir -p "$LOG_DIR" "$RUNTIME_DIR"
cd "$WS"
exec 9>"$RUNTIME_DIR/workcell.lock"
if ! flock -n 9; then
  echo "[start] ERROR: another start/stop operation is running"
  exit 10
fi

exec > >(tee -a "$LOG_DIR/start.log") 2>&1

source_workspace()
{
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  set -u
  export RCUTILS_COLORIZED_OUTPUT=0
  # Every ROS control node runs on this IPC; remote operators use HTTP/SSH.
  # Pin DDS to UDP loopback so unplugging or changing an external NIC cannot
  # split the control graph, and stale Fast DDS SHM locks cannot block startup.
  export ROS_LOCALHOST_ONLY=1
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
}

reset_ros2_daemon()
{
  local daemon_pattern='[f]rom ros2cli.daemon.daemonize import main'
  timeout 3 ros2 daemon stop >/dev/null 2>&1 || true
  pkill -TERM -f "$daemon_pattern" 2>/dev/null || true
  local cli_pattern='/opt/ros/humble/bin/ros2 (action|bag|control|doctor|interface|node|param|pkg|run|service|topic)( |$)'
  pkill -TERM -f "$cli_pattern" 2>/dev/null || true
  for _ in 1 2 3 4 5; do
    if ! pgrep -f "$daemon_pattern|$cli_pattern" >/dev/null 2>&1; then
      break
    fi
    sleep 0.1
  done
  pkill -KILL -f "$daemon_pattern" 2>/dev/null || true
  pkill -KILL -f "$cli_pattern" 2>/dev/null || true
}

fail()
{
  echo "[start] ERROR: $*"
  exit 1
}

require_fresh_artifact()
{
  local source_dir="$1" artifact="$2" label="$3"
  [[ -e "$artifact" ]] || fail "installed artifact missing: $artifact"
  local build_stamp="$WS/build/$label/colcon_build.rc"
  [[ -e "$build_stamp" ]] || fail "build stamp missing for $label; rebuild before production start"
  if find "$source_dir" -type f \
    \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name 'CMakeLists.txt' -o -name 'package.xml' \) \
    -newer "$build_stamp" -print -quit | grep -q .; then
    fail "$label source is newer than its installed binary; rebuild before production start"
  fi
}

launcher_alive()
{
  [[ -s "$PID_FILE" ]] || return 1
  local pid
  pid="$(cat "$PID_FILE")"
  [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null
}

control_mode_matches()
{
  [[ -r "$MODE_FILE" ]] && [[ "$(<"$MODE_FILE")" == "$CONTROL_MODE" ]]
}

related_processes()
{
  pgrep -f 'fixed_spectrometer_cell\.launch\.py|motion_server_node|spectrometer_cell_node|ros2_control_node|laser_distance_node|auto_recover_camera\.sh|panthera_task_framework.*application_bringup|workflow_executor_node|pose_tuner_node|move_group' || true
}

start_camera_recovery()
{
  [[ "$CAMERA_AUTO_RECOVER" == "1" ]] || return 0
  local pid_file="$RUNTIME_DIR/camera_auto_recovery.pid"
  if [[ -s "$pid_file" ]] && kill -0 "$(cat "$pid_file")" 2>/dev/null; then
    echo "[start] camera auto-recovery is already running"
    return 0
  fi
  nohup env WS="$WS" HMI_PORT="$HMI_PORT" \
    bash "$WS/scripts/auto_recover_camera.sh" \
    >"$LOG_DIR/camera_auto_recovery.log" 2>&1 < /dev/null 9>&- &
  printf '%s\n' "$!" >"$pid_file"
  echo "[start] camera auto-recovery started asynchronously"
}

controllers_ready()
{
  local state
  state="$(timeout 5 ros2 control list_controllers 2>/dev/null | sed -r $'s/\\x1B\\[[0-9;]*[mK]//g')" || return 1
  grep -Eq '^arm_controller[[:space:]].*active' <<<"$state" &&
    grep -Eq '^gripper_controller[[:space:]].*active' <<<"$state" &&
    grep -Eq '^joint_state_broadcaster[[:space:]].*active' <<<"$state"
}

motion_ready()
{
  local reply
  reply="$(timeout 5 ros2 service call /motion/health std_srvs/srv/Trigger '{}' 2>/dev/null)" || return 1
  grep -q 'success=True' <<<"$reply" && grep -q 'controller=ready' <<<"$reply" && grep -q 'state=settled' <<<"$reply"
}

hmi_ready()
{
  curl -fsS --max-time 2 "http://127.0.0.1:${HMI_PORT}/api/status" | python3 -c '
import json, sys
d=json.load(sys.stdin)
cell=d.get("spectrometer_cell", {})
state=cell.get("state", "")
age=cell.get("state_age_sec", 999)
laser=d.get("laser", {})
laser_age=laser.get("age_sec")
laser_distance=laser.get("distance_mm")
services=d.get("services", {})
required=("auto_mode", "actual_cycle_outlet_1", "detection_done", "speed_scale")
ok=state in {"IDLE", "WAIT_DISCHARGE", "PAUSED"} and age is not None and age < 5
ok=ok and all(services.get(name, {}).get("ready", False) for name in required)
laser_mode="sensor" if laser.get("valid", False) and isinstance(laser_age, (int, float)) and laser_age < 2 else "default_150mm"
print(f"state={state} age={age} laser={laser_distance} laser_age={laser_age} positioning={laser_mode}")
raise SystemExit(0 if ok else 1)
'
}

actuators_holding()
{
  curl -fsS --max-time 2 "http://127.0.0.1:${HMI_PORT}/api/status" | python3 -c '
import json, sys
d=json.load(sys.stdin)
joint=d.get("joint_state", {})
names=joint.get("names", [])
efforts=joint.get("efforts", [])
age=joint.get("age_sec")
by_name=dict(zip(names, efforts))
# A braked-but-readable SDK connection reports approximately zero while every
# ROS controller still appears active. At the current all-zero Home, however,
# the healthy measured sum can be only 0.2-0.4 Nm, so this is a liveness check
# rather than a payload/gravity threshold.
gravity_effort=sum(abs(float(by_name.get(name, 0.0))) for name in ("joint2", "joint3", "joint4"))
ok=isinstance(age, (int, float)) and age < 1.0 and gravity_effort > 0.05
print(f"joint_age={age} gravity_effort={gravity_effort:.3f}")
raise SystemExit(0 if ok else 1)
'
}

recover_empty_startup_error()
{
  curl -fsS --max-time 2 "http://127.0.0.1:${HMI_PORT}/api/status" | python3 -c '
import json, sys
d=json.load(sys.stdin)
cell=d.get("spectrometer_cell", {})
context=cell.get("context") or {}
empty=(not context.get("has_active_task") and not context.get("cup_in_gripper")
       and not context.get("spectrometer_occupied"))
raise SystemExit(0 if cell.get("state") == "ERROR" and empty else 1)
' || return 1
  # The ROS 2 daemon can retain an invalid rcl context after a long recovery
  # call. Recreate only the CLI discovery process; control nodes stay running.
  reset_ros2_daemon
  timeout 12 ros2 service call /spectrometer_cell/restart_cleaning_motor \
    std_srvs/srv/Trigger '{}' >"$LOG_DIR/startup_motor_recovery.log" 2>&1 &&
    grep -q 'success=True' "$LOG_DIR/startup_motor_recovery.log"
}

cleanup_failed_attempt()
{
  if [[ "$KEEP_RUNNING_ON_FAILURE" -eq 1 ]]; then
    echo "[start] hardware remains enabled and holding position; inspect before shutdown"
    return
  fi
  WORKCELL_LOCK_HELD=1 STOP_WAIT_SEC=0 "$WS/scripts/stop_workcell.sh" --for-restart || true
}

trap 'rc=$?; if [[ $rc -ne 0 ]]; then ln -sfn "$LOG_DIR" "$RUNTIME_DIR/latest_log"; cleanup_failed_attempt; echo "[start] failed; logs: $LOG_DIR"; fi' EXIT
trap 'echo "[start] interrupted; performing guarded cleanup"; exit 130' INT
trap 'echo "[start] terminated; performing guarded cleanup"; exit 143' HUP TERM

echo "[start] preflight workspace=$WS speed=${SPEED_SCALE} control_mode=${CONTROL_MODE} hmi_port=${HMI_PORT}"
[[ -r /opt/ros/humble/setup.bash ]] || fail "ROS 2 Humble is not installed"
[[ -r "$WS/install/setup.bash" ]] || fail "workspace is not built; run colcon build first"
[[ -r "$ROBOT_CONFIG" ]] || fail "robot hardware config is missing: $ROBOT_CONFIG"
[[ -r "$CELL_CONFIG" ]] || fail "workcell process config is missing: $CELL_CONFIG"
source_workspace
# ros2controlcli uses the ROS 2 daemon even for local service calls. A daemon
# left behind by a previous DDS/NIC configuration can remain in !rclpy.ok().
reset_ros2_daemon
for command in ros2 curl python3 flock setsid pgrep pkill chrt; do
  command -v "$command" >/dev/null || fail "required command missing: $command"
done
[[ "$rt_priority_limit" =~ ^[0-9]+$ && "$rt_priority_limit" -ge 50 ]] ||
  fail "realtime priority is unavailable (ulimit -r=$rt_priority_limit); log in again after installing config/system/99-panthera-realtime.conf"
[[ "$locked_memory_limit" == "unlimited" ]] ||
  fail "locked memory is limited; install config/system/99-panthera-realtime.conf and log in again"
chrt -f 50 true >/dev/null 2>&1 ||
  fail "SCHED_FIFO 50 is unavailable for user $(id -un)"
[[ -c /dev/ttyS8 && -r /dev/ttyS8 && -w /dev/ttyS8 ]] || fail "/dev/ttyS8 is missing or inaccessible"
[[ -c /dev/ttyS4 && -r /dev/ttyS4 && -w /dev/ttyS4 ]] || fail "/dev/ttyS4 is missing or inaccessible"
[[ "$SPEED_SCALE" =~ ^(0\.[0-9]+|1(\.0+)?)$ ]] || fail "SPEED_SCALE must be in (0,1]"
[[ "$CONTROL_MODE" =~ ^(mit_gravity_compensation|position_velocity)$ ]] ||
  fail "CONTROL_MODE must be mit_gravity_compensation or position_velocity"
[[ "$HMI_PORT" =~ ^[0-9]+$ ]] || fail "HMI_PORT must be numeric"

for package in panthera_hardware panthera_motion panthera_spectrometer_cell panthera_web_hmi; do
  ros2 pkg prefix "$package" >/dev/null 2>&1 || fail "installed package missing: $package"
done
require_fresh_artifact \
  "$WS/src/panthera_hardware" \
  "$WS/install/panthera_hardware/lib/libpanthera_hardware.so" \
  panthera_hardware
require_fresh_artifact \
  "$WS/src/panthera_motion" \
  "$WS/install/panthera_motion/lib/panthera_motion/motion_server_node" \
  panthera_motion
require_fresh_artifact \
  "$WS/src/panthera_spectrometer_cell" \
  "$WS/install/panthera_spectrometer_cell/lib/panthera_spectrometer_cell/spectrometer_cell_node" \
  panthera_spectrometer_cell
python3 "$WS/scripts/check_home.py" --self-test >/dev/null

if launcher_alive && control_mode_matches && hmi_ready >/dev/null && motion_ready &&
  python3 "$WS/scripts/check_home.py" --timeout 3 --tolerance 0.05 >/dev/null 2>&1 &&
  actuators_holding >/dev/null; then
  [[ -L "$RUNTIME_DIR/active_log" ]] && ln -sfn "$(readlink -f "$RUNTIME_DIR/active_log")" "$RUNTIME_DIR/latest_log"
  echo "[start] workcell is already healthy (pid=$(cat "$PID_FILE"))"
  echo "[start] HMI: http://${HMI_ADVERTISED_HOST}:${HMI_PORT}"
  start_camera_recovery
  exit 0
fi

if [[ -n "$(related_processes)" ]]; then
  echo "[start] stale/conflicting workcell processes found; performing guarded cleanup"
  WORKCELL_LOCK_HELD=1 "$WS/scripts/stop_workcell.sh" --for-restart || fail "existing process cleanup was not safe/successful"
fi
rm -f "$PID_FILE"

for attempt in $(seq 1 "$START_ATTEMPTS"); do
  if (( attempt > 1 )); then
    echo "[start] waiting ${START_RETRY_DELAY_SEC}s before hardware reconnect"
    sleep "$START_RETRY_DELAY_SEC"
  fi
  echo "[start] launch attempt $attempt/$START_ATTEMPTS"
  setsid --wait ros2 launch panthera_motion fixed_spectrometer_cell.launch.py \
    default_speed_scale:="$SPEED_SCALE" \
    control_mode:="$CONTROL_MODE" \
    hardware_config_file:="$ROBOT_CONFIG" \
    cell_config_file:="$CELL_CONFIG" \
    mit_kp:="$MIT_KP" \
    mit_kd:="$MIT_KD" \
    mit_gravity_scale:="$MIT_GRAVITY_SCALE" \
    payload_mass_kg:="$PAYLOAD_MASS_KG" \
    payload_com_xyz_m:="$PAYLOAD_COM_XYZ_M" \
    payload_frame:="$PAYLOAD_FRAME" \
    start_hardware:=true simulation:=false start_hmi:=false hmi_port:="$HMI_PORT" \
    >"$LOG_DIR/launch_attempt_${attempt}.log" 2>&1 < /dev/null 9>&- &
  launch_pid=$!
  printf '%s\n' "$launch_pid" >"$PID_FILE.tmp"
  mv "$PID_FILE.tmp" "$PID_FILE"

  home_checked=0
  restart_required=0
  attempt_deadline=$((SECONDS + START_TIMEOUT_SEC))
  second=0
  while (( SECONDS < attempt_deadline )); do
    second=$((second + 1))
    if ! kill -0 "$launch_pid" 2>/dev/null; then
      echo "[start] launcher exited during startup"
      break
    fi
    if (( second >= 5 )) &&
      ! pgrep -f 'controller_manager/ros2_control_node' >/dev/null 2>&1 &&
      grep -q 'Failed to initialize Panthera robot' "$LOG_DIR/launch_attempt_${attempt}.log"; then
      echo "[start] hardware initialization failed; check controller power/network"
      break
    fi
    if controllers_ready && motion_ready; then
      if python3 "$WS/scripts/check_home.py" --timeout 3 --tolerance 0.05 >>"$LOG_DIR/home_check.log" 2>&1; then
        home_checked=1
      else
        home_rc=$?
        if [[ "$home_rc" -eq 2 ]]; then
          if timeout 45 ros2 service call /spectrometer_cell/recover_home std_srvs/srv/Trigger '{}' \
              >"$LOG_DIR/startup_home_recovery.log" 2>&1 &&
            grep -q 'success=True' "$LOG_DIR/startup_home_recovery.log" &&
            python3 "$WS/scripts/check_home.py" --timeout 3 --tolerance 0.05 \
              >>"$LOG_DIR/home_check.log" 2>&1; then
            echo "[start] recovered startup pose to commissioned Home; restarting control stack"
            home_checked=1
            restart_required=1
            break
          else
            KEEP_RUNNING_ON_FAILURE=1
            fail "robot is not safely recoverable to commissioned Home; system left powered and holding"
          fi
        fi
      fi
    fi
    if (( second % 5 == 0 )) && recover_empty_startup_error; then
      if python3 "$WS/scripts/check_home.py" --timeout 3 --tolerance 0.05 \
          >>"$LOG_DIR/home_check.log" 2>&1; then
        echo "[start] restored empty startup prerequisite; restarting control stack"
        home_checked=1
        restart_required=1
        break
      fi
    fi
    if [[ "$home_checked" -eq 1 ]] &&
      hmi_ready >>"$LOG_DIR/health_wait.log" 2>&1 &&
      actuators_holding >>"$LOG_DIR/health_wait.log" 2>&1; then
      echo "[start] READY pid=$launch_pid state=settled controllers=active Home=verified"
      echo "[start] HMI: http://${HMI_ADVERTISED_HOST}:${HMI_PORT}"
      echo "[start] logs: $LOG_DIR"
      ln -sfn "$LOG_DIR" "$RUNTIME_DIR/active_log"
      ln -sfn "$LOG_DIR" "$RUNTIME_DIR/latest_log"
      printf '%s\n' "$CONTROL_MODE" >"$MODE_FILE.tmp"
      mv "$MODE_FILE.tmp" "$MODE_FILE"
      start_camera_recovery
      trap - EXIT
      exit 0
    fi
    sleep 1
  done

  if [[ "$home_checked" -eq 1 ]]; then
    if [[ "$restart_required" -eq 1 ]]; then
      echo "[start] attempt $attempt completed bounded recovery; safe restart required"
    else
      echo "[start] attempt $attempt failed health checks; safe cleanup before retry"
    fi
    WORKCELL_LOCK_HELD=1 STOP_WAIT_SEC=0 "$WS/scripts/stop_workcell.sh" --for-restart || fail "failed attempt could not be cleaned safely"
    rm -f "$PID_FILE"
  elif pgrep -f 'controller_manager/ros2_control_node' >/dev/null 2>&1; then
    KEEP_RUNNING_ON_FAILURE=1
    fail "startup health unavailable; system left powered to avoid an unsafe disable"
  else
    echo "[start] hardware process is not active; cleaning failed launch before retry"
    WORKCELL_LOCK_HELD=1 STOP_WAIT_SEC=0 "$WS/scripts/stop_workcell.sh" --for-restart ||
      fail "failed launch could not be cleaned safely; refusing duplicate retry"
    rm -f "$PID_FILE"
  fi
done

fail "workcell did not become ready after $START_ATTEMPTS attempts"
