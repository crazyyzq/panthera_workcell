#!/usr/bin/env bash
set +u

WS="${WS:-$HOME/panthera_workcell_ws}"
LOG_ROOT="$WS/validation_logs"
RUNTIME_DIR="$WS/.runtime"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LOG_ROOT/${STAMP}_stop_workcell"
NO_HOME="${1:-}"

mkdir -p "$LOG_DIR" "$RUNTIME_DIR"
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "[stop] log directory: $LOG_DIR" | tee "$LOG_DIR/stop.log"

wait_for_arm_home_finished()
{
  local timeout_sec="${1:-120}"
  local i
  for i in $(seq 1 "$timeout_sec"); do
    local status
    status="$(
      curl -sS --max-time 2 http://127.0.0.1:8080/api/status 2>/dev/null | \
      python3 -c "import json,sys; d=json.load(sys.stdin); w=d.get('workflow',{}); print(f\"{w.get('name','')}|{w.get('state','')}|{w.get('message','')}\")" 2>/dev/null || true
    )"
    echo "[stop] arm_home wait ${i}/${timeout_sec}: ${status:-no_hmi_status}" >> "$LOG_DIR/arm_home_wait.log"
    if echo "$status" | grep -q "^arm_home|2|.*finished"; then
      return 0
    fi
    if echo "$status" | grep -q "^arm_home|3|"; then
      return 1
    fi
    sleep 1
  done
  return 1
}

if [ "$NO_HOME" != "--no-home" ]; then
  echo "[stop] moving robot to home/safe pose before shutdown..." | tee -a "$LOG_DIR/stop.log"
  timeout 20s ros2 service call /run_workflow panthera_interfaces/srv/RunWorkflow \
    "{workflow_name: 'arm_home', dry_run: false}" \
    > "$LOG_DIR/arm_home.log" 2>&1
  HOME_RC=$?

  if [ "$HOME_RC" -eq 0 ] && wait_for_arm_home_finished "${STOP_HOME_WAIT_SEC:-120}"; then
    echo "[stop] arm_home finished; continuing shutdown." | tee -a "$LOG_DIR/stop.log"
  else
    echo "[stop] arm_home not confirmed finished, trying pose_tuner safe_joint_center..." | tee -a "$LOG_DIR/stop.log"
    timeout 90s ros2 service call /pose_tuner/run_target panthera_interfaces/srv/RunWorkflow \
      "{workflow_name: 'safe_joint_center', dry_run: false}" \
      > "$LOG_DIR/safe_joint_center.log" 2>&1
    SAFE_RC=$?
    if [ "$SAFE_RC" -ne 0 ]; then
      echo "[stop] ERROR: robot home/safe motion was not confirmed; not killing processes." | tee -a "$LOG_DIR/stop.log"
      echo "[stop] Use scripts/stop_workcell.sh --no-home only after manually confirming it is safe." | tee -a "$LOG_DIR/stop.log"
      exit 3
    fi
  fi
else
  echo "[stop] --no-home: skipping robot home motion" | tee -a "$LOG_DIR/stop.log"
fi

echo "[stop] terminating workcell ROS processes..." | tee -a "$LOG_DIR/stop.log"
pkill -TERM -f "ros2 launch panthera_task_framework application_bringup.launch.py" 2>/dev/null || true
pkill -TERM -f "ros2 launch panthera_web_hmi spectrometer_cell_hmi.launch.py" 2>/dev/null || true
pkill -TERM -f "ros2 launch panthera_ht_config gemini305_grasp_max.launch.py" 2>/dev/null || true
pkill -TERM -f "ros2 service call" 2>/dev/null || true
pkill -TERM -f "spectrometer_cell_node|web_hmi_node|workflow_executor_node|pose_tuner_node|laser_distance_node|rs485_state_signal_node|gpio_io_node" 2>/dev/null || true
pkill -TERM -f "move_group|ros2_control_node|controller_manager|robot_state_publisher|rviz2|component_container" 2>/dev/null || true

sleep 3

pkill -KILL -f "spectrometer_cell_node|web_hmi_node|workflow_executor_node|pose_tuner_node|laser_distance_node|rs485_state_signal_node|gpio_io_node" 2>/dev/null || true
pkill -KILL -f "move_group|ros2_control_node|controller_manager|robot_state_publisher|component_container" 2>/dev/null || true
pkill -KILL -f "ros2 service call" 2>/dev/null || true

rm -f "$RUNTIME_DIR"/*.pid
ln -sfn "$LOG_DIR" "$RUNTIME_DIR/latest_stop_log"
echo "[stop] done. logs: $LOG_DIR" | tee -a "$LOG_DIR/stop.log"
