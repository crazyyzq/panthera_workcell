#!/usr/bin/env bash
set -eo pipefail

WS="${WS:-$HOME/panthera_workcell_ws}"
LOG_ROOT="$WS/validation_logs"
RUNTIME_DIR="$WS/.runtime"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LOG_ROOT/${STAMP}_start_workcell"

mkdir -p "$LOG_DIR" "$RUNTIME_DIR"
cd "$WS"

source_workspace()
{
  source /opt/ros/humble/setup.bash
  if [ -f "$WS/install/setup.bash" ]; then
    source "$WS/install/setup.bash"
  fi
}

ensure_required_packages()
{
  local required_packages=(
    hightorque_robot
    panthera_hardware
    panthera_ht_config
    panthera_ht_ros_description
    panthera_interfaces
    panthera_rs485
    panthera_task_framework
    panthera_spectrometer_cell
    panthera_pose_tuner
    panthera_web_hmi
  )
  local missing=()

  source_workspace
  for package_name in "${required_packages[@]}"; do
    if ! ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
      missing+=("$package_name")
    fi
  done

  if [ "${#missing[@]}" -eq 0 ]; then
    return 0
  fi

  echo "[start] workspace install is incomplete; missing packages: ${missing[*]}"
  echo "[start] running colcon build --symlink-install before launch..."
  colcon build --symlink-install > "$LOG_DIR/prestart_build.log" 2>&1
  source_workspace

  missing=()
  for package_name in "${required_packages[@]}"; do
    if ! ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
      missing+=("$package_name")
    fi
  done
  if [ "${#missing[@]}" -ne 0 ]; then
    echo "[start] ERROR: packages still missing after build: ${missing[*]}"
    echo "[start] build log: $LOG_DIR/prestart_build.log"
    exit 2
  fi
}

source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
fi

echo "[start] log directory: $LOG_DIR"
ensure_required_packages

if pgrep -f "ros2 launch .*panthera_" >/dev/null 2>&1 || pgrep -f "spectrometer_cell_node|web_hmi_node|workflow_executor_node" >/dev/null 2>&1; then
  echo "[start] existing workcell ROS processes detected."
  echo "[start] run scripts/stop_workcell.sh first if you want a clean restart."
fi

nohup ros2 launch panthera_task_framework application_bringup.launch.py \
  start_hardware:=true \
  start_workflow:=true \
  start_laser:=true \
  start_state_signal:=false \
  start_io:=false \
  execute_motion:=true \
  rviz:=false \
  laser_port:=/dev/ttyS4 \
  > "$LOG_DIR/base_bringup.log" 2>&1 &
echo $! > "$RUNTIME_DIR/base_bringup.pid"
echo "[start] base bringup pid=$(cat "$RUNTIME_DIR/base_bringup.pid")"

sleep 6

nohup ros2 launch panthera_web_hmi spectrometer_cell_hmi.launch.py \
  simulation:=false \
  start_cell:=true \
  start_camera:=true \
  start_workflow:=false \
  start_laser:=false \
  start_pose_tuner:=true \
  camera_color_width:=1280 \
  camera_color_height:=800 \
  camera_depth_width:=1280 \
  camera_depth_height:=800 \
  camera_color_fps:=30 \
  camera_depth_fps:=30 \
  camera_max_width:=640 \
  camera_target_fps:=30.0 \
  camera_worker_threads:=4 \
  > "$LOG_DIR/hmi_cell_camera.log" 2>&1 &
echo $! > "$RUNTIME_DIR/hmi_cell_camera.pid"
echo "[start] hmi/cell/camera pid=$(cat "$RUNTIME_DIR/hmi_cell_camera.pid")"

ln -sfn "$LOG_DIR" "$RUNTIME_DIR/latest_log"
echo "[start] HMI: http://$(hostname -I | awk '{print $1}'):8080"
echo "[start] logs: $LOG_DIR"
