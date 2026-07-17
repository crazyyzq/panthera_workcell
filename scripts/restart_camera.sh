#!/usr/bin/env bash
set +u

WS="${WS:-$HOME/panthera_workcell_ws}"
LOG_ROOT="$WS/validation_logs"
RUNTIME_DIR="$WS/.runtime"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$LOG_ROOT/${STAMP}_restart_camera"

mkdir -p "$LOG_DIR" "$RUNTIME_DIR"
cd "$WS" || exit 1
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "[camera] stopping existing Gemini305/camera container processes..."
pkill -TERM -f "ros2 launch panthera_ht_config gemini305_grasp_max.launch.py" 2>/dev/null || true
pkill -TERM -f "gemini305|orbbec|camera_container|component_container" 2>/dev/null || true
sleep 2
pkill -KILL -f "gemini305|orbbec|camera_container|component_container" 2>/dev/null || true

echo "[camera] starting Gemini305 1280x800@30..."
nohup ros2 launch panthera_ht_config gemini305_grasp_max.launch.py \
  color_width:=1280 \
  color_height:=800 \
  color_fps:=30 \
  color_format:=YUYV \
  depth_width:=1280 \
  depth_height:=800 \
  depth_fps:=30 \
  depth_format:=Y16 \
  depth_registration:=false \
  align_mode:=SW \
  align_target_stream:=COLOR \
  enable_point_cloud:=false \
  enable_frame_sync:=false \
  > "$LOG_DIR/camera.log" 2>&1 &
echo $! > "$RUNTIME_DIR/camera_restart.pid"
ln -sfn "$LOG_DIR" "$RUNTIME_DIR/latest_camera_log"
echo "[camera] pid=$(cat "$RUNTIME_DIR/camera_restart.pid") logs=$LOG_DIR"
