# panthera_pose_tuner

> 生产环境已停用。本包不能与固定轨迹 Motion Server/状态机并行运行；日常调点使用
> HMI 的独占调试模式和原子热重载。本 README 仅保留旧工具维护参考。

`panthera_pose_tuner` 是用于现场点位调试的小工具包。它和正式流水线状态机分开，方便你在 C++ 代码里直接写坐标和姿态，逐个点位做规划验证或真实运动。

## 你主要改哪个文件

只改这个文件：

```text
src/panthera_pose_tuner/src/TuningTargets.cpp
```

点位示例：

```cpp
poseTarget(
  "outlet_1_high_check",
  "OUTLET_1 visible check pose above the cup.",
  -25.0, -428.53, 180.0,     // x_mm, y_mm, z_mm
  {0.0, 0.0, -90.0})          // roll_deg, pitch_deg, yaw_deg
```

约定：

- `xyz_mm` 是 `base_link` 坐标系下的 `gripper_center` 目标位置，单位 mm。
- `rpy_deg` 是 `base_link` 坐标系下的 `gripper_center` 欧拉角，单位 degree。
- 调试时先跑 `*_high_check`，确认姿态和方向后再降低 Z。
- `safe_joint_center` 是关节过渡位，不是笛卡尔坐标点。

## 启动

先启动机械臂、MoveIt 和 workflow 基础进程：

```bash
cd ~/panthera_workcell_ws
source install/setup.bash
ros2 launch panthera_task_framework application_bringup.launch.py \
  rviz:=false start_hardware:=true start_workflow:=true \
  start_laser:=false start_state_signal:=false start_io:=false \
  execute_motion:=true
```

另开终端启动调试节点：

```bash
cd ~/panthera_workcell_ws
source install/setup.bash
ros2 launch panthera_pose_tuner pose_tuner.launch.py execute_motion:=true
```

如果只想规划、不想真的动机械臂：

```bash
ros2 launch panthera_pose_tuner pose_tuner.launch.py execute_motion:=false
```

## 查看点位

```bash
ros2 service call /pose_tuner/list_targets std_srvs/srv/Trigger "{}"
```

## 规划但不执行

```bash
ros2 service call /pose_tuner/run_target panthera_interfaces/srv/RunWorkflow \
  "{workflow_name: 'outlet_1_high_check', dry_run: true}"
```

## 真实执行

```bash
ros2 service call /pose_tuner/run_target panthera_interfaces/srv/RunWorkflow \
  "{workflow_name: 'outlet_1_high_check', dry_run: false}"
```

`dry_run: true` 只规划，不下发到控制器。`dry_run: false` 会真实运动；同时 launch 参数 `execute_motion` 也必须为 `true`。

## 急停式停止当前 MoveIt 执行

```bash
ros2 service call /pose_tuner/stop std_srvs/srv/Trigger "{}"
```

这个服务只调用 MoveIt 的 `stop()`，现场仍应以真实硬件急停为最高优先级。

## 当前内置点位

- `safe_joint_center`
- `outlet_1_high_check`
- `outlet_1_grip_check`
- `outlet_2_high_check`
- `outlet_2_grip_check`
- `spectrometer_high_check`
- `spectrometer_place_check`
- `clean_high_check`
- `clean_dump_check`

## 调试建议

1. 每次只改一个点。
2. 先 `dry_run: true`。
3. 再执行 `safe_joint_center`。
4. 再执行目标的 `*_high_check`。
5. 位置确认后，每次把 Z 改小 5 到 10 mm，逐步接近目标。
6. 确认姿态方向后，再把点位迁移到正式状态机配置或 `panthera_spectrometer_cell` 动作逻辑。
