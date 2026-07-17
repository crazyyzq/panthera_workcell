# panthera_pose_tuner 调点包使用说明

这个包用于现场调试机械臂目标点位。它不替代正式流水线状态机，只负责让你可以在 C++ 代码里直接写坐标和姿态，然后通过 ROS2 service 做干跑规划或真实运动。

## 主要文件

```text
src/panthera_pose_tuner/
├── include/panthera_pose_tuner/TuningTargets.hpp
├── src/TuningTargets.cpp
├── src/pose_tuner_node.cpp
├── launch/pose_tuner.launch.py
└── README.md
```

各文件作用：

- `TuningTargets.cpp`：你主要修改的文件。所有可调点位都写在这里。
- `TuningTargets.hpp`：点位数据结构定义，一般不用改。
- `pose_tuner_node.cpp`：MoveIt 执行节点，提供服务接口，一般不用改。
- `pose_tuner.launch.py`：启动调点节点。
- `README.md`：包内快速说明。

## 写点位的位置

只改：

```text
src/panthera_pose_tuner/src/TuningTargets.cpp
```

示例：

```cpp
poseTarget(
  "outlet_1_high_check",
  "OUTLET_1 visible check pose above the cup.",
  -25.0, -428.53, 180.0,
  {0.0, 0.0, -90.0})
```

含义：

- `-25.0, -428.53, 180.0` 是 `base_link` 下 `gripper_center` 的目标坐标，单位 mm。
- `{0.0, 0.0, -90.0}` 是 `roll, pitch, yaw`，单位 degree。
- 名字必须唯一，后续 service 通过这个名字调用。

## 编译

```bash
cd ~/panthera_workcell_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select panthera_pose_tuner
source install/setup.bash
```

## 启动

先启动机械臂和 MoveIt：

```bash
cd ~/panthera_workcell_ws
source install/setup.bash
ros2 launch panthera_task_framework application_bringup.launch.py \
  rviz:=false start_hardware:=true start_workflow:=true \
  start_laser:=false start_state_signal:=false start_io:=false \
  execute_motion:=true
```

再启动调点节点：

```bash
cd ~/panthera_workcell_ws
source install/setup.bash
ros2 launch panthera_pose_tuner pose_tuner.launch.py execute_motion:=true
```

如果只想规划、不想真实运动：

```bash
ros2 launch panthera_pose_tuner pose_tuner.launch.py execute_motion:=false
```

## 服务接口

查看所有点位：

```bash
ros2 service call /pose_tuner/list_targets std_srvs/srv/Trigger "{}"
```

只规划，不运动：

```bash
ros2 service call /pose_tuner/run_target panthera_interfaces/srv/RunWorkflow \
  "{workflow_name: 'outlet_1_high_check', dry_run: true}"
```

真实运动：

```bash
ros2 service call /pose_tuner/run_target panthera_interfaces/srv/RunWorkflow \
  "{workflow_name: 'outlet_1_high_check', dry_run: false}"
```

停止当前 MoveIt 执行：

```bash
ros2 service call /pose_tuner/stop std_srvs/srv/Trigger "{}"
```

## 建议调试顺序

1. 修改 `TuningTargets.cpp` 中的目标点。
2. 编译 `panthera_pose_tuner`。
3. `dry_run: true` 先验证能否规划。
4. 执行 `safe_joint_center`。
5. 执行目标点的 `*_high_check`。
6. 确认方向和位置后，再逐步降低 Z，每次建议 5 到 10 mm。
7. 点位确认后，再迁移到正式状态机或工作流配置。

## 本次验证记录

验证时间：2026-06-15

编译日志：

```text
/home/b1/panthera_workcell_ws/validation_logs/20260615_104951_pose_tuner_build/build.log
```

服务和干跑验证日志：

```text
/home/b1/panthera_workcell_ws/validation_logs/20260615_105109_pose_tuner_start_verify/verify.log
```

真实运动验证日志：

```text
/home/b1/panthera_workcell_ws/validation_logs/20260615_105211_pose_tuner_real_motion/real_motion.log
```

最终在线检查日志：

```text
/home/b1/panthera_workcell_ws/validation_logs/20260615_105448_pose_tuner_final_check/final_check.log
```

已验证内容：

- `panthera_pose_tuner` 编译通过。
- `/pose_tuner/list_targets` 服务正常。
- `/pose_tuner/run_target` 服务正常。
- `safe_joint_center` 干跑规划成功。
- `outlet_1_high_check` 干跑规划成功。
- `safe_joint_center` 真实执行成功。
- `outlet_1_high_check` 真实执行成功。

真实执行后 HMI 读到的 `gripper_center` 位置：

```text
target:  x=-25.00 mm, y=-428.53 mm, z=180.00 mm, yaw=-90 deg
actual:  x=-24.68 mm, y=-426.42 mm, z=178.69 mm, yaw=-87.33 deg
```

这个结果说明调点包可以用于当前机械臂和 MoveIt 环境。
