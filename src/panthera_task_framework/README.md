# panthera_task_framework

`panthera_task_framework` 是机械臂应用层流程执行器。它把 MoveIt 运动、夹爪动作、传感器输入、外部状态信号都放进 YAML 流程里，后续改作业流程主要改 YAML，不需要反复改 C++ 主逻辑。

## 一键启动

真实机械臂 + MoveIt + 工作流 + RS485 激光：

```bash
cd ~/Panthera-HT_ROS2-humble
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch panthera_task_framework application_bringup.launch.py
```

常用参数：

```bash
ros2 launch panthera_task_framework application_bringup.launch.py \
  rviz:=true \
  start_laser:=true \
  laser_port:=/dev/ttyS4 \
  start_state_signal:=true \
  state_port:=/dev/ttyS3 \
  start_io:=false \
  state_auto_run_workflow:=false \
  default_workflow:=cup_pick_place
```

同时启用 IO 输入时增加：

```bash
ros2 launch panthera_task_framework application_bringup.launch.py \
  start_io:=true \
  io_enabled:=true \
  io_backend:=sysfs \
  io_allow_output_writes:=false
```

当前这台 B1S-3588 检测到 DIDO 对应引脚被 CAN/SPI 等功能占用，真实 IO 默认关闭。未确认 DIDO 接口板前，用 `io_backend:=mock` 做软件验证。

只启动流程执行器：

```bash
ros2 launch panthera_task_framework workflow_executor.launch.py
```

## 控制接口

运行默认流程：

```bash
ros2 service call /run_default_workflow std_srvs/srv/Trigger "{}"
```

运行指定流程：

```bash
ros2 service call /run_workflow panthera_interfaces/srv/RunWorkflow \
  "{workflow_name: cup_pick_place, dry_run: false}"
```

修改 YAML 后不重启节点，直接重载流程文件：

```bash
ros2 service call /reload_workflows std_srvs/srv/Trigger "{}"
```

只规划不执行：

```bash
ros2 service call /run_workflow panthera_interfaces/srv/RunWorkflow \
  "{workflow_name: cup_pick_place, dry_run: true}"
```

查看流程状态：

```bash
ros2 topic echo /workflow/status
```

`/workflow/status` 会发布当前流程名、当前步骤、步骤编号、状态和错误信息。

## YAML 流程结构

默认文件：

```text
src/panthera_task_framework/config/cup_pick_place_workflows.yaml
```

基本结构：

```yaml
pose_sources:
  cup:
    topic: /detected_cup_pose
    max_age_sec: 1.0

defaults:
  orientation_xyzw: [0.008, 0.056, -0.066, 0.996]
  cartesian_eef_step: 0.005
  cartesian_jump_threshold: 0.0
  cartesian_min_fraction: 0.95

workflows:
  my_workflow:
    steps:
      - name: wait for station ready
        type: wait_for_signal
        topic: /workflow/external_signal
        signal_name: station_state
        code: 1
        active: true
        timeout_sec: 30.0

      - name: move above cup
        type: move_pose
        source: cup
        offset_xyz: [0.04, 0.0, 0.15]
        cartesian: false
```

## 支持的步骤

`wait_for_pose`：等待某个传感器发布目标位姿。

```yaml
- name: wait cup
  type: wait_for_pose
  source: cup
  timeout_sec: 5.0
```

`wait_for_signal`：等待统一外部信号，适合 RS485 状态机、GPIO/DIDO、PLC、按钮、工位信号。

```yaml
- name: wait object ready
  type: wait_for_signal
  topic: /workflow/external_signal
  source: rs485_process_state
  signal_name: station_state
  code: 1
  active: true
  timeout_sec: 30.0
```

兼容旧字段 `signal_source`，但新 YAML 建议统一写 `source`。

`wait_for_bool`：等待 `std_msgs/msg/Bool`。

```yaml
- name: wait bool ready
  type: wait_for_bool
  topic: /ready
  expected: true
  timeout_sec: 10.0
```

`move_pose`：移动到固定位置或传感器位姿加偏移。

```yaml
- name: move pre-grasp
  type: move_pose
  source: cup
  offset_xyz: [0.04, 0.0, 0.15]
  cartesian: false
```

固定位置：

```yaml
- name: move fixed point
  type: move_pose
  frame: base_link
  position: [0.30, 0.00, 0.25]
  orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
  cartesian: false
```

`move_relative`：从当前位置按 XYZ 偏移移动。

```yaml
- name: lift
  type: move_relative
  offset_xyz: [0.0, 0.0, 0.08]
  cartesian: true
```

`move_named`：MoveIt 命名姿态。

```yaml
- name: home
  type: move_named
  state: home
```

`move_joint`：关节角目标，单位 rad。

```yaml
- name: joint target
  type: move_joint
  positions: [0.0, 1.0, 1.1, 0.0, 0.0, 0.0]
```

`gripper`：夹爪开合。

```yaml
- name: open gripper
  type: gripper
  position: 0.05
  duration_sec: 2.0
```

`position` 会经过 `gripper_min_position` / `gripper_max_position` 检查。默认范围是 `0.0..0.05`，需要改夹爪开合范围时改 `launch/workflow_executor.launch.py`。

`sleep`：等待。

```yaml
- name: settle
  type: sleep
  seconds: 0.3
```

`collision_object` / `attach_object` / `detach_object`：MoveIt 场景物体。

## 后续加功能的方式

加一个新传感器时，优先让传感器节点输出以下两类之一：

- 目标位置：`geometry_msgs/msg/PoseStamped`，然后加入 `pose_sources`。
- 工艺/状态信号：`panthera_interfaces/msg/ExternalSignal`，然后在流程里用 `wait_for_signal`。

加一个新作业流程时，只在 YAML 的 `workflows` 下新增一个流程名和步骤列表，再调用：

```bash
ros2 service call /run_workflow panthera_interfaces/srv/RunWorkflow \
  "{workflow_name: 新流程名, dry_run: false}"
```
