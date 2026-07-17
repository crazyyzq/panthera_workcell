# 固定轨迹目录与验收手册

## 当前状态

`panthera_motion` 已实现轨迹目录解析、引用校验、确定性 IK 采样、关节/Cartesian
段编译、垂直约束、自碰撞与限位检查、时间参数化、缓存、起点保护、速度缩放、
取消/停止和直接 `FollowJointTrajectory` 执行。

当前正式目录只启用了两个已有且明确记录的关节点与两条往返调试路线。旧光谱工作流
尚未切换为生产默认：其 TCP、IK 分支和毛刷异常姿态必须先经现场确认和分级验收。
因此本阶段禁止把旧状态机与新 Motion Server 同时用于真实运动。

## 无硬件检查

```bash
cd /home/b1/panthera_workcell_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch panthera_motion motion_server.launch.py
ros2 service call /motion/list_routes std_srvs/srv/Trigger '{}'
ros2 action send_goal /motion/execute \
  panthera_interfaces/action/ExecuteMotion \
  '{route_name: home_to_safe_center, speed_scale: 0.2, dry_run: true}' \
  --feedback
```

`dry_run: true` 不读取控制器、不要求 joint state，也不会发送机械臂轨迹。

## 目录结构

主数据源是 `src/panthera_motion/config/motion_catalog.yaml`。

- 点位可含 `joints`、`pose`，或两者都含。Cartesian 目标必须含 `pose`。
- pose-only 点位应指定已确认的 `ik_seed`，避免切换 IK 分支。
- 路线必须有固定 `start`，然后按顺序引用 `joint` 或 `linear` 段。
- 抓取/放置下降与抬升段使用 `linear`，并声明 `vertical_axis: z`、
  `keep_orientation: true` 和允许的最大横向误差。
- 生产速度写在路线中；运行倍率只能把已编译轨迹减速，不能超过编译速度。

## HMI 点位与路线维护

HMI 的“点位”页顶部是固定轨迹主目录，底部是迁移期旧状态机参数。

1. 启动 Motion Server；未启动时 HMI 会安全拒绝保存，因为无法做完整编译。
2. 新增点位后录入六关节值，或录入 `pose.xyz`、`pose.rpy` 和明确的 `ik_seed`。
3. 删除点位前查看引用；被路线引用的点位不能删除。
4. 路线编辑器使用 JSON 表示 YAML 中对应的路线对象。
5. “校验并保存”会检查引用，然后让 Motion Server 编译全部启用路线。
6. 保存采用同目录临时文件、`fsync` 和原子替换。编译失败时自动恢复旧文件，
   再次编译旧缓存。

## 真实机械臂首次验收

只有在现场确认工位无人、硬急停可用、控制器健康、轨迹起点匹配后，才使用：

```bash
ros2 launch panthera_motion fixed_motion_bringup.launch.py \
  default_speed_scale:=0.20
```

依次完成：控制器只读检查、dry-run、安全关节单段、垂直 Cartesian 单段、无物料
空跑、空杯分段、单杯整轮、多轮与故障注入。任何一级失败都不得进入下一级。

## 固定与传感器模式

`spectrometer_cell.yaml` 当前为：

```yaml
positioning:
  mode: fixed
  fixed_axis_position_mm: 150.0
```

固定模式不创建激光接口，也不等待激光。未来切换 `sensor_offset` 前必须确认修正轴、
零点、比例、方向、量程和异常处置；传感器只提供受限局部偏移，不能绕过轨迹编译。

## 尚未解除的现场阻塞

- `clean_brush.rpy[0] = 30.0` 是 30 度还是 30 弧度。
- 固定光谱仪最终 TCP，以及未来激光修正轴是 X 还是 Y。
- 硬急停、两个出料信号和光谱检测完成信号的实际接线/ROS 接口。
- 生产速度、加速度、目标节拍和是否允许空杯低速验收。

这些信息确认前，不迁移或执行可能接触杯子、光谱仪、毛刷和工装的正式路线。
