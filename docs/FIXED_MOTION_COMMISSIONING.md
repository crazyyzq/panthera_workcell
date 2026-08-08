# 固定轨迹目录与验收手册

## 当前状态

`panthera_motion` 已实现轨迹目录解析、引用校验、确定性 IK 采样、关节/Cartesian
段编译、垂直约束、自碰撞与限位检查、时间参数化、缓存、起点保护、速度缩放、
取消/停止和直接 `FollowJointTrajectory` 执行。

当前目录包含日常工艺点、关联接近点、安全/恢复点，以及生产、调试和恢复路线。
所有启用路线保存时都执行 IK、限位、自碰撞、笛卡尔约束和动态约束编译。正式状态机
默认后端为 `fixed_cache`；旧光谱工作流仅作受控回退。

兼容状态机单循环的普通空间移动调用已从 27 次降为 15 次（不计倒料关节和毛刷内部
动作）。固定路线把跨工位移动与垂直进退编译成一条连续轨迹，中间采样点不会逐点停顿。
真实模式下状态机只向 Motion Server action 提交路线，不创建 MoveGroup。禁止同时启动
旧 workflow executor、pose tuner 或任何其他机械臂轨迹发布者。

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

HMI 的“点位”页是固定轨迹主目录。默认列表只显示
`outlet_1_grasp`、`outlet_2_grasp`、`spectrometer_place`、`spectrometer_pick`、
`spectrometer_wait`、`clean_dump` 和 `brush_center`。位置按 mm、姿态按度编辑；勾选“显示安全/维护点”才会
看到 hover、恢复点和 IK 种子。

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
ros2 launch panthera_motion fixed_spectrometer_cell.launch.py \
  default_speed_scale:=0.20
```

启动前机械臂必须实际位于 `motion.fixed_start_point`；当前生产 Home 是厂商绝对零点
`home_near = [0, 0, 0, 0, 0, 0]`。配置名称不能替代实际编码器位置，
Motion Server 仍会以 `0.05 rad` 起点容差逐关节拒绝错误起点。

依次完成：控制器只读检查、dry-run、安全关节单段、垂直 Cartesian 单段、无物料
空跑、空杯分段、单杯整轮、多轮与故障注入。任何一级失败都不得进入下一级。

## 固定与传感器模式

`spectrometer_cell.yaml` 当前为：

```yaml
positioning:
  mode: sensor_optional
  fixed_axis_position_mm: <HMI 当前校准值>
spectrometer_axis:
  axis: x
  axis_zero_laser_mm: <HMI 当前校准值>
```

标准品机械 X 为 `162 mm`；激光零点以 HMI 在标准品位置执行校准后写入的值为准。
`sensor_optional` 在本次运行从未见过有效激光时使用固定点；
一旦见过有效激光，掉线后会等待恢复。放杯、取杯各自采样并只修正光谱仪 X 轴，
不能绕过轨迹编译和安全接近段。

## v1.0.0 固化路线

清洗返程为：沿杯口轴退出 → 保持倒料姿态垂直抬到 `z=0.45 m` → 高位回正 →
直接跨区返回本轮原出料口，删除低位回倒料点的绕行。倒料主翻腕使用 70%/50%，
摇料使用 60%/45%。后续任何点位或速度修改都必须先在 `beta` 做分段净空验证，
再进行带杯完整循环验收；本文档不把本次纯软件发布检查等同于新的实机验收。
