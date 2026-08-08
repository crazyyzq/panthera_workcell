# MIT MoveIt Integration Record

> 历史兼容文件名。MIT 接口已经集成，但 **v1.0.0 生产默认模式是
> `position_velocity`**。本文件只记录实现边界，不是切换生产控制模式的操作指令。

## 当前实现

- `panthera_hardware` 支持显式的 `mit_gravity_compensation` 模式。
- MIT 模式通过厂商 SDK 的位置、速度、力矩、Kp、Kd 命令发送，并使用 Pinocchio
  计算重力前馈。
- `hardware_moveit_rviz_mit.launch.py` 是独立的 MIT 调试入口。
- 固定轨迹生产入口和 `scripts/start_workcell.sh` 默认使用 `position_velocity`；不会
  因安装了 MIT 支持而自动切换。
- 夹爪继续使用独立位置命令，不走六轴 MIT 增益。
- 生产仍由单一 Motion Server 执行缓存轨迹，不为每个动作在线规划。

当前 MIT 调试入口默认参数与代码一致：

```text
Kp = [75, 105, 135, 135, 75, 75]
Kd = [5.5, 5.5, 5.5, 5.5, 5.5, 5.5]
gravity_scale = [0, 1.04, 1.10, 1.52, 0, 0]
```

这些值不是生产位置速度模式的 PD 参数，也不得在未做实机验收时写成“稳定生产参数”。

## 使用边界

只有明确的 MIT 调试任务才允许设置：

```bash
CONTROL_MODE=mit_gravity_compensation scripts/start_workcell.sh
```

切换前必须停止工作站，确保只有一个 `ros2_control_node` 和一个轨迹控制器。禁止在
生产运行中热切换控制模式。HMI 显示的模式来自本次启动配置，不从厂商状态反馈帧的
类型字节推断。

## 保留的保护

- 增益、重力比例和负载参数必须是完整、有限的六轴向量。
- URDF/动力学模型加载失败时 MIT 初始化失败，不发送未补偿命令。
- 重力力矩按关节配置限幅；无效 SDK 状态和不可用电机不下发六轴命令。
- 任何调试都不得启动第二个 MoveGroup、旧 workflow executor 或 pose tuner 与生产
  Motion Server 争用控制器。

## 历史来源

- 初始工作站基线：`23500b7`（`validated-motion-20260718`）。
- 上游参考：`HighTorque-Robotics/Panthera-HT_ROS2` `humble` 分支，提交
  `3815fce1a2b805e39544f66368ed46a0089a5cb0`。
- 厂商早期建议的统一 `Kp=60`、`Kd=5` 仅是最初调试起点，已不作为当前启动默认值。

MIT 的实机验收、温度/电流和长周期稳定性必须单独记录；软件构建通过不代表硬件验收。
