# 光谱检测工作站点位微调说明

本文说明如何修改 `src/panthera_spectrometer_cell/config/spectrometer_cell.yaml`。

## 基本规则

- 所有 `xyz` 默认是 `base_link` 坐标系下的 `gripper_center` 目标点。
- 坐标单位是 m，不是 mm。比如 428.53 mm 要写成 `0.42853`。
- 姿态 `rpy` 单位是 rad。90 度是 `1.5708`，180 度是 `3.14159265`。
- 修改 YAML 后，在 HMI 点位页点击“重载生效”即可让运行节点重新读取配置；如果重载失败，再重启节点或重新执行一键启动脚本。
- 调点时建议每次只改 5 到 20 mm，也就是 `0.005` 到 `0.020`。

## 想改什么，改哪里

| 目标 | YAML 字段 |
| --- | --- |
| 出料口 1 取杯位置 | `named_poses.outlet_1_pick.xyz/rpy` |
| 出料口 1 放回位置 | `named_poses.outlet_1_return.xyz/rpy` |
| 出料口 2 取杯位置 | `named_poses.outlet_2_pick.xyz/rpy` |
| 出料口 2 放回位置 | `named_poses.outlet_2_return.xyz/rpy` |
| 出料口区域接近高度 | `motion.outlet_high_z` |
| 出料口夹取高度 | `motion.outlet_grip_z` |
| 取到杯子后抬高避障高度 | `motion.outlet_transfer_z` |
| 出料口前方绕障 Y | `motion.outlet_approach_y` |
| 出料口靠近杯子前的中间 Y | `motion.outlet_near_y` |
| 光谱仪基准点 | `named_poses.spectrometer_base.xyz/rpy` |
| 激光 150mm 对应的光谱仪基准 | `spectrometer_axis.axis_zero_laser_mm` |
| 激光变化换算方向和比例 | `spectrometer_axis.axis_scale_m_per_mm` |
| 光谱仪变化轴 | `spectrometer_axis.axis` |
| 光谱仪放杯单独补偿 | `spectrometer_axis.place_offset_xyz` |
| 光谱仪取杯单独补偿 | `spectrometer_axis.pick_offset_xyz` |
| 光谱仪前方接近偏移 | `motion.spectrometer_approach_x_offset` |
| 光谱仪区域高位过渡高度 | `motion.spectrometer_high_z` |
| 清理/倒料点 | `named_poses.clean_dump.xyz/rpy` |
| 清理区接近点 | `named_poses.clean_approach.xyz/rpy` |
| 清理区离开点 | `named_poses.clean_leave.xyz/rpy` |
| 倒料翻转角度 | `cleaning.pour_angle_rad` |
| 倒料摆动幅度和次数 | `cleaning.shake_angle_rad` / `cleaning.shake_count` |
| 机械臂回零/安全位 | `motion.safe_joint_pose` |

## 出料口动作路径

取杯时不是直接去杯子中心，而是走一组派生点：

```text
open gripper
-> outlets.OUTLET_N.pick_approach_pose 指向的 named pose
-> (pick.x, motion.outlet_approach_y, motion.outlet_high_z)
-> (pick.x, motion.outlet_approach_y, motion.outlet_grip_z)
-> (pick.x, motion.outlet_near_y, motion.outlet_grip_z)
-> (pick.x, pick.y, motion.outlet_grip_z)
-> close gripper
-> (pick.x, pick.y, motion.outlet_transfer_z)
```

因此：

- 左右位置不准，改 `named_poses.outlet_*_pick.xyz` 的 X/Y。
- 进入取料口前的停靠点不合适，改 `named_poses.outlet_*_pick_approach.xyz/rpy`。
- 夹得太高或太低，优先改 `motion.outlet_grip_z`。
- 拿到杯子后容易被挡住，优先改高 `motion.outlet_transfer_z`。
- 进出出料口路径容易撞挡板，调整 `motion.outlet_approach_y`、`motion.outlet_near_y`、`motion.outlet_high_z`。

放回杯子也使用同一组 `motion.outlet_*` 参数，只是目标点来自 `outlet_*_return`，并且会先走 `outlet_*_return_approach`。

## 光谱仪动作路径

光谱仪目标点由下面公式计算：

```text
target = named_poses.spectrometer_base
target += place_offset_xyz 或 pick_offset_xyz
target[axis] += (laser_mm - axis_zero_laser_mm) * axis_scale_m_per_mm
```

例子：

- `axis: y`
- `axis_zero_laser_mm: 150.0`
- `axis_scale_m_per_mm: 0.001`
- 激光读数 `160mm`

则目标 Y 会在 `spectrometer_base.y` 基础上增加 `0.010m`。

如果实际运动方向反了，把 `axis_scale_m_per_mm` 改成 `-0.001`。

## 清理倒料动作路径

清理时的路径大致是：

```text
-> cleaning.approach_pose 指向的 named pose
-> (clean_dump.x, motion.clean_approach_y, motion.clean_high_z)
-> (clean_dump.x, motion.clean_approach_y, motion.clean_pre_z)
-> (clean_dump.x, clean_dump.y, motion.clean_ready_z)
-> clean_dump
-> wrist pour and shake
-> clean ready
-> clean retreat
-> cleaning.leave_pose 指向的 named pose
```

因此：

- 倒料口位置不准，改 `named_poses.clean_dump.xyz/rpy`。
- 进入清理区路径不合适，改 `motion.clean_approach_y`、`motion.clean_high_z`、`motion.clean_pre_z`。
- 倒料不彻底，改 `cleaning.pour_angle_rad`、`cleaning.shake_angle_rad`、`cleaning.shake_count`。

## 推荐调试顺序

1. 先单独调 `outlet_1_pick` 和 `outlet_2_pick`。
2. 再调 `outlet_1_return` 和 `outlet_2_return`。
3. 调 `spectrometer_base`，并确认激光 150mm 时目标点正确。
4. 移动光谱仪后确认 `axis_scale_m_per_mm` 正负方向正确。
5. 最后调 `clean_dump` 和倒料参数。
6. 单点都通过后再跑完整流程。

## 常见问题

### 改了 YAML 没生效

先确认 HMI 点位页已经点击“重载生效”，且状态机处于空闲、等待、暂停、错误或急停等允许重载的状态。若仍未生效，再重启节点或重新执行一键启动脚本。

### 规划失败

先把过渡高度调高，例如：

- `motion.outlet_transfer_z`
- `motion.spectrometer_high_z`
- `motion.clean_high_z`

再检查姿态 `rpy` 是否和实际夹爪方向一致。

### 激光测距导致光谱仪目标点偏反方向

只改 `spectrometer_axis.axis_scale_m_per_mm` 的正负号，不要同时乱改 `spectrometer_base`。

### 出料口夹取时位置对但姿态不对

改 `named_poses.outlet_*_pick.rpy` 和 `named_poses.outlet_*_return.rpy`。当前出料口默认 yaw 为 `-1.5708`。
