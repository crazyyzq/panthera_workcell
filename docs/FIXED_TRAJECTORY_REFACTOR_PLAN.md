# Panthera 固定轨迹系统实施记录

> 历史兼容文件名。原实施计划已经完成，旧阶段、点位数量和待办可从 Git 历史追溯。
> 当前操作以 [固定轨迹验收手册](FIXED_MOTION_COMMISSIONING.md) 和
> [AGENTS.md](../AGENTS.md) 为准。

## 已落地架构

生产运动采用“保存/启动时编译，运行时直接执行”：

```text
HMI / 光谱状态机
        |
        v
panthera_motion Motion Server（唯一运动所有者）
  - 解析 motion_catalog.yaml
  - 确定性 IK 与 Cartesian 采样
  - 限位、自碰撞、连续性和工艺约束校验
  - 时间参数化、缓存、速度缩放、取消和恢复
        |
        v
/arm_controller/follow_joint_trajectory
```

正常生产不在每个动作前调用 MoveIt 在线规划，不启动旧 workflow executor 或
pose tuner。MoveIt 模型仍用于目录编译时的运动学、碰撞和约束检查。

## 唯一数据源

固定点位和路线只维护：

```text
src/panthera_motion/config/motion_catalog.yaml
```

- 点位可以包含关节值、TCP pose 或二者；pose-only 点使用明确的 `ik_seed`。
- 路线由固定起点和有序 `joint` / `linear` 段组成。
- 抓取、放置和毛刷进退的直线段可声明固定姿态、垂直轴和横向误差要求。
- HMI 只展示可示教工艺点；关联 hover/pregrasp/return 由 follower 规则同步。
- 保存先写临时文件并 `fsync`，编译全部启用路线后原子替换；失败回滚文件和缓存。

## 运行约束

- 只有一个节点拥有六轴轨迹控制器。
- 执行前检查新鲜编码器、路线起点和静止状态；明确回 Home 使用从实测关节生成的
  恢复轨迹，不受“离 Home 太远”门槛阻止。
- 运行速度倍率只允许在已编译上限内重定时，不改变点位或 IK 分支。
- 生产默认控制模式为 `position_velocity`；MIT 是显式调试选项，不是另一套轨迹。
- 激光修正只改变光谱仪 X 轴目标，仍经过同一编译、接近和执行链。
- 点位示教、生产动作、恢复动作互斥，任何时候不得出现第二个轨迹发布者。

## 现行集成

- `panthera_spectrometer_cell` 只编排业务状态和提交具名路线。
- `panthera_web_hmi` 通过后端服务进入独占调试、点动、直接坐标和热保存。
- `scripts/start_workcell.sh` 启动硬件、控制器、Motion Server、状态机、激光适配器和
  非阻塞相机恢复；常驻 HMI 独立运行。
- `scripts/stop_workcell.sh` 完成本轮安全条件、回全零 Home、停毛刷并清理生产进程；
  无法确认安全时保持上电而不是直接失能。

## 验证要求

软件变更至少执行：YAML/脚本/launch 语法检查、相关单元测试、全工作空间构建和
`colcon test-result --verbose`。轨迹或点位变更另需在 `beta` 按 dry-run、分段、空载、
空杯、单杯整轮和重复循环逐级验收。软件构建通过不代表新的实机轨迹已经验收。

真实硬急停、现场出料/完成信号接线等外部电气闭环，必须按现场硬件单独验收；HTTP
外部指令接口见 [EXTERNAL_COMMAND_API.md](EXTERNAL_COMMAND_API.md)。
