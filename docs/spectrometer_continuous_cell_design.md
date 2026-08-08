# 光谱检测流水线状态机实施记录

> 历史兼容文件名。早期设计已实现并经过多轮重构，旧伪代码和未实施建议已删除；
> 详细现状以 [`panthera_spectrometer_cell` README](../src/panthera_spectrometer_cell/README.md)
> 和 [外部指令接口](EXTERNAL_COMMAND_API.md) 为准。

## 当前职责

`panthera_spectrometer_cell` 是长期在线的 C++ 业务状态机。它负责：

- 接收两个出料口任务并保持原出料口上下文；
- 在放杯和取杯前分别获取稳定激光值；
- 放杯、等待扫描完成、重新取杯、倒料、毛刷清洁和放回原出料口；
- 暂停、急停、人工复位、夹爪恢复和回 Home 协调；
- 发布状态、上下文、活动命令和可读错误。

它不规划轨迹，也不直接维护另一份点位。所有六轴动作提交给唯一 Motion Server，
点位/路线来自 `motion_catalog.yaml`。

## 正常状态流

```text
INIT -> IDLE -> WAIT_DISCHARGE -> SELECT_TASK
-> PICK_FROM_OUTLET
-> MEASURE_SPECTROMETER_BEFORE_PLACE
-> PLACE_TO_SPECTROMETER -> START_DETECTION -> WAIT_DETECTION_DONE
-> MEASURE_SPECTROMETER_BEFORE_PICK
-> PICK_FROM_SPECTROMETER -> CLEAN_CUP -> RETURN_CUP
-> COMPLETE_CYCLE -> SELECT_TASK 或 WAIT_DISCHARGE
```

`PAUSED`、`ERROR`、`ESTOP` 和 `RESET` 是受控旁路状态。活动动作通过 command id
匹配异步结果；延迟到动作边界的暂停不会重复注入事件或跳过下一个业务状态。

## 设备适配

- 激光：`panthera_rs485` 发布距离；量程、滤波、标准品校准和扫描完成判断由当前
  配置执行。未接传感器可使用固定基准，一旦本轮采用传感器则掉线时安全等待恢复。
- 光谱仪：自动扫描阶段判断与外部 `DETECTION_DONE` 并存。
- 毛刷：AQMD6030NS-A3 通过 `/dev/ttyS8`、Modbus 站号 `2` 控制；先套杯再启动。
- 夹爪：独立位置控制并持续保持目标；杯子阻挡目标不等于失败，不按位置误差复位。
- 相机：HMI 可显示 RGB/深度；相机缺失或恢复失败不改变生产 READY。
- 外部系统：经 HMI HTTP API 使用带 `request_id` 的幂等指令。

## 配置边界

- 状态机、工艺时间、激光和毛刷：
  `src/panthera_spectrometer_cell/config/spectrometer_cell.yaml`
- 点位和轨迹：`src/panthera_motion/config/motion_catalog.yaml`
- 激光串口：`src/panthera_rs485/config/rs485_devices.yaml`

HMI 工艺时间和点位保存均使用原子写入、后端校验、热重载与失败回滚。禁止恢复旧的
硬编码点位、同步 MoveIt 动作或多个机械臂命令源。

## 验证

纯软件验收覆盖配置解析、事件优先级、任务排队、暂停/恢复、急停/复位、激光超时、
外部幂等指令、毛刷失败路径和完整模拟循环。涉及点位、控制器或硬件行为的变更必须
另行完成分级实机验收，不能用模拟测试替代。
