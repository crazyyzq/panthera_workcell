# panthera_spectrometer_cell

`panthera_spectrometer_cell` 是光谱检测流水线的长期运行 C++ 状态机包。它用于持续等待出料、抓取杯子、根据激光测量的光谱仪位置放杯、等待检测、重新测量后取杯、清理杯子，并把杯子放回原出料口。

当前版本默认运行模拟接口，不会驱动真实机械臂。真实硬件接入点已经预留在 `RobotActions`、`Sensors`、`Spectrometer` 内部。

## 启动

```bash
cd /home/b1/panthera_workcell_ws
source install/setup.bash
ros2 launch panthera_spectrometer_cell spectrometer_cell.launch.py simulation:=true
```

## 状态主题

```bash
ros2 topic echo /spectrometer_cell/state
ros2 topic echo /spectrometer_cell/context
ros2 topic echo /spectrometer_cell/error
```

`/spectrometer_cell/context` 会发布当前状态、cycle id、当前杯子来源、放杯前激光位置、取杯前激光位置和错误信息。

## 模拟服务

手动模拟 1 号出料口完成：

```bash
ros2 service call /spectrometer_cell/simulate_outlet_1_done std_srvs/srv/Trigger "{}"
```

手动模拟 2 号出料口完成：

```bash
ros2 service call /spectrometer_cell/simulate_outlet_2_done std_srvs/srv/Trigger "{}"
```

模拟急停：

```bash
ros2 service call /spectrometer_cell/simulate_estop std_srvs/srv/Trigger "{}"
```

清除模拟急停：

```bash
ros2 service call /spectrometer_cell/clear_estop std_srvs/srv/Trigger "{}"
```

人工复位：

```bash
ros2 service call /spectrometer_cell/request_reset std_srvs/srv/Trigger "{}"
```

## 可调配置

主配置文件：

```text
config/spectrometer_cell.yaml
```

常改项：

| 目标 | 配置 |
| --- | --- |
| 状态机 tick 频率 | `loop.tick_rate_hz` |
| 等待出料提示间隔 | `loop.wait_discharge_soft_timeout_sec` |
| 光谱仪检测超时 | `loop.detection_timeout_sec` |
| 夹爪开合位置 | `gripper.open_position` / `gripper.close_position` |
| 两个出料口取杯/放回点位 | `outlets` + `named_poses` |
| 光谱仪轴向和激光换算 | `spectrometer_axis` |
| 清理位置和抖动次数 | `cleaning` |
| 障碍物尺寸和位置 | `collision_objects` |
| 模拟失败率 | `simulation.random_failure_rate` |
| 动作超时 | `loop.action_timeout_sec` |
| 激光测距超时 | `loop.sensor_timeout_sec` |
| 光谱仪启动超时 | `loop.spectrometer_start_timeout_sec` |

## 代码分层

| 文件 | 作用 |
| --- | --- |
| `Types.h` | 状态、出料口、动作结果、测量结果、任务上下文。 |
| `Config.h/.cpp` | YAML 配置结构和解析校验。 |
| `StateMachine.h/.cpp` | 长期循环状态机，只做条件判断、动作调用和状态切换。 |
| `RobotActions.h/.cpp` | 机械臂动作封装。后续 MoveIt、夹爪控制器、PlanningScene 在这里接入。 |
| `Sensors.h/.cpp` | 出料信号、激光、复位、急停等输入封装。后续 RS485/IO 在这里接入。 |
| `Spectrometer.h/.cpp` | 光谱仪检测开始和完成状态封装。 |
| `main.cpp` | ROS2 节点入口，只负责加载配置、创建对象、启动 tick 定时器。 |

## 关键约束

- 主程序长期运行，不执行一轮后退出。
- `WAIT_DISCHARGE` 只接收出料完成信号并维护任务队列，不直接进入取杯动作。
- `SELECT_TASK` 从 `pending_outlets` 中取任务并创建本轮 `CycleContext`。
- 两个出料口有独立 `OutletStatus`，同一个出料口杯子未放回前不会重复入队。
- 当前任务执行中收到另一个出料口信号，只入队，不覆盖当前任务上下文。
- `MEASURE_SPECTROMETER_BEFORE_PLACE` 和 `MEASURE_SPECTROMETER_BEFORE_PICK` 会分别读取激光。
- 取杯前必须使用 `spectrometer_position_before_pick_valid` 判断重新测距是否有效，禁止用放杯前测距值替代。
- `RETURN_CUP` 使用 `CycleContext.outletId`，杯子必须回原出料口。
- `PLACE_TO_SPECTROMETER` 和 `START_DETECTION` 已拆开，放杯成功后才启动光谱仪。
- 任意动作失败进入 `ERROR`，不会继续执行下一步。
- `ERROR` 保留上下文，不清空杯子位置、出料口状态和 active action 信息。
- `ESTOP` 优先级最高，急停下不执行任何机械臂动作，必须释放急停并复位后才能回 `IDLE`。
- `RESET` 后默认回 `IDLE`，不会自动恢复自动循环；需要 `/spectrometer_cell/auto_mode` 再进入 `WAIT_DISCHARGE`。
- 每个机械臂动作都会生成 `active_command_id` 和 `active_action_name`，动作反馈只接受当前匹配的 command。
- 每次 tick 后执行 invariant 检查，发现上下文自相矛盾会进入 `ERROR`。
- 点位和障碍物都在 YAML 中改，不需要改 C++ 源码。

## 当前状态集合

```text
INIT
IDLE
WAIT_DISCHARGE
SELECT_TASK
PICK_FROM_OUTLET
MEASURE_SPECTROMETER_BEFORE_PLACE
PLACE_TO_SPECTROMETER
START_DETECTION
WAIT_DETECTION_DONE
MEASURE_SPECTROMETER_BEFORE_PICK
PICK_FROM_SPECTROMETER
CLEAN_CUP
RETURN_CUP
COMPLETE_CYCLE
PAUSED
ERROR
ESTOP
RESET
```

正常循环：

```text
INIT -> IDLE -> WAIT_DISCHARGE -> SELECT_TASK
-> PICK_FROM_OUTLET
-> MEASURE_SPECTROMETER_BEFORE_PLACE
-> PLACE_TO_SPECTROMETER
-> START_DETECTION
-> WAIT_DETECTION_DONE
-> MEASURE_SPECTROMETER_BEFORE_PICK
-> PICK_FROM_SPECTROMETER
-> CLEAN_CUP
-> RETURN_CUP
-> COMPLETE_CYCLE
-> SELECT_TASK 或 WAIT_DISCHARGE
```

HMI/调试服务语义：

| 服务 | 语义 |
| --- | --- |
| `/spectrometer_cell/manual_mode` | 请求暂停自动流程，能立即暂停的状态进入 `PAUSED`，动作中状态会延后到动作完成后暂停。 |
| `/spectrometer_cell/auto_mode` | 启动或恢复自动流程。`IDLE` 下进入 `WAIT_DISCHARGE`，`PAUSED` 下回到暂停前状态。 |
| `/spectrometer_cell/step_once` | 单步请求，不会简单枚举 +1，也不会绕过测距、检测完成、清理和回原出料口等必要条件。 |
| `/spectrometer_cell/request_reset` | 复位请求。`ERROR/ESTOP` 必须经过 `RESET`，普通中间状态也会先退出自动并复位。 |

## 后续接真实硬件

推荐顺序：

1. 保持 `StateMachine` 不动。
2. 在 `Sensors::readSpectrometerPosition()` 接 `/laser_distance_node/read_once` 或 `/sensors/laser/distance`。
3. 在 `Sensors::readDischargeDone()` 接 RS485/IO 出料完成信号。
4. 在 `Spectrometer::pollDetectionDone()` 接光谱仪完成信号。
5. 在 `RobotActions` 内接 MoveIt 和夹爪控制器。
6. 在 `RobotActions::applyCollisionObjects()` 把 YAML 障碍物加入 MoveIt PlanningScene。

## 已验证

| 日志目录 | 结果 |
| --- | --- |
| `/home/b1/panthera_workcell_ws/validation_logs/20260614_125442_rebuild_spectrometer_cell_cycle_reset_fix` | `panthera_spectrometer_cell` 构建通过。 |
| `/home/b1/panthera_workcell_ws/validation_logs/20260614_125328_validate_spectrometer_cell_sim` | 模拟模式完成至少一轮完整循环：取杯、放杯前测激光、放到光谱仪、等待检测、取杯前重新测激光、清理、回原出料口。 |
| `/home/b1/panthera_workcell_ws/validation_logs/20260614_125453_validate_spectrometer_cell_error_reset` | 模拟急停触发 `ERROR`，调用 `robot.stop()`，人工复位后回到 `WAIT_DISCHARGE`。 |
| `/home/b1/panthera_workcell_ws/validation_logs/20260614_131202_full_test_summary` | 全量模拟测试汇总：构建、自动循环、手动出料、急停复位、失败进入 ERROR 均通过。 |
| `/home/b1/panthera_workcell_ws/validation_logs/20260614_194741_state_machine_refactor_rclpy_validation3` | 重构后隔离 ROS domain 模拟验证通过：两个出料口连续触发排队并完成两轮；暂停/恢复、ESTOP、清除急停后 RESET 均通过。 |
| `/home/b1/panthera_workcell_ws/validation_logs/20260614_194855_state_machine_refactor_full_build` | 重构后全工作空间 10 个包构建通过。 |
