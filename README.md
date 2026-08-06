# Panthera Workcell

Panthera 六轴机械臂光谱检测与杯具清洗工作站，基于 ROS 2 Humble。工程包含位置速度
控制、固定轨迹生产流程、光谱仪激光偏置、RS485 毛刷控制、点位示教和常驻 Web HMI。

## 分支

- `main`：已验证的稳定生产版本。
- `beta`：下一版现场验证版本；验证通过后再合入 `main`。

生产现场不要直接在 `main` 上试验性修改，也不要将 `build/`、`install/`、`log/`、
`validation_logs/` 或 `.runtime/` 提交到仓库。

## 操作

Ubuntu 启动后 HMI 常驻运行。在工控机浏览器打开：

```text
http://127.0.0.1:8080
```

通过界面的“启动工作站”“安全停止”“安全重启”管理生产服务。HMI 不随工作站停止，
因此生产进程异常时仍可操作恢复。

命令行备用操作：

```bash
cd /home/b1/panthera_workcell_ws
scripts/start_workcell.sh
scripts/stop_workcell.sh
```

默认控制模式为 `position_velocity`。正常生产不需要 MoveIt 在线规划；动作由已验证的
固定轨迹和笛卡尔末端段组成。点位应通过 HMI 示教、保存并热重载。

## 安全约定

- 正常停止必须先回到编码器确认的 Home，再失能。
- 未知姿态、持杯或活动动作期间，不得直接掉电或强制失能。
- `scripts/stop_workcell.sh --force` 只用于机械上已经确认安全、且明确要求原地停机的维护场景。
- 调试前确认周围无人、杯具状态正确，并只移动已确认安全的轴或点位。
- 故障恢复优先保留驱动使能和当前保持力；不要用额外的位置、速度或力门限制造误停机。

## 文档入口

- [工作站脚本](scripts/README.md)
- [操作与备份手册](docs/OPERATION_AND_BACKUP_MANUAL.md)
- [点位调试说明](docs/SPECTROMETER_CELL_POINT_TUNING.md)
- [外部指令接口](docs/EXTERNAL_COMMAND_API.md)
- [固定轨迹调试记录](docs/FIXED_MOTION_COMMISSIONING.md)
- [生产状态机](src/panthera_spectrometer_cell/README.md)
- [Web HMI](src/panthera_web_hmi/README.md)

长期有效的工程约束、现场标定值和维护规则记录在 [AGENTS.md](AGENTS.md)。

## 构建与测试

```bash
cd /home/b1/panthera_workcell_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

涉及硬件的测试必须区分“纯软件测试”“夹爪测试”和“机械臂实机运动”，并在提交说明中
记录实际执行过的范围。
