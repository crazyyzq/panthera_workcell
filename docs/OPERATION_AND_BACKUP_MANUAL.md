# Panthera 光谱检测工作站操作与备份手册

本文档记录当前项目的日常启动、HMI 操作、相机恢复、速度调整、停机回零和源码文档备份流程。适用工作空间：

```bash
/home/b1/panthera_workcell_ws
```

Windows 共享路径：

```text
\\192.168.137.186\b1s_share\panthera_workcell_ws
```

HMI 地址：

```text
http://192.168.137.186:8080
```

## 1. 一键启动

HMI 随 Ubuntu 开机自动启动，工作站未启动时也可一直访问。日常操作打开 HMI，
在“工作站服务”区域点击“启动工作站”，确认现场安全即可。页面会显示启动中、
启动完成或失败原因，并阻止重复点击。

工控机桌面的“机械臂工作站 HMI”快捷方式固定打开
`http://127.0.0.1:8080`；局域网其他电脑使用本机实际 IP 加 `:8080`。

终端脚本保留为维护备用：

```bash
cd /home/b1/panthera_workcell_ws
scripts/start_workcell.sh
```

该脚本会启动：

- 机械臂硬件接口和 `ros2_control`
- MIT 重力补偿硬件模式（Kp `75/105/135/135/75/75`、Kd `5.5`）
- 固定轨迹 Motion Server（生产运行不实时规划）
- C++ 光谱检测长期状态机
- 激光传感器适配器（启动时未连接则使用配置中的固定参考位置）
- Gemini305 相机自动恢复 watchdog（相机未连接不阻塞工作站 READY）

Web HMI 由独立的 `panthera-hmi.service` 常驻运行，不属于生产启停进程组。
生产一键启动不会启动旧 MoveIt workflow 或点位调试节点，确保只有
Motion Server 一个机械臂命令源。脚本只有在控制器 active、Motion Server 静止、
编码器位于原 Home、HMI 服务全部就绪后才报告成功；默认工作速度为 100%。
启动前会清理可能卡在 `!rclpy.ok()` 的旧 ROS CLI 守护进程；温和退出失败时只强制
结束 CLI 守护进程和残留 CLI 客户端，不会结束控制器或机械臂硬件节点。

启动日志保存到：

```text
validation_logs/<时间>_start_workcell
```

## 2. 一键关闭

日常操作在 HMI 点击“安全停止”。它会等待安全状态、回到 Home、停止毛刷并清理
生产进程，HMI 页面不会关闭。需要完整重连硬件和服务时点击“安全重启”。

终端脚本保留为维护备用：

```bash
cd /home/b1/panthera_workcell_ws
scripts/stop_workcell.sh
```

该脚本只在流程已结束、杯子已释放、机械臂静止且编码器确认位于原 Home 时关闭：

1. 暂停自动状态机，阻止新任务进入
2. 明确停止毛刷并取消残留运动目标
3. 先终止一键启动拥有的整个进程组
4. 清理生产链及旧链残留进程，并确认常驻 HMI 继续在线

如果流程仍在运行，脚本会等待本轮正常完成；超时或无法确认 Home 时保持硬件上电，
拒绝强行失能。

只在维护或紧急清进程时使用不回零版本：

```bash
scripts/stop_workcell.sh --force
```

该参数会跳过流程和 Home 保护，只能在人工确认机械安全的维护/紧急场景使用。

## 3. HMI 常用功能

HMI 页面主要区域：

- 工作站服务：启动、安全停止和安全重启；操作期间显示进度并禁止重复操作。
- 状态机状态：显示 `WAIT_DISCHARGE`、`PICK_FROM_OUTLET`、`ERROR`、`ESTOP` 等状态。
- 激光测距：显示当前 RS485 激光距离，单位 mm。
- 相机图像：支持 RGB 和深度图切换。
- 夹取中心实时位姿：显示 `base_link -> gripper_center` 的坐标和四元数。
- 流程按钮：触发出料完成、检测完成、回安全位、手动调试流程。
- 点位调试：先点击“进入调试（不移动）”；夹爪、毛刷和工艺时间无需前往点位即可操作，需要示教时再点击“前往所选点位”。
- 动作速度：运行时速度比例，范围 `20%~120%`，默认 `100%`。
- 相机恢复：启动时自动恢复；运行中 RGB 连续 3 次失效会自动重启驱动。人工按钮保留为维护备用。

## 4. 动作速度设置

HMI 速度滑块用于临时调整运行时动作速度：

- 范围：`20%~120%`
- 调试、靠近障碍物或首次验证点位时建议降低速度
- 调整后点击 HMI 上的“应用速度”

ROS2 service：

```bash
ros2 service call /spectrometer_cell/set_speed_scale panthera_interfaces/srv/SetSpeedScale "{scale: 1.0}"
```

参数 `scale` 范围：

```text
0.20 ~ 1.20
```

## 5. 相机重启

一键启动会运行 `auto_recover_camera.sh`。它每 5 秒检查 HMI 的 RGB 帧龄，连续
3 次失效后调用相机重启；失败重试有界，之后等待 300 秒再试，不影响机械臂和
状态机运行。工作站安全停止会同时清理 watchdog。

HMI 上点击“重启相机”等价于：

```bash
cd ~/panthera_workcell_ws
scripts/restart_camera.sh
```

该脚本只重启 Gemini305 相机驱动，不重启机械臂和状态机。相机重启日志保存到：

```text
validation_logs/<时间>_restart_camera
```

验证相机：

```bash
source /opt/ros/humble/setup.bash
source ~/panthera_workcell_ws/install/setup.bash

ros2 topic hz /camera/color/image_raw/compressed
ros2 topic hz /camera/depth/image_raw
curl http://127.0.0.1:8080/api/status | python3 -m json.tool
```

当前验证结果：

- RGB 压缩流约 `28~30 FPS`
- 深度 raw 输入为 `1280x800`
- HMI 深度图会缩放后显示，降低浏览器和网络负载

## 6. 回零/回安全位

如果系统正在运行，手动让机械臂回安全位：

```bash
source /opt/ros/humble/setup.bash
source ~/panthera_workcell_ws/install/setup.bash

ros2 service call /run_workflow panthera_interfaces/srv/RunWorkflow \
  "{workflow_name: 'arm_home', dry_run: false}"
```

注意：当前 `arm_home` 是项目定义的安全位/回零工作流，不是重新标定电机零点。

查看执行是否完成：

```bash
ros2 topic echo /workflow/status --once
```

或者在 HMI 页面查看 workflow 状态是否显示 `finished`。

## 7. 源码和文档备份

只备份源码和文档，不备份编译、安装、测试日志和运行产物：

```bash
cd ~/panthera_workcell_ws
scripts/backup_source_docs.sh
```

备份内容包括：

- `src/`
- `docs/`
- `scripts/`
- `README.md`

排除内容包括：

- `build/`
- `install/`
- `log/`
- `Log/`
- `validation_logs/`
- `.runtime/`
- `.last_*`
- `.runtime_last_build_log`
- `.git/`

默认备份输出目录：

```text
/home/b1/panthera_workcell_ws_backups
```

备份包命名示例：

```text
panthera_workcell_ws_source_docs_20260615_203000.tar.gz
```

查看备份包内容：

```bash
tar -tzf /home/b1/panthera_workcell_ws_backups/<备份包名>.tar.gz | head
```

解压验证：

```bash
mkdir -p /tmp/panthera_backup_check
tar -xzf /home/b1/panthera_workcell_ws_backups/<备份包名>.tar.gz -C /tmp/panthera_backup_check
```

## 8. 重新编译

从源码备份恢复后，重新编译：

```bash
cd ~/panthera_workcell_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只改了 HMI、状态机或接口时，可以编译相关包：

```bash
colcon build --packages-select panthera_interfaces panthera_spectrometer_cell panthera_web_hmi
```

## 9. 关键配置文件

| 目标 | 文件 |
| --- | --- |
| 固定轨迹点位、路线和局部速度 | `src/panthera_motion/config/motion_catalog.yaml` |
| 光谱状态机、扫描时间、毛刷时间和硬件参数 | `src/panthera_spectrometer_cell/config/spectrometer_cell.yaml` |
| HMI 启动参数 | `src/panthera_web_hmi/launch/spectrometer_cell_hmi.launch.py` |
| 一键启动/停止/备份脚本 | `scripts/` |
| RS485 激光和流程信号 | `src/panthera_rs485/config/rs485_devices.yaml` |
| IO 输入输出 | `src/panthera_io/config/gpio_io.yaml` |
| MoveIt 和 ros2_control 配置 | `src/panthera_ht_config/` |
| URDF 和机械臂模型 | `src/panthera_ht_ros_description/` |

## 10. 点位和工艺时间微调

生产主数据源是 `src/panthera_motion/config/motion_catalog.yaml`。日常不要直接改旧
`named_poses`；在 HMI 调试页选择带 `tunable` 标签的点位示教并保存。当前关键基准为：

| 点位 | 当前 XYZ（m） |
| --- | --- |
| `outlet_1_grasp` | `[0.484046, -0.097, 0.212]` |
| `outlet_1_return` | `[0.484046, -0.097, 0.222]`（放回比取杯高 10 mm，自动跟随） |
| `outlet_2_grasp` | `[0.484102, 0.085769, 0.211]` |
| `outlet_2_return` | `[0.484102, 0.085769, 0.221]`（放回比取杯高 10 mm，自动跟随） |
| `spectrometer_place` | `[0.162, 0.480, 0.320]` |
| `spectrometer_pick` | `[0.162, 0.479497, 0.316902]` |
| `clean_dump` | `[-0.05874, -0.39374, 0.205]` |
| `brush_center` | `[0.101107, -0.39374, 0.12488564]` |
| `spectrometer_wait` | `[0.216, 0.190, 0.450]` |

光谱仪标准品机械基准 X 为 `162 mm`。激光参考值会随现场重新校准而变化，以 HMI
校准后写入 `axis_zero_laser_mm` 的值为准，不要在程序或操作中写死旧读数。放杯和取杯
各自重新采集稳定激光值并修正 X，不要求光谱仪停在标准位。

HMI 点击“进入调试（不移动）”后，可直接修改：

- 毛刷清洁时间：`cleaning.brush_hold_sec`，默认 `6 s`，范围 `0..60 s`。
- 光谱扫描完成时间：`loop.scan_duration_sec`，默认 `40 s`，范围 `1..300 s`。

保存会原子备份、热重载，失败自动回滚。固定轨迹目录保存还会编译全部路线；无需为
普通点位或时间修改重启整机。生产轨迹速度统一使用 100% 比例，六轴上限为
`2.0 rad/s`；倒料主翻腕保持 50% 加速度，摇料保持 45% 加速度。调试和异常恢复
路线仍使用较低速度。夹爪速度独立，不随机械臂速度比例变化。

夹爪改装后以机械闭合位置重新设置绝对零点；生产夹杯/完全打开目标分别为 `0.015 m` / `0.050 m`，夹杯保留 30% 开度，不再顶到机械零点。
开、合轨迹时长均为 `1.8667 s`，对应 `0.01875 m/s` 的硬件上限和 35 mm 有效行程；持续保持逻辑不变。编码器确认窗口和命令
收尾余量均为 `5 s`，失败重试前也等待 `5 s`，避免夹爪驱动短暂保护时立即重复冲击。

毛刷清洁严格按“杯子先套入 `brush_center` → 再启动毛刷 → 清洁 6 秒 → 退出毛刷
→ 停止毛刷”执行。毛刷启动失败时会在电机未转的状态下先安全退出杯子再报错。
毛刷生产默认转速为 40%。

清洗后路线为：沿杯口轴退出毛刷 → 保持倒料姿态垂直抬到 `z=0.45 m` → 高位回正
→ 直接跨区放回原出料口。禁止低位直接横穿，也不再返回低位倒料点。

## 11. 推荐现场流程

1. 确认机械臂周围安全，电源和急停状态正常。
2. 执行 `scripts/start_workcell.sh`。
3. 打开 HMI：`http://192.168.137.186:8080`。
4. 确认状态为 `WAIT_DISCHARGE`、速度 100%、机械臂和 HMI 服务正常。
5. 点击 `OUTLET_1 真实循环`；光谱仪移动和稳定计时会自动判断检测完成，`检测完成`按钮只作为外部信号/维护备用。
6. 本轮结束并回到 `WAIT_DISCHARGE` 后，才执行 `scripts/stop_workcell.sh`。
7. 关闭脚本显示 `Home=verified ... processes=clean` 后再切断设备电源。
8. 修改源码或文档后执行 `scripts/backup_source_docs.sh` 生成干净备份。
