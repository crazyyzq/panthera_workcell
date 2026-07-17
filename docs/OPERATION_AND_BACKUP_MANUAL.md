# Panthera 光谱检测工作站操作与备份手册

本文档记录当前项目的日常启动、HMI 操作、相机恢复、速度调整、停机回零和源码文档备份流程。适用工作空间：

```bash
/home/b1/panthera_workcell_ws
```

Windows 共享路径：

```text
\\192.168.199.168\b1s_share\panthera_workcell_ws
```

HMI 地址：

```text
http://192.168.199.168:8080
```

## 1. 一键启动

推荐使用脚本启动整套系统：

```bash
cd ~/panthera_workcell_ws
scripts/start_workcell.sh
```

该脚本会启动：

- 机械臂硬件接口和 `ros2_control`
- MoveIt `move_group`
- YAML 工作流执行器 `/run_workflow`
- RS485 激光测距节点
- C++ 光谱检测长期状态机
- Web HMI
- 点位调试节点
- Gemini305 相机，默认请求 `1280x800@30fps`

启动日志保存到：

```text
validation_logs/<时间>_start_workcell
```

## 2. 一键关闭

正常关闭使用：

```bash
cd ~/panthera_workcell_ws
scripts/stop_workcell.sh
```

该脚本会先尝试让机械臂回到安全位：

1. 优先调用 `/run_workflow` 执行 `arm_home`
2. 如果失败，再尝试点位调试目标 `safe_joint_center`
3. 然后终止 HMI、状态机、相机、MoveIt、控制器等 ROS 进程

只在维护或紧急清进程时使用不回零版本：

```bash
scripts/stop_workcell.sh --no-home
```

## 3. HMI 常用功能

HMI 页面主要区域：

- 状态机状态：显示 `WAIT_DISCHARGE`、`PICK_FROM_OUTLET`、`ERROR`、`ESTOP` 等状态。
- 激光测距：显示当前 RS485 激光距离，单位 mm。
- 相机图像：支持 RGB 和深度图切换。
- 夹取中心实时位姿：显示 `base_link -> gripper_center` 的坐标和四元数。
- 流程按钮：触发出料完成、检测完成、回安全位、手动调试流程。
- 点位调试：打开调试模式后，可以先 `dry_run` 规划，再真实执行。
- 动作速度：运行时速度比例，范围 `20%~120%`，默认 `100%`。
- 相机重启：相机卡死或图像停止刷新时，点击 HMI 上的“重启相机”。

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
| 光谱检测状态机点位、速度、倒料参数 | `src/panthera_spectrometer_cell/config/spectrometer_cell.yaml` |
| HMI 启动参数 | `src/panthera_web_hmi/launch/spectrometer_cell_hmi.launch.py` |
| 一键启动/停止/备份脚本 | `scripts/` |
| RS485 激光和流程信号 | `src/panthera_rs485/config/rs485_devices.yaml` |
| IO 输入输出 | `src/panthera_io/config/gpio_io.yaml` |
| MoveIt 和 ros2_control 配置 | `src/panthera_ht_config/` |
| URDF 和机械臂模型 | `src/panthera_ht_ros_description/` |

## 10. 点位微调位置

光谱检测长期状态机的主要点位都在：

```bash
/home/b1/panthera_workcell_ws/src/panthera_spectrometer_cell/config/spectrometer_cell.yaml
```

单位规则：

- `xyz` 单位是米，现场 mm 要除以 1000。
- `rpy` 单位是弧度。
- `yaw=1.5708` 约等于绕 Z 轴 90 度。

### 10.1 出料口取放杯

```yaml
named_poses:
  outlet_1_pick:
    xyz: [-0.02500, -0.42853, 0.140]
    rpy: [0.0, 0.0, -1.5708]

  outlet_2_pick:
    xyz: [0.15500, -0.42853, 0.140]
    rpy: [0.0, 0.0, -1.5708]
```

改法：

- 杯子中心偏左/右/前/后：改对应点的 `xyz`。
- 夹爪方向不对：改对应点的 `rpy`，通常先微调第三个值 `yaw`。
- 取杯高度不对：改 `outlet_1_pick.xyz[2]` 或 `outlet_2_pick.xyz[2]`。

### 10.2 取杯后先抬高避障

夹完杯子后、去光谱仪前的安全抬升高度：

```yaml
motion:
  outlet_transfer_z: 0.400
```

这里的 `0.400` 表示 `gripper_center` 先抬到 `Z=400mm`，再规划去光谱仪。现场如果还会碰障碍，继续加大；如果太高导致规划困难，再降低。

### 10.3 光谱仪放杯/取杯基准

```yaml
named_poses:
  spectrometer_base:
    xyz: [0.58414, -0.12103, 0.290]
    rpy: [0.0, 0.0, 0.0]
```

如果激光为 `150mm` 时光谱仪位置正确，只微调 `spectrometer_base`。

激光轴补偿在：

```yaml
spectrometer_axis:
  axis_zero_laser_mm: 150.0
  axis_scale_m_per_mm: 0.001
  axis: y
```

- 激光 150mm 对应基准位置：改 `axis_zero_laser_mm`。
- 光谱仪移动方向反了：把 `axis_scale_m_per_mm` 改成负数。
- 光谱仪实际沿其它轴动：改 `axis`。

### 10.4 清理/倒料区

```yaml
named_poses:
  clean_approach:
    xyz: [0.250, 0.260, 0.240]
    rpy: [0.0, 0.0, 1.5708]
  clean_dump:
    xyz: [0.250, 0.400, 0.120]
    rpy: [0.0, 0.0, 1.5708]
```

倒料速度和摆动幅度：

```yaml
cleaning:
  pour_velocity_scale: 0.10
  pour_acceleration_scale: 0.10
  shake_angle_rad: 0.40
```

### 10.5 修改后如何生效

改完配置后重新编译并重启：

```bash
cd /home/b1/panthera_workcell_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select panthera_spectrometer_cell
scripts/stop_workcell.sh
scripts/start_workcell.sh
```

## 11. 推荐现场流程

1. 确认机械臂周围安全，电源和急停状态正常。
2. 执行 `scripts/start_workcell.sh`。
3. 打开 HMI：`http://192.168.199.168:8080`。
4. 检查状态机、激光、相机、关节状态。
5. 如需调点，打开点位调试模式，先 `dry_run`，再真实执行。
6. 正常生产使用自动流程按钮或外部 RS485/IO 信号触发。
7. 相机卡死时先点“重启相机”。
8. 下班或维护前执行 `scripts/stop_workcell.sh`，让机械臂先回安全位再关闭进程。
9. 修改源码或文档后执行 `scripts/backup_source_docs.sh` 生成干净备份。
