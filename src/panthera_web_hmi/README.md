# panthera_web_hmi

`panthera_web_hmi` 是 Panthera 光谱检测工作站的 Web 上位机包。

浏览器不直接控制硬件，所有状态和命令都经过 ROS2 后端节点：

```text
Web UI <-> HTTP/SSE <-> panthera_web_hmi ROS2 node <-> ROS2 topics/services
```

## 功能

- 显示长期运行状态机状态、当前 Cycle、来源出料口、放杯前/取杯前激光测距。
- 显示 Gemini305 相机图像，支持 RGB / 深度图切换。
- 显示激光传感器实时距离、RS485/IO 流程信号、机械臂关节状态。
- 提供 OUTLET_1/OUTLET_2 出料完成触发、急停、清除急停、人工复位按钮。
- 急停按钮按现场工业急停风格设计，点击后立即发送急停请求，不做二次确认。
- 操作台按安全、自动运行、流程信号模拟和点位示教分区。
- 点位示教支持 XYZ 和基坐标系 Roll/Pitch/Yaw 点动、夹爪、毛刷及实测点位热保存。
- 进入调试后无需前往点位即可修改毛刷清洁时间和光谱扫描完成时间；保存采用原子
  备份、运行时热重载和失败自动回滚。
- 高风险动作有确认弹层，按钮会根据 ESTOP、ERROR、PAUSED、动作中等状态自动禁用。
- 使用 SSE 实时刷新，HTTP 状态接口可用于调试和第三方接入。

## 操作规则

### 急停与恢复顺序

Web HMI 的急停是 ROS2 软件急停入口，真实产线仍必须保留硬接线急停回路。

急停后的恢复顺序固定为：

```text
释放硬件急停 -> 清除急停 -> 人工复位 -> 启动自动
```

页面互锁规则：

- `急停`：任何状态下都允许点击，且不弹确认框。
- `清除急停`：只在 `ESTOP` 状态开放。
- `人工复位`：急停清除后或 `ERROR` 状态开放。
- `启动自动` / `恢复自动`：只在安全空闲、等待出料或暂停状态开放。
- `点位示教`：仅允许空任务、未持杯且工位未占用时进入，进入后自动保持 `PAUSED`。

## 启动

只启动 Web 上位机，监控已经运行的系统：

```bash
cd /home/b1/panthera_workcell_ws
source install/setup.bash
ros2 launch panthera_web_hmi web_hmi.launch.py port:=8080
```

同时启动光谱检测长期状态机和 Web 上位机：

```bash
cd /home/b1/panthera_workcell_ws
source install/setup.bash
ros2 launch panthera_web_hmi spectrometer_cell_hmi.launch.py simulation:=true port:=8080
```

同时启动状态机、Web 上位机和 Gemini305 相机驱动：

```bash
cd /home/b1/panthera_workcell_ws
source install/setup.bash
ros2 launch panthera_web_hmi spectrometer_cell_hmi.launch.py \
  simulation:=true \
  port:=8080 \
  start_camera:=true
```

浏览器打开：

```text
http://192.168.137.186:8080
```

## Gemini305 相机

默认订阅：

| 图像 | 默认 topic |
| --- | --- |
| RGB | `/camera/color/image_raw` |
| 深度 | `/camera/depth/image_raw` |

如果 Orbbec Gemini305 驱动实际发布的 topic 不同，先查看：

```bash
ros2 topic list | grep -E "image|depth|color"
```

启动时覆盖：

```bash
ros2 launch panthera_web_hmi spectrometer_cell_hmi.launch.py \
  simulation:=true \
  port:=8080 \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_max_width:=640
```

后端直接订阅 `sensor_msgs/msg/Image`，然后转成 BMP 给浏览器显示，不依赖 OpenCV 或 Pillow。

已支持的常见编码：

| 类型 | 编码 |
| --- | --- |
| RGB 图 | `rgb8`, `bgr8`, `rgba8`, `bgra8`, `mono8`, `8UC1` |
| 深度图 | `16UC1`, `mono16`, `32FC1`, `mono8`, `8UC1` |

深度图会按当前画面中的有效深度值自动做伪彩色显示。无图像、无有效深度或编码不支持时，页面会显示占位图，同时状态显示为无图像或延迟。

## Web API

状态快照：

```text
GET /api/status
```

实时事件流：

```text
GET /api/events
```

相机图像：

```text
GET /api/camera/rgb.bmp
GET /api/camera/depth.bmp
```

发送命令：

```text
POST /api/command
Content-Type: application/json

{"command": "outlet_1_done"}
```

可用命令：

| Web 命令 | 默认 ROS2 service |
| --- | --- |
| `outlet_1_done` | `/spectrometer_cell/simulate_outlet_1_done` |
| `outlet_2_done` | `/spectrometer_cell/simulate_outlet_2_done` |
| `simulate_estop` | `/spectrometer_cell/simulate_estop` |
| `clear_estop` | `/spectrometer_cell/clear_estop` |
| `request_reset` | `/spectrometer_cell/request_reset` |
| `restart_cleaning_motor` | `/spectrometer_cell/restart_cleaning_motor` |
| `manual_mode` | `/spectrometer_cell/manual_mode` |
| `auto_mode` | `/spectrometer_cell/auto_mode` |

## ROS2 订阅接口

| Topic | 类型 | 页面显示 |
| --- | --- | --- |
| `/spectrometer_cell/state` | `std_msgs/msg/String` | 当前状态机状态 |
| `/spectrometer_cell/context` | `std_msgs/msg/String` | cycle、出料口、测距上下文 |
| `/spectrometer_cell/error` | `std_msgs/msg/String` | 错误信息 |
| `/sensors/laser/distance` | `panthera_interfaces/msg/LaserDistance` | 激光测距 |
| `/workflow/status` | `panthera_interfaces/msg/WorkflowStatus` | YAML workflow 状态 |
| `/workflow/external_signal` | `panthera_interfaces/msg/ExternalSignal` | RS485/IO 流程信号 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 关节位置 |
| `rgb_topic` 参数 | `sensor_msgs/msg/Image` | RGB 图像 |
| `depth_topic` 参数 | `sensor_msgs/msg/Image` | 深度图像 |

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `host` | `0.0.0.0` | HTTP 监听地址 |
| `port` | `8080` | HTTP 端口 |
| `start_camera` | `false` | 联动启动时是否一起启动 Gemini305 驱动 |
| `rgb_topic` | `/camera/color/image_raw` | RGB 图像 topic |
| `depth_topic` | `/camera/depth/image_raw` | 深度图 topic |
| `camera_max_width` | `640` | 浏览器图像最大宽度，越大带宽和 CPU 越高 |
| `service_outlet_1_done` | `/spectrometer_cell/simulate_outlet_1_done` | OUTLET_1 触发 service |
| `service_outlet_2_done` | `/spectrometer_cell/simulate_outlet_2_done` | OUTLET_2 触发 service |
| `service_simulate_estop` | `/spectrometer_cell/simulate_estop` | 急停 service |
| `service_clear_estop` | `/spectrometer_cell/clear_estop` | 清除急停 service |
| `service_request_reset` | `/spectrometer_cell/request_reset` | 人工复位 service |
| `service_manual_mode` | `/spectrometer_cell/manual_mode` | 切换人工/暂停 service |
| `service_auto_mode` | `/spectrometer_cell/auto_mode` | 启动/恢复自动 service |

## 现场建议

- 真机部署时，把页面运行在只允许内网访问的网络里。
- 急停按钮目前调用 ROS2 service，不能替代硬接线安全急停；真实产线必须保留硬件急停回路。
- 如果相机图像延迟，先把 `camera_max_width` 降到 `480` 或 `320`。
- 如果深度图没有显示，先确认 Gemini305 驱动是否开启 depth stream，并确认 topic 的 encoding。

## 验证

建议每次验证保存到 `validation_logs/<时间>_<说明>/`：

```bash
cd /home/b1/panthera_workcell_ws
source install/setup.bash
colcon build --symlink-install --packages-select panthera_web_hmi
ros2 launch panthera_web_hmi spectrometer_cell_hmi.launch.py simulation:=true port:=18080
curl http://127.0.0.1:18080/api/status
curl -I http://127.0.0.1:18080/api/camera/rgb.bmp
curl -I http://127.0.0.1:18080/api/camera/depth.bmp
```
## 机械臂点位示教

标准操作顺序：

1. 点击 `进入调试（不移动）`；系统只暂停自动流程，不移动机械臂。
2. 此时即可操作夹爪、毛刷，并修改毛刷清洁/光谱扫描时间；不需要选择或前往点位。
3. 需要示教时再选择带 `tunable` 标签的工艺点，点击 `前往所选点位`；系统验证或恢复 Home，再经过安全调试点到目标点。
4. 可使用 2–20 mm 平移步进或 0.1–10° 旋转步进按钮；也可填入基座坐标 XYZ/RPY 后执行。每次空间位移不得超过 20 mm、姿态变化不得超过 10°。
5. 确认合适后点击点位保存；后端保存最终命令目标、更新关联接近点，编译全部路线后原子热重载。
6. 点击退出调试；进入过点位时沿已验证路线回 Home，未前往点位时不移动机械臂。自动流程保持暂停，由操作员确认后恢复。

坐标映射固定为 `+X=右`、`+Y=前`、`+Z=上`，旋转也以 `base_link` 为基准。单次平移上限 20 mm，单次旋转上限 10 度。任一步编译或热重载失败都会恢复原目录。

`spectrometer_wait` 是独立的光谱仪检测安全等待位。生产放杯后先垂直退出到
`spectrometer_hover`，再移动到该等待位；因此示教等待位不会修改取杯/放杯的
垂直工艺列。

MIT 示教会保留上一命令点与实测 TCP 之间的负载偏置，再把操作员输入的物理位移叠加到命令点；这样小步进不会因重新使用重力下沉后的实测坐标而丢失补偿。页面会显示位置误差、姿态误差、负载补偿量和稳定修正次数，保存的是最终补偿后的命令点。

HMI 不提供电机零点设置。绝对零点维护使用厂商独立工具，不与生产控制进程并行运行。高级目录维护仍可编辑 `motion_catalog.yaml`，但不应作为日常点位微调入口。

示教 API：

```text
POST /api/debug/enter
POST /api/debug/goto
POST /api/debug/jog
POST /api/debug/move_to
POST /api/debug/save
POST /api/debug/exit
POST /api/debug/gripper
POST /api/debug/brush
POST /api/debug/process_timing
```

`/api/debug/process_timing` 仅在调试模式 ready 状态接受：

```json
{"brush_hold_sec": 6, "scan_duration_sec": 40}
```
