# panthera_rs485

`panthera_rs485` 提供 RS485/Modbus RTU 底层库和 ROS 2 节点。当前已支持崛箭中文数显激光位移传感器的距离读取，以及一个通用 RS485 状态寄存器输入节点。

## 分层设计

```text
SerialPort
  -> ModbusRtuMaster
    -> LaserDisplacementSensor
      -> laser_distance_node
    -> rs485_state_signal_node
```

以后加新的 485 传感器时，优先复用 `SerialPort` 和 `ModbusRtuMaster`，只新增一个具体设备类和一个 ROS 节点。

## B1S-3588 串口说明

根据工控机手册，B1S-3588 的 RS485 方向收发由系统自动控制，应用程序不需要手动控制 RTS。

手册中的 RS485 设备名包括：

| 端子 | 设备 |
| --- | --- |
| A3/B3 | `/dev/ttyS3` |
| A4/B4 | `/dev/ttyS4` |
| A7/B7 | `/dev/ttyS7` |
| A8/B8 | `/dev/ttyS8` |

默认配置里：

- 激光传感器使用 `/dev/ttyS4`
- 工艺状态信号使用 `/dev/ttyS3`

如果实际接线不同，启动时改 `laser_port` 或 `state_port`。

## 激光传感器协议

根据激光传感器说明书：

- 通信：RS485, Modbus RTU
- 默认参数：9600, 8N1, 无校验
- 默认地址：`0x01`
- 距离读取：功能码 `0x04`，起始寄存器 `0x0000`，数量 `0x0002`

说明书没有明确写出两个寄存器的数值编码方式，因此节点提供可配置解码：

| `decode_mode` | 含义 |
| --- | --- |
| `float32_abcd` | 两寄存器按 ABCD 字节序解释为 float32 |
| `float32_cdab` | 两寄存器交换后解释为 float32 |
| `uint32_abcd` | 无符号 32 位整数，再乘 `scale` |
| `uint32_cdab` | 交换寄存器的无符号 32 位整数，再乘 `scale` |
| `int32_abcd` | 有符号 32 位整数，再乘 `scale` |
| `int32_cdab` | 交换寄存器的有符号 32 位整数，再乘 `scale` |

第一次接传感器时建议先查看原始寄存器和距离：

```bash
ros2 topic echo /sensors/laser/distance
```

如果距离明显不对，依次尝试 `float32_cdab`、`int32_abcd`、`uint32_abcd`，并按传感器实际单位调整 `scale` 和 `unit`。

## 启动

只启动 RS485 节点：

```bash
ros2 launch panthera_rs485 rs485_sensors.launch.py
```

指定端口和解码方式：

```bash
ros2 launch panthera_rs485 rs485_sensors.launch.py \
  start_laser:=true \
  laser_port:=/dev/ttyS4 \
  laser_slave_id:=1 \
  laser_decode_mode:=uint32_abcd \
  laser_scale:=0.1 \
  start_state_signal:=true \
  state_port:=/dev/ttyS3
```

当前这只 120..280 mm 量程激光传感器已实测：`/dev/ttyS4`、地址 `1`、寄存器原始值 `0x0000 0x05BC`，按 `uint32_abcd * 0.1` 解码为 `146.8 mm`。

节点启动后如果串口暂时不存在或传感器未上电，不会直接退出；它会按 `reconnect_interval_sec` 周期重连。连续读失败达到 `reopen_after_failures` 后会重开串口。

配置文件：

```text
src/panthera_rs485/config/rs485_devices.yaml
```

## 激光节点接口

节点：

```text
laser_distance_node
```

发布：

```text
/sensors/laser/distance  panthera_interfaces/msg/LaserDistance
```

服务：

```text
/laser_distance_node/read_once  panthera_interfaces/srv/GetLaserDistance
```

读取一次：

```bash
ros2 service call /laser_distance_node/read_once panthera_interfaces/srv/GetLaserDistance "{slave_id: 1}"
```

可选发布目标位姿：

```yaml
publish_pose: true
pose_topic: /laser/object_pose
pose_frame_id: base_link
pose_axis: z
pose_origin_xyz: [0.0, 0.0, 0.0]
reconnect_interval_sec: 2.0
reopen_after_failures: 5
```

这会把 `pose_origin_xyz` 沿 `pose_axis` 加上激光距离，发布为 `geometry_msgs/msg/PoseStamped`。正式使用前需要标定激光原点在 `base_link` 下的位置。

## RS485 状态信号节点

节点：

```text
rs485_state_signal_node
```

发布：

```text
/workflow/external_signal  panthera_interfaces/msg/ExternalSignal
```

配置示例：

```yaml
state_map:
  - "1:cup_pick_place:object_ready"
  - "2:laser_pick_example:laser_ready"
auto_run_workflow: false
reconnect_interval_sec: 2.0
reopen_after_failures: 5
```

当寄存器值为 `1` 时，节点发布：

```text
name: station_state
code: 1
active: true
workflow_name: cup_pick_place
detail: object_ready
```

如果 `auto_run_workflow=true`，节点会在状态码变化时自动调用 `/run_workflow`。如果保持 `false`，推荐在 YAML 流程里用 `wait_for_signal` 等待该信号。
