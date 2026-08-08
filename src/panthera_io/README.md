# panthera_io

`panthera_io` 是通用数字 IO 接入包，用来把工控机 GPIO、DIDO 板、按钮、限位开关、光电开关等信号统一转换成 ROS2 接口。它和 `panthera_rs485` 并列存在，二者都可以发布 `/workflow/external_signal`，所以后续流程既可以由 RS485 状态机触发，也可以由 IO 信号触发。

## 当前 B1S-3588 硬件结论

根据《艾默特B1S-3588 Ubuntu系统用户手册V1.0》：

- B1S-3588 的可定制端子区域可以按不同接口板出厂。
- DIDO 版本为 `4xDI + 3xDO(固态继电器)`。
- DI/DO 采用 `2KVrms` 光耦隔离。
- DI 支持干接点和湿接点，默认干接点。
- DO 是固态继电器常开输出。

当前这台工控机在 2026-06-12 检查到：

- 系统有 `/dev/gpiochip0..7`，也有 sysfs GPIO。
- `/dev/gpiochip*` 权限为 `root:root 600`，普通 `b1` 用户不能直接访问字符设备。
- 有 `/dev/ttyS0`、`/dev/ttyS1`、`/dev/ttyS3`、`/dev/ttyS4`、`/dev/ttyS7`、`/dev/ttyS8`。
- 有 `can0..can3`。
- 手册中 DIDO 对应的 GPIO 当前被 pinmux 到 CAN/SPI/reset 等功能，不像已经装了 DIDO 接口板。

因此本包默认 `enabled: false`，不会触碰真实 GPIO。后续确认硬件为 DIDO 版本、接线检查完成后，再显式启用。

## B1S-3588 DIDO 电气性能

DI 输入：

| 类型 | 参数 |
| --- | --- |
| 隔离 | 2KVrms 光耦全隔离 |
| 数量 | 4 路，DI-P1..DI-P4 |
| 干接点 | 支持无源开关、按钮、限位开关、行程开关 |
| 湿接点 | 支持光电开关、NPN/PNP 传感器等有源外设 |
| 湿接点导通电压 | DC 10..30V |
| 湿接点输入电流 | 4..14mA |
| 湿接点逻辑 1 | +3.5..+30V |
| 湿接点逻辑 0 | +1V Max |
| 干接点逻辑 1 | Open，开路 |
| 干接点逻辑 0 | Close to ISO-GND，接隔离地 |

DO 输出：

| 类型 | 参数 |
| --- | --- |
| 输出形式 | 固态继电器常开输出 |
| 数量 | 3 路，DO1..DO3 |
| 每通道最大负载电流 | 500mA |
| 电压范围 | DC 0..60V |
| 导通电阻 | 240mΩ |

## 手册 GPIO 映射

| 端子 | 手册 GPIO 名称 | Linux GPIO 编号 |
| --- | --- | --- |
| DI-P1 | GPIO3_C4 | 116 |
| DI-P2 | GPIO3_C5 | 117 |
| DI-P3 | GPIO4_B4 | 140 |
| DI-P4 | GPIO4_B5 | 141 |
| DO-P1 | GPIO1_A1 | 33 |
| DO-P2 | GPIO1_A0 | 32 |
| DO-P3 | GPIO1_A2 | 34 |

Linux GPIO 编号按 Rockchip 常用规则计算：

```text
bank * 32 + group * 8 + bit
GPIO3_C4 = 3 * 32 + 2 * 8 + 4 = 116
```

## 启动

只查看参数，不启动真实 IO：

```bash
ros2 launch panthera_io gpio_io.launch.py --show-args
```

启动 mock 模式：

```bash
ros2 launch panthera_io gpio_io.launch.py \
  start_io:=true \
  io_enabled:=true \
  io_backend:=mock
```

mock 模式下输入默认处于 inactive 状态，可以用服务模拟按钮/限位触发：

```bash
ros2 service call /gpio_io_node/set_input panthera_interfaces/srv/SetDigitalInput \
  "{name: di1, active: true}"
```

真实 DIDO 板确认后启动：

```bash
sudo -E bash -c '
cd /home/b1/panthera_workcell_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch panthera_io gpio_io.launch.py \
  start_io:=true \
  io_enabled:=true \
  io_backend:=sysfs \
  io_allow_output_writes:=false
'
```

当前系统 `/dev/gpiochip*` 和 sysfs GPIO 需要 root 权限。正式部署时推荐写 udev 规则或 systemd 服务，让 IO 节点以受控权限运行，不要手工长期 `sudo`。

真实 DO 输出默认禁止写入。确认 DIDO 板、负载和接线后，再增加：

```bash
io_allow_output_writes:=true
```

## 一键启动中启用 IO

485 不删，IO 作为并列输入：

```bash
ros2 launch panthera_task_framework application_bringup.launch.py \
  start_laser:=true \
  start_state_signal:=true \
  start_io:=true \
  io_enabled:=true
```

如果只是先验证流程，不接真实 IO：

```bash
ros2 launch panthera_task_framework application_bringup.launch.py \
  start_io:=true \
  io_enabled:=true \
  io_backend:=mock
```

## 配置文件

默认配置：

```text
src/panthera_io/config/gpio_io.yaml
```

输入格式：

```text
name:gpio[:active_high|active_low[:code[:workflow_name[:detail]]]]
```

例子：

```yaml
inputs:
  - "di1:116:active_low:1:cup_pick_place:io_di1_active"
```

含义：

- `di1`：信号名。
- `116`：Linux GPIO 编号。
- `active_low`：低电平或闭合状态表示 active。
- `1`：发布到 `ExternalSignal.code` 的状态码。
- `cup_pick_place`：可选流程名。
- `io_di1_active`：调试说明。

输出格式：

```text
name:gpio[:active_high|active_low[:initial_value]]
```

例子：

```yaml
outputs:
  - "do1:33:active_high:false"
```

## ROS 接口

输入状态：

```text
/io/digital_inputs  panthera_interfaces/msg/DigitalInput
```

输出状态：

```text
/io/digital_outputs  panthera_interfaces/msg/DigitalOutput
```

统一流程信号：

```text
/workflow/external_signal  panthera_interfaces/msg/ExternalSignal
```

读取输入：

```bash
ros2 service call /gpio_io_node/get_input panthera_interfaces/srv/GetDigitalInput \
  "{name: di1}"
```

设置输出：

```bash
ros2 service call /gpio_io_node/set_output panthera_interfaces/srv/SetDigitalOutput \
  "{name: do1, active: true}"
```

mock 输入注入：

```bash
ros2 service call /gpio_io_node/set_input panthera_interfaces/srv/SetDigitalInput \
  "{name: di1, active: true}"
```

## 在流程 YAML 中等待 IO

IO 和 RS485 都发布 `ExternalSignal`，所以工作流写法一致：

```yaml
- name: wait_io_di1
  type: wait_for_signal
  topic: /workflow/external_signal
  source: gpio_dido
  signal_name: di1
  active: true
  timeout_sec: 30.0
```

`source` 和旧写法 `signal_source` 都可以使用。新流程建议写 `source`。

## 注意事项

- 当前机器疑似不是 DIDO 配置，不要直接启用真实 GPIO 输出。
- 真实 DO 输出默认被 `allow_output_writes=false` 拦住，接线确认前不要打开。
- DO 每通道最大 500mA，只适合驱动小负载或中间继电器输入，不要直接带大功率设备。
- 湿接点 DI 允许 DC 10..30V，接线前确认传感器 NPN/PNP 类型和公共端接法。
- 干接点 DI 默认开路为逻辑 1、接 ISO-GND 为逻辑 0；配置中通常写 `active_low`。
- 如果后续使用外置 PLC/IO 模块，也可以继续用 RS485 状态节点，不需要删本包。
