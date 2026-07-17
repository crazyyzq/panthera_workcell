# panthera_interfaces

`panthera_interfaces` 是机械臂应用层共用接口包。新增传感器、IO、状态机和流程控制时，优先复用这里的消息和服务。

## 流程控制

服务：

```text
/run_workflow  panthera_interfaces/srv/RunWorkflow
```

请求：

```text
string workflow_name
bool dry_run
```

响应：

```text
bool success
string message
```

状态话题：

```text
/workflow/status  panthera_interfaces/msg/WorkflowStatus
```

字段：

```text
uint8 STATE_IDLE=0
uint8 STATE_RUNNING=1
uint8 STATE_SUCCEEDED=2
uint8 STATE_FAILED=3
std_msgs/Header header
string workflow_name
string current_step
uint32 step_index
uint32 step_count
uint8 state
string message
```

## 激光距离输入

话题：

```text
/sensors/laser/distance  panthera_interfaces/msg/LaserDistance
```

字段：

```text
std_msgs/Header header
string source
uint8 slave_id
bool valid
float64 distance_mm
float64 distance_m
uint16[] raw_registers
string decode_mode
string status
```

一次读取服务：

```text
/laser_distance_node/read_once  panthera_interfaces/srv/GetLaserDistance
```

## 通用外部信号输入

话题：

```text
/workflow/external_signal  panthera_interfaces/msg/ExternalSignal
```

字段：

```text
std_msgs/Header header
string source
string name
int32 code
bool active
string workflow_name
string detail
```

约定：

- `source` 表示信号来源，例如 `rs485_process_state`、`gpio_dido`。
- `name` 表示信号名，例如 `station_state`。
- `code` 是外部状态码，例如 `1` 表示物体到位。
- `active` 表示当前信号是否有效。
- `workflow_name` 可选，用于把外部状态码映射到某个流程。
- `detail` 写可读说明，方便调试。

工作流可通过 `wait_for_signal` 等待该消息。

## 通用数字 IO

输入话题：

```text
/io/digital_inputs  panthera_interfaces/msg/DigitalInput
```

输出话题：

```text
/io/digital_outputs  panthera_interfaces/msg/DigitalOutput
```

读取输入：

```text
/gpio_io_node/get_input  panthera_interfaces/srv/GetDigitalInput
```

设置输出：

```text
/gpio_io_node/set_output  panthera_interfaces/srv/SetDigitalOutput
```

mock 输入注入：

```text
/gpio_io_node/set_input  panthera_interfaces/srv/SetDigitalInput
```

数字输入可以同时发布 `/workflow/external_signal`，流程 YAML 和 RS485 状态信号使用同一种 `wait_for_signal` 动作。

约定：

- `DigitalInput.value` / `DigitalOutput.value` 表示物理原始电平。
- `active` 表示按 `active_high` / `active_low` 换算后的逻辑有效状态。
- `SetDigitalOutput.active=true` 表示把输出置为逻辑有效，不要求调用者关心是否需要反相。
- `SetDigitalInput` 只用于 mock/仿真输入，真实 DI 由外部电气信号决定。
