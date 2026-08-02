# 外部以太网指令接口

外部 PLC、上位机或检测软件通过工控机 HMI 的 HTTP 接口发指令。接口复用现有
状态机，只有 Motion Server 能控制机械臂，不会产生第二个轨迹发布者。

## 地址和数据格式

- 基地址：`http://<工控机IP>:8080`
- 指令：`POST /api/external/command`
- 查询：`GET /api/external/status`
- 编码：UTF-8 JSON
- 超时建议：连接 2 秒、响应 8 秒
- 重试建议：网络超时后使用**相同** `request_id` 重试

每个业务指令必须带唯一的 `request_id`。格式为 1～64 位字母、数字、点、
冒号、短横线或下划线。工控机持久保存最近 256 条结果，同一编号、同一指令
只执行一次；同一编号换成另一条指令会被拒绝。

## 支持的指令

| command | 含义 | 允许状态 |
|---|---|---|
| `PING` | 检查接口，不改变设备状态 | 任意 |
| `OUTLET_1_DISCHARGE_DONE` | 1 号料口出料完成，启动 1 号杯流程 | `IDLE` / `WAIT_DISCHARGE` |
| `OUTLET_2_DISCHARGE_DONE` | 2 号料口出料完成，启动 2 号杯流程 | `IDLE` / `WAIT_DISCHARGE` |
| `DETECTION_DONE` | 光谱仪检测完成，继续取杯和清洗 | `WAIT_DETECTION_DONE` |

状态不匹配时返回 HTTP 409，指令不会排队到错误的工艺阶段。上位机应等待正确
状态后使用新的 `request_id` 再发。

## 示例

连通性测试：

```bash
curl -sS -X POST http://<工控机IP>:8080/api/external/command \
  -H 'Content-Type: application/json' \
  -d '{"request_id":"plc-20260728-0001","command":"PING"}'
```

1 号料口完成：

```bash
curl -sS -X POST http://<工控机IP>:8080/api/external/command \
  -H 'Content-Type: application/json' \
  -d '{"request_id":"plc-20260728-0002","command":"OUTLET_1_DISCHARGE_DONE"}'
```

检测完成：

```bash
curl -sS -X POST http://<工控机IP>:8080/api/external/command \
  -H 'Content-Type: application/json' \
  -d '{"request_id":"spectrometer-20260728-0088","command":"DETECTION_DONE"}'
```

查询某条指令：

```bash
curl -sS \
  'http://<工控机IP>:8080/api/external/status?request_id=plc-20260728-0002'
```

不带 `request_id` 查询接口能力和实时工位状态：

```bash
curl -sS http://<工控机IP>:8080/api/external/status
```

典型响应：

```json
{
  "success": true,
  "api_version": "1.0",
  "request_id": "plc-20260728-0002",
  "command": "OUTLET_1_DISCHARGE_DONE",
  "accepted": true,
  "duplicate": false,
  "status": "accepted",
  "message": "queued OUTLET_1 discharge done",
  "workcell": {
    "state": "WAIT_DISCHARGE",
    "cycle_id": 12,
    "active_outlet": "NONE",
    "has_active_task": false
  }
}
```

`accepted` 表示状态机已接收，不表示整套机械动作已经结束。流程完成情况从响应中
的 `workcell.state`、查询接口或 `/api/status` 判断。完整周期结束后状态会回到
`WAIT_DISCHARGE`，并且 `has_active_task=false`。

如果料口信号到了但夹爪实际没有碰到杯子，机械臂会沿已验证的安全路线自动回
Home、张开夹爪并回到 `WAIT_DISCHARGE`，不会带着“空抓成功”继续运行。补杯后
必须使用新的 `request_id` 再发送对应料口指令。

如果工控机在写入请求后、写入处理结果前异常重启，该请求会标记为
`uncertain`。为避免重复抓取，外部系统必须先检查实时工位状态，再使用新编号
决定后续动作。

## 可选访问令牌

隔离产线网络可直接使用以上接口。需要令牌时，在一键启动前设置
`PANTHERA_EXTERNAL_COMMAND_TOKEN`，请求增加：

```text
X-Panthera-Token: <现场配置的令牌>
```

令牌不写入仓库。配置令牌后，缺少或错误令牌返回 HTTP 401。

## 激光传感器定位

当前配置为 `positioning.mode: sensor_optional`：

- 从未收到有效激光时使用已标定的固定点；
- 一旦收到有效激光，放杯和取杯分别等待一组新的稳定样本；中途失联不会盲目回退；
- 有效量程是 `120..280 mm`，超量程、非数和超过 1 秒的旧数据直接丢弃；
- 实时值为最近 5 个有效样本的中值；稳定值为 15 个实时滤波值的中值，跨度必须不大于 `1.0 mm`；
- 标准品机械坐标固定为 `X=162.0 mm`，激光基准由 HMI“校准标准品位置”按钮写入；
- 当前换算为：

```text
X目标 = 162.0mm + (当前滤波距离mm - 标定基准距离mm)
```

校准前必须确认光谱仪位于标准品位置；校准会原子更新
`axis_zero_laser_mm` 和 `fixed_axis_position_mm`，热重载失败时自动回滚。放杯和取杯分别
采集一次独立稳定窗口，均按光谱仪当时的随机位置执行，不要求回标准位。机械臂先到修正后的点位正上方，再沿 Z 轴
垂直下降；夹爪操作后垂直抬起，最后回到默认安全悬停点。动态轨迹仍由 Motion
Server 校验、编译和执行。

检测开始后先等待光谱仪回到标准品位置（误差不大于 `2 mm`，连续 5 个滤波样本），
再等待它离开基准 `+5 mm` 连续 5 个滤波样本后寻找最远点；
最大值超过 2 秒不再增加即按 `loop.scan_duration_sec` 计时（默认 40 秒）。该时间和
`cleaning.brush_hold_sec`（默认 6 秒）都可在 HMI 调试模式保存并热重载。一直不移动时持续等待，不报错。
外部 `DETECTION_DONE` 指令仍保留，可直接完成同一等待阶段。
