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

- 激光数据新鲜、有效且在 `120..280 mm` 内：按激光值修正光谱仪 Y 坐标；
- 激光未接、数据超时或无效：立即使用默认 `150 mm`，不会阻塞生产；
- `150 mm` 对应 `motion_catalog.yaml` 中已经标定的光谱仪默认点；
- 当前换算为：

```text
Y目标 = Y默认 + (激光距离mm - 150mm) × 0.001m/mm
```

放杯和取杯分别在动作前读取一次。机械臂先到修正后的点位正上方，再沿 Z 轴
垂直下降；夹爪操作后垂直抬起，最后回到默认安全悬停点。动态轨迹仍由 Motion
Server 校验、编译和执行。

若某条产线要求激光故障必须停机，可把模式改为 `sensor_offset`；日常生产不要
修改 `axis_zero_laser_mm: 150.0`，除非重新完成机械标定。
