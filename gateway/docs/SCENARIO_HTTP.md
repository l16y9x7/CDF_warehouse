# 场景 HTTP 合同（Rokae）

Gateway 不跑业务、不控臂。云端 MQTT 命令转成下面的 HTTP；场景把过程/终态推回 Gateway。

本机配置：`default_scenario=rokae`，场景地址 `http://127.0.0.1:8094`。场景进程目前还没起来时，Gateway 仍上报导航 OSD，命令会回 `GATEWAY_SCENARIO_UNREACHABLE`。

## Gateway → 场景

| 云端 method | HTTP | 说明 |
| --- | --- | --- |
| `rokae_start` | `POST /tasks` | 强制 scenario=`rokae` |
| `task_start` / `StartTask` | `POST /tasks` | `data.scenario`，缺省用配置 `default_scenario`（本机 `rokae`） |
| `task_stop` | `POST /tasks/stop` | 所有已启用场景 |

请求正文：

```json
{
  "task_id": "TASK-001",
  "scenario": "rokae",
  "idempotency_key": "TID-001",
  "payload": { }
}
```

`payload` 是云端 `data` 原样拷贝。Gateway 不解释里面的字段。

场景应返回 JSON，并带 `accepted: true`（或 HTTP 2xx 且无 error_code）。例如：

```json
{ "accepted": true, "status": "running", "task_id": "TASK-001" }
```

拒绝时：

```json
{ "accepted": false, "error_code": "RESOURCE_BUSY", "message": "busy" }
```

## 场景 → Gateway

Gateway **不会**轮询场景的事件。场景自己 POST：

| 时机 | 地址 |
| --- | --- |
| 过程步骤 | `POST http://127.0.0.1:8088/events` |
| 任务终态 | `POST http://127.0.0.1:8088/results` |

字段见 [EVENT_HTTP.md](EVENT_HTTP.md)。MQTT 短暂断线时 Gateway 会把事件先排进内存，重连后再发。

## 场景 `GET /state`

OSD 优先用这里的 `task`。建议：

```json
{
  "task": { "task_id": "TASK-001", "status": 1 },
  "self_check": { "status": 0, "message": "就绪" }
}
```

`task.status`：`0` 空闲，`1` 运行，`2` 完成，`3` 失败。

`GET /health` 建议 `{ "ok": true }`。本机 `scenarios.rokae.required=false`，场景没起来不会把整机 OSD 自检打红。默认也不轮询场景 `/state`；若要让场景状态进 OSD，在场景配置里加 `"poll_state": true`。

## 不要做的事

- 不要让 Gateway 直连 xCore / 夹爪
- 不要改 MQTT Topic 或 OSD 信封字段名
- 底盘走导航 `http://127.0.0.1:8001`，不要经 Gateway 转发 `robot_move`
