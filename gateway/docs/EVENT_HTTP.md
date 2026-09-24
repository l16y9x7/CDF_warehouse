# 任务事件临时 HTTP 接口

Gateway 本机调试口，把场景的 `TaskEvent` / `TaskResult` 发到 MQTT `event` Topic。  
不解释业务 stage、不控硬件、不是云端 `POST /commands`。

监听：`http://<机器人IP>:8088`（配置 `debug_http`）。  
交互文档：`http://<机器人IP>:8088/docs`。  
Topic：`thing/product/robot/{sn}/event`，当前 SN 为 `ROBOT_011`。  
信封字段见 [MQTT_TOPICS.md](MQTT_TOPICS.md)。

场景把过程事件 POST 到 `/events`，终态 POST 到 `/results`。`thingking` 即使带了也会被清成空串。

## POST /events

必填：`task_id`、`step_index`（≥1）、`stage`、`title`。可选：`description`、`image_url`，以及成对的 `phase` + `confirmation_nonce`。不允许 `data.type`。

也可以把字段放在 `data` 里，和云端 event 信封一样。

```bash
curl -s -X POST http://127.0.0.1:8088/events \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id": "TASK-001",
    "step_index": 1,
    "stage": "pick_pem",
    "title": "grasp pose",
    "description": "pose estimate completed"
  }'
```

成功：`ok=true`，`published=true`，`data` 为发出去的 event 字段。  
校验失败 400 `INVALID_REQUEST`。MQTT 未连上 502，`published=false`。

未强制覆盖 OSD 时，过程事件会把 `data.task` 标成运行中。

## POST /results

必填：`task_id`、`terminal_state`（或 `stage`）。`step_index` 缺省 1。

`terminal_state` 映射：

| 场景终态 | event `stage` | OSD `task.status` |
| --- | --- | --- |
| `SUCCEEDED` / `completed` | `task_completed` | 2 |
| `FAILED` / `TIMED_OUT` / `REJECTED` | `task_failed` | 3 |
| `CANCELLED` | `task_cancelled` | 3 |

```bash
curl -s -X POST http://127.0.0.1:8088/results \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id": "TASK-001",
    "terminal_state": "SUCCEEDED",
    "title": "任务完成",
    "step_index": 8
  }'
```

核对：另开终端订 MQTT `thing/product/robot/ROBOT_011/event`，再 POST 上面两条。
