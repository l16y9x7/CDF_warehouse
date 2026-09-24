# OSD Task 临时 HTTP 接口

Gateway 本机调试口，只改即将上报到 MQTT OSD 的 `data.task`。  
不转发场景、不控硬件、不是云端 `POST /commands`。

监听：`http://<机器人IP>:8088`（配置 `debug_http`，默认 `0.0.0.0:8088`）。  
交互文档：`http://<机器人IP>:8088/docs`。  
Topic：`thing/product/robot/{sn}/osd`，当前 SN 为 `ROBOT_011`。  
完整 OSD 信封见 [MQTT_TOPICS.md](MQTT_TOPICS.md)。

调用后最多 1 秒会出现在下一条 OSD。日志：`runtime/logs/gateway.log`。

## 调用前

```bash
cd /home/admin/gateway
bash gateway.sh status
tail -f runtime/logs/gateway.log
```

日志里应有：

```text
OSD task HTTP enabled: http://0.0.0.0:8088/osd/task
temporary OSD task HTTP listening on 0.0.0.0:8088
```

## status 取值

| 值 | 别名 | OSD `data.task.status` |
| --- | --- | --- |
| `0` | `idle` | 未运行，`task_id` 会被清空 |
| `1` | `running` | 运行中 |
| `2` | `completed` / `succeeded` | 已完成 |
| `3` | `failed` / `error` | 失败，可带 `error` |

非 0 时 `task_id` 必填。

## GET /osd/task

查当前 Gateway 内存里的 task（等于下一秒 OSD 要用的 task）。

```bash
curl -s http://127.0.0.1:8088/osd/task
```

响应：

```json
{
  "ok": true,
  "task": {
    "task_id": "",
    "status": 0
  }
}
```

`GET /health` 返回同样内容。

日志：

```text
HTTP GET /osd/task from 127.0.0.1 -> 200 task_id= status=0 ...
```

## POST /osd/task

写入 task，并强制覆盖场景 `/state`（如果以后场景起来了也以这次为准）。

```bash
curl -s -X POST http://127.0.0.1:8088/osd/task \
  -H 'Content-Type: application/json' \
  -d '{
    "task_id": "TASK-001",
    "status": 1
  }'
```

请求字段：

| 字段 | 必填 | 说明 |
| --- | --- | --- |
| `status` | 是 | `0/1/2/3` 或英文别名 |
| `task_id` | status≠0 时必填 | 业务任务 ID |
| `error` | 否 | 失败说明，status=3 时进 OSD |

成功响应：

```json
{
  "ok": true,
  "task": {
    "task_id": "TASK-001",
    "status": 1
  }
}
```

失败示例（400）：

```json
{"ok": false, "error": "INVALID_REQUEST", "message": "task_id is required when status is not idle"}
```

其它常用调用：

```bash
# 完成
curl -s -X POST http://127.0.0.1:8088/osd/task \
  -H 'Content-Type: application/json' \
  -d '{"task_id":"TASK-001","status":2}'

# 失败
curl -s -X POST http://127.0.0.1:8088/osd/task \
  -H 'Content-Type: application/json' \
  -d '{"task_id":"TASK-001","status":3,"error":"manual fail"}'

# 空闲（等同 clear，task_id 会被丢掉）
curl -s -X POST http://127.0.0.1:8088/osd/task \
  -H 'Content-Type: application/json' \
  -d '{"status":0}'
```

日志：

```text
HTTP POST /osd/task from 127.0.0.1 body={'task_id': 'TASK-001', 'status': 1, ...}
OSD task updated via HTTP: before={...} after={...} forced=True
OSD published: count=... task_id=TASK-001 task_status=1 ... forced=True
```

最后一行表示已经发到 MQTT `thing/product/robot/ROBOT_011/osd`。强制覆盖期间每秒打一次。

## POST /osd/task/clear

清空为 idle，并取消强制覆盖。

```bash
curl -s -X POST http://127.0.0.1:8088/osd/task/clear
```

响应：

```json
{
  "ok": true,
  "task": {
    "task_id": "",
    "status": 0
  }
}
```

## 对应 OSD 报文

HTTP 写入后，MQTT OSD 的 `data.task` 即为上述 `task`：

```json
{
  "tid": "<uuid>",
  "timestamp": 178...,
  "deviceType": "term",
  "data": {
    "device_sn": "ROBOT_100",
            "alias": "珞石机器人01",
    "task": {
      "task_id": "TASK-001",
      "status": 1
    },
    "self_check": {"status": 1, "message": "..."}
  }
}
```

位姿仍取决于导航/场景 `/state`。地图底图请用临时接口 `POST /map/sync` 上云，见 [MAP_SYNC_HTTP.md](MAP_SYNC_HTTP.md)。

## 配置

`config/gateway.json`：

```json
"debug_http": {
  "enabled": true,
  "host": "0.0.0.0",
  "port": 8088
}
```

改配置后执行 `bash gateway.sh restart`。
