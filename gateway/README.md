# Gateway

端侧平台接入进程：只处理云端 MQTT，把命令转成场景任务，不解释业务 payload，也不控硬件。

云端协议对齐 `dog_device-SMT`（Topic、命令信封、回执、Event / OSD / Trajectory）。平台 HTTP（`POST /commands`）本期不做。

```text
云端 MQTT Broker
    │  services / services_reply / event / osd / trajectory
    ▼
Gateway
    │  payload 原样转发
    ▼
场景  POST /tasks  /tasks/stop  /tasks/recover
```

`accepted` 只表示 Gateway 已把命令转给场景，不等于任务完成。

## 职责

- 订阅下行命令，按 `tid` 去重，回复 `CommandReply`
- 按 method 路由到场景，`payload` 原样转发
- 周期上报 `RobotOSD`、`Trajectory`；本机 `POST /events` `/results` 转发 `TaskEvent` / `TaskResult`

不做：解释托盘数 / 物料码 / 曲名、直接控臂或底盘、`robot_move`、现场管理面。

## 命令路由

| method                         | 去向                                               |
| ------------------------------ | -------------------------------------------------- |
| `task_start` / `StartTask` | `data.scenario`，缺省配置 `default_scenario`（本机 `rokae`） → `POST /tasks` |
| `start_material_task`        | 强制`wrc` → `POST /tasks`                     |
| `play` / `PlayPiece`       | 强制`piano` → `POST /tasks`                   |
| `retail_start`               | 强制`retail` → `POST /tasks`                  |
| `rokae_start`                | 强制`rokae` → `POST /tasks`                   |
| `task_stop`                  | 所有已启用场景 →`POST /tasks/stop`              |
| `task_recovery`              | `smt` → `POST /tasks/recover`                 |

未知 method 回 `GATEWAY_UNKNOWN_METHOD`。场景未配置回 `GATEWAY_SCENARIO_NOT_CONFIGURED`。场景不可达回 `GATEWAY_SCENARIO_UNREACHABLE`。

转发给场景的正文：

```json
{
  "task_id": "TASK-001",
  "scenario": "smt",
  "idempotency_key": "TID-001",
  "payload": { }
}
```

`payload` 是云端 `data` 原样拷贝。

场景 HTTP 合同见 [docs/SCENARIO_HTTP.md](docs/SCENARIO_HTTP.md)。

## MQTT Topic

`{sn}` 必须等于配置里的 `device.sn`。各 Topic 的 JSON 字段、示例和数据来源见 [docs/MQTT_TOPICS.md](docs/MQTT_TOPICS.md)。

| 方向     | Topic                                       | QoS |
| -------- | ------------------------------------------- | --- |
| 下行命令 | `thing/product/robot/{sn}/services`       | 1   |
| 命令回执 | `thing/product/robot/{sn}/services_reply` | 1   |
| 业务事件 | `thing/product/robot/{sn}/event`          | 1   |
| 设备状态 | `thing/product/robot/{sn}/osd`            | 0   |
| 位置轨迹 | `thing/product/robot/{sn}/trajectory`     | 0   |

下行：

```json
{
  "method": "task_start",
  "bid": "BID-001",
  "tid": "TID-001",
  "data": {
    "task_id": "TASK-001"
  }
}
```

回执：`bid` / `tid` 原样带回。`code=0` 且 `data.result=0` 表示已接受。重复 `tid` 会返回处理中或缓存结果。

Event 使用 SMT 信封：`data.task_id` / `step_index` / `stage` / `title` / `thingking=""`。OSD 对齐 `dog_device-SMT`：`host_status` 由 Gateway 读本机 thermal；`battery` / `chassis_status` 来自导航 `GET /state`（本机是珞石 MATRIX / 导航 `:8001`）。缺字段就省略，不编造位姿。无效 `x/y/yaw` 的周期不发 Trajectory。

## 配置

默认配置：`config/gateway.json`。SMT 和中免共用这一套代码；拷到车上后改这份 JSON 里的机器人字段即可，不必整机互拷，也不要从 vision.json 自动认车。

关键项：

- `device.sn` / `name` / `alias`：写入 Topic 和 OSD。SMT 和中免用同一份代码，拷到车上后改这三项
- `mqtt.client_id`：可不填，默认 `gateway_{device.sn}`
- `scenarios.<id>.url` + `enabled`：这台车要转发的场景
- `default_scenario`：`task_start` 未带 `scenario` 时的去向
- `map_sync.default_source_map_id`：这台车的本地图号
- `media.stream_key`：这台车的 RTMP `?sign=`，不是地图 HTTP 的 `resource-token`
- `state.*`：OSD 拉取导航 / 控制 / 相机状态的地址
- `trajectory.enabled`：是否上报轨迹

改 SN 后 Topic 和默认 `client_id` 都会跟着变。不要从 vision.json 自动认车。

## 运行

本机用 conda 环境 `smt`，用 `gateway.sh` 启停，不装 systemd 服务：

```bash
cd /home/admin/gateway
bash gateway.sh status
bash gateway.sh restart
```

日志：`runtime/logs/gateway.log`。地图缓存也在 `runtime/`。本机只有这一份数据目录，不再分 SMT / 中免。拷代码不要带上它。

也可前台跑：

```bash
conda activate smt
cd /home/admin/gateway
PYTHONPATH=. python -m gateway --config config/gateway.json --log-level DEBUG
```

解释器优先 `/home/admin/miniconda3/envs/smt/bin/python`，可用环境变量覆盖：

```bash
PYTHON_BIN=/path/to/python GATEWAY_CONFIG=/path/to/gateway.json bash gateway.sh start
```

## 临时：改 OSD task

本机 HTTP，只改 OSD `data.task`，不走场景、不控硬件。`status`：`0` 空闲，`1` 运行中，`2` 完成，`3` 失败。

查当前：

```bash
curl -s http://127.0.0.1:8088/osd/task
```

设为运行中：

```bash
curl -s -X POST http://127.0.0.1:8088/osd/task \
  -H 'Content-Type: application/json' \
  -d '{"task_id":"TASK-001","status":1}'
```

设为失败：

```bash
curl -s -X POST http://127.0.0.1:8088/osd/task \
  -H 'Content-Type: application/json' \
  -d '{"task_id":"TASK-001","status":3,"error":"manual fail"}'
```

清空：

```bash
curl -s -X POST http://127.0.0.1:8088/osd/task/clear
```

下一秒 OSD 就会带上新 task。改完配置后需要 `bash gateway.sh restart`。

完整字段和校验说明见 [docs/OSD_TASK_HTTP.md](docs/OSD_TASK_HTTP.md)。交互文档：`http://127.0.0.1:8088/docs`。

## 临时：上报任务事件

场景把过程事件 POST 到 Gateway，再发到 MQTT `event`。终态走 `/results`。

```bash
curl -s -X POST http://127.0.0.1:8088/events \
  -H 'Content-Type: application/json' \
  -d '{"task_id":"TASK-001","step_index":1,"stage":"pick_pem","title":"grasp pose"}'

curl -s -X POST http://127.0.0.1:8088/results \
  -H 'Content-Type: application/json' \
  -d '{"task_id":"TASK-001","terminal_state":"SUCCEEDED","title":"任务完成","step_index":8}'
```

完整字段和校验说明见 [docs/EVENT_HTTP.md](docs/EVENT_HTTP.md)。场景侧合同见 [docs/SCENARIO_HTTP.md](docs/SCENARIO_HTTP.md)。

## 临时：同步地图上云

云端 2D 底图不走 MQTT。本机把占用栅格 POST 到 `robotDog/map`。成功后 OSD 的 `map_id` 会从本地图号换成 UUID。

2D 占用栅格来自导航 `POST /load_map` 的 `occupancy`（`resolution` / `origin.x,y` / `data` 原样上云），不用 Desktop yaml，也不从底盘 MATRIX 读。换图时只打一次 `/load_map`。之后只看导航 `GET /state` 的 `map.map_id`：图号变了再拉；没变不拉。`NO_MAP` 或没有 `map_id` 时不拉。没有 occupancy 就失败。平台画站点用 OSD 的站点 `x/y`，画底图用这份 occupancy 的 origin + resolution。

```bash
curl -s http://127.0.0.1:8088/map/sync
curl -s -X POST http://127.0.0.1:8088/map/sync \
  -H 'Content-Type: application/json' \
  -d '{}'
```

说明见 [docs/MAP_SYNC_HTTP.md](docs/MAP_SYNC_HTTP.md)。

## 云端推流

Gateway 不碰相机。启动时若 `media.push_on_start=true`，会让 vision Media `:8005` 把本机 MJPEG 推到 SMT 同一套 RTMP：

`rtmp://robotsolution.cn:1935/dksl/stream/{sn}_{slot}_99-0-0_normal-0`

槽位：`head=1`，`hand_left=2`，`hand_right=3`。`media.stream_key` 填这台车自己的钥匙，只给 RTMP `?sign=`，**不是**地图 HTTP 的 `resource-token`。

```bash
curl -s http://127.0.0.1:8088/media/push
curl -s -X POST http://127.0.0.1:8088/media/push/start -H 'Content-Type: application/json' -d '{}'
curl -s -X POST http://127.0.0.1:8088/media/push/stop -H 'Content-Type: application/json' -d '{"camera_id":"head"}'
```

也可以直打 Media：`http://127.0.0.1:8005/push`。

## 测试

```bash
conda activate smt
cd /home/admin/gateway
PYTHONPATH=. python -m unittest discover -s tests -v
```

## 目录

```text
gateway/
  app.py              进程入口，拉起 MQTT / OSD / Trajectory
  dispatcher.py       method 路由、去重、转场景
  scenario_client.py  调用场景 HTTP
  mqtt/client.py      MQTT 连接、收命令、发回执和遥测
  mqtt/osd.py         RobotOSD
  mqtt/trajectory.py  Trajectory
  mqtt/uplink.py      TaskEvent / TaskResult（MQTT 断线会排队）
  debug_http.py       临时 HTTP：OSD task、事件上报、地图同步、推流起停
  map_sync.py         换图 POST /load_map occupancy 原样上云 robotDog/map
  media_push.py       调 vision Media 起停云端 RTMP
  state_sources.py    状态源解析（导航 / 控制 / 相机 URL）
config/gateway.json
docs/MQTT_TOPICS.md    MQTT 各 Topic 载荷格式
docs/OSD_TASK_HTTP.md 临时 OSD task HTTP 接口文档
docs/EVENT_HTTP.md    临时任务事件 HTTP 接口文档
docs/SCENARIO_HTTP.md 珞石场景 HTTP 合同
docs/MAP_SYNC_HTTP.md 临时地图同步 HTTP 接口文档
gateway.sh            start / stop / status / restart
tests/
```
