# MQTT Topic 数据格式

云端协议对齐 `dog_device-SMT`。Gateway 只收发这些 Topic，不编造位姿、电量或任务终态。  
`{sn}` 必须等于配置里的 `device.sn`。拷到另一台车后改 `config/gateway.json` 的 SN，不要留 `auto`。

| 方向 | Topic | QoS | 用途 |
| --- | --- | --- | --- |
| 下行 | `thing/product/robot/{sn}/services` | 1 | 云端命令 |
| 上行 | `thing/product/robot/{sn}/services_reply` | 1 | 命令回执（已接受，不是任务完成） |
| 上行 | `thing/product/robot/{sn}/event` | 1 | 任务过程 / 终态事件 |
| 上行 | `thing/product/robot/{sn}/osd` | 0 | 设备状态，约 1 秒一条 |
| 上行 | `thing/product/robot/{sn}/trajectory` | 0 | 当前位姿采样，约 1 秒一条 |

不上 MQTT 的：2D 底图文件（HTTP `robotDog/map`）、相机画面（RTMP）。OSD 只带 `map.map_id`。

时间戳均为毫秒 Unix 时间。载荷是 JSON 对象。

---

## 1. `services` 下行命令

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

| 字段 | 必填 | 说明 |
| --- | --- | --- |
| `method` | 是 | 见下表。未知 method 回 `Unknown method` |
| `bid` | 建议 | 批次 ID，回执原样带回 |
| `tid` | 建议 | 幂等键；优先用 `tid` 去重，没有则用 `bid` |
| `data` | 否 | 对象；若是 JSON 字符串会再解析一次。原样转给场景，Gateway 不解释业务字段 |

`data` 里常见字段（Gateway 只认这些来路由，其余进 `payload`）：

| 字段 | 说明 |
| --- | --- |
| `task_id` | 任务 ID。`task_start` 等启动类命令缺少则拒绝 |
| `scenario` | 仅 `task_start` / `StartTask` 使用；缺省配置里的 `default_scenario` |
| `idempotency_key` | 可选；缺省用 `tid` 或 `bid` |

| method | 场景 | 场景 HTTP |
| --- | --- | --- |
| `task_start` / `StartTask` | `data.scenario`，缺省 `default_scenario` | `POST /tasks` |
| `start_material_task` | 强制 `wrc` | `POST /tasks` |
| `play` / `PlayPiece` | 强制 `piano` | `POST /tasks` |
| `retail_start` | 强制 `retail` | `POST /tasks` |
| `rokae_start` | 强制 `rokae` | `POST /tasks` |
| `task_stop` | 所有已启用场景 | `POST /tasks/stop` |
| `task_recovery` | `smt` | `POST /tasks/recover` |

转发给场景的正文：

```json
{
  "task_id": "TASK-001",
  "scenario": "smt",
  "idempotency_key": "TID-001",
  "payload": { }
}
```

`payload` 是云端 `data` 的拷贝。

---

## 2. `services_reply` 命令回执

`bid` / `tid` 与下行相同。`code=0` 且 `data.result=0` 表示 **Gateway 已把命令转给场景**，不是任务成功。

接受：

```json
{
  "bid": "BID-001",
  "tid": "TID-001",
  "code": 0,
  "message": "accepted",
  "timestamp": 1725868800123,
  "data": {
    "result": 0,
    "task_id": "TASK-001",
    "scenario": "smt",
    "status": "accepted"
  }
}
```

拒绝 / 失败：

```json
{
  "bid": "BID-001",
  "tid": "TID-001",
  "code": -1,
  "message": "unknown method",
  "timestamp": 1725868800123,
  "data": {
    "result": -1,
    "error": "GATEWAY_UNKNOWN_METHOD"
  }
}
```

同一 `tid` 仍在处理中：

```json
{
  "bid": "BID-001",
  "tid": "TID-001",
  "code": 0,
  "message": "command is processing",
  "timestamp": 1725868800123,
  "data": {
    "result": 0,
    "processing": true
  }
}
```

`data.error` 常见值：`GATEWAY_UNKNOWN_METHOD`、`GATEWAY_SCENARIO_NOT_CONFIGURED`、`GATEWAY_SCENARIO_UNREACHABLE`、`GATEWAY_SCENARIO_REJECTED`。

---

## 3. `event` 业务事件

过程事件和任务终态都走同一 Topic、同一外层信封。外层 `tid` / `bid` 固定空字符串。云端只发 `task_event` 这一种 event 名。

场景从本机 HTTP 喂给 Gateway：`POST /events`（过程）、`POST /results`（终态），见 [EVENT_HTTP.md](EVENT_HTTP.md)。Gateway 不主动轮询场景。

```json
{
  "tid": "",
  "bid": "",
  "timestamp": 1725868800456,
  "sn": "ROBOT_100",
  "data": {
    "task_id": "TASK-001",
    "step_index": 4,
    "stage": "pick_pem",
    "title": "grasp pose",
    "thingking": "",
    "description": "pose estimate completed",
    "image_url": "https://example.test/pick_pem.png"
  }
}
```

`data` 必填：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `task_id` | string | 任务 ID |
| `step_index` | int ≥ 1 | 步骤序号 |
| `stage` | string | 阶段名。Gateway 不解释业务含义 |
| `title` | string | 展示标题 |
| `thingking` | string | **始终空串**。场景即使带了推理文本也会被清掉 |

`data` 可选：

| 字段 | 说明 |
| --- | --- |
| `description` | 补充说明 |
| `image_url` | 配图 |
| `phase` + `confirmation_nonce` | 必须成对出现。nonce 为 16–128 位 `[A-Za-z0-9_-]` |

不允许 `data.type`。终态由场景 `TaskResult` 映射进同一信封，`stage` 为：

| 场景终态 | `data.stage` |
| --- | --- |
| `SUCCEEDED` / `completed` | `task_completed` |
| `FAILED` / `TIMED_OUT` / `REJECTED` | `task_failed` |
| `CANCELLED` | `task_cancelled` |

---

## 4. `osd` 设备状态

约每秒一条。缺字段就省略，不填 0、不编造位姿。

外层：

```json
{
  "tid": "8f14e45f-ea8d-4c2d-9c1a-0e1a2b3c4d5e",
  "timestamp": 1725868801000,
  "deviceType": "term",
  "robotType": "wheel_arm",
  "data": { }
}
```

| 字段 | 说明 |
| --- | --- |
| `tid` | 本条 OSD 的 UUID，与命令 `tid` 无关 |
| `timestamp` | 上报时刻 |
| `deviceType` | 配置 `device.deviceType`，默认 `term` |
| `robotType` | 配置了才出现 |
| `data` | 见下表 |

`data` 字段与来源（Gateway 合并各进程 `GET /state`，后写覆盖先写）：

| `data` 字段 | 何时出现 | 来源 |
| --- | --- | --- |
| `device_sn` | 每条都有 | `device.sn` |
| `alias` | 配置了别名 | `device.alias` / `device.name` |
| `task` | 每条都有 | 场景 `/state` 优先；否则 Gateway 内存；可用本机 HTTP 强制覆盖，见 [OSD_TASK_HTTP.md](OSD_TASK_HTTP.md) |
| `self_check` | 每条都有 | 各能力 `/health` 与 `/state.self_check` 合成。`status`：`0` 就绪，`1` 未就绪 |
| `position` | x/y/yaw 都是有限数字 | navigation `:8001/state` |
| `map` | 有非空且非 `NO_MAP` 的 `map_id` | 导航本地图号；同步成功后改写成云端 UUID。`stations` / `nodes` / `edges` 原样带上；底图用 `/load_map` occupancy 的 origin + resolution，见 [MAP_SYNC_HTTP.md](MAP_SYNC_HTTP.md) |
| `navigation_status` | 有内容 | navigation `:8001/state` |
| `battery` | 有内容 | navigation `:8001/state` |
| `chassis_status` | 有内容 | 同上。只保留 `state` + `motion`，丢掉 `drive_mode` |
| `host_status` | 读到温度或运行时长 | Gateway 本机 `/sys/class/thermal`，不经过 navigation |
| `manipulator_status` | 左右臂都新鲜 | 进程内 `ManipulatorStatePort.get_state()` → `to_legacy_dual_arm_mapping()`，不打 `:8092` |
| 相机列表 | **不上 OSD** | vision `:8003/state.cameras` 不在 OSD 白名单里。本机 camera `required=false`，相机未就绪不把整机自检打红 |

`/state` 里其它已识别的块有内容才带上：`robot_mode`、`arm_action`、`alarm_status`、`collector`、`transport`、`devices_info`、`imu`、`imu_status`、`motors`、`motor_status`、`odometry`、`locomotion`、`mainboard`、`safety`、`stm32_status`。空对象、空 `alarms` 不上报。相机不上 OSD。

完整示例（现场有导航、电池、主机温度时）：

```json
{
  "tid": "8f14e45f-ea8d-4c2d-9c1a-0e1a2b3c4d5e",
  "timestamp": 1725868801000,
  "deviceType": "term",
  "robotType": "wheel_arm",
  "data": {
    "device_sn": "ROBOT_100",
    "alias": "珞石机器人01",
    "position": { "x": 0.17, "y": 0.31, "yaw": 0.63 },
    "map": {
      "map_id": "3b241101-e2bb-4f3a-b20a-eaf62a0ee8d5",
      "stations": [{ "station_id": "start", "x": 0.0, "y": 0.0, "yaw": 0.0 }]
    },
    "navigation_status": { "state": "IDLE", "station_id": "start" },
    "battery": {
      "capacity_percent": 66.0,
      "voltage": 49.7,
      "temperature": 33.0,
      "charging": false,
      "cycle": 31
    },
    "chassis_status": {
      "state": "idle",
      "motion": {
        "stopped": true,
        "linear_x_mps": 0.0,
        "linear_y_mps": 0.0,
        "linear_speed_mps": 0.0,
        "angular_radps": 0.0
      }
    },
    "host_status": {
      "runtime_sec": 3600,
      "temperature_c": 47.5
    },
    "task": { "task_id": "TASK-001", "status": 1 },
    "self_check": { "status": 0, "message": "就绪" }
  }
}
```

`data.task.status`：`0` 未运行（此时 `task_id` 为空），`1` 运行中，`2` 完成，`3` 失败（可带 `error`，最长 300 字）。

`data.battery` 保留 `capacity_percent`、`voltage`、`temperature`、`charging`、`cycle`；其余非空键原样带上。  
`data.chassis_status.motion.linear_speed_mps` 有实测就报；没有则用 `linear_x_mps` / `linear_y_mps` 合成，再没有为 `null`。

位姿 / 电池 / 底盘再往下的真实数据源：navigation 订 ROS `/retail_nav/v1/pose`（厂家 `slam/LocationData`），电量和底盘运动来自 `http://6.6.7.6:8080/api/AMR/GetState`。Gateway 不直连这些口。

---

## 5. `trajectory` 位置轨迹

独立 Topic，不是规划折线。每秒从与 OSD 同一份 `/state` 快照里取当前 `position`。`x` / `y` / `yaw` 任一无效则 **本周期不发**。

```json
{
  "timestamp": 1725868801000,
  "device_sn": "ROBOT_100",
  "frame_id": "map",
  "child_frame_id": "base_link",
  "translation": { "x": 0.17, "y": 0.31, "z": 0.0 },
  "rotation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 },
  "map_id": "3b241101-e2bb-4f3a-b20a-eaf62a0ee8d5"
}
```

| 字段 | 说明 |
| --- | --- |
| `frame_id` | 配置 `trajectory.map_frame`，默认 `map` |
| `child_frame_id` | 配置 `trajectory.robot_frame`，默认 `base_link` |
| `translation.z` | 固定 `0.0` |
| `rotation` | 由 yaw 转成四元数（绕 z） |
| `map_id` | 有图号才出现；同步成功后与 OSD 一样是 UUID |

关闭上报：`trajectory.enabled=false`。
