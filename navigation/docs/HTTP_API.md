# Navigation HTTP `:8001`

业务拒绝多数是 **HTTP 200 + `error_code`**。未知路径才是 404 `{"ok":false,"error_code":"NOT_FOUND"}`。JSON 非法是 400 `INVALID_JSON`。

交互文档：`http://127.0.0.1:8001/docs`。

---

## GET `/health`

未定位 / Adapter 未就绪为 `false`。

```json
{"ok": true}
```

```json
{"ok": false}
```

---

## GET `/state`

OSD 碎片。字段按有数据才出现：`position` 要有 x/y；`map` 要有非空 `map_id`；`battery` / `chassis_status` 厂家有才给。珞石还会按 SROS 新鲜度补底盘块：`robot_mode` / `odometry` / `locomotion` / `safety` / `alarm_status` / `devices_info` / `motors` / `mainboard` / `imu` / `stm32_status` / `collector` / `transport`。`collector`/`transport` 只含底盘源，没有臂/腰。无值整块省略。信封 `tid` / `device_sn` 由 Gateway 填。

```json
{
  "self_check": {"status": 0, "message": "就绪"},
  "position": {"x": 1.262, "y": -18.971, "yaw": 1.55},
  "map": {
    "map_id": "test",
    "occupancy_revision": "a1b2c3d4e5f60789",
    "stations": [
      {"station_id": "A", "x": 0.2887, "y": -0.8487, "yaw": 0.0},
      {"station_id": "B", "x": 0.501, "y": -0.8487, "yaw": 0.0}
    ]
  },
  "navigation_status": {"state": "RUNNING", "station_id": "A", "request_id": "REQ-B1"},
  "battery": {
    "capacity_percent": 72.0,
    "temperature": 33.0,
    "charging": false,
    "cycle": 0,
    "voltage": 54.1
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
  }
}
```

| 字段 | 说明 |
|---|---|
| `self_check.status` | `0` 就绪，`1` 未就绪 |
| `navigation_status.state` | `IDLE` / `RUNNING` / `SUCCEEDED` / `FAILED` / `CANCELLED` |
| `chassis_status.state` | `emergency` / `error` / `paused` / `initializing` / `idle` / `moving` |
| `request_id` | 仅当前有未结束或刚结束的 `goto` 时出现 |
| `map.occupancy_revision` | 栅格内容指纹，有就带。换图看 `map_id` 即可再打 `POST /load_map`；栅格本身不上 `/state` |

未就绪示例：

```json
{
  "self_check": {"status": 1, "message": "导航未就绪"},
  "navigation_status": {"state": "IDLE"}
}
```

---

## GET `/status`

有进行中的 `goto` 时，返回该次动作（同 `/status/{request_id}`）。否则底盘摘要：

```json
{
  "state": "IDLE",
  "station_id": "A",
  "position": {"x": 0.2887, "y": -0.8487, "yaw": 0.0},
  "map_id": "AB_0619"
}
```

---

## GET `/status/{request_id}`

一次 `goto` 的句柄 / 终态。未知 id：

```json
{
  "accepted": false,
  "error_code": "NAVIGATION_UNKNOWN_REQUEST",
  "request_id": "NO-SUCH"
}
```

进行中：

```json
{
  "accepted": true,
  "request_id": "REQ-B1",
  "task_id": "TASK-001",
  "station_id": "B",
  "idempotency_key": "field-b-1",
  "state": "RUNNING",
  "terminal_state": null,
  "error_code": "",
  "message": "",
  "evidence": null
}
```

成功：

```json
{
  "accepted": true,
  "request_id": "REQ-B1",
  "task_id": "TASK-001",
  "station_id": "B",
  "idempotency_key": "field-b-1",
  "state": "SUCCEEDED",
  "terminal_state": "SUCCEEDED",
  "error_code": "",
  "message": "",
  "evidence": {"arrived": true, "station_id": "B", "vendor_id": 2}
}
```

`state` / `terminal_state`：`ACCEPTED` → `RUNNING` → `SUCCEEDED` | `FAILED` | `REJECTED` | `CANCELLED` | `TIMED_OUT`。

---

## GET `/map`

当前 **统一站点表**（不是 MATRIX/天机原文件）。

```json
{
  "map_id": "AB_0619",
  "stations": [
    {"station_id": "A", "x": 0.2887, "y": -0.8487, "yaw": 0.0},
    {"station_id": "B", "x": 0.501, "y": -0.8487, "yaw": 0.0}
  ]
}
```

`goto` 的 `station_id` 必须能在这张表里解析（名称 / `vendor_id` / aliases）。真机启动时会从 MATRIX 当前图拉一次站点；之后轮询网页地图目录的 `md5`，MATRIX 保存后会自动刷新。`/goto` 只查内存表，不下载。

---

## POST `/refresh_stations`

启动已经拉过一次。平时只轮询 MATRIX 网页目录 `GET /api/v0/map`（`md5` / `modify_time`），**不变就不下 FMS、不下栅格**。目录变了才拉站点；底盘 `map_name` 变了才拉栅格。`occupancy_revision` 有就带，Gateway 也可以只看 `map_id`。`rokae.matrix.poll_sec` 可改间隔，`0` 关闭轮询。成功后写入 `maps/{map_id}.json`（站点）和 `maps/{map_id}.occupancy.json`（仅换图或手动刷新时）。

```json
{
  "accepted": true,
  "map_id": "hjl",
  "stations": [
    {"station_id": "站点1", "x": 0.2887, "y": -0.8487, "yaw": 0.0},
    {"station_id": "站点2", "x": 0.501, "y": -0.8487, "yaw": 0.0}
  ]
}
```

失败仍 HTTP 200：`accepted: false`，常见 `NAVIGATION_VENDOR_UNREACHABLE`（底盘 HTTP 不通）或拓扑里没有 `data.station`。

---

## POST `/goto`

必填：`task_id`、`request_id`、`timeout_sec>0`、`idempotency_key`、`station_id`。

```json
{
  "task_id": "TASK-001",
  "request_id": "REQ-B1",
  "timeout_sec": 60,
  "idempotency_key": "field-b-1",
  "station_id": "B"
}
```

立刻返回 handle（`accepted: true`，`state` 为 `ACCEPTED` 或已变成 `RUNNING`），再 `GET /status/REQ-B1` 看终态。同一 `idempotency_key` 再发同一站点会返回原动作；换站点是 `NAVIGATION_IDEMPOTENCY_CONFLICT`。底盘忙是 `RESOURCE_BUSY`。未定位是 `NAVIGATION_NOT_READY`。缺字段是 `NAVIGATION_INVALID_REQUEST`。未知站点立刻 `accepted: false` + `NAVIGATION_UNKNOWN_STATION`，不进 RUNNING、不发车。

拒绝示例（仍 HTTP 200）：

```json
{
  "accepted": false,
  "terminal_state": "REJECTED",
  "error_code": "NAVIGATION_NOT_READY",
  "message": "navigation adapter not ready"
}
```

---

## POST `/stop` / `/cancel`

`/cancel` 需要 `request_id`。珞石 sros 可取消；天机 HTTP 返回 `NAVIGATION_UNAVAILABLE`。

---

## POST `/load_map`

Gateway 同步栅格会打这个口：`{"map_id":"<当前图号>"}`。**当前图幂等**：不切 MATRIX、不重载底盘，只回站点 + 占用栅格。OSD 的图号/站点仍看 `GET /state` 的 `map`，不要把栅格塞进 `/state`。`/state` 和 `/load_map` 各有哪些字段，见 [MAP_PAYLOAD.md](MAP_PAYLOAD.md)。

成功（当前图）：

```json
{
  "accepted": true,
  "map_id": "test_zhongmian",
  "stations": [{"station_id": "AGV_C", "x": 0.29, "y": -0.85, "yaw": 0.0}],
  "occupancy": {
    "width": 742,
    "height": 906,
    "resolution": 0.002,
    "origin": {"x": -0.53, "y": -0.33},
    "data": [100, 100, 0]
  }
}
```

`occupancy.data` 长度必须等于 `width * height`：`0` 空闲、`100` 占用、`-1` 未知；行序与 ROS OccupancyGrid 相同，原点在左下。`origin` 也可以是 `[x, y]`。栅格从 MATRIX FMS 导出包的 `{map}0.pgm` 转出来。

仍可加载统一站点 JSON（`maps/{map_id}.json`），不直接吃厂家包当切图。路网仍在厂家侧。

| 请求 | 作用 |
|---|---|
| `{"map_id":"<当前图>"}` | 幂等：站点 + occupancy，不重载底盘 |
| `{"map_id":"AB_0619"}`（非当前图） | 读 `maps/AB_0619.json` 并应用站点表 |
| `{"map_id":"duty-ab","stations":[...]}` | 写成文件并应用 |
| `{"map_id":"snap-1","save_current":true}` | 把当前 `/map` 存成统一文件 |

文件不存在且厂家也不会切图：`NAVIGATION_UNKNOWN_MAP` / `NAVIGATION_UNAVAILABLE`。缺 `map_id`：`NAVIGATION_INVALID_REQUEST`。当前图缺少栅格时仍 `accepted`，但没有 `occupancy` 字段，Gateway 上云会找不到栅格。

仓库默认图：`maps/AB_0619.json`。目录可用配置 `maps_dir` 覆盖。
