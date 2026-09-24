# 地图与站点更新 / 上报

导航 `:8001` 不切 MATRIX 原图，也不自己上云。权威源是底盘 MATRIX（`192.168.71.50`）。导航只做：发现变化 → 更新内存和 `maps/` → 给 Gateway 读。

**本期已做完。** Helios `test_zhongmian` 现场已拉到站点 + 栅格（`maps/test_zhongmian.json` + `.occupancy.json`）。对接说明给 Gateway 看第 3 节即可。

```text
MATRIX 网页建图/改站
    │
    ├─ 目录  GET /api/v0/map                 （name / md5 / modify_time，很小）
    └─ 站点+栅格  GET /api/v0/map/{id}/export?type=FMS
                  gzip+tar：json 里 data.station，{map}0.pgm 是占用图
              │
              ▼
         nav :8001
              │
    ┌─────────┼─────────┐
    │         │         │
 GET /state  GET /map  POST /load_map
 图号+站点    站点表     当前图栅格（幂等）
    │
    ▼
 Gateway 1Hz 拉 /state → MQTT OSD
 只在 map_id 变了时打一次 /load_map → HTTP 上云
 站点不用另拉，OSD 里已经是最新的
```

---

## 1. 怎么知道变了

MATRIX 没有“站点变更推送”。网页地图列表用的就是：

```text
GET http://192.168.71.50/api/v0/map
```

现场返回示例：

```json
{
  "maps": [
    {
      "name": "test_zhongmian",
      "md5": "8ded2fbd6144513da6a553487cf558cd",
      "modify_time": "2026-09-17 18:38:37",
      "map_version": "1.13.0"
    }
  ]
}
```

网页保存后 **图名可以不变**，`md5` / `modify_time` 会变。这就是同图改站的信号。

另外：SROS `SystemState.map_name` 是当前定位图号。换图时这个字段会变。

| 信号 | 来源 | 含义 |
|---|---|---|
| `map_name` 变了 | 底盘状态 | 换了一张图 |
| 目录 `md5` 变了 | `GET /api/v0/map` | 同一张图被保存（改站、改路网、改栅格都算） |
| 两者都没变 | — | 什么都不下载 |

默认 10 秒看一次目录（`rokae.matrix.poll_sec`，`0` 关闭）。目录 JSON 很小，不是每次下整张图。

---

## 2. 更新业务逻辑

### 启动

1. 读当前 `map_name`。
2. 拉 FMS，解析 `data.station`，写入内存 + `maps/{map_id}.json`。
3. 同一份 FMS 包抽出 `{map}0.pgm`，写成 `maps/{map_id}.occupancy.json`。这版 MATRIX 不支持 `type=NAV`（HTTP 500 `90010`）。
4. 记下该图的目录 `md5`。

`navigation.json` 里的 A/B 只是拉失败时的兜底。`/goto` 只查内存表。

### 运行中（每秒 `/state` 时顺带检查）

```text
map_name 变了？
    是 → 拉站点 + 拉栅格
    否 → 到了 poll_sec？
            否 → 结束
            是 → GET /api/v0/map
                  md5 没变 → 结束（不下 FMS）
                  md5 变了 → 只拉站点，不拉栅格
                  md5 变了 → 只拉站点，不拉栅格
```

换图才下栅格。同图改站只更新站点表。栅格从 FMS 包的 `{map}0.pgm` 转 OccupancyGrid，**不会每 10 秒重打大包**。

### 落盘

| 文件 | 内容 | 何时写 |
|---|---|---|
| `maps/{map_id}.json` | 统一站点表 | 启动 / 目录 md5 变 / 换图 / 手动刷新 |
| `maps/{map_id}.occupancy.json` | OccupancyGrid | 启动或换图（或手动 `POST /refresh_stations`） |

不切 MATRIX 网页上的原图，路网仍在厂家侧。

### 手动口

`POST /refresh_stations`：立刻再拉站点 + 尝试栅格。现场怀疑缓存过期时用。

---

JSON 字段长什么样、和 Gateway 上云体的差别，见 [MAP_PAYLOAD.md](MAP_PAYLOAD.md)。

## 3. 上报逻辑（给 Gateway）

导航不上 MQTT、不 POST 云端。Gateway 继续 1Hz `GET :8001/state`。

**Gateway 要做的只有两件事：**

1. OSD 原样带 `map.map_id` + `map.stations`（站点 nav 已经刷好，下一秒就是新表）。
2. `map_id` 从无到有、或和上次不同 → **打一次** `POST /load_map {"map_id":"<这个 id>"}`，把返回的 `occupancy` 上云。

**不用做：** 解析 MATRIX 目录 md5、为站点再调接口、读 `maps/*.json`、每秒 `/load_map`、把 occupancy 塞进 OSD。`occupancy_revision` 可留可丢，换图看 `map_id` 就够。同图只改站：`map_id` 不变，不要调 `/load_map`。

### OSD：`GET /state`

`map` 里只有图号和站点，**没有栅格**：

```json
{
  "map": {
    "map_id": "test_zhongmian",
    "stations": [
      {"station_id": "AGV_C", "x": 0.29, "y": -0.85, "yaw": 0.0}
    ],
    "occupancy_revision": "a1b2c3d4e5f60789"
  }
}
```

`occupancy_revision` 有栅格才出现。没有栅格就省略。站点改了下一秒 OSD 就是新表，不必再调刷新口。

Gateway OSD 请保留 `map_id` + `stations`。`occupancy_revision` 可留可丢；**换图看 `map_id` 即可**。

### 栅格：`POST /load_map`

不要每秒打。`map_id` 从无到有，或和上次不同，打一次：

```http
POST /load_map
{"map_id":"<state 里的 map_id>"}
```

当前图幂等：不切 MATRIX、不重载底盘。成功体带 `occupancy`（ROS OccupancyGrid：`0` 空闲、`100` 占用、`-1` 未知，原点左下，`data.length = width*height`）。

没有 `occupancy` 字段 = FMS 包里没有可用 pgm，不要用天机本地 bmap 顶替。

### 不要做的

- 不要把 occupancy 塞进 OSD
- 不要一直轮询 `/load_map` / `/refresh_stations`
- 不要用 `/map` 当实时上报（留给以后站点增删改查）

### 联调怎么看

1. MATRIX 网页加站并保存 → 最多约 10 秒 → `/state.map.stations` 出现新站。
2. MATRIX 切到另一张图并定位 → `/state.map.map_id` 变 → Gateway 打一次 `/load_map`。
3. 日志正常时应是偶尔 `matrix catalog unchanged`（debug），启动/换图时一次 `matrix occupancy raster: ...0.pgm`。

---

## 4. 现场限制

- 这版 MATRIX 只支持 `type=FMS`。`NAV`/`ALL` 会 500 `90010 Exported map types are not supported`。FMS 包里常有空的 `{map}.pgm`，真正的占用图是 `{map}0.pgm`（全分辨率），`{map}.png` 只是预览。
- 只改像素却不点保存：目录 `md5` 不变，导航不会刷新。同图改栅格也不重拉 occupancy（只换 `map_id` 才拉）。
- 云端改图不会回流到底盘。本期权威源是 MATRIX，不是公司平台。
- Helios 这台没有 `navigation.service`。启停用 `bash /home/admin/nav/navigation.sh start` 和 `stop`，不要 `systemctl`。

现场（2026-09-18，图 `test_zhongmian`）：8 站（含 `AGV_C` / `站点8`）；`POST /load_map` 有 occupancy，`742×906`，分辨率 `0.002` m，原点 `(-0.53, -0.33)`。车上 `maps/` 只保留 `test_zhongmian.json` 和 `test_zhongmian.occupancy.json`，旧的 `AB_0619.occupancy.json` / `hhhhh` 可删。
