# 地图 / 站点数据样式

nav **不上 MQTT、不上云**。它只在 `:8001` 给 Gateway 读。图和站点走 **两个口**，字段不一样。

现场图 `test_zhongmian`（Helios，2026-09-18）。单位除特别注明外都是 **米、弧度**。

```text
MATRIX FMS（毫米）
    │  pos.x/y ×0.001 → 米
    │  pgm + zero_offset → ROS 栅格
    ▼
nav :8001
    │
    ├─ GET  /state      1Hz OSD：图号 + 站点，没有栅格
    ├─ GET  /map        站点表，没有栅格
    └─ POST /load_map   当前图：站点 + occupancy（分辨率、原点、格子）
              │
              ▼
         Gateway
    ├─ MQTT OSD         只带 /state 的 map_id + stations
    └─ HTTP robotDog/map  必须用 /load_map 的 occupancy 上云
```

画点公式（站点、车位姿、栅格同一套）：

```text
世界x = origin.x + col * resolution
世界y = origin.y + row * resolution
```

`data[0]` 是左下角（ROS OccupancyGrid），`row` 往上增加。

---

## 1. nav 返回什么

### GET `/state`（OSD）

**没有** `resolution`、**没有** `origin`、**没有** 栅格。这是合同，不是漏字段。

```json
{
  "self_check": {"status": 0, "message": "就绪"},
  "position": {"x": -0.222, "y": 0.234, "yaw": -0.021},
  "map": {
    "map_id": "test_zhongmian",
    "occupancy_revision": "9a488a4d59f67a49",
    "stations": [
      {"station_id": "SORTING_BASKET_3", "x": -0.378, "y": 0.148, "yaw": 3.1662},
      {"station_id": "SORTING_BASKET_2", "x": -0.383, "y": -0.249, "yaw": 3.1482},
      {"station_id": "SORTING_BASKET_4", "x": -0.391, "y": 0.51, "yaw": 3.1159},
      {"station_id": "SORTING_BASKET_5", "x": -0.383, "y": 0.88, "yaw": 3.107},
      {"station_id": "AGV_C", "x": -0.235, "y": 0.239, "yaw": 6.2582},
      {"station_id": "AGV_R", "x": -0.25, "y": -0.182, "yaw": 6.2652},
      {"station_id": "AGV_L", "x": -0.236, "y": 0.655, "yaw": 6.2732},
      {"station_id": "站点8", "x": -0.386, "y": 0.261, "yaw": 3.1662}
    ]
  },
  "navigation_status": {"state": "IDLE"}
}
```

| 字段 | 有没有 | 说明 |
|---|---|---|
| `map.map_id` | 有 | 厂家图名，如 `test_zhongmian`。云端 UUID 由 Gateway 替换 |
| `map.stations[].station_id` | 有 | 站名 |
| `map.stations[].x/y/yaw` | 有 | 米 / 弧度。MATRIX 毫米已在 nav 转过 |
| `map.occupancy_revision` | 可选 | 栅格指纹，换图看 `map_id` 即可 |
| `position` | 有（已定位） | 车当前位姿，与站点同一坐标系 |
| `occupancy` / `resolution` / `origin` | **无** | 在 `/load_map` |

`GET /map` 只有 `map_id` + `stations`，同样没有栅格。

`POST /refresh_stations` 刷新内存表，成功体带 `stations` 和可选 `occupancy_revision`，**不带回整张 occupancy**。

### POST `/load_map`（栅格）

Gateway 换图时打一次：`{"map_id":"<当前 /state 里的 map_id>"}`。当前图幂等，不切 MATRIX。

```json
{
  "accepted": true,
  "map_id": "test_zhongmian",
  "stations": [
    {"station_id": "AGV_C", "x": -0.235, "y": 0.239, "yaw": 6.2582}
  ],
  "occupancy": {
    "width": 742,
    "height": 906,
    "resolution": 0.002,
    "origin": {"x": -0.53, "y": -0.33},
    "data": [0, 100, -1]
  }
}
```

| 字段 | 值（`test_zhongmian`） | 含义 |
|---|---|---|
| `occupancy.width` / `height` | 742 × 906 | 格数 |
| `occupancy.resolution` | `0.002` | 米/格（MATRIX `resolution=2` + `length_unit=mm`） |
| `occupancy.origin` | `{"x": -0.53, "y": -0.33}` | 左下角世界坐标，米。也允许 `[x, y]` |
| `occupancy.data` | 长度 `width*height` | `0` 空闲、`100` 占用、`-1` 未知 |

物理尺寸：`width * resolution` ≈ **1.484 m**，`height * resolution` ≈ **1.812 m**。  
站点必须落在：

```text
origin.x ≤ x ≤ origin.x + width * resolution
origin.y ≤ y ≤ origin.y + height * resolution
```

没有 `occupancy` 字段 = 当前图没有可用 pgm，不要用别的本地 yaml/bmap 顶替。

---

## 2. 不是 nav 的接口

### Gateway MQTT OSD

1Hz 读 `/state`，把 `map.map_id` 换成云端 UUID，**站点数组原样上云**。OSD 里仍然没有分辨率和原点。

平台若只根据 OSD 画底图，会觉得「没上传分辨率/原点」。那两项只在 `/load_map`。

### Gateway HTTP `robotDog/map`（当前仓库 `gateway/`）

当前这份 Gateway **还没打** nav `/load_map`。它读本地 `grid_yaml` / bmap / pgm，再 POST：

```json
{
  "sn": "ROBOT_001",
  "timestamp": 0,
  "map_id": "<UUID>",
  "info": {
    "width": 1092,
    "height": 1080,
    "resolution": 0.05,
    "origin": {
      "position": {"x": 0.0, "y": 0.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  },
  "data": []
}
```

外层再包：`{"data": "<上面整段 JSON 字符串>"}`。

和 nav 的差别：

| | nav `/load_map` | Gateway 当前上云 |
|---|---|---|
| 栅格来源 | MATRIX `{map}0.pgm` | 桌面 yaml / 本地 bmap |
| `resolution` | 顶层 `occupancy.resolution`，现场 `0.002` | `info.resolution`，yaml 常是 `0.05` |
| `origin` | 扁平 `{"x","y"}` | ROS 位姿 `info.origin.position.x/y` |
| 和站点 | 同一套米 | 可能是另一张图、另一套比例 |

要对齐站点，上云必须把 `/load_map` 的 `occupancy` 拷过去（`resolution`、`origin.x/y` 不要改单位、不要改结构除非平台明确要 Pose）。

---

## 3. 平台怎么用

1. 底图：HTTP 那份 occupancy 的 `width/height/resolution/origin/data`。
2. 蓝点：OSD `map.stations[].x/y`。
3. 红点：OSD `position.x/y`。
4. 三套必须用同一公式。默认 `resolution=0.05` 或 `origin=(0,0)` 都会让点飞。
5. 读 origin 时：nav 是 `origin.x`；若 Gateway 改写成 Pose，读 `origin.position.x`。

Helios 实测：nav 站点都在 `x∈[-0.53, 0.954]`、`y∈[-0.33, 1.482]` 内。料框在扫描区左侧灰色，是 MATRIX 标点位置，不是少传字段。
