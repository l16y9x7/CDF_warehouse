# 地图同步 HTTP 接口

Gateway 本机调试口，把占用栅格 POST 到云端实时地图接口，让页面能画出 2D 底图。

本机用导航 `POST /load_map` 返回的 `occupancy` 上云。换图时只打一次 `/load_map`。`sync_on_start=true` 时启动会再拉一次。之后 OSD 周期只看导航 `GET /state` 的 `map.map_id`：变了再 `POST /load_map`；没变不拉。`NO_MAP` 不拉。必须用导航栅格：`resolution` / `origin.{x,y}` / `data` 原样上云。Gateway **不**直连底盘 MATRIX，也不再读本地 bmap / Desktop yaml。

Gateway 向导航 `POST /load_map`，请求 `{"map_id":"<当前图号>"}`。导航成功时除 `accepted` / `stations` 外，需要带占用栅格：

```json
{
  "accepted": true,
  "map_id": "test_zhongmian",
  "stations": [{"station_id": "A", "x": 0.0, "y": 0.0, "yaw": 0.0}],
  "occupancy": {
    "width": 650,
    "height": 815,
    "resolution": 0.02,
    "origin": {"x": -3.46, "y": -3.3},
    "data": [0, 100, -1]
  }
}
```

`data` 长度必须等于 `width * height`：`0` 空闲、`100` 占用、`-1` 未知。行序与 ROS 占用图相同（原点在左下）。`origin` 也可用 `[x, y]`。没有 `occupancy` 会报错。

MQTT OSD **不传地图文件**，只带 `map.map_id` 和站点 `x/y`。平台画站点用 OSD 的 `x/y`，画底图用 occupancy 的 `origin` + `resolution`。云端要求 map ID 是 UUID，并且栅格已经通过 HTTP 上传过。导航给的是本地图号。上传成功后，下一秒 OSD / Trajectory 的 `map_id` 会换成 UUID。站点列表不变。

监听：`http://<机器人IP>:8088`（配置 `debug_http`）。  
路径：`GET/POST /map/sync`。  
云端接口：`POST https://robotsolution.cn/dj-prod-api/robotDog/map`。

## 调用前

```bash
cd /home/admin/gateway
bash gateway.sh status
tail -f ~/.local/share/gateway/logs/gateway.log
```

本机 `map_sync.enabled=true`，`sync_on_start=true`，启动时会再 `POST /load_map` 并上云（同一图号也会刷新栅格）。若这次启动上传失败，MQTT 连上云端后会再试；已经成功则 MQTT 重连不再传。之后 OSD 周期对照导航 `state.map.map_id`，图号变化才再拉。也可以手动 POST `/map/sync`。

```text
debug HTTP enabled: http://0.0.0.0:8088/osd/task /events /results /map/sync /media/push
map sync uploaded: sn=ROBOT_011 source=test_zhongmian map=52985de5-... size=650x815
```

## GET /map/sync

查当前本地图号、UUID 映射、上次同步结果。

```bash
curl -s http://127.0.0.1:8088/map/sync
```

响应字段：

| 字段 | 说明 |
| --- | --- |
| `current_source_map_id` | 当前跟的本地图号。有导航 `state.map.map_id`（且不是 `NO_MAP`）就用它；否则用 `default_source_map_id` |
| `current.map_id` | 对应的云端 UUID；尚未同步时为空对象 |
| `current.last_upload_ok` | 上次上传是否成功 |
| `items` | 已持久化的全部映射 |
| `last` | 本进程最近一次 `POST /map/sync` 结果 |

映射文件：`runtime/map_index.json`。同一张源图重复同步会复用同一个 UUID。拷到另一台时不要带上 `runtime/`。

## POST /map/sync

向导航 `POST /load_map` → 取 occupancy → 分配/复用 UUID → 上传云端。

```bash
curl -s -X POST http://127.0.0.1:8088/map/sync \
  -H 'Content-Type: application/json' \
  -d '{}'
```

指定本地图号：

```bash
curl -s -X POST http://127.0.0.1:8088/map/sync \
  -H 'Content-Type: application/json' \
  -d '{"source_map_id":"test_zhongmian"}'
```

请求字段都可选：

| 字段 | 说明 |
| --- | --- |
| `source_map_id` | 本地图号。省略时：有导航 `/state` 的 `map.map_id`（且不是 `NO_MAP`）就用它，否则用 `default_source_map_id` |
| `resource` / `resource_token` | 可选。云端 HTTP 鉴权头，本次请求覆盖配置里的 `platform_api` |

若云端返回 401，按 `platform_headers.py` 带上 `resource` / `resource-token`。可写在顶层 `platform_api`、`map_sync.platform_api`，或本次 POST。

```bash
curl -s -X POST http://127.0.0.1:8088/map/sync \
  -H 'Content-Type: application/json' \
  -d '{"resource":"robot_dog_service","resource_token":"<token>"}'
```

成功响应示例：

```json
{
  "ok": true,
  "source_map_id": "test_zhongmian",
  "map_id": "52985de5-49e9-470b-9f99-50245c9584ae",
  "width": 650,
  "height": 815,
  "resolution": 0.02,
  "origin": {"x": -3.46, "y": -3.3},
  "uploaded": true
}
```

失败示例：

```json
{"ok": false, "error": "MAP_SYNC_UPLOAD_FAILED", "message": "map upload failed: ..."}
{"ok": false, "error": "MAP_SYNC_BMAP_NOT_FOUND", "message": "nav load_map unreachable: ..."}
```

## 和 OSD 的关系

同步成功前，OSD `data.map.map_id` 仍是导航给的本地图号。  
成功后改为 UUID，站点列表不变。轨迹 Topic 里的 `map_id` 同样替换。

这个接口不改导航进程，也不控底盘。

## 配置

`config/gateway.json`：

```json
"map_sync": {
  "enabled": true,
  "sync_on_start": true,
  "report_url": "https://robotsolution.cn/dj-prod-api/robotDog/map",
  "upload_timeout_sec": 60,
  "identity_file": "map_index.json",
  "default_source_map_id": "test_zhongmian",
  "platform_api": {
    "resource": "robot_dog_service",
    "resource_token": "<token>"
  }
}
```

有 `state.navigation` 时走 `POST {navigation}/load_map`。导航 occupancy 的 `resolution` / `origin.{x,y}` / `data` 原样上云。

改配置后执行 `bash gateway.sh restart`。
