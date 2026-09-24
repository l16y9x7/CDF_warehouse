# Navigation

统一架构里的导航能力进程。HTTP `:8001`。厂家对接写在 Adapter 里，不依赖 `common`，也不进 TianJi 仓。

同一份代码兼容 **天机** 和 **珞石**：公共层（`http_app` / `service`）只认 Adapter 合同，厂家差异分别在 `adapters/tianji.py` + `tianji_runtime/`、`adapters/rokae.py` + `rokae_runtime/`，两边互不 import。切厂家只改配置，不改 Python。

```text
中免 Agent / curl
    │  GET /navigation/health
    │  POST /navigation/navigate
    ▼
zhongmian :8081     ← 仅 adapter=rokae 时启动（apps/zhongmian/）
    │  转成 POST /goto，轮询 /status
    ▼
navigation :8001    ← 能力层（始终）
    │  config/navigation.json 的 "adapter"
    ├─ tianji → 厂家 HTTP /navigation/navigate（不启 zhongmian，避 :8081）
    ├─ rokae  → sros sim 或直连 192.168.71.50:5001（进程内）
    └─ fake   → 本机假底盘
```

中免接口见 [docs/ZHONGMIAN.md](docs/ZHONGMIAN.md)。真机总条例 [docs/测试条例.md](docs/测试条例.md)。能力层字段 [docs/HTTP_API.md](docs/HTTP_API.md)。地图/站点 JSON 长什么样 [docs/MAP_PAYLOAD.md](docs/MAP_PAYLOAD.md)。地图/站点何时更新 [docs/MAP_STATION_SYNC.md](docs/MAP_STATION_SYNC.md)。

仓库默认：`adapter=rokae`，`backend=sros`，`sros.sim=false`（真机）。本机假底盘把 `"sim"` 改 `true`。`python -m navigation` 在珞石模式下同时开 `:8001` 和 `:8081`。

珞石 Adapter 已对齐天机合同（`ready` / `snapshot` / `goto(station_id)`）。本机 sim、旧车 `.51` 的 A↔B 已通过。当前默认上机：**旧 Orin `192.168.71.51`**（底盘仍是 `.50`）。新车 `.105` 见 [ROKAE_TEST.md](ROKAE_TEST.md)。暂停、急停、任意位姿下一期。

## 切厂家（天机 / 珞石）

只动 `config/navigation.json`。
`rokae` 配置块可以留着不用删。`apps/zhongmian/zhongmian.json` 不用改（切天机时进程不会拉中免）。

### 珞石（仓库默认）

```json
"adapter": "rokae",
"ros": { "enabled": false }
```

`rokae.sros.sim`：本机 `true`，真机 `false`。中免 `:8081` 会一起起。

### 切到天机

两处必改：

```json
"adapter": "tianji",
"ros": {
  "enabled": true,
  "pose_topic": "/retail_nav/v1/pose",
  "status_topic": "/retail_nav/v1/status",
  "event_topic": "/retail_nav/v1/event"
}
```

`tianji` 块按车填（仓库里是旧 Orin 样例）：

| 字段 | 作用 |
| ---- | ---- |
| `base_url` | 天机导航 HTTP，样例 `http://127.0.0.1:8081` |
| `stations_yaml` | 车上站点 yaml |
| `map_id` | 当前图号 |
| `get_state` | OSD 用的 `GetState`；没有就把 `enabled` 设 `false` |

改完重启。日志应有 `adapter=tianji` 和 `skip zhongmian: tianji adapter already uses :8081`。

切完和珞石的差别：中免 Agent 那条 `:8081` 没有了，上游改打天机自己的 `/navigation/navigate`；`/stop` `/cancel` 天机 HTTP 不可用；站点来自 yaml，不是 MATRIX。

本仓库默认请保持 `"adapter": "rokae"`。现场切天机改本地/车上那份 json，不要把默认改成天机再合 MR。

## 启动

车上用 `bash navigation.sh start`，见文末「后台启停」。本机 sim：

```bash
# 仓库根 + sros-sdk-py（apps/zhongmian 启动时会挂上 sys.path）
export PYTHONPATH="$PWD:/path/to/sros-sdk-py"
python3 -m navigation --config config/navigation.json --log-level DEBUG
```

SDK 两种装法，`PYTHONPATH` 不一样，见 [ROKAE_TEST.md](ROKAE_TEST.md)：

| 车 | 管理口 | SDK | `PYTHONPATH` |
|--|--|--|--|
| 旧 Orin（当前默认） | `192.168.71.51` | `/home/admin/sros_sdk_py` | `/home/admin/nav:/home/admin` |
| 新 Orin | `192.168.130.105` | pip wheel → `~/.local/...` | `/home/admin/nav` |

真机日志还要有 `sros tcp ok`（避开 SDK 默认 bind 8888）。不要和 `sr_amr_control` 同时抢 SDK。MATRIX 已定位（`location_state == 3`）。`timeout_sec` 用 60。

| | `sros.sim` | 日志 | 车 |
|--|--|--|--|
| 本机 | `true` | `rokae sros sim started` | 不动 |
| 真机 | `false` | `Connect succeed: 192.168.71.50:5001` | 会动 |

`station_id`：珞石用 `A`/`B`/`1`/`2`（对应 MATRIX 站点 1/2）。路网、前进/后退以车上 MATRIX 为准，不是本机 json 里的坐标。天机用厂家 `target_id`（如 `start`），不要拿去打珞石。

```bash
curl -s http://127.0.0.1:8001/health
curl -s http://127.0.0.1:8001/state
curl -s -X POST http://127.0.0.1:8001/goto \
  -H 'Content-Type: application/json' \
  -d '{"task_id":"TASK-001","request_id":"REQ-B","timeout_sec":10,"idempotency_key":"sim-b-1","station_id":"B"}'
sleep 1
curl -s http://127.0.0.1:8001/status/REQ-B
```

先回 `RUNNING`，再查 `/status/{request_id}` 看终态。同一把 `idempotency_key` 不能换站点，失败后重试也要换新 key。

常见厂家错误（网页「导航到站点」也会同样弹）：

- `321005`：起点不在路网上，先把车开到边上
- `321008`：起点到终点没有路径，检查站点是否挂边、路是否双向

`stop`：sros 走 `cancel_movement_task`；ROS Cancel 实际不停，返回 `NAVIGATION_UNAVAILABLE`。`load_map`：当前图只回站点 + occupancy，不切换 MATRIX 原图。栅格从 FMS 包的 `{map}0.pgm` 转出。真机启动拉一次当前图站点+栅格；之后轮询 MATRIX 网页目录 `md5`，保存后才刷新站点，换图号才拉栅格。`/goto` 只查内存表。

## 接口

字段和示例见 [docs/HTTP_API.md](docs/HTTP_API.md)。

| 方法 | 路径 | 说明 |
| ---- | ---- | ---- |
| GET | `/health` | `{"ok": true/false}`，未定位为 false |
| GET | `/state` | OSD：`position` / `map` / `navigation_status` / `battery` / `chassis_status` / `self_check`。`map` 含 `stations`、`nodes`、`edges` |
| GET | `/status` | 当前底盘摘要或进行中的 goto |
| GET | `/status/{request_id}` | 一次 goto 的句柄 / 终态 |
| GET | `/map` | `map_id` + `stations` + `nodes` + `edges` |
| POST | `/goto` | 先返回 handle，再 `GET /status/{id}` |
| POST | `/stop` `/cancel` | 天机不可用；珞石 sros 可取消 |
| POST | `/refresh_stations` | 手动再刷 MATRIX 当前图站点 + 栅格；平时靠网页目录 `md5` 轮询 |
| POST | `/load_map` | 当前图：站点 + occupancy（幂等，不重载底盘） |
| POST | `/maps/import` | 把底盘格式 json 写入底盘地图目录。`switch` 默认 `false`，不切当前图 |

`POST /goto` 必填：`task_id`、`request_id`、`timeout_sec>0`、`idempotency_key`、`station_id`。未知路径 404。缺字段 / 忙 / 未就绪多数是 **HTTP 200 + `error_code`**。交互文档：`http://127.0.0.1:8001/docs`。

`POST /maps/import` 只给珞石。必填 `map_name`、`topology`（底盘那份 json：`meta`、`data`，坐标毫米）。可选 `switch`，布尔值，缺省是 `false`。图名只能是字母、数字、`_`、`-`，而且不能以数字结尾，否则底盘从 `{图名}0.pgm` 认不出图名。正在定位的图名会拒绝，避免盖掉当前图。nav 按 `meta.size` 生成空白 `{图名}0.pgm`（头三行注释是偏移和分辨率），再打给底盘 `POST /api/v0/map/import`。`switch` 为 `false` 时只在目录里多一张图，车还停在原图上。`switch` 为 `true` 时，导入成功后按当前位姿切到这张图：底盘会取消定位、换图、再按这个位姿重新定位。天机回 `NAVIGATION_UNAVAILABLE`。`/goto` 进行中回 `RESOURCE_BUSY`。

## 后台启停

车上：

```bash
cd /home/admin/nav
bash navigation.sh start
```

停：`bash navigation.sh stop`。日志在 `runtime/logs/navigation.log`。没有 `navigation.service`。

## 测试

```bash
PYTHONPATH=. python -m unittest discover -s tests -v
PYTHONPATH=. python -m unittest discover -s tests -p 'test_rokae.py' -v
PYTHONPATH=. python -m unittest discover -s tests -p 'test_zhongmian.py' -v
```
