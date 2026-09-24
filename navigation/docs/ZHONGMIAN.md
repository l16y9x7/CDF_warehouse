# 中免导航业务层 `:8081`

和能力层在 **同一个 nav 仓库**，文件夹 `apps/zhongmian/`。`python -m navigation` 会把 `apps/` 挂上 `sys.path`；单独 `python -m zhongmian` 时才需要 `PYTHONPATH` 含 `apps/`。

```text
Agent
  GET  /navigation/health
  POST /navigation/navigate   Idempotency-Key + {"nav_id":"AGV_L"}
    │
    ▼
zhongmian :8081     （python -m navigation 时一并拉起）
    │  校验 nav_id → 点位表 → POST :8001/goto → 轮询 /status
    ▼
navigation :8001
    └── rokae Adapter（进程内，无单独端口）
```

`adapter=tianji` 时不启 zhongmian，避免和天机 `:8081` 抢口。

点位表在 `apps/zhongmian/zhongmian.json`（和业务代码放一起）。MATRIX 站名与 `nav_id` **同名**。兼容 `target_id`。

| `nav_id`（也是底盘站名） | 含义 |
|---|---|
| `AGV_L` | AGV 左侧作业点。右手取左列商品时使用 |
| `AGV_C` | AGV 中间作业点。左手取左列、或右手取右列时使用 |
| `AGV_R` | AGV 右侧作业点。左手取右列商品时使用 |
| `SORTING_BASKET_1`～`5` | 拣选侧篮筐架第 1～5 列 |
| `REVIEW_BASKET_1`～`5` | 复核侧篮筐架第 1～5 列 |
| `REVIEW_TABLE` | 复核工作台 |

## 启动

一条命令两个口（先停掉旧的单独 zhongmian 进程）：

```bash
cd /home/alin/Code/working/work/nav
export PYTHONPATH="$PWD"
python3 -m navigation --config config/navigation.json --log-level DEBUG
```

日志里应有 `zhongmian started: http://0.0.0.0:8081` 和 `HTTP listening on 0.0.0.0:8001`。

关掉：`navigation.json` 里 `"zhongmian": {"enabled": false}`。只跑中免对接（调试用）：`PYTHONPATH="$PWD/apps" python3 -m zhongmian`。

```bash
curl -sS http://127.0.0.1:8081/navigation/health
curl -sS -X POST http://127.0.0.1:8081/navigation/navigate \
  -H 'Content-Type: application/json' \
  -H 'Idempotency-Key: task-001:nav' \
  -d '{"nav_id":"AGV_L"}'
```

成功 `200 {"status":"SUCCEEDED"}`。失败非 2xx `{"error_code":"EXECUTION_FAILED"}`。接口字段见下文。

真机总条例（可贴飞书）：[测试条例.md](测试条例.md)。地图/站点/Gateway：[MAP_STATION_SYNC.md](MAP_STATION_SYNC.md)。

## GET `/navigation/health`

```json
{"status": "READY"}
```

nav 未就绪仍 HTTP 200：`{"status":"ERROR"}`。

## POST `/navigation/navigate`

Header 必填 `Idempotency-Key`（透传到 nav，不自动生成；缺了 400）。Body：`nav_id`（兼容 `target_id`）。

```json
{"status": "SUCCEEDED"}
```

内部：`AGV_L` → `/goto` 的 `station_id=AGV_L` → 轮询 `/status/{id}`。能力层启动拉一次当前图站点，每次移动先校验站名在不在表里。同一把 key 再发同一站点直接返回上次结果。未知 `nav_id`、缺字段、nav 连不上：非 2xx。

```json
{"error_code": "EXECUTION_FAILED"}
```

`GET /` 和 `/json/version` 的 404 可忽略（浏览器探活）。
