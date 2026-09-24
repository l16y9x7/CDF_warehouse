# 通用机器人 Agent

本项目按照 `主方案文档.md` 实现 Sorting 与 Review 机器人任务编排，分为四层：

```text
Workflow -> Skill -> Capability -> External HTTP Module
```

- Capability 封装导航、视觉、相机、估姿和操作模块的 HTTP/JSON 契约。
- Skill 组合一次局部机器人操作，并统一事件与错误语义。
- Workflow 编排单个异步业务任务并持久化节点状态。
- Runtime 负责 SQLite 去重、异步执行、回调重试和进程中断封存。

## 安装和启动

后台启动、停止和重启：

```bash
./scripts/start.sh          # 后台启动 Agent（默认 0.0.0.0:8090）
./scripts/stop.sh           # 停止 Agent
./scripts/restart.sh        # 重启 Agent

./scripts/start-mocks.sh    # 后台启动 Mock（默认 28081-28088，不保存日志）
./scripts/stop-mocks.sh     # 停止 Mock
./scripts/restart-mocks.sh  # 重启 Mock
```

两套脚本互不影响：`start.sh` 不会启动 Mock，`start-mocks.sh` 也不会启动 Agent。
脚本会优先使用项目 `.venv` 中的可执行文件，也可分别用 `AGENT_SERVER_EXECUTABLE`、
`AGENT_MOCK_EXECUTABLE` 指定。Agent PID 默认写入 `run/agent-server.pid`，Mock PID
默认写入 `run/agent-mocks.pid`，均可由 `AGENT_PID_FILE`、`AGENT_MOCK_PID_FILE`
覆盖。后台进程的控制台输出一律丢弃；Agent 项目日志仍由自身日志系统写入
`logs/agent.jsonl` 和日志索引数据库，Mock **不保存日志**。停止默认等待 30 秒后强制
退出，可通过 `AGENT_STOP_TIMEOUT` / `AGENT_MOCK_STOP_TIMEOUT` 调整。`AGENT_HOST`、
`AGENT_PORT` 以及 `AGENT_MOCK_*` 等环境变量会原样传递给对应后台进程。

`agent-mocks` 和 `agent-server` 是安装后的控制台脚本，不在系统 PATH 里。未激活
`.venv` 时直接输入这些命令会报 `command not found`。请先安装：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e .
```

需要前台排查时，可直接运行虚拟环境中的命令：

```bash
./.venv/bin/agent-server    # 前台启动 Agent
./.venv/bin/agent-mocks     # 前台启动 Mock
```

Mock 端口使用独立的 `AGENT_MOCK_<MODULE>_PORT` 环境变量覆盖，例如
`AGENT_MOCK_CAMERA_PORT=29085 ./scripts/start-mocks.sh`。正式模块地址仍由
`configs/modules.yaml` 配置，不与 Mock 端口共用。

模块地址通过 `configs/modules.yaml` 配置；Workflow 固定工作手通过
`configs/workflows.yaml` 配置，默认使用 `RIGHT`。当前 Sorting 与 Review 均只接入
标准抓取；灵巧手和 VLA 仅保留 Capability，未接入 Skill、Workflow 和任务前健康检查。

## 正式任务接口

Agent 提供三个异步业务入口和一个机器人级终止入口：

- `POST /agent/sorting/item`：单件商品拣选。
- `POST /agent/sorting/finish`：目标篮筐推筐收尾。
- `POST /agent/review`：取筐、逐件条码盘点和差异汇总。
- `POST /agent/terminate`：终止当前任务，无请求体且无需 `task_id`。

受理成功返回 `{"task_id":"...","status":"ACCEPTED"}`。任务受理后会通过
`callback_url` 按技能实时推送进度。回调顶层固定为 `task_id`、`status`、`info`：
`status` 使用 `ACCEPTED`（收到）、`RUNNING`（进行中）、`FAILED`（失败）、
`CANCELLED`（已终止）和 `SUCCEEDED`（完成），技能、进度和错误说明放在 `info` 中并
使用中文。动作结果未知等
内部 `WAITING_CONFIRMATION` 状态对外映射为 `FAILED`，`info` 会说明需要人工确认。
同一个全局唯一 `task_id` 重复提交不会重复执行。

每个 Agent 进程只控制一台机器人，任意时刻只运行一个 Workflow。机器人忙碌时提交
不同任务返回 HTTP 409 和 `ROBOT_BUSY`。终止接口有任务时返回
`{"task_id":"...","status":"TERMINATION_REQUESTED"}`，空闲时返回 HTTP 409 和
`NO_ACTIVE_TASK`。终止是协作式的：当前同步 Capability 调用返回或超时后停止后续节点，
不等同于硬件急停。

```json
{"task_id":"sorting-item-001","status":"RUNNING","info":{"skill":"导航","progress":"正在执行导航"}}
```

任务直接接收独立行列字段，不接收组合货位或前置篮筐定位任务。商品 ID 等于条码；
Sorting 会校验识别条码，Review 将识别结果直接作为实际 SKU。

## 调试台

`agent-server` 同时提供 `http://127.0.0.1:8090/debug/` 调试台，可在 Mock 与真机
目标之间切换，独立调用当前方案中的 Capability、Skill 和三个 Workflow。物理动作
必须使用服务端签发、绑定完整请求且一次有效的确认票据。
活动 Workflow 可在执行详情中直接点击“终止任务”。
直接相机调用以及 Skill、Workflow 内部产生的彩色图和深度图会显示在执行详情的图片
画廊中；Workflow 运行期间随轮询更新，原始 NPY 深度数据会转换为灰度预览图。

调试运行、事件与请求历史保存在 Agent SQLite 数据库中。用户下单或直接调用
`/agent/sorting/*`、`/agent/review` 产生的 Workflow 也会写入同一份执行历史，可在调试台
点击查看结果、调用链、原始事件和图片。执行历史同时列出 Mock 与真机记录。调试台默认无认证，只应部署

在受信任的开发或测试网络。

## 用户下单页

`agent-server` 同时提供 `http://127.0.0.1:8090/orders/` 用户下单页。用户可以选择商品、
数量和目标篮筐，提交后查看每件商品的抓取与入筐进度。商品清单来自
`configs/products.yaml`，对应 SKU 还必须存在于 `configs/workflows.yaml`。

订单会将每件商品依次提交给现有 Sorting Workflow，全部成功后即完成；同一时间
只允许一个活动订单。步骤失败会自动重试一次，仍失败时页面允许人工重试或取消。订单、
执行单元和实时事件与 Agent 任务保存在同一个 SQLite 数据库的独立表中。

执行详情中的“调用链”按 `Workflow → Skill → Capability` 展示输入、输出、错误和耗时；
Workflow 节点作为 Skill 标签显示，任务提交时的健康检查单独归入“执行前检查”。调用追踪
默认保留 7 天、最多使用约 200 MiB，可通过 `AGENT_TRACE_RETENTION_DAYS` 和
`AGENT_TRACE_MAX_MB` 调整。认证信息会强制脱敏，图片和深度数据只记录文件引用。

## 运行日志

Agent 输出 UTF-8 JSON 结构化日志。`event`、`status`、`error_code` 等机器字段保持
稳定英文，面向运维人员的 `message`、`status_text`、`error_message` 和
`suggestion` 使用中文。API 请求、Workflow、节点、Skill、Capability 和回调发送通过
`request_id`、`task_id`、`run_id` 关联。

原始日志默认写入 `logs/agent.jsonl` 并按 50 MiB 滚动；`INFO` 及以上日志同时索引到
业务数据库同目录下的 `agent-logs.db`。调试台底部“运行日志”区域支持按级别、任务、
运行和中文说明查询。日志仅用于排障，任务状态仍以 `agent-tasks.db` 为准。日志不会记录
完整请求体、回调地址、票据、图片或二进制内容，敏感字段会自动脱敏。

可使用以下环境变量调整日志行为：

```bash
AGENT_LOG_LEVEL=INFO             # DEBUG / INFO / WARNING / ERROR
AGENT_LOG_DIR=logs               # JSONL 日志目录
AGENT_LOG_CONSOLE=true           # 是否同时输出到 stdout/stderr
AGENT_LOG_RETENTION_DAYS=7       # 最长保留天数
AGENT_LOG_MAX_TOTAL_MB=500       # 项目日志总预算，索引使用其中约 20%
AGENT_LOG_QUEUE_SIZE=10000       # 异步日志队列容量
AGENT_LOG_DATABASE_PATH=agent-logs.db  # 可选：覆盖索引数据库路径
```

日志队列、文件或索引异常不会改变机器人任务结果。部署时建议由 systemd/journald 额外
采集控制台输出；调试台仍然没有身份认证，不应暴露到公网。

## 测试

```bash
PYTHONPATH=src python -m unittest discover -s tests -v
```

能力契约、业务细节和端口表以 `主方案文档.md` 为准。
