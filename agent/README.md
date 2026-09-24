# Agent 接口适配

`rokae/rokae_web/agent_*.py` 是 AGX 共享业务核心中的 HTTP 适配与完整预规划逻辑，不是新的独立 Agent 调度进程。它与运控代码共用任务互锁、取消事件和 SDK broker。

- [AGENT_API.md](rokae/AGENT_API.md)：8082 `/pose`、8086 `/manipulation`，包含瓶、盒、软管的抓取/扫码/放置。
- [MANIPULATOR_STATE_API.md](rokae/MANIPULATOR_STATE_API.md)：schema 1 缓存状态接口与设备侧 Python 接入。
- [UPPER_BODY_API.md](rokae/UPPER_BODY_API.md)：8092 兼容状态接口。
- `rokae/rokae_robot_state/`：只通过 HTTP 拉取缓存的独立 Python 客户端，不建立 SDK 连接。

完整运行与测试先执行仓库根目录的打包命令。单独使用状态客户端可将本目录的 `rokae/` 加入调用方 `PYTHONPATH`；该客户端只依赖 Python 标准库。
