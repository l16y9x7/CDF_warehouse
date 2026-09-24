# 业务接口与网页 — 8082 / 8086 / 8091

本目录保存由 `mui.target` 管理、日常开发一起启停的业务层。

| 入口 | 端口 | 职责 |
| --- | --- | --- |
| `rokae_web_control/control_runtime.py` | 8082 | `/pose`：姿态准备、相机坐标变换 |
| 同一 `control_runtime.py` 进程 | 8086 | `/manipulation`：抓取、扫码、放置 |
| `rokae_web_control/server.py` | 8091 | 网页入口及转发到业务核心的操作接口 |

`rokae_web/` 保存动作业务、完整预规划、取消/互锁、网页处理、外部视觉定位调用，以及业务侧的相机取图/预览/拍照接入。接口适配 `agent_*.py` 也放在这里，不按名称另放 `agent/`。动作点位 `poses/` 同样属于这一业务组。

`system/mui-control.service`、`mui-web.service`、`mui.target` 及启动脚本保留原部署行为。基础 SDK、共享类型与状态采集从 `pose` 一份源码装配，不复制或另开 SDK 连接。网页通过 `.run/control.sock` 访问业务核心；业务核心通过 `.run/sdk.sock` 访问常驻 SDK。

先从仓库根目录生成完整运行目录，再运行或测试。详见 [动作接口](rokae_web_control/AGENT_API.md) 和 [服务生命周期](rokae_web_control/SERVICE_LIFECYCLE.md)。原项目 [README](rokae_web_control/README.md) 保留为功能参考，其中早期底盘代启动描述以最新生命周期文档为准。

相机 8085、远端定位 `/infer` 和底盘服务作为外部依赖保留。本次未收相机驱动、相机权限安装脚本或相机服务定义；相关部署由相机服务维护方负责。网页现有相机管理调用仍需现场已有服务及固定授权。
