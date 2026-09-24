# 常驻基础运控与状态服务 — 8092

本目录按 `mui-sdk.service` 的生命周期归档，负责开机后持续运行的基础运控和上报。名称 `pose` 不对应业务端口 8082。

- `rokae_web_control/hardware_service.py`：唯一 SDK 所有者入口。
- `rokae_web_control/rokae_web/sdk_broker.py`、`sdk_wire.py`、`hardware_owner.py`：进程互斥、Unix socket 调用协议、命令会话。
- `rokae_web_control/rokae_web/manipulator_*.py`、`telemetry*.py`：状态采集与 8092 查询接口。
- `rokae_web_control/rokae_robot_state/`：供其他设备程序使用的 HTTP 缓存客户端，不新建 SDK 连接。
- `rokae_web_control/system/mui-sdk.service`：常驻服务定义。
- `arm_motion_control/`：基础运动学、躯干保护和规划工具。

SDK 入口需要的配置、日志、硬件适配、坐标/关节校验、底层运动辅助等依赖也集中在这里。共享文件原样保留；例如 `backends.py` 同时含机器人和 ROS2 底盘适配，业务服务复用该文件，不复制第二份。

8092 只提供 `/health`、`/api/telemetry/manipulator-state` 及兼容 `/api/telemetry/upper-body`。运动命令仍由业务服务经 `.run/sdk.sock` 调用，不能向 8092 下发动作。

接口说明：[状态协议](rokae_web_control/MANIPULATOR_STATE_API.md)、[兼容上半身状态](rokae_web_control/UPPER_BODY_API.md)。先按根目录 README 装配完整运行包；此归档不自动部署或重启服务。
