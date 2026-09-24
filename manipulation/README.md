# ROKAE 运控

- `rokae/rokae_web/`：抓取、扫码、放置、MoveL、夹爪/吸盘、SDK broker、状态采集、网页与共享业务核心。
- `rokae/control_runtime.py`：网页后台与两组 Agent 接口共用同一业务实例。
- `rokae/hardware_service.py`：常驻 SDK 连接所有者与上报。
- `rokae/server.py`、`rokae/static/`：网页代理与前端。
- `arm_motion_control/`：URDF 运动学、躯干保护、规划与独立诊断工具；来自 AGX 同名配套库。
- `rokae/system/`：运控及网页 systemd 定义，仅作为部署来源保存。

该目录中的 `rokae_web` 是完整包的一部分，请先从仓库根目录打包。原项目 [README](rokae/README.md) 保留作功能参考；服务启停以 [SERVICE_LIFECYCLE.md](rokae/SERVICE_LIFECYCLE.md) 为准，原 README 的早期底盘代启动描述已被新生命周期规则取代。

本次导入不修改运动逻辑。固定动作点位、姿态相关模块在 `pose`；相机、定位及 Agent 适配放在各自目录，装配后恢复原来的相对导入。
