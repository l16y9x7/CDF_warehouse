# 姿态与坐标

`rokae/rokae_web/` 包含记忆点存储/回放、头部运动学、左右肩 SDK 世界系与躯干坐标转换、独立动作点位读取。

`rokae/poses/` 是现有机器人使用的独立业务动作快照，包括 L2 动作、软管预抓取高度、软管扫码与历史 A 点。现场可编辑记忆点库 `memory_points/hardware.json` 和手眼标定不进入公开仓库。

8082 `/pose` 的 HTTP 适配在 `agent/rokae/rokae_web/agent_http.py`；它仍由 `mui-control.service` 提供，不另起一个姿态服务。

先执行根目录打包命令再运行或测试，坐标约定见 [原项目文档](../manipulation/rokae/README.md)。
