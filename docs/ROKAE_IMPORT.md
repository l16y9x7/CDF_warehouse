# ROKAE AGX 源码导入

来源：AGX `/home/admin/mui/rokae_web_control`，2026-09-24 11:02:35 快照；另包含同机 `/home/admin/mui/arm_motion_control` 配套 Python 规划库。以当时现场磁盘源码为准，包含 9 月 23 日 21:01 的后续软管改动，而非早期本地暂存版本。

## 目录映射

`tools/rokae_bundle_manifest.json` 逐文件记录仓库来源、运行包目标、初始 SHA-256 和执行权限。原主项目 161 个文件按职责分布在 `agent/camera/estimation/manipulation/navigation/pose` 下的 `rokae/` 子目录，规划库 15 个文件放在 `manipulation/arm_motion_control/`。构建后共 176 个源码/测试/资源/文档文件，另生成一份 `BUNDLE_MANIFEST.json`。

只调整存放位置，原文件内容保持不变。`rokae_web` 各部分仍通过相对导入组成同一个包；通过装配步骤避免更改业务代码和启动入口。目录划分不等于进程拆分，也不会新增 SDK 连接。后续若要把模块改为可独立安装的软件包，应单独处理接口边界。

## 未公开的现场数据与外部依赖

- `config.json`、`memory_points/hardware.json`、三份现场标定 JSON 不提交；保留 `config.example.json` 和独立业务动作快照。
- 日志、照片、录像、临时状态、历史备份、凭据不提交。
- 厂家 xCore SDK、机器人 URDF、ROS2/底盘工作区、外部 `vision` 项目与推理服务不复制。这些仍按 AGX 现有环境配置。
- 运行时依赖 Python 3.10+、NumPy；相机处理与相关离线测试需要 OpenCV。真实相机/底盘适配需要现场 ROS2 消息与服务包。右腕发布脚本需要 RealSense 和外部视觉项目。
- `arm_motion_control` 原本位于主项目旁边；运行包自带同名库，可由运行包根目录直接导入，不需要测试从现场目录借用 Python 源码。URDF 仍从明确配置的外部路径读取。

## 部署约定

先构建到新目录，核对 `BUNDLE_MANIFEST.json`，再按现场部署方式进行差异安装；不要把输出目录盲目覆盖到运行目录，也不要删除未纳入仓库的现场配置。

systemd 文件保留既有绝对路径。日常网页/接口用 `mui.target` 启停，SDK 上报、底盘和相机继续独立常驻。修改它们时需要另行维护，不属于本次源码导入。

原主项目 README 保留历史说明；当前服务生命周期以 `manipulation/rokae/SERVICE_LIFECYCLE.md` 为准。构建工具不执行任何部署或服务命令，当前 AGX 上的软件与服务不受此次仓库提交影响。

## 验证记录

首次构建使用 `--verify-import` 校验 176 个导入文件与快照字节一致；Python 语法、JavaScript 语法、Shell 语法检查通过。

- 构建工具 5 项测试通过，覆盖目录还原、初始哈希检查、拒绝覆盖、重复目标与越界路径拒绝。
- AGX 隔离目录中运行主项目 481 项 Python 测试：473 通过，4 失败，4 跳过。未把现场主项目目录加入 `PYTHONPATH`；只读取部分测试明确依赖的现场 URDF，不连接真实控制器。
- 4 项跳过均来自需要外部 `vision` 源码的相机协议测试；该项目未纳入此仓库。
- 4 项失败已在未整理的原始源码快照中逐项复现：`test_box_clearance` 两项、`test_pose_estimation` 一项仍按 170 mm 工具长度计算，现行代码为 175 mm；`test_pose_protocol` 一项仍期望软管不支持，现行代码已经支持。
- 13 组 Node.js 网页测试中 12 组通过，`test_left_box_ui.js` 在原始快照和装配结果上均失败：测试提供旧 `scan-sequence-v4` 状态，当前页面要求 v5。

本次不改写这些历史测试，也不修改现行业务逻辑来迎合旧断言。完整测试命令会显式报告上述失败，不能将这次验证表述为全部回归通过。
