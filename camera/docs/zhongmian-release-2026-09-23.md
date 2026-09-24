# 中免现场代码归档（2026-09-23）

来源：ROBOT_011，192.168.130.105:/home/admin/vision，2026-09-23 16:36 左右只读采集。基线 GitLab `feat/rokae-helios-adapter` 的 `d23aab2`。现场目录没有 `.git`；本次补入与远端不同的 13 个源码、脚本、测试和 ROS 配置文件，保留现场逻辑，仅将 CRLF 行尾统一为 LF。本次是现场版本发布，未对机器人修改或重启。

## 当前接口（优先于较早的验收文档）

Owner 主入口 `:8085`，Media `:8005`，Camera Adapter `:8003`。

- `GET /camera/list` 返回相机数组；`GET /list` 保留 `{ok,cameras}` 格式。
- 对外相机名 `head` / `left_wrist` / `right_wrist`；`hand_wrist`、`hand_right` 仍作为右腕输入别名。
- `GET /camera/snapshot?camera=head&type=color` 返回 JSON 路径与元数据；旧的二进制图片客户端改用 `/camera/frame?camera=head`。
- `GET /camera/rgbd?camera=head` 返回 RGB-D 文件路径、标定和时间字段；深度接口限头部。
- `GET /camera/stream?camera=head&type=color` 返回 MJPEG；深度预览使用 `type=depth&format=preview`。
- 头部使用软件 D2C，Owner 消费 `/camera/head/synced/*`；按相等的同步话题时间戳匹配。`same_shot` 不代表硬件同步曝光，原始深度采集时间差未由当前接口完整保留。
- 抓拍文件路径属于机器人本地 `/shared/frames`，跨机消费者需要已有共享目录或文件传输方案。

## 现场配置与启动边界

现场 `config/vision.json`、凭据、运行数据、备份和帧文件不上传；仓库已有默认配置并非此机器的现场配置。重新拉取代码不等于复制现场配置，应保留机器已有配置。

三路彩色启用，配置目标均为 1280×720@15；仅头部要求深度和同步。手腕 `enable_depth=false`、`require_depth=false`、`require_synced=false`。Media 仅开启 head/slot1、hand_left/slot2，右腕 slot3 关闭。现场使用 NVIDIA GStreamer 推流后端。

现场活动左腕服务名 `vision-left-wrist-rgb.service`，实际执行 Conda `nav` 中的 `vision.rokae_runtime.realsense_color_publisher --camera hand_left`。仓库本次收录的同名 service 模板及 `left_wrist_rgb.py` 是现场遗留文件，不能用它覆盖当前活动 unit。`scripts/head-stream-install.sh` 仅支持头部单路，使用 `/usr/bin/python3`，不用于重装当前多相机 Conda 部署。当前机器 systemd 配置未在本次更改。

## 验证与已知问题

本地命令：`PYTHONPATH=. python -m unittest discover -s tests -v`，Python 3.10 nav 环境，163 项测试，154 通过、5 failures、4 errors，未宣称回归全绿。

- 右腕输出名由 hand_wrist 改为 right_wrist：3 项旧断言失败/报错。
- 头部 SW D2C 替代关闭对齐：1 项旧参数断言失败。
- 新健康检查要求头部深度，fake 仅彩色：2 项健康状态测试失败。
- snapshot 改为需要 type 的 JSON 接口：1 项旧 JPEG 测试报错。
- 遗留左腕测试依赖现场 config/vision.json，并引用旧 health 数据字段：2 项报错。

现场只读检查：头部 RGB-D 在线；头部推流计数增长。左腕存在 ROS_IMAGE_STALE 与推流重复重建；12 秒订阅窗口观测头部约13.56Hz、左腕约1.08Hz、右腕约10.75Hz，受订阅链路与采样窗口影响，不能当作设备曝光帧率。左腕低帧率尚未修复；没有本轮云端回拉验收。
