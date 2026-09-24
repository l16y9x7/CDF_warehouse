# Vision

机器人视觉服务，提供相机取图、RGB-D 抓拍与视频推流，支持天机、珞石和模拟相机。采集由相机 Owner 或厂家驱动统一管理，不重复打开设备；本项目不做估姿。

## 服务

| 进程 | 默认端口 | 职责 |
| --- | --- | --- |
| Camera | `8003` | 相机状态与取图入口 |
| Media | `8005` | 视频推流与启停控制 |
| 珞石 Owner | `8085` | 图像、RGB-D 抓拍与预览 |

Camera / Media 通过 Adapter 接入相机；Media 也支持直接订阅 ROS 彩色话题。具体能力与端口以设备配置为准。

## 快速启动

使用 Python 3.10+，按设备选择 [配置样例](config/) 并配置 `config/vision.json`，或通过 `VISION_CONFIG` 指定配置文件。

```bash
python -m pip install -r requirements.txt
bash vision.sh start
bash vision.sh status
bash vision.sh stop
```

已有 ROS / systemd 部署按接入文档管理服务，避免重复启动采集。

## 日志与测试

`vision.sh` 默认将日志写入 `runtime/logs/`：`camera.log`、`media.log`、`rokae_owner.log`（启用时）。

在仓库根目录运行测试：

```bash
PYTHONPATH=. python -m unittest discover -s tests -v
```

## 文档

- [珞石接入与部署](docs/rokae-vision.md)
- [相机接口说明](docs/camera-capture-api-3.13-merged.md)
- [中免当前版本与接口变更](docs/zhongmian-release-2026-09-23.md)
