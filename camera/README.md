# 相机接入

`rokae/rokae_web/` 包含 HTTP/ROS2 相机适配、图像采集与预览、扫码图像记录以及三路摄像头服务重启管理。`rokae/camera_right_rgb.py` 是右腕 RGB-only 发布脚本；`rokae/tools/` 保存手眼标定工具，`rokae/system/` 保存相关部署定义和固定权限安装脚本。

头部/左腕驱动和统一视觉服务由 AGX 上的外部视觉项目维护，本次没有复制该项目。右腕脚本仍依赖其 `vision.rokae_runtime` 包、现有相机配置、ROS2 和 RealSense 环境。具体部署路径见保存的 service 文件；打包不会安装或启动它们。

扫码照片、录像、相机序列号配置、手眼标定结果与日志留在 AGX。相机后端与网页共用代码先通过根目录脚本装配后运行。
