# 底盘接入

`rokae/start_chassis_node.sh` 保存现有底盘节点启动入口，用于独立部署/诊断。正常开机运行由外部 `sr-amr-control.service` 管理，网页和接口不会代启或清理底盘节点。

目前手动遥控、急停解除、避障开关和状态订阅仍在共享模块 `manipulation/rokae/rokae_web/backends.py` 的 `Ros2ChassisBackend` 中。该文件同时包含机器人后端，此次归档保留完整文件，没有为目录划分而修改行为。

厂家 ROS2 工作区、导航算法、地图及配置不属于此次运控源码导入。调用方先完成根目录装配，并在现场使用已配置的 ROS2 环境。
