# 视觉定位调用

`rokae/rokae_web/pose_estimation.py` 调用现有外部视觉服务，`pose_protocol.py` 处理返回协议，`pose_targets.py` 将结果转换为运控目标；`rokae/tools/transform_head_camera_point.py` 提供坐标换算工具。

这里只收录 AGX 运控侧的调用和转换代码。视觉推理模型及 `/infer` 服务由其他项目提供，未放入本目录或 `perception`。程序继续使用现有字段、单位、有限数值和动作几何约束，不额外增加视觉质量门。

实际相机外参、机器人 URDF 和外部视觉服务地址使用现场配置。先从仓库根目录生成完整运行包，再运行相关测试。
