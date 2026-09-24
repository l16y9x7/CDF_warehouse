# 上海中免前置仓

按业务职责组织机器人源码。当前收录 2026-09-24 从 AGX 获取的 ROKAE 运控、网页、接口、相机适配和状态上报代码。

## 目录

| 目录 | 已收录内容 |
| --- | --- |
| [agent](agent/README.md) | `/pose`、`/manipulation` 的 HTTP 适配、接口预规划、状态缓存客户端与接口文档 |
| [manipulation](manipulation/README.md) | SDK 唯一连接所有者、抓取/扫码/放置、夹爪/吸盘、网页、运行入口与运控基础库 |
| [pose](pose/README.md) | 记忆点、姿态回放、坐标转换、独立动作点位 |
| [camera](camera/README.md) | 相机访问、拍照与预览、右腕 RGB 发布脚本、相机重启管理 |
| [estimation](estimation/README.md) | 外部视觉定位调用、协议处理和目标坐标转换 |
| [navigation](navigation/README.md) | 现有底盘启动入口及 ROS2 对接说明 |
| `perception` | 预留；外部视觉推理实现未包含在此次运控项目导入中 |
| `hand_vla`、`gripper_vla` | 预留；本次没有相应的 VLA 模型或训练代码 |

这些目录是**源码职责划分**，不代表各自新增一个运行服务。现有 `rokae_web` Python 包跨目录归档，通过打包步骤恢复完整包；不要直接把各个 `rokae/` 子目录分别加入 `PYTHONPATH`。

## 生成运行目录

要求 Python 3.10 或更新版本，打包步骤仅使用标准库：

```bash
python3 tools/build_rokae_bundle.py
```

输出到 `.build/rokae_web_control/`，恢复原有入口、`rokae_web/`、`tests/`、`static/`、`poses/`、`system/` 和状态客户端。运控依赖 `arm_motion_control/` 一并放入输出根目录，Python 可以直接导入。输出目录已存在时命令拒绝覆盖；再次构建请选择新的 `--output` 目录。

首次导入可额外核对所有原文件的 SHA-256：

```bash
python3 tools/build_rokae_bundle.py --verify-import --output .build/import-verification
```

后续开发直接修改对应源码。`--verify-import` 用于核验初始导入，正常修改后使用不带该参数的构建命令。新增文件须登记到 [装配清单](tools/rokae_bundle_manifest.json)，旧文件只保留一份来源，不维护第二份复制源码。

## 验证

Linux 上安装测试依赖 NumPy、OpenCV；网页脚本测试还需要 Node.js。以下均为离线假硬件测试：

```bash
python3 -m unittest discover -s tools/tests -v
python3 tools/build_rokae_bundle.py --output .build/test-runtime
cd .build/test-runtime
PYTHONPATH="$PWD:$PWD/tests" python3 -m unittest discover -s tests -p 'test_*.py'
for test_file in tests/test_*_ui.js; do node "$test_file" || exit; done
```

部分历史运动学测试依赖现场 URDF，详情与实际导入验证结果见 [ROKAE 导入说明](docs/ROKAE_IMPORT.md)。未把厂家 SDK、URDF 和其他项目的视觉/底盘实现复制进本仓库。

## 部署与服务边界

构建脚本**只生成新目录**，不安装文件、不启动程序、不重启服务。源码归档不会改变 AGX 当前运行版本。

- `mui-sdk.service`：唯一 SDK 连接所有者及状态上报，开机常驻。
- `mui-control.service`：共享业务核心，提供 8082 `/pose` 和 8086 `/manipulation`。
- `mui-web.service`：8091 网页代理。
- `mui.target`：只管理网页和接口；底盘、相机、SDK 保持独立。

正式部署保留 AGX 原有 `config.json`、手眼标定和现场记忆点；这些文件不在公开仓库中。独立业务动作快照在 `pose/rokae/poses/`，有明确机器人适用范围，不能当作其他机器人的通用标定。详见 [服务生命周期](manipulation/rokae/SERVICE_LIFECYCLE.md) 与 [导入说明](docs/ROKAE_IMPORT.md)。

日常命令仍在 AGX 执行：

```bash
systemctl --user start mui.target
systemctl --user restart mui.target
systemctl --user stop mui.target
```

涉及 SDK/采集器、底盘或相机的变更，另行安排对应服务维护。不要直接运行仓库中的诊断或恢复脚本来验证源码归档。
