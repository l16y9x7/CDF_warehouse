# 上海中免前置仓

## ROKAE 服务目录

按服务生命周期组织本次 AGX 运控源码：

| 目录 | 服务 / 端口 | 职责 | 生命周期 |
| --- | --- | --- | --- |
| [pose](pose/README.md) | `mui-sdk.service` / **8092** | 基础运控、唯一 SDK 连接、状态采集与上报 | 开机常驻 |
| [manipulation](manipulation/README.md) | `mui-control.service` / **8082、8086**；`mui-web.service` / **8091** | 姿态准备、抓取/扫码/放置、网页 | `mui.target` 统一启停 |

8082 的路由虽然叫 `/pose`，其代码仍归 `manipulation`，因为它属于动作业务。`pose` 中的底层类型、校验、SDK 协议和运动学代码由业务服务复用，只维护一份。

相机服务独立维护，本次不导入 8085 服务、相机驱动、相机部署文件，不改 `camera/`。`manipulation` 保留运行必需的相机接口调用、预览和拍照逻辑。其他已有业务目录不作调整。

## 生成现有运行目录

本次只整理源码位置，保留原有 `rokae_web` 包、入口和运行行为。由于共享包分别归档在两个服务目录，先在仓库根目录装配：

```bash
python3 tools/build_rokae_bundle.py
```

Python 3.10+；打包工具仅用标准库。输出 `.build/rokae_web_control/`，恢复完整 Python 包、业务动作点位、网页、测试和服务定义。基础库 `arm_motion_control/` 也放入输出根目录，可直接导入。输出目录存在时拒绝覆盖，再次构建可指定新的 `--output`。

初始导入的逐文件核验：

```bash
python3 tools/build_rokae_bundle.py --verify-import --output .build/import-check
```

日常修改后使用不带 `--verify-import` 的构建命令。新增文件登记到 [装配清单](tools/rokae_bundle_manifest.json)。构建命令只生成新目录，不安装到 AGX，不连接硬件或启停服务。

## 验证与部署

离线测试需要 Linux、NumPy、OpenCV；网页脚本测试使用 Node.js：

```bash
python3 -m unittest discover -s tools/tests -v
python3 tools/build_rokae_bundle.py --output .build/test-runtime
cd .build/test-runtime
PYTHONPATH="$PWD:$PWD/tests" python3 -m unittest discover -s tests -p 'test_*.py'
for test_file in tests/test_*_ui.js; do node "$test_file" || exit; done
```

历史测试中有尚未更新的断言，具体结果见 [服务归档记录](docs/ROKAE_SERVICE_LAYOUT.md)。本次保留原业务和测试内容，不通过修改动作行为消除旧断言。

部署时保留现场配置、标定和可编辑记忆点。独立业务动作快照保存在 `manipulation/rokae_web_control/poses/`，只适用于对应机器人。厂家 SDK、URDF、相机/视觉/底盘外部服务仍按现场环境提供。详细服务边界见 [SERVICE_LIFECYCLE.md](manipulation/rokae_web_control/SERVICE_LIFECYCLE.md)。

AGX 日常开发只启停业务组：

```bash
systemctl --user start mui.target
systemctl --user restart mui.target
systemctl --user stop mui.target
```

SDK 与上报独立常驻；若修改其通信或采集行为，另行安排 `mui-sdk.service` 的维护。相机与底盘不纳入日常业务组。
