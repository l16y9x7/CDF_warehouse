上海中免前置仓

## Ubuntu 一键启动 perception 和 estimation

仓库根目录的 `start_services.sh` 按两个服务的 README 启动后台进程：

| 服务 | 默认端口 | 健康检查 |
| --- | --- | --- |
| perception | 25546 | `/perception/health` |
| estimation | 25540 | `/health` |

### 首次安装

使用 Python 3.10 及以上版本（例如 Ubuntu 22.04 / 24.04 自带的 Python）：

```bash
sudo apt-get update
sudo apt-get install -y python3 python3-venv python3-pip curl iproute2 util-linux
cd /path/to/CDF_warehouse
bash start_services.sh install
```

`install` 默认为两个服务分别创建 `.venv` 并安装各自的 `requirements.txt`；
已有 `estimation/deploy/.venv` 时会复用它。安装需要可访问 Python 包源。

### 启动和日常管理

先按照 [perception README](perception/README.md) 部署 SAM3 并等待模型加载完成，
默认地址为 `http://127.0.0.1:25541/api/v1/segment`。
处理篮筐时，还需按照 [estimation README](estimation/README.md) 启动 FoundationPose，
默认地址为 `http://127.0.0.1:25550/infer`。
上述模型服务的代码不在本仓库内，脚本仅管理 perception 和 estimation。

```bash
bash start_services.sh          # 一键启动两个服务，先 estimation 后 perception
bash start_services.sh status  # 查看进程及 HTTP 健康状态
bash start_services.sh stop    # 停止两个服务
bash start_services.sh restart # 重启两个服务
```

脚本自动定位仓库目录，可从任意工作目录调用。日志与 PID 记录保存在
`.runtime/perception-estimation/`，服务退出终端后仍在后台运行。
健康检查仅确认这两个 HTTP 服务已就绪，不代表 SAM3 / FoundationPose 模型加载或真实推理成功。
重复启动会复用本脚本已启动且健康的服务；端口被其他进程占用时会报错，不会终止其他进程。
本次启动失败时，会停止本次新启动的服务，保留此前已运行的服务。

```bash
tail -f .runtime/perception-estimation/perception.log
tail -f .runtime/perception-estimation/estimation.log
```

### 使用已有 Conda / venv 环境或自定义配置

启动时优先使用指定的 Python，其次使用服务的 `.venv`（estimation 也支持
`deploy/.venv`），最后使用当前环境的 `python3`。因此也可以直接指定 README 中的
`slim` 环境和现有 estimation 虚拟环境，无需在同一终端切换 Conda 环境：

```bash
export PERCEPTION_PYTHON="$HOME/miniconda3/envs/slim/bin/python"
export ESTIMATION_PYTHON="/path/to/estimation/deploy/.venv/bin/python"
export SAM3_URL="http://127.0.0.1:25541/api/v1/segment"
bash start_services.sh
```

如在设置 Python 路径后运行 `install`，依赖会安装到指定环境中。

| 环境变量 | 默认值 / 用途 |
| --- | --- |
| `SERVICE_HOST` | `0.0.0.0`，两个服务的监听地址 |
| `PERCEPTION_PORT` / `ESTIMATION_PORT` | `25546` / `25540` |
| `SAM3_URL` | 两个服务共用的 SAM3 地址 |
| `SAM3_BACKEND` | `multipart_segment`；TRT 旧协议设为 `legacy_18003`，默认地址随之变为 `http://127.0.0.1:25551/infer` |
| `SKU_API_URL` | 自动指向本次 estimation 的本机地址和端口 |
| `BASKET_FP_URL` | 可覆盖 estimation 配置中的 FoundationPose 地址 |
| `START_TIMEOUT` / `STOP_TIMEOUT` | 每个服务等待启动 / 停止的秒数，默认 `60` / `20` |

其他服务环境变量会原样传入；estimation 的拟合并行度和 BLAS 线程默认值沿用原生产启动脚本，
支持环境变量覆盖。其余配置继续读取 `estimation/deploy/config.json`。
本脚本将 SAM3 协议名称分别转换为两个服务要求的值，并绕过原启动脚本里写死的机器路径。
更改端口、地址等设置后需使用相同配置执行 `restart`；后续 `status` 也应使用相同配置。
