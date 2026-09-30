上海中免前置仓

## Ubuntu 一键启动 SAM3、FoundationPose、estimation 和 perception

仓库根目录的 `start_services.sh` 按服务 README 依次启动后台进程：

| 服务 | 默认端口 | 健康检查 |
| --- | --- | --- |
| SAM3 | 25541（固定） | 默认探测 `/api/v1/segment`，可指定 `SAM3_HEALTH_URL` |
| FoundationPose | 25550（固定） | `/health` |
| estimation | 25540 | `/health` |
| perception | 25546 | `/perception/health` |

### 首次安装

使用 Python 3.10 及以上版本（例如 Ubuntu 22.04 / 24.04 自带的 Python）：

```bash
sudo apt-get update
sudo apt-get install -y python3 python3-venv python3-pip curl iproute2 util-linux
cd /path/to/CDF_warehouse
bash start_services.sh install
```

`install` 默认为 perception 和 estimation 分别创建 `.venv` 并安装各自的 `requirements.txt`；
已有 `estimation/deploy/.venv` 时会复用它。安装需要可访问 Python 包源。
SAM3 和 FoundationPose 的代码、模型权重和 GPU / Python 环境需要提前准备，`install` 不安装这些内容。

### 启动和日常管理

先按照 [perception README](perception/README.md) 准备 SAM3：默认代码目录为
`/data/steven/sam3_api`，Conda 环境为 `sam3`。脚本会激活该环境，进入 SAM3 目录，
执行 `python -u backend/app.py`，等待 HTTP 接口响应后再启动后续服务。
SAM3 默认地址为 `http://127.0.0.1:25541/api/v1/segment`，最多等待 300 秒。
如果代码目录或环境名不同，在启动前设置：

```bash
export SAM3_DIR="/path/to/sam3_api"
export SAM3_CONDA_ENV="sam3"
# Conda 不在 PATH 且不在 ~/miniconda3、~/anaconda3、/opt/conda 时：
# export SAM3_CONDA_SH="/path/to/miniconda3/etc/profile.d/conda.sh"
```

FoundationPose 使用仓库内的 [start_25550.sh](start_25550.sh)，默认部署目录为
`/data/quinn/foundationpose`，需要以下文件：

```text
/data/quinn/foundationpose/
├── current/deploy/http_server.py
├── env/bin/python
└── cad/Basket/Basket.obj
```

一键脚本调用 `start_25550.sh --foreground`，由主脚本负责后台运行、PID 和日志管理，
固定传入 `PORT=25550`。FoundationPose 的 `/health` 成功后才启动 estimation / perception，
默认最多等待 300 秒。部署位置不同可设置 `FOUNDATIONPOSE_ROOT`；Python 和模型文件还可用
`FOUNDATIONPOSE_PYTHON`、`CAD_MESH` 单独指定。

```bash
bash start_services.sh          # SAM3 → FoundationPose → estimation → perception
bash start_services.sh status   # 查看四个服务的进程及 HTTP 状态
bash start_services.sh stop     # perception → estimation → FoundationPose → SAM3
bash start_services.sh restart  # 重启四个服务
```

脚本自动定位仓库目录，可从任意工作目录调用。日志与 PID 记录保存在
`.runtime/perception-estimation/`，服务退出终端后仍在后台运行。
SAM3 README 没有定义健康接口，因此默认向分割地址发送 GET，收到 2xx 或 405 即视为接口可访问；
不会发送推理请求。若部署提供专用健康接口，可通过 `SAM3_HEALTH_URL` 指定，脚本要求其返回成功状态。
这些探测不代表模型加载或真实推理成功。
重复启动会复用本脚本已启动且健康的服务；端口被其他进程占用时会报错，不会终止其他进程。
本次启动失败时，会停止本次新启动的服务，保留此前已运行的服务。

```bash
tail -f .runtime/perception-estimation/sam3.log
tail -f .runtime/perception-estimation/foundationpose.log
tail -f .runtime/perception-estimation/perception.log
tail -f .runtime/perception-estimation/estimation.log
```

### 使用已有 Conda / venv 环境或自定义配置

perception / estimation 启动时优先使用指定的 Python，其次使用服务的 `.venv`（estimation 也支持
`deploy/.venv`），最后使用当前环境的 `python3`。因此也可以直接指定 README 中的
`slim` 环境和现有 estimation 虚拟环境，无需在同一终端切换 Conda 环境：

```bash
export SAM3_DIR="/data/steven/sam3_api"
export SAM3_CONDA_ENV="sam3"
export FOUNDATIONPOSE_ROOT="/data/quinn/foundationpose"
export PERCEPTION_PYTHON="$HOME/miniconda3/envs/slim/bin/python"
export ESTIMATION_PYTHON="/path/to/estimation/deploy/.venv/bin/python"
export SAM3_URL="http://127.0.0.1:25541/api/v1/segment"
bash start_services.sh
```

如在设置 perception / estimation 的 Python 路径后运行 `install`，依赖会安装到指定环境中。
SAM3 默认会完整激活 Conda 环境，包括其环境变量和激活钩子；如已自行准备好运行环境，
也可以设置 `SAM3_PYTHON="/path/to/env/bin/python"` 直接运行，此时跳过 Conda 激活。

如果 SAM3 已由其他方式启动、位于远程服务器，或使用外部 TRT 服务，可以关闭本地 SAM3 管理：

```bash
export START_SAM3=0
export SAM3_URL="http://your-sam3-host:25541/api/v1/segment"
bash start_services.sh
```

此模式跳过 SAM3 的启动和检查，不会接管外部 SAM3 进程。
`stop` / `restart` 仍会清理本脚本此前启动且记录在 PID 文件中的本地 SAM3。
FoundationPose 已由其他方式运行或不处理篮筐时，也可以设置 `START_FOUNDATIONPOSE=0`，
跳过 FoundationPose 的启动和检查；远程地址可通过 `BASKET_FP_URL` 指定。
停止和重启同样只处理本脚本已记录的进程，不会接管外部 FoundationPose。

复制来的 `start_25550.sh` 原有 `start / stop / restart / status` 用法仍可独立使用，
独立模式日志和 PID 位于 `$FOUNDATIONPOSE_ROOT/logs/`。
经一键脚本启动的进程统一通过 `start_services.sh stop/restart` 管理，避免混用两套 PID 文件。

FoundationPose 内部的 `SAM3_API_URL` 保留原脚本默认值 `http://127.0.0.1:25551/infer`，
可通过同名环境变量覆盖。它与 perception / estimation 使用的 `SAM3_URL` 是不同配置，
未自动替换为 `25541/api/v1/segment`，因为两者的请求协议不同。

| 环境变量 | 默认值 / 用途 |
| --- | --- |
| `SERVICE_HOST` | `0.0.0.0`，两个服务的监听地址 |
| `PERCEPTION_PORT` / `ESTIMATION_PORT` | `25546` / `25540` |
| `START_SAM3` | 默认 `1`，启动本地 SAM3；设为 `0` 使用外部服务 |
| `SAM3_DIR` | `/data/steven/sam3_api`，应包含 `backend/app.py` |
| `SAM3_CONDA_ENV` / `SAM3_CONDA_SH` | 环境名默认 `sam3`；可指定 Conda 激活脚本路径 |
| `SAM3_PYTHON` | 可选，直接指定 SAM3 Python，跳过 Conda 激活 |
| `SAM3_URL` | 两个服务共用的 SAM3 地址 |
| `SAM3_BACKEND` | `multipart_segment`；外部 TRT 设为 `legacy_18003` 并设置 `START_SAM3=0`，默认地址随之变为 `http://127.0.0.1:25551/infer` |
| `SAM3_HEALTH_URL` | 可选，部署提供的专用健康检查地址 |
| `SAM3_START_TIMEOUT` | SAM3 接口启动等待秒数，默认 `300` |
| `START_FOUNDATIONPOSE` | 默认 `1`；设为 `0` 跳过本地 FoundationPose 的启动和检查 |
| `FOUNDATIONPOSE_SCRIPT` | 默认仓库内 `start_25550.sh`；自定义脚本须支持 `--foreground` 并 `exec` 服务进程 |
| `FOUNDATIONPOSE_ROOT` | 默认 `/data/quinn/foundationpose`，FoundationPose 代码、环境和模型根目录 |
| `FOUNDATIONPOSE_PYTHON` | 默认 `$FOUNDATIONPOSE_ROOT/env/bin/python` |
| `FOUNDATIONPOSE_HEALTH_URL` | 默认 `http://127.0.0.1:25550/health` |
| `FOUNDATIONPOSE_START_TIMEOUT` | FoundationPose 等待启动的秒数，默认 `300` |
| `SAM3_API_URL` | FoundationPose 自身的 SAM3 配置，默认 `http://127.0.0.1:25551/infer` |
| `SKU_API_URL` | 自动指向本次 estimation 的本机地址和端口 |
| `BASKET_FP_URL` | 本地启动时指向 `http://127.0.0.1:25550/infer`；跳过本地启动时可指定远程地址 |
| `START_TIMEOUT` / `STOP_TIMEOUT` | estimation / perception 等待启动 `60` 秒；每个服务等待停止 `20` 秒 |

其他服务环境变量会原样传入；estimation 的拟合并行度和 BLAS 线程默认值沿用原生产启动脚本，
支持环境变量覆盖。其余配置继续读取 `estimation/deploy/config.json`。
本脚本将 SAM3 协议名称分别转换为两个服务要求的值，并绕过原启动脚本里写死的机器路径。
更改端口、地址等设置后需使用相同配置执行 `restart`；后续 `status` 也应使用相同配置。
