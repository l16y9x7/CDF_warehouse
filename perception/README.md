# Perception 视觉条码服务

Ubuntu 上可在仓库根目录运行 `bash start_services.sh`，一并启动 SAM3、FoundationPose、estimation 和 perception。
默认使用下文的 SAM3 目录及 Conda 环境；路径覆盖、依赖准备和管理命令见[仓库 README](../README.md)。
以下命令保留为手动分步启动方式。

## Step 1：部署 SAM3 服务

端口固定为 `25541`。

```bash
conda activate sam3
cd /data/steven/sam3_api
nohup python3 ./backend/app.py > ./backend/app.log 2>&1 &
```

## Step 2：配置 SAM3 地址

```bash
export SAM3_URL=http://127.0.0.1:25541/api/v1/segment
```

Windows PowerShell：

```powershell
$env:SAM3_URL = "http://127.0.0.1:25541/api/v1/segment"
```

## Step 3：启动 perception

```bash
conda activate slim
cd perception
nohup python -m uvicorn main:app --host 0.0.0.0 --port 25546 > perception.log 2>&1 &
```

## Step 4：验证

调用条形码识别接口，测试 `tests/1.jpg`：

```bash
curl -X POST "http://127.0.0.1:25546/perception/recognize_sku_barcode" \
  -H "Content-Type: application/json" \
  -d '{
    "sku_id": "test_sku",
    "name": "测试商品",
    "image_path": "tests/1.jpg"
  }'
```

成功时返回示例：

```json
{"status": "FOUND", "barcode_content": "..."}
```

## 请求日志和原始图片归档

`recognize_sku_barcode` 和 `locate_sku_qr_code` 每次请求都会生成独立 `request_id`，
通过响应头 `X-Request-ID` 返回。原有响应 JSON 字段保持兼容。健康检查不生成归档。

使用根目录的一键脚本且没有设置日志覆盖变量时，默认目录为：

```text
/data/CDF_warehouse/logs/perception/
├── perception.log                 # Uvicorn 控制台和 HTTP 访问日志
├── requests.log                   # 请求开始/结束、状态、总耗时、归档错误
├── recognize_sku_barcode.log       # 识别及定位的详细流程摘要
└── requests/YYYY-MM-DD/request_id/
    ├── request.json               # 请求参数、时间、接口、图片引用及 SHA-256
    ├── input.jpg / input.png      # 输入原始字节（也支持其他格式）
    ├── response.json              # http_status、body、request_id
    └── trace.json                 # 状态、耗时、候选框、解码尝试及异常
```

图片在推理前落盘。路径输入只读取一次并复制原始字节，Base64 输入先解码成原始文件字节；
不重新压缩成 JPEG，推理也使用这同一份字节。文件扩展名按内容判断，未知或损坏格式保存为
`input.bin`。`request.json` 的图片元数据包含 `file`、`bytes`、`sha256`、`saved`。
请求中的大段 Base64 用占位元数据替代，校验错误响应里的同名字段也会去除重复数据；定位响应的
`mask` 完整保存在 `response.json` 中。

成功、`NOT_FOUND`、参数校验失败、读图失败、SAM3 错误及未预期的服务异常都生成请求和响应记录。
参数校验失败时，如果图片来源仍可确定，也保存图片；路径不存在、Base64 无法解码或来源冲突时，
无法生成原图文件，可查看 `request.json` 和 `trace.json` 的错误信息。
非法 JSON 请求另外保留 `request_body.bin`。`trace.json` 先标记为 `received`，完成后原子替换为
`completed`；进程被强制终止时可能只留下未完成的记录。

详细摘要含 `request_id` 和原图路径。SAM3 失败会明确标记 `ERROR`，无候选或中心过滤失败时也保留
图片引用。不再生成旧的 `not_found_*.jpg` 副本，已有旧文件保留原位。
磁盘不可写等归档错误会写入请求日志，能够写出的 trace 会包含 `archive_errors`，不改变识别返回值。
两份业务日志不再同时复制到控制台；无法打开业务日志时退回 stderr。

| 环境变量 | 默认行为 |
| --- | --- |
| `RECOGNIZE_SKU_BARCODE_LOG_PATH` | 手动运行默认 `perception/logs/recognize_sku_barcode.log`，一键脚本指定上述集中目录 |
| `PERCEPTION_REQUEST_LOG_PATH` | 默认与识别日志同目录的 `requests.log` |
| `PERCEPTION_REQUEST_DIR` | 默认与识别日志同目录的 `requests/` |
| `PERCEPTION_LOG_MAX_BYTES` | `10485760`，业务日志每个文件约 10 MiB 时轮转 |
| `PERCEPTION_LOG_BACKUP_COUNT` | `5`，每份业务日志最多保留 5 份备份 |
| `PERCEPTION_REQUEST_RETENTION_DAYS` | `0`，默认保留全部请求；例如设为 `7` 开启过期清理 |

轮转适用于一键脚本默认的单个 Uvicorn 进程；`perception.log` 仍由启动脚本追加，不包含在业务日志
轮转范围。启用归档过期清理后，有请求到来时每个进程最多每小时检查一次，仅删除超过保留日期且
标记为完成的本系统请求目录；未完成记录、未知目录和符号链接跳过。仓库 `logs/` 及
`perception/logs/` 已加入 `.gitignore`。

查找某次请求：

```bash
# curl 的 -i 会显示 X-Request-ID 响应头
curl -i http://127.0.0.1:25546/perception/recognize_sku_barcode \
  -H 'Content-Type: application/json' \
  -d '{"sku_id":"test","name":"测试商品","image_path":"/path/to/input.jpg"}'
find /data/CDF_warehouse/logs/perception/requests -name '返回的request_id' -type d
```

本地回归检查会使用临时目录和模拟的模型调用，不访问 SAM3 或 GPU：

```bash
python -m pip install -r perception/requirements-test.txt
python -m unittest discover -s perception/tests -p 'test_*.py' -v
```
