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

## 条码方法对照

识别接口默认额外运行两种方法：`opencv_sr`（加载超分模型的 OpenCV BarcodeDetector）和
`zxing_cpp`（ZXing-C++，限定一维条码）。三种方法使用同一次 SAM3 定位选出的条码区域及相同的
bbox 扩边比例，不增加 SAM3 请求。OpenCV 超分沿用原方法的预处理和旋转顺序；ZXing 使用原始裁剪，
由其内部完成旋转、缩小等搜索。

**新增方法只写日志，响应始终使用原 OpenCV 方法的结果。** 原方法返回 `NOT_FOUND` 时，即使
对照方法识别成功，接口仍返回 `NOT_FOUND`；新增方法缺依赖、模型缺失或解码异常也不会替换原返回值。
没有选中候选框时跳过对照，定位接口和健康检查不运行对照。

对照目前同步执行，会增加请求耗时。`timings_ms.decode` 仍是原方法耗时，
`timings_ms.decode_comparisons` 单独记录新增耗时，总耗时包含二者。
如需停用，设置 `PERCEPTION_BARCODE_COMPARISON_ENABLED=0` 后重启服务。

在运行 perception 的 Python 环境安装 `requirements.txt`，其中包含 `zxing-cpp>=2.3,<4`。
超分默认读取以下文件，无需移动现有目录，也不使用该仓库的 `detect.*` 模型：

```text
perception/opencv_3rdparty-wechat_qrcode/sr.prototxt
perception/opencv_3rdparty-wechat_qrcode/sr.caffemodel
```

模型在线程内缓存复用，各线程独立持有检测器。`runtime.sr_configured=true` 只表示模型成功加载；
是否实际执行超分由 OpenCV 内部按条码尺寸决定，Python 接口无法直接观察，不代表每次都经过超分。

## 请求日志和原始图片归档

`recognize_sku_barcode` 和 `locate_sku_qr_code` 每次请求都会生成独立 `request_id`，
通过响应头 `X-Request-ID` 返回。原有响应 JSON 字段保持兼容。健康检查不生成归档。

使用根目录的一键脚本且没有设置日志覆盖变量时，默认目录为：

```text
/data/CDF_warehouse/logs/perception/
├── perception.log                 # Uvicorn 控制台和 HTTP 访问日志
├── requests.log                   # 请求开始/结束、状态、总耗时、归档错误
├── recognize_sku_barcode.log       # 识别及定位的详细流程摘要
├── barcode_opencv_sr.log           # OpenCV 超分对照结果，每条包含 request_id
├── barcode_zxing_cpp.log           # ZXing-C++ 对照结果，每条包含 request_id
└── requests/YYYY-MM-DD/request_id/
    ├── request.json               # 请求参数、时间、接口、图片引用及 SHA-256
    ├── input.jpg / input.png      # 输入原始字节（也支持其他格式）
    ├── response.json              # http_status、body、request_id
    ├── decode_opencv_sr.json       # 仅识别接口：OpenCV 超分对照详情
    ├── decode_zxing_cpp.json       # 仅识别接口：ZXing-C++ 对照详情
    └── trace.json                 # 状态、耗时、候选框、解码尝试及异常
```

两份对照记录包含 `comparison_only=true`、原业务结果、方法结果、条码类型、耗时、尝试过程、
版本/模型配置及错误信息。`status` 为 `FOUND`、`NOT_FOUND`、`ERROR` 或 `SKIPPED`；
`matches_business_result` 表示是否有任意一条对照结果匹配原返回值，原方法无结果或对照未完成时为
`null`。两份详情也保存在 `trace.json` 的 `pipeline.decode_comparisons` 下。
参数错误、SAM3 失败等导致未运行解码时，仍生成独立 `SKIPPED` 记录并保存原因。

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
业务日志不再同时复制到控制台；无法打开业务日志时退回 stderr。

| 环境变量 | 默认行为 |
| --- | --- |
| `RECOGNIZE_SKU_BARCODE_LOG_PATH` | 手动运行默认 `perception/logs/recognize_sku_barcode.log`，一键脚本指定上述集中目录 |
| `PERCEPTION_REQUEST_LOG_PATH` | 默认与识别日志同目录的 `requests.log` |
| `PERCEPTION_REQUEST_DIR` | 默认与识别日志同目录的 `requests/` |
| `PERCEPTION_BARCODE_COMPARISON_ENABLED` | `1`，运行两种对照方法；`0`/`false`/`no`/`off` 停用并记录 `SKIPPED` |
| `PERCEPTION_BARCODE_SR_MODEL_DIR` | `opencv_3rdparty-wechat_qrcode`；相对路径以 `perception/` 为基准，也可使用绝对路径 |
| `PERCEPTION_OPENCV_SR_LOG_PATH` | 默认与识别日志同目录的 `barcode_opencv_sr.log` |
| `PERCEPTION_ZXING_CPP_LOG_PATH` | 默认与识别日志同目录的 `barcode_zxing_cpp.log` |
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

本地回归检查会使用临时目录和模拟的 SAM3 调用，不访问 SAM3 服务或 GPU。
检测到本地超分模型或 ZXing 依赖时，还会用生成的标准 EAN-13 样例执行真实解码检查：

```bash
python -m pip install -r perception/requirements-test.txt
python -m unittest discover -s perception/tests -p 'test_*.py' -v
```
