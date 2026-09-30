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
