# 化妆品与篮筐位姿定位服务

## 1. 前置服务

在启动本定位服务前，需确保以下服务已在后台运行并完成模型加载：

1. **SAM3 实例分割服务**（必选）
   - 提供图像两阶段分割能力（纸盒与目标物）。
   - 默认服务地址：`http://127.0.0.1:25541/api/v1/segment`。
2. **FoundationPose 姿态估计服务**（可选，仅处理周转篮 `basket` 时需要）
   - 提供周转篮筐的 6D 位姿估计。
   - 依赖本地 CAD 模型：`deploy/cad/Basket/Basket.obj`。
   - 默认服务地址：`http://127.0.0.1:25550/infer`。

---

## 2. 需要的端口

| 端口 | 服务名称 | 说明 |
| :--- | :--- | :--- |
| **`25540`** | **本定位服务** | 对外提供定位接口（`/health`, `/infer`） |
| **`25541`** | **SAM3 分割服务** | PyTorch 原生分割服务端口（默认上游） |
| **`25550`** | **FoundationPose 服务** | 篮筐 6D 位姿估计服务端口 |
| *`25551` / `25552`* | *SAM3 TRT 服务* | 备用 TRT 模式端口（25551 纸盒 / 25552 目标） |

---

## 3. 需要修改的配置文件

### 核心配置文件：`deploy/config.json`

根据实际部署环境修改以下字段：

```json
{
  "service": {
    "default_host": "0.0.0.0",
    "default_port": 25540        // 本服务监听端口
  },
  "sam3": {
    "backend": "multipart_segment", // 上游协议：multipart_segment 或 legacy_18003
    "urls": {
      "multipart_segment": "http://127.0.0.1:25541/api/v1/segment", // SAM3 原生服务地址
      "legacy_18003": "http://127.0.0.1:25551/infer"               // SAM3 TRT 服务地址
    }
  },
  "basket": {
    "foundationpose_url": "http://127.0.0.1:25550/infer"           // FoundationPose 服务地址
  },
  "prompts": {
    "container_box": "each individual open cardboard box",          // 纸盒提示词
    "bottle": "The main cylindrical body of each white bottle",     // 瓶身提示词
    "box": "All visible top surfaces of the blue green boxes",      // 纸盒顶面提示词
    "tube": "All individual gray green package ends"                // 软管封尾提示词
  }
}
```

> **注意**：若使用 `deploy/start_25540.sh` 启动，脚本内 `apply_env()` 导出的环境变量（如 `SAM3_URL`）会覆盖 `config.json` 的对应项。

---

## 4. 启动命令

### 方式一：Linux 生产运维脚本（推荐）

```bash
cd deploy

# 启动服务（后台运行）
./start_25540.sh start

# 停止服务
./start_25540.sh stop

# 重启服务
./start_25540.sh restart

# 查看服务状态
./start_25540.sh status

# 前台调试运行
./start_25540.sh --foreground
```

- 日志文件：`deploy/logs/25540_service.log`
- 进程文件：`deploy/logs/25540_service.pid`

### 方式二：命令行直接启动（Python）

```bash
# Linux
python3 deploy/server.py --host 0.0.0.0 --port 25540

# Windows
python deploy/server.py --host 0.0.0.0 --port 25540
```

### 验证命令

```bash
# 检查服务健康状态
curl http://127.0.0.1:25540/health
```
