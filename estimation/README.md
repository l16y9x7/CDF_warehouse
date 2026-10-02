# 化妆品与篮筐位姿定位服务

Ubuntu 上可在仓库根目录运行 `bash start_services.sh`，按顺序启动 SAM3、FoundationPose、
estimation 和 perception；部署路径、依赖准备及停止/重启命令见[仓库 README](../README.md)。

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
    "bottle": "the main cylindrical body of each individual cosmetic bottle", // 瓶身
    "box": "each individual small retail product carton", // 商品盒整体
    "tube": "the sealed ends of tubes" // 软管封尾
  }
}
```

> **注意**：若使用 `deploy/start_25540.sh` 启动，脚本内 `apply_env()` 导出的环境变量（如 `SAM3_URL`）会覆盖 `config.json` 的对应项。

### 服务器端 SKU 视觉配置库（2026-10-01）

在 [deploy/sku_profiles.json](deploy/sku_profiles.json) 中按业务 `sku_id` 维护提示词、类别、
商品分割阈值、面积上限和可选瓶身半径。机器人不需要维护这些算法参数。

- `sku_id` **可选**：旧 agent 继续发送 `sku_typ` + `side`，使用去掉颜色限制的通用提示词。
- 传入已配置的 `sku_id` 后，`sku_typ` 可以省略；如果两者都传，类别必须一致。
- SKU 库中配置的 prompt、阈值、面积上限、半径优先于请求中的同名算法参数。
  `side`、RGB-D、K、采集时刻外参等仍由请求决定。
- 未知 ID、空字符串/非字符串 ID、类别冲突返回 HTTP 400。`sku_id: null` 按未提供处理。
- `basket` 仍不接收 `sku_id` / `sku_typ`，不走商品库。
- 每个带 ID 的请求重新读取并验证库，无需重启。建议编辑临时文件后原子替换正式文件，并更新 `revision`。
  文件路径可以用 `SKU_PROFILES_PATH` 环境变量覆盖；相对路径按 `deploy/` 解析。
  `config.json` 的通用配置和代码更新仍需重启服务。

库里预置了 agent 商品表已有的 3 个 ID，以及本次三种外观的占位 ID：
`demo_dark_blue_box`、`demo_cream_box`、`demo_white_tube`。占位 ID 后续改为真实条码/业务 ID；
2026-10-02 已用三张失败日志的原始 RGB-D 调用真实 SAM3 回放：不传 ID 和传对应占位 ID 的
六条请求均保留 5 个箱内商品候选，并通过当时的几何质量检查。
当前通用 box 提示词调整为 `each individual small retail product carton`，分割商品盒整体后由深度几何提取顶面；
本次整体提示词调整尚未做实图回归。tube 提示词为 `the sealed ends of tubes`。
该结果仅覆盖这些固定场景，不代表已验证定位精度或机器人抓取；agent 商品表中真实 ID 的对应实物仍待回归。

请求新增字段示例（其余 RGB-D、K、外参字段仍必填）：

```json
{"target_type": "sku", "sku_id": "demo_cream_box", "side": "LEFT"}
```

响应及落盘结果包含 `sku_id`（旧请求为 null），`diagnostics.sku_profile` 记录配置来源、
库版本及应用参数。现有颜色专用提示词可以通过 SKU 配置保留，通用类别默认不限制颜色。

### 商品面积过滤

商品分割后、几何拟合前，计算 `完整商品 mask 像素数 / 所选纸箱 ROI 像素数`。
默认拒绝 **大于 0.5** 的候选，拦截把整只外箱当作商品的误检。
分母为裁剪到图像内并栅格化后的外箱包围框，不是外箱分割 mask。
可以在 SKU 条目的 `max_mask_area_ratio`、`config.json` 的类别配置中调整；
不带 SKU ID 的诊断请求也可用 `box_selection.max_mask_area_ratio` 覆盖。
范围为 `(0, 1]`，实际占箱比例较大的商品需要调整此值。
`box_selection.object_membership` 会记录面积、比例和阈值；拒绝原因是
`mask_area_ratio_above_threshold`。原有箱内归属、前排和几何质量检查继续生效。

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
