# Deploy 模块代码审查与优化待办清单 (Code Review & Optimization Notes)

> **状态核定结论**：
> - **重构完成度**：代码重构与 4+1 架构分层（`core/`, `perception/`, `pipeline/`, `algorithms/`）全量落地；
> - **根目录彻底纯净化**：已物理删除 `deploy/` 根目录 19 个 Facade 门面文件、历史 `backups/` 备份快照及失效的 `25540_service.pid`；
> - **测试链路无缝对齐**：`tests/` 全部 11 个测试文件与 `start_25540.sh` 脚本路径均已对齐子包导入与真实路径；
> - **遗留彻底收口**：
>   1. 全链路计时命名已彻底收拢为单一单词 `mark`（已消灭所有 `mark_fn` 参数与回退）；
>   2. 彻底移除 `inference_mode` 解析及校验逻辑，固定全图过滤流；
>   3. 彻底清除 `SKU_SAM3_MODE` 变量导出、配置读取与 `/health` 输出痕迹；
> - **测试闭环**：本地单元测试执行完成，实际结果为 **39 项测试运行：37 项通过，2 项 live A800 远端测试跳过（未连接实机），0 项失败**；
> - **运行态说明**：25540 端口当前在本地开发机未作为常驻生产服务启动（已保留空 `logs/` 与 `.gitkeep`），待后续实机环境联调验证；
> - **Git 工作区状态**：最终干净版本已完成收口与测试验证，归档提交。

---

## 优化项记录清单（全量执行闭环）

### 1. 彻底移除 `server.py` 中的 `process()` 封装函数
* **所在文件与行号**：[server.py](file:///D:/TermiTech/Cosmetics_Sort/cosmetics_pose_gitlab/deploy/server.py#L94-L102)
* **现有代码片段**：
  ```python
  def process(req, request_id, root):
      if not hasattr(_STAGES, 'd') or _STAGES.d is None:
          _STAGES.d = {}
      return execute_pipeline(req, request_id, root, mark_fn=_mark)
  ```
* **决策与理由**：
  - 测试原则统一：实际业务测试统一采用真实的 **HTTP 接口请求（黑盒契约测试）**，不保留纯 Python 层的 `process()` 中间封装。
  - 减少一层无实质逻辑的薄代理，使调用链更加扁平直观。
  - 同步消除其内部 `if not hasattr(_STAGES, 'd')...` 的冗余初始化代码。
* **改动落地计划**：
  1. **`server.py`**：删除 `def process(...)`；在 `Handler.do_POST`（第 184 行）中直接改为：
     ```python
     result = execute_pipeline(req, rid, root, mark_fn=_mark)
     ```
  2. **`tests/test_pipeline_execution.py`**：将历史测试中调用的 3 处 `server.process(...)` 改为 HTTP 请求或直接调用 `target_pipeline.execute_pipeline`。

---

### 2. 收敛 SAM3 推理地址为单卡单一主 URL，移除 `SAM3_URL_TARGET`
* **所在文件与行号**：
  - [server.py](file:///D:/TermiTech/Cosmetics_Sort/cosmetics_pose_gitlab/deploy/server.py#L44)（导入 `SAM3_URL_TARGET`）与 [L132](file:///D:/TermiTech/Cosmetics_Sort/cosmetics_pose_gitlab/deploy/server.py#L132)（`/health` 中输出 `'sam3_target_url': SAM3_URL_TARGET or SAM3_URL`）
  - [target_pipeline.py](file:///D:/TermiTech/Cosmetics_Sort/cosmetics_pose_gitlab/deploy/target_pipeline.py#L697)（`_target_url = SAM3_URL_TARGET or SAM3_URL`）
  - [config_loader.py](file:///D:/TermiTech/Cosmetics_Sort/cosmetics_pose_gitlab/deploy/config_loader.py#L77) 与 `start_25540.sh`（环境变量导出）
* **现有代码片段**：
  ```python
  # server.py L132
  'sam3_target_url': SAM3_URL_TARGET or SAM3_URL,

  # target_pipeline.py L697
  _target_url = SAM3_URL_TARGET or SAM3_URL
  ```
* **决策与理由**：
  - **部署架构定调**：后续所有场景均固定为**单卡环境**，不需要为两阶段分割分别部署多套 SAM3 实例/多端口分流。
  - 直接统一收拢为单一的主 `SAM3_URL`，消灭双 URL 带来的环境配置复杂度与冗余三元回退逻辑。
* **改动落地计划**：
  1. **`server.py`**：删除 `SAM3_URL_TARGET` 导入；`/health` 自省中统一只返回 `'sam3_url': SAM3_URL`（或平滑保留键名但值直接写 `SAM3_URL`）。
  2. **`target_pipeline.py`**：删除 `SAM3_URL_TARGET` 导入；第二阶段 SKU 推理直接使用 `SAM3_URL`，删除 `_target_url` 回退判断。
  3. **`config_loader.py`**：移除 `SAM3_URL_TARGET` 环境变量读取与全局导出。
  4. **`start_25540.sh`**：清理脚本中 `export SAM3_URL_TARGET=...` 的冗余多实例配置。

---

### 3. 重构 `/health` 响应结构为模块化嵌套子字典
* **所在文件与行号**：[server.py](file:///D:/TermiTech/Cosmetics_Sort/cosmetics_pose_gitlab/deploy/server.py#L117-L149)
* **现有问题**：
  当前 `/health` 响应在顶层平铺了 26 个字段（包括 7 个 basket 相关字段、6 个 sam3 字段、7 个算法脚本路径），结构较为扁平冗长，缺乏层次感。
* **重构方案**：
  在保持原有信息量与非阻断轻量自省机制不变的前提下，收拢为模块化嵌套字典：
  ```python
  {
      "ok": True,
      "service": "spatial-localization-diagnostic",
      "basket": {
          "pipeline": "SAM3 central white plastic basket -> local FoundationPose 25550",
          "prompt": BASKET_DEFAULT_PROMPT,
          "threshold": BASKET_DEFAULT_THRESHOLD,
          "cad_path": str(BASKET_MESH_PATH),
          "mesh_scale": BASKET_MESH_SCALE,
          "foundationpose_url": BASKET_FP_URL,
          "cad_transport": "registered_default" if BASKET_FP_REGISTERED_CAD else "per_request_upload",
      },
      "sku": {
          "supported_types": sorted(CLASS_CONFIG),
          "pipeline": "box_selection_then_product_localization",
          "sam3_url": SAM3_URL,
          "mask_threshold": SAM3_MASK_THRESHOLD,
          "default_front_rule": FRONT_RULE_DEFAULT,
      },
      "render": {
          "enable_3d": ENABLE_3D_RENDER,
          "async_3d": ASYNC_3D_RENDER,
      },
      "flags": {
          "weights_loaded_here": False,
          "robot_actions": False,
      }
  }
  ```
* **收益**：JSON 结构逻辑清晰、分类明确，便于上位机按模块读取与人眼查阅。

---

### 4. 优化 `final_response_json_write` 打点时序或剔除
* **所在文件与行号**：[server.py](file:///D:/TermiTech/Cosmetics_Sort/cosmetics_pose_gitlab/deploy/server.py#L192-L196)
* **现有代码片段**：
  ```python
  _diag['stage_timing_ms'] = dict(getattr(_STAGES, 'd', {}))   # Line 192

  t_save = time.perf_counter()
  save_json(root / 'response.json', result)
  _mark('final_response_json_write', t_save)                   # Line 196
  ```
* **问题分析**：
  `_mark('final_response_json_write')` 记录的时间发生在 `stage_timing_ms` 被提取打包进 `result` 之后，且在 `save_json` 之后。导致客户端收到的响应和磁盘落盘的 `response.json` 中均无法看到该阶段的耗时。
* **改动计划**：
  重构时要么将耗时提至打包前（若需要统计），要么直接剔除这行幽灵打点。

---

### 5. 全链路统一计时插桩函数命名为 `mark`
* **所在文件与行号**：
  - [server.py](file:///D:/TermiTech/Cosmetics_Sort/cosmetics_pose_gitlab/deploy/server.py#L82)（`def _mark`）
  - [target_pipeline.py](file:///D:/TermiTech/Cosmetics_Sort/cosmetics_pose_gitlab/deploy/target_pipeline.py#L959) 等多处函数形参（`mark_fn=None`）与内部解包（`mark = mark_fn or ...`）
* **现有问题**：
  同一概念在不同文件中存在 3 种别名（`_mark`、`mark_fn`、`mark`），增加认知负担和传参别扭感（`mark_fn=_mark`）。
* **决策与方案**：
  彻底统一为单一单词 `mark`：
  1. `server.py`：直接声明为公共函数 `def mark(name: str, t0: float) -> float:`，模块内打点直接调用 `mark(...)`；
  2. 传参直接写：`execute_pipeline(..., mark=mark)`；
  3. `target_pipeline.py`：所有函数形参统一为 `mark=None`，内部解包 `mark = mark or (lambda ...)`。
* **收益**：全链路单个名词打通，彻底消除命名割裂。

---

### 6. 统一在 `execute_pipeline` 顶层解码 `common_input`，消除分支不对称性
* **所在文件与行号**：
  - [target_pipeline.py](file:///D:/TermiTech/Cosmetics_Sort/cosmetics_pose_gitlab/deploy/target_pipeline.py#L966)（basket 分支内部解码）与 [L686](file:///D:/TermiTech/Cosmetics_Sort/cosmetics_pose_gitlab/deploy/target_pipeline.py#L686)（sku 分支内部解码）
* **现有问题**：
  在 `execute_pipeline` 中，basket 分支在顶层调用 `common_input(req)` 并显式传递 `(rgb, depth, K, T_m)` 给 `process_basket`；而 sku 分支却直接丢原始 `req` 给 `process_sku`，由其内部自行解码。导致两个并列分支在数据准备和函数签名上不对称。
* **重构方案**：
  将 `common_input(req)` 解码统一提至 `execute_pipeline` 开头，解码一次并完成打点，然后将解出的 `rgb, depth, K, T_m` 对称传入下游两个分支：
  ```python
  def execute_pipeline(req: Dict[str, Any], request_id: str, root: Path, mark=None) -> Dict[str, Any]:
      mark = mark or (lambda name, t0: time.perf_counter())
      set_embed_visualizations(bool(req.get('return_visualizations', False)))
      class_name = normalize_request_target(req)

      t0 = time.perf_counter()
      rgb, depth, K, T_m = common_input(req)
      mark('rgbd_decode', t0)

      if req.get('target_type') == 'basket':
          return process_basket(req, request_id, root, rgb, depth, K, T_m, mark=mark)
      return process_sku(req, request_id, root, rgb, depth, K, T_m, class_name=class_name, mark=mark)
  ```
* **收益**：输入解码单一职责，分支签名高度对称，避免未来重复解析。

---

### 7. 彻底剔除 `crop_box` 局部裁剪分支与冗余代码
* **所在文件与行号**：
  - [target_pipeline.py](file:///D:/TermiTech/Cosmetics_Sort/cosmetics_pose_gitlab/deploy/target_pipeline.py#L810-L820)
  - [box_selection.py](file:///D:/TermiTech/Cosmetics_Sort/cosmetics_pose_gitlab/deploy/box_selection.py#L261) 及 `crop_region_from_roi`、`restore_crop_detections`
  - [server.py](file:///D:/TermiTech/Cosmetics_Sort/cosmetics_pose_gitlab/deploy/server.py#L142)
* **决策与方案**：
  - 实测全图模式 `full_image_filter` 在上下文保留、边缘无截断、检测精度上全面优于 `crop_box`；
  - 生产中 100% 默认使用 `full_image_filter`，`crop_box` 属于未使用的实验性死代码；
  - 彻底删除 `crop_box` 分支及其配套的边距、Padding 校验和局部坐标还原映射逻辑，精简数十行冗余代码。

---

### 8. 单卡部署架构固化为串行模式，剔除 `dual_parallel` 线程池复杂度
* **所在文件与行号**：
  - [target_pipeline.py](file:///D:/TermiTech/Cosmetics_Sort/cosmetics_pose_gitlab/deploy/target_pipeline.py#L698-L713) 与 [L830-L840]
  - [start_25540.sh](file:///D:/TermiTech/Cosmetics_Sort/cosmetics_pose_gitlab/deploy/start_25540.sh#L33)（已显式设定 `SKU_SAM3_MODE=serial`）
* **决策与方案**：
  - `dual_parallel` 仅适用于历史多卡/多实例环境。在单卡单实例下，并发请求会导致显卡显存翻倍抢占（易 OOM）且算力无法并行；若无纸箱还会白白浪费商品推理算力；
  - 线上实际配置已固定为 `serial`；
  - 彻底移除 `ThreadPoolExecutor(max_workers=2)`、`sam3_fut` 预取暂存及未命中清理逻辑，全流程保持干净、线性的串行执行。

---

### 9. 修复 `box_selection.py` 单箱场景直接崩溃缺陷，引入自适应单/双箱策略
* **所在文件与行号**：
  - [box_selection.py](file:///D:/TermiTech/Cosmetics_Sort/cosmetics_pose_gitlab/deploy/box_selection.py#L44-L55)（`choose_pair` 中硬编码 `combinations(candidates, 2)`）
  - [target_pipeline.py](file:///D:/TermiTech/Cosmetics_Sort/cosmetics_pose_gitlab/deploy/target_pipeline.py#L742)
* **缺陷事实**：
  现有代码假设视野内“必须同时存在左右两个箱子”，若工位上只有 1 个箱子（如单工位产线、或其中一箱已被搬走/遮挡），`combinations([], 2)` 为空，直接抛出 `no_separable_left_right_box_pair`，导致系统 100% 报定位失败退出。
* **重构方案**：
  引入**自适应单/双箱策略（Adaptive Single/Dual Box）**：
  - 若 `len(candidates) >= 2`：走原有的两两组合、高度重叠率检查与中线黄金分割裁切逻辑；
  - 若 `len(candidates) == 1`：自适应降级，直接将该唯一箱体作为目标 ROI，正常放行进入第二阶段 SKU 定位；
  - 若 `len(candidates) == 0`：才报错抛出 `no_cardboard_box_detected`。
* **收益**：彻底解决单箱、清箱残留、局部遮挡场景下的误报瘫痪问题，极大增强产线鲁棒性。

---
