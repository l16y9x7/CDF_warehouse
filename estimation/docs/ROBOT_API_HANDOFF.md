# 位姿估计定位 HTTP 接口交接说明

> **2026-09-21 协议切换（严格替换）**：正式类别字段由 `sku_id` 改为 `sku_typ`，类别值 `Avene`/`estee`/`origins` 改为 `bottle`/`box`/`tube`（Avene→bottle、estee→box、origins→tube）。请求中出现 `sku_id` 或 `class_name` 一律 HTTP 400 拒绝；响应统一输出 `sku_typ`（`class_name` 仅作为同值冗余保留），不再输出 `sku_id`；`/health` 返回 `supported_sku_types`。历史章节（第 10、11 节）保留旧名称作为当时事实记录。

## 1. 接口用途与边界

控制端把**同一时刻、同一相机模型**的 RGB、对齐深度、相机内参和逐帧外参发送给服务。所有 `target_type=sku` 请求统一执行“箱体 SAM3 → 按 `side` 选左/右箱 → 商品 SAM3 → 箱内过滤 → 类别几何定位”；`sku_typ=bottle/box/tube` 只决定商品提示词、阈值及圆柱轴线/顶面/可见上边缘几何策略。请求中出现旧 `sku_id`（Avene/estee/origins）或 `class_name` 一律 HTTP 400 拒绝。

控制端可以只发送公共输入和类别名；SAM3 提示词、阈值、拟合算法、瓶身半径、参考高度和选箱参数均有服务端类别默认值。若请求显式发送这些可选配置，服务会验证并使用请求值；缺省字段使用服务端默认值。配置覆盖只影响当前请求，不修改服务端全局配置。

请求产物按目标类型分目录保存：`/home/quinn/cosmetics_pose/requests/basket/<request_id>/` 或 `/home/quinn/cosmetics_pose/requests/sku/<request_id>/`。Basket 目录还会保存 FoundationPose 的姿态框/坐标轴叠加图（`foundationpose_pose_overlay.png`）以及实例分割和检测姿态图。

叠加图与点云**总是**写入请求目录，但默认**不**把 base64 负载内嵌进响应：`artifacts` 只给服务器端路径，单帧响应体因此只有几十 KB。需要本地直接查看叠加图/点云时，发送 `"return_visualizations": true`，服务端才会额外内嵌 `*_jpeg_base64` / `*_ply_base64`（单帧响应会涨到约 4 MB）。

**HTTP 线上响应与服务端落盘诊断的分离原则**：
- 服务端本地目录（如 `/home/quinn/cosmetics_pose/requests/<type>/<request_id>/response.json`）完整保留服务端全部计算结果与内部排查数据（含点云参数、多重拟合中间过程），便于离线复盘与算法诊断；
- 线上 POST `/infer` 返回给控制端/机器人的 HTTP JSON 经过严格白名单结构收敛：彻底剔除点云（`raw`、`fit`）、内部 2D 掩码（`fm`、`ins`）、侵蚀多重拟合大对象（`analyses`）以及任何 `center_fit` / `left_fit` / `right_fit` 内部字典；
- 线上响应大小严格保持在几十 KB 量级，避免终端刷屏或引发网络 `BrokenPipeError`。

当 `target_type=basket` 时，控制端仍发送同一帧 RGB、对齐深度、K 和 `T_chassis_camera`，不需要发送 `sku_typ` 或 CAD。25540 用服务器内置提示词生成篮筐 mask，再调用本机 25550 FoundationPose，返回**篮筐 CAD 模型中心点**（`model_center_camera_mm` / `reference_point_camera_mm` / `reference_point_chassis_mm`）和**完整 4x4 位姿**（`pose_4x4`、`pose_4x4_input_m`、`rotation_euler_zyx_rad`）。注意 `pose_4x4` 的平移列是 **CAD 原点**（该 CAD 的原点在箱体角上），不是模型中心；两者相差约 318 mm。字段见第 6.1 节。

当前接口是离线诊断定位接口，不等于机器人抓取成功率、毫米精度或安全性已经验证。bottle 的正式参考点采用 `visible_axis_midpoint`，不再使用固定 `chassis_link` Z 平面作为抓取点；旧的 `z_ref_mm` 请求字段已从服务中移除，发送它不会改变任何结果（响应中的 `reference_z_mm` 固定为 null）。

## 2. 地址

- 健康检查：`GET http://211.137.21.33:25540/health`
- 定位请求：`POST http://211.137.21.33:25540/infer`
- `Content-Type: application/json; charset=utf-8`
- 当前请求体上限：32 MiB
- 建议客户端超时：120 秒

健康检查只证明包装服务正在监听，不证明 SAM3、当前相机数据或圆柱拟合一定成功。

## 3. 机器人端每帧必须发送的信息

| 字段 | 类型 | 必填 | 单位/坐标系 | 说明 |
|---|---:|---:|---|---|
| `rgb_base64` | string | 是 | BGR/RGB 图像文件字节 | JPG/PNG 文件的 base64，不是像素数组的 base64 |
| `depth_npy_base64` | string | 是 | mm | 与 RGB 对齐的二维浮点 `.npy` 文件字节的 base64 |
| `depth_unit` | string | 是 | `mm` | 当前服务内部几何统一使用毫米；不能再次乘 `0.001` |
| `K` | 3x3 array | 是 | pixel | 与当前 RGB 分辨率和投影模型一致的针孔内参 |
| `T_chassis_camera` | 4x4 array | 是 | 见 `T_unit` | 当前帧变换，满足 `p_chassis = R @ p_camera + t` |
| `T_unit` | string | 是 | `m` 或 `mm` | 只描述矩阵平移列 `t` 的单位；旋转无量纲 |
| `camera_frame` | string | 是 | — | 固定为 `head_camera_color_optical_frame` |
| `base_frame` | string | 是 | — | 固定为 `chassis_link` |
| `target_type` | string | 是 | `sku` 或 `basket` | `sku` 走商品定位；`basket` 由服务器自动生成篮筐 mask 并调用 FoundationPose，返回篮筐模型中心点与位姿 |
| `sku_typ` | string | 是 | — | `bottle`、`box` 或 `tube`；由服务端选择对应识别及几何策略 |
| `side` | string | 是 | — | `LEFT`/`RIGHT`，选择当前图像所选箱对的左箱/右箱；所有 SKU 均实际使用 |
| `sam3_prompt` | string | 否 | — | 商品提示词覆盖；缺省使用类别默认 |
| `sam3_threshold` | number | 否 | 0–1 | 直接识别或商品阶段阈值覆盖；缺省使用类别默认 |
| `box_selection` | object | 否 | — | 诊断覆盖字段；普通控制端只需发送 `side`，服务器统一启用选箱 |
| `return_visualizations` | bool | 否 | — | 是否在响应 `artifacts` 内嵌 `*_base64` 可视化负载；缺省 `false`（只回路径）。需要本地看图时发 `true` |
| `body_radius_mm` | number | 否 | mm | bottle 默认 28.5；不适用于瓶口、瓶盖或瓶肩 |
| `fit_mode` | string | 否 | — | bottle 默认 `cylinder_3d`；可选 `prior_2d` |
| `front_rule` | object | 否 | chassis | bottle 前排规则；box-first 时还用于商品前后过滤和箱体前挡板拟合。省略时使用服务端联调默认值，详见下一节 |

数据一致性要求：

1. RGB、深度、K 和 `T_chassis_camera` 必须来自同一帧/同一采集时刻。
2. 深度必须已经对齐 RGB，且二维尺寸等于 RGB 的高、宽。
3. `T_chassis_camera` 必须是采集时头部关节状态对应的逐帧外参，不能用当前姿态或固定手眼矩阵代替。
4. 矩阵旋转必须正交且行列式约为 1，末行为 `[0,0,0,1]`。
5. 当前服务按针孔 K 反投影；若相机输出仍带畸变，采集端必须保证 RGB、对齐深度和 K 使用一致投影模型，不能重复去畸变。

### SKU 可选选箱覆盖字段

```json
"box_selection": {
  "enabled": true,
  "target_box": 1,
  "box_prompt": "large open cardboard box",
  "box_threshold": 0.5,
  "target_threshold": 0.5,
  "inference_mode": "full_image_filter",
  "min_inside_ratio": 0.9,
  "crop_padding_px": 0
}
```

`box_selection` 整体可以省略，控制端优先发送 `side`。所有 SKU 都默认使用 `full_image_filter`。旧 `enabled` 字段仅记录到 `legacy_enabled_requested`；即使发送 `false` 也不能跳过箱体阶段。

可选覆盖示例：

```json
{
  "target_type": "sku",
  "sku_typ": "bottle",
  "sam3_prompt": "The main cylindrical body of each white bottle",
  "sam3_threshold": 0.5,
  "body_radius_mm": 28.5,
  "fit_mode": "cylinder_3d"
}
```

如果不发送这些字段，服务使用同样的类别默认配置；`front_rule` 省略时使用服务端联调默认值 `[1,0,0]`、`[0,0,0]`、`100000 mm`。box/tube 的 `box_selection.target_threshold`（若存在）优先于顶层 `sam3_threshold`；`box_threshold` 只控制箱体阶段。

`tube` 的默认 `target_threshold` 是 `0.2`，`box` 是 `0.5`。正式控制端优先发送 `side=LEFT/RIGHT`，服务映射为当前图像所选箱对的 `target_box=1/2`；它不是机器人左右轴、SAM3 ID 或跨帧编号。箱体 ROI 是 `box_bbox` 二维归属近似。箱位失败时 `box_selection.valid=false`、正式 XYZ 为 null，第二次商品推理不执行，也不回退单阶段。箱位成功但箱内无商品使用 `stage_status=no_target_in_selected_box`，与箱位失败明确区分。默认使用 `full_image_filter`；`crop_box` 会改变 SAM3 上下文和分数排序，仅作为显式对照模式，恢复到原图坐标后才进入原深度/K几何链。

## 4. 前排规则

```json
"front_rule": {
  "front_axis_chassis": [1.0, 0.0, 0.0],
  "front_origin_chassis": [0.0, 0.0, 0.0],
  "front_band_mm": 100000.0
}
```

服务对每个 mask 的深度支持中位点变换到 `chassis_link`，计算：

```text
d = dot(point_chassis - front_origin_chassis, normalize(front_axis_chassis))
```

仅保留 `abs(d) <= front_band_mm` 的实例，然后按原始 SAM3 score 降序、实例 ID 升序确定性选择一个实例。

上面的 `100000 mm` 是联调时用于几乎不过滤实例的宽范围，只能验证协议和最高分选择逻辑，**不能作为正式前排参数**。机器人实际运行前必须用箱体/工位标定确定 `front_axis_chassis`、`front_origin_chassis` 和 `front_band_mm`。

## 5. 完整请求示例

控制端最小商品请求结构如下（base64 和矩阵必须替换为当前帧真实数据）：

```json
{
  "target_type": "sku",
  "sku_typ": "box",
  "side": "LEFT",
  "rgb_base64": "<当前RGB文件字节的base64>",
  "depth_npy_base64": "<当前对齐深度NPY文件字节的base64>",
  "depth_unit": "mm",
  "K": [
    [<fx>, 0.0, <cx>],
    [0.0, <fy>, <cy>],
    [0.0, 0.0, 1.0]
  ],
  "T_chassis_camera": [
    [<r00>, <r01>, <r02>, <tx_m>],
    [<r10>, <r11>, <r12>, <ty_m>],
    [<r20>, <r21>, <r22>, <tz_m>],
    [0.0, 0.0, 0.0, 1.0]
  ],
  "T_unit": "m",
  "camera_frame": "head_camera_color_optical_frame",
  "base_frame": "chassis_link",
  "front_rule": {
    "front_axis_chassis": [1.0, 0.0, 0.0],
    "front_origin_chassis": [0.0, 0.0, 0.0],
    "front_band_mm": 100000.0
  }
}
```

上述 K、单位矩阵和 `100000 mm` 都只是结构示例，不能作为机器人现场参数。控制端必须替换当前帧 K、当前姿态对应的 `T_chassis_camera`，并在现场确认前后轴、原点和有效带宽。所有 SKU 请求都必须按作业箱传 `LEFT` 或 `RIGHT`；仅诊断调用可改用 `box_selection.target_box=1/2`。

真实 base64 很长，示例文件 [`examples/infer_request.example.json`](examples/infer_request.example.json) 使用占位符。可执行请求见 [`robot_request_example.py`](robot_request_example.py) 或 [`test_client.py`](test_client.py)。命令行示例所需的简化外参文件见 [`examples/transform.example.json`](examples/transform.example.json)。

本次已核验帧 `120045958` 的 K 与外参数字已经放入示例 JSON；这些数值仅属于该历史帧，机器人实时运行必须填入当前帧数据。

## 6. 返回值与机器人消费规则

关键字段：

| 字段 | 含义 |
|---|---|
| `ok` | 对应类别的几何质量门与正式点均通过时才为 true |
| `sam3_call_count` | 所有 SKU 选箱成功后为 2；选箱失败为 1，且不执行商品 SAM3 |
| `selected_instance_id` | 第二阶段 SAM3 原始 `detections` 数组的 1-based 编号；与 `upstream_instance_id` 一致 |
| `filtered_instance_id` | 通过箱内归属过滤后的临时顺序，仅用于追溯 |
| `box_selection` | 箱候选、选中箱、ROI、两阶段阈值/耗时、每个商品归属和拒绝原因 |
| `front_panel_valid` | 选中箱前挡板平面和可见上沿中点是否通过独立质量门 |
| `front_panel_top_edge_midpoint_camera_mm` | 前挡板可见上沿中点，相机光学坐标，mm；无效时为 null |
| `front_panel_plane_point_camera_mm` | 前挡板平面上一点，相机光学坐标，mm |
| `front_panel_plane_normal_camera` | 前挡板平面单位法向，相机光学坐标，无量纲 |
| `sam3_score` | 选中实例的原始 SAM3 分数，只用于候选排序，不代表三维精度 |
| `axis_fit_valid` | 圆柱轴线是否通过几何质量门 |
| `reference_point_valid` | 可见瓶身轴段中心点是否通过轴线、内点、残差和 bootstrap 稳定性质量门 |
| `axis_point_camera_mm` | 相机光学坐标系中的轴线上一点；诊断字段 |
| `axis_direction_camera_up` | 相机光学坐标系中的朝上单位轴向量 |
| `reference_point_camera_mm` | 可见有效瓶身轴段的中心轴中点，相机光学坐标系，mm；无效时为 null |
| `reference_point_chassis_mm` | 同一可见轴段中点的 chassis 表示，mm；核查用，不要求固定 Z |
| `reference_z_mm` | bottle 兼容字段；当前模式下为 null/不参与正式计算 |
| `rejection_reasons` | 轴线或参考点拒绝原因 |
| `diagnostics` | 深度率、残差、跨度、bootstrap 稳定性和前排候选 |
| `artifacts` | 服务器上的 mask overlay、拟合 overlay、PLY 和日志路径 |

机器人端唯一允许进入后续控制的条件：

```python
usable = (
    response.get("ok") is True
    and response.get("axis_fit_valid") is True
    and response.get("reference_point_valid") is True
    and response.get("reference_point_camera_mm") is not None
    and response.get("axis_direction_camera_up") is not None
)
```

`box` 不使用固定 Z 参考平面；它返回实际拟合顶面上的 `top_point_camera_mm`。方盒消费条件为：

```python
usable = (
    response.get("ok") is True
    and response.get("top_plane_valid") is True
    and response.get("top_point_valid") is True
    and response.get("top_point_camera_mm") is not None
)
```

`tube` 也不使用固定 Z 参考平面。正式点是可见上边缘拟合线段中点，端点来自同一次拟合：

```python
usable = (
    response.get("ok") is True
    and response.get("edge_valid") is True
    and response.get("point_valid") is True
    and response.get("top_edge_center_camera_mm") is not None
    and response.get("top_edge_endpoints_camera_mm") is not None
    and response.get("point_semantics") == "visible_top_edge_midpoint"
)
```

其中 `point_semantics` 从 2026-09-23 起固定为 `visible_top_edge_midpoint`，与机器人边侧现有契约一致；后续不得改用其他字符串。`top_edge_center_camera_mm`、`top_edge_endpoints_camera_mm` 和 `edge_direction_camera` 均属于 `head_camera_color_optical_frame`，长度单位 mm（方向无量纲）。深度只在 mask 真实上轮廓向内 `top_boundary_depth_band_px`（默认 4 px）内取样；无深度列直接跳过。因此输出是“可见上边缘附近的内部诊断参考”，不是被遮挡完整封口的中心、物理封边毫米真值或固定 chassis 高度。

如果 `usable` 为 false：

- 不使用旧 XYZ；
- 不把 `axis_point_camera_mm` 当成固定高度参考点；
- 不自动改选第二高分实例；
- 不自动切换到 `prior_2d`；
- 本帧返回 invalid，由上层决定重新采集/重识别。

如果控制动作还依赖“箱体前挡板距离/上沿位置”，除商品自身的 `usable` 外还必须额外满足：

```python
front_panel_usable = (
    response.get("front_panel_valid") is True
    and response.get("front_panel_top_edge_midpoint_camera_mm") is not None
    and response.get("front_panel_plane_point_camera_mm") is not None
    and response.get("front_panel_plane_normal_camera") is not None
)
```

商品定位通过但 `front_panel_usable=false` 时，可以保留商品诊断结果，但不得执行依赖前挡板距离的动作。

控制端若以后要沿瓶轴加偏移，只能在参考点有效后计算：

```text
target_camera_mm = reference_point_camera_mm
                   + delta_h_mm * axis_direction_camera_up
```

`delta_h_mm` 的值、碰撞检查、夹爪补偿和运动规划均属于控制端，本接口不提供这些验证。

### 6.1 `target_type=basket` 输出

`target_type=basket` 成功时主要读取：

| 字段 | 含义 |
|---|---|
| `ok` / `pose_valid` | 是否通过有限值与齐次矩阵检查 |
| `model_center_camera_mm` | **篮筐 CAD 模型中心点**，相机光学坐标，mm |
| `reference_point_camera_mm` | 同一个模型中心点（控制端消费字段） |
| `reference_point_chassis_mm` | 同一中心点的 chassis 表示，mm |
| `xyz_camera_mm` | 同一中心点（兼容字段） |
| `point_semantics` | 固定 `basket_model_center` |
| `object_origin_camera_mm` | `pose_4x4` 的平移列，即 CAD 原点在相机系的位置，mm（仅追溯用） |
| `model_center_offset_m` | 模型中心在 CAD 网格坐标系中的偏移，m |
| `pose_4x4` | CAD 网格坐标 → 相机光学坐标的 4x4 位姿，平移列 mm |
| `pose_4x4_input_m` | 同一 4x4 位姿，平移列 m |
| `rotation_euler_zyx_rad` | ZYX 欧拉角，rad |
| `xyzrxryrz_camera_mm_rad` | `[中心 x, 中心 y, 中心 z, rx, ry, rz]` 打包 |

与 SKU 的 `front_panel_*` 等字段不同，basket 输出不包含商品几何位；`selected_instance_id`、`sam3_score`、`mask_prompt` 用于记录所选篮筐 mask。

消费判据：

```python
basket_usable = (
    response.get("ok") is True
    and response.get("pose_valid") is True
    and finite_xyz(response.get("model_center_camera_mm"))
    and response.get("point_semantics") == "basket_model_center"
)
```

`pose_4x4` 与 `model_center_camera_mm` 是同一个位姿的两种表达：

```text
model_center_camera_mm = (R @ model_center_offset_m) * 1000 + t_mm
```

**不要**直接拿 `pose_4x4` 的平移列当作篮筐中心——该 CAD 的原点在箱体角上，两者相差约 318 mm。该输出目前只完成服务链路和有限值检查，不是机器人篮筐抓取精度验收。

## 7. 响应示例

- 成功契约：[`examples/infer_response_ok.example.json`](examples/infer_response_ok.example.json)
- 质量门失败：[`examples/infer_response_invalid.example.json`](examples/infer_response_invalid.example.json)

这些响应文件是关键字段结构示例，不是完整响应，也不是新的实测结果。真实结果还会包含 `front_row`、`input_summary`、完整 `diagnostics` 和 `artifacts`；以每次 HTTP 响应和对应 `request_id` 日志为准。

可复制的本地 fixture 命令：

```powershell
$py = 'C:\Users\14817\miniconda3\envs\pt\python.exe'
& $py 'D:\TermiTech\Cosmetics_Sort\deploy\robot_request_example.py' `
  --url 'http://211.137.21.33:25540/infer' `
  --sku-typ bottle `
  --rgb 'D:\TermiTech\Cosmetics_Sort\data\20260915\120045958\head_rgb.jpg' `
  --depth 'D:\TermiTech\Cosmetics_Sort\data\20260915\120045958\head_depth_aligned.npy' `
  --camera 'D:\TermiTech\Cosmetics_Sort\data\20260915\120045958\camera.json' `
  --transform-json 'D:\TermiTech\Cosmetics_Sort\deploy\examples\transform.example.json' `
  --out 'D:\TermiTech\Cosmetics_Sort\deploy\robot_api_response.json'
```

tube 的完整验证应继续使用 `test_client.py`（它会校验最高分选择、端点中点和 chassis 水平约束）：

```powershell
& $py 'D:\TermiTech\Cosmetics_Sort\deploy\test_client.py' --sku-typ tube --target-box 1
```

该命令只发离线诊断 HTTP 请求，不调用机器人控制接口。

## 8. 错误处理

- HTTP 200：请求被处理；仍必须检查 JSON 中的 `ok` 和两个 validity 字段。
- HTTP 400：输入 schema、base64、尺寸、K、外参、帧名或拟合过程异常；响应包含 `error`。
- 网络超时/连接失败：本帧无结果，不能复用上帧坐标。
- `/health` 失败：停止发送定位请求并报警；不得执行动作。

## 9. 参考点复核

机器人端可在消费前复核返回参考点：

```python
p_chassis_mm = R @ p_camera_mm + t_mm
assert response.get("reference_mode") == "visible_axis_midpoint"
assert np.isfinite(p_camera_mm).all()
```

若 `T_unit="m"`，必须先执行 `t_mm = 1000 * t`。该检查只验证坐标换算和可见轴段中心语义，不证明物理抓取精度。

## 9.1 完整服务器处理流程

三类商品共用以下前半段，不要求调用端自行生成 mask：

```text
当前 RGB + 对齐深度(mm) + K + T_chassis_camera
  -> 校验尺寸、内参、外参、单位和坐标系
  -> 按 sku_typ 读取服务器类别配置
  -> 箱体 SAM3 -> 按 side 选择左/右箱 -> 商品 SAM3 -> 箱内 ROI 过滤
  -> 可选 chassis 前后带宽过滤
  -> 每个请求只建立一次相机点图和 chassis_link 点图
  -> 按类别执行圆柱轴线 / 顶面 / 可见上边缘拟合
  -> 类别质量门
  -> 返回相机坐标正式点、chassis 核查字段、日志和可视化
```

可选前后规则：

```json
"front_rule": {
  "front_axis_chassis": [0.0, 1.0, 0.0],
  "front_origin_chassis": [0.0, 0.0, 0.0],
  "front_band_mm": 100000.0
}
```

`front_axis_chassis` 的正负方向及 `front_origin_chassis` 必须由现场坐标系确认。`100000 mm` 只是联调时“不实际裁掉候选”的宽带配置，不证明已经完成真实前后排筛选。正式使用应配置经过场景测量的原点和带宽；不提供时，box/tube 只执行选箱 ROI 过滤，服务不会猜测任意相机轴或 chassis 轴。

在当前 `125311198` 离线帧中，`[1,0,0]` 与实测前挡板法向一致，`[0,1,0]` 会被法向先验门拒绝；这只是该场景证据，部署到不同机器人/货架位仍需复核。box-first 响应另外返回：

- `front_panel_valid`；
- `front_panel_top_edge_midpoint_camera_mm` / `..._chassis_mm`；
- `front_panel_plane_point_camera_mm` / `..._chassis_mm`；
- `front_panel_plane_normal_camera` / `..._chassis`；
- `box_selection.front_panel` 中的点数、残差、跨度、竖直误差和先验夹角。

前挡板失败与商品定位质量门分开记录：不得伪造前挡板点，但不会把已经独立通过的商品几何结果改写为成功的前挡板结果。

## 10. 历史记录：远程 SAM3 与板端诊断迁移（2026-09-17，已被 A800 25540 取代）

本节仅保留历史追溯，不是当前控制端地址。当前接口以本文第 2 节的 A800 `211.137.21.33:25540` 为准。

板端诊断服务地址为 `http://192.168.130.105:18082/infer`，上游配置为：

```text
SAM3_BACKEND=multipart_segment
SAM3_URL=http://211.137.21.33:25541/api/v1/segment
SAM3_MASK_THRESHOLD=0.5
```

它只向远程 SAM3 发送无损 PNG、prompt、threshold 和 mask_threshold；depth、K、`T_chassis_camera`、关节状态和凭据只留在板端。新后端失败时服务返回明确错误，不会自动切回旧 18003。

当前迁移验收未通过，不能把板端服务作为正式默认：

- estee 左箱端到端有效，5 次重复均选择 #3，正式点约 `[-374.6013, 81.1745, 531.9154] mm`；
- Avene 的类别名是 `Avene`，SAM3 分割提示词是 `The main cylindrical body of each white bottle`，`threshold=0.5`、`mask_threshold=0.5`。此前把类别名误作 prompt 的 0/5 结果属于无效配置测试；修正后板端 5/5 检出 7 个实例并通过定位门，均选择 #3、score `0.7521561`；
- origins 的箱体阶段稳定返回跨左右箱的合并候选，现有保守规则以 `merged_box_proposal` 拒绝，第二次商品推理没有执行。

当时 Windows 默认 URL 仍为旧 4090-1 服务；以下命令仅为历史板端诊断示例，不用于当前接入：

```powershell
& $py 'D:\TermiTech\Cosmetics_Sort\deploy\test_client.py' `
  --url 'http://192.168.130.105:18082/infer' `
  --class-name estee --target-box 1
```

板端服务使用系统 Python 3.10.12，并将唯一缺失依赖 `pycocotools` 隔离安装在 `/home/admin/sam3/deploy/vendor`。旧 4090-1 服务和 18003 均保留。若需回退客户端，仅恢复旧 URL 或使用板端备份 `/home/admin/sam3/deploy/backups/20260917_multipart_migration_before/test_client.py`。

新 SAM3 是明文 HTTP；没有 HTTPS 或受控网络保障时，不应声称传输已加密。板端服务只是无动作视觉诊断，不得据此触发机器人动作。

## 11. A800 `quinn-server` 25540 服务（2026-09-18）

服务目录：`/home/quinn/cosmetics_pose/deploy`（2026-09-19 由 `/home/quinn/pose` 改名）。启动脚本已封装 tmux，可直接运行：

```bash
cd /home/quinn/cosmetics_pose/deploy
./start_25540.sh
```

服务监听 `0.0.0.0:25540`，tmux 会话名为 `pose_25540`；上游 SAM3 固定为同机 `http://127.0.0.1:25541/api/v1/segment`。25540 服务不加载 SAM3 权重、不执行机器人动作。该脚本目前是手动 tmux 启动方式，尚未配置 systemd 开机自启动。

已从 Windows 原客户端实际验证：

- URL：`http://211.137.21.33:25540/infer`；
- 类别：`Avene`，SAM3 prompt=`The main cylindrical body of each white bottle`；
- HTTP 200，`ok=true`，`sam3_call_count=2`；
- 选中实例 `#3`，score `0.752156138420105`，7 个前排候选中最高；
- `axis_fit_valid=true`、`reference_point_valid=true`；
- chassis 参考高度 Z=1200 mm，复核误差约 `-2.8e-10 mm`；
- 客户端 `accepted=true`，正式相机参考点为 `[162.7780935, 27.6334503, 532.4414145] mm`。

本次 Avene 使用 `side=RIGHT`，右箱 ROI 和最终瓶身叠加图已人工检查；该记录仍是无动作诊断通过，不代表机器人抓取已经完成。
