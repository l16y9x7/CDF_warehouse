# `/estimation/pick_pose` 与 Cosmetics_Sort 定位服务适配说明

> **2026-09-21 协议切换（严格替换）**：25540 正式类别字段为 `sku_typ`，值 `bottle`/`box`/`tube`（对应旧 `Avene`/`estee`/`origins`：Avene→bottle、estee→box、origins→tube）。请求中出现 `sku_id` 或 `class_name` 将被 HTTP 400 拒绝；响应统一输出 `sku_typ`，不再输出 `sku_id`。文中历史实测小节保留旧名称。

## 1. 当前接口关系

机器人系统可以继续对外保留：

```http
POST /estimation/pick_pose
Content-Type: application/json
```

当前已经部署并实测的是 A800 无动作视觉定位服务：

```http
GET  http://211.137.21.33:25540/health
POST http://211.137.21.33:25540/infer
Content-Type: application/json
```

`/estimation/pick_pose` 如果由机器人控制系统提供，需要在控制端或网关中完成一次轻量字段转换，再调用 `/infer`。25540 不主动控制机器人、不生成运动轨迹，也没有实现 `/estimation/pick_pose` 路由。

## 2. 上层 `/estimation/pick_pose` 请求

| 字段 | 类型 | 必填 | 说明 |
|---|---|---:|---|
| `task_type` | string | 是 | `SORTING` 或 `REVIEW`，用于上层任务审计；25540 不据此切换几何算法 |
| `target_type` | string | 是 | `sku` 或 `basket`；`basket` 使用服务器内置 Basket CAD 和 FoundationPose，返回篮筐模型中心点与位姿 |
| `sku_typ` | string | 是 | `bottle`、`box` 或 `tube`（旧 `Avene`/`estee`/`origins` 的严格替换）；SORTING/REVIEW 均必须明确类别 |
| `side` | string | 是 | 所有 SKU 均填写 `LEFT` 或 `RIGHT`，选择当前图像所选箱对的左箱或右箱 |
| `rgb` | string | 是 | 当前帧 RGB 文件路径；由适配层读取文件字节，不把机器人本地路径直接交给 A800 |
| `depth` | string | 是 | 同帧、对齐 RGB 的二维浮点 NPY 路径，数值单位 mm |
| `K` | 3x3 array | 是 | 当前 RGB 投影模型和分辨率对应的针孔内参 |
| `T_chassis_camera` | 4x4 array | 是 | 当前帧外参，满足 `p_chassis = R @ p_camera + t` |
| `T_unit` | string | 是 | `m` 或 `mm`，只描述外参平移列单位 |
| `camera_frame` | string | 是 | 固定 `head_camera_color_optical_frame` |
| `base_frame` | string | 是 | 固定 `chassis_link` |
| `front_rule` | object | 否 | chassis 前后方向、原点和带宽；省略时使用服务端联调默认值，bottle 用于前排选择，box-first 还用于前挡板和商品前后过滤 |
| `mask` | string | 否 | 当前 25540 不需要；服务会按 `sku_typ` 调用 SAM3。上层保留该字段时，也不能把它解释为已经替代服务内 SAM3 |

SAM3 prompt、阈值、bottle 半径、拟合算法、选箱开关以及 `front_rule` 均由服务器提供默认配置。调用方可以使用 `/infer` 已支持的可选覆盖字段，但普通机器人请求不需要重复发送全部内部参数。当前服务端 `front_rule` 默认值为 `front_axis_chassis=[1,0,0]`、`front_origin_chassis=[0,0,0]`、`front_band_mm=100000`，与回归客户端一致；这是联调宽带，不是现场标定值。

## 3. 类别与处理流程

```text
target_type=sku
└─ 统一识别：箱体 SAM3 → side 选左/右箱 → 前挡板 → 商品 SAM3 → 箱内过滤
   ├─ sku_typ=bottle → 已知半径圆柱轴线 → 可见瓶身轴段中心点
   ├─ sku_typ=box   → 顶面点
   └─ sku_typ=tube → 可见上边缘中点
```

所有 SKU：

```text
side=LEFT  -> 当前图像所选箱对的左箱 -> target_box=1
side=RIGHT -> 当前图像所选箱对的右箱 -> target_box=2
```

`side` 不是机器人 chassis 左右轴、SAM3 实例编号或跨帧箱体 ID。服务会把 `LEFT/RIGHT` 映射为 `target_box=1/2`。

## 4. 适配到 `/infer`

适配层必须：

1. 读取 `rgb` 指向的 JPG/PNG 文件字节，Base64 编码为 `rgb_base64`；
2. 读取 `depth` 指向的完整 `.npy` 文件字节，Base64 编码为 `depth_npy_base64`；
3. 保持 RGB、depth、K、外参来自同一次采集；
4. 发送当前帧 `T_chassis_camera`，不能使用当前实时姿态替换采集姿态；
5. 所有 SKU 都将 `side` 映射到箱体选择；
6. 不把文件路径当作 A800 能直接访问的共享路径。

转换后的 `/infer` 请求示意：

```json
{
  "target_type": "sku",
  "sku_typ": "box",
  "side": "LEFT",
  "rgb_base64": "<RGB文件字节base64>",
  "depth_npy_base64": "<NPY文件字节base64>",
  "depth_unit": "mm",
  "K": [[<fx>, 0.0, <cx>], [0.0, <fy>, <cy>], [0.0, 0.0, 1.0]],
  "T_chassis_camera": [
    [<r00>, <r01>, <r02>, <tx>],
    [<r10>, <r11>, <r12>, <ty>],
    [<r20>, <r21>, <r22>, <tz>],
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

`front_rule` 整体可以省略，服务端会使用上述默认值；显式发送时会覆盖服务端默认。尖括号均为占位符，不能原样发送。`front_band_mm=100000` 只是当前联调宽带，几乎不会过滤后排；机器人动作前必须使用现场确认的轴、原点和带宽。

## 5. 上层请求示例

### 5.1 bottle

```json
{
  "task_type": "SORTING",
  "target_type": "sku",
  "sku_typ": "bottle",
  "side": "RIGHT",
  "rgb": "/shared/frames/capture-003/rgb.jpg",
  "depth": "/shared/frames/capture-003/depth_mm.npy",
  "K": "<当前帧3x3内参>",
  "T_chassis_camera": "<当前帧4x4矩阵>",
  "T_unit": "m",
  "camera_frame": "head_camera_color_optical_frame",
  "base_frame": "chassis_link",
  "front_rule": "<可选：现场确认的chassis前排规则>"
}
```

bottle 的 `side` 会实际选择左箱或右箱；不能省略后再假设服务会自动选择正确箱体。

### 5.2 box

```json
{
  "task_type": "SORTING",
  "target_type": "sku",
  "sku_typ": "box",
  "side": "LEFT",
  "rgb": "/shared/frames/capture-004/rgb.jpg",
  "depth": "/shared/frames/capture-004/depth_mm.npy",
  "K": "<当前帧3x3内参>",
  "T_chassis_camera": "<当前帧4x4矩阵>",
  "T_unit": "m",
  "camera_frame": "head_camera_color_optical_frame",
  "base_frame": "chassis_link",
  "front_rule": "<现场确认的chassis前后规则>"
}
```

## 6. 商品正式输出

| `sku_typ` | 正式相机坐标字段 | 其他必要字段 |
|---|---|---|
| `bottle` | `reference_point_camera_mm` | `axis_fit_valid`、`reference_point_valid`、`axis_direction_camera_up` |
| `box` | `top_point_camera_mm` | `top_plane_valid`、`top_point_valid` |
| `tube` | `top_edge_center_camera_mm` | `edge_valid`、`point_valid`、`top_edge_endpoints_camera_mm` |

所有正式 XYZ 均为：

```text
frame = head_camera_color_optical_frame
unit  = mm
```

当前服务不输出经过验证的完整 6D 姿态。上层若保留：

```json
"pose": [x, y, z, rx, ry, rz]
```

不得伪造 `rx/ry/rz`。建议适配层返回 `pose: null`，同时透传上述真实商品字段；或者由控制端另行定义只包含 XYZ 的字段。

## 7. 前挡板输出

box-first 请求还会返回：

```json
{
  "front_panel_valid": true,
  "front_panel_top_edge_midpoint_camera_mm": [x, y, z],
  "front_panel_top_edge_midpoint_chassis_mm": [x, y, z],
  "front_panel_plane_point_camera_mm": [x, y, z],
  "front_panel_plane_point_chassis_mm": [x, y, z],
  "front_panel_plane_normal_camera": [nx, ny, nz],
  "front_panel_plane_normal_chassis": [nx, ny, nz]
}
```

平面在 chassis 中表示为：

```text
dot(front_panel_plane_normal_chassis,
    p_chassis - front_panel_plane_point_chassis_mm) = 0
```

前挡板使用第一次箱体 SAM3 的选中 mask，不增加第三次 SAM3。前挡板有效性和商品有效性分开；一个通过不能代替另一个。

## 7.1 响应示例

以下为 25540 `/infer` 返回、控制端需要消费的关键字段精简示例。建议 `/estimation/pick_pose` 适配层原样透传这些字段，不要重命名成含义不明确的 `pose`。真实响应还会包含 `request_id`、完整 `box_selection`、`diagnostics`、`artifacts` 和 `pipeline_steps`。

### 7.1.1 bottle 成功

数值来自当前 A800 25540 实测结果：

```json
{
  "ok": true,
  "target_type": "sku",
  "sku_typ": "bottle",
  "class_name": "bottle",
  "sam3_call_count": 2,
  "localization_method": "bottle_cylinder_axis",
  "selected_instance_id": 3,
  "upstream_instance_id": 3,
  "sam3_score": 0.752156138420105,
  "axis_fit_valid": true,
  "reference_point_valid": true,
  "axis_point_camera_mm": [
    171.384239628949,
    -183.304701396738,
    441.527730118060
  ],
  "axis_direction_camera_up": [
    0.037441295923,
    -0.917692728411,
    -0.395522699200
  ],
  "reference_point_camera_mm": [
    162.778093460864,
    27.633450272577,
    532.441414545861
  ],
  "reference_point_chassis_mm": [
    574.681812178851,
    -169.314441068786,
    1199.999999999720
  ],
  "reference_z_mm": 1200.0,
  "output_frame": "head_camera_color_optical_frame",
  "output_unit": "mm",
  "side_audit": {
    "target_type": "sku",
    "sku_typ": "bottle",
    "side_received": "RIGHT",
    "side_applied": true,
    "side_note": null
  },
  "rejection_reasons": []
}
```

注意：`axis_point_camera_mm` 是无限轴线上的规范点，不是最终抓取点；正式参考点是 `reference_point_camera_mm`，其当前语义为 `visible_axis_midpoint`，即有效瓶身拟合内点投影范围的稳健中点。

### 7.1.2 box 商品和前挡板均成功

数值来自当前左箱实测结果：

```json
{
  "ok": true,
  "target_type": "sku",
  "sku_typ": "box",
  "class_name": "box",
  "sam3_call_count": 2,
  "localization_method": "box_top_surface",
  "selected_instance_id": 3,
  "upstream_instance_id": 3,
  "sam3_score": 0.8906083106994629,
  "top_plane_valid": true,
  "top_point_valid": true,
  "point_semantics": "visible_top_interior_point",
  "top_point_camera_mm": [
    -374.601276734913,
    81.174467468787,
    531.915411698958
  ],
  "top_point_chassis_mm": [
    576.881473603392,
    368.084170057180,
    1146.624234420660
  ],
  "front_panel_valid": true,
  "front_panel_top_edge_midpoint_camera_mm": [
    -253.999600630634,
    110.800761540226,
    497.638041925899
  ],
  "front_panel_top_edge_midpoint_chassis_mm": [
    527.518439364515,
    249.587486340972,
    1135.704231805650
  ],
  "front_panel_plane_point_camera_mm": [
    -199.767709234280,
    148.942265851011,
    515.218374686212
  ],
  "front_panel_plane_normal_camera": [
    0.051102398948,
    0.411292830353,
    -0.910069641578
  ],
  "output_frame": "head_camera_color_optical_frame",
  "output_unit": "mm",
  "rejection_reasons": []
}
```

`top_point_camera_mm` 是商品顶面点；`front_panel_top_edge_midpoint_camera_mm` 是箱体前挡板可见上沿中点，两者不可混用。

### 7.1.3 tube 选箱阶段拒绝

这是当前真实的保守失败分支：

```json
{
  "ok": false,
  "target_type": "sku",
  "sku_typ": "tube",
  "class_name": "tube",
  "sam3_call_count": 1,
  "localization_method": "tube_top_edge",
  "selected_instance_id": null,
  "edge_valid": false,
  "point_valid": false,
  "top_edge_center_camera_mm": null,
  "top_edge_endpoints_camera_mm": null,
  "front_panel_valid": false,
  "front_panel_top_edge_midpoint_camera_mm": null,
  "rejection_reasons": [
    "box selection failed: merged_box_proposal"
  ]
}
```

该响应虽然可能是 HTTP 200，但不得进入抓取动作，也不得复用上一帧坐标。

## 8. 调用方成功条件

调用方不能只检查 HTTP 200。

以下伪代码中的 `finite_xyz(value)` 表示：值能够转换为长度为 3 的浮点数组，并且三个数均为有限值（不是 `NaN` 或 `Inf`）。

bottle：

```python
product_usable = (
    response.get("ok") is True
    and response.get("axis_fit_valid") is True
    and response.get("reference_point_valid") is True
    and finite_xyz(response.get("reference_point_camera_mm"))
    and finite_xyz(response.get("axis_direction_camera_up"))
)
```

box：

```python
product_usable = (
    response.get("ok") is True
    and response.get("top_plane_valid") is True
    and response.get("top_point_valid") is True
    and finite_xyz(response.get("top_point_camera_mm"))
)
```

tube：

```python
product_usable = (
    response.get("ok") is True
    and response.get("edge_valid") is True
    and response.get("point_valid") is True
    and finite_xyz(response.get("top_edge_center_camera_mm"))
)
```

依赖前挡板距离或上沿的动作，还必须检查：

```python
front_panel_usable = (
    response.get("front_panel_valid") is True
    and finite_xyz(response.get("front_panel_top_edge_midpoint_camera_mm"))
    and finite_xyz(response.get("front_panel_plane_point_camera_mm"))
    and finite_xyz(response.get("front_panel_plane_normal_camera"))
)
```

任一所需条件失败时：不执行动作、不复用上一帧 XYZ、不自动改选第二高分实例、不自动切换拟合算法，并记录 `rejection_reasons` 后重新采集或交给上层决策。

## 9. 当前验证和未完成项

当前 A800 `25540` 已完成：

- Avene 右箱统一两阶段真实 HTTP 请求通过；
- estee 左箱统一两阶段真实 HTTP 请求通过；
- estee 当前帧前挡板拟合有效并已查看叠加图；
- 服务不执行机器人动作。

仍未完成：

- `target_type=basket` 已接入 25540：服务器使用 `the central white plastic basket`、threshold=0.7 生成 mask，再调用本机 25550 FoundationPose；客户端不上传 CAD。返回篮筐 CAD 模型中心点与完整 4x4 位姿，字段见下方补充。

### Basket 响应补充

`target_type=basket` 成功时，主要读取：

```json
{
  "ok": true,
  "target_type": "basket",
  "pose_valid": true,
  "cad_id": "Basket",
  "point_semantics": "basket_model_center",
  "model_center_camera_mm": [x, y, z],
  "reference_point_camera_mm": [x, y, z],
  "reference_point_chassis_mm": [x, y, z],
  "xyz_camera_mm": [x, y, z],
  "object_origin_camera_mm": [x, y, z],
  "model_center_offset_m": [0.245, 0.1825, -0.087651],
  "pose_4x4": [[r00, r01, r02, tx], [r10, r11, r12, ty], [r20, r21, r22, tz], [0, 0, 0, 1]],
  "pose_4x4_input_m": [[r00, r01, r02, tx_m], [r10, r11, r12, ty_m], [r20, r21, r22, tz_m], [0, 0, 0, 1]],
  "rotation_euler_zyx_rad": [rx, ry, rz],
  "xyzrxryrz_camera_mm_rad": [x, y, z, rx, ry, rz],
  "output_frame": "head_camera_color_optical_frame",
  "output_unit": "mm",
  "rejection_reasons": []
}
```

`model_center_camera_mm`（以及同值的 `reference_point_camera_mm` / `xyz_camera_mm`）是**篮筐 CAD 模型中心点**，控制端直接用这个点，不要用 `pose_4x4` 的平移列。

`pose_4x4` 是同一帧的完整位姿（Basket CAD 网格坐标 → 相机光学坐标），旋转无量纲、平移列以 mm 表示（`pose_4x4_input_m` 是同一矩阵、平移列改用 m）。该 CAD 的坐标原点落在箱体角上，所以 `pose_4x4` 的平移列（`object_origin_camera_mm`）**不等于**模型中心，两者相差约 318 mm。换算关系：

```text
model_center_camera_mm = (R @ model_center_offset_m) * 1000 + t_mm
```

消费判据：

```python
basket_usable = (
    response.get("ok") is True
    and response.get("pose_valid") is True
    and finite_xyz(response.get("model_center_camera_mm"))
    and response.get("point_semantics") == "basket_model_center"
)
```

该输出目前只完成服务链路和有限值检查，尚不是机器人篮筐抓取精度验收。
- origins 当前上游箱体结果包含跨两箱合并 proposal，服务保守拒绝；
- `front_band_mm=100000` 只是联调宽带，真实前后排范围尚需现场标定；
- 未验证机器人抓取成功率、碰撞安全或实际毫米精度；
- `/estimation/pick_pose` 路由本身需要机器人控制端/网关实现，当前部署路由是 `/infer`。

底层完整字段说明见 [ROBOT_API_HANDOFF.md](ROBOT_API_HANDOFF.md)。
