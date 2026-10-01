# 位姿估计可视化简化

## 修改目标
简化 `/debug` 界面的位姿估计结果可视化，从"显示所有数据"转变为"只显示验证准确性所需的最小集"。

## 修改内容

### 1. 精简显示的3D点
**之前：** 显示8个点
- reference_point_camera_mm（参考点）
- axis_point_camera_mm（轴线点）
- front_panel_top_edge_midpoint_camera_mm（前挡板上沿）
- front_panel_plane_point_camera_mm（前挡板平面点）
- top_point_camera_mm（顶面点）
- top_edge_center_camera_mm（上沿中点）
- model_center_camera_mm（篮筐中心）
- object_origin_camera_mm（CAD原点）

**之后：** 只显示2个关键点
- ✅ **reference_point_camera_mm**（参考点）- 实际抓取目标位置
- ✅ **model_center_camera_mm**（篮筐中心）- 仅用于篮筐定位

**理由：** 其他点是算法中间步骤，对验证最终位姿准确性无直接作用。

### 2. 参考点颜色编码
**新增功能：** 参考点根据ok状态和sam3_score动态着色

| 状态 | 颜色 | 说明 |
|------|------|------|
| ok=False | 🔴 红色 `#ef4444` | 位姿估计失败 |
| ok=True, score≤0.7 | 🟠 橙色 `#fb923c` | 低置信度 |
| ok=True, score>0.7 | 🟢 绿色 `#22c55e` | 高置信度成功 |

**同时增大参考点尺寸：** 添加 `"size": "large"` 属性，便于快速识别。

### 3. 简化线段显示
**之前：** 显示3类线段
- 抓取轴线
- 可见上沿线段
- CAD坐标系（XYZ轴）

**之后：** 只显示2类
- ✅ **抓取轴线** - 从 axis_point 沿 axis_direction 延伸，长度增加到±120mm，加粗到3px
- ✅ **CAD坐标系**（仅篮筐） - XYZ三轴，长度120mm，宽度2px
- ❌ 删除可见上沿线段（中间步骤）

### 4. 删除所有检测框
**之前：** 显示多种框
- container_roi_xyxy（选中箱ROI）
- left_right_rois_xyxy（左右箱）
- box_candidates（候选框，最多6个）

**之后：** 全部删除
- `_boxes()` 函数直接返回空列表

**理由：** 这些是perception模块的中间输出，不是位姿估计的最终结果，对验证位姿准确性无帮助。

### 5. 精简实例标记
**之前：** 显示所有实例质心（最多8个）

**之后：** 只显示选中的实例
- 根据 `selected_instance_id` 过滤
- 只保留匹配的那一个实例标记

**理由：** 只需要确认"系统认为在抓哪个物体"，其他候选实例是干扰信息。

### 6. 简化HUD显示
**之前：** 4行信息
```
ok · target_type · sku_typ · localization_method
score 0.895 · 实例 12
参考点 123.4, 56.7, 890.1 mm
拒绝: depth_invalid, ...
```

**之后：** 3行精简信息
```
ok · sku · bottle · score 0.752
参考点 162.8, 27.6, 532.4 mm
拒绝: depth_invalid, out_of_reach  (仅失败时显示)
```

**改动：**
- ❌ 删除 `localization_method` 和 `point_semantics`（调试细节）
- ❌ 删除 `实例 {id}`（已有实例标记）
- ✅ 将 `score` 合并到第一行

## 效果对比

### SKU抓取（成功）
**之前：** 8个点 + 2条线 + 多个框 + 多个实例标记
**之后：** 1个大号绿色参考点 + 1条加粗轴线 + 1个实例标记

### SKU抓取（失败）
**之前：** 同样的8个点（看不出失败）
**之后：** 1个红色参考点 + HUD显示拒绝原因

### 篮筐定位
**之前：** 8个点 + XYZ轴 + 多余的框
**之后：** 2个点（参考点+篮筐中心） + XYZ轴

## 测试验证
新增测试用例：
- `test_reference_point_color_coding` - 验证颜色编码
- `test_only_selected_instance_marker` - 验证实例过滤

所有测试通过 ✅

## 修改文件
- `agent/src/agent/debug/overlay.py` - 主要修改
- `agent/tests/test_debug_overlay.py` - 更新测试

## 向前兼容性
- ✅ 保留了 `overlay` 数据结构的所有字段（points, lines, boxes, markers, hud）
- ✅ 前端渲染逻辑无需修改（只是数据量减少）
- ✅ 只是内容精简，不影响API契约
