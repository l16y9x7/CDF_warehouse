# Agent 放置流程简化 - 修改总结

## 修改目标

简化 agent 侧的放置流程，移除位姿估计步骤，直接适配 mui 底层的放置接口（使用固定的胸部中线方法）。

## 修改的文件

### 1. agent/src/agent/skills/place_sku_in_basket.py

**改动**：
- 移除 `infer_head_basket_pose()` 调用
- 移除依赖：`CameraCapability`, `EstimationCapability`, `BodyPoseCapability`
- 移除 import: `basket_infer`
- 简化 `__init__` 方法，只保留 `ManipulationCapability`
- `execute` 方法直接调用 `manipulation.place()`，不传递 `localization_result`

**影响**：
- PlaceSkuInBasketSkill 不再进行位姿估计
- 放置操作完全依赖 mui 底层实现

### 2. agent/src/agent/capabilities/manipulation/contract.py

**改动**：
- `PlaceRequest.__post_init__` 验证逻辑
- 移除对 `localization_result` 的强制要求
- 错误消息从 "basket place requires SORTING sku_typ and basket localization" 改为 "basket place requires SORTING sku_typ"

**影响**：
- `PlaceRequest` 可以不提供 `localization_result`
- 保持向后兼容：仍然接受带 `localization_result` 的请求

### 3. agent/src/agent/capabilities/manipulation/http_adapter.py

**改动**：
- `place()` 方法的 HTTP body 构建逻辑
- 只在 `localization_result` 不为 `None` 时才添加到 body 中
- 从 `body["localization_result"] = dict(request.localization_result or {})` 改为条件添加

**影响**：
- 不带 `localization_result` 的请求不会发送空字典
- HTTP 请求更简洁，符合 mui 接口期望

### 4. agent/src/agent/application.py

**改动**：
- `build_application_from_capabilities` 函数中的技能构建
- PlaceSkuInBasketSkill 初始化从 `PlaceSkuInBasketSkill(c, e, m, capabilities["pose"])` 改为 `PlaceSkuInBasketSkill(m)`

**影响**：
- 简化依赖注入
- 减少不必要的能力初始化

### 5. agent/tests/test_skills.py

**改动**：
- `test_place_sku_in_basket_uses_head_basket_infer_then_place` 测试方法
- 移除 `camera`, `estimation`, `pose` 的 mock 对象
- 移除对位姿估计的验证
- 验证 `localization_result` 为 `None`

**影响**：
- 测试反映新的简化流程
- 验证不再进行位姿估计

### 6. agent/tests/test_capabilities.py

**新增**：
- `test_place_basket_without_localization_result` 测试方法
- 验证不带 `localization_result` 的放置请求
- 确认 HTTP body 不包含 `localization_result` 字段

**影响**：
- 增加测试覆盖率
- 确保新流程正确工作

## HTTP 接口变化

### 修改前（带位姿估计）

```json
POST /manipulation/place
{
  "task_type": "SORTING",
  "target_type": "sku",
  "destination_type": "basket",
  "hand": "RIGHT",
  "sku_typ": "bottle",
  "localization_result": {
    "pose_valid": true,
    "point_semantics": "basket_model_center",
    "model_center_camera_mm": [200.0, 30.0, 520.0],
    ...
  }
}
```

### 修改后（直接放置）

```json
POST /manipulation/place
{
  "task_type": "SORTING",
  "target_type": "sku",
  "destination_type": "basket",
  "hand": "RIGHT",
  "sku_typ": "bottle"
}
```

## 工作流程对比

### 修改前

```
sorting_item workflow (S-I08)
  ↓
place_sku_in_basket skill
  ↓
infer_head_basket_pose()
  ├─ pose.camera_transform() (获取相机外参)
  ├─ camera.capture() (拍摄 RGB-D)
  └─ estimation.estimate_basket_pose() (位姿估计)
      ↓
manipulation.place(localization_result=...)
  ↓
HTTP POST /manipulation/place (带 localization_result)
  ↓
mui agent_actions.place() (忽略 localization_result)
  ↓
placement_sequence.execute() (使用固定胸部中线)
```

### 修改后

```
sorting_item workflow (S-I08)
  ↓
place_sku_in_basket skill
  ↓
manipulation.place() (不传 localization_result)
  ↓
HTTP POST /manipulation/place (不带 localization_result)
  ↓
mui agent_actions.place()
  ↓
placement_sequence.execute() (使用固定胸部中线)
```

## 向后兼容性

✅ 保持向后兼容：
- `PlaceRequest` 仍然接受 `localization_result` 参数（可选）
- HTTP 适配器在提供时仍会发送 `localization_result`
- 旧代码如果传递 `localization_result` 仍然可以工作
- mui 底层可以选择使用或忽略 `localization_result`

## 测试验证

所有测试通过：
- ✅ PlaceRequest 不要求 localization_result
- ✅ PlaceSkuInBasketSkill 不再需要 camera/estimation/pose
- ✅ HTTP body 构建正确（不包含 localization_result）
- ✅ 向后兼容（仍支持带 localization_result 的调用）

测试脚本：`test_placement_simplification.py`

## 依赖变化

### PlaceSkuInBasketSkill 依赖

**修改前**：
- CameraCapability
- EstimationCapability
- ManipulationCapability
- BodyPoseCapability

**修改后**：
- ManipulationCapability

## 不受影响的组件

- ✅ sorting workflow (`workflows/sorting.py`)
- ✅ 其他技能（push_basket, pick_review_basket 等仍使用位姿估计）
- ✅ mui 底层实现（无需修改）
- ✅ 配置文件（workflows.yaml, products.yaml）

## 性能改进

- 减少每次放置操作的位姿估计调用
- 减少 RGB-D 图像拍摄和处理
- 简化技能依赖，减少初始化开销
- 更快的放置响应时间

## 相关文档

- 计划文档：`.claude/plans/placement-simplification.md`
- 测试脚本：`test_placement_simplification.py`
