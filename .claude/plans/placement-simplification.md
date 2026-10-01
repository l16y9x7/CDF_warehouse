# 简化 Agent 放置流程 - 移除位姿估计

## 问题重述

**现状**：Agent 侧在放置前先进行位姿估计，然后调用 manipulation.place()
**目标**：简化流程，直接放置，不进行位姿估计，适配 mui 当前的接口

## mui 底层接口（目标接口）

`/manipulation/place` 接收的参数（参考 agent_actions.py）：
```python
def place(self, payload):
    # 只需要 sku_typ，不需要 localization_result
    kind = payload['sku_typ']
    self.service.placement.execute({'sku_typ': kind}, preflight_all=True)
```

底层使用固定的胸部中线方法（`placement_reference`），不依赖视觉位姿估计。

## Agent 侧需要修改的文件

### 1. place_sku_in_basket.py 技能（主要修改）

**当前代码**：
```python
def execute(self, context: ExecutionContext, data: PlaceSkuInBasketInput):
    key = action_id(context, self.name)
    emit_started(context, self.name, action_id=key)
    try:
        # 调用位姿估计 - 需要移除
        result = infer_head_basket_pose(
            context, self.camera, self.estimation, self.pose, self.name
        )
        # 传递 localization_result - 需要移除
        self.manipulation.place(
            PlaceRequest(
                task_type=TaskType.SORTING,
                target_type=TargetType.SKU,
                destination_type=DestinationType.BASKET,
                hand=data.hand,
                sku_typ=data.sku_typ,
                localization_result=result.localization_result,  # 移除
            ),
            idempotency_key=key,
        )
```

**修改后**：
```python
def execute(self, context: ExecutionContext, data: PlaceSkuInBasketInput):
    key = action_id(context, self.name)
    emit_started(context, self.name, action_id=key)
    try:
        # 直接调用 place，不进行位姿估计
        self.manipulation.place(
            PlaceRequest(
                task_type=TaskType.SORTING,
                target_type=TargetType.SKU,
                destination_type=DestinationType.BASKET,
                hand=data.hand,
                sku_typ=data.sku_typ,
                # 不传递 localization_result
            ),
            idempotency_key=key,
        )
```

### 2. manipulation/contract.py（调整契约）

**当前代码**：
```python
@dataclass(frozen=True)
class PlaceRequest:
    task_type: TaskType
    target_type: TargetType
    destination_type: DestinationType
    hand: Hand
    pose: Pose6D | None = None
    sku_id: str | None = None
    sku_typ: str | None = None
    localization_result: Mapping[str, Any] | None = None  # 需要验证逻辑

    def __post_init__(self) -> None:
        if self.destination_type is DestinationType.BASKET:
            if (
                self.task_type is not TaskType.SORTING
                or self.target_type is not TargetType.SKU
                or not isinstance(self.sku_typ, str)
                or not self.sku_typ.strip()
                or not isinstance(self.localization_result, Mapping)  # 强制要求 - 需要移除
                or self.pose is not None
                or self.sku_id is not None
            ):
                raise ValueError("basket place requires SORTING sku_typ and basket localization")
```

**修改后**：
```python
def __post_init__(self) -> None:
    if self.destination_type is DestinationType.BASKET:
        if (
            self.task_type is not TaskType.SORTING
            or self.target_type is not TargetType.SKU
            or not isinstance(self.sku_typ, str)
            or not self.sku_typ.strip()
            or self.pose is not None
            or self.sku_id is not None
        ):
            raise ValueError("basket place requires SORTING sku_typ")
        # localization_result 变为可选，不强制验证
```

### 3. manipulation/http_adapter.py（调整 HTTP 调用）

**当前代码**：
```python
def place(self, request: PlaceRequest, *, idempotency_key: str | None = None) -> ActionResult:
    body = {
        "task_type": request.task_type.value,
        "target_type": request.target_type.value,
        "destination_type": request.destination_type.value,
        "hand": request.hand.value,
    }
    if request.destination_type.value == "basket":
        body["sku_typ"] = request.sku_typ
        body["localization_result"] = dict(request.localization_result or {})  # 需要调整
```

**修改后**：
```python
def place(self, request: PlaceRequest, *, idempotency_key: str | None = None) -> ActionResult:
    body = {
        "task_type": request.task_type.value,
        "target_type": request.target_type.value,
        "destination_type": request.destination_type.value,
        "hand": request.hand.value,
    }
    if request.destination_type.value == "basket":
        body["sku_typ"] = request.sku_typ
        # 只在提供时才传递 localization_result（向后兼容）
        if request.localization_result is not None:
            body["localization_result"] = dict(request.localization_result)
```

### 4. place_sku_in_basket.py（依赖调整）

**当前依赖**：
```python
def __init__(
    self,
    camera: CameraCapability,           # 不再需要
    estimation: EstimationCapability,   # 不再需要
    manipulation: ManipulationCapability,
    pose: BodyPoseCapability,           # 不再需要
):
```

**修改后**：
```python
def __init__(
    self,
    manipulation: ManipulationCapability,
):
    self.manipulation = manipulation
```

### 5. application.py（构建技能时的依赖注入）

需要找到构建 `place_sku_in_basket` 技能的地方，移除不需要的依赖注入。

## 可选：移除的文件和代码

如果 `basket_infer.py` 只被放置使用，可以考虑标记为废弃：
- `agent/src/agent/skills/basket_infer.py` - 可能不再需要（除非其他地方使用）

## 修改步骤

1. ✅ 修改 `manipulation/contract.py` - PlaceRequest 不强制要求 localization_result
2. ✅ 修改 `manipulation/http_adapter.py` - 只在提供时才传递 localization_result
3. ✅ 修改 `place_sku_in_basket.py` - 移除位姿估计调用和相关依赖
4. ✅ 修改 `application.py` 或技能构建逻辑 - 调整依赖注入
5. ✅ 运行测试验证

## 影响范围

### 不需要修改
- ✅ `workflows/sorting.py` - S-I08 步骤继续调用 place_sku_in_basket，接口不变
- ✅ mui 底层实现 - 保持不变

### 需要移除的能力依赖
- camera
- estimation  
- pose

### 保持的能力依赖
- manipulation

## 测试验证

需要验证：
1. 直接放置流程能正常工作
2. 不会因为缺少 localization_result 而报错
3. mui 底层能正确处理简化后的请求
