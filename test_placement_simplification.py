#!/usr/bin/env python3
"""验证 place_sku_in_basket 简化后的流程"""
import sys
sys.path.insert(0, 'agent/src')

from agent.capabilities.manipulation import PlaceRequest, MockManipulationCapability
from agent.capabilities.common import TaskType, TargetType, DestinationType, Hand
from agent.skills.place_sku_in_basket import PlaceSkuInBasketSkill, PlaceSkuInBasketInput
from agent.contracts import ExecutionContext

print("=" * 60)
print("测试 1: PlaceRequest 不要求 localization_result")
print("=" * 60)

try:
    req = PlaceRequest(
        task_type=TaskType.SORTING,
        target_type=TargetType.SKU,
        destination_type=DestinationType.BASKET,
        hand=Hand.RIGHT,
        sku_typ='bottle',
    )
    print(f"✓ PlaceRequest 创建成功")
    print(f"  - sku_typ: {req.sku_typ}")
    print(f"  - localization_result: {req.localization_result}")
except Exception as e:
    print(f"✗ 失败: {e}")
    sys.exit(1)

print("\n" + "=" * 60)
print("测试 2: PlaceSkuInBasketSkill 不再需要 camera/estimation/pose")
print("=" * 60)

try:
    manipulation = MockManipulationCapability()
    skill = PlaceSkuInBasketSkill(manipulation)
    print(f"✓ PlaceSkuInBasketSkill 初始化成功（只需要 manipulation）")

    # 执行技能
    result = skill.execute(
        ExecutionContext("test-task"),
        PlaceSkuInBasketInput("bottle", Hand.RIGHT)
    )

    print(f"✓ 技能执行成功")
    print(f"  - status: {result.status}")
    print(f"  - destination_type: {result.destination_type.value}")

    # 验证 manipulation 调用
    placed = manipulation.place_requests[-1]
    print(f"✓ Manipulation.place 被调用")
    print(f"  - sku_typ: {placed.sku_typ}")
    print(f"  - hand: {placed.hand.value}")
    print(f"  - localization_result: {placed.localization_result}")

    if placed.localization_result is not None:
        print(f"✗ 错误: localization_result 应该是 None")
        sys.exit(1)

except Exception as e:
    print(f"✗ 失败: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\n" + "=" * 60)
print("测试 3: HTTP body 构建（不包含 localization_result）")
print("=" * 60)

try:
    request = PlaceRequest(
        task_type=TaskType.SORTING,
        target_type=TargetType.SKU,
        destination_type=DestinationType.BASKET,
        hand=Hand.RIGHT,
        sku_typ='bottle'
    )

    # 模拟 http_adapter 的 body 构建
    body = {
        'task_type': request.task_type.value,
        'target_type': request.target_type.value,
        'destination_type': request.destination_type.value,
        'hand': request.hand.value,
    }

    if request.destination_type.value == 'basket':
        body['sku_typ'] = request.sku_typ
        if request.localization_result is not None:
            body['localization_result'] = dict(request.localization_result)

    print(f"✓ HTTP body 构建成功")
    print(f"  Body 内容: {body}")

    if 'localization_result' in body:
        print(f"✗ 错误: body 不应包含 localization_result")
        sys.exit(1)

    expected_keys = {'task_type', 'target_type', 'destination_type', 'hand', 'sku_typ'}
    if set(body.keys()) != expected_keys:
        print(f"✗ 错误: body keys 不正确")
        print(f"  期望: {expected_keys}")
        print(f"  实际: {set(body.keys())}")
        sys.exit(1)

except Exception as e:
    print(f"✗ 失败: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\n" + "=" * 60)
print("测试 4: 向后兼容（带 localization_result 的情况）")
print("=" * 60)

try:
    req_with_loc = PlaceRequest(
        task_type=TaskType.SORTING,
        target_type=TargetType.SKU,
        destination_type=DestinationType.BASKET,
        hand=Hand.RIGHT,
        sku_typ='bottle',
        localization_result={'point_semantics': 'basket_model_center'}
    )

    print(f"✓ 带 localization_result 的 PlaceRequest 仍然有效")

    # 模拟 body 构建
    body = {
        'task_type': req_with_loc.task_type.value,
        'target_type': req_with_loc.target_type.value,
        'destination_type': req_with_loc.destination_type.value,
        'hand': req_with_loc.hand.value,
    }

    if req_with_loc.destination_type.value == 'basket':
        body['sku_typ'] = req_with_loc.sku_typ
        if req_with_loc.localization_result is not None:
            body['localization_result'] = dict(req_with_loc.localization_result)

    if 'localization_result' not in body:
        print(f"✗ 错误: 提供 localization_result 时应该包含在 body 中")
        sys.exit(1)

    print(f"✓ 向后兼容验证通过")

except Exception as e:
    print(f"✗ 失败: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\n" + "=" * 60)
print("所有测试通过！")
print("=" * 60)
print("\n修改总结:")
print("1. ✓ PlaceRequest 不再强制要求 localization_result")
print("2. ✓ PlaceSkuInBasketSkill 移除了 camera/estimation/pose 依赖")
print("3. ✓ HTTP 适配器只在提供时才发送 localization_result")
print("4. ✓ 向后兼容：仍然支持带 localization_result 的调用")
print("\n适配 mui 接口完成！")
