# 独立躯干保护规划模块

本模块是待接入的规划组件，不会自动应用到网页、记忆点或 `execute_live.py`。导入模块不会连接机器人；规划函数不会上电、切模式或下发运动。用户确认使用 URDF `Chest_link` 胸部坐标系。

## 平面和距离

设偏移距离为 `d = offset_mm >= 0`。

- 右臂几何平面：`y_chest = -d`，安全侧为 Y−。
- 左臂镜像平面：`y_chest = +d`，安全侧为 Y+；规划器可接受左右臂运动学回调，目前附带的 SDK 适配器针对右臂。
- 世界坐标下的肘点先通过 `T_chassis_Chest_link` 的逆变换回到胸部系，再做判断，不能直接比较 `elbow_world_y`。
- 右臂实际判定：`elbow_chest_y <= -(offset_mm + elbow_radius_mm + margin_mm)`。

`offset_mm` 必须由调用者指定，没有写入任何现场配置。值越大，平面越往外，限制越严格。`margin_mm` 默认 10 mm，用于提前留出距离；`elbow_radius_mm` 默认 0 mm，对应用户提出的肘点模型，可设置为经测量确认的肘部包络半径。肘点使用既有 URDF 模型的 J4 轴原点。

`PlaneMeasurement.clearance_mm` 是扣除半径和余量后的净距离：大于等于 0 通过，小于 0 拒绝。平面边界本身不会包围全部躯干；这个约束只用于肘部侧向侵入过滤。

## 搜索行为

`plan_guarded_movel` 从实际当前臂角 `a` 出发，按 `a, a-5, a+5, a-10, a+10, ...` 依次尝试。默认最大变化 ±90°，不把角度跨 ±180° 包回另一端。找到第一个整条路径通过的候选即返回，不等其余候选。

每个候选从同一个实际起点重新规划。第一段运动中，臂角从实际值逐渐变到候选值；后续段保持这个臂角。调整臂角的过渡段也必须通过保护检查。如果需要先在末端位姿保持不变的情况下调整臂角，可将当前法兰位姿作为第一个目标段，仍会检查这段自运动。

检查内容：

- 当前起点、全部目标段及中间采样的肘点保护余量；不是只看目标位姿。
- 逆解、关节限位、相邻关节变化和 FK 位姿一致性。
- 默认笛卡尔位置间隔不超过 2 mm、姿态和臂角间隔不超过 1°。
- IK 样本之间额外按不大于 0.5° 的最大单轴间隔检查关节插值，并至少检查中点，以发现点间凸出。

默认每次相邻 IK 关节变化最多 5°，超出视为构型不连续。默认计算上限为 60 秒、20,000 次 IK、100,000 次 FK，可配置。取消、超时、计算上限、全部候选失败时不会返回可执行的部分路径。

当前肘点如果已经进入保护余量区，返回 `start_inside_protection`，不自动寻找脱离运动。应先调整平面或另行规划脱离路径。通信、数据损坏或其他模型异常中止规划，不伪装成逆解无解继续尝试。

## API

纯算法入口在 `torso_guard.py`。任何运动学实现只要提供以下接口即可：

```python
ik(flange_world_mm, arm_angle_deg, seed_deg) -> 七个关节角（度）或 None
fk(joints_deg) -> (4x4 法兰位姿矩阵，3维 J4 肘点)
within_limits(joints_deg) -> bool
```

IK 的 `None` 仅表示运动学不可达，通信等异常应抛出。所有位姿和肘点必须属于同一个外部坐标系，通常为 `chassis_link`；长度单位 mm，关节和臂角单位度。躯干和工具配置必须是规划时冻结的同一份状态。

已有 `RightArmReadOnly` 对象可包裹为 `torso_guard_adapter.RightArmGuardKinematics`。这个适配器复用对象已经持有的 SDK 连接和 FK，单独区分已知 IK 错误与通信错误，不沿用旧 `ik()` 对所有 RuntimeError 都返回 None 的行为。构造适配器本身不会建立连接。

```python
from torso_guard import TorsoPlane, SearchOptions, plan_guarded_movel, start_matches
from torso_guard_adapter import RightArmGuardKinematics

# readonly_arm 是调用方在明确读取硬件的流程里取得的静止状态快照。
kin = RightArmGuardKinematics(readonly_arm)
plane = TorsoPlane(
    side="right",
    offset_mm=chosen_offset_mm,   # 需要现场逐步调整的距离
    margin_mm=10.0,
    elbow_radius_mm=0.0,
)
result = plan_guarded_movel(
    kin,
    readonly_arm.current_joints_deg,
    readonly_arm.arm_angle_deg,
    [target_flange_chassis_mm],   # 也支持多个 4x4 目标组成的路径
    world_from_torso_mm=kin.world_from_torso_mm,
    plane=plane,
    options=SearchOptions(max_arm_angle_change_deg=90),
)
# result.plan 为 None 时不可发送运动。
# 有结果时，调用方可立即取得所选臂角、完整采样路径、最小净余量和尝试原因。
# 本示例故意没有运动下发步骤；具体接入位置待用户指定。
```

返回 `PlanningResult` 的 `attempts` 记录每个已尝试臂角和失败位置/原因；`plan.selected_arm_angle_deg` 是最终选择；`plan.waypoints` 包含各段法兰目标、七轴关节、臂角、肘点和净余量。结果用不可变元组保存，输入数组后续变化不会修改已返回的方案。

后续执行器可用 `start_matches()` 在下发前复核当前关节和胸部变换是否仍与规划起点一致；用 `inspect_runtime_sample()` 根据回读判断是否需要停止。躯干变化或肘点进入保护余量区时，返回 `stop_required=True`。执行器应先停止并确认静止，再以新状态重新规划；不要在旧指令仍运行时直接叠加新运动。上述函数本身不发送停止或运动指令。

## 接入边界

本次仅新增组件和测试，没有改变旧规划器的扫描顺序、旧平面定义或任何现有运动入口。新组件的轨迹生成针对 MoveL 候选；MoveJ 的关节路径不能套用 MoveL 的检查结论。后续若入口会回退 MoveJ，必须对实际关节运动路径另做保护检查。

采样验证不是连续碰撞证明。点间关节插值检查也不等同于 SDK 的真实 MoveL 插补。接入时需要保留实际采样段、核对 SDK 构型/臂角插补、执行前原生 `checkPath` 及状态有效性，并把运行时判断接入停止机制。当前实现不宣称已覆盖前臂、夹爪、另一只手臂或环境碰撞。

## 离线测试

在本目录运行：

```bash
python3 -m unittest test_torso_guard -v
```

测试无 SDK 连接或实机动作。远端存在整机 URDF 时，额外使用真实 J4 运动学验证胸部系余量在躯干移动后的不变性；本地缺少该 URDF 时明确跳过该项。
