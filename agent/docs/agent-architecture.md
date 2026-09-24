# 通用 Agent 架构设计

## 目标

Agent 负责将业务任务可靠地转换为机器人动作。架构应支持不同硬件模块、可复用的机器人技能，以及可恢复的业务流程。大模型（如未来引入）只能提供候选决策，最终动作必须经过技能契约和 Workflow 状态机校验。

## 分层

```text
Workflow
  ↓
Skill
  ↓
Capability Contract
  ↓
Capability Adapter
  ↓
External Module
```

### 1. Capability 模块能力层

Capability 是对外部模块的稳定端口，例如 `Navigation`、`Perception`、`Pose`、`Estimation`、`Manipulation`、`VLA`、`Hand` 和 `Camera`。各能力模块彼此独立，分别拥有目录、端口、健康检查和 Adapter。Adapter 负责 HTTP/JSON、超时、健康检查和错误转换；上层不感知 URL、端口或具体传输方式。Capability 不包含订单或拣选业务逻辑。

当前业务链路只接入标准抓取。`VLA` 和 `Hand` 保留为独立 Capability 供后续联调，暂不注册对应 Skill、不进入 Workflow，也不参加任务前健康检查。

每个 Capability 独立成包，代码边界为：

```text
capabilities/navigation/
capabilities/pose/
capabilities/perception/
capabilities/camera/
capabilities/estimation/
capabilities/manipulation/
capabilities/vla/
capabilities/hand/
```

每个目录都包含 `contract.py`（接口与请求/响应数据结构）、`http_adapter.py`（HTTP 实现）和 `mock.py`（测试实现）。需要组合多个动作时，由 Skill 负责编排。

### 2. Skill 技能层

Skill 是具有明确输入、输出、前置条件和失败语义的短流程。它可以调用多个 Capability，例如 `PickSku` 可以依次执行姿态准备、图像定位、位姿估计、抓取和结果确认。Skill 不直接访问 Workflow 内部状态，也不依赖某个具体业务流程。

### 3. Workflow 工作流层

Workflow 面向完整业务目标，负责状态、节点顺序、条件分支、循环、取消、任务级超时和恢复。例如 `SortingWorkflow` 和 `ReviewWorkflow` 都通过 Skill 完成具体动作。

Workflow 的具体状态模型、Sorting/Review 流程和能力接口契约统一以根目录的 `主方案文档.md` 为准，避免多份流程文档产生冲突。

## Runtime 与可靠性

Runtime 统一执行 Workflow/Skill，并持久化状态快照、节点结果和事件。所有物理动作都遵循：

```text
以 `task_id` 关联任务和节点记录 → 持久化执行中 → 调用 Capability
→ 保存结果 → 更新 Workflow 状态 → 进入下一节点
```

物理动作结果未知时，Workflow 立即进入 `WAITING_CONFIRMATION`，不得自动重复物理动作；恢复和人工确认机制由具体实现定义。进程重启时，尚未结束的任务也封存为 `WAITING_CONFIRMATION`。日志和事件至少关联 `task_id`、节点/Skill 标识和错误码。

## 目录与依赖规则

`src/agent/capabilities` 只定义设备端口和适配器，`skills` 只组合能力，`workflows` 只编排技能，`runtime` 提供通用执行服务。依赖只能向下，禁止 Workflow 直接调用 HTTP，禁止 Capability 依赖 Skill 或 Workflow。外部模块替换时只修改对应 Adapter。

## 演进建议

先用进程内 Runtime 和内存状态实现闭环，再接入 PostgreSQL 等持久化存储。每新增设备先实现 Capability Adapter，每新增动作组合先实现 Skill，只有出现新的业务目标时才新增 Workflow。
