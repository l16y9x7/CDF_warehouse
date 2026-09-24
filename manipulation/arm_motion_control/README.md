# 右臂目标抓取：独立运动规划目录

本目录与 `rokae_web_control` 网页代码分开。默认流程只有读取、坐标转换和 MOVE L 候选规划；另附显式门控的 `execute_live.py`，**本次不运行，也不启动机器人**。先前 L2-A→预抓取的实际路径曾靠近躯干并触发急停，因此任何 `SAMPLED_IK_PASS` 都不能单独证明无碰撞。

## 坐标和抓取约定

- `chassis_link` 是世界系，X 前、Y 左、Z 上。4090 请求中的 `T_chassis_camera` 是拍摄瞬间的相机到世界变换，已包含当时躯干姿态。物品目标先固定在这个世界系中；躯干随后移动也不改变该目标。
- 新版网页的 `right_arm_sdk_world` 原点位于右肩，方向与胸部／世界轴大致一致。代码从四个实时躯干关节和 URDF 推出右肩原点；**不使用斜装物理手臂基座的旋转**。右臂实际回读和 URDF 模型位置需在 5 mm、姿态在 2° 内一致，否则只读规划立即报错。
- 4090 的 `reference_point_chassis_mm` 位于固定 Z=1200 mm 诊断平面，不能当抓取点。代码在点云上取距拟合轴线约 `body_radius_mm±6 mm` 的点，取轴向 99.5 百分位作为“可见顶端”，沿向上轴减去 20 mm。此启发式仍待实物验证。
- 正面抓取时法兰 +Z 指向世界 +X，法兰 +X 指向世界 +Z。抓取时沿法兰 +Z 的虚拟夹爪长度为 255 mm（实物约 205 mm 加虚拟 50 mm）；预抓取目标的法兰在同一条 +Z 轴线上再后退 100 mm。
- 肘点使用 URDF 右臂 J4 轴原点。右臂安全侧定义为 `elbow_world_y <= torso_plane_y_mm - elbow_plane_margin_mm`。用户可在 `config.json` 修改平面 Y；默认 0 mm，另留 30 mm 裕量。逐点和相邻关节样本中点均检查。

## 只读流程

从 `config.example.json` 复制一份独立的 `config.json`，根据现场设定保护平面和搜索范围，不要修改网页配置。

1. 在 L2 观察点取得同一次 4090 请求和响应 JSON（必须含点云 PLY、有效标志和 `T_chassis_camera`），先冻结目标：

   ```bash
   python3 freeze_target.py REQUEST.json RESPONSE.json frozen_target.json
   ```

2. 躯干由现场其他流程移动；本目录不会控制躯干。确认底盘、纸箱和物品未移动之后，只读回读当前躯干和右臂，重投影世界目标到**当前**右肩系，进行 MOVE L 候选采样：

   ```bash
   python3 plan_live.py frozen_target.json config.json --base-stationary-confirmed --output proposed_plan.json
   ```

臂角从当前值开始，依次尝试 +5°、−5°、+10°、−10°……。从当前位置到预抓取点，以及预抓取点到抓取点，两段都要连续通过逆解、关节软限位、每步关节变化、法兰正解误差和肘点平面检查。规划结果中的 `execution_allowed` 永远是 `false`。

## 显式门控的实机执行入口（本次不运行）

`execute_live.py` 具备按 5 mm 左右采样点逐个发送右臂 `MoveLCommand` 的能力，但仅在配置中主动设置 `execution_enabled: true`、命令行包含全部执行标志、现场操作员输入确认短语、拍摄时间小于 10 分钟、网页硬件服务已停止、机器人状态与规划起点一致时才会发送。速度上限为 5 mm/s；每点等待到位并回读关节、躯干与肘点，异常时尝试 `stop()`。本次不会设置启用标志，也不会调用此脚本。

仍需针对当前工具 TCP、夹爪张开形状、整条手臂和躯干、箱壁和货架进行现场检查；确认 4090 顶端提取、控制器 MOVE L 插补与采样一致，并清除急停后的旧运动队列。肘点平面只能过滤一类躯干风险，不能证明其它部位无碰撞。网页 Pose 当前使用 MoveJ，不要用它替代这里的 MOVE L。

## 本地测试

```bash
python3 test_motion.py
```

测试覆盖 4090 有效门控、世界目标、255/355 mm 虚拟夹爪、右肩系与现场新版网页回读的毫米级核对、以及肘点平面和 5° 臂角搜索；不连接机器人。
