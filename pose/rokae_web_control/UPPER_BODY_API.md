# 上身查询接口与抓取坐标系修正

2026-09-23更新：最新接口结构见`MANIPULATOR_STATE_API.md`，启停命令见`SERVICE_LIFECYCLE.md`。新上报与旧8092格式共用常驻SDK连接和同一份采样缓存；新协议目标10Hz/500ms过期，旧格式维持3秒过期规则。下文保留早期实现与历史验证记录，早期1Hz采样、直接启动server.py等说明不再代表最新版部署方式。新采集器需经用户确认后重启一次SDK服务才加载；日常只重启mui.target。

## 查询接口

- 全部：`GET http://192.168.130.105:8092/api/telemetry/upper-body`
- 左臂：`GET /api/telemetry/upper-body/left_arm`
- 右臂：`GET /api/telemetry/upper-body/right_arm`
- 躯干：`GET /api/telemetry/upper-body/trunk`
- 同机有线地址 `192.168.71.51` 可使用相同端口和路径。

请求不需要网页解锁，不发送运动或模式切换指令。由独立的 SDK 常驻服务持有三套 SDK 对象，后台默认每秒采集一次；外部请求只读取缓存，不按客户端数量增加 SDK 轮询。SDK 锁被运控占用时跳过当轮采集。网页进程不直接连接控制器；抓取规划和运控业务逻辑仍留在网页进程，通过本机 Unix socket 调用常驻服务。

每个模块包含：

| 字段 | 含义 |
|---|---|
| `joint_positions_deg` | 左右臂各 7 个关节、躯干 4 个关节，单位度；躯干不包含头部外部轴 |
| `end_pose.position_mm` | 法兰相对该控制器基座的位置，单位毫米 |
| `end_pose.rpy_deg` | 法兰相对该控制器基座的 RPY，单位度，Rz·Ry·Rx |
| `end_pose.coordinate_type` | 固定为 SDK `flangeInBase`，遵循指标表；与网页 TCP/肩部系位姿有区别 |
| `end_pose.frame` | `left_arm_controller_base`、`right_arm_controller_base`、`trunk_controller_base` |
| `state` | SDK `operationState` 的枚举名称，例如 `idle`、`moving` |
| `valid`、`stale`、`error` | 数据有效性、过期标志及读取错误 |
| `sampled_at`、`age_ms`、`sequence` | UTC 采样时间、含读取耗时的数据年龄、该模块样本序号 |

返回结构：`{"ok":true,"data":{"schema_version":1,"mode":"hardware","all_valid":true,"modules":{"left_arm":{...},"right_arm":{...},"trunk":{...}}}}`。

查询涉及的模块全部有效时 HTTP 200；任一个无效时 HTTP 503，同时保留其他有效模块的数据。未取得、读取失败或超过 3 秒的模块，三项数据为 `null`，不会补零或把旧值标成当前值。三个模块依次采样，不保证同一硬件时刻，按各自时间戳使用。

```python
import requests
response = requests.get(
    "http://192.168.130.105:8092/api/telemetry/upper-body", timeout=2
)
payload = response.json()  # 503 也有各模块的有效性和错误说明
right = payload["data"]["modules"]["right_arm"]
if right["valid"]:
    print(right["joint_positions_deg"], right["end_pose"], right["state"])
```

采样配置位于 `/home/admin/mui/rokae_web_control/config.json` 的 `telemetry.poll_interval_seconds=1.0`、`telemetry.stale_after_seconds=3.0`，修改后重启服务。不要让上报程序另建 SDK 连接；查询客户端退出不会触发 SDK 断连。

## 开机服务与网页独立生命周期

**常驻服务是 `mui-sdk.service`，不启动网页。** admin 的 linger 已开启，用户未登录时也能随系统启动。启动只自动读取状态，不自动解锁、上电、开合夹爪或执行运动。

| 进程 | 生命周期和职责 |
|---|---|
| `hardware_service.py` / `mui-sdk.service` | 开机运行，持有 SDK 连接、采集数据，8092 只读 HTTP 接口 |
| `server.py` / `start_hardware.sh` | 用户按原方式启动/关闭，8091 网页；保留抓取规划、业务运控、记忆点、日志、相机及底盘逻辑 |

```bash
# 查看独立服务
systemctl --user status mui-sdk.service
journalctl --user -u mui-sdk.service -n 80 --no-pager
# 日常开发仍按原方式运行；Ctrl-C 关闭网页不停止 SDK 服务
cd /home/admin/mui/rokae_web_control
./start_hardware.sh
```

服务文件在项目 `system/mui-sdk.service`。SDK 服务不管理其他项目的 ROS 服务；网页原来的 ROS 使用方式保留。`start_hardware.sh` 只确保 SDK 服务已经运行，再启动它自己的网页子进程。

网页通过权限 0600 的 `.run/sdk.sock` 调用有限的 SDK 方法。一个网页进程持有运控会话，可同时服务多个浏览器标签。网页退出后清理该会话的未完成动作并释放会话，不断开控制器连接；只读采集和 8092 接口继续运行。SDK 请求在通信中断时不会自动重发。

初次部署时，若旧版网页仍直接连接控制器，常驻服务会等待并返回明确的无效数据状态，不抢占连接、不停止旧网页。旧网页退出后，常驻服务接管 SDK；以后网页启停只影响自己的会话。

网页和抓取算法开发可继续反复启停 `server.py`。只有修改常驻服务自身的 SDK 通信层或其配置时才需要重启 `mui-sdk.service`。其他人仍直接使用 SDK 的独立程序不受本项目的会话锁约束，本次不修改这些私人项目。

## 请求示例之外的配置

`config.json` 的 `hardware_service` 指定 HTTP 地址、端口与 Unix socket；`telemetry` 指定采样周期和过期时间。公开端口只接受读取，POST/PUT/PATCH/DELETE 返回 405。SDK 服务日志位于 `logs/hardware/`，网页全过程日志仍在原 `logs/`。

## 抓取坐标系

用户明确指定：**“躯干”是躯干 SDK 使用的参考系 `trunk_controller_ref`（`endInRef`）；“胸部”是 URDF `Chest_link`。**

抓取版本 `grasp-test-v8`。D/d、预抓取/抓取夹爪偏移、赋予的 RPY [180,-90,0]°、两次抬升及两次后退都按躯干 SDK 参考系计算，再转换到右肩 SDK 世界系执行。前挡板过滤的前向轴来源改为 `trunk_sdk_x`。肘部保护平面仍按此前明确指定的 `Chest_link`，130/65/10 mm 配置不变。

坐标桥使用同次回读的躯干关节、SDK TCP 位姿和真实工具偏移，结合此机器人 URDF 中 PCB4 输出法兰 `Chest_link`，计算 `T_chassis_ref = T_chassis_flange × T_flange_tcp × inverse(T_ref_tcp)`；不把移动中的胸部原点当作 SDK 参考系原点。缺少回读或工具/参考系改变时拒绝转换。

流程仍为：预抓取（d+230）→抓取（170）→可用且已解锁时合爪→躯干 SDK Z+40→右臂沿躯干 SDK X−(d+70)→躯干 SDK Z+75→上身沿躯干 SDK X−100→结束。后四段手臂仍逐段回读新臂角；六段运动保持网页保存的平移和旋转速度；不增加躯干 IK/FK 预检或返回动作。

## 验证与部署状态

隔离环境完整离线回归 187 项通过；包含胸部/参考系/工具均旋转时的方向和姿态、直接在躯干 SDK X 上后退、缓存 HTTP、掉线/过期/互斥、重复启动和端口冲突。JS 语法检查通过；18 个真实 SDK 数据对象的跨进程编码往返核对通过。独立进程 MOCK 实测：常驻进程不变、两个网页客户端进程先后退出后，采样序号仍从 4 增长至 7。测试未连接机器人或执行实机运动。

2026-09-18 20:14 已安装并启用独立的 `mui-sdk.service`，备份为 `/home/admin/mui/rokae_web_control/.backup-telemetry-trunk-20260918-201410`。安装前后的原文件/新文件哈希核对通过；正式目录 18 项专项离线测试通过。

用户自行关闭旧网页后，独立服务完成了连接接管。2026-09-18 20:24–20:25 已完成真实只读生命周期验证：启动新版网页→回读→退出本次创建的网页→再次启动→回读。全过程常驻 PID 387595 和 6 条 SDK TCP 连接的端点、socket inode 均未改变；无网页期间三路查询仍为 HTTP 200、全部有效。右臂样本序号依次为 467→552→554→556，另两路同样持续增加。

新版网页现运行于 8091（此次验证结束时启动器 PID 394153、网页 PID 394181），版本 `grasp-test-v8`，保持未解锁。两次网页回读均成功，包括左右臂各 7 轴、躯干 4 轴、头部 2 轴、位姿、臂角及躯干工具信息；头部相机订阅、底盘状态和电池回读正常。实际执行运动仍待用户实机测试，本次未通过运动验证该转发层。原始验证记录见同目录 `independent-sdk-live-verification-20260918.json`。

已核验开机启用配置与 linger；没有为验证而重启机器人。后续按原方式关闭/启动网页即可，独立服务无需跟随网页重启；如需关闭本次后台启动的网页，可只向网页 PID 394181 发送 SIGTERM（先重新核对 PID 对应进程），不要停止 `mui-sdk.service`。

从用户 Windows 电脑访问 192.168.130.105，合并查询、三条单模块查询及网页状态均为 HTTP 200，确认外部客户端可直接调用。

使用三个已存档记忆点进行了离线坐标核对，推导的躯干 SDK 参考系原点距 chassis_link 原点约 0.04 mm、旋转差约 0.006°，与模型舍入量级一致；该核对不是新的实机标定。

本次没有执行实机运动、发送新的视觉估计请求、修改其他私人项目或登录远端 Codex。
