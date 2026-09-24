# 珞石轮臂机器人 Web 控制台（第一阶段）

当前服务管理以 [SERVICE_LIFECYCLE.md](SERVICE_LIFECYCLE.md) 为准：`mui.target`只管理网页8091和接口8082/8086，常驻SDK/上报、独立底盘和相机不跟随重启。新版状态协议和Python缓存入口见 [MANIPULATOR_STATE_API.md](MANIPULATOR_STATE_API.md)。首次加载上报升级需要单独确认一次SDK服务重启。

这个项目只借鉴天机 `perception` 页面的功能组织，不复用天机硬件实现。当前版本包含：

- 左臂 7 轴、右臂 7 轴、躯干 4 轴、头部外部轴 2 轴的角度回读与控制。
- 左臂、右臂、躯干 Pose 回读与控制；双臂臂角可回读和设定，页面使用 mm/°，SDK 使用 m/rad。
- 左右臂末端按钮门控的笛卡尔自由拖拽开关。
- 默认运动速度设置。
- 右臂 Robotiq 2F-85 夹爪：跟随顶部总控制启用，保留初始化、全开、全闭和 0–255 位置滑块。滑块松开后才发送位置指令，0 为全开，数值越大闭合越多。
- 珞石随车 AMR ROS2 包的底盘遥控：全向平移、旋转、按住运动、松手停止、服务端 0.6 秒租约。
- 头部 Orbbec Gemini 335L 的 1280×720@15 RGB，以及对齐到 RGB 的深度流。
- 头部 RGB-D 改为订阅现有开机服务发布的 1280×720@15 ROS2 同步话题，深度已对齐 RGB；网页不再通过相机 SDK 占用 USB。
- 网页只提供现有脚本的“异常恢复检查”，不负责摄像头启停。不连续传输视频，仅在点击“获取当前帧”时获取并显示一张 RGB；无有效数据时画面纯黑。目前外部 ROS2 服务仅启用头部相机，腕部历史 SDK 文件保留，但硬件入口不再使用。
- 每个画面有独立记录按钮，只保存该摄像头当前 RGB、对齐深度，以及一份上身全部关节角、Pose、双臂臂角和运行状态。
- “被抓取物品的位姿估计”使用拍照时的上身关节角和相机外参，把 4090 轴线及点云的可见顶端转换到 `chassis_link` 世界系；沿朝上轴线反向 20 mm 得到抓取点，世界抓取朝向固定为 Rx=180°、Ry=-90°、Rz=0°。4090 的固定高度参考点只用于校验，不作为顶端或抓取点。
- “读取当前躯干并转换到右肩坐标系”从已保存的最近一次有效世界抓取结果出发，重新回读躯干 J1–J4，以 URDF 计算当前右肩 SDK 世界系原点和朝向，显示右肩系抓取 6D Pose。两按钮均不会发送运动指令；旧照片对应的底盘或物品一旦移动，应重新拍照估计。

## 安全设计

- 网页双臂 Pose 的位置和姿态统一采用各自控制器的 **SDK 世界坐标系**：原点分别为左肩、右肩，朝向为已配置的 Z 向上世界系；不是斜装的手臂基坐标系，也不是胸部中间公共工件系。左右臂世界系朝向一致但原点不同，不能直接互用 XYZ。
- 当前 AR SDK 仅提供 `flangeInBase` 和 `endInRef`。网页回读通过 `T_world_tcp = T_world_ref @ T_ref_tcp` 换算，执行前用逆变换把世界系目标转回当前工件系，并对该目标做 IK 预检和 MoveJ。变换每次读取 SDK `toolset.ref`，不依赖写死的胸部偏移，不修改已标定工件、TCP、负载或安装参数；躯干 Pose 约定不变。
- 回读、记录和状态接口包含 `pose_frames`。双臂 `/api/pose/<module>` 请求必须携带匹配的 `frame`（`left_arm_sdk_world` / `right_arm_sdk_world`），防止旧网页的胸部系数值被误当作新的肩部世界系目标。更新后需要重启控制服务、刷新网页并重新回读。

- 不带 `--hardware` 参数时严格使用 Mock，不导入 xCore SDK、不连接任何机器人 IP、不发布 ROS2 速度。
- 即使使用 `--hardware` 启动，页面控制也默认锁定；点击“解锁控制”后保持解锁，直到手动锁定或服务重启。
- 解锁无需输入确认文字；关节、Pose、拖拽和底盘启用按钮直接触发，不再逐次弹出确认框。
- 夹爪随顶部总控制解锁和锁定，网页不再有单独的夹爪锁定/解锁按钮。解锁总控制不会自动初始化或开合；仍需手动初始化夹爪后才能开合。服务重启后总控制和夹爪控制均保持锁定。
- 夹爪位置指令固定采用已在 105 上小幅测试的低速 `20/255`、低力 `10/255`；初始化可能让手指走完整行程。网页会回读实际位置和故障码。全闭 `255` 的实物空载测试尚未做过。
- 任一机械臂处于拖拽状态时，禁止所有关节和 Pose 运动。
- 手臂、躯干、头部和 Pose 均不设置网页端单次步长限制，由控制器软限位保护。网页在每次服务启动后首次读取一次控制器软限位并缓存。
- 双臂 Pose 运动会使用页面设定的臂角、保留当前构型，并在下发前通过机器人模型做逆解预检，避免目标把关节推出软限位。
- 底盘采用“按住运动”，页面释放、失焦、隐藏或关闭时发送零速度，后端超时也会补发零速度。

## Mock 检查

```bash
cp config.example.json config.json
./start_mock.sh
```

浏览器打开启动输出中的“访问链接”。本机配置默认为 `http://192.168.130.105:8091/`。Mock 模式可以完整检查页面、校验和操作流，不会连接机器人。

## 硬件测试前准备

1. 确认 `/home/admin/mui/xCoreSDK-Python-AR-v0.7.1.ar_4/rokae_xcore/` 中存在匹配当前 Python 版本的 aarch64 `.so`，并包含 `ArRobot` 接口。
2. 启动统一硬件控制服务：

   ```bash
   ./start_hardware.sh
   ```

3. 浏览器打开 `http://192.168.130.105:8091/`，先回读；确认数值、方向和坐标系正确后点击解锁并小步测试。

摄像头网页入口只需要 ROS2 的 `rclpy` / `sensor_msgs` 和 OpenCV / NumPy，以及已运行的外部摄像头 ROS2 服务，不需要导入 Orbbec 或 RealSense Python SDK。USB 权限、相机驱动和开机服务由 `/home/admin/vision` 原项目负责，网页不修改该项目。

`./start_hardware.sh start|stop|restart|status` 对应 `systemctl --user` 管理 `mui.target`，统一管理运控核心和网页，SDK 上报及相机独立运行。后端代码修改在下一次 `systemctl --user restart mui.target` 后生效。

底盘是可选依赖：`start_control_runtime.sh` 检测到已有底盘节点时直接复用；没有时在独立进程组中后台启动。上半身不等待底盘发现/就绪，底盘启动失败或掉线不会让运控核心退出，网页底盘区域显示错误，恢复后自动清除。退出只清理由本次脚本启动的底盘进程组，TERM 后最多等待约 3 秒再清理残留，不触碰外部底盘节点；运控核心自身仍走原有机器人停止和 SDK 会话关闭流程。

网页启动通过本地 `/api/ready` 检查核心已完成初始化并开始服务，不再调用完整 `/api/status` 读取设备状态。该检查不代表所有外设健康；实时设备状态仍由网页正常状态接口显示。核心异常退出码会传给 systemd，使原有自动重试生效。`start_chassis_node.sh` 可用于单独诊断底盘。

底盘返回 `70002` 表示急停仍然生效。先物理释放急停，再通过 `/sr_amr_control/release_emergency_stop` 解除底盘急停，确认 `/sr_amr_control/system_state` 中 `estop_active: false` 后才能启用遥控。

网页已经提供“解除底盘急停”按钮。仅当物理急停已经松开，但启用底盘遥控仍提示 `70002` 时使用；先解锁网页控制，再点击该按钮，成功后重新启用底盘遥控。正常情况下不需要点击，也不能用它代替释放物理急停。

遥控区域提供“手动遥控避障”开关，使用厂家 ROS2 `/sr_amr_control/remote_control_oba_enabled`（`std_srvs/SetBool`）。它控制手动遥控整体避障；不修改自动导航或 `nav.stop_distance`，也不改变急停、速度限制和网页指令租约。切换需解锁网页控制，且不能与上身运动作业并行。

网页显示 `/sr_amr_control/system_state` 的 `remote_control_oba_active` 实时状态。超过 3 秒未更新或通信失败显示未知，并禁用切换；只有厂家服务确认目标状态后才显示切换成功。既有厂家服务在启用/禁用遥控模式时自动恢复避障，网页通过回读同步。启动网页不会自动开关避障，切换失败不自动重试。

网页内部接口为 `POST /api/chassis/obstacle-avoidance`，JSON 必须为 `{"enabled": true}` 或 `{"enabled": false}`。响应及 `/api/status` 的 `chassis.obstacle_avoidance` 包含 `enabled`（未知为 null）、`service_ready`、`stale`、`age_seconds`、`source`、`error`。请求、成功、失败均写入控制日志。安装后下次重启 `mui.target` 并刷新网页生效；无需重启 SDK 上报或相机服务。

## 远程连接不因空闲自动断开

首次执行一次：

```bash
sudo ./configure_remote_no_timeout.sh
```

脚本为 SSH 启用 30 秒服务端心跳并关闭空闲探测断开，同时持久关闭当前 Wi-Fi 连接的省电模式并屏蔽系统休眠入口。它只重载 SSH 配置，不会重启网络或中断当前会话。物理掉电、路由器重启或 Wi-Fi 信号丢失仍会导致网络连接中断。

## 躯干 J2 越过软限位后的恢复

先停止硬件 Web 服务，再执行只读检查：

```bash
./recover_trunk_j2.sh status
```

确认现场安全后执行恢复：

```bash
./recover_trunk_j2.sh recover
```

恢复脚本会备份原软限位，临时关闭躯干软限位，以 5% 关节速度只将 J2 退到 45°，然后恢复并回读确认原软限位。脚本只接受不超过原软上限 10° 的小范围越界，并拒绝与硬件 Web 服务并发运行。

## 运动日志

服务将所有控制请求、接受或失败结果、关节/Pose 目标、拖拽状态、速度设置和底盘速度写入：

```text
/home/admin/mui/rokae_web_control/logs/motion-YYYY-MM-DD.jsonl
```

关节或 Pose 指令执行后，以及拖拽开启期间，服务会自动采样全身关节角、Pose 和控制器运行状态；拖拽关闭后再补录末尾状态。每行都是带时间、会话 ID 和顺序号的独立 JSON，可直接查看：

```bash
tail -f logs/motion-$(date +%F).jsonl
```

## 摄像头数据记录

硬件网页只订阅现有摄像头 ROS2 服务，不再打开 USB、启动或关闭摄像头驱动。相机由 `/home/admin/vision` 中已有的开机服务管理；该目录中的代码和配置保持不变。当前仅头部服务启用，1280×720、15 fps。

订阅配置位于 `ros_camera`：启动时在子进程只读 source `/home/admin/vision/config/ros/setup.bash` 解析摄像头 Domain（当前 51），创建独立 ROS Context，不改变底盘使用的 ROS Domain。订阅 `/camera/head/synced/color/image_raw`、`/camera/head/synced/depth/image_raw`、`/camera/head/synced/color/camera_info`；BEST_EFFORT / VOLATILE QoS 兼容现有 RELIABLE 发布端。三路必须有相同时间戳、光学 frame_id 和 1280×720 尺寸，否则拒绝记录。深度话题来自上游已对齐 RGB 的同步转发节点；上游会将深度时间戳重标为 RGB 时间戳，因此元数据明确标记为转发后的同步时间，并不声称两次硬件曝光完全同时。

“获取当前帧”和“记录头部当前数据”均等待请求之后的新 RGB-D 数据，超过 2 秒的旧数据不可用。网页不自动传输画面；无有效数据时保持黑色。记录使用捕获前后两次 SDK 全身回读，将捕获后的关节、Pose、臂角保存在 `robot_state.json`，同时记录前一次状态和回读时间区间，便于检查运动期间的时序；相机与 SDK 回读不属于硬件触发同步。

“重启摄像头服务（三路）”一次提交原开机服务的两组重启任务：系统级头部/左腕服务和用户级右腕服务。状态共享到三路相机，重复点击被拒绝，命令失败、超时或重启后未运行会显示具体错误；命令完成不等同于图像已就绪，图像就绪仍以回读为准。网页内部请求固定为 `POST /api/cameras/restart`，JSON `{"camera":"all"}`，不接受命令或脚本路径；MOCK模式禁止触发外部重启。

系统服务固定为 `vision-head-rgbd.service`、`vision-head-synced-rgbd.service`、`vision-head-owner.service`、`vision-head-camera.service`、`vision-head-media.service`、`vision-left-wrist-rgb.service`；用户服务固定为 `mui-right-wrist-rgb.service`。不调用原有条件式健康检查，不重启机器人SDK、运控或导航，不改自启动启用状态。

首次使用由管理员运行 `sudo bash /home/admin/mui/rokae_web_control/system/install_camera_restart_permission.sh`。脚本只安装两条固定相机命令的免密规则（查询和重启），并从本项目中保存的原开机定义恢复缺失的右腕用户服务；不启动、重启或enable服务，也不覆盖不同的已有文件。视觉项目源码和现场相机配置保持不变。右腕原开机定义已于2026-09-23迁移到 `/home/admin/termitech-zhongmian/vision`。

如果现场另有终端手启相机驱动，按钮会报告其PID并中止，不启动第二份驱动。须先由该进程的操作人员退出手动实例，再用原开机服务接管。服务缺失或重启权限未装好时，两个重启组均不下发。

已有头部手眼外参保持不变，位姿估计输入改为同一组 ROS2 RGB-D，并使用订阅的 RGB CameraInfo 内参。

点击“记录当前数据”时会创建：

```text
data/YYYYMMDD/HHMMSSmmm/
├── head_rgb.jpg                         # 头部开启时存在
├── head_depth_aligned.npy               # 头部开启时存在
├── head_camera_metadata.json            # 头部开启时存在
├── left_wrist_rgb.jpg                   # 左腕开启并记录左腕时存在
├── left_wrist_depth_aligned.npy         # 左腕开启并记录左腕时存在
├── left_wrist_camera_metadata.json      # 左腕开启并记录左腕时存在
├── right_wrist_rgb.jpg                  # 右腕开启时存在
├── right_wrist_depth_aligned.npy        # 右腕开启时存在
├── right_wrist_camera_metadata.json     # 右腕开启时存在
├── robot_state.json
└── record_metadata.json
```

点击哪个画面的记录按钮，就只为该摄像头生成对应的三个文件；即使两台摄像头都已开启，也不会混在一次记录中。`*_depth_aligned.npy` 为与对应 RGB 同尺寸的 `float32` 毫米深度矩阵，0 表示无效深度；`robot_state.json` 保存当次上身全部关节角、双臂/躯干 Pose、双臂臂角和运行状态；`record_metadata.json` 记录本次摄像头及文件映射。

## 头部相机眼在手标定

`rokae_web/head_kinematics.py` 按整机 URDF 把躯干 4 关节和头部 2 外部轴组成一条 6 自由度运动链：

```text
chassis_link
  -> Calf_joint -> Thigh_joint -> Waist_joint -> Chest_joint
  -> Neck_joint -> Head_joint -> Head_link
```

`tools/calibrate_head_handeye.py` 从 `data/20260914` 读取 45 组头部 RGB-D 和同步保存的关节状态，使用 14×9 ChArUco 板（`DICT_5X5_100`、方格 20 mm、标记 15 mm）标定 `Head_link <- head_camera_color_optical_frame`。它只读离线记录，不连接控制器，也不会控制机器人运动：

```bash
python3 tools/calibrate_head_handeye.py
```

结果保存为 `calibration/head_camera_handeye_20260914.json`。当前 45 张图全部识别；URDF 正运动学相对 SDK 躯干 Pose 的最大位置误差为 0.067 mm，手眼结果的固定标定板一致性平移中位误差约 1.17 mm、P95 约 2.06 mm。

把头部相机光学坐标中的三维点换算到 `chassis_link` 时，使用记录目标帧时的 6 个关节角：

```bash
python3 tools/transform_head_camera_point.py \
  --state data/20260914/154654109/robot_state.json \
  --point-mm 0 0 1000
```

换算链为 `T_chassis_head(q) × T_head_camera × p_camera`。相机安装位置发生位移或拆装后必须重新标定。

## 已确认的硬件映射

| 模块 | 控制器/IP | SDK 映射 |
| --- | --- | --- |
| 左臂 | `192.168.71.161` | AR SDK `ArRobot(remote_ip, 192.168.71.51)`，取前 7 轴 |
| 右臂 | `192.168.71.160` | AR SDK `ArRobot(remote_ip, 192.168.71.51)`，取前 7 轴 |
| 躯干 | `192.168.71.162` | `PCB4Robot`，4 轴 |
| 头部 | 躯干控制器外部轴 | 2 轴，随 `JointPosition.external` 下发 |
| 底盘 | `192.168.71.50` | `/sr_amr_control/remote_control_enabled` + `/sr_amr_control/remote_control_cmd_vel` |

## 测试

```bash
/usr/bin/python3 -m unittest discover -s tests -v
/usr/bin/python3 -m compileall -q .
```


## 网页单帧采集接口（2026-09-18）

“获取当前帧”按 `camera-capture-api-3.13-merged.md` 调用现有相机 Owner 的
`GET http://127.0.0.1:8085/camera/capture?camera=head&streams=color`。
已配置的腕部摄像头使用 `left_wrist` / `right_wrist` 参数。接口返回 JSON 后，
校验 `ok`、`capture_id`、摄像头及 JPEG 信息，再读取 `/shared/frames/{capture_id}/`
内的图片并通过网页原有单帧地址返回；失败显示接口错误，不回退旧帧。
网页预览只请求 color；数据记录、位姿估计仍使用原有同步 ROS RGB-D 流。
相机服务会为每次点击生成采集文件，其保留策略由相机服务管理。
如 Owner 端口变更，可在 `ros_camera.capture_url` 配置完整采集接口地址。
更改 Python 后端文件后，需要重新启动网页服务才能加载。


### 左手 D405 RGB 采集

左腕序列号 `262622270751`，由 `vision-left-wrist-rgb.service` 随开机启动，
只发布 `/camera/left_wrist/color/image_raw`，1280×720、15 fps；不启动深度流。
网页“摄像头采集”区可分别获取头部和左手当前帧，均沿用 HTTP capture 接口。
原“记录头部当前数据”保持同步 RGB-D 和机器人状态记录逻辑。
新增“采集左手 RGB 画面”通过 `POST /api/cameras/left_wrist/record-rgb` 获取新单帧，
只保存 JPEG，不读取机器人状态。默认目录：
`/home/admin/mui/rokae_web_control/left_wrist_rgb/YYYY-MM-DD/HHMMSSmmm.jpg`。
可通过 `ros_camera.left_wrist_rgb_directory` 指定目录；同毫秒文件自动避重，不覆盖已有照片。

### 抓取后的扫码采集

位姿估计区提供 Avene物品、estee物品、origins物品、篮筐四个按钮，全部使用头部同步 RGB-D。
`POST /api/pose-estimation/grasp-object` 接受 `{"target":"Avene","side":"RIGHT"}`；
商品 target 也可为 `estee` / `origins`，side 为图像中的 LEFT / RIGHT；篮筐使用 `{"target":"basket"}`。
请求只传对应类别及公共输入，类别几何参数采用定位服务默认值。直接使用返回的本帧坐标，不按 ok、各类 *_valid 或置信度做质量门拦截，不按相机/底盘重复字段差值拦截。仍须包含计算所需字段及正确单位、格式；空坐标不补造。
响应 `localization` 是该类正式参考点及方向/矩阵，保存到本次结果文件；篮筐中心与 CAD 原点分开表示。
Avene 保留原固定躯干高度抓取规划，与新接口可见轴段中点分开显示。
其他类别只定位，不生成 Avene 抓取目标；最近结果切换到其他类别时，禁止回退旧 Avene 结果执行抓取。

“下一步：扫码采集”为独立按钮，调用 `POST /api/scan-sequence/start`（空 JSON）。
启动时冻结扫码1–5、L2抓取记忆点及网页速度；每段双臂先完成排队，再在 SDK 常驻服务中并发发送启动指令。
等待两臂的关节和 TCP 均到位且控制器静止后，采集到位之后的新左腕 ROS RGB 帧。
每次执行创建 `left_wrist_rgb/YYYY-MM-DD/scan_HHMMSSmmm/`，五张图命名为 `扫码1.jpg` 至 `扫码5.jpg`。
任何采集或运动失败都会取消后续阶段，保留已保存的部分图片，并在网页显示当前数量和目录。

拍完五张后，以扫码5的双臂 SDK world 位姿为基准，仅把左 Y 替换为 +50 mm、右 Y 替换为 -50 mm；
该段进行 MoveL 轨迹检查，检查失败即停止。随后双臂一起返回 L2抓取，均到位后才恢复 L2抓取躯干关节。
头部一直保持启动时位置，夹爪不操作。启动指令并发发送不等同于控制器的硬件触发同步。
`POST /api/scan-sequence/stop` 取消后续动作并停止控制器；停止未确认时保持运动互锁。
测试只使用模拟控制器；实机部署后不会自动启动此流程。SDK broker 更新也需要重启 `mui-sdk.service`。


## 深处商品抓取补偿（grasp-test-v14）

左右臂在预抓取到抓取段规划无解时，按50/100/150/200mm顺序演算躯干SDK X+前移，并与手臂同步运动，物品目标冻结不变。普通可达路径不前移。左臂在提起后沿躯干X−后退d+30mm，再躯干后退100mm。

有前移A时，提起后右臂相对肩部退d+20−A、左臂退d+30−A（保留负值），右侧躯干同步退A，两者到位后右臂再抬升75mm，到位后躯干独立后退100mm结束；左侧躯干同步退A，再单独躯干退100mm。对应控制器均到位才继续，任一失败停止两者，不释放物品。速度沿用网页快照；保护仍是Chest_link终点规则。

实现：`rokae_web/grasp_compensation.py`；候选常量 `ADVANCE_OPTIONS_MM`。接口完整预规划在 `agent_planning.py`，网页/接口共用 `grasp_test.py` 执行；接口执行保持预规划选中的档位。每档结果、目标、起点、臂角、同步下发时间差、到位/失败均记控制日志。具体接口行为见 `AGENT_API.md`。安装后下次重启mui.target并刷新生效，不需重启SDK上报/相机。


## 规划前张开右夹爪（grasp-test-v15）

接口和网页白罐右臂抓取均在规划前确保夹爪完全张开：目标0、到位状态3、实测0–5。未到位则发送张开并等待，受阻、故障、超时、取消均中止；停止未确认保持锁定。已张开不重复发指令。位姿质量标志仅作为原始响应保存，不拦截坐标使用。缺少必要数值、错误坐标单位或无法定义的几何计算仍报告具体数据错误。


## 左臂距离调整（grasp-test-v16 / placement-v4）

左臂抓取后退余量为30 mm：普通路径退d+30，有前移补偿A时相对肩部退d+30−A。右臂仍使用20 mm余量；躯干后退顺序和距离不变。盒子预放置采用篮筐左肩参考点Y+110 mm。网页与接口共用上述参数。

## 右臂补偿后退分段（grasp-test-v17）

同步后退只收回躯干前移量A；右臂抬升75 mm到位后，再独立执行躯干后退100 mm。最后100 mm纳入接口完整预规划，任一前序步骤失败或取消均不继续下发。手臂退距和总后退距离不变，左臂仍使用30 mm余量和放置Y+110 mm。

## 软管抓取（grasp-test-v20）

网页选择 `tube / RIGHT`，识别或转换最新软管结果后点击“软管抓取测试”。Manipulation 抓取接口支持 `sku_typ=tube, hand=RIGHT, level=L2`，与网页共用流程；瓶子、盒子的运动流程保持原样。

软管使用视觉返回的 `top_edge_center_camera_mm`，无 Y+10 mm 补偿。不按质量标记、重复坐标差值或辅助边缘字段拦截中点；保留必要坐标、单位和既有运动几何约束。预抓取法兰高度来自 `poses/tube_grasp.json`，为“软管预抓取点”保存状态换算出的躯干 SDK Z=812.9056959008101 mm。只提取高度，不回放记忆点的 XY、关节或臂角。

规划前右夹爪预开到130（目标130、到位状态3、实测130±5），然后在任何手臂/躯干运动前演算五段及对应躯干终点；已在130不重复下发，受阻/超时/停止未确认仍沿用开爪互锁。白罐仍开到0：

1. 水平到预抓取点：虚拟长度 d+250 mm。
2. 水平到抓取点：虚拟长度175 mm，法兰保持同一高度。
3. 保持法兰XYZ，原地下俯30°；到位后合爪并确认接触/闭合。
4. 单条 MoveL 同时回正30°并后退 d−20 mm，法兰不抬升。
5. 右臂退40 mm、躯干退100 mm并发启动，双方均到位后完成并保持夹持。

抓取段不可达时，仍按 A=50/100/150/200 mm 尝试躯干前移补偿，执行时固定使用完整预规划选中的 A。第4段手臂相对肩部退 d−20−A mm，同时躯干退 A 回到起始位置；第5段仍为40/100 mm。负的相对手臂退距保留，表示前伸。没有额外40 mm提起、75 mm抬升或独立100 mm后退。旋转按躯干参考系矩阵变换后转换到肩部，不直接修改奇异姿态的欧拉角分量。

实现 `rokae_web/tube_grasp.py`；所有速度取网页启动时快照，沿用终点肘部保护和停止互锁。这里的同步是两个控制器并发启动、等待双方到位，不保证同一时刻到达。安装后按原方式重启 mui.target 并刷新加载 v20；安装本身不启动运动。

## 右臂夹具长度、软管扫码与放置

右臂白罐和软管抓取均使用175 mm虚拟夹具长度（网页及接口的白罐/软管共用几何），左臂仍165 mm。预抓取 d+250、其他退距、软管预开130保持原值。

网页选择软管后，扫码按钮运行 `scan-sequence-v5`：

1. 双臂到“软管扫码1”，拍第一张左腕RGB。
2. 右臂保护MoveL沿右肩Y−右移100 mm，保持姿态。
3. 仅右臂回放历史翻转A的完整7轴关节角/位姿；左臂保持扫码1。
4. A到位后，右臂保持A朝向沿右肩Y+左移100 mm，到位后拍第二张左腕RGB。
5. 右臂保持姿态沿右肩Y−右移100 mm。
6. 双臂回独立L2抓取，均到位后躯干沿当前SDK X+前进100 mm结束。

A来自2026-09-23 19:19:33.278成功扫码2后右移100 mm的实际到位回读，完整记录及来源在 `poses/tube_scan_turn.json`；不修改原软管扫码1/2或教学记忆点。A与扫码1右移后的点相距81.35 mm，用户确认直接回放历史点，因此该段并非严格原地旋转。原扫码2记忆点不再下发。两张照片保存在同一目录；进入末段躯干前进前，头部、躯干保持不动，全程不改变夹爪/吸盘。网页和 `/manipulation/rotate` 共用流程；每个阶段必须到位才能继续，失败停止且不自动回位。

网页“软管放置”和 `POST /manipulation/place` 的tube/RIGHT分支共用 `placement-v7`：

1. 镜像L2盒子预放置法兰位置，Y=篮筐右肩参考Y−80 mm；法兰改为水平朝胸部正前方（右肩RPY [180,−90,0]），右臂保护MoveL到位。
2. 躯干沿SDK X+100 mm前进到位，双臂关节保持不动。
3. 保持这个起始朝向直接将右臂J6减30°，到位后夹爪全开至0并确认。
4. J6恢复下摆前的值，并确认右臂位姿恢复；再双臂回L2抓取，头部/躯干回L2抓取。

取消原Z−100 mm下降，不插入恢复旧放置朝向的步骤。完整预规划覆盖J6两段限位和回位；任一步失败/取消均停止后续动作，不自动松手或回位。法兰位置/方向先计算，再按实际右TCP换算，兼容非零TCP。现场标定、记忆点、其他项目未改。

软管的 `/manipulation/pick`、`/manipulation/rotate`、`/manipulation/place` 已全部接通，共用网页执行与互锁。pick使用完整视觉返回且不设质量标志门，规划前预开130；rotate返回两张左腕图片路径；place使用完整篮筐定位并全开夹爪。三个POST均使用Idempotency-Key，重复相同请求不会再次运动。字段、动作和返回值见 `AGENT_API.md`。
