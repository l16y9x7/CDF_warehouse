# ROKAE Helios 视觉

## SMT 天机相机模型对照（Helios）

权威模型：`kim/dog_device` SMT（`CAMERA_TOPIC_INTEGRATION.md` / `RosTopicVideoAdapter`）。
Helios 用独立 `vision` 仓落地**相同所有权规则**，不在本机整仓跑 dog_device
`camera_bringup`（SMT tip 驱动为 RealSense/V4L2，无 Orbbec）。

| SMT 规则 | SMT 落点 | Helios systemd / 代码 |
| --- | --- | --- |
| 唯一开设备 | `camera_bringup` | `vision-rokae-preview-head-rgbd` / `right-rgbd` |
| 推流不开 USB | `RosTopicVideoAdapter` | `vision-rokae-preview-media` + `media.push.source=ros` |
| 快照/HTTP 不开第二份设备 | 订 Topic 或只读 gateway | `vision-rokae-preview-owner`（HTTP:8085，订阅 ROS） |
| 软件同步输出 | synced forwarder | `vision-rokae-preview-synced-rgbd` |
| FFmpeg 挂 ≠ 重启相机 | video worker 退避 | Media `Restart=on-failure`，无 `Requires/PartOf` 到 head-rgbd |
| capture ≠ platform | `capture.*` / `platform.*` | Owner 采集宽高 fps vs Media 编码 640×480@8 |

`vision.ownership.SOURCE_RUNTIME_UNITS` 固化上表单元名。`media.push.source=ros` 时
`ensure_ros_media_subscriber_only()` 若进程已加载 `capture`/`devices`/厂家 SDK 则 fail closed。

现场验收（`.135`）：只重启 Media 时 head 主 PID 与 `NRestarts` 必须不变。

## ROS 直接推流与 HTTP 并行

`media.push.source="ros"` 时，Media 从原生 ROS 彩色 Topic 获取最新帧，直接向 FFmpeg
提供原始像素，再编码为 H.264/FLV 上传 RTMP。该路径不读取 HTTP 图像或 HTTP 相机状态，
也不打开硬件。未配置 source 或设为 `http` 时保留原有 MJPEG 来源；ROS 故障不会静默回退 HTTP。

HTTP Owner 继续独立订阅 ROS，为现有快照和 MJPEG 预览服务。原生彩色、深度和 CameraInfo
话题保持不变；Media 只消费彩色，不改深度配对或标定。采集帧率与编码帧率独立：当前现场
推流保持640×480@8、300k，头部彩色采集仍1280×720@15。

Media、Owner、同步与 HTTP 应用进程统一使用服务模板中的 Conda nav Python3.10，并先加载
原有 ROS setup；拉起厂家节点的 RGB-D supervisor 保持 ROS Humble 系统 Python3.10 ABI，
厂家节点及 ROS/L4T 依赖仍保持现场匹配版本。迁移时备份现场 source/config/service，
停止旧 Media 后再启动新 Media，避免同槽位双推流。不要覆盖现场密钥或相机启用状态。
回退时恢复原配置、代码和服务，仅重启 Media。

验收必须分别检查 HTTP 快照、ROS 新鲜度、编码器输入及平台 WSS 实际解码；进程运行或状态
online 不能单独证明云端播放成功。禁止在日志和验收材料中记录推流密钥。

码率是现场参数：2026-09-17实测旧1000k配置出现TCP发送积压，两路改为300k，
保持640×480@8。网络条件变化时重新评估画质和带宽，不修改通用三路模板。

## 当前 ROS RGB-D 部署（2026-09-15）

`rokae.owner.source=ros` 时，厂家 ROS 驱动唯一持有设备；Owner 订阅彩色
`sensor_msgs/Image` 后继续提供原 HTTP/JPEG/MJPEG，Media 继续推送 ROBOT_001
槽位 1/3。`source=direct` 保留此前 V4L2/SDK 彩色方案；两种来源不能同时开相机。

合同按 `kim/dog_device` SMT @ `db852628aad39f23358f56f0ecaf0cdd301b1f4f`，
ROS 角色使用 `head` / `right_wrist`，与天机一致；HTTP 业务角色为 `head` /
`hand_wrist`，并兼容输入 `hand_right` / `right_wrist` / `right`。Media 内部仍使用
`hand_right`。本台现场 `hand_left` 未安装，现场配置保持禁用、不启动左手节点；通用模板保留三路。

| 数据 | Topic（`<id>` 为 head 或 right_wrist） | 类型 |
| --- | --- | --- |
| 彩色 | `/camera/<id>/color/image_raw` | Image，RGB8 |
| 彩色标定 | `/camera/<id>/color/camera_info` | CameraInfo，厂家内参与畸变 |
| 对齐深度 | `/camera/<id>/aligned_depth_to_color/image_raw` | Image，16UC1，毫米 |
| 对齐深度标定 | `/camera/<id>/aligned_depth_to_color/camera_info` | CameraInfo，彩色坐标系 |
| 原始深度 | `/camera/<id>/depth/image_rect_raw` | Image，16UC1，深度坐标系 |

RealSense 还提供 `/camera/right_wrist/depth/camera_info` 原始深度标定。
Orbbec 软件对齐模式的原生 `depth/camera_info` 描述对齐深度，已一起映射到
`aligned_depth_to_color/camera_info`；不能把这份内参当作未对齐深度内参。
RGB-D 消费按 SMT 核心三元组（彩色、对齐深度、彩色 CameraInfo）进行。

2026-09-16：头部彩色1280×720@15、原始深度640×480@15；右手彩色和深度保持640×480@15。头部软件对齐深度及对应CameraInfo输出1280×720，原始深度仍为640×480。平台编码为640×480@8、H.264；2026-09-17针对现场上行带宽将两路码率各调为300k。
ROS 消息保留厂家源时间戳。深度和标定不从 JPEG 推测，也不伪造硬件同步时间。
厂家提供的是相机内参/传感器间标定；本次不建立机器人或机械臂到相机的外参。

### Domain 与启动

四项 ROS systemd 服务统一读取 `config/ros/ros.env`：当前按机器人业务网 IP 尾号生成 `ROS_DOMAIN_ID=51`、
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`。现场默认 Fast DDS 共享内存路径曾出现
能发现 topic 却无数据；服务使用仓库 `fastdds-udp.xml` 指定 UDP 传输。
配置依据 [Fast DDS UDP XML 文档](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/udp/udp.html)。

在现场运行 ROS 命令或 SMT 消费程序前：

```bash
source /home/admin/vision/config/ros/setup.bash
echo "$ROS_DOMAIN_ID"
ros2 topic hz /camera/right_wrist/color/image_raw
```

`ros.env` 中 `VISION_ROS_DOMAIN_ID=auto` 按 `VISION_ROS_NETWORK=192.168.71.0/24`
选择唯一 IPv4 地址，在启动时取尾号；临时断网不会修改运行中的域。多地址或尾号
超出本部署 1..101 范围时拒绝启动，不取模、不回退到 0。可将
`VISION_ROS_DOMAIN_ID` 改为明确的 1..101 值。变更 IP/Domain 后统一重启两项 RGB-D 服务和 Owner；所有
跨进程/跨机器消费者也必须使用相同 Domain。不同机器上的 XML 路径应指向其本地副本。外部订阅端显式设置
`ROS_DOMAIN_ID=51`，不能按订阅电脑自身 IP 计算。同一 DDS 网络中的机器人必须
保证域号唯一，不同子网相同尾号也需显式分配。上述 setup 脚本需要 Bash。
范围采用 [ROS 官方建议](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html)
中的通用区间，并按用户要求排除 0。

服务模板在 `config/ros/`：`vision-rokae-preview-{head-rgbd,right-rgbd,owner,synced-rgbd,health}.service`
以及 `vision-rokae-preview-health.timer`。健康检查从现场配置读取 Owner 端口，逐路检查所有
启用的 Owner 相机、ROS RGB-D/synced Topic 和 Media stream；默认连续三次失败后才按
`health.service_prefix` 与 `health.repair_units` 定位责任单元。`.51` 使用
`vision-rokae-preview-`，`.105` 未配置时继续兼容 `vision-head-`。
ROS 门禁可通过 `health.ros_requirements.<owner_camera_id>` 为每路指定
`require_depth`、真实深度 Topic、`require_synced` 和角色专属 `repair_unit`。
彩色-only 角色必须同时设置 `require_depth=false`、`require_synced=false`；其故障只重启
对应 source unit，不能重启头部。ROBOT_001 的 head 使用原生
`/camera/head/depth/image_rect_raw` 且不要求 synced；hand_right 映射 ROS
`right_wrist`，使用 aligned depth 并要求 synced。未配置覆盖时继续默认检查 aligned depth
和 synced，保持 ROBOT_011 单头合同不变。
Owner 与 synced 服务只声明启动顺序，不通过 `Wants=` 拉起相机驱动；部署时仅 enable
现场配置中启用角色对应的 RGB-D unit，单头机器不得因启动 Owner 而带起右腕驱动。
相机启动读取现场 `vision.json` 的 enabled、backend、精确 SN、彩色宽高及 fps
（旧现场可继续使用 `ros_fps`）。RealSense 启动前通过 SDK 枚举指定 serial 的真实
color profile；精确的宽×高@fps 不存在时明确失败，不让驱动静默协商成其他规格。
原生驱动启动后由 `driver_supervisor` 监视：**稳态只按彩色投递**判定是否退出重启
（深度抖动不得杀 Orbbec）。重启前尽量 USB reset；systemd `RestartSec=12`。
这与 SMT「平台推流失败不重启相机」一致：Media/FFmpeg 故障只动 Media 单元。

切换前先备份原配置与 Owner 服务，停止旧直采 Owner 释放设备，再启动原生 ROS
驱动及 ROS 来源 Owner。回退先停止两项 RGB-D 服务，再恢复原配置/Owner 服务。
禁止在原生驱动仍占用设备时恢复直采 Owner。

验收使用 SMT 原版 `capture_to_capture_dir`，检查 `rgb.png`、毫米 `depth.png`、
`camera.json`、`capture_manifest.json`，执行 100ms 时间偏差和 2s 新鲜度限制；
云端槽位解码另行验证。视觉展示成功不能代替 RGB-D 数据合同验收。

## 来源和边界

对照 [dog_device SMT](http://192.168.100.100/kim/dog_device/-/tree/SMT) @ `b5a94086f155e8b1e6df9196666d642e0beabf89`：
`doc/ROKAE_BRANCH_CONSOLIDATION.md`、`doc/UNIFIED_ROBOT_RUNTIME_CONSOLIDATION.md`、`doc/CAMERA_TOPIC_INTEGRATION.md`、`doc/OSD_DATA_FORMAT.md`。
ROKAE 历史保留在 `a4a9846` 的 `src/camera/helios_camera.py` 和 `hand_camera_handler.py`，现代 SMT 尚无可部署的 Helios Profile。

| 合同角色 | RTMP slot | 支持的采集方式（实际绑定以现场为准） |
| --- | --- | --- |
| head | 1 | V4L2 彩色或显式选择 Orbbec SDK |
| hand_left | 2 | V4L2 彩色，或按 serial 独占的 RealSense ROS 彩色源 |
| hand_right | 3 | V4L2 彩色；优先按已确认的 USB 序列号绑定 |

相机型号和左右绑定以现场为准，不因逻辑角色推断三台都支持深度。

原生 ROS 模式还支持独立的 `vision-rokae-preview-left-color.service`。该服务统一使用
Conda `nav` 的 Python 3.10（并设置 `PYTHONNOUSERSITE=1`），通过其中已验证的
`pyrealsense2` 与 `rclpy` 直接发布彩色 Topic，不依赖
`realsense2_camera` ROS 包，也不会启动深度、红外或同步流。通用发布模块同时接受
`hand_left` / `hand_right` 角色；服务模板按现场角色传入参数。例如 D405 左腕可配置
`backend=realsense`、`match.type=realsense_serial`、精确 serial、`1280x720@15`、
`enable_depth=false`、`require_depth=false`、`require_synced=false`，只发布
`/camera/left_wrist/color/image_raw`，发布使用 Sensor Data/BEST_EFFORT QoS，frame_id 为
`left_wrist_color_optical_frame`（右腕对应 `right_wrist_color_optical_frame`）。启动会按 serial
和精确的宽×高@fps 检查设备 profile；不支持时直接失败，不能静默协商。Owner 与 Media
都只是 Topic 订阅者，不再次打开 USB。

部署门禁：必须在与 systemd 完全相同的隔离条件下验证依赖，普通交互式 `nav` 能导入不算
通过，因为它可能误用了 `~/.local` 用户目录：

```bash
PYTHONNOUSERSITE=1 /home/admin/miniconda3/envs/nav/bin/python -c \
  'import pyrealsense2, rclpy, sensor_msgs'
```

若失败，必须先把 `pyrealsense2` 实际安装到 `nav` 环境，再进行候选启动；不能移除
`PYTHONNOUSERSITE=1`，也不能为绕过依赖切回 uv、用户目录或另一套 Python。
Media 的 `width/height/fps/bitrate` 属于独立编码规格，不会反向改变相机采集 profile。
可按对象键和 `camera_id` 合并的配置片段见 `config/ros/left-color.example.json`；它不是完整
现场配置，不能用片段中的 `streams` 数组整体替换现场数组，部署时必须保留原有头部、SN、
密钥、Domain 和其他角色配置。
此前 `source=direct` 模式只提供 RGB JPEG / MJPEG；完整 ROS RGB-D 模式见上节。
SMT 的 RGB-D Topic 合同不可等同于本仓库 HTTP FrameRef。

## 启动

复制 `config/vision.rokae.example.json` 为现场配置，并设置 `VISION_CONFIG`。
默认三路均为 `backend=v4l2`，与已有现场基线一致。头部 `match.type=entity`，按现场填写 `match.value`（例如 `Orbbec Gemini 335L`）。
Orbbec 是相机厂商名，不代表必须使用 SDK。切换后端前须用实际画面确认，不能仅凭品牌或格式名称推断图像正确性。
可选 SDK 模式需要显式设置 `head.backend=orbbec`、`head.match={"type":"orbbec","serial":"现场SN"}`，并安装匹配固件/平台的 `pyorbbecsdk`。
SDK 用 `get_stream_profile_by_index(...).as_video_stream_profile()` 读取视频参数，参考 [官方绑定](https://github.com/orbbec/pyorbbecsdk/blob/main/src/pyorbbecsdk/stream_profile.cpp)。
SDK 留空序列号仅允许唯一 Gemini 候选；多台设备时拒绝猜测。
左右手配置 `match.type=usb_path` 与各自物理 USB 路径，不依赖 `/dev/videoN` 编号。
有已确认序列号时使用 `match={"type":"usb_serial","value":"现场相机SN"}`：
仅从该 USB 设备选择可采彩色节点，设备缺失或序列号对应多个物理设备时拒绝选源；
此模式不允许 `match.device` 绕过序列号约束，也不会使用父级 USB Hub 的序列号。
发现阶段只接受已验证的 YUYV/MJPG/UYVY 节点；不从无法出图节点兜底。

```bash
VISION_CONFIG=config/vision.local.json bash vision.sh start
curl -fsS http://127.0.0.1:8085/camera/list
curl -fsS http://127.0.0.1:8003/list
curl -fsS http://127.0.0.1:8003/frame/head
```

Owner `:8085` 唯一持有设备；Adapter `:8003` 消费 HTTP；Media `:8005` 消费 MJPEG。
重复 start 不再打开已运行设备。配置错误只使对应角色不可用。
同一 Owner 中拒绝重复 V4L2 节点/SN；设备选择不能混用 SDK 与 V4L2 指向同一物理相机。
多进程设备互斥仍依赖部署只启动一个 Owner，不能同时运行官方驱动与本地直采。

## 状态与异常

`enabled` 表示配置启用，`ready` 表示最近有效帧，不能互相替代。
列表内任何显式 `ready/fresh/online=false` 会令该路不可用；不可用帧请求返回 `CAMERA_NOT_READY`。
聚合 `ok` 延续既有“至少一路可用”语义，不能代表三路验收通过；Owner health 增加 `all_ready`（所有启用角色就绪，至少启用一路）。
三路验收需检查三个角色均 enabled/ready。

帧过期后快照不可用，已有 MJPEG 会话结束。客户端/FFmpeg 需要重新连接。
坏帧转换异常不会终止 SDK 循环，停止后不再返回旧缓存；SIGTERM 正常退出。
SDK 初始化和采集由同一个线程拥有；`start()` 表示已安排采集，不表示已经出帧。
设备启动较晚或首次 SDK 初始化失败时，每两秒按原序列号重试。
连续五秒无有效彩色帧（含异常或坏帧）时清除缓存，关闭旧 pipeline，退避后重建；首次以唯一 Gemini 选定的设备也会固定其序列号，恢复时不改选其他设备。
已启动的 V4L2 连续读帧失败后会释放旧句柄、清理旧帧，并按原 match 重新发现设备后重开；重开有退避且排除其他角色已占用的节点。
SDK 等待期间 Owner 列表提供 `DEVICE_NOT_FOUND`、`DEVICE_AMBIGUOUS`、`CAPTURE_START_FAILED`、`WAITING_FOR_FRAME` 等状态，真正出帧后清除错误。
如果 SDK 的停止调用抛出异常或采集线程仍未退出，保留资源引用、禁止重开；该保护不能识别 SDK 内部吞掉的停止错误，也不能强制打断原生阻塞。

推流目标保持 `{sn}_{slot}_99-0-0_normal-0`；SN 与机器人身份一致，stream_key 只在现场填写。
相机采集和推流独立；推流失败不改变相机就绪状态。
ROKAE 推流启动前检查目标相机的 `enabled/ready`，不能用头部就绪代表右手就绪。
目标没有帧时等待输入，不反复启动 FFmpeg，也不增长 RTMP 故障退避；输入恢复后自动启动推流。
Camera `/list` 和 `/state` 透传 `DEVICE_NOT_FOUND` 等原因；`/frame/{id}` 继续以 HTTP 404 和
`error_code=CAMERA_NOT_READY` 保持合同兼容，并通过 `reason` 区分设备缺席、角色禁用和 Owner 不可达。

## 验证

```bash
python -m unittest discover -s tests -v
```

测试涵盖三路不同图像、单路故障/恢复、重复绑定、坏配置、Orbbec 精确选择/坏帧/资源释放、SIGTERM 和三路 FFmpeg 编解码。
测试使用 fake/mock，不代表真机色彩、实际左右安装位置、持续帧率、USB 恢复或云端流已经验收。
现场应检查三路真实图像、遮挡对应关系、帧龄、CPU/内存、延迟及断流恢复后再发布。

## 语言选择

当前保留 Python 控制面与 OpenCV/SDK/FFmpeg 原生处理链。先测实际三路 CPU、拷贝开销和端到端延迟。
若采集/传输层成为瓶颈，再单独评估 C++ Owner，保留 HTTP 合同；本轮不进行整仓 C 重写。

复查补充：如果原生采集 read 阻塞到停止超时，保留线程和设备引用并拒绝再次打开，避免双线程访问同一原生句柄；这不是自动解除驱动阻塞。Owner 会保留停止失败的 worker，待其结束再停止，必要时由进程管理器重启。
Media 无效 camera_id 返回 CAMERA_NOT_FOUND，不影响其他流；禁用流不能显式启动。

## V4L2 格式与恢复

每路可显式配置 `fourcc`（`MJPG` / `YUYV` / `UYVY`）。未配置时保持驱动默认；配置后设备探测和正式采集使用同一格式，不改变后端。
现场曾出现默认格式读超时，而相同设备以 V4L2 MJPEG 640×480@30 可采帧。该配置需要按具体硬件验证，不能套用于所有相机。

采集预热在单读线程中执行；`start()` 成功不等于首帧已经就绪。连续三次读失败后清空缓存，等待一秒再重新解析原绑定；没有匹配时不退回旧 `/dev/videoN`，也不切换 SDK。
重开期间 `ready=false`。首次启动就没有匹配设备时，Owner 保留等待线程，每秒按原绑定重新发现；设备接入且采到新帧后恢复，无需重启 Owner。缺失角色不会占用其他角色的节点，重复绑定仍被拒绝。
`stop()` 能打断退避等待，但不能保证打断原生 open/read 或发现阶段的阻塞系统调用。

## 查看与平台

Owner 的 `/preview` 提供同源预览，仅显示启用的角色，自动重连。这是本地检查入口；平台效果以平台对应 SN 的播放源为准。
沿用 Tianji 的 Media→RTMP→平台 WSS 流程；SN由部署配置指定，头部slot1、左手slot2、右手slot3固定。只有头部与右手硬件时，保持左手禁用，不复用其他画面填槽。
变更 SN 时先停止旧 Media 实例，更新现场配置并重启，使旧推流退出，避免新旧身份并存。不要将现场 SN/签名硬编码进公共模板。

平台验证须从对应 `wss://robotsolution.cn:11081/dksl/stream/{sn}_{slot}_99-0-0_normal-0.live.flv?originTypeStr=rtmp_push` 拉取并解码；仅进程在线不代表云端播放成功。

## 2026-09-14 图像选源复盘

先前按普通 USB Camera 名称和端口绑定头部/右手，虽然可以出帧和云端解码，实际图像来源错误。
现场 `/home/admin/smt/dog_device/config/device_config.local.json` 的相机角色和序列号，以及历史头部录制脚本，提供了比节点枚举更可靠的身份依据。
当前头部 Gemini 335L 无 V4L2 节点，单独重新绑定接口仍未恢复；使用现场已有 SDK 环境实采后确认头部清晰图像并上线。此现场例外不改变公共模板的 V4L2 默认值。
右手应为现场 SMT 配置的 RealSense 序列号；缺少该设备时应返回 `DEVICE_NOT_FOUND`，不能替换成普通 USB Webcam。
平台将 640×480 图像压成浅条是独立显示问题：已发现前端 CameraGrid 配置 `isResize:false`。
应在平台源码中改为保持比例并正确显示离线角色，不能通过更换采集后端、拉伸编码画面或给缺失槽位填入其他相机来掩盖问题。
# RealSense serial binding (2026-09-15)

For D435i units whose USB descriptor has no `serial`, use the optional installed
`pyrealsense2` SDK for discovery while retaining V4L2 capture:

```json
{"backend":"v4l2","match":{"type":"realsense_serial","value":"261722071471"},"fourcc":"YUYV"}
```

`realsense_serial` selects exactly one SDK device and exactly one sensor exposing
`stream.color`, then probes that sensor's `physical_port` video node. It ignores
an explicit `match.device` override and respects nodes claimed by other cameras.
Missing SDK/device, ambiguous identity, missing RGB sensor or invalid sysfs port
returns unavailable; the existing V4L2 worker retries discovery.

Do not select the first YUYV node on a RealSense USB device: a D435i infrared node
can also negotiate YUYV and return a valid three-channel image. This was observed
on the field unit; visual verification caught its black-and-white projector dots.
Only the SDK RGB sensor identifies the intended color node.

On this deployment the newly connected D435i reports SDK serial `261722071471`;
the prior SMT right-camera serial `261222078991` describes different hardware.
Head remains `CP2N1630002S`; left has no installed hardware and stays disabled.
These are deployment values, not universal defaults or proof of physical mounting.

SDK API reference: https://realsenseai.github.io/librealsense/python_docs/_generated/pyrealsense2.html

## 与天机统一的同步输出

ROS 相机命名为 head/left_wrist/right_wrist；本台现场左手无硬件，现场配置保持禁用。
此前珞石的 `/camera/hand_right/*` 已迁移到 `/camera/right_wrist/*`，旧 ROS 订阅需要更新。
HTTP 对外规范名为 `hand_wrist`，兼容旧输入别名；ROBOT_001 槽 3 与 Media 内部
`hand_right` 保持不变。

直接复用天机 `vision.tianji_runtime.synced_rgbd_forwarder`，提供 `/camera/<id>/synced/color/image_raw`、`/camera/<id>/synced/depth/image_raw`、`/camera/<id>/synced/color/camera_info`。QoS为RELIABLE，近似同步窗口100ms，输出三者时间戳统一为彩色时间戳。这是软件配对，原生话题保留源时间戳；不能把输出同时间戳解释为硬件同时曝光。头部彩色1280×720@15、原始深度640×480@15；右手保持640×480@15。原始深度标定差异见上文。

正式部署路径为 `/home/admin/vision`；服务名保留vision-rokae-preview前缀以兼容原有运维命令。原目录内容完整备份，未删除。

## Independent depth dimensions (2026-09-16)

`width`/`height` select native color resolution. Optional `depth_width`/`depth_height` select native depth resolution and default to color dimensions when absent. `ros_fps` applies to both streams. Values must be positive. Software-aligned depth follows the color image grid; this does not increase native depth sensor resolution.

### 现场参数与三路模板边界

通用模板保留 `head`、`hand_left`、`hand_right` 三路。以下仅为本台珞石
`rokae.owner` 的现场覆盖片段，不是完整配置，也不是通用模板默认值：

```json
{
  "source": "ros",
  "cameras": {
    "head": {"enabled": true, "width": 1280, "height": 720, "depth_width": 640, "depth_height": 480, "ros_fps": 15},
    "hand_left": {"enabled": true, "backend": "realsense", "match": {"type": "realsense_serial", "value": "D405_SERIAL"}, "width": 1280, "height": 720, "fps": 15, "enable_depth": false, "require_depth": false, "require_synced": false},
    "hand_right": {"enabled": false}
  }
}
```

应用时保留现场 `match` 身份、序列号及其余配置。仓库 `config/ros/synced_rgbd.yaml`
保留三路启用模板。同步器只启用需要 RGB-D 配对的角色；彩色-only 左腕不得在
`synced_rgbd.yaml` 中启用。
部署时保留该现场覆盖，不能用模板直接覆盖现场文件。
