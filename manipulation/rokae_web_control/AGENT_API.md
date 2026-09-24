# Mui Agent 接口（2026-09-22）

机器人地址 `192.168.130.105`。网页 8091；BodyPose 8082；Manipulation 8086；相机统一 8085；原只读上身遥测 8092 保留。

## 运行方式

`mui-sdk.service` 继续独立持有 SDK。新增 `mui-control.service` 持有唯一运控会话，提供 8082、8086 和本机 `.run/control.sock`。网页 `server.py --hardware` 只展示静态页面并代理原 `/api/*`，可独立反复启停。网页与 Agent 可同时在线，运动互斥，网页停止/锁定会取消 Agent 后续步骤。

日常抓取、扫码、放置和接口更新在**下次重启 `mui.target` 后生效**；仅重启网页前端不能加载后台修改。`mui.target`只管理网页与接口，常驻SDK/上报、底盘和相机保持运行。准确启停边界见`SERVICE_LIFECYCLE.md`；首次升级上报采集器需要单独确认一次SDK服务重启。

```bash
cd /home/admin/mui/rokae_web_control
./start_hardware.sh
systemctl --user status mui-control.service mui-sdk.service
# 后续更新后台代码，在机器人空闲时：
systemctl --user restart mui.target
```

平移/旋转速度使用网页保存的同一组值，服务启动默认 50 mm/s、6°/s。MoveJ/MoveAbsJ 沿用现有 SDK 关节比例规则，不将其声称为独立的 °/s 限速。

## 公共约定

运动 POST 使用 JSON 对象，并且必须携带 `Idempotency-Key`。同一键和同一报文重试返回已保存结果，不重复执行；同键不同报文返回 409；执行中或重启后结果未知返回 409，不自动重发。换键意味着一次新动作。

接口同步等待完整动作结束；成功 `HTTP 200 {"status":"SUCCEEDED",...}`，错误非 2xx，包含 `status=ERROR,error_code,message`。客户端需允许较长等待（建议 30 分钟），超时后先以**同一个幂等键**查询结果，不生成新键盲目重试。网页忙、机器人仍运动、模式/层号不支持、定位字段缺失、规划失败等均报错并记录日志。

## BodyPose：8082

### GET /pose/health

检查控制器状态和右夹爪，不检查左吸盘。右夹爪未激活且机器人空闲时会激活右夹爪；忙时返回错误，不插入激活动作。该接口有明确的夹爪激活副作用，机械臂/躯干不运动。

### POST /pose/prepare

```json
{"pose_type":"AGV_carton_item_inspect","level":"L2"}
```

| pose_type | level | 当前行为 |
|---|---|---|
| AGV_carton_item_inspect | L1–L5 | 均执行完整 `L2观察` 记忆点 |
| AGV_item_barcode_scan | 不传 | 执行 poses/barcode_trunk.json 固定扫码躯干标定，只动躯干，双臂/头部保持 |
| basket_item_place_prepare | L1–L4 | 均执行完整 `L2抓取` |
| basket_push | L1–L4 | 均执行完整 `L2抓取`，这里只是准备姿态 |
| 所有 Review 姿态 | — | 501 NOT_IMPLEMENTED，不运动 |

业务位姿保存在独立 poses 文件中，不依赖网页可删除的记忆点；标定缺失、重名或模式不匹配会报错。最近抓取状态只记日志和 last_pick_trunk.json，不覆盖固定扫码标定。

### GET /pose/camera_transform?camera=head

实时回读躯干与头部关节，经现有 URDF 和手眼标定返回头部相机**光学坐标系到 chassis_link** 的 4×4 变换：

```json
{
  "T_chassis_camera": [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
  "t_unit": "m",
  "camera": "head",
  "camera_frame": "head_camera_color_optical_frame",
  "base_frame": "chassis_link",
  "sampled_at_unix_s": 0,
  "upper_body_joints_deg": [0,0,0,0,0,0]
}
```

上例矩阵仅展示结构，实际每次计算。该接口不拍照，不做硬件触发同步。按用户确认的流程，定位方先调用它，再拍照定位，再传定位结果；抓取侧使用实时回读，不要求回传原请求元数据。当前仅提供头部外参。

## Manipulation：8086

### GET /manipulation/health

只回读状态，不激活夹爪、不使能或移动机器人。右夹爪未就绪返回错误，先调用 `/pose/health`。

### POST /manipulation/pick

动作外层和视觉结果内统一使用 `sku_typ`，支持 `bottle/RIGHT`、`tube/RIGHT` 和 `box/LEFT`，目前只配置 `L2` 抓取。旧外层 `sku_id`、`class_name` 拒绝；Review 和其他抓取层号尚不支持。

软管抓取、扫码、放置已接入对应 Manipulation 接口，与网页共用同一动作实现（grasp-test-v20 / scan-sequence-v5 / placement-v7）。

```python
import json, requests
result = json.load(open("box_localization_response.json", encoding="utf-8"))
response = requests.post("http://192.168.130.105:8086/manipulation/pick",
    headers={"Idempotency-Key": "box-0001-pick"}, timeout=1800,
    json={"task_type":"SORTING", "target_type":"sku", "sku_typ":"box",
          "hand":"LEFT", "level":"L2", "localization_result":result})
print(response.status_code, response.json())
```

罐子或软管使用同一接口，分别改为 `sku_typ=bottle` 或 `sku_typ=tube`、`hand=RIGHT`，并传对应类别的完整定位结果。软管请求示例：

```json
{"task_type":"SORTING","target_type":"sku","sku_typ":"tube","hand":"RIGHT","level":"L2","localization_result":{"sku_typ":"tube","request_id":"tube-001","output_frame":"head_camera_color_optical_frame","output_unit":"mm","top_edge_center_camera_mm":[500,2,700],"front_panel_plane_point_camera_mm":[300,0,700],"front_panel_plane_normal_camera":[-1,0,0],"front_panel_top_edge_midpoint_camera_mm":[300,0,800]}}
```

示例坐标仅说明字段，实际请求直接传本次视觉返回值；请求头必须包含独立 `Idempotency-Key`。

`localization_result` 放定位服务返回的 JSON 对象，不把字段展平到动作外层，不需要 RGB/深度图片。建议直接保留完整定位响应，关闭大图可视化。计算所需字段：

| 类别 | 字段 |
| --- | --- |
| 公共 | 匹配的 `sku_typ`、`output_frame=head_camera_color_optical_frame`、`output_unit=mm` |
| 罐子 | `reference_point_camera_mm` |
| 盒子 | `top_point_camera_mm` |
| 软管 | `top_edge_center_camera_mm` |
| 箱沿 | `front_panel_plane_point_camera_mm`、`front_panel_plane_normal_camera`、`front_panel_top_edge_midpoint_camera_mm` |

上述点和法向量各含三个有限数值。直接使用本帧相机系数据计算，不按 `ok`、各类 `*_valid` 或置信度做质量门拦截，也不按相机/底盘重复字段差值做一致性阈值拦截；`request_id` 用于关联日志。目标方向由本服务赋为躯干 SDK Z+，不用视觉返回的轴方向。盒子应识别左箱，罐子和软管应识别右箱。

白罐右臂接口在请求基本参数校验通过后，先确认夹爪完全张开：已在张开端则直接继续，否则发送目标位置0并等待回读。必须同时满足目标位置0、到位状态3和实测位置0–5（兼容标定后张开端约3）；受阻、故障、超时或取消均不进入目标计算、预规划或躯干动作。网页右臂抓取也执行此检查；未解锁或未识别夹爪时不再跳过夹爪继续抓取。接口与网页共用确认逻辑，起身后的再次确认不会重复下发已完成的张开命令。

软管接口则确认夹爪预开到130：目标130、到位状态3、实测130±5；之后才冻结目标和完整预规划，受阻或失败不继续。已到位不重复下发，白罐仍完全张开至0。

盒子接口在请求基本参数校验通过后，先关闭吸盘并确认继电器回读，再开始目标计算和预规划；关闭失败或未确认即报错，不规划、不起身。此关闭动作每次新任务只执行一次，躯干起身后不重复关闭。

三类都按当前头部外参计算目标及箱沿，冻结在躯干 SDK 参考系下；预先演算躯干到固定 `L2抓取` 的目标、到位后重投影的全部手臂段和最后躯干后退终点。任一预规划失败，记录错误并返回，手臂和躯干不启动；右夹爪此前已到对应预开位置、盒子吸盘此前已关闭，规划失败后保持该状态。

预规划通过后先动躯干，双臂/头保持；躯干到位回读并重投影冻结目标，再执行对应网页抓取流程：

- **罐子 / 右手**：固定高度 792.90086 mm，躯干 Y+10 mm 补偿；预抓取（虚拟长度 d+250 mm）→抓取（175 mm）→合爪→躯干 Z+40 mm→手臂躯干 X−(d+20 mm)→躯干 Z+75 mm→躯干 X−100 mm，结束。右夹爪必须就绪。
- **盒子 / 左手**：读取 `poses/box_grasp.json` 独立抓取/下压/提起高度（当前约 802.900785 / 772.908198 / 892.900662 mm），不做 Y 补偿；沿用规划前已关闭的吸盘→预抓取（d+230 mm）→抓取（165 mm）→仅调整法兰躯干 Z 到下压高度→打开吸盘并确认→仅调整法兰躯干 Z 到提起高度→左臂沿躯干 X−后退 (d+30) mm→躯干 X−100 mm，结束并保持吸盘打开。不读取右手夹爪。网页抓取测试仍在自身流程开始时关闭吸盘。

- **软管 / 右手**：固定法兰高度取 `poses/tube_grasp.json`（812.9056959008101 mm），无 Y 补偿；预开130→水平预抓取（d+250 mm）→水平抓取（175 mm）→法兰XYZ不变下俯30°→合爪255→单条MoveL回正并沿躯干X退d−20 mm→右臂退40 mm与躯干退100 mm同步、两者到位后结束。没有额外提起或抬升。

三类的“预抓取→抓取”若所有臂角均无可行保护 MoveL，依次演算躯干 SDK X+前移 A=50、100、150、200 mm；取首个躯干终点逆解和手臂保护 MoveL 均通过的档位。抓取点冻结不变，手臂目标换算为躯干前移后的肩部坐标；前移时对应手臂和躯干同时启动，每个控制器各一条 MoveL。取消、通信错误、工具或状态改变不触发补偿。全部档位无解直接报错，不先移动躯干试探。

补偿后的下降/提起仍保持抓取点的躯干 SDK X/Y。提起之后，右臂相对肩部沿躯干 X−移动 `d+20−A` mm，左臂移动 `d+30−A` mm（负数表示相对肩部前伸，不截为0）：罐子后退时躯干同步退 A mm，两者到位后右臂再抬升75 mm，到位后躯干独立后退100 mm，结束；盒子与躯干同时后退 A mm，两者到位后躯干单独后退100 mm，结束并保持吸盘打开。未使用补偿时按上面的普通流程执行。

软管使用补偿A后，下俯抓取仍围绕补偿后的固定法兰XYZ；回正后退段手臂相对肩部退d−20−A mm、躯干同步退A，最后仍同步退40/100 mm。保留负的相对退距。网页和接口均提前演算软管全部5段以及对应躯干段；接口还包含起始躯干到L2的准备动作。

接口仍在任何手臂/躯干运动前完成整套预规划，包含选中补偿档位的后续提起、同步后退及最终动作。选中的 A 内部传给执行流程，实际执行前使用最新臂角复核该档位，不临时换成另一未完成全流程预规划的档位。白罐/盒子网页按钮在抓取段实时选择档位；软管网页先完成全流程预规划。同步启动不代表等时到达，两者均到位才继续；所有 MoveL 使用任务开始保存的网页平移/旋转速度。

每段手臂执行前用新的实测臂角做保护 MoveL；初次全流程演算使用前一段预计终点的臂角。保护仍按 Chest_link 终点规则，并非全路径碰撞检测。运行中状态变化或通信失败仍可能中止；失败不会自动释放盒子或重发动作。

成功返回 `status=SUCCEEDED`、`box_clearance` 和 `completed_moves`。后者与网页抓取流程一致：普通罐子6、补偿罐子6、盒子6、软管5；同步手臂/躯干段计一个运动目标，不包含接口前置的躯干到 L2 动作。

### POST /manipulation/rotate

罐子：`{"sku_typ":"bottle","hand":"RIGHT"}`；兼容原 `{"hand":"RIGHT"}`。

盒子：

```json
{"sku_typ":"box","hand":"LEFT"}
```

罐子复用网页五次扫码并返回左腕五张 RGB；盒子复用已验证的单次扫码：双臂同时到固定“盒子扫码”位姿，躯干不动，右腕拍一张 RGB；随后双臂同时回固定 L2抓取，两臂到位后躯干回 L2抓取。盒子扫码全程不主动改变吸盘状态。

软管请求：`{"sku_typ":"tube","hand":"RIGHT"}`。双臂到“软管扫码1”→左腕拍照→右臂沿右肩Y−右移100 mm→仅右臂按完整七轴关节/位姿回放历史翻转A点→保持A朝向沿右肩Y+左移100 mm→第二次左腕拍照→右臂Y−右移100 mm→双臂回L2抓取并均到位→躯干从当前SDK X前进100 mm。A取2026-09-23 19:19:33.278成功“软管扫码2后右移100 mm”的回读，独立存于 `poses/tube_scan_turn.json`，不再直接回放原软管扫码2记忆点。A与扫码1右移后的点有约81.35 mm位置差，本段包含平移和翻转，不是固定XYZ旋转。保持左臂、头部和夹持状态；返回 `camera=left_wrist`，`image_paths` 按第1、2次拍照顺序含两张图片，全部动作到位才返回成功。

盒子返回示例：

```json
{"status":"SUCCEEDED","camera":"right_wrist","image_paths":["/shared/frames/capture-box/rgb.jpg"]}
```

实际路径由服务生成，指机器人本机或共享挂载上的文件，不是 HTTP 下载地址。扫码完成全部回位才返回成功；罐子照片顺序对应扫码1–5。

### POST /manipulation/place

```python
result = json.load(open("basket_localization_response.json", encoding="utf-8"))
response = requests.post("http://192.168.130.105:8086/manipulation/place",
    headers={"Idempotency-Key":"box-0001-place"}, timeout=1800,
    json={"task_type":"SORTING","target_type":"sku","sku_typ":"box",
          "destination_type":"basket","hand":"LEFT","localization_result":result})
print(response.status_code, response.json())
```

罐子或软管分别改为 `sku_typ=bottle` 或 `sku_typ=tube`、`hand=RIGHT`。`localization_result` 放完整篮筐定位响应：包含 `point_semantics=basket_model_center`、`model_center_camera_mm`、`pose_4x4` 等定位数据；`ok`、`pose_valid` 等质量标志不拦截坐标使用；坐标系应为头部相机光学系、平移单位 mm。篮筐响应不带商品 `sku_typ`，商品类别只放动作外层。

接口实时计算篮筐到对应肩部坐标系的参考点，不读取网页定位文件。三类放置均在首步运动前演算全部动作，包括末尾回位；失败返回错误并记录阶段，整套动作不启动。

- **罐子**：沿用原右臂放置全流程（L2放置1、对齐篮筐 Y−30、右臂目标 X=篮筐 X−350 与躯干 X+200 同步、J7−30、松夹爪、恢复 J7、同步回退、取消 Y 对齐、右臂回起点）。
- **盒子**：取固定 L2盒子预放置左臂位姿，仅改 Y=篮筐左肩 Y+110 mm→躯干 SDK X+100 mm 前进到位（双臂关节保持不动）→左臂保护 MoveL 沿左肩 Z−100 mm 下降到位→关闭吸盘并确认→双臂同时回 L2抓取→头部与躯干回 L2抓取。接口与网页按钮共用此顺序；前进、下降分两步，不同时启动。全流程预规划也先预测躯干前进，再用前进后的状态演算左臂下降。右夹爪不参与，平移及旋转速度沿用网页设置。

- **软管**：镜像L2盒子预放置的法兰位置，Y=篮筐右肩Y−80 mm；法兰姿态改为右肩 `[180°,−90°,0°]`，即夹具水平朝胸部正前方，保留原位置高度。保护MoveL到位→躯干SDK X+100 mm到位→右臂当前J6−30°→夹爪完全张开至0并确认→J6恢复下摆前原值并确认整臂回到下摆前位姿→双臂回L2抓取→头部/躯干回L2抓取。取消Z−100 mm下降，不在前进后切换回旧朝向。完整预规划覆盖J6上下摆的关节限位和最后回位；任何一步失败或取消都停止后续动作，不自动释放或回位。

完整放置及回位结束后返回 `{"status":"SUCCEEDED"}`。独立业务标定位姿不受网页记忆点删除影响。

### POST /manipulation/pick_review_item、/manipulation/push

返回 501 NOT_IMPLEMENTED，不运动。

## Camera：8085

| 路径 | 输入 | 输出 |
|---|---|---|
| GET /camera/health | — | READY/ERROR，头部检查 RGB+对齐深度，腕部 RGB |
| GET /camera/list | — | CameraInfo 数组，规范 ID：head/left_wrist/right_wrist |
| GET /camera/snapshot | camera、type=color/depth；depth 必须 format=raw/preview | JSON image_path、尺寸、capture_id |
| GET /camera/rgbd | camera=head | 同一 ROS 时间戳的 RGB 与对齐深度文件路径，raw 深度 uint16 mm `.npy` |
| GET /camera/stream | camera、type；深度仅 format=preview | MJPEG |

腕部 depth/rgbd 明确返回不支持，不会自动开启深度。head raw 深度仅用于文件单帧，MJPEG 深度只能预览。

保留原 `/camera/capture` 供兼容；`/list` 保留旧包装；旧 vision 图片 URI 改用 `/camera/frame` 二进制路由，正式 `/camera/snapshot` 按文档返回 JSON。旧相机驱动、推流和自动恢复继续使用原服务。网页预览、保存、估姿取图、扫码都使用 8085；只保留 ROS 驱动在相机服务内部。

右腕服务：`systemctl --user status mui-right-wrist-rgb.service`，SDK SN `262622272351`，RGB8 1280×720@15，深度关闭。配置位于 `/home/admin/vision/config/vision.json` 的 `rokae.owner.cameras.hand_right`。没有启用右腕公网推流。

## 日志与验证边界

沿用 `logs/motion-YYYY-MM-DD.jsonl`，包含 Agent 入参、请求 ID、预检输入输出、原生 SDK 演算、速度、动作步骤、失败和回读。幂等记录 `.run/agent/actions.sqlite3`，扫码躯干姿态 `.run/agent/barcode_trunk.json`。未知结果不得删记录后重试。

开发验证使用离线 SDK 替身、HTTP 回归和真实相机只读取图；**未执行实机抓取/放置/扫码/准备姿态，也未调用会激活夹爪的实机 health**。实机运动仍需用户现场测试。

自动健康检查 `head-stream-check.py` 已兼容 CameraInfo 数组和旧包装，避免新接口触发误恢复；真实只读检查 source/owner/push_progress 全部通过。


## 位姿协议

视觉 `/infer` 使用 `sku_typ=bottle/box/tube`，不发旧 `sku_id`、`class_name` 或 `z_ref_mm`；篮筐不发商品类别。新响应严格校验类别，冗余 `class_name` 若存在须与 `sku_typ` 同值。历史本地存档只读兼容旧名称，新接口请求不接受旧商品字段。
