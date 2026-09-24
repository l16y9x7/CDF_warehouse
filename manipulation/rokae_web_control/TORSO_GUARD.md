# 网页及接口 MoveL 终点躯干保护

配置文件为本目录的 `torso_guard.json`。左右臂共用设置，每次任务读取；修改代码后需重启 `mui-control.service`，单独修改配置值在下一次任务读取时生效。

```json
{
  "plane_offset_mm": 130.0,
  "elbow_radius_mm": 65.0,
  "margin_mm": 10.0,
  "front_plane_x_mm": 220.0
}
```

所有平面使用 URDF `Chest_link` 胸部坐标系，随胸部运动。右臂允许肘点中心 `Y <= -205 mm`，左臂允许 `Y >= 205 mm`，其中205为平移130、半径65和余量10之和。

额外允许肘点中心 **X>220 mm** 时越过上述Y保护边界。X恰好等于220不豁免；X阈值不再叠加半径或余量。`front_plane_x_mm` 缺失或设为 `null` 时关闭该豁免，恢复单Y平面规则。

候选仍按当前/初始臂角、−5、+5、−10、+10……依序尝试。原生checkPath可达、SDK和URDF关节限制以及关节步长限制仍需通过；找到第一个满足组合条件的候选即返回单条MoveL。普通无保护MoveL不使用这项规则。

此逻辑由 `rokae_web/arm_movel.py` 的 `HardwareMoveL.plan` 共用，适用于网页保护运动、抓取/放置中的保护MoveL及接口对应的预规划。它只检查终点肘点，不是运动全过程或全身碰撞检测。独立 `arm_motion_control` 目录的旧分段规划功能保持原样。

日志记录胸部系肘点XYZ、原Y平面余量、X平面阈值及中心超出距离；`accepted_by=front_plane` 表示使用X豁免，`accepted_by=y_plane` 表示满足原Y规则。
