# 机械臂状态缓存接口

本项目适配用户提供的`MANIPULATOR_STATE_API.md`返回格式。SDK连接、状态采集和8092服务仍归开机常驻`mui-sdk.service`；device侧使用本地Python包读取HTTP缓存，不另建SDK连接，不占用独占运控会话。

```python
from rokae_robot_state import build_manipulator_state_service
from rokae_web.config import load_config

service = build_manipulator_state_service(load_config('/home/admin/mui/rokae_web_control/config.json'))
service.start()
snapshot = service.get_state()  # 仅复制缓存，无SDK/HTTP操作
data = snapshot.to_mapping()
legacy = snapshot.to_legacy_dual_arm_mapping()
service.stop()  # 仅停止自己的HTTP缓存线程，常驻SDK/上报继续运行
```

外部集成将项目根目录加入Python路径，或复制独立的`rokae_robot_state`包；包只依赖Python标准库。默认读取本机8092，可配置`manipulator_state.url`为对应机器人地址。

## 返回结构

```json
{
  "schema_version": 1,
  "measured_at_ms": 1785945651000,
  "dual_arm_available": false,
  "health": {
    "left_arm": {"connected": false, "fresh": false, "sample_count": 0, "error_count": 0, "last_sample_age_ms": null, "last_error_code": "not_sampled"},
    "right_arm": {"connected": false, "fresh": false, "sample_count": 0, "error_count": 0, "last_sample_age_ms": null, "last_error_code": "not_sampled"},
    "body": {"connected": false, "fresh": false, "sample_count": 0, "error_count": 0, "last_sample_age_ms": null, "last_error_code": "not_sampled"}
  }
}
```

对应采样在500ms内且读取成功时才增加`left_arm/right_arm/body/head`组件；过期/失败省略，不补零。左右臂7轴、body4轴、head2轴。body和head由同一次PCB4采样批次拆出，共享时间戳和健康状态；各SDK getter不是硬件原子快照，不声称三套控制器同一硬件时刻。

组件字段遵循参考文档：`state`公共状态、`power_state`、`operate_mode`、原始`operation_state`、`joint_positions_deg`、真实速度/力矩（SDK不提供则省略）、法兰基座`end_pose=[x,y,z,roll,pitch,yaw]`（mm/deg）、软限位、软限位开关、`measured_at_ms`、控制器信息和最近至多3条warning/error日志。头部没有伪造的法兰位姿、力矩或软限位总开关。

右夹爪上报真实原始状态位与0–255的`requested_position/position/current_raw`，不将电流原始值换算为安培。左手是吸盘，因此不伪造左Robotiq夹爪字段。夹爪使用单次只读Modbus请求，无激活、开合、故障复位或重试等待。

`dual_arm_available`只表示双臂新鲜，躯干另看`health.body.fresh`。旧OSD方法只返回双臂的`state/joint_positions_deg/end_pose`，任一臂不可用则返回空字典。完整结构不要直接塞入尚未扩展的旧`data.manipulator_status`；本次没有改动device或外部平台合同。

## 采样和查询

- 常驻层三条采样线程目标10Hz，采用现有SDK对象；不增加控制器连接。
- 每个getter单独尝试SDK锁，运控请求等待/执行时跳过，不持锁睡眠。已经开始的单次SDK读取不能中途抢占；实际频率取决于SDK耗时和运控负载，忙时宁可字段过期省略。
- 控制器日志最多每5秒查询一次，右夹爪最多每1秒查询一次；控制器信息/限位至多每60秒刷新。可选字段读取失败省略，不影响其他已测字段。
- SDK读取成功次数/失败次数在常驻进程生命周期内累计，锁忙不记作连接错误。健康错误摘要最多160字符，成功后清空。
- 客户端HTTP错误使组件立即不可用；相同样本重复拉取不重置数据年龄，get_state还会累计本地缓存等待时间。
- `GET http://192.168.130.105:8092/api/telemetry/manipulator-state`返回`{"ok":true,"data":上述结构}`。HTTP200表示缓存查询成功，是否有可用组件必须看health。
- 原`/api/telemetry/upper-body`及单模块路由继续保留，维持嵌套end_pose、原始state和3秒过期规则；复用新采样缓存，没有第二轮SDK采样。
- `/health`的`manipulator_state_version=1`表示新版采集已加载。8092所有写请求仍返回405。

默认配置新增于项目默认配置，现场`config.json`不需要修改：

```json
{"manipulator_state":{"sample_hz":10,"freshness_ms":500,"log_query_interval_sec":5,"gripper_query_interval_sec":1,"include_grippers":true}}
```

与参考机器人实现的差异：保持相同Python调用和输出结构，device侧只开HTTP缓存线程，三条SDK采样线程放在唯一常驻所有者中。服务生命周期和命令见`SERVICE_LIFECYCLE.md`。本次首次加载常驻层需用户确认一次SDK服务重启，日常网页/接口开发不需要。
