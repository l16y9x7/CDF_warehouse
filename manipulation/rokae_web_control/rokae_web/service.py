from __future__ import annotations

import copy
import math
import threading
import time
import sys
from pathlib import Path
from typing import Any

from .audit import AuditSink, NullAuditLogger
from .backends import JOINT_COUNTS, POSE_MODULES, BackendError
from .camera_manager import CAMERA_LABELS, DisabledCameraManager
from .pose_estimation import PoseEstimationClient, PoseEstimationError
from .pose_frames import POSE_FRAMES
from .memory_points import MemoryPoints
from .arm_movel import ArmMoveL
from .grasp_test import GraspTest
from .scan_sequence import ScanSequence
from .placement_sequence import PlacementSequence
from .control_trace import context, fields


class ValidationError(ValueError):
    pass


def _finite_values(values: Any, count: int, label: str) -> list[float]:
    if not isinstance(values, list) or len(values) != count:
        raise ValidationError(f"{label} 必须包含 {count} 个数值")
    result: list[float] = []
    for value in values:
        if isinstance(value, bool):
            raise ValidationError(f"{label} 包含非法数值")
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValidationError(f"{label} 包含非法数值") from exc
        if not math.isfinite(number):
            raise ValidationError(f"{label} 包含非有限数值")
        result.append(number)
    return result


class ControlService:
    def __init__(
        self,
        config: dict[str, Any],
        robot_backend: Any,
        chassis_backend: Any,
        hardware_enabled: bool,
        audit: AuditSink | None = None,
        camera_backend: Any | None = None,
        pose_estimator: Any | None = None,
    ) -> None:
        self.config = config
        self.robot = robot_backend
        self.chassis = chassis_backend
        self.hardware_enabled = hardware_enabled
        self.speed_mm_s = float(config["motion"]["default_speed_mm_s"])
        self.rotation_deg_s = float(config["motion"]["default_rotation_deg_s"])
        self.armed = False
        self.gripper_unlocked = False
        self._suction_result = {'commanded_open': None, 'confirmed': False}
        self._lock = threading.RLock()
        self.audit: AuditSink = audit or NullAuditLogger()
        self._audit_error = None
        self._audit_failures = 0
        self.robot.audit_callback = self.audit_event
        self.chassis.audit_callback = self.audit_event
        self.camera = camera_backend or DisabledCameraManager()
        self.pose_estimator = pose_estimator or PoseEstimationClient(config)
        self._pose_estimation_lock = threading.Lock()
        self._sample_condition = threading.Condition()
        self._sample_stop = threading.Event()
        self._sample_thread: threading.Thread | None = None
        self._sample_until = 0.0
        self._sample_min_until = 0.0
        self._sample_idle_count = 0
        self._drag_sampling: set[str] = set()
        self._sample_context = {}
        self._sample_contexts = {}
        self.memory = MemoryPoints(self)
        self.arm_movel = ArmMoveL(self)
        self.grasp_test = GraspTest(self)
        self.scan_sequence = ScanSequence(self)
        self.placement = PlacementSequence(self)
        self.audit_event(
            "service_started",
            hardware_enabled=hardware_enabled,
            motion_limits=config["motion"],
        )

    def _is_armed(self) -> bool:
        return self.armed

    def _require_armed(self) -> None:
        self.memory.ensure_idle()
        self.arm_movel.ensure_idle()
        self.grasp_test.ensure_idle()
        self.scan_sequence.ensure_idle()
        self.placement.ensure_idle()
        if not self._is_armed():
            try:
                self.chassis.stop()
            except Exception:
                pass
            raise ValidationError("控制未解锁，请先解锁")

    def audit_event(self, event: str, **data: Any) -> None:
        try:
            self.audit.record(event, **data)
        except Exception as exc:
            # A failed log write must never prevent a stop/cancel command.
            self._audit_error = str(exc)
            self._audit_failures += 1
            if self._audit_failures == 1:
                try:
                    print(f"CONTROL LOG ERROR: {exc}", file=sys.stderr)
                except Exception:
                    pass

    def _sampling_active_locked(self) -> bool:
        return bool(self._drag_sampling) or time.monotonic() < self._sample_until

    def _ensure_sample_thread(self) -> None:
        if not self.hardware_enabled:
            return
        with self._sample_condition:
            if self._sample_thread is None or not self._sample_thread.is_alive():
                self._sample_thread = threading.Thread(
                    target=self._sample_state_loop,
                    name="rokae-state-audit",
                    daemon=True,
                )
                self._sample_thread.start()

    def _request_motion_sampling(self, trigger: str, module: str) -> None:
        if not self.hardware_enabled:
            return
        now = time.monotonic()
        maximum = float(self.config["logging"]["state_sample_max_seconds"])
        with self._sample_condition:
            self._sample_context = fields()
            if self._sample_context.get("operation_id"):
                self._sample_contexts[self._sample_context["operation_id"]] = dict(self._sample_context)
            self._sample_until = max(self._sample_until, now + maximum)
            self._sample_min_until = max(self._sample_min_until, now + 2.0)
            self._sample_idle_count = 0
            self._sample_condition.notify_all()
        self._ensure_sample_thread()
        self.audit_event("state_sampling_requested", trigger=trigger, module=module)

    def _set_drag_sampling(self, side: str, enabled: bool) -> None:
        if not self.hardware_enabled:
            return
        now = time.monotonic()
        with self._sample_condition:
            if enabled:
                self._sample_context = fields()
                if self._sample_context.get("operation_id"):
                    self._sample_contexts[self._sample_context["operation_id"]] = dict(self._sample_context)
                self._drag_sampling.add(side)
            else:
                self._drag_sampling.discard(side)
                self._sample_until = max(self._sample_until, now + 2.0)
                self._sample_min_until = max(self._sample_min_until, now + 1.0)
            self._sample_idle_count = 0
            self._sample_condition.notify_all()
        self._ensure_sample_thread()

    def _sample_state_loop(self) -> None:
        interval = float(self.config["logging"]["state_sample_interval_seconds"])
        while not self._sample_stop.is_set():
            with self._sample_condition:
                while not self._sampling_active_locked() and not self._sample_stop.is_set():
                    self._sample_condition.wait(timeout=1.0)
                if self._sample_stop.is_set():
                    return
                drag_sides = sorted(self._drag_sampling)
                sample_context = dict(self._sample_context)
                sample_context["related_operation_ids"] = list(self._sample_contexts)
                sample_context["related_request_ids"] = list({v["request_id"] for v in self._sample_contexts.values()
                                                               if "request_id" in v})
            token = context.set(sample_context)
            try:
                with self._lock:
                    state = self.robot.read_state()
                self.audit_event(
                    "robot_state_sample",
                    drag_sides=drag_sides,
                    state=state,
                )
                names = [str(value).lower() for value in state.get("operation_state", {}).values()]
                active_tokens = ("moving", "drag", "jog", "execution", "rtcontrolling", "unknown")
                controller_active = any(
                    token in name for name in names for token in active_tokens
                )
                with self._sample_condition:
                    if controller_active:
                        self._sample_until = max(self._sample_until, time.monotonic() + interval * 4)
                    if not self._drag_sampling and time.monotonic() >= self._sample_min_until:
                        self._sample_idle_count = 0 if controller_active else self._sample_idle_count + 1
                        if self._sample_idle_count >= 3:
                            self.audit_event("sampled_motion_idle", state=state)
                            self._sample_until = 0.0
                            self._sample_idle_count = 0
                            self._sample_contexts.clear()
            except Exception as exc:
                self.audit_event("robot_state_sample_failed", error=str(exc))
            finally:
                context.reset(token)
            if self._sample_stop.wait(max(0.05, interval)):
                return

    def status(self) -> dict[str, Any]:
        with self._lock:
            chassis_status = self.chassis.status()
            chassis_status["limits"] = {
                "min_linear_m_s": self.config["chassis"]["min_linear_m_s"],
                "max_linear_m_s": self.config["chassis"]["max_linear_m_s"],
                "min_angular_rad_s": self.config["chassis"]["min_angular_rad_s"],
                "max_angular_rad_s": self.config["chassis"]["max_angular_rad_s"],
                "lease_seconds": self.config["chassis"]["lease_seconds"],
            }
            motion_limits = copy.deepcopy(self.config["motion"])
            if self.hardware_enabled and hasattr(self.robot, "soft_limit_status"):
                try:
                    motion_limits.update(self.robot.soft_limit_status())
                    motion_limits.pop("joint_limit_error", None)
                except Exception as exc:
                    motion_limits["joint_limit_error"] = str(exc)
            return {
                "mode": "hardware" if self.hardware_enabled else "mock",
                "hardware_enabled": self.hardware_enabled,
                "armed": self._is_armed(),
                "suction": self.suction_status(),
                "gripper": {
                    "model": "2F-85",
                    "unlocked": self.gripper_unlocked,
                    "min_position": 0,
                    "max_position": 255,
                },
                "speed_mm_s": self.speed_mm_s,
                "rotation_deg_s": self.rotation_deg_s,
                "pose_frames": dict(POSE_FRAMES),
                "chassis": chassis_status,
                "cameras": self.camera.status(),
                "limits": motion_limits,
                "memory_execution": self.memory.status(),
                "arm_movel": self.arm_movel.status(),
                "grasp_test": self.grasp_test.status(),
                "scan_sequence": self.scan_sequence.status(),
                "placement": self.placement.status(),
                "logging": {"schema_version": 2, "directory": self.config["logging"]["directory"],
                            "error": self._audit_error, "failed_records": self._audit_failures,
                            "enabled": not isinstance(self.audit, NullAuditLogger)},
            }

    def set_camera_enabled(self, payload: dict[str, Any]) -> dict[str, Any]:
        camera_id = payload.get("camera", "head")
        if camera_id not in CAMERA_LABELS:
            raise ValidationError("未知摄像头")
        enabled = payload.get("enabled")
        if not isinstance(enabled, bool):
            raise ValidationError("enabled 必须为布尔值")
        result = self.camera.set_enabled(camera_id, enabled)
        self.audit_event(
            "camera_changed",
            camera_id=camera_id,
            enabled=enabled,
            status=result,
        )
        return result

    def camera_frame(
        self,
        camera_id: str,
        kind: str,
        after_sequence: int,
        timeout: float,
    ) -> tuple[bytes | None, int, bool]:
        return self.camera.frame_jpeg(camera_id, kind, after_sequence, timeout)

    def restart_camera(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(payload, dict) or set(payload) - {"camera"}:
            raise ValidationError("摄像头重启不接受命令或脚本路径")
        if payload.get("camera", "all") != "all":
            raise ValidationError("请刷新网页，使用三路摄像头服务重启按钮")
        if not self.hardware_enabled or not hasattr(self.camera, "restart"):
            raise BackendError("当前模式不能重启外部摄像头服务")
        result = self.camera.restart("all")
        self.audit_event("camera_services_restart_requested", cameras=list(CAMERA_LABELS), result=result)
        return result

    def record_camera_rgb(self, camera_id: str) -> dict[str, Any]:
        if camera_id not in ('left_wrist', 'right_wrist'):
            raise ValidationError('仅腕部相机支持独立 RGB 采集')
        if not hasattr(self.camera, 'record_rgb'):
            raise BackendError('当前模式不支持左手 RGB 采集')
        result = self.camera.record_rgb(camera_id)
        self.audit_event('camera_rgb_recorded', camera_id=camera_id, path=result['path'])
        return result

    def record_camera_snapshot(self, camera_id: str) -> dict[str, Any]:
        if camera_id not in CAMERA_LABELS:
            raise ValidationError("未知摄像头")
        if camera_id != 'head' and self.camera.__class__.__name__ == 'HttpCameraManager':
            result = self.camera.record_rgb(camera_id)
            self.audit_event('camera_snapshot_recorded', camera_id=camera_id, result=result)
            return dict(result, active_cameras=[camera_id], files=[Path(result['path']).name])
        if getattr(self.camera, "externally_managed", False):
            # Bracket the exact subscribed RGB-D pair with controller readbacks.
            # SDK reads are not hardware-trigger synchronized; retain the interval.
            with self._lock:
                before_started = time.time()
                before = self.robot.read_state()
                before_finished = time.time()
                snapshot = self.camera.fresh_snapshot(camera_id)
                after_started = time.time()
                state = self.robot.read_state()
                after_finished = time.time()
            state["capture_synchronization"] = {
                "source": "ros2",
                "camera_sequence": snapshot.sequence,
                "camera_color_timestamp_ms": snapshot.color_timestamp_ms,
                "state_before": before,
                "state_before_read_started_unix": before_started,
                "state_before_read_finished_unix": before_finished,
                "state_after_read_started_unix": after_started,
                "state_after_read_finished_unix": after_finished,
                "robot_state_selection": "state_after_rgbd_capture",
                "hardware_trigger_synchronized": False,
            }
            result = self.camera.save_snapshot(camera_id, snapshot, state)
        else:
            with self._lock:
                state = self.robot.read_state()
            result = self.camera.record(camera_id, state)
        self.audit_event(
            "camera_snapshot_recorded",
            camera_id=camera_id,
            directory=result["directory"],
            active_cameras=result["active_cameras"],
            robot_state=state,
        )
        return result

    def pose_estimation_health(self) -> dict[str, Any]:
        return self.pose_estimator.health()

    def estimate_grasp_object_pose(self, payload=None) -> dict[str, Any]:
        selected = None
        if payload:
            from .pose_targets import selection
            try:
                selected = selection(payload, self.config['pose_estimation'].get('side', 'RIGHT'))
            except ValueError as exc:
                raise ValidationError(str(exc)) from exc
        if not self._pose_estimation_lock.acquire(blocking=False):
            raise ValidationError("被抓取物品位姿估计正在进行，请等待当前请求完成")
        try:
            camera_status = self.camera.status().get("head", {})
            if camera_status.get("available") is False:
                raise BackendError("头部摄像头不可用")
            if not camera_status.get("enabled") and not getattr(self.camera, "externally_managed", False):
                started = self.camera.set_enabled("head", True)
                self.audit_event(
                    "camera_changed",
                    camera_id="head",
                    enabled=True,
                    source="pose_estimation",
                    status=started,
                )

            with self._lock:
                if getattr(self.camera, "externally_managed", False):
                    state_before = self.robot.read_state()
                    snapshot = self.camera.fresh_snapshot("head", self.pose_estimator.fresh_frame_timeout)
                else:
                    initial = self.camera.snapshot("head")
                    state_before = self.robot.read_state()
                    frame, sequence, enabled = self.camera.frame_jpeg(
                        "head", "rgb", initial.sequence, self.pose_estimator.fresh_frame_timeout,
                    )
                    if not enabled or frame is None or sequence <= initial.sequence:
                        raise BackendError("等待头部摄像头新 RGB-D 帧超时")
                    snapshot = self.camera.snapshot("head")
                state_after = self.robot.read_state()

            result = self.pose_estimator.estimate(snapshot, state_before, state_after,
                                                   **({'selected': selected} if selected else {}))
            self.audit_event(
                "grasp_object_pose_estimated",
                status=result["status"],
                usable=result["usable"],
                result_id=result["result_id"],
                relative_directory=result["relative_directory"],
                elapsed_seconds=result["elapsed_seconds"],
            )
            return result
        finally:
            self._pose_estimation_lock.release()

    def pose_estimation_image(self, day: str, leaf: str) -> bytes:
        return self.pose_estimator.result_image(day, leaf)

    def reproject_latest_grasp_to_right_shoulder(self, sku_typ='bottle') -> dict[str, Any]:
        """Read the current torso and re-express the saved world pose; no motion."""
        if sku_typ not in ('bottle', 'tube'):
            raise ValidationError('右臂转换仅支持 bottle、tube')
        with self._lock:
            state_before = self.robot.read_state()
            state_after = self.robot.read_state()
        try:
            project = (self.pose_estimator.reproject_latest_tube_to_right_shoulder if sku_typ == 'tube'
                       else self.pose_estimator.reproject_latest_to_right_shoulder)
            result = project(
                state_before, state_after
            )
        except PoseEstimationError as exc:
            raise ValidationError(str(exc)) from exc
        self.audit_event(
            "grasp_pose_reprojected_to_right_shoulder",
            source_result_id=result["source_result_id"],
            current_trunk_joints_deg=result["current_trunk_joints_deg"], result=result,
        )
        return result

    def reproject_latest_grasp_to_left_shoulder(self) -> dict[str, Any]:
        """Read the current torso and re-express the saved world pose; no motion."""
        with self._lock:
            state_before = self.robot.read_state()
            state_after = self.robot.read_state()
        try:
            result = self.pose_estimator.reproject_latest_to_left_shoulder(
                state_before, state_after
            )
        except PoseEstimationError as exc:
            raise ValidationError(str(exc)) from exc
        self.audit_event(
            "grasp_pose_reprojected_to_left_shoulder",
            source_result_id=result["source_result_id"],
            current_trunk_joints_deg=result["current_trunk_joints_deg"], result=result,
        )
        return result

    def arm(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            self.armed = True
            self.gripper_unlocked = True
            self.audit_event("control_unlocked", gripper_unlocked=True)
            return self.status()

    def disarm(self) -> dict[str, Any]:
        self.placement.stop()
        self.scan_sequence.stop()
        self.grasp_test.stop()
        self.arm_movel.stop()
        self.memory.stop()
        with self._lock:
            self.chassis.stop()
            self.armed = False
            self.gripper_unlocked = False
            self.audit_event("control_locked")
            return self.status()

    def suction_status(self):
        from .suction import validate_config, SuctionError
        try:
            validate_config(self.config.get('suction'))
            available, message = True, ''
        except SuctionError as exc:
            available, message = False, str(exc)
        return dict(self._suction_result, available=available, message=message)

    def set_suction(self, payload):
        from .suction import validate_config
        if set(payload) != {'open'} or type(payload['open']) is not bool:
            raise ValidationError('吸盘开关只接受 open 布尔值')
        with self._lock:
            self._require_armed()
            cfg = validate_config(self.config.get('suction'))
            self._suction_result = {'commanded_open': None, 'confirmed': False}
            try:
                result = self.robot.suction_set(payload['open'], cfg)
            except Exception as exc:
                self._suction_result['error'] = str(exc)
                self.audit_event('suction_command_failed', requested_open=payload['open'], error=str(exc))
                raise
            self._suction_result = result
            self.audit_event('suction_command', **result)
            return self.suction_status()

    def gripper_status(self) -> dict[str, Any]:
        with self._lock:
            result = dict(self.robot.gripper_status())
            result["unlocked"] = self.gripper_unlocked
            return result

    def set_gripper_unlocked(self, payload: dict[str, Any]) -> dict[str, Any]:
        unlocked = payload.get("unlocked")
        if not isinstance(unlocked, bool):
            raise ValidationError("unlocked 必须为布尔值")
        with self._lock:
            if unlocked:
                self._require_armed()
                state = self.robot.gripper_status()
                if state["fault_code"]:
                    raise ValidationError(f"夹爪故障码 0x{state['fault_code']:02X}，不能解锁")
            self.gripper_unlocked = unlocked
            self.audit_event("gripper_control_changed", unlocked=unlocked)
            return {"unlocked": unlocked}

    def _require_gripper_unlocked(self) -> None:
        self._require_armed()
        if not self.gripper_unlocked:
            raise ValidationError("夹爪控制未解锁")

    def activate_gripper(self) -> dict[str, Any]:
        with self._lock:
            self._require_gripper_unlocked()
            self.audit_event("gripper_activation_requested")
            try:
                result = self.robot.gripper_activate()
            except Exception as exc:
                self.audit_event("gripper_activation_failed", error=str(exc))
                raise
            self.audit_event("gripper_activation_completed", status=result)
            return {**result, "unlocked": self.gripper_unlocked}

    def move_gripper(self, payload: dict[str, Any]) -> dict[str, Any]:
        position = payload.get("position")
        if isinstance(position, bool) or not isinstance(position, int) or not 0 <= position <= 255:
            raise ValidationError("夹爪目标位置必须是 0–255 的整数")
        with self._lock:
            self._require_gripper_unlocked()
            self.audit_event("gripper_position_requested", position=position)
            try:
                result = self.robot.gripper_move(position)
            except Exception as exc:
                self.audit_event("gripper_position_failed", position=position, error=str(exc))
                raise
            self.audit_event("gripper_position_completed", position=position, status=result)
            return {**result, "unlocked": self.gripper_unlocked}

    def readback(self) -> dict[str, Any]:
        with self._lock:
            state = self.robot.read_state()
            self.audit_event("robot_state_readback", state=state)
            return state

    def set_speed(self, payload: dict[str, Any]) -> dict[str, Any]:
        values = _finite_values([payload.get("speed_mm_s")], 1, "速度")
        speed = values[0]
        motion = self.config["motion"]
        if not float(motion["min_speed_mm_s"]) <= speed <= float(motion["max_speed_mm_s"]):
            raise ValidationError(
                f"速度范围应为 {motion['min_speed_mm_s']}–{motion['max_speed_mm_s']} mm/s"
            )
        with self._lock:
            # Legacy clients may save translation alone; update the pair atomically.
            rotation = _finite_values([payload.get("rotation_deg_s", self.rotation_deg_s)], 1, "旋转速度")[0]
            if not float(motion["min_rotation_deg_s"]) <= rotation <= float(motion["max_rotation_deg_s"]):
                raise ValidationError(
                    f"旋转速度范围应为 {motion['min_rotation_deg_s']}–{motion['max_rotation_deg_s']} °/s"
                )
            self.speed_mm_s = speed
            self.rotation_deg_s = rotation
            result = {"speed_mm_s": speed, "rotation_deg_s": rotation}
            self.audit_event("speed_changed", **result)
            return result

    def _ensure_no_drag(self, state: dict[str, Any]) -> None:
        dragging = state.get("dragging", {})
        if dragging.get("left_arm") or dragging.get("right_arm"):
            raise ValidationError("任一机械臂处于拖拽状态时禁止关节或 Pose 运动")

    def move_joints(self, module: str, payload: dict[str, Any]) -> dict[str, Any]:
        if module not in JOINT_COUNTS:
            raise ValidationError("未知关节模块")
        values = _finite_values(payload.get("values"), JOINT_COUNTS[module], f"{module} 关节")
        with self._lock:
            self._require_armed()
            state = self.robot.read_state()
            self._ensure_no_drag(state)
            current = state["joints_deg"][module]
            max_step_value = self.config["motion"]["max_joint_step_deg"][module]
            if max_step_value is not None:
                max_step = float(max_step_value)
                deltas = [abs(target - now) for target, now in zip(values, current)]
                if max(deltas, default=0.0) > max_step:
                    raise ValidationError(f"单次关节变化不得超过 {max_step:g}°，请分步移动")
            self.audit_event(
                "joint_motion_requested",
                module=module,
                target_deg=values,
                speed_mm_s=self.speed_mm_s,
                state_before=state,
            )
            try:
                self.robot.move_joints(module, values, self.speed_mm_s)
            except Exception as exc:
                self.audit_event("joint_motion_failed", module=module, error=str(exc))
                raise
            self.audit_event("joint_motion_accepted", module=module, target_deg=values)
            self._request_motion_sampling("joint_motion", module)
        return {"accepted": True, "module": module, "values": values}

    def move_pose(self, module: str, payload: dict[str, Any]) -> dict[str, Any]:
        if module not in POSE_MODULES:
            raise ValidationError("未知 Pose 模块")
        values = _finite_values(payload.get("values"), 6, f"{module} Pose")
        elbow_deg = None
        if module in ("left_arm", "right_arm"):
            elbow_deg = _finite_values(
                [payload.get("elbow_deg")],
                1,
                f"{module} 臂角",
            )[0]
            if payload.get("frame") != POSE_FRAMES[module]:
                raise ValidationError(
                    "手臂 Pose 必须使用各自肩部原点的 SDK 世界坐标系；请刷新网页并重新回读"
                )
        with self._lock:
            self._require_armed()
            state = self.robot.read_state()
            self._ensure_no_drag(state)
            self.audit_event(
                "pose_motion_requested",
                module=module,
                target_mm_deg=values,
                target_elbow_deg=elbow_deg,
                pose_frame=POSE_FRAMES[module],
                speed_mm_s=self.speed_mm_s,
                state_before=state,
            )
            try:
                self.robot.move_pose(module, values, self.speed_mm_s, elbow_deg)
            except Exception as exc:
                self.audit_event("pose_motion_failed", module=module, error=str(exc))
                raise
            self.audit_event(
                "pose_motion_accepted",
                module=module,
                target_mm_deg=values,
                target_elbow_deg=elbow_deg,
                pose_frame=POSE_FRAMES[module],
            )
            self._request_motion_sampling("pose_motion", module)
        return {
            "accepted": True,
            "module": module,
            "values": values,
            "elbow_deg": elbow_deg,
            "frame": POSE_FRAMES[module],
        }

    def set_drag(self, side: str, payload: dict[str, Any]) -> dict[str, Any]:
        if side not in ("left_arm", "right_arm"):
            raise ValidationError("未知机械臂")
        enabled = payload.get("enabled")
        if not isinstance(enabled, bool):
            raise ValidationError("enabled 必须为布尔值")
        if enabled:
            self._require_armed()
        with self._lock:
            self.memory.ensure_idle()
            self.arm_movel.ensure_idle()
            self.grasp_test.ensure_idle()
            self.scan_sequence.ensure_idle()
            self.placement.ensure_idle()
            state = self.robot.read_state()
            self.audit_event(
                "drag_requested",
                side=side,
                enabled=enabled,
                require_end_button=True,
                state_before=state,
            )
            try:
                self.robot.set_drag(side, enabled)
            except Exception as exc:
                self.audit_event("drag_failed", side=side, enabled=enabled, error=str(exc))
                raise
            self.audit_event(
                "drag_changed",
                side=side,
                enabled=enabled,
                require_end_button=True,
            )
            self._set_drag_sampling(side, enabled)
        return {"side": side, "enabled": enabled}

    def set_chassis_enabled(self, payload: dict[str, Any]) -> dict[str, Any]:
        enabled = payload.get("enabled")
        if not isinstance(enabled, bool):
            raise ValidationError("enabled 必须为布尔值")
        if enabled:
            self._require_armed()
        with self._lock:
            if enabled:
                self._require_armed()
            self.chassis.set_enabled(enabled)
            status = self.chassis.status()
            self.audit_event("chassis_remote_changed", enabled=enabled, status=status)
        return status

    def set_chassis_obstacle_avoidance(self, payload: dict[str, Any]) -> dict[str, Any]:
        if set(payload) != {"enabled"} or type(payload["enabled"]) is not bool:
            raise ValidationError("遥控避障只接受 enabled 布尔值")
        self._require_armed()
        with self._lock:
            self._require_armed()
            enabled = payload["enabled"]
            self.audit_event("chassis_obstacle_avoidance_requested", enabled=enabled)
            try:
                self.chassis.set_obstacle_avoidance(enabled)
            except Exception as exc:
                self.audit_event("chassis_obstacle_avoidance_failed", enabled=enabled, error=str(exc))
                raise
            status = self.chassis.status()
            self.audit_event("chassis_obstacle_avoidance_changed", enabled=enabled, status=status)
            return status

    def release_chassis_emergency_stop(self) -> dict[str, Any]:
        self._require_armed()
        with self._lock:
            self._require_armed()
            result = self.chassis.release_emergency_stop()
            self.audit_event("chassis_emergency_stop_released", result=result)
        return result

    def chassis_command(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._require_armed()
        values = _finite_values(
            [payload.get("linear_x"), payload.get("linear_y"), payload.get("angular_z")],
            3,
            "底盘速度",
        )
        chassis = self.config["chassis"]
        if abs(values[0]) > float(chassis["max_linear_m_s"]) or abs(values[1]) > float(
            chassis["max_linear_m_s"]
        ):
            raise ValidationError("底盘线速度超过配置上限")
        if abs(values[2]) > float(chassis["max_angular_rad_s"]):
            raise ValidationError("底盘角速度超过配置上限")
        with self._lock:
            self._require_armed()
            self.chassis.command(*values)
            self.audit_event(
                "chassis_velocity_command",
                linear_x=values[0],
                linear_y=values[1],
                angular_z=values[2],
            )
        return {"accepted": True, "lease_seconds": chassis["lease_seconds"]}

    def chassis_stop(self) -> dict[str, Any]:
        self.chassis.stop()
        self.audit_event("chassis_stop")
        return {"stopped": True}

    def close(self) -> None:
        self.audit_event("service_stopping")
        self.placement.close()
        self.scan_sequence.close()
        self.grasp_test.close()
        self.arm_movel.close()
        self.memory.close()
        self._sample_stop.set()
        with self._sample_condition:
            self._sample_condition.notify_all()
        if self._sample_thread is not None:
            self._sample_thread.join(timeout=2.0)
        try:
            self.camera.close()
        finally:
            try:
                self.chassis.close()
            finally:
                try:
                    self.robot.close()
                finally:
                    self.audit_event("service_stopped")
                    self.audit.close()
