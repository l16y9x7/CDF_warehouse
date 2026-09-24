"""TX-S2 NavAdapter — wires server / slam / move ROS2 clients.

Keeps all vendor details behind NavAdapter; /retail_nav/v1 contract unchanged.
"""

from __future__ import annotations

import math
import threading
import time
import uuid
from enum import Enum
from pathlib import Path
from typing import Any, Optional

from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from move.action import MoveRelative
from move.srv import SetControlMode, SetParkingMode
from server.action import StopMapping as ServerStopMapping
from server.msg import State, TargetInfoWithId, TargetInfoWithPose
from server.srv import (
    CancelAllTask,
    CancelTask,
    CreateTaskById,
    CreateTaskByPose,
    GetMapInfo,
    StartMapping as ServerStartMapping,
    SwitchMap,
)
from slam.msg import LocationData
from slam.srv import Relocation, SetMap

from navigation.tianji_runtime.adapters.base import (
    AdapterResult,
    HealthInfo,
    NavGoal,
    Pose2D,
)
from navigation.tianji_runtime.constants import ErrorCode, ResultCode
from navigation.tianji_runtime.station_align import AlignConfig, Pose2D as AlignPose, run_align
from navigation.tianji_runtime.station_nudge import run_manual_body_moves
from navigation.tianji_runtime.stations import Station, StationCatalog, align_enabled_for_station


class _NavMode(str, Enum):
    NONE = "none"
    STATION = "station"
    POSE = "pose"
    RELATIVE = "relative"


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_to_quat(yaw: float) -> Quaternion:
    half = 0.5 * float(yaw)
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def _pose_yaw(pose: Pose) -> float:
    """Vendor LocationData / State: orientation.z is yaw (rad).

    Docs mark w unused, but the chassis often fills w=1. Treating that as a
    quaternion turns yaw≈±π into ~161° and blocks GoToStation arrival.
    """
    return _wrap_angle(float(pose.orientation.z))


def _make_pose(x: float, y: float, yaw: float) -> Pose:
    """Standard quaternion pose (used by slam/Relocation / LocationData style)."""
    return Pose(
        position=Point(x=float(x), y=float(y), z=0.0),
        orientation=_yaw_to_quat(yaw),
    )


def _make_vendor_task_pose(x: float, y: float, yaw: float) -> Pose:
    """Pose for server/CreateTaskByPose: orientation.w = yaw (rad), per 接口文档26.0."""
    return Pose(
        position=Point(x=float(x), y=float(y), z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=float(yaw)),
    )


_WS_IDLE = 1
_WS_TASK = 2
_WS_EXCEPTION = 3
_WS_SOFT_MANUAL = 14
_WS_RELATIVE = 17


class _S2AlignHardware:
    """AlignHardware over the S2 adapter. MoveRelative w is yaw radians."""

    def __init__(self, adapter: "S2NavAdapter") -> None:
        self._a = adapter

    def get_pose(self) -> AlignPose:
        p = self._a.get_pose()
        return AlignPose(
            x=float(p.x), y=float(p.y), yaw=float(p.yaw), confidence=int(p.confidence)
        )

    def check_safe(self) -> tuple[bool, str]:
        st = self._a._state
        if st is None:
            return False, "no /server/State; refuse to align blind"
        if st.is_scram:
            return False, "scram"
        if st.has_task or int(st.work_status) == _WS_TASK:
            return False, f"vendor still has a task (work_status={st.work_status})"
        if int(st.work_status) == _WS_EXCEPTION:
            return False, "vendor work_status=exception"
        if int(st.work_status) == _WS_RELATIVE:
            return False, "vendor already relative-moving"
        if int(st.work_status) not in (_WS_IDLE, _WS_SOFT_MANUAL):
            return False, f"vendor work_status={st.work_status} not idle/manual"
        return True, ""

    def set_control_mode(self, mode: int) -> None:
        req = SetControlMode.Request()
        req.mode = int(mode)
        resp = self._a._call_service(
            self._a._cli_control_mode,
            req,
            label=f"SetControlMode({mode})",
            timeout_sec=8.0,
        )
        if int(resp.error_code) != 0:
            raise RuntimeError(
                f"SetControlMode({mode}) error_code={resp.error_code} {resp.error_msg}"
            )

    def set_parking(self, enable: bool) -> None:
        req = SetParkingMode.Request()
        req.enable = bool(enable)
        resp = self._a._call_service(
            self._a._cli_parking,
            req,
            label=f"SetParkingMode({enable})",
            timeout_sec=8.0,
        )
        if int(resp.error_code) != 0:
            raise RuntimeError(
                f"SetParkingMode({enable}) error_code={resp.error_code} {resp.error_msg}"
            )

    def wait_soft_auto(self, want_auto: bool, timeout_sec: float) -> None:
        deadline = time.monotonic() + max(float(timeout_sec), 0.0)
        while time.monotonic() < deadline:
            st = self._a._state
            if st is not None:
                if want_auto:
                    if bool(st.is_soft_auto) and int(st.work_status) == _WS_IDLE:
                        return
                elif (not bool(st.is_soft_auto)) or int(st.work_status) == _WS_SOFT_MANUAL:
                    return
            time.sleep(0.05)
        raise TimeoutError(f"timeout waiting is_soft_auto={want_auto}")

    def settle(self, sec: float) -> None:
        time.sleep(max(float(sec), 0.0))

    def move_relative(
        self, dx: float, dy: float, dyaw: float, timeout_sec: float
    ) -> None:
        last_err: Optional[Exception] = None
        for attempt in range(2):
            try:
                self._move_relative_once(dx, dy, dyaw, timeout_sec)
                return
            except RuntimeError as exc:
                last_err = exc
                if "rejected" not in str(exc).lower() or attempt == 1:
                    raise
                self._a._log.warn(f"MoveRelative rejected, retry ({exc})")
                time.sleep(0.8)
        raise last_err or RuntimeError("MoveRelative failed")

    def _move_relative_once(
        self, dx: float, dy: float, dyaw: float, timeout_sec: float
    ) -> None:
        act = self._a._act_move_rel
        if not act.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("move/MoveRelativeCmd not available")
        goal = MoveRelative.Goal()
        goal.relative_pose = Pose(
            position=Point(x=float(dx), y=float(dy), z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=float(dyaw)),
        )
        send_future = act.send_goal_async(goal)
        handle = self._a._wait_future(
            send_future, min(timeout_sec, 8.0), "MoveRelative send"
        )
        if handle is None or not handle.accepted:
            raise RuntimeError("MoveRelative rejected")
        wrapped = self._a._wait_future(
            handle.get_result_async(), timeout_sec, "MoveRelative result"
        )
        if wrapped is None:
            raise RuntimeError("MoveRelative no result")
        res = wrapped.result
        if int(res.error_code) != 0:
            raise RuntimeError(
                f"MoveRelative error_code={res.error_code} {res.error_msg}"
            )


class S2NavAdapter:
    """TX-S2 ROS2 adapter implementing NavAdapter.

    Mapping (per handoff doc / 接口文档 26.0):
      start_mapping      -> server/StartMapping (map_id=0)
      tick_stop_mapping  -> server/StopMapping Action
      load_map           -> server/SwitchMap and/or slam/SetMap
      relocate           -> slam/Relocation
      start_nav(station) -> CreateTaskByPose (default) or CreateTaskById
      start_nav(pose)    -> server/CreateTaskByPose
      start_nav(relative)-> move/MoveRelativeCmd
      get_pose           -> slam/LocationData
      health / busy      -> server/State (+ LocationData fallback)
      cancel_nav         -> server/CancelTask / CancelAllTask

    Field note: HMI「导航节点」坐标写入 retail_stations.yaml；本机 CreateTaskById
    对导航点会报 Station Not Exist，故 station_nav_mode 默认 pose。
    """

    def __init__(
        self,
        node: Node,
        *,
        catalog: Optional[StationCatalog] = None,
        service_timeout_sec: float = 60.0,
        map_dir: str = "",
        arrive_stable_ticks: int = 2,
        station_nav_mode: str = "pose",
        navigation_mode: int = 0,
        require_server_state: bool = False,
        align_enable: bool = False,
        align_cfg: Optional[AlignConfig] = None,
        align_fail_nav: bool = False,
    ) -> None:
        self._node = node
        self._catalog = catalog
        self._service_timeout_sec = max(service_timeout_sec, 0.5)
        self._map_dir = map_dir
        self._arrive_stable_ticks = max(arrive_stable_ticks, 1)
        mode = str(station_nav_mode or "pose").strip().lower()
        if mode not in ("pose", "id", "id_then_pose"):
            mode = "pose"
        self._station_nav_mode = mode
        self._navigation_mode = 1 if int(navigation_mode) == 1 else 0
        self._require_server_state = bool(require_server_state)
        self._align_enable = bool(align_enable)
        self._align_cfg = align_cfg or AlignConfig()
        self._align_fail_nav = bool(align_fail_nav)
        self._align_done = False
        self._align_wait_t0 = 0.0
        self._log = node.get_logger()
        self._log.info(
            f"station align enable={self._align_enable} fail_nav={self._align_fail_nav} "
            f"xy_tol={self._align_cfg.xy_tol_m*100:.0f}cm "
            f"xy_max={self._align_cfg.xy_max_m*100:.0f}cm"
        )
        # Service/action responses must not share the node's default mutually
        # exclusive group with pose/status timers; otherwise a lock wait inside
        # a timer callback can prevent CreateTaskById replies from being handled
        # and surface as a multi-second "vendor timeout".
        self._client_cb_group = ReentrantCallbackGroup()
        self._sub_cb_group = ReentrantCallbackGroup()

        self._pose = Pose2D()
        self._state: Optional[State] = None
        self._holding = False
        self._paused = False
        self._mapping = False
        self._map_id = ""
        self._map_name = catalog.map_name if catalog else ""
        self._localized = False

        self._nav_mode = _NavMode.NONE
        self._goal: Optional[NavGoal] = None
        self._task_id = 0
        self._nav_started_at = 0.0
        self._arrive_hits = 0
        self._nav_progress = 0.0

        self._relative_goal_handle = None
        self._relative_result_future = None
        self._relative_progress = 0.0

        self._stop_goal_handle = None
        self._stop_result_future = None
        self._stop_progress = 0.0
        self._stop_started = False
        self._stop_done = False

        self._lock = threading.RLock()

        qos_state = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_pose = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._state_sub = node.create_subscription(
            State,
            "server/State",
            self._on_state,
            qos_state,
            callback_group=self._sub_cb_group,
        )
        self._loc_sub = node.create_subscription(
            LocationData,
            "slam/LocationData",
            self._on_location,
            qos_pose,
            callback_group=self._sub_cb_group,
        )

        self._cli_start_mapping = node.create_client(
            ServerStartMapping,
            "server/StartMapping",
            callback_group=self._client_cb_group,
        )
        self._cli_create_by_id = node.create_client(
            CreateTaskById,
            "server/CreateTaskById",
            callback_group=self._client_cb_group,
        )
        self._cli_create_by_pose = node.create_client(
            CreateTaskByPose,
            "server/CreateTaskByPose",
            callback_group=self._client_cb_group,
        )
        self._cli_cancel_task = node.create_client(
            CancelTask, "server/CancelTask", callback_group=self._client_cb_group
        )
        self._cli_cancel_all = node.create_client(
            CancelAllTask,
            "server/CancelAllTask",
            callback_group=self._client_cb_group,
        )
        self._cli_switch_map = node.create_client(
            SwitchMap, "server/SwitchMap", callback_group=self._client_cb_group
        )
        self._cli_get_map_info = node.create_client(
            GetMapInfo, "server/GetMapInfo", callback_group=self._client_cb_group
        )
        self._cli_set_map = node.create_client(
            SetMap, "slam/SetMap", callback_group=self._client_cb_group
        )
        self._cli_reloc = node.create_client(
            Relocation, "slam/Relocation", callback_group=self._client_cb_group
        )

        self._act_stop_mapping = ActionClient(
            node,
            ServerStopMapping,
            "server/StopMapping",
            callback_group=self._client_cb_group,
        )
        self._act_move_rel = ActionClient(
            node,
            MoveRelative,
            "move/MoveRelativeCmd",
            callback_group=self._client_cb_group,
        )
        self._cli_control_mode = node.create_client(
            SetControlMode,
            "move/SetControlMode",
            callback_group=self._client_cb_group,
        )
        self._cli_parking = node.create_client(
            SetParkingMode,
            "move/SetParkingMode",
            callback_group=self._client_cb_group,
        )

    # ------------------------------------------------------------------ helpers
    def _call_service(
        self, client: Any, request: Any, *, label: str, timeout_sec: float | None = None
    ) -> Any:
        """Non-blocking-friendly sync call (Event wait; safe with MultiThreadedExecutor)."""
        timeout = float(
            self._service_timeout_sec if timeout_sec is None else timeout_sec
        )
        if not client.wait_for_service(timeout_sec=min(timeout, 5.0)):
            raise TimeoutError(f"{label} not available")
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=timeout):
            raise TimeoutError(f"{label} timed out")
        result = future.result()
        if result is None:
            raise RuntimeError(f"{label} returned no result")
        return result

    def _wait_future(self, future: Any, timeout_sec: float, label: str) -> Any:
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=max(float(timeout_sec), 0.1)):
            raise TimeoutError(f"{label} timed out")
        return future.result()

    def _vendor_fail(self, code: int, msg: str) -> AdapterResult:
        return AdapterResult(
            error_code=ErrorCode.VENDOR,
            error_msg=f"vendor[{code}]: {msg}" if msg else f"vendor[{code}]",
            result_code=ResultCode.ERROR.value,
            pose=self.get_pose(),
        )

    def _on_state(self, msg: State) -> None:
        with self._lock:
            self._state = msg
            if msg.map_name:
                self._map_name = msg.map_name
            if msg.map_id:
                self._map_id = str(msg.map_id)
            if self._pose.confidence <= 0:
                self._pose = Pose2D(
                    x=float(msg.global_pose.position.x),
                    y=float(msg.global_pose.position.y),
                    yaw=_pose_yaw(msg.global_pose),
                    confidence=50 if not msg.is_scram else 0,
                    map_id=self._map_id,
                )

    def _on_location(self, msg: LocationData) -> None:
        with self._lock:
            self._pose = Pose2D(
                x=float(msg.pose.position.x),
                y=float(msg.pose.position.y),
                yaw=_pose_yaw(msg.pose),
                confidence=int(msg.confidence),
                map_id=self._map_id,
            )
            if msg.confidence > 0 and not msg.is_relocating:
                self._localized = True

    def _resolve_vendor_target_id(self, goal: NavGoal) -> int:
        if self._catalog and goal.station_id:
            station = self._catalog.get(goal.station_id)
            if station is not None and station.vendor_target_id > 0:
                return int(station.vendor_target_id)
        return 0

    def _map_path_for(self, map_id: str = "", map_name: str = "") -> str:
        name = map_name or map_id or self._map_name or "map"
        if Path(name).is_absolute() or "/" in name or name.endswith(
            (".yaml", ".png", ".pcd")
        ):
            return name
        if self._map_dir:
            return str(Path(self._map_dir) / name)
        return name

    def export_vendor_map_data(self) -> dict[int, bytes]:
        """Fetch the current vendor map in every documented GetMapInfo format.

        The returned bmap is the vendor-native artifact and may contain HMI
        navigation-node edits. Its internal point format is vendor-owned and
        deliberately not parsed or converted to retail_stations.yaml here.
        """
        data_by_type: dict[int, bytes] = {}
        for map_type in range(4):
            req = GetMapInfo.Request()
            req.map_type = map_type
            try:
                resp = self._call_service(
                    self._cli_get_map_info,
                    req,
                    label=f"server/GetMapInfo(map_type={map_type})",
                )
            except Exception as exc:  # noqa: BLE001
                if map_type == 0:
                    raise RuntimeError(f"bmap export unavailable: {exc}") from exc
                self._log.warn(
                    f"optional map export type={map_type} unavailable: {exc}"
                )
                continue
            if resp.error_code != 0:
                if map_type == 0:
                    raise RuntimeError(
                        f"bmap export failed [{resp.error_code}]: {resp.error_msg}"
                    )
                self._log.warn(
                    f"optional map export type={map_type} failed "
                    f"[{resp.error_code}]: {resp.error_msg}"
                )
                continue
            payload = bytes(resp.data)
            if not payload:
                if map_type == 0:
                    raise RuntimeError("bmap export returned empty data")
                self._log.warn(f"optional map export type={map_type} returned no data")
                continue
            data_by_type[map_type] = payload
        return data_by_type

    # ------------------------------------------------------------------ health / pose
    def get_health(self) -> HealthInfo:
        with self._lock:
            state = self._state
            has_pose = self._pose.confidence > 0 or self._localized
        mapping_ready = self._cli_start_mapping.service_is_ready() and (
            self._act_stop_mapping.server_is_ready()
        )
        navigation_ready = (
            self._cli_create_by_id.service_is_ready()
            or self._cli_create_by_pose.service_is_ready()
        )
        localization_ready = self._cli_reloc.service_is_ready()
        motion_ready = (not self._holding) and (
            self._act_move_rel.server_is_ready() or navigation_ready
        )
        detail_parts = []
        if state is None:
            detail_parts.append("no server/State yet")
            # 本机 server 常不发 State；可用定位+导航服务判定 runtime
            if self._require_server_state:
                runtime = False
            else:
                runtime = bool(has_pose and (navigation_ready or localization_ready))
                if runtime:
                    detail_parts.append("runtime via LocationData fallback")
        else:
            runtime = not bool(state.is_scram)
            if state.is_scram:
                detail_parts.append("scram")
            elif not state.is_normal:
                detail_parts.append("not normal")
        if self._holding:
            detail_parts.append("holding")
        return HealthInfo(
            runtime_ready=runtime,
            mapping_ready=mapping_ready,
            navigation_ready=navigation_ready,
            localization_ready=localization_ready,
            motion_ready=motion_ready,
            detail="; ".join(detail_parts) or "s2 adapter",
        )

    def get_pose(self) -> Pose2D:
        with self._lock:
            return Pose2D(
                x=self._pose.x,
                y=self._pose.y,
                yaw=self._pose.yaw,
                confidence=self._pose.confidence,
                map_id=self._pose.map_id or self._map_id,
            )

    # ------------------------------------------------------------------ mapping
    def start_mapping(self, map_name: str = "") -> AdapterResult:
        if self._nav_mode != _NavMode.NONE:
            return AdapterResult(
                error_code=ErrorCode.INVALID_STATE,
                error_msg="cannot map while navigating",
            )
        req = ServerStartMapping.Request()
        req.map_id = 0
        try:
            resp = self._call_service(
                self._cli_start_mapping, req, label="server/StartMapping"
            )
        except Exception as exc:  # noqa: BLE001
            return AdapterResult(error_code=ErrorCode.VENDOR, error_msg=str(exc))
        if resp.error_code != 0:
            return self._vendor_fail(resp.error_code, resp.error_msg)
        self._mapping = True
        self._localized = False
        self._map_name = map_name or self._map_name or f"map_{int(time.time())}"
        self._reset_stop_mapping()
        return AdapterResult(session_id=str(uuid.uuid4()), map_name=self._map_name)

    def _reset_stop_mapping(self) -> None:
        self._stop_goal_handle = None
        self._stop_result_future = None
        self._stop_progress = 0.0
        self._stop_started = False
        self._stop_done = False

    def _on_stop_feedback(self, feedback_msg: Any) -> None:
        self._stop_progress = float(feedback_msg.feedback.progress)

    def tick_stop_mapping(self, map_name: str = "", dt: float = 0.1) -> AdapterResult:
        del dt
        name = map_name or self._map_name
        if not self._stop_started:
            if not self._act_stop_mapping.wait_for_server(
                timeout_sec=self._service_timeout_sec
            ):
                return AdapterResult(
                    error_code=ErrorCode.NOT_READY,
                    error_msg="server/StopMapping not available",
                    result_code=ResultCode.ERROR.value,
                )
            goal = ServerStopMapping.Goal()
            send_future = self._act_stop_mapping.send_goal_async(
                goal, feedback_callback=self._on_stop_feedback
            )
            event = threading.Event()
            send_future.add_done_callback(lambda _f: event.set())
            if not event.wait(timeout=self._service_timeout_sec):
                return AdapterResult(
                    error_code=ErrorCode.TIMEOUT,
                    error_msg="StopMapping accept timeout",
                    result_code=ResultCode.TIMEOUT.value,
                )
            handle = send_future.result()
            if handle is None or not handle.accepted:
                return AdapterResult(
                    error_code=ErrorCode.VENDOR,
                    error_msg="StopMapping rejected",
                    result_code=ResultCode.ERROR.value,
                )
            self._stop_goal_handle = handle
            self._stop_result_future = handle.get_result_async()
            self._stop_started = True
            return AdapterResult(progress=self._stop_progress, result_code="")

        if self._stop_result_future is not None and self._stop_result_future.done():
            wrapped = self._stop_result_future.result()
            result = wrapped.result
            self._stop_done = True
            self._mapping = False
            if result.error_code != 0:
                return AdapterResult(
                    error_code=ErrorCode.VENDOR,
                    error_msg=result.error_msg
                    or f"StopMapping failed ({result.error_code})",
                    result_code=ResultCode.ERROR.value,
                    progress=1.0,
                )
            with self._lock:
                state = self._state
                if state is not None and state.map_id:
                    self._map_id = str(state.map_id)
                if state is not None and state.map_name:
                    self._map_name = state.map_name
                else:
                    self._map_name = name
            return AdapterResult(
                progress=1.0,
                result_code=ResultCode.DONE.value,
                map_id=self._map_id,
                map_name=self._map_name,
            )

        return AdapterResult(
            progress=min(0.99, max(0.0, self._stop_progress)), result_code=""
        )

    def cancel_mapping(self, reason: str = "") -> AdapterResult:
        if self._stop_goal_handle is not None and not self._stop_done:
            try:
                self._stop_goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        self._mapping = False
        self._reset_stop_mapping()
        return AdapterResult(error_msg=reason)

    # ------------------------------------------------------------------ map / reloc
    def load_map(self, map_id: str = "", map_name: str = "") -> AdapterResult:
        last_err = ""
        if map_id and str(map_id).isdigit():
            req = SwitchMap.Request()
            req.map_id = int(map_id)
            try:
                resp = self._call_service(
                    self._cli_switch_map, req, label="server/SwitchMap"
                )
                if resp.error_code == 0:
                    self._map_id = str(map_id)
                    if map_name:
                        self._map_name = map_name
                    return AdapterResult(map_id=self._map_id, map_name=self._map_name)
                last_err = resp.error_msg or f"SwitchMap[{resp.error_code}]"
            except Exception as exc:  # noqa: BLE001
                last_err = str(exc)

        path = self._map_path_for(map_id=map_id, map_name=map_name)
        req = SetMap.Request()
        req.path = path
        try:
            resp = self._call_service(self._cli_set_map, req, label="slam/SetMap")
        except Exception as exc:  # noqa: BLE001
            return AdapterResult(
                error_code=ErrorCode.VENDOR,
                error_msg=last_err or str(exc),
            )
        if resp.error_code != 0:
            return self._vendor_fail(
                resp.error_code, resp.error_msg or last_err or "SetMap failed"
            )
        if map_id:
            self._map_id = map_id
        if map_name:
            self._map_name = map_name
        elif path:
            self._map_name = Path(path).stem
        return AdapterResult(map_id=self._map_id, map_name=self._map_name)

    def relocate(
        self,
        *,
        station: Optional[Station] = None,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
    ) -> AdapterResult:
        if station is not None:
            x, y, yaw = station.x, station.y, station.yaw
        req = Relocation.Request()
        req.pose = _make_pose(x, y, yaw)
        try:
            resp = self._call_service(self._cli_reloc, req, label="slam/Relocation")
        except Exception as exc:  # noqa: BLE001
            return AdapterResult(error_code=ErrorCode.VENDOR, error_msg=str(exc))
        if resp.error_code != 0:
            return self._vendor_fail(resp.error_code, resp.error_msg)
        self._localized = True
        with self._lock:
            self._pose = Pose2D(
                x=x, y=y, yaw=yaw, confidence=100, map_id=self._map_id
            )
        return AdapterResult(pose=self.get_pose())

    # ------------------------------------------------------------------ navigation
    def _reset_nav(self) -> None:
        self._nav_mode = _NavMode.NONE
        self._goal = None
        self._task_id = 0
        self._nav_started_at = 0.0
        self._arrive_hits = 0
        self._nav_progress = 0.0
        self._relative_goal_handle = None
        self._relative_result_future = None
        self._relative_progress = 0.0
        self._paused = False
        self._align_done = False
        self._align_wait_t0 = 0.0

    def start_nav(self, goal: NavGoal) -> AdapterResult:
        if self._holding:
            return AdapterResult(
                error_code=ErrorCode.INVALID_STATE,
                error_msg="holding chassis",
            )
        if self._mapping:
            return AdapterResult(
                error_code=ErrorCode.INVALID_STATE,
                error_msg="mapping in progress",
            )

        self._reset_nav()
        self._goal = goal
        self._nav_started_at = time.monotonic()

        if goal.relative:
            return self._start_relative(goal)

        # GoToStation: 默认按 YAML 导航点坐标走 CreateTaskByPose（适配 HMI 导航节点）
        if goal.station_id:
            return self._start_station(goal)
        return self._start_by_pose(goal)

    def _start_station(self, goal: NavGoal) -> AdapterResult:
        target_id = self._resolve_vendor_target_id(goal)
        mode = self._station_nav_mode
        if mode == "pose":
            self._log.info(
                f"station '{goal.station_id}' -> CreateTaskByPose "
                f"({goal.x:.3f},{goal.y:.3f},{goal.yaw:.3f})"
            )
            return self._start_by_pose(goal)
        if mode == "id":
            if target_id <= 0:
                return AdapterResult(
                    error_code=ErrorCode.UNKNOWN_STATION,
                    error_msg=f"no vendor_target_id for station '{goal.station_id}'",
                )
            return self._start_by_id(goal, target_id)
        # id_then_pose
        if target_id > 0:
            result = self._start_by_id(goal, target_id)
            if result.ok:
                return result
            self._log.warn(
                f"CreateTaskById failed ({result.error_msg}); fallback CreateTaskByPose"
            )
            self._reset_nav()
            self._goal = goal
            self._nav_started_at = time.monotonic()
        return self._start_by_pose(goal)

    def _start_by_id(self, goal: NavGoal, target_id: int) -> AdapterResult:
        target = TargetInfoWithId()
        target.target_id = int(target_id)
        target.action_type = 0
        target.action_param = 0.0
        req = CreateTaskById.Request()
        req.navigation_mode = int(self._navigation_mode)
        req.max_speed = float(goal.max_speed or 0.5)
        req.max_loop_times = 1
        req.target_list = [target]
        try:
            resp = self._call_service(
                self._cli_create_by_id, req, label="server/CreateTaskById"
            )
        except Exception as exc:  # noqa: BLE001
            self._reset_nav()
            return AdapterResult(error_code=ErrorCode.VENDOR, error_msg=str(exc))
        if resp.error_code != 0:
            self._reset_nav()
            return self._vendor_fail(resp.error_code, resp.error_msg)
        self._nav_mode = _NavMode.STATION
        self._task_id = int(resp.task_id)
        self._log.info(
            f"CreateTaskById ok task_id={self._task_id} target_id={target_id} "
            f"nav_mode={self._navigation_mode}"
        )
        return AdapterResult(session_id=str(self._task_id))

    def _start_by_pose(self, goal: NavGoal) -> AdapterResult:
        target = TargetInfoWithPose()
        target.target_pose = _make_vendor_task_pose(goal.x, goal.y, goal.yaw)
        target.action_type = 0
        target.action_param = 0.0
        req = CreateTaskByPose.Request()
        req.max_speed = float(goal.max_speed or 0.5)
        req.max_loop_times = 1
        req.target_list = [target]
        try:
            resp = self._call_service(
                self._cli_create_by_pose, req, label="server/CreateTaskByPose"
            )
        except Exception as exc:  # noqa: BLE001
            self._reset_nav()
            return AdapterResult(error_code=ErrorCode.VENDOR, error_msg=str(exc))
        if resp.error_code != 0:
            self._reset_nav()
            return self._vendor_fail(resp.error_code, resp.error_msg)
        self._nav_mode = _NavMode.POSE
        self._task_id = int(resp.task_id)
        self._log.info(
            f"CreateTaskByPose ok task_id={self._task_id} "
            f"pose=({goal.x:.3f},{goal.y:.3f},yaw={goal.yaw:.3f})"
        )
        return AdapterResult(session_id=str(self._task_id))

    def _on_relative_feedback(self, feedback_msg: Any) -> None:
        self._relative_progress = float(feedback_msg.feedback.progress)

    def _start_relative(self, goal: NavGoal) -> AdapterResult:
        if not self._act_move_rel.wait_for_server(timeout_sec=self._service_timeout_sec):
            self._reset_nav()
            return AdapterResult(
                error_code=ErrorCode.NOT_READY,
                error_msg="move/MoveRelativeCmd not available",
            )
        msg = MoveRelative.Goal()
        msg.relative_pose = _make_pose(goal.x, goal.y, goal.yaw)
        send_future = self._act_move_rel.send_goal_async(
            msg, feedback_callback=self._on_relative_feedback
        )
        event = threading.Event()
        send_future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=self._service_timeout_sec):
            self._reset_nav()
            return AdapterResult(
                error_code=ErrorCode.TIMEOUT,
                error_msg="MoveRelative accept timeout",
            )
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self._reset_nav()
            return AdapterResult(
                error_code=ErrorCode.VENDOR,
                error_msg="MoveRelative rejected",
            )
        self._nav_mode = _NavMode.RELATIVE
        self._relative_goal_handle = handle
        self._relative_result_future = handle.get_result_async()
        return AdapterResult()

    def tick_nav(self, dt: float = 0.1) -> AdapterResult:
        del dt
        if self._goal is None or (self._nav_mode == _NavMode.NONE and not self._paused):
            return AdapterResult(
                error_code=ErrorCode.NOT_READY,
                error_msg="no active goal",
                result_code=ResultCode.ERROR.value,
            )
        if self._paused:
            return AdapterResult(progress=self._nav_progress, pose=self.get_pose())

        goal = self._goal
        elapsed = time.monotonic() - self._nav_started_at
        if goal.timeout_sec > 0 and elapsed > goal.timeout_sec:
            self.cancel_nav("navigation timeout")
            return AdapterResult(
                error_code=ErrorCode.TIMEOUT,
                error_msg="navigation timeout",
                result_code=ResultCode.TIMEOUT.value,
                pose=self.get_pose(),
            )

        with self._lock:
            state = self._state

        if state is not None:
            if state.is_scram:
                self.cancel_nav("scram")
                return AdapterResult(
                    error_code=ErrorCode.SAFETY,
                    error_msg="scram",
                    result_code=ResultCode.ERROR.value,
                    pose=self.get_pose(),
                )
            if state.work_status == 3:
                self.cancel_nav("vendor work_status=exception")
                return AdapterResult(
                    error_code=ErrorCode.VENDOR,
                    error_msg="vendor work_status exception",
                    result_code=ResultCode.BLOCKED.value,
                    pose=self.get_pose(),
                )

        if self._nav_mode == _NavMode.RELATIVE:
            return self._tick_relative()
        return self._tick_task_nav(state)

    def _tick_relative(self) -> AdapterResult:
        if (
            self._relative_result_future is not None
            and self._relative_result_future.done()
        ):
            wrapped = self._relative_result_future.result()
            result = wrapped.result
            if result.error_code != 0:
                self._reset_nav()
                return AdapterResult(
                    error_code=ErrorCode.VENDOR,
                    error_msg=result.error_msg
                    or f"MoveRelative[{result.error_code}]",
                    result_code=ResultCode.ERROR.value,
                    pose=self.get_pose(),
                )
            self._reset_nav()
            return AdapterResult(
                progress=1.0,
                result_code=ResultCode.ARRIVED.value,
                pose=self.get_pose(),
            )
        self._nav_progress = min(0.99, max(0.0, self._relative_progress))
        return AdapterResult(
            progress=self._nav_progress, pose=self.get_pose(), result_code=""
        )

    def _vendor_idle_for_align(self, state: Optional[State]) -> bool:
        if state is None:
            return False
        if state.has_task or int(state.work_status) == _WS_TASK:
            return False
        if int(state.work_status) == _WS_RELATIVE:
            return False
        return int(state.work_status) in (_WS_IDLE, _WS_SOFT_MANUAL)

    def _align_enabled_for(self, goal: NavGoal) -> bool:
        station = None
        if self._catalog and goal.station_id:
            station = self._catalog.get(goal.station_id)
        enabled = align_enabled_for_station(self._align_enable, station)
        if self._align_enable and not enabled:
            self._log.info(
                f"station align skipped station={goal.station_id or '-'}"
            )
        return enabled

    def _run_station_align(self, goal: NavGoal):
        target = AlignPose(x=float(goal.x), y=float(goal.y), yaw=float(goal.yaw))
        self._log.info(
            f"station align start target=({goal.x:.3f},{goal.y:.3f},{goal.yaw:.3f}) "
            f"station={goal.station_id or '-'}"
        )
        result = run_align(
            _S2AlignHardware(self),
            target,
            cfg=self._align_cfg,
            log=lambda msg: self._log.info(str(msg)),
        )
        self._log.info(
            f"station align {result.status}: {result.message} "
            f"moves={result.moves} restored_auto={result.restored_auto} "
            f"restored_parking={result.restored_parking}"
        )
        return result

    def run_body_moves(self, steps, timeout_sec: float = 12.0) -> AdapterResult:
        cfg = AlignConfig(
            xy_tol_m=self._align_cfg.xy_tol_m,
            yaw_tol_rad=self._align_cfg.yaw_tol_rad,
            xy_max_m=self._align_cfg.xy_max_m,
            yaw_max_rad=self._align_cfg.yaw_max_rad,
            min_confidence=self._align_cfg.min_confidence,
            translate_timeout_sec=max(float(timeout_sec), 1.0),
            rotate_timeout_sec=self._align_cfg.rotate_timeout_sec,
            mode_wait_sec=self._align_cfg.mode_wait_sec,
            unpark_settle_sec=self._align_cfg.unpark_settle_sec,
            restore_parking=True,
            restore_auto=True,
        )
        result = run_manual_body_moves(
            _S2AlignHardware(self),
            list(steps or ()),
            cfg=cfg,
            log=lambda msg: self._log.info(str(msg)),
        )
        if result.ok:
            return AdapterResult(pose=self.get_pose())
        code = ErrorCode.VENDOR
        if result.status == "blocked":
            code = (
                ErrorCode.SAFETY
                if "scram" in str(result.message).lower()
                else ErrorCode.NOT_READY
            )
        elif result.status == "rejected":
            code = ErrorCode.UNREACHABLE
        return AdapterResult(error_code=code, error_msg=result.message, pose=self.get_pose())

    def align_to_station(self, station) -> AdapterResult:
        goal = NavGoal(
            station_id=getattr(station, "station_id", ""),
            x=float(station.x),
            y=float(station.y),
            yaw=float(station.yaw),
        )
        result = self._run_station_align(goal)
        if result.ok:
            return AdapterResult(pose=self.get_pose())
        code = ErrorCode.VENDOR
        if result.status == "blocked":
            code = (
                ErrorCode.SAFETY
                if "scram" in str(result.message).lower()
                else ErrorCode.NOT_READY
            )
        elif result.status == "rejected":
            code = ErrorCode.UNREACHABLE
        return AdapterResult(error_code=code, error_msg=result.message, pose=self.get_pose())

    def _tick_task_nav(self, state: Optional[State]) -> AdapterResult:
        pose = self.get_pose()
        goal = self._goal
        assert goal is not None

        arrived_flag = False
        if state is not None:
            if state.has_arrived_target:
                arrived_flag = True
            elif (
                self._task_id
                and (not state.has_task)
                and state.work_status in (1, 5)
                and time.monotonic() - self._nav_started_at > 0.5
                # Vendor often clears task_id to 0 as soon as the task ends.
                and int(state.task_id) in (0, int(self._task_id))
            ):
                dist = math.hypot(goal.x - pose.x, goal.y - pose.y)
                if dist <= max(goal.arrive_tol_m * 3.0, 0.25):
                    arrived_flag = True

        dist = math.hypot(goal.x - pose.x, goal.y - pose.y)
        dyaw = abs(_wrap_angle(goal.yaw - pose.yaw))
        geo_ok = dist <= goal.arrive_tol_m and dyaw <= goal.yaw_tol_rad
        if arrived_flag or geo_ok:
            self._arrive_hits += 1
        else:
            self._arrive_hits = 0

        if self._arrive_hits >= self._arrive_stable_ticks:
            if self._align_enabled_for(goal) and not self._align_done:
                if not self._vendor_idle_for_align(state):
                    if self._align_wait_t0 <= 0.0:
                        self._align_wait_t0 = time.monotonic()
                    if time.monotonic() - self._align_wait_t0 < 3.0:
                        self._nav_progress = max(self._nav_progress, 0.97)
                        return AdapterResult(
                            progress=self._nav_progress, pose=pose, result_code=""
                        )
                    self._log.warn(
                        "vendor not idle within 3s after arrive; skip station align"
                    )
                    self._align_done = True
                else:
                    try:
                        align_res = self._run_station_align(goal)
                    except Exception as exc:  # noqa: BLE001
                        self._log.error(f"station align raised: {exc}")
                        align_res = None
                    self._align_done = True
                    if (
                        self._align_fail_nav
                        and align_res is not None
                        and not align_res.ok
                    ):
                        self._reset_nav()
                        return AdapterResult(
                            error_code=ErrorCode.VENDOR,
                            error_msg=(
                                f"station align {align_res.status}: {align_res.message}"
                            ),
                            result_code=ResultCode.ERROR.value,
                            pose=self.get_pose(),
                        )
            self._reset_nav()
            return AdapterResult(
                progress=1.0,
                result_code=ResultCode.ARRIVED.value,
                pose=self.get_pose(),
            )

        approx = max(0.0, 1.0 - dist / max(dist + 1.0, 1e-3))
        self._nav_progress = max(self._nav_progress, min(0.95, approx))
        return AdapterResult(
            progress=self._nav_progress, pose=pose, result_code=""
        )

    def cancel_nav(self, reason: str = "") -> AdapterResult:
        if self._nav_mode == _NavMode.RELATIVE and self._relative_goal_handle is not None:
            try:
                self._relative_goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        elif self._task_id:
            req = CancelTask.Request()
            req.task_id = int(self._task_id)
            try:
                self._call_service(self._cli_cancel_task, req, label="server/CancelTask")
            except Exception:  # noqa: BLE001
                try:
                    self._call_service(
                        self._cli_cancel_all,
                        CancelAllTask.Request(),
                        label="server/CancelAllTask",
                    )
                except Exception:  # noqa: BLE001
                    pass
        else:
            try:
                if self._cli_cancel_all.service_is_ready():
                    self._call_service(
                        self._cli_cancel_all,
                        CancelAllTask.Request(),
                        label="server/CancelAllTask",
                    )
            except Exception:  # noqa: BLE001
                pass
        self._reset_nav()
        return AdapterResult(
            result_code=ResultCode.CANCELLED.value,
            error_msg=reason,
            pose=self.get_pose(),
        )

    def pause_nav(self) -> AdapterResult:
        # No dedicated Pause API on S2; cancel motion and keep goal for resume.
        saved_goal = self._goal
        if self._nav_mode == _NavMode.RELATIVE and self._relative_goal_handle is not None:
            try:
                self._relative_goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        elif self._task_id:
            req = CancelTask.Request()
            req.task_id = int(self._task_id)
            try:
                self._call_service(self._cli_cancel_task, req, label="server/CancelTask")
            except Exception:  # noqa: BLE001
                pass
        self._task_id = 0
        self._relative_goal_handle = None
        self._relative_result_future = None
        self._nav_mode = _NavMode.NONE
        self._goal = saved_goal
        self._paused = True
        return AdapterResult(pose=self.get_pose())

    def resume_nav(self) -> AdapterResult:
        if not self._paused:
            return AdapterResult(pose=self.get_pose())
        goal = self._goal
        self._paused = False
        if goal is None:
            return AdapterResult(pose=self.get_pose())
        return self.start_nav(goal)

    def hold(self, reason: str = "") -> AdapterResult:
        if self._nav_mode != _NavMode.NONE:
            self.cancel_nav(reason or "hold")
        self._holding = True
        self._paused = True
        return AdapterResult(error_msg=reason, pose=self.get_pose())

    def release_hold(self) -> AdapterResult:
        self._holding = False
        self._paused = False
        return AdapterResult(pose=self.get_pose())
