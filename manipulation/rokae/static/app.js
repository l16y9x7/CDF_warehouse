"use strict";

const jointModules = {
  left_arm: { title: "左臂", count: 7, labels: ["J1", "J2", "J3", "J4", "J5", "J6", "J7"] },
  right_arm: { title: "右臂", count: 7, labels: ["J1", "J2", "J3", "J4", "J5", "J6", "J7"] },
  trunk: { title: "躯干", count: 4, labels: ["J1 小腿俯仰", "J2 大腿俯仰", "J3 腰俯仰", "J4 腰偏航"] },
  head: { title: "头部", count: 2, labels: ["O1 头偏航", "O2 头俯仰"] },
};
const poseModules = {
  left_arm: "左臂 Pose",
  right_arm: "右臂 Pose",
  trunk: "躯干 Pose",
};
const poseLabels = ["X mm", "Y mm", "Z mm", "Rx °", "Ry °", "Rz °"];
const armPoseFrames = {
  left_arm: "left_arm_sdk_world",
  right_arm: "right_arm_sdk_world",
};
let serverStatus = null;
let speedDirty = false;
let speedSaving = false;
let speedEditRevision = 0;
let batteryExpiresAt = 0;
let chassisRemoteEnabled = false;
let chassisAvoidanceBusy = false;
let gripperState = null;
let gripperBusy = false;
let suctionBusy = false;
let gripperStatusBusy = false;
let gripperSliderInitialized = false;
const cameraSpecs = {
  head: { label: "头部摄像头" },
  left_wrist: { label: "左手摄像头", colorOnly: true },
  right_wrist: { label: "右手摄像头", colorOnly: true },
};
const cameraEnabled = { head: false, left_wrist: false, right_wrist: false };
const cameraFrameAvailable = { head: false, left_wrist: false, right_wrist: false };
let cameraPreviewRequest = 0;
const cameraBusy = {
  head: { frame: false, record: false, recovery: false },
  left_wrist: { frame: false, record: false, recovery: false },
  right_wrist: { frame: false, record: false, recovery: false },
};
let poseEstimateBusy = false;
let poseSelectedTarget = null;
let poseReprojectBusy = false;
let holdTimer = null;
let toastTimer = null;
let memoryPoints = [];
let memoryBusy = false;
let memoryReady = false;
let memoryCanDelete = false;
let memoryListGeneration = 0;
let memoryExecution = { active: false, phase: "idle", message: "请选择或新增记忆点" };
let memoryPollBusy = false;
let movelExecution = { active: false, phase: "idle", message: "MoveL 待命" };
let movelBusy = false;
let graspExecution = { active: false, phase: "idle", message: "抓取测试待命" };
let graspBusy = false;
let scanExecution = { active: false, phase: "idle", message: "扫码采集待命" };
let scanBusy = false;
let placementExecution = {active:false, phase:'idle', message:'放置待命'};
let placementBusy = false;
let basketSourceId = null;
let basketReferencePoint = null;
let boxBasketSourceId = null;
let boxBasketReferencePoint = null;
let graspSourceId = null;

async function api(path, options = {}) {
  const method = options.method || "GET";
  const headers = options.body ? { "Content-Type": "application/json" } : {};
  if (method === "POST") {
    headers["X-Control-Request-Id"] = globalThis.crypto?.randomUUID?.() || `${Date.now()}-${Math.random().toString(16).slice(2)}`;
    headers["X-Control-Sent-At"] = new Date().toISOString();
  }
  const response = await fetch(path, {
    method,
    headers,
    body: options.body ? JSON.stringify(options.body) : undefined,
    keepalive: Boolean(options.keepalive),
  });
  const payload = await response.json().catch(() => ({ ok: false, error: "服务器返回格式错误" }));
  if (!response.ok || !payload.ok) throw new Error(payload.error || `HTTP ${response.status}`);
  return payload.data;
}

function toast(message, error = false) {
  const element = document.getElementById("toast");
  element.textContent = message;
  element.className = `toast show${error ? " error" : ""}`;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { element.className = "toast"; }, 3600);
}

function renderCards() {
  document.getElementById("jointGrid").innerHTML = Object.entries(jointModules).map(([key, spec]) => `
    <article class="module-card">
      <div class="module-head"><h3>${spec.title}</h3><span class="badge" id="jointState-${key}">未回读</span></div>
      <div class="field-grid">
        ${Array.from({ length: spec.count }, (_, index) => `
          <label>${spec.labels[index]} °
            <input id="joint-${key}-${index}" type="number" step="0.1" value="0">
            <small class="joint-limit" id="jointLimit-${key}-${index}">软限位载入中</small>
          </label>
        `).join("")}
      </div>
      <div class="card-actions"><button class="primary" data-joint-move="${key}">执行关节目标</button></div>
    </article>
  `).join("");

  document.getElementById("poseGrid").innerHTML = Object.entries(poseModules).map(([key, title]) => `
    <article class="module-card">
      <div class="module-head"><h3>${title}</h3><span class="badge" id="poseState-${key}">未回读</span></div>
      ${key === "trunk" ? "" : `<p class="pose-frame-label muted">SDK 世界坐标系 · 原点：${key === "left_arm" ? "左肩" : "右肩"} · Z 向上</p>`}
      <div class="field-grid pose">
        ${poseLabels.map((label, index) => `
          <label>${label}<input id="pose-${key}-${index}" type="number" step="0.1" value="0"></label>
        `).join("")}
        ${key === "trunk" ? "" : `<label>臂角 °<input id="poseElbow-${key}" type="number" step="0.1" min="-180" max="180" value="0" placeholder="留空使用当前臂角"></label>`}
      </div>
      <div class="card-actions">
        <button class="primary" data-pose-move="${key}">MoveJ 运动</button>
        ${key === "trunk" ? "" : `<button data-movel="${key}" data-mode="plain">MoveL 运动</button>
        <button class="primary" data-movel="${key}" data-mode="protected">躯干保护运动</button>
        <button class="danger" data-movel-stop="${key}" disabled>停止 MoveL</button>`}
      </div>
      ${key === "trunk" ? "" : `<p class="muted">保护运动仅筛选终点肘部，单条 MoveL 直达，不检查途中肘部。臂角为搜索起点，留空使用当前臂角；平面参数读取 torso_guard.json。</p>
      <p class="memory-status" id="movelStatus-${key}" role="status">MoveL 待命</p>`}
    </article>
  `).join("");

  document.getElementById("dragGrid").innerHTML = [["left_arm", "左臂"], ["right_arm", "右臂"]].map(([key, title]) => `
    <article class="module-card drag-card">
      <div><h3>${title}拖拽</h3><p id="dragText-${key}">当前：未知（需按住末端按钮）</p></div>
      <div class="drag-actions">
        <button data-drag="${key}" data-enabled="true">开启</button>
        <button data-drag="${key}" data-enabled="false" class="danger">关闭</button>
      </div>
    </article>
  `).join("");
}

function formatNumber(value) {
  return Number(value).toFixed(3).replace(/\.?0+$/, "");
}

function formatRange(minimum, maximum, unit = "°") {
  const signed = value => `${Number(value) > 0 ? "+" : ""}${formatNumber(value)}${unit}`;
  return `${signed(minimum)} ～ ${signed(maximum)}`;
}

function updateSpeedFields(status) {
  const ready = Number.isFinite(status?.rotation_deg_s);
  document.getElementById("saveSpeedButton").disabled = !ready || speedSaving;
  document.getElementById("rotationSpeedInput").disabled = !ready;
  if (!speedDirty && !speedSaving) {
    document.getElementById("speedInput").value = status.speed_mm_s;
    document.getElementById("rotationSpeedInput").value = ready ? status.rotation_deg_s : 6;
  }
  document.getElementById("speedSaveStatus").textContent = !ready
    ? "旋转速度设置尚未加载，请重启网页服务后刷新。"
    : speedSaving ? "正在保存…" : speedDirty ? "有未保存的速度修改" : "";
}

function updateConstraintDisplay(status) {
  const limits = status.limits || {};
  const speedInput = document.getElementById("speedInput");
  speedInput.min = limits.min_speed_mm_s;
  speedInput.max = limits.max_speed_mm_s;
  document.getElementById("speedInputLimit").textContent =
    `范围 ${formatNumber(limits.min_speed_mm_s)}–${formatNumber(limits.max_speed_mm_s)} mm/s`;

  const rotation = document.getElementById("rotationSpeedInput");
  rotation.min = limits.min_rotation_deg_s ?? 0.1;
  rotation.max = limits.max_rotation_deg_s ?? 200;
  document.getElementById("rotationSpeedInputLimit").textContent =
    `范围 ${formatNumber(rotation.min)}–${formatNumber(rotation.max)} °/s`;

  Object.entries(jointModules).forEach(([module, spec]) => {
    const ranges = limits.joint_limits_deg?.[module] || [];
    const softLimitEnabled = limits.joint_soft_limit_enabled?.[module];
    Array.from({ length: spec.count }, (_, index) => {
      const element = document.getElementById(`jointLimit-${module}-${index}`);
      const range = ranges[index];
      const state = softLimitEnabled === false ? "（已关闭）" : "";
      element.textContent = range ? `控制器软限位${state} ${formatRange(range[0], range[1])}` : "软限位读取失败";
    });
  });

  const chassisLimits = status.chassis?.limits || {};
  const linearInput = document.getElementById("linearSpeed");
  const angularInput = document.getElementById("angularSpeed");
  linearInput.min = chassisLimits.min_linear_m_s;
  linearInput.max = chassisLimits.max_linear_m_s;
  angularInput.min = chassisLimits.min_angular_rad_s;
  angularInput.max = chassisLimits.max_angular_rad_s;
  document.getElementById("linearSpeedLimit").textContent = `范围 ${formatNumber(chassisLimits.min_linear_m_s)}–${formatNumber(chassisLimits.max_linear_m_s)}`;
  document.getElementById("angularSpeedLimit").textContent = `范围 ${formatNumber(chassisLimits.min_angular_rad_s)}–${formatNumber(chassisLimits.max_angular_rad_s)}`;
}

function valuesFor(prefix, module, count) {
  return Array.from({ length: count }, (_, index) => Number(document.getElementById(`${prefix}-${module}-${index}`).value));
}

function setValues(prefix, module, values) {
  values.forEach((value, index) => {
    document.getElementById(`${prefix}-${module}-${index}`).value = Number(value).toFixed(3).replace(/\.?0+$/, "");
  });
}

function applyReadback(data) {
  Object.entries(armPoseFrames).forEach(([module, frame]) => {
    if (data.pose_frames?.[module] !== frame) {
      throw new Error("手臂坐标系版本不一致，请重启控制服务后刷新网页并重新回读");
    }
  });
  Object.entries(data.joints_deg).forEach(([module, values]) => {
    setValues("joint", module, values);
    const badge = document.getElementById(`jointState-${module}`);
    badge.textContent = "已回读";
    badge.className = "badge ok";
  });
  Object.entries(data.poses).forEach(([module, values]) => {
    setValues("pose", module, values);
    const badge = document.getElementById(`poseState-${module}`);
    badge.textContent = "已回读";
    badge.className = "badge ok";
  });
  Object.entries(data.arm_elbow_deg || {}).forEach(([module, value]) => {
    const element = document.getElementById(`poseElbow-${module}`);
    if (element) element.value = formatNumber(value);
  });
  Object.entries(data.dragging).forEach(([side, enabled]) => {
    document.getElementById(`dragText-${side}`).textContent = `当前：${enabled ? "拖拽待命，按住末端按钮生效" : "拖拽已关闭"}`;
  });
  document.getElementById("readbackTime").textContent = `最近回读：${new Date().toLocaleTimeString()}`;
}

function updateStatus(status) {
  serverStatus = status;
  placementExecution = status.placement || {active:false, phase:'idle', message:'放置服务尚未加载'};
  scanExecution = status.scan_sequence || { active: false, phase: "idle", message: "扫码采集服务尚未加载" };
  updateBattery(status.chassis?.battery);
  const hardware = status.hardware_enabled;
  const modeBadge = document.getElementById("modeBadge");
  modeBadge.textContent = hardware ? "HARDWARE" : "MOCK";
  modeBadge.className = `badge ${hardware ? "danger" : "ok"}`;
  const logBadge = document.getElementById("logBadge");
  const logs = status.logging;
  logBadge.textContent = logs?.error ? "日志写入异常" : logs?.enabled ? "日志记录中" : "日志未启用";
  logBadge.className = `badge ${logs?.enabled && !logs?.error ? "ok" : "danger"}`;
  logBadge.title = logs?.error || logs?.directory || "需重启网页服务加载全过程日志";
  const banner = document.getElementById("safetyBanner");
  banner.className = `safety-banner${hardware ? "" : " mock"}`;
  document.getElementById("safetyText").textContent = hardware
    ? "真实硬件适配器已启用；控制默认锁定，解锁后操作按钮会直接触发指令。"
    : "当前为 MOCK 模式：不会导入 xCore SDK、不会连接控制器、不会发布真实底盘指令。";
  const armBadge = document.getElementById("armBadge");
  armBadge.textContent = status.armed ? "控制已解锁" : "控制已锁定";
  armBadge.className = `badge ${status.armed ? "danger" : ""}`;
  document.getElementById("armButton").textContent = status.armed ? "立即锁定" : "解锁控制";
  updateGripperControls();
  updateSpeedFields(status);
  document.getElementById("chassisReleaseEmergencyButton").disabled = !status.armed;
  updateConstraintDisplay(status);
  updateCameraStatuses(status.cameras || {});
  chassisRemoteEnabled = Boolean(status.chassis.remote_enabled);
  updateChassisBadge();
  if (status.memory_execution) memoryExecution = status.memory_execution;
  else {
    memoryReady = false;
    memoryExecution = { active: false, phase: "idle", message: "记忆点后端尚未加载，请重启网页控制服务后刷新。" };
  }
  updateMemoryControls();
  if (status.arm_movel) movelExecution = status.arm_movel;
  graspExecution = status.grasp_test || { active: false, phase: "idle", message: "抓取测试后端尚未加载，请重启网页服务后刷新。" };
  updateMoveLControls();
}

function updateGraspControls() {
  const job = graspExecution;
  const busy = graspBusy || graspExecution.active || scanExecution.active || scanBusy || placementExecution.active || placementBusy;
  const selected = document.getElementById('poseTargetSelect').value;
  document.getElementById("graspTestButton").textContent = selected === "tube" ? "软管抓取测试" : "抓取测试";
  const ready = serverStatus?.grasp_test?.version === "grasp-test-v20";
  document.getElementById("graspTestButton").disabled = !ready || busy || !graspSourceId ||
    !serverStatus?.armed || !['bottle', 'box', 'tube'].includes(selected) || selected !== poseSelectedTarget ||
    poseEstimateBusy || poseReprojectBusy || memoryExecution.active || movelExecution.active || movelBusy;
  document.getElementById("graspTestStop").disabled = !graspExecution.active;
  ['poseEstimateButton', 'poseTargetSelect', 'poseTargetSide'].forEach(id => {
    document.getElementById(id).disabled = busy || poseEstimateBusy || poseReprojectBusy;
  });
  if (['bottle', 'box', 'tube', 'basket'].includes(selected)) {
    if (selected !== 'basket') document.getElementById('poseTargetSide').value = selected === 'box' ? 'LEFT' : 'RIGHT';
    document.getElementById('poseTargetSide').disabled = true;
  }
  document.getElementById("poseReprojectButton").disabled = busy || poseEstimateBusy || poseReprojectBusy ||
    !['bottle', 'tube'].includes(selected);
  document.getElementById('poseReprojectLeftButton').disabled = busy || poseEstimateBusy || poseReprojectBusy || selected !== 'box';
  const node = document.getElementById("graspTestStatus");
  const details = [graspExecution.message];
  if (serverStatus?.grasp_test && !ready) details.push("抓取测试版本尚未加载，请重启 mui-control.service 后刷新");
  if (job.box_clearance?.valid) details.push(`本次 D=${formatNumber(job.box_clearance.D_mm)} mm，d=${formatNumber(job.box_clearance.d_mm)} mm${job.module === "left_arm" ? `；下压 Z=${formatNumber(job.descend_height_trunk_mm)} mm，提起 Z=${formatNumber(job.lift_height_trunk_mm)} mm` : `；右臂后退 ${formatNumber(job.arm_retreat_mm)} mm`}`);
  if (job.advance_mm > 0) details.push(`躯干前移补偿 ${formatNumber(job.advance_mm)} mm；同步后退：手臂 ${formatNumber(job.arm_retreat_mm)} mm、躯干 ${formatNumber(job.paired_trunk_retreat_mm)} mm`);
  if (job.sku_typ === "tube") details.push(`规划前夹爪开到130；法兰高度 ${formatNumber(job.flange_height_trunk_mm)} mm；回正并退 d−20（收回补偿A），最后同步退40/100 mm`);
  if (job.rotation_deg_s != null) details.push(`${job.speed_mm_s} mm/s · ${job.rotation_deg_s} °/s`);
  if (job.first_dispatch_ms != null) details.push(`后端至首条指令启动返回 ${Math.round(job.first_dispatch_ms)} ms`);
  if (job.timings_ms) {
    const labels = { initial_snapshot: "起始回读", gripper_ready: "夹爪检查", reproject_pose: "坐标转换", trunk_endpoint_preflight: "躯干目标准备", pregrasp_plan: "预抓取演算", pregrasp_dispatch: "首段下发" };
    for (const [key, label] of Object.entries(labels)) {
      if (job.timings_ms[key] != null) details.push(`${label} ${Math.round(job.timings_ms[key])} ms`);
    }
  }
  if (graspExecution.active) details.push(`已完成 ${graspExecution.completed_moves || 0}/${graspExecution.total_moves || 6} 个运动目标`);
  if (graspExecution.planning_ms != null) details.push(`本步演算 ${formatNumber(graspExecution.planning_ms)} ms`);
  if (graspExecution.selected_arm_angle_deg != null) details.push(`臂角 ${formatNumber(graspExecution.selected_arm_angle_deg)}°`);
  if (graspExecution.gripper_message) details.push(graspExecution.gripper_message);
  if (graspExecution.phase === "idle" && ready) details.push(!graspSourceId ? "先估计位姿或转换已保存结果" :
    !serverStatus?.armed ? "请解锁控制" : (selected === "box" ? "可执行左臂抓取：下压吸附，提起后手臂退d+30、躯干退100 mm" : (selected === "tube" ? "可执行软管抓取：规划前夹爪开到130，原地下俯30°后合爪，回正并退d−20，最后同步退40/100 mm" : "可执行右臂抓取；规划前确认夹爪完全张开")));
  node.textContent = details.filter(Boolean).join(" · ");
  node.className = `pose-estimation-status ${graspExecution.phase === "failed" ? "error" : busy ? "working" : "muted"}`;
}

async function executeGraspTest() {
  if (graspBusy || graspExecution.active || scanExecution.active || scanBusy || placementExecution.active || placementBusy) return;
  const kind = document.getElementById('poseTargetSelect').value;
  if (!['bottle', 'box', 'tube'].includes(kind) || kind !== poseSelectedTarget || !graspSourceId) {
    toast('请先识别或转换当前物品的抓取点', true); return;
  }
  const elbow = document.getElementById(`poseElbow-${kind === 'box' ? 'left_arm' : 'right_arm'}`).value.trim();
  if (elbow !== "" && !Number.isFinite(Number(elbow))) { toast("初始臂角无效", true); return; }
  graspBusy = true;
  stopChassis();
  updateMoveLControls();
  try {
    graspExecution = await api("/api/pose-estimation/grasp-test", { method: "POST", body: {
      source_result_id: graspSourceId, sku_typ: kind, elbow_deg: elbow === "" ? null : Number(elbow),
    } });
    toast("抓取测试已开始");
  } catch (error) { toast(error.message, true); }
  finally { graspBusy = false; updateMoveLControls(); }
}

async function stopGraspTest() {
  try {
    graspExecution = await api("/api/pose-estimation/grasp-test/stop", { method: "POST", body: {} });
    updateMoveLControls();
  } catch (error) { toast(error.message, true); }
}

function updateScanControls() {
  const selected = document.getElementById("poseTargetSelect").value;
  document.getElementById("scanSequenceButton").textContent = selected === "tube" ? "下一步：软管扫码采集" : "下一步：扫码采集";
  const blocked = scanExecution.stop_unconfirmed;
  const busy = scanBusy || scanExecution.active;
  document.getElementById("scanSequenceButton").disabled = !serverStatus?.armed || busy || blocked ||
    serverStatus?.scan_sequence?.version !== "scan-sequence-v5" ||
    !["bottle", "box", "tube"].includes(document.getElementById("poseTargetSelect").value) ||
    memoryExecution.active || movelExecution.active || movelBusy || graspExecution.active || graspBusy || placementExecution.active || placementBusy;
  document.getElementById("scanSequenceStop").disabled = !scanExecution.active && !blocked;
  const node = document.getElementById("scanSequenceStatus");
  node.textContent = [scanExecution.message, `已保存 ${scanExecution.completed_points || 0}/${scanExecution.total_photos || (selected === "box" ? 1 : selected === "tube" ? 2 : 5)} 张`,
    scanExecution.directory ? `保存目录：${scanExecution.directory}` : ""].filter(Boolean).join(" · ");
  node.className = `pose-estimation-status ${scanExecution.phase === "failed" ? "error" : busy ? "working" : "muted"}`;
}

async function executeScanSequence() {
  const kind = document.getElementById("poseTargetSelect").value;
  if (scanBusy || scanExecution.active || !["bottle", "box", "tube"].includes(kind)) return;
  scanBusy = true;
  stopChassis();
  updateMoveLControls();
  try {
    scanExecution = await api("/api/scan-sequence/start", { method: "POST", body: { sku_typ: kind } });
    toast("扫码采集已开始");
  } catch (error) { toast(error.message, true); }
  finally { scanBusy = false; updateMoveLControls(); }
}

async function stopScanSequence() {
  try {
    scanExecution = await api("/api/scan-sequence/stop", { method: "POST", body: {} });
    updateMoveLControls();
  } catch (error) { toast(error.message, true); }
}

function updatePlacementControls() {
  const busy = placementBusy || placementExecution.active;
  document.getElementById('placementButton').disabled = !serverStatus?.armed || busy ||
    placementExecution.stop_unconfirmed || !basketSourceId || !gripperState?.unlocked ||
    serverStatus?.placement?.version !== 'placement-v7' || poseEstimateBusy || poseReprojectBusy ||
    scanBusy || scanExecution.active || graspBusy || graspExecution.active || memoryExecution.active || movelBusy || movelExecution.active;
  document.getElementById('tubePlacementButton').disabled = document.getElementById('placementButton').disabled;
  document.getElementById('boxPlacementButton').disabled = !serverStatus?.armed || busy ||
    placementExecution.stop_unconfirmed || !boxBasketSourceId || !serverStatus?.suction?.available ||
    serverStatus?.placement?.version !== 'placement-v7' || poseEstimateBusy || poseReprojectBusy ||
    scanBusy || scanExecution.active || graspBusy || graspExecution.active || memoryExecution.active || movelBusy || movelExecution.active;
  document.getElementById('placementStop').disabled = !placementExecution.active && !placementExecution.stop_unconfirmed;
  const node = document.getElementById('placementStatus');
  const bottleDetails = placementExecution.sku_typ !== 'box';
  node.textContent = [placementExecution.message, bottleDetails && !basketSourceId ? '右臂放置：请先识别篮筐' : '',
    bottleDetails && basketSourceId && basketReferencePoint ? `篮筐右肩参考点 ${formatPoseVector(basketReferencePoint)} mm` : '',
    bottleDetails && !gripperState?.unlocked ? '右臂放置需解锁总控制并初始化右手夹爪' : ''].filter(Boolean).join(' · ');
  document.getElementById('boxPlacementStatus').textContent = [
    boxBasketSourceId && boxBasketReferencePoint ? `篮筐左肩 Y=${formatNumber(boxBasketReferencePoint[1])} mm；盒子预放置 Y=${formatNumber(boxBasketReferencePoint[1] + 110)} mm` : '盒子放置：请先识别篮筐，取得左肩参考点',
    !serverStatus?.suction?.available ? '吸盘配置不可用' : '',
    serverStatus?.placement?.version !== 'placement-v7' ? '放置新版尚未加载，请重启 mui-control.service 后刷新' : ''
  ].filter(Boolean).join(' · ');
  document.getElementById('tubePlacementStatus').textContent = [
    basketSourceId && basketReferencePoint ? `篮筐右肩 Y=${formatNumber(basketReferencePoint[1])} mm；软管预放置 Y=${formatNumber(basketReferencePoint[1] - 80)} mm` : '软管放置：请先识别篮筐，取得右肩参考点',
    serverStatus?.placement?.version !== 'placement-v7' ? '放置新版尚未加载，请重启 mui-control.service 后刷新' : ''
  ].filter(Boolean).join(' · ');
  node.className = `pose-estimation-status ${placementExecution.phase === 'failed' ? 'error' : busy ? 'working' : 'muted'}`;
}

async function executePlacement(kind = 'bottle') {
  const source = kind === 'box' ? boxBasketSourceId : basketSourceId;
  if (!['bottle', 'box', 'tube'].includes(kind) || placementBusy || placementExecution.active || !source) return;
  placementBusy = true;
  stopChassis();
  updateMoveLControls();
  try {
    placementExecution = await api('/api/placement/start', {method:'POST',body:kind !== 'bottle' ? {source_result_id:source,sku_typ:kind} : {source_result_id:source}});
    toast('放置流程已开始');
  } catch (error) { toast(error.message,true); }
  finally { placementBusy = false; updateMoveLControls(); }
}

async function stopPlacement() {
  try {
    placementExecution = await api('/api/placement/stop', {method:'POST',body:{}});
    updateMoveLControls();
  } catch (error) { toast(error.message,true); }
}

function updateBattery(battery, error = "") {
  const badge = document.getElementById("batteryStatus");
  const value = document.getElementById("batteryValue");
  const detail = document.getElementById("batteryDetail");
  const fill = document.getElementById("batteryFill");
  const level = battery?.percentage;
  const valid = battery?.available && !battery?.stale && Number.isFinite(level) && level >= 0 && level <= 100;
  batteryExpiresAt = 0;
  if (!valid) {
    badge.className = "battery-status";
    value.textContent = "电量 --";
    detail.textContent = error || (battery?.source === "mock" ? "MOCK · 无电池数据" : battery?.stale ? "数据已过期" : "电量未知");
    badge.title = error || battery?.error || "等待电池接口数据；更新网页后请确认后端已重启";
    fill.style.width = "0%";
    return;
  }
  badge.className = `battery-status ${level <= 10 ? "danger" : level <= 20 ? "warning" : "ok"}`;
  value.textContent = `电量 ${Math.round(level)}%`;
  detail.textContent = battery.charging === true ? "充电中" : battery.charging === false ? "未充电" : "充电状态未知";
  fill.style.width = `${level}%`;
  badge.title = `机器人电池 ${level}% · ${detail.textContent}${Number.isFinite(battery.voltage_v) ? ` · ${battery.voltage_v.toFixed(2)} V` : ""}`;
  const freshness = Number.isFinite(battery.stale_after_seconds) ? battery.stale_after_seconds : 15;
  const age = Number.isFinite(battery.age_seconds) ? battery.age_seconds : 0;
  batteryExpiresAt = Date.now() + Math.max(0, freshness - age) * 1000;
}

function updateMoveLControls() {
  const active = Boolean(movelExecution.active || movelBusy);
  document.querySelectorAll("[data-movel]").forEach(button => {
    button.disabled = active || graspExecution.active || graspBusy || scanExecution.active || scanBusy || placementExecution.active || placementBusy || memoryExecution.active || !serverStatus?.armed || !serverStatus?.arm_movel || !Number.isFinite(serverStatus?.rotation_deg_s) ||
      (button.dataset.mode === "protected" && serverStatus.arm_movel.planner_version !== "endpoint-v1");
  });
  document.querySelectorAll("[data-movel-stop]").forEach(button => {
    button.disabled = !movelExecution.active || movelExecution.module !== button.dataset.movelStop;
  });
  for (const module of ["left_arm", "right_arm"]) {
    const node = document.getElementById(`movelStatus-${module}`);
    if (!serverStatus?.arm_movel) {
      node.textContent = "MoveL 后端尚未加载，请重启网页控制服务后刷新。";
    } else if (!Number.isFinite(serverStatus?.rotation_deg_s)) {
      node.textContent = "旋转速度设置尚未加载，请重启网页控制服务后刷新。";
    } else if (serverStatus.arm_movel.planner_version !== "endpoint-v1") {
      node.textContent = "终点保护后端尚未加载，请重启网页控制服务后刷新。";
    } else if (movelExecution.module === module) {
      const angle = movelExecution.selected_arm_angle_deg;
      const clearance = movelExecution.endpoint_clearance_mm;
      node.textContent = [movelExecution.message,
        angle == null ? "" : `选中臂角 ${formatNumber(angle)}°`,
        movelExecution.endpoint_accepted_by === "front_plane"
          ? `终点肘点已超过胸部 X=${formatNumber(movelExecution.endpoint_front_plane_x_mm)} mm 平面 ${formatNumber(movelExecution.endpoint_front_clearance_mm)} mm，允许越过 Y 保护平面`
          : clearance == null ? "" : `终点肘部净余量 ${formatNumber(clearance)} mm`,
        movelExecution.planning_ms == null ? "" : `演算 ${formatNumber(movelExecution.planning_ms)} ms`,
        movelExecution.dispatch_ms == null ? "" : `请求至启动指令返回 ${formatNumber(movelExecution.dispatch_ms)} ms`,
        movelExecution.speed_mm_s == null ? "" : `平移 ${movelExecution.speed_mm_s} mm/s`,
        movelExecution.rotation_deg_s == null ? "" : `旋转 ${movelExecution.rotation_deg_s} °/s`].filter(Boolean).join(" · ");
      node.className = `memory-status${movelExecution.phase === "failed" ? " error" : ""}`;
    }
  }
  updateMemoryControls();
}

function selectedMemoryPoint() {
  return memoryPoints.find(point => point.id === document.getElementById("memorySelect").value);
}

function updateMemoryControls() {
  const active = Boolean(memoryExecution.active);
  const moving = active || movelExecution.active || movelBusy || graspExecution.active || graspBusy || scanExecution.active || scanBusy || placementExecution.active || placementBusy;
  const busy = memoryBusy || moving;
  const selected = Boolean(selectedMemoryPoint());
  document.getElementById("memorySelect").disabled = busy || !memoryReady;
  document.getElementById("memoryCreate").disabled = busy || !memoryReady;
  document.getElementById("memoryOverwrite").disabled = busy || !selected || !memoryReady;
  document.getElementById("memoryDelete").disabled = busy || !selected || !memoryReady || !memoryCanDelete;
  document.getElementById("memoryExecute").disabled = busy || !selected || !memoryReady || !serverStatus?.armed || !Number.isFinite(serverStatus?.rotation_deg_s);
  document.getElementById("memoryRefresh").disabled = memoryBusy;
  document.getElementById("memoryStop").disabled = !active;
  document.getElementById("memoryNameSave").disabled = memoryBusy;
  const names = { idle: "待命", planning: "预检中", arms: "双臂运动", head_trunk: "头部与躯干运动", stopping: "停止中", completed: "已到位", failed: "执行失败", cancelled: "已停止" };
  const badge = document.getElementById("memoryPhase");
  badge.textContent = names[memoryExecution.phase] || memoryExecution.phase;
  badge.className = `badge ${memoryExecution.phase === "failed" ? "danger" : active ? "warning" : ""}`;
  const modes = Object.entries(memoryExecution.arm_modes || {}).map(([arm, mode]) =>
    `${jointModules[arm].title}：${mode.motion}${mode.reason ? `（${mode.reason}）` : ""}`).join("；");
  const status = document.getElementById("memoryStatus");
  status.textContent = [memoryExecution.name, memoryExecution.message, modes].filter(Boolean).join(" · ");
  status.className = `memory-status${memoryExecution.phase === "failed" ? " error" : ""}`;
  document.querySelectorAll("[data-joint-move], [data-pose-move], [data-drag]").forEach(button => {
    button.disabled = moving;
  });
  document.querySelectorAll("[data-movel]").forEach(button => {
    button.disabled = moving || !serverStatus?.armed || !serverStatus?.arm_movel || !Number.isFinite(serverStatus?.rotation_deg_s) ||
      (button.dataset.mode === "protected" && serverStatus.arm_movel.planner_version !== "endpoint-v1");
  });
  document.querySelectorAll(".hold").forEach(button => { button.disabled = moving; });
  document.getElementById("chassisEnableButton").disabled = moving;
  document.getElementById("chassisReleaseEmergencyButton").disabled = moving || !serverStatus?.armed;
  updateChassisAvoidance();
  updateGripperControls();
  updateGraspControls();
  updateScanControls();
  updatePlacementControls();
}

function applyMemoryList(data, selectId) {
  const select = document.getElementById("memorySelect");
  const previous = selectId || select.value;
  memoryPoints = data.points;
  select.replaceChildren();
  const placeholder = document.createElement("option");
  placeholder.value = "";
  placeholder.textContent = memoryPoints.length ? "请选择记忆点" : "尚无记忆点，请先新增";
  select.append(placeholder);
  for (const point of memoryPoints) {
    const option = document.createElement("option");
    option.value = point.id;
    option.textContent = point.name;
    select.append(option);
  }
  if (memoryPoints.some(point => point.id === previous)) select.value = previous;
  memoryReady = true;
  memoryCanDelete = data.can_delete === true;
  memoryExecution = data.execution;
  updateMemoryControls();
}

async function refreshMemoryPoints(silent = false) {
  if (memoryPollBusy || memoryBusy) return;
  memoryPollBusy = true;
  const generation = memoryListGeneration;
  try {
    const data = await api("/api/memory-points");
    if (generation === memoryListGeneration) applyMemoryList(data);
  }
  catch (error) {
    if (generation !== memoryListGeneration) return;
    memoryReady = false;
    updateMemoryControls();
    document.getElementById("memoryStatus").textContent = error.message;
    document.getElementById("memoryCreate").disabled = true;
    document.getElementById("memoryExecute").disabled = true;
    if (!silent) toast(error.message, true);
  } finally { memoryPollBusy = false; }
}

async function saveMemoryPoint(overwrite = false, name = null) {
  if (memoryBusy || memoryExecution.active) return;
  const selected = selectedMemoryPoint();
  if (overwrite && !selected) return;
  if (!overwrite && name === null) {
    document.getElementById("memoryNameInput").value = "";
    document.getElementById("memoryNameDialog").showModal();
    return;
  }
  memoryBusy = true;
  memoryListGeneration += 1;
  updateMemoryControls();
  try {
    const data = await api(`/api/memory-points/${overwrite ? "overwrite" : "create"}`, {
      method: "POST", body: overwrite ? { id: selected.id, revision: selected.revision } : { name },
    });
    applyMemoryList(data, data.point.id);
    if (!overwrite) document.getElementById("memoryNameDialog").close();
    toast(`${overwrite ? "已覆盖" : "已新增"}记忆点：${data.point.name}（当前关节已重新回读）`);
  } catch (error) { toast(error.message, true); }
  finally { memoryBusy = false; updateMemoryControls(); }
}

async function deleteMemoryPoint() {
  const point = selectedMemoryPoint();
  if (!point || memoryBusy || !memoryReady || !memoryCanDelete || memoryExecution.active ||
      movelExecution.active || movelBusy || graspExecution.active || graspBusy || scanExecution.active || scanBusy || placementExecution.active || placementBusy) return;
  memoryBusy = true;
  memoryListGeneration += 1;
  updateMemoryControls();
  try {
    const data = await api("/api/memory-points/delete", {
      method: "POST", body: { id: point.id, revision: point.revision },
    });
    applyMemoryList(data);
    toast(`已删除记忆点：${data.point.name}`);
  } catch (error) { toast(error.message, true); }
  finally { memoryBusy = false; updateMemoryControls(); }
}

async function executeMemoryPoint() {
  const point = selectedMemoryPoint();
  if (!point || memoryBusy || memoryExecution.active) return;
  memoryBusy = true;
  updateMemoryControls();
  try {
    memoryExecution = await api("/api/memory-points/execute", {
      method: "POST", body: { id: point.id, revision: point.revision },
    });
    toast(`开始执行记忆点：${point.name}`);
  } catch (error) { toast(error.message, true); }
  finally { memoryBusy = false; updateMemoryControls(); }
}

async function stopMemoryPoint() {
  try {
    memoryExecution = await api("/api/memory-points/stop", { method: "POST", body: {} });
    updateMemoryControls();
  } catch (error) { toast(error.message, true); }
}

function updateSuctionControls() {
  const state = serverStatus?.suction;
  const busy = memoryExecution.active || movelExecution.active || movelBusy || graspExecution.active || graspBusy || scanExecution.active || scanBusy || placementExecution.active || placementBusy;
  for (const id of ['suctionOpenButton', 'suctionCloseButton']) {
    const button = document.getElementById(id);
    if (button) button.disabled = !serverStatus?.armed || !state?.available || suctionBusy || busy;
  }
  const label = document.getElementById('suctionStatus');
  if (!label) return;
  label.textContent = suctionBusy ? '吸盘：正在发送指令…' :
    state?.error ? `吸盘：状态未确认 · ${state.error}` :
    !state?.available ? `吸盘：${state?.message || '正在读取配置'}` :
    state?.confirmed ? `吸盘：${state.commanded_open ? '打开' : '关闭'}指令已确认` : '吸盘：尚未发送指令';
}

async function setSuction(open) {
  if (suctionBusy) return;
  suctionBusy = true;
  updateSuctionControls();
  try {
    const result = await api('/api/suction/set', { method: 'POST', body: { open } });
    serverStatus.suction = result;
    toast(`吸盘${open ? '打开' : '关闭'}指令已确认`);
  } catch (error) {
    if (serverStatus) serverStatus.suction = { ...serverStatus.suction, commanded_open: null, confirmed: false, error: error.message };
    toast(error.message, true);
  } finally {
    suctionBusy = false;
    updateSuctionControls();
  }
}

function updateGripperControls() {
  updateSuctionControls();
  const locallyUnlocked = Boolean(serverStatus?.armed && serverStatus?.gripper?.unlocked);
  const canControl = Boolean(serverStatus?.armed && locallyUnlocked && !memoryExecution.active && !movelExecution.active && !movelBusy && !graspExecution.active && !graspBusy && !scanExecution.active && !scanBusy && !placementExecution.active && !placementBusy);
  const ready = gripperState?.activation_state === 3 && gripperState?.fault_code === 0;
  const badge = document.getElementById("gripperControlBadge");
  badge.textContent = locallyUnlocked ? "已随总控制启用" : "请解锁顶部总控制";
  badge.className = `badge ${locallyUnlocked ? "ok" : ""}`;
  document.getElementById("gripperActivateButton").disabled = !canControl || gripperBusy || !gripperState || ready;
  for (const id of ["gripperOpenButton", "gripperCloseButton", "gripperPositionSlider"]) {
    document.getElementById(id).disabled = !canControl || gripperBusy || !ready;
  }
}

function updateGripperState(state) {
  gripperState = state;
  const badge = document.getElementById("gripperStateBadge");
  if (state.fault_code) {
    badge.textContent = `故障 0x${Number(state.fault_code).toString(16).padStart(2, "0")}`;
    badge.className = "badge danger";
  } else if (state.activation_state !== 3) {
    badge.textContent = "待初始化";
    badge.className = "badge warning";
  } else {
    badge.textContent = "夹爪就绪";
    badge.className = "badge ok";
  }
  document.getElementById("gripperReadback").textContent =
    `当前位置：${state.measured_position} / 255 · 最近目标：${state.requested_position} · ${state.object_state === 2 ? "闭合时接触物品" : state.object_state === 1 ? "张开时受阻" : "无接触"}`;
  if (!gripperSliderInitialized) {
    document.getElementById("gripperPositionSlider").value = state.requested_position;
    document.getElementById("gripperPositionValue").textContent = state.requested_position;
    gripperSliderInitialized = true;
  }
  updateGripperControls();
  updateGraspControls();
  updateScanControls();
  updatePlacementControls();
}

async function refreshGripperStatus(quiet = true) {
  if (gripperStatusBusy || gripperBusy) return;
  gripperStatusBusy = true;
  try {
    updateGripperState(await api("/api/gripper/status"));
  } catch (error) {
    gripperState = null;
    const badge = document.getElementById("gripperStateBadge");
    badge.textContent = "夹爪未连接";
    badge.className = "badge danger";
    document.getElementById("gripperReadback").textContent = `状态读取失败：${error.message}`;
    updateGripperControls();
    if (!quiet) toast(error.message, true);
  } finally {
    gripperStatusBusy = false;
  }
}

function clearCameraFrame(cameraId) {
  const rgb = document.getElementById(`rgbCameraFrame-${cameraId}`);
  rgb.hidden = true;
  rgb.removeAttribute("src");
}

function updateCameraActionButtons(cameraId) {
  document.getElementById(`cameraFrameButton-${cameraId}`).disabled =
    !cameraFrameAvailable[cameraId] || cameraBusy[cameraId].frame;
  document.getElementById(`cameraRecordButton-${cameraId}`).disabled =
    !cameraEnabled[cameraId] || cameraBusy[cameraId].record;
}

function updateCameraStatus(cameraId, camera) {
  const wasFrameAvailable = cameraFrameAvailable[cameraId];
  cameraEnabled[cameraId] = Boolean(camera.enabled);
  cameraFrameAvailable[cameraId] = Boolean(camera.frame_available ?? camera.enabled);
  const available = camera.available !== false;
  const recoveryRunning = Boolean(camera.recovery_running || Object.values(cameraBusy).some(item => item.recovery));
  const badge = document.getElementById(`cameraBadge-${cameraId}`);
  badge.textContent = `${cameraId === "head" ? "头部" : cameraId === "left_wrist" ? "左手" : "右手"} ${cameraEnabled[cameraId] ? "已就绪" : available ? "等待 ROS2" : "订阅不可用"}`;
  badge.className = `badge ${cameraEnabled[cameraId] ? "ok" : "warning"}`;
  const restartButton = document.getElementById(`cameraRestartButton-${cameraId}`);
  if (restartButton) {
    restartButton.disabled = camera.recovery_available !== true || camera.recovery_scope !== "all" || recoveryRunning;
    restartButton.textContent = recoveryRunning ? "正在重启三路摄像头…" : "重启摄像头服务（三路）";
  }
  if (wasFrameAvailable && !cameraFrameAvailable[cameraId]) {
    cameraBusy[cameraId].frame = false;
    clearCameraFrame(cameraId);
  }
  updateCameraActionButtons(cameraId);

  const profile = camera.rgb_profile;
  const fps = Number(camera.actual_capture_fps || 0);
  const details = [];
  if (cameraEnabled[cameraId]) {
    details.push(camera.model || cameraSpecs[cameraId].label);
    if (profile) details.push(`${profile.width}×${profile.height}@${profile.fps}`);
    details.push(`相机 8085 · ${cameraSpecs[cameraId].colorOnly ? "仅 RGB，不启用深度" : "深度已对齐 RGB"}`);
    if (fps > 0) details.push(`订阅 ${formatNumber(fps)} 帧`);
  } else {
    details.push(camera.error || (available ? (cameraSpecs[cameraId].colorOnly ? "等待手腕 RGB 数据" : "等待 ROS2 同步 RGB-D 数据") : "ROS2 摄像头订阅不可用"));
    if (cameraFrameAvailable[cameraId]) details.push(cameraSpecs[cameraId].colorOnly ? "可尝试获取手腕 RGB 当前帧" : "RGB 当前帧可单独尝试获取；记录仍需同步 RGB-D");
  }
  if (recoveryRunning) {
    details.push("正在重启头部、左腕和右腕摄像头服务");
  } else if (camera.recovery_result) {
    const result = camera.recovery_result;
    if (result.error) details.push(result.error);
    else if (result.command_ok) details.push("三路重启命令已完成；图像状态以各摄像头回读为准");
  }
  document.getElementById(`cameraInfo-${cameraId}`).textContent = details.join(" · ");
}

function updateCameraStatuses(cameras) {
  Object.keys(cameraSpecs).forEach(cameraId => updateCameraStatus(cameraId, cameras[cameraId] || { available: false }));
}

async function restartCamera() {
  if (Object.values(cameraBusy).some(item => item.recovery) || serverStatus?.cameras?.head?.recovery_running) return;
  const button = document.getElementById("cameraRestartButton-head");
  Object.values(cameraBusy).forEach(item => { item.recovery = true; });
  button.disabled = true;
  button.textContent = "正在重启三路摄像头…";
  try {
    const result = await api("/api/cameras/restart", { method: "POST", body: { camera: "all" } });
    toast(result.message || "已开始重启头部、左腕和右腕摄像头服务");
  } catch (error) {
    toast(error.message, true);
  } finally {
    Object.values(cameraBusy).forEach(item => { item.recovery = false; });
    await refreshStatus(true);
  }
}

function getCameraFrame(cameraId) {
  if (!cameraFrameAvailable[cameraId] || cameraBusy[cameraId].frame) return;
  const image = document.getElementById(`rgbCameraFrame-${cameraId}`);
  const request = ++cameraPreviewRequest;
  cameraBusy[cameraId].frame = true;
  updateCameraActionButtons(cameraId);
  image.onload = () => {
    image.onload = null;
    image.onerror = null;
    if (request === cameraPreviewRequest) {
      Object.keys(cameraSpecs).forEach(id => {
        document.getElementById(`rgbCameraFrame-${id}`).hidden = id !== cameraId;
      });
      document.getElementById("cameraPreviewCaption").textContent = `${cameraSpecs[cameraId].label} RGB 单帧预览 · 1280 × 720`;
    }
    cameraBusy[cameraId].frame = false;
    updateCameraActionButtons(cameraId);
    toast(`${cameraSpecs[cameraId].label}当前帧已更新`);
  };
  image.onerror = () => {
    image.onload = null;
    image.onerror = null;
    clearCameraFrame(cameraId);
    cameraBusy[cameraId].frame = false;
    updateCameraActionButtons(cameraId);
    toast(`${cameraSpecs[cameraId].label}当前帧获取失败`, true);
  };
  image.src = `/api/cameras/${cameraId}/frame/rgb?t=${Date.now()}`;
}

async function recordCameraRgb(cameraId) {
  if (!cameraEnabled[cameraId] || cameraBusy[cameraId].record) return;
  cameraBusy[cameraId].record = true;
  updateCameraActionButtons(cameraId);
  try {
    const result = await api(`/api/cameras/${cameraId}/record-rgb`, { method: "POST", body: {} });
    toast(`已保存${cameraSpecs[cameraId].label} RGB：${result.relative_path || result.path}`);
  } catch (error) {
    toast(error.message, true);
  } finally {
    cameraBusy[cameraId].record = false;
    updateCameraActionButtons(cameraId);
  }
}

async function recordCameraData(cameraId) {
  cameraBusy[cameraId].record = true;
  updateCameraActionButtons(cameraId);
  try {
    const result = await api(`/api/cameras/${cameraId}/record`, { method: "POST", body: {} });
    toast(`已记录${cameraSpecs[cameraId].label}：${result.relative_directory}`);
  } catch (error) {
    toast(error.message, true);
  } finally {
    cameraBusy[cameraId].record = false;
    updateCameraActionButtons(cameraId);
  }
}

function setPoseServiceBadge(available, text) {
  const badge = document.getElementById("poseServiceBadge");
  badge.textContent = text;
  badge.className = `badge ${available ? "ok" : "danger"}`;
}

async function checkPoseService(quiet = true) {
  try {
    const result = await api("/api/pose-estimation/health");
    if (result.available) {
      setPoseServiceBadge(true, "定位服务 已连接");
    } else {
      setPoseServiceBadge(false, "定位服务 不可用");
      if (!quiet) toast(result.error || "定位服务 定位服务不可用", true);
    }
    return Boolean(result.available);
  } catch (error) {
    setPoseServiceBadge(false, "定位服务 检查失败");
    if (!quiet) toast(error.message, true);
    return false;
  }
}

function formatPoseVector(value) {
  if (!Array.isArray(value) || value.length !== 3) return "—";
  return `[${value.map(item => Number(item).toFixed(3)).join(", ")}]`;
}

function formatPoseValues(value, count) {
  if (!Array.isArray(value) || value.length !== count ||
      !value.every(item => Number.isFinite(Number(item)))) return "—";
  return `[${value.map(item => Number(item).toFixed(3)).join(", ")}]`;
}

function renderBoxClearance(box, basis) {
  const node = document.getElementById("poseBoxClearance");
  if (!box?.valid) {
    node.textContent = box?.error ? `箱体距离不可用：${box.error}` : "箱体距离：等待有效的物品和前挡板结果";
    return;
  }
  node.textContent = `${basis}躯干 SDK 系（trunk_controller_ref）：D（原点沿 X+ 到平面）${formatNumber(box.D_mm)} mm；d（箱沿到物体，沿 X）${formatNumber(box.d_mm)} mm；预抓取虚拟夹爪长度 ${formatNumber(box.pregrasp_virtual_length_mm)} mm${box.sku_typ === "box" ? "；左臂下压吸附后提起，手臂后退d+30 mm、躯干后退100 mm" : box.sku_typ === "tube" ? `；软管回正并退 ${formatNumber(box.d_mm - 20)} mm，最后右臂/躯干同步退40/100 mm` : `；右臂后退 ${formatNumber(box.d_mm + 20)} mm`}`;
}

function clearShoulderResult() {
  document.getElementById("poseShoulderData").hidden = true;
  document.getElementById("poseReprojectStatus").textContent =
    "尚未转换；使用最近一次有效的世界坐标抓取点";
  document.getElementById("poseReprojectStatus").className = "pose-estimation-status muted";
  for (const id of ["poseShoulderGrasp", "poseShoulderPregrasp", "poseShoulderTrunk"]) {
    document.getElementById(id).textContent = "—";
  }
}

function renderWorldGrasp(world) {
  const valid = Array.isArray(world?.grasp_pose_world_mm_deg) &&
    world.grasp_pose_world_mm_deg.length === 6;
  document.getElementById("poseWorldGrasp").textContent =
    formatPoseValues(world?.grasp_pose_world_mm_deg?.slice(0, 3), 3);
  document.getElementById("poseCaptureTrunk").textContent =
    formatPoseValues(world?.capture_upper_body_joints_deg?.slice(0, 4), 4);
  document.getElementById("poseFixedHeight").textContent =
    Number.isFinite(world?.grasp_height_trunk_mm)
      ? `${world.grasp_height_sku_typ || "商品"}：躯干 SDK Z = ${formatNumber(world.grasp_height_trunk_mm)} mm（识别点沿躯干 Z+ 与该类别标定平面相交）`
      : "—";
  document.getElementById("poseRecognitionHeight").textContent =
    Number.isFinite(world?.recognition_height_trunk_mm) ? `${formatNumber(world.recognition_height_trunk_mm)} mm${world.sku_typ === "box" ? "；仅用于显示，下降高度采用独立标定" : ""}` : "—";
  document.getElementById("poseGraspOffset").textContent =
    formatPoseValues(world?.grasp_offset_trunk_mm, 3);
  document.getElementById("poseWorldData").hidden = !valid;
}

function clearPoseResult() {
  document.getElementById('poseTargetSemantics').textContent = '—';
  document.getElementById('poseLocationChassis').textContent = '—';
  document.getElementById('poseLocationShoulder').textContent = '—';
  document.getElementById('poseLocationShoulderRow').hidden = true;
  document.getElementById('poseLocationLeftShoulder').textContent = '—';
  document.getElementById('poseLocationLeftShoulderRow').hidden = true;
  document.getElementById('poseLocationJson').textContent = '';
  document.getElementById('poseLocationDetails').hidden = true;
  renderBoxClearance(null);
  graspSourceId = null;
  updateGraspControls();
  updateScanControls();
  updatePlacementControls();
  const image = document.getElementById("poseResultImage");
  image.removeAttribute("src");
  document.getElementById("poseResultFigure").hidden = true;
  document.getElementById("poseResultData").hidden = true;
  document.getElementById("poseWorldData").hidden = true;
  document.getElementById("poseReferenceCamera").textContent = "—";
  for (const id of ["poseWorldGrasp", "poseCaptureTrunk"]) {
    document.getElementById(id).textContent = "—";
  }
  document.getElementById("poseSavedAt").textContent = "—";
  clearShoulderResult();
}

function renderPoseResult(result) {
  const location = result.localization;
  poseSelectedTarget = result.selected_target || poseSelectedTarget;
  if (result.selected_target === 'basket') basketSourceId = result.usable && location?.valid &&
    location.right_shoulder_frame === 'right_arm_sdk_world' && Array.isArray(location.point_right_shoulder_mm)
    ? result.result_id : null;
  if (result.selected_target === 'basket') {
    basketReferencePoint = basketSourceId ? location.point_right_shoulder_mm : null;
    boxBasketSourceId = result.usable && location?.valid && location.left_shoulder_frame === 'left_arm_sdk_world' &&
      Array.isArray(location.point_left_shoulder_mm) ? result.result_id : null;
    boxBasketReferencePoint = boxBasketSourceId ? location.point_left_shoulder_mm : null;
  }
  document.getElementById('poseTargetSemantics').textContent = location?.valid
    ? `${location.label} · ${location.point_semantics}` : '本帧无有效定位';
  document.getElementById('poseLocationChassis').textContent = formatPoseVector(location?.point_chassis_mm);
  const isBasket = result.selected_target === 'basket';
  document.getElementById('poseLocationShoulderRow').hidden = !isBasket;
  document.getElementById('poseLocationShoulder').textContent = isBasket && location?.valid
    ? (location.right_shoulder_error ? '不可用：' + location.right_shoulder_error
      : formatPoseVector(location.point_right_shoulder_mm)) : '—';
  document.getElementById('poseLocationLeftShoulderRow').hidden = !isBasket;
  document.getElementById('poseLocationLeftShoulder').textContent = isBasket && location?.valid
    ? (location.left_shoulder_error ? '不可用：' + location.left_shoulder_error
      : formatPoseVector(location.point_left_shoulder_mm)) : '—';
  document.getElementById('poseLocationJson').textContent = JSON.stringify({localization:location, response:result.response}, null, 2);
  document.getElementById('poseLocationDetails').hidden = false;
  graspSourceId = result.grasp_test_usable === true && result.world_grasp ? result.result_id : null;
  renderBoxClearance(result.box_clearance, "拍照时");
  updateGraspControls();
  updateScanControls();
  updatePlacementControls();
  const response = result.response || {};
  const status = document.getElementById("poseEstimateStatus");
  status.textContent = result.message || (result.usable ? "定位成功" : "定位无效");
  status.className = `pose-estimation-status ${result.usable ? "ok" : "error"}`;

  document.getElementById("poseReferenceCamera").textContent =
    formatPoseVector(location ? location.point_camera_mm : response.reference_point_camera_mm);
  renderWorldGrasp(result.world_grasp);
  document.getElementById("poseSavedAt").textContent = result.relative_directory || "—";
  document.getElementById("poseResultData").hidden = false;

  if (result.image_url) {
    const image = document.getElementById("poseResultImage");
    image.onload = () => { document.getElementById("poseResultFigure").hidden = false; };
    image.onerror = () => {
      document.getElementById("poseResultFigure").hidden = true;
      toast("位姿估计结果图加载失败", true);
    };
    image.src = `${result.image_url}?t=${Date.now()}`;
  }
  if (result.status === "connection_error") {
    setPoseServiceBadge(false, "定位服务 连接失败");
  }
}

function selectPoseTarget() {
  poseSelectedTarget = document.getElementById('poseTargetSelect').value;
  clearPoseResult();
  const status = document.getElementById('poseEstimateStatus');
  status.textContent = '尚未进行位姿估计';
  status.className = 'pose-estimation-status muted';
}

async function estimateGraspObjectPose() {
  if (poseEstimateBusy || poseReprojectBusy || graspExecution.active || graspBusy || scanExecution.active || scanBusy || placementExecution.active || placementBusy) return;
  const target = document.getElementById('poseTargetSelect').value;
  if (target === 'basket') {
    basketSourceId = null;
    basketReferencePoint = null;
    boxBasketSourceId = null;
    boxBasketReferencePoint = null;
  }
  const button = document.getElementById('poseEstimateButton');
  poseSelectedTarget = target;
  const originalLabel = button.textContent;
  const reprojectButton = document.getElementById("poseReprojectButton");
  poseEstimateBusy = true;
  button.disabled = true;
  reprojectButton.disabled = true;
  button.textContent = "正在采集并估计…";
  clearPoseResult();
  const status = document.getElementById("poseEstimateStatus");
  status.textContent = "正在采集同一帧 RGB、深度和头部姿态，并调用 定位服务…";
  status.className = "pose-estimation-status working";
  try {
    const result = await api("/api/pose-estimation/grasp-object", {
      method: "POST",
      body: target === 'basket' ? {target} : {sku_typ:target, side:document.getElementById('poseTargetSide').value},
    });
    renderPoseResult(result);
    await refreshStatus(true);
    if (result.usable) {
      setPoseServiceBadge(true, "定位服务 已连接");
      toast(`位姿估计成功：${result.relative_directory}`);
    } else {
      toast(result.message || "本帧位姿估计无效", true);
    }
  } catch (error) {
    status.textContent = error.message;
    status.className = "pose-estimation-status error";
    toast(error.message, true);
    await checkPoseService(true);
  } finally {
    poseEstimateBusy = false;
    button.disabled = false;
    reprojectButton.disabled = false;
    button.textContent = originalLabel;
    updateGraspControls();
  updateScanControls();
  updatePlacementControls();
  }
}

async function reprojectGraspToRightShoulder() { return reprojectGraspToShoulder('right'); }
async function reprojectGraspToLeftShoulder() { return reprojectGraspToShoulder('left'); }
async function reprojectGraspToShoulder(arm) {
  const expectedType = arm === 'left' ? 'box' : (document.getElementById('poseTargetSelect').value === 'tube' ? 'tube' : 'bottle');
  if (document.getElementById('poseTargetSelect').value !== expectedType) return;
  if (poseEstimateBusy || poseReprojectBusy || graspExecution.active || graspBusy || scanExecution.active || scanBusy || placementExecution.active || placementBusy) return;
  const button = document.getElementById(arm === "left" ? "poseReprojectLeftButton" : "poseReprojectButton");
  const estimateButton = document.getElementById("poseEstimateButton");
  const status = document.getElementById("poseReprojectStatus");
  poseReprojectBusy = true;
  renderBoxClearance(null);
  graspSourceId = null;
  updateGraspControls();
  updateScanControls();
  updatePlacementControls();
  button.disabled = true;
  estimateButton.disabled = true;
  document.getElementById("poseShoulderData").hidden = true;
  status.textContent = "正在回读当前躯干 J1–J4 并转换最近一次有效抓取点…";
  status.className = "pose-estimation-status working";
  try {
    const result = await api(`/api/pose-estimation/reproject-${arm}-shoulder`, {
      method: "POST", body: arm === "right" ? {sku_typ: expectedType} : {},
    });
    if (result.sku_typ !== expectedType) throw new Error('转换结果类别与所选物品不一致');
    poseSelectedTarget = expectedType;
    renderWorldGrasp(result.world_grasp);
    graspSourceId = result.source_result_id;
    renderBoxClearance(result.box_clearance, "当前");
    const shoulder = result.shoulder_grasp || {};
    document.getElementById("poseShoulderGrasp").textContent =
      formatPoseValues(shoulder[`grasp_pose_${arm}_shoulder_mm_deg`], 6);
    document.getElementById("poseShoulderPregrasp").textContent =
      formatPoseValues(shoulder[`pregrasp_pose_${arm}_shoulder_mm_deg`], 6);
    document.getElementById("poseShoulderTrunk").textContent =
      formatPoseValues(result.current_trunk_joints_deg, 4);
    document.getElementById('poseShoulderGraspLabel').textContent = `${arm === 'left' ? '左' : '右'}肩抓取法兰 6D Pose（mm / °）`;
    document.getElementById('poseShoulderPregraspLabel').textContent = `${arm === 'left' ? '左' : '右'}肩预抓取法兰 6D Pose（mm / °）`;
    document.getElementById('poseShoulderDescendRow').hidden = arm !== 'left';
    document.getElementById('poseShoulderDescend').textContent = formatPoseValues(shoulder.descend_pose_left_shoulder_mm_deg, 6);
    document.getElementById('poseShoulderLift').textContent = formatPoseValues(shoulder.lift_pose_left_shoulder_mm_deg, 6);
    document.getElementById('poseShoulderLiftRow').hidden = expectedType !== "box";
    document.getElementById("poseShoulderData").hidden = false;
    status.textContent = `已按当前躯干位姿转换到${arm === "left" ? "左" : "右"}肩坐标系；未发送运动指令`;
    status.className = "pose-estimation-status ok";
  } catch (error) {
    status.textContent = error.message;
    status.className = "pose-estimation-status error";
    toast(error.message, true);
  } finally {
    poseReprojectBusy = false;
    button.disabled = false;
    estimateButton.disabled = false;
    updateGraspControls();
  updateScanControls();
  updatePlacementControls();
  }
}

function updateChassisBadge() {
  const badge = document.getElementById("chassisBadge");
  const chassis = serverStatus?.chassis;
  const offline = chassis?.ros_ready === false;
  badge.textContent = offline ? "底盘未就绪 / 离线" : chassisRemoteEnabled ? "遥控已启用" : "遥控未启用";
  badge.className = `badge ${offline || chassisRemoteEnabled ? "danger" : ""}`;
  const error = document.getElementById("chassisError");
  error.hidden = !offline;
  error.textContent = offline ? (chassis.error || "底盘服务未就绪或已离线；上半身可独立使用") : "";
  document.getElementById("chassisEnableButton").textContent = chassisRemoteEnabled ? "禁用底盘遥控" : "启用底盘遥控";
  updateChassisAvoidance();
}

function updateChassisAvoidance() {
  const state = serverStatus?.chassis?.obstacle_avoidance;
  const known = state && !state.stale && typeof state.enabled === "boolean";
  const badge = document.getElementById("chassisAvoidanceBadge");
  badge.textContent = known ? `遥控避障：${state.enabled ? "已开启" : "已关闭"}` : "遥控避障：状态未知";
  badge.className = `badge ${known ? state.enabled ? "ok" : "danger" : ""}`;
  const button = document.getElementById("chassisAvoidanceButton");
  button.textContent = chassisAvoidanceBusy ? "正在切换…" : known ? state.enabled ? "关闭遥控避障" : "开启遥控避障" : "切换遥控避障";
  button.className = known && state.enabled ? "warning" : "";
  const moving = memoryExecution.active || movelExecution.active || movelBusy || graspExecution.active || graspBusy || scanExecution.active || scanBusy || placementExecution.active || placementBusy;
  button.disabled = chassisAvoidanceBusy || moving || !serverStatus?.armed || !known || !state.service_ready;
  const error = document.getElementById("chassisAvoidanceError");
  error.textContent = !state ? "需重启运控服务加载遥控避障开关" : state.error || (!state.service_ready ? "手动遥控避障服务未就绪" : "");
  error.hidden = !error.textContent;
}

async function toggleChassisAvoidance() {
  updateChassisAvoidance();
  if (document.getElementById("chassisAvoidanceButton").disabled) return;
  const enabled = !serverStatus.chassis.obstacle_avoidance.enabled;
  chassisAvoidanceBusy = true;
  updateChassisAvoidance();
  try {
    const result = await api("/api/chassis/obstacle-avoidance", {method: "POST", body: {enabled}});
    serverStatus.chassis = {...serverStatus.chassis, ...result};
    toast(`手动遥控避障已${enabled ? "开启" : "关闭"}`);
  } catch (error) {
    serverStatus.chassis.obstacle_avoidance = {...serverStatus.chassis.obstacle_avoidance, enabled:null, stale:true, error:"切换未确认，请等待状态回读"};
    toast(error.message, true);
  } finally {
    chassisAvoidanceBusy = false;
    updateChassisAvoidance();
  }
}

async function refreshStatus(quiet = true) {
  try { updateStatus(await api("/api/status")); }
  catch (error) {
    updateBattery(null, "连接中断");
    if (serverStatus?.chassis?.obstacle_avoidance) {
      serverStatus.chassis.obstacle_avoidance = {...serverStatus.chassis.obstacle_avoidance, enabled:null, stale:true, error:"连接中断，遥控避障状态未知"};
      updateChassisAvoidance();
    }
    if (!quiet) toast(error.message, true);
  }
}

async function readback() {
  const button = document.getElementById("readbackButton");
  button.disabled = true;
  try {
    applyReadback(await api("/api/readback", { method: "POST", body: {} }));
    toast("全身状态回读完成");
  } catch (error) { toast(error.message, true); }
  finally { button.disabled = false; }
}

async function moveJoints(module) {
  const spec = jointModules[module];
  const values = valuesFor("joint", module, spec.count);
  try {
    await api(`/api/joints/${module}`, { method: "POST", body: { values } });
    toast(`${spec.title}关节指令已接受`);
  } catch (error) { toast(error.message, true); }
}

async function movePose(module) {
  const values = valuesFor("pose", module, 6);
  const body = { values };
  if (module !== "trunk") {
    if (serverStatus?.pose_frames?.[module] !== armPoseFrames[module]) {
      toast("请重启控制服务后刷新网页并重新回读，手臂现使用肩部原点的 SDK 世界系", true);
      return;
    }
    body.frame = armPoseFrames[module];
    const elbow = document.getElementById(`poseElbow-${module}`).value.trim();
    if (!elbow) { toast("MoveJ 请填写目标臂角，或先回读", true); return; }
    body.elbow_deg = Number(elbow);
  }
  try {
    await api(`/api/pose/${module}`, { method: "POST", body });
    toast(`${poseModules[module]} 指令已接受`);
  } catch (error) { toast(error.message, true); }
}

async function moveLinear(module, mode) {
  if (movelBusy || movelExecution.active || memoryExecution.active) return;
  const elbow = document.getElementById(`poseElbow-${module}`).value.trim();
  const body = { values: valuesFor("pose", module, 6), frame: armPoseFrames[module],
    elbow_deg: elbow === "" ? null : Number(elbow) };
  movelBusy = true;
  updateMoveLControls();
  try {
    movelExecution = await api(`/api/movel/${module}/${mode}`, { method: "POST", body });
    toast(mode === "protected" ? "躯干保护 MoveL 规划已开始" : "MoveL 预检已开始");
  } catch (error) { toast(error.message, true); }
  finally { movelBusy = false; updateMoveLControls(); }
}

async function stopMoveL() {
  try {
    movelExecution = await api("/api/movel/stop", { method: "POST", body: {} });
    updateMoveLControls();
  } catch (error) { toast(error.message, true); }
}

async function setDrag(side, enabled) {
  const title = side === "left_arm" ? "左臂" : "右臂";
  try {
    await api(`/api/drag/${side}`, { method: "POST", body: { enabled } });
    document.getElementById(`dragText-${side}`).textContent = `当前：${enabled ? "拖拽待命，按住末端按钮生效" : "拖拽已关闭"}`;
    toast(`${title}拖拽已${enabled ? "进入按钮门控待命" : "关闭"}`);
  } catch (error) { toast(error.message, true); }
}

async function toggleArm() {
  try {
    if (serverStatus?.armed) {
      updateStatus(await api("/api/control/disarm", { method: "POST", body: {} }));
      toast("控制已锁定，底盘速度已置零");
    } else {
      updateStatus(await api("/api/control/arm", { method: "POST", body: {} }));
      toast("控制已解锁");
    }
  } catch (error) { toast(error.message, true); }
}

async function activateGripper() {
  if (gripperBusy) return;
  gripperBusy = true;
  updateGripperControls();
  try {
    updateGripperState(await api("/api/gripper/activate", { method: "POST", body: {} }));
    toast("夹爪初始化完成");
  } catch (error) { toast(error.message, true); }
  finally {
    gripperBusy = false;
    updateGripperControls();
    refreshGripperStatus(true);
  }
}

async function moveGripper(position) {
  if (gripperBusy || !Number.isInteger(position) || position < 0 || position > 255) return;
  gripperBusy = true;
  updateGripperControls();
  document.getElementById("gripperPositionSlider").value = position;
  document.getElementById("gripperPositionValue").textContent = position;
  try {
    const result = await api("/api/gripper/position", {
      method: "POST", body: { position },
    });
    updateGripperState(result);
    toast(`夹爪目标 ${position}，当前位置 ${result.measured_position}`);
  } catch (error) { toast(error.message, true); }
  finally {
    gripperBusy = false;
    updateGripperControls();
    refreshGripperStatus(true);
  }
}

async function saveSpeed() {
  if (speedSaving || !Number.isFinite(serverStatus?.rotation_deg_s)) return;
  const speed = Number(document.getElementById("speedInput").value);
  const rotation = Number(document.getElementById("rotationSpeedInput").value);
  const revision = speedEditRevision;
  speedSaving = true;
  updateSpeedFields(serverStatus);
  try {
    const result = await api("/api/speed", { method: "POST", body: { speed_mm_s: speed, rotation_deg_s: rotation } });
    Object.assign(serverStatus, result);
    if (speedEditRevision === revision) speedDirty = false;
    toast(`速度已保存：平移 ${result.speed_mm_s} mm/s，旋转 ${result.rotation_deg_s} °/s`);
  } catch (error) { toast(error.message, true); }
  finally { speedSaving = false; updateSpeedFields(serverStatus); }
}

async function toggleChassis() {
  const target = !chassisRemoteEnabled;
  try {
    const result = await api("/api/chassis/enable", { method: "POST", body: { enabled: target } });
    serverStatus.chassis = {...serverStatus.chassis, ...result};
    chassisRemoteEnabled = result.remote_enabled;
    updateChassisBadge();
    toast(`底盘遥控已${target ? "启用" : "禁用"}`);
  } catch (error) { toast(error.message, true); }
}

async function releaseChassisEmergencyStop() {
  const button = document.getElementById("chassisReleaseEmergencyButton");
  button.disabled = true;
  try {
    const result = await api("/api/chassis/release-emergency-stop", {
      method: "POST",
      body: {},
    });
    chassisRemoteEnabled = false;
    updateChassisBadge();
    toast(result.message || "底盘急停已解除，可重新启用底盘遥控");
  } catch (error) {
    toast(error.message, true);
  } finally {
    button.disabled = !serverStatus?.armed;
  }
}

function chassisVector(scale) {
  const linear = Math.abs(Number(document.getElementById("linearSpeed").value));
  const angular = Math.abs(Number(document.getElementById("angularSpeed").value));
  return { linear_x: scale[0] * linear, linear_y: scale[1] * linear, angular_z: scale[2] * angular };
}

async function sendChassis(scale) {
  try { await api("/api/chassis/command", { method: "POST", body: chassisVector(scale) }); }
  catch (error) { stopChassis(); toast(error.message, true); }
}

function startChassis(event) {
  event.preventDefault();
  if (!chassisRemoteEnabled) { toast("请先启用底盘遥控", true); return; }
  stopChassis(false);
  const scale = event.currentTarget.dataset.vector.split(",").map(Number);
  event.currentTarget.setPointerCapture?.(event.pointerId);
  sendChassis(scale);
  holdTimer = setInterval(() => sendChassis(scale), 150);
}

function stopChassis(send = true) {
  clearInterval(holdTimer);
  holdTimer = null;
  if (send) api("/api/chassis/stop", { method: "POST", body: {}, keepalive: true }).catch(() => {});
}

function bindEvents() {
  document.getElementById("memoryCreate").addEventListener("click", () => saveMemoryPoint());
  document.getElementById("memoryOverwrite").addEventListener("click", () => saveMemoryPoint(true));
  document.getElementById("memoryDelete").addEventListener("click", deleteMemoryPoint);
  document.getElementById("memoryExecute").addEventListener("click", executeMemoryPoint);
  document.getElementById("memoryStop").addEventListener("click", stopMemoryPoint);
  document.getElementById("memoryRefresh").addEventListener("click", () => refreshMemoryPoints());
  document.getElementById("memorySelect").addEventListener("change", updateMemoryControls);
  document.getElementById("memoryNameCancel").addEventListener("click", () => document.getElementById("memoryNameDialog").close());
  document.getElementById("memoryNameForm").addEventListener("submit", event => {
    event.preventDefault();
    saveMemoryPoint(false, document.getElementById("memoryNameInput").value);
  });
  document.getElementById("readbackButton").addEventListener("click", readback);
  document.getElementById("armButton").addEventListener("click", toggleArm);
  document.getElementById("gripperActivateButton").addEventListener("click", activateGripper);
  document.getElementById('suctionOpenButton').addEventListener('click', () => setSuction(true));
  document.getElementById('suctionCloseButton').addEventListener('click', () => setSuction(false));
  document.getElementById("gripperOpenButton").addEventListener("click", () => moveGripper(0));
  document.getElementById("gripperCloseButton").addEventListener("click", () => moveGripper(255));
  document.getElementById("gripperPositionSlider").addEventListener("input", event => {
    document.getElementById("gripperPositionValue").textContent = event.target.value;
  });
  document.getElementById("gripperPositionSlider").addEventListener("change", event => {
    moveGripper(Number(event.target.value));
  });
  document.getElementById("saveSpeedButton").addEventListener("click", saveSpeed);
  for (const id of ["speedInput", "rotationSpeedInput"]) {
    document.getElementById(id).addEventListener("input", () => {
      speedDirty = true;
      speedEditRevision += 1;
      if (serverStatus) updateSpeedFields(serverStatus);
    });
  }
  document.getElementById('poseEstimateButton').addEventListener('click', estimateGraspObjectPose);
  document.getElementById('poseTargetSelect').addEventListener('change', selectPoseTarget);
  document.getElementById("poseReprojectLeftButton").addEventListener("click", reprojectGraspToLeftShoulder);
  document.getElementById("poseReprojectButton").addEventListener("click", reprojectGraspToRightShoulder);
  document.getElementById("graspTestButton").addEventListener("click", executeGraspTest);
  document.getElementById("graspTestStop").addEventListener("click", stopGraspTest);
  document.getElementById("scanSequenceButton").addEventListener("click", executeScanSequence);
  document.getElementById("scanSequenceStop").addEventListener("click", stopScanSequence);
  document.getElementById("placementButton").addEventListener("click", () => executePlacement("bottle"));
  document.getElementById("tubePlacementButton").addEventListener("click", () => executePlacement("tube"));
  document.getElementById("boxPlacementButton").addEventListener("click", () => executePlacement("box"));
  document.getElementById("placementStop").addEventListener("click", stopPlacement);
  document.querySelectorAll("[data-camera-restart]").forEach(button => button.addEventListener("click", () => restartCamera()));
  document.querySelectorAll("[data-camera-frame]").forEach(button => button.addEventListener("click", () => getCameraFrame(button.dataset.cameraFrame)));
  document.querySelectorAll("[data-camera-rgb-record]").forEach(button => button.addEventListener("click", () => recordCameraRgb(button.dataset.cameraRgbRecord)));
  document.querySelectorAll("[data-camera-record]").forEach(button => button.addEventListener("click", () => recordCameraData(button.dataset.cameraRecord)));
  document.getElementById("chassisEnableButton").addEventListener("click", toggleChassis);
  document.getElementById("chassisAvoidanceButton").addEventListener("click", toggleChassisAvoidance);
  document.getElementById("chassisReleaseEmergencyButton").addEventListener("click", releaseChassisEmergencyStop);
  document.getElementById("chassisStopButton").addEventListener("click", () => stopChassis());
  document.querySelectorAll("[data-joint-move]").forEach(button => button.addEventListener("click", () => moveJoints(button.dataset.jointMove)));
  document.querySelectorAll("[data-pose-move]").forEach(button => button.addEventListener("click", () => movePose(button.dataset.poseMove)));
  document.querySelectorAll("[data-movel]").forEach(button => button.addEventListener("click", () => moveLinear(button.dataset.movel, button.dataset.mode)));
  document.querySelectorAll("[data-movel-stop]").forEach(button => button.addEventListener("click", stopMoveL));
  document.querySelectorAll("[data-drag]").forEach(button => button.addEventListener("click", () => setDrag(button.dataset.drag, button.dataset.enabled === "true")));
  document.querySelectorAll(".hold").forEach(button => {
    button.addEventListener("pointerdown", startChassis);
    button.addEventListener("pointerup", () => stopChassis());
    button.addEventListener("pointercancel", () => stopChassis());
    button.addEventListener("lostpointercapture", () => stopChassis());
  });
  window.addEventListener("blur", () => stopChassis());
  document.addEventListener("visibilitychange", () => { if (document.hidden) stopChassis(); });
  window.addEventListener("beforeunload", () => stopChassis());
}

renderCards();
bindEvents();
refreshStatus(false);
refreshMemoryPoints();
setInterval(() => { if (memoryExecution.active) refreshMemoryPoints(true); }, 700);
refreshGripperStatus(true);
checkPoseService(true);
setInterval(() => refreshStatus(true), 3000);
setInterval(() => {
  if (batteryExpiresAt && Date.now() >= batteryExpiresAt) updateBattery(null, "数据已过期");
}, 1000);
setInterval(() => refreshGripperStatus(true), 5000);
