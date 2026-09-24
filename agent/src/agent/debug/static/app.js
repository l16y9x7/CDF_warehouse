const REAL_TARGET_PASSWORD = "zhongmian123";
const state = { target: "mock", layer: "capability", catalog: [], selected: null, run: null, trace: null, traceRunId: null, detail: "result", logs: [], logCursor: null, selectedLog: null, selectedMediaKey: null, newestCaptureId: null, viewGeneration: 0, executing: false };
const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => [...document.querySelectorAll(selector)];
const json = (value) => JSON.stringify(value ?? null, null, 2);
const escapeHtml = (value) => String(value).replace(/[&<>"']/g, char => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[char]);
const formatDuration = (ms, empty = "—") => ms == null || !Number.isFinite(Number(ms)) ? empty : `${Number((Number(ms) / 1000).toFixed(3))} 秒`;
const nextViewGeneration = () => { state.viewGeneration += 1; return state.viewGeneration; };
const isCurrentView = (generation) => state.viewGeneration === generation;

async function api(path, options = {}) {
  const response = await fetch(path, { headers: { "Content-Type": "application/json" }, ...options });
  const data = response.status === 204 ? null : await response.json();
  if (!response.ok) throw new Error(data?.message || data?.detail || data?.error_code || `HTTP ${response.status}`);
  return data;
}

function toast(message) {
  const el = $("#toast"); el.textContent = message; el.classList.remove("hidden");
  clearTimeout(toast.timer); toast.timer = setTimeout(() => el.classList.add("hidden"), 3200);
}

function groups(items) {
  return items.reduce((acc, item) => ((acc[item.group] ||= []).push(item), acc), {});
}

function renderCatalog() {
  const term = $("#search").value.trim().toLowerCase();
  const items = state.catalog.filter(item => item.layer === state.layer && `${item.title} ${item.name} ${item.group}`.toLowerCase().includes(term));
  $("#catalog").innerHTML = Object.entries(groups(items)).map(([group, values]) => `
    <section class="catalog-group"><h3>${escapeHtml(group)}</h3>${values.map(item => `
      <button class="catalog-item ${state.selected?.name === item.name ? "active" : ""}" data-name="${escapeHtml(item.name)}" type="button">
        <span>${escapeHtml(item.title)}</span>${item.physical ? '<i class="physical-dot"></i>' : ""}
      </button>`).join("")}</section>`).join("");
  $$(".catalog-item").forEach(button => button.addEventListener("click", () => selectItem(button.dataset.name)));
}

function selectItem(name) {
  state.selected = state.catalog.find(item => item.layer === state.layer && item.name === name);
  renderCatalog(); renderForm();
  $(".catalog-panel").classList.remove("open");
}

function optionValue(field, option) {
  return field.type === "json" ? JSON.stringify(option) : String(option);
}

function inputFor(field, fieldId) {
  const required = field.required ? "required" : "";
  const value = field.default === undefined ? "" : field.type === "json" ? json(field.default) : String(field.default);
  if (field.type === "select") {
    const raw = [...(field.options || [])];
    if ((!field.required || field.default === undefined) && !raw.some(option => optionValue(field, option) === "")) raw.unshift("");
    const options = raw.map(option => {
      const opt = optionValue(field, option);
      const label = opt || (field.required ? "请选择" : "（空）");
      return `<option value="${escapeHtml(opt)}"${opt === String(value) ? " selected" : ""}>${escapeHtml(label)}</option>`;
    }).join("");
    return `<select id="${fieldId}" name="${escapeHtml(field.name)}" ${required}>${options}</select>`;
  }
  const choices = (field.options || []).map(option => optionValue(field, option));
  const placeholder = field.placeholder || "请选择或输入自定义值";
  const menu = choices.length ? `<button class="select-toggle" type="button" tabindex="-1" aria-label="打开选项">▾</button><ul class="select-menu hidden">${choices.map(opt => `<li data-value="${escapeHtml(opt)}">${escapeHtml(opt)}</li>`).join("")}</ul>` : "";
  return `<div class="editable-select"><input id="${fieldId}" name="${escapeHtml(field.name)}" type="${field.type === "number" ? "number" : "text"}" value="${escapeHtml(value)}" placeholder="${escapeHtml(placeholder)}" autocomplete="off" ${required}>${menu}</div>`;
}

function renderForm() {
  const item = state.selected;
  $("#selection-empty").classList.toggle("hidden", !!item);
  $("#operation").classList.toggle("hidden", !item);
  if (!item) return;
  $("#operation-path").textContent = `${item.layer} / ${item.name}`;
  $("#operation-title").textContent = item.title;
  $("#operation-description").textContent = item.description;
  $("#physical-badge").classList.toggle("hidden", !item.physical);
  if (!state.executing) setRunButtonIdle();
  $("#fields").innerHTML = item.fields.map((field, index) => {
    const fieldId = `field-${item.layer}-${item.name.replace(/[^a-zA-Z0-9_-]/g, "-")}-${index}`;
    return `<div class="field ${field.type === "json" ? "wide" : ""}"><label for="${fieldId}"><span class="field-title">${escapeHtml(field.label)} <code>${escapeHtml(field.name)}</code></span><span>${field.required ? "必填" : "可选"}</span></label><p class="field-description">${escapeHtml(field.description || "")}</p>${inputFor(field, fieldId)}</div>`;
  }).join("") || '<div class="field wide no-parameters"><strong>入参说明</strong><p>此操作无需参数。</p></div>';
  const testCaseInput = $('[name="test_case"]');
  if (testCaseInput) {
    const handler = () => applyTestCase(testCaseInput.value);
    testCaseInput.addEventListener("change", handler);
    testCaseInput.addEventListener("input", handler);
  }
  bindEditableSelects();
}

function closeSelectMenus(except = null) {
  $$(".select-menu").forEach(menu => { if (menu !== except) menu.classList.add("hidden"); });
}

function bindEditableSelects() {
  $$(".editable-select").forEach(wrap => {
    const input = wrap.querySelector("input");
    const toggle = wrap.querySelector(".select-toggle");
    const menu = wrap.querySelector(".select-menu");
    if (!input || !menu) return;
    const open = () => { closeSelectMenus(menu); menu.classList.remove("hidden"); };
    const pick = value => {
      input.value = value;
      input.dispatchEvent(new Event("input", { bubbles: true }));
      input.dispatchEvent(new Event("change", { bubbles: true }));
      menu.classList.add("hidden");
    };
    input.addEventListener("focus", open);
    input.addEventListener("click", open);
    toggle?.addEventListener("mousedown", event => event.preventDefault());
    toggle?.addEventListener("click", event => {
      event.preventDefault();
      menu.classList.contains("hidden") ? open() : menu.classList.add("hidden");
      input.focus();
    });
    menu.querySelectorAll("[data-value]").forEach(item => item.addEventListener("mousedown", event => {
      event.preventDefault();
      pick(item.dataset.value);
    }));
  });
}

let testCaseMeta = null;
async function applyTestCase(value) {
  if (!value || value.startsWith("(")) return;
  try {
    testCaseMeta = testCaseMeta || (await api("/debug/api/test-cases")).cases || [];
  } catch (error) {
    toast(error.message); return;
  }
  const item = testCaseMeta.find(entry => entry.name === value);
  if (!item) return;
  const skip = new Set(["test_case", "target_type", "sku_typ", "side"]);
  for (const field of state.selected.fields) {
    if (skip.has(field.name) || !(field.name in item) || item[field.name] == null) continue;
    const input = $(`[name="${field.name}"]`);
    if (!input) continue;
    input.value = field.type === "json" ? json(item[field.name]) : String(item[field.name]);
  }
}

function payloadFromForm() {
  const payload = {};
  for (const field of state.selected.fields) {
    const input = $(`[name="${field.name}"]`); let value = input.value;
    if (!value && !field.required) continue;
    if (field.type === "json") { try { value = JSON.parse(value); } catch { throw new Error(`${field.label} 不是合法 JSON`); } }
    if (field.type === "number") value = Number(value);
    payload[field.name] = value;
  }
  return payload;
}

function setRunButtonRunning() {
  const button = $("#run-button");
  state.executing = true;
  button.disabled = true;
  button.classList.add("is-running");
  button.textContent = "执行中…";
}

function setRunButtonIdle() {
  const button = $("#run-button");
  state.executing = false;
  button.disabled = false;
  button.classList.remove("is-running");
  button.textContent = state.selected?.layer === "workflow" ? "提交 Workflow" : "执行";
}

function beginPendingRun() {
  const generation = nextViewGeneration();
  state.run = null;
  state.trace = null;
  state.traceRunId = null;
  state.selectedMediaKey = null;
  state.newestCaptureId = null;
  $("#terminate-button").classList.add("hidden");
  $("#related-logs").classList.add("hidden");
  const badge = $("#result-status");
  badge.textContent = "RUNNING";
  badge.className = "status-badge running";
  $("#metrics").innerHTML = `<div class="metric"><span>目标</span><strong>${escapeHtml(state.target)}</strong></div><div class="metric"><span>层级</span><strong>${escapeHtml(state.selected?.layer || "—")}</strong></div><div class="metric"><span>耗时</span><strong>进行中</strong></div>`;
  $("#media-preview").classList.add("hidden");
  $("#trace-view").innerHTML = '<div class="trace-empty">正在执行…</div>';
  $("#detail-json").textContent = "正在执行…";
  highlightHistory();
  return generation;
}

async function execute(payload, confirmationToken = null, generation = state.viewGeneration) {
  const item = state.selected;
  let run;
  if (item.layer === "workflow") {
    run = await api(`/debug/api/workflows/${item.name}`, { method: "POST", body: JSON.stringify({ target: state.target, payload }) });
  } else {
    run = await api("/debug/api/runs", { method: "POST", body: JSON.stringify({ target: state.target, layer: item.layer, operation: item.name, payload, confirmation_token: confirmationToken }) });
  }
  if (!isCurrentView(generation)) return;
  showRun(run, generation); await loadHistory();
  if (item.layer === "workflow" && ["ACCEPTED", "RUNNING"].includes(run.status) && isCurrentView(generation)) pollRun(run.run_id, generation);
}

async function executePhysical(payload, generation = state.viewGeneration) {
  const ticket = await api("/debug/api/confirmations", { method: "POST", body: JSON.stringify({ target: state.target, layer: state.selected.layer, operation: state.selected.name, payload }) });
  if (!isCurrentView(generation)) return;
  await execute(payload, ticket.token, generation);
}

function statusClass(status) {
  if (status === "SUCCEEDED") return "success";
  if (status === "CANCELLED") return "cancelled";
  if (["FAILED", "WAITING_CONFIRMATION"].includes(status)) return "failed";
  if (["RUNNING", "ACCEPTED"].includes(status)) return "running";
  return "neutral";
}

function showRun(run, generation = state.viewGeneration) {
  if (!isCurrentView(generation)) return;
  if (state.run?.run_id !== run.run_id) {
    state.selectedMediaKey = null;
    state.newestCaptureId = null;
    state.trace = null;
    state.traceRunId = null;
  }
  state.run = run;
  const canTerminate = run.layer === "workflow" && ["ACCEPTED", "RUNNING"].includes(run.status) && run.target === state.target;
  $("#terminate-button").classList.toggle("hidden", !canTerminate);
  $("#terminate-button").disabled = false;
  $("#related-logs").classList.toggle("hidden", !(run.run_id || run.task_id));
  const badge = $("#result-status"); badge.textContent = run.status; badge.className = `status-badge ${statusClass(run.status)}`;
  $("#metrics").innerHTML = `<div class="metric"><span>目标</span><strong>${escapeHtml(run.target)}</strong></div><div class="metric"><span>层级</span><strong>${escapeHtml(run.layer)}</strong></div><div class="metric"><span>耗时</span><strong>${formatDuration(run.duration_ms, "进行中")}</strong></div>${run.task_id ? `<div class="metric"><span>任务</span><strong>${escapeHtml(run.task_id)}</strong></div>` : ""}`;
  renderDetail(); renderMedia(run); highlightHistory();
  loadTrace(run.run_id, generation).catch(error => toast(error.message));
}

function renderDetail() {
  const run = state.run;
  const traceMode = state.detail === "trace";
  $("#trace-view").classList.toggle("hidden", !traceMode);
  $("#detail-json").classList.toggle("hidden", traceMode);
  if (!run) {
    if (traceMode) $("#trace-view").innerHTML = '<div class="trace-empty">正在执行…</div>';
    else $("#detail-json").textContent = "正在执行…";
    return;
  }
  if (traceMode) { renderTrace(); return; }
  let value = run.result;
  if (state.detail === "events") value = run.events || [];
  if (state.detail === "request") value = run.request;
  if (run.error_code && state.detail === "result") value = { error_code: run.error_code, message: run.message };
  $("#detail-json").textContent = json(value);
}

async function loadTrace(runId, generation = state.viewGeneration) {
  const trace = await api(`/debug/api/runs/${runId}/trace`);
  if (!isCurrentView(generation) || state.run?.run_id !== runId) return;
  state.trace = trace;
  state.traceRunId = runId;
  if (state.detail === "trace") renderTrace();
}

function spanLabel(span) {
  const labels = { workflow: "Workflow", skill: "Skill", capability: "Capability", preflight: "执行前检查" };
  return `${labels[span.kind] || span.kind} · ${span.name}${span.operation && span.operation !== "execute" && span.operation !== "run" ? `.${span.operation}` : ""}`;
}

function errorSourceLabel(source) {
  return { remote: "接口错误", transport: "传输失败", local: "能力内部" }[source] || "";
}

function spanError(span) {
  if (!span.error_message) return null;
  const error = {
    code: span.error_code,
    type: span.error_type,
    message: span.error_message,
    stack: span.error_stack,
  };
  if (span.error_source) error.source = span.error_source;
  if (span.http_status != null) error.http_status = span.http_status;
  if (span.error_operation) error.operation = span.error_operation;
  return error;
}

function spanDetails(span) {
  const failed = span.status === "FAILED";
  const sourceLabel = failed ? errorSourceLabel(span.error_source) : "";
  const sourceBadge = sourceLabel
    ? `<span class="source-badge source-${escapeHtml(span.error_source)}">${escapeHtml(sourceLabel)}</span>`
    : "";
  const detail = {
    input: span.input,
    output: span.output,
    error: spanError(span),
  };
  return `<div class="trace-node-wrap"><details class="trace-span trace-${escapeHtml(span.kind)} ${failed ? "trace-failed" : ""}">
    <summary><span class="trace-name">${escapeHtml(spanLabel(span))}</span>${span.node_id ? `<span class="trace-node">节点 ${escapeHtml(span.node_id)}</span>` : ""}${sourceBadge}<span class="status-badge ${statusClass(span.status)}">${escapeHtml(span.status)}</span><span class="trace-duration">${formatDuration(span.duration_ms, "进行中")}</span></summary>
    <pre>${escapeHtml(json(detail))}</pre>
  </details><div class="trace-children" data-trace-parent="${escapeHtml(span.span_id)}"></div></div>`;
}

function traceTree(spans, parentId = null) {
  return spans.filter(span => (span.parent_span_id || null) === parentId).map(span => {
    const html = spanDetails(span);
    const children = traceTree(spans, span.span_id);
    return html.replace(`<div class="trace-children" data-trace-parent="${escapeHtml(span.span_id)}"></div>`, `<div class="trace-children">${children}</div>`);
  }).join("");
}

function renderTrace() {
  const el = $("#trace-view");
  if (!state.trace || state.traceRunId !== state.run?.run_id) {
    el.innerHTML = '<div class="trace-empty">正在加载调用链…</div>';
    return;
  }
  const spans = state.trace.spans || [];
  if (!spans.length) {
    el.innerHTML = `<div class="trace-empty">${state.trace.expired ? "调用追踪已过保留期" : "本次运行暂无调用追踪"}</div>`;
    return;
  }
  const preflightIds = new Set(spans.filter(span => span.kind === "preflight").map(span => span.span_id));
  const preflight = spans.filter(span => span.kind === "preflight" || preflightIds.has(span.parent_span_id));
  const business = spans.filter(span => !preflight.includes(span));
  el.innerHTML = `${preflight.length ? `<section class="trace-section"><h3>执行前检查</h3>${traceTree(preflight)}</section>` : ""}<section class="trace-section"><h3>业务调用</h3>${traceTree(business)}</section>`;
}

function clearPoseOverlay() {
  const canvas = $("#preview-overlay");
  if (!canvas) return;
  const context = canvas.getContext("2d");
  if (context) context.clearRect(0, 0, canvas.width, canvas.height);
}

function containedImageRect(image) {
  const containerW = image.clientWidth;
  const containerH = image.clientHeight;
  const naturalWidth = image.naturalWidth || 0;
  const naturalHeight = image.naturalHeight || 0;
  if (!containerW || !containerH || !naturalWidth || !naturalHeight) return null;
  const scale = Math.min(containerW / naturalWidth, containerH / naturalHeight);
  const width = naturalWidth * scale;
  const height = naturalHeight * scale;
  return { x: (containerW - width) / 2, y: (containerH - height) / 2, width, height, scale, naturalWidth, naturalHeight };
}

function projectCameraPoint(K, xyz) {
  if (!Array.isArray(K) || K.length < 2 || !Array.isArray(xyz) || xyz.length < 3) return null;
  const z = Number(xyz[2]);
  if (!(z > 1e-6)) return null;
  const fx = Number(K[0]?.[0]), cx = Number(K[0]?.[2]), fy = Number(K[1]?.[1]), cy = Number(K[1]?.[2]);
  if (![fx, fy, cx, cy].every(Number.isFinite)) return null;
  return [fx * Number(xyz[0]) / z + cx, fy * Number(xyz[1]) / z + cy];
}

function overlayToCanvas(rect, uv) {
  if (!rect || !uv) return null;
  return [rect.x + uv[0] * rect.scale, rect.y + uv[1] * rect.scale];
}

function drawOverlayDot(context, point, color) {
  context.fillStyle = color;
  context.strokeStyle = "#041016";
  context.lineWidth = 2;
  context.beginPath();
  context.arc(point[0], point[1], 5, 0, Math.PI * 2);
  context.fill();
  context.stroke();
}

function drawOverlayBox(context, rect, box, kind, color) {
  const values = Array.isArray(box) ? box.map(Number) : [];
  if (values.length !== 4 || values.some(value => !Number.isFinite(value))) return;
  const [a, b, c, d] = values;
  const xyxy = kind === "xywh" ? [a, b, a + c, b + d] : [a, b, c, d];
  const topLeft = overlayToCanvas(rect, [xyxy[0], xyxy[1]]);
  const bottomRight = overlayToCanvas(rect, [xyxy[2], xyxy[3]]);
  if (!topLeft || !bottomRight) return;
  context.strokeStyle = color;
  context.lineWidth = 2;
  context.strokeRect(topLeft[0], topLeft[1], bottomRight[0] - topLeft[0], bottomRight[1] - topLeft[1]);
}

function drawPoseOverlay(image, overlay) {
  const canvas = $("#preview-overlay");
  if (!canvas) return;
  const context = canvas.getContext("2d");
  const stage = image.parentElement;
  if (!context || !stage) return;
  const width = Math.max(1, Math.round(stage.clientWidth));
  const height = Math.max(1, Math.round(stage.clientHeight));
  if (canvas.width !== width) canvas.width = width;
  if (canvas.height !== height) canvas.height = height;
  context.clearRect(0, 0, canvas.width, canvas.height);
  if (!overlay || image.classList.contains("hidden")) return;
  const rect = containedImageRect(image);
  if (!rect) return;
  const K = overlay.K;
  for (const box of overlay.boxes || []) {
    drawOverlayBox(context, rect, box.box, box.kind, box.color || "#fb923c");
  }
  for (const line of overlay.lines || []) {
    const points = (line.xyz_mm || []).map(xyz => overlayToCanvas(rect, projectCameraPoint(K, xyz))).filter(Boolean);
    if (points.length < 2) continue;
    context.strokeStyle = line.color || "#fbbf24";
    context.lineWidth = 2;
    context.beginPath();
    context.moveTo(points[0][0], points[0][1]);
    points.slice(1).forEach(point => context.lineTo(point[0], point[1]));
    context.stroke();
  }
  for (const point of overlay.points || []) {
    const mapped = overlayToCanvas(rect, projectCameraPoint(K, point.xyz_mm));
    if (mapped) drawOverlayDot(context, mapped, point.color || "#22d3ee");
  }
  for (const marker of overlay.markers || []) {
    const mapped = overlayToCanvas(rect, marker.xy);
    if (mapped) drawOverlayDot(context, mapped, marker.color || "#facc15");
  }
}

function bindPoseOverlayResize() {
  const stage = $(".media-stage");
  if (!stage || typeof ResizeObserver === "undefined" || stage._poseOverlayBound) return;
  stage._poseOverlayBound = true;
  new ResizeObserver(() => {
    const selected = (state.run?.media || []).find(item => `${item.capture_id}|${item.stream}|${item.path}` === state.selectedMediaKey);
    if (selected?.overlay) drawPoseOverlay($("#preview-image"), selected.overlay);
  }).observe(stage);
}

function renderMedia(run) {
  const media = Array.isArray(run?.media) ? run.media.filter(item => item && typeof item.path === "string") : [];
  $("#media-preview").classList.toggle("hidden", media.length === 0);
  if (!media.length) {
    state.selectedMediaKey = null;
    state.newestCaptureId = null;
    clearPoseOverlay();
    return;
  }

  const keyFor = item => `${item.capture_id}|${item.stream}|${item.path}`;
  const newestCaptureId = media[media.length - 1].capture_id;
  if (!state.selectedMediaKey || state.newestCaptureId !== newestCaptureId || !media.some(item => keyFor(item) === state.selectedMediaKey)) {
    const newest = media.filter(item => item.capture_id === newestCaptureId);
    state.selectedMediaKey = keyFor(newest.find(item => item.stream === "color") || newest[0]);
  }
  state.newestCaptureId = newestCaptureId;
  const selected = media.find(item => keyFor(item) === state.selectedMediaKey) || media[0];
  const image = $("#preview-image");
  const source = `/debug/api/media?path=${encodeURIComponent(selected.path)}`;
  $("#media-error").classList.add("hidden");
  image.classList.remove("hidden");
  image.alt = `${selected.camera} ${selected.stream === "color" ? "彩色图" : "深度图"}`;
  const paint = () => {
    image.classList.remove("hidden");
    $("#media-error").classList.add("hidden");
    drawPoseOverlay(image, selected.overlay);
  };
  image.onload = paint;
  image.onerror = () => { image.classList.add("hidden"); $("#media-error").classList.remove("hidden"); clearPoseOverlay(); };
  if (image.getAttribute("src") !== source) image.src = source;
  else if (image.complete && image.naturalWidth) paint();
  bindPoseOverlayResize();
  $("#media-title").textContent = `${selected.camera} · ${selected.stream === "color" ? "彩色图" : "深度图"}`;
  $("#media-meta").textContent = [selected.capture_id, selected.skill, selected.width && selected.height ? `${selected.width} × ${selected.height}` : null, selected.format].filter(Boolean).join(" · ");
  $("#media-thumbnails").innerHTML = media.map((item, index) => `
    <button type="button" role="listitem" data-media-index="${index}" class="media-thumbnail ${keyFor(item) === state.selectedMediaKey ? "active" : ""}" aria-label="${escapeHtml(`${item.camera} ${item.stream === "color" ? "彩色图" : "深度图"}`)}">
      <img src="/debug/api/media?path=${encodeURIComponent(item.path)}" alt="" loading="lazy">
      <span>${escapeHtml(item.camera)} · ${item.stream === "color" ? "彩色" : "深度"}</span>
    </button>`).join("");
  $$("#media-thumbnails [data-media-index]").forEach(button => button.addEventListener("click", () => {
    state.selectedMediaKey = keyFor(media[Number(button.dataset.mediaIndex)]);
    renderMedia(state.run);
  }));
}

async function pollRun(runId, generation = state.viewGeneration) {
  for (let count = 0; count < 180; count++) {
    await new Promise(resolve => setTimeout(resolve, 1000));
    if (!isCurrentView(generation)) return;
    const run = await api(`/debug/api/runs/${runId}`);
    if (!isCurrentView(generation)) return;
    showRun(run, generation);
    if (!["ACCEPTED", "RUNNING"].includes(run.status)) { await loadHistory(); return; }
  }
}

async function terminateWorkflow() {
  const button = $("#terminate-button");
  button.disabled = true;
  try {
    const result = await api("/debug/api/terminate", { method: "POST", body: JSON.stringify({ target: state.target }) });
    toast("已请求终止任务 " + result.task_id);
  } catch (error) {
    button.disabled = false;
    throw error;
  }
}

function highlightHistory() {
  $$("#history-body tr[data-id]").forEach(row => row.classList.toggle("active", row.dataset.id === state.run?.run_id));
}

async function selectHistoryRun(runId) {
  const generation = nextViewGeneration();
  $$("#history-body tr[data-id]").forEach(row => row.classList.toggle("active", row.dataset.id === runId));
  try {
    const run = await api(`/debug/api/runs/${runId}`);
    showRun(run, generation);
    if (["ACCEPTED", "RUNNING"].includes(run.status) && isCurrentView(generation)) pollRun(run.run_id, generation);
  } catch (error) {
    toast(error.message);
  }
}

async function loadHistory() {
  const params = new URLSearchParams({ limit: "100" });
  if ($("#history-layer").value) params.set("layer", $("#history-layer").value);
  if ($("#history-status").value) params.set("status", $("#history-status").value);
  const data = await api(`/debug/api/runs?${params}`);
  $("#history-count").textContent = `${data.runs.length} 条`;
  $("#history-body").innerHTML = data.runs.map(run => `<tr data-id="${run.run_id}" class="${state.run?.run_id === run.run_id ? "active" : ""}"><td>${new Date(run.started_at * 1000).toLocaleString()}</td><td>${run.target}</td><td>${run.layer}</td><td>${run.operation}</td><td><span class="status-badge ${statusClass(run.status)}">${run.status}</span></td><td>${formatDuration(run.duration_ms)}</td></tr>`).join("") || '<tr><td colspan="6">暂无执行记录</td></tr>';
  $$("#history-body tr[data-id]").forEach(row => row.addEventListener("click", () => selectHistoryRun(row.dataset.id)));
}

function logParams(cursor = null) {
  const params = new URLSearchParams({ limit: "100" });
  const values = { level: $("#log-level").value, event: $("#log-event").value.trim(), component: $("#log-component").value.trim(), task_id: $("#log-task-id").value.trim(), run_id: $("#log-run-id").value.trim(), search: $("#log-search").value.trim() };
  Object.entries(values).forEach(([key, value]) => { if (value) params.set(key, value); });
  if (cursor) params.set("cursor", cursor);
  const from = $("#log-from").value;
  const to = $("#log-to").value;
  params.set("from", String(from ? new Date(from).getTime() / 1000 : Date.now() / 1000 - 3600));
  if (to) params.set("to", String(new Date(to).getTime() / 1000));
  return params;
}

function renderLogs() {
  $("#logs-count").textContent = `${state.logs.length} 条` + (state.logCursor ? "，还有更多" : "");
  $("#logs-body").innerHTML = state.logs.map((entry, index) => `<tr data-log-index="${index}"><td>${new Date(entry.timestamp).toLocaleString()}</td><td><span class="log-level ${String(entry.level).toLowerCase()}">${escapeHtml(entry.level)}</span></td><td class="log-message">${escapeHtml(entry.message)}${entry.error_message ? `<small>${escapeHtml(entry.error_message)}</small>` : ""}</td><td><code>${escapeHtml(entry.event)}</code></td><td>${escapeHtml(entry.task_id || entry.run_id || entry.request_id || "—")}</td><td>${formatDuration(entry.duration_ms)}</td></tr>`).join("") || '<tr><td colspan="6">最近一小时暂无匹配日志</td></tr>';
  $("#load-more-logs").classList.toggle("hidden", !state.logCursor);
  $$("#logs-body tr[data-log-index]").forEach(row => row.addEventListener("click", () => {
    state.selectedLog = state.logs[Number(row.dataset.logIndex)];
    $("#log-detail").textContent = json(state.selectedLog);
    $("#log-dialog").showModal();
  }));
}

async function loadLogs(append = false) {
  const data = await api(`/debug/api/logs?${logParams(append ? state.logCursor : null)}`);
  state.logs = append ? [...state.logs, ...data.logs] : data.logs;
  state.logCursor = data.next_cursor;
  renderLogs();
}

async function showRelatedLogs() {
  $("#log-task-id").value = state.run?.task_id || "";
  $("#log-run-id").value = state.run?.run_id || "";
  await loadLogs();
  $("#logs-section").scrollIntoView({ behavior: "smooth" });
}

async function refreshTargets() {
  const data = await api("/debug/api/targets"); const target = data.targets.find(item => item.name === state.target);
  if (!target) throw new Error("当前运行目标不可用");
  const ready = Object.values(target.modules).every(value => value === "READY");
  const el = $("#target-state"); el.className = `target-state ${ready ? "ready" : "error"}`; el.innerHTML = `<span></span>${ready ? "全部就绪" : "部分离线"}`;
}

function bind() {
  $$(".target-switch button").forEach(button => button.addEventListener("click", async () => {
    if (button.dataset.target === "real" && state.target !== "real") {
      const password = window.prompt("请输入实机操作密码");
      if (password === null) return;
      if (password !== REAL_TARGET_PASSWORD) { toast("实机操作密码错误"); return; }
    }
    state.target = button.dataset.target; $$(".target-switch button").forEach(item => item.classList.toggle("active", item === button));
    $("#real-warning").classList.toggle("hidden", state.target !== "real"); await Promise.all([refreshTargets(), loadHistory()]);
  }));
  $$("#layer-tabs button").forEach(button => button.addEventListener("click", () => { state.layer = button.dataset.layer; state.selected = null; $$("#layer-tabs button").forEach(item => item.classList.toggle("active", item === button)); renderCatalog(); renderForm(); }));
  $("#search").addEventListener("input", renderCatalog);
  $("#run-form").addEventListener("submit", async event => {
    event.preventDefault();
    if (state.executing) return;
    let payload;
    try { payload = payloadFromForm(); } catch (error) { toast(error.message); return; }
    const generation = beginPendingRun();
    setRunButtonRunning();
    try {
      state.selected.physical && state.selected.layer !== "workflow" ? await executePhysical(payload, generation) : await execute(payload, null, generation);
    } catch (error) {
      if (isCurrentView(generation)) {
        const badge = $("#result-status");
        badge.textContent = "FAILED";
        badge.className = "status-badge failed";
        $("#detail-json").textContent = json({ message: error.message });
      }
      toast(error.message);
    } finally {
      setRunButtonIdle();
    }
  });
  $("#reset-button").addEventListener("click", renderForm);
  $("#terminate-button").addEventListener("click", () => terminateWorkflow().catch(error => toast(error.message)));
  $("#related-logs").addEventListener("click", () => showRelatedLogs().catch(error => toast(error.message)));
  $$(".detail-tabs button").forEach(button => button.addEventListener("click", () => { state.detail = button.dataset.detail; $$(".detail-tabs button").forEach(item => item.classList.toggle("active", item === button)); renderDetail(); }));
  $("#refresh-history").addEventListener("click", loadHistory); $("#history-layer").addEventListener("change", loadHistory); $("#history-status").addEventListener("change", loadHistory);
  $("#mobile-open").addEventListener("click", () => $(".catalog-panel").classList.add("open")); $("#mobile-close").addEventListener("click", () => $(".catalog-panel").classList.remove("open"));
  $("#refresh-logs").addEventListener("click", () => loadLogs().catch(error => toast(error.message)));
  $("#load-more-logs").addEventListener("click", () => loadLogs(true).catch(error => toast(error.message)));
  ["#log-level", "#log-event", "#log-component", "#log-task-id", "#log-run-id", "#log-from", "#log-to"].forEach(selector => $(selector).addEventListener("change", () => loadLogs().catch(error => toast(error.message))));
  $("#log-search").addEventListener("keydown", event => { if (event.key === "Enter") loadLogs().catch(error => toast(error.message)); });
  $("#close-log").addEventListener("click", () => $("#log-dialog").close());
  $("#copy-log").addEventListener("click", async () => { await navigator.clipboard.writeText(json(state.selectedLog)); toast("日志已复制"); });
  document.addEventListener("mousedown", event => { if (!event.target.closest(".editable-select")) closeSelectMenus(); });
  setInterval(() => { if ($("#log-auto-refresh").checked) loadLogs().catch(error => toast(error.message)); }, 5000);
  setInterval(() => { loadHistory().catch(() => {}); }, 2000);
}

async function init() {
  bind();
  const catalog = await api("/debug/api/catalog");
  state.catalog = catalog.items;
  renderCatalog();
  loadHistory().catch(error => toast(error.message));
  loadLogs().catch(error => toast(error.message));
  refreshTargets().catch(error => toast(error.message));
}
init().catch(error => toast(error.message));
