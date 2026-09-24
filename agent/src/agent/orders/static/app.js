const $ = selector => document.querySelector(selector);
const state = { products: [], cart: new Map(), locations: new Map(), order: null, stream: null, builder: true, mock: false, steps: [], seenEventIds: new Set(), stepsOrderId: null };
const terminal = new Set(["SUCCEEDED", "CANCELLED"]);

async function api(path, options = {}) {
  const response = await fetch(path, { headers: { "Content-Type": "application/json", ...(options.headers || {}) }, ...options });
  if (response.status === 204) return null;
  const body = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(body.message || body.error_code || "请求失败");
  return body;
}

function escapeHtml(value) {
  return String(value ?? "").replace(/[&<>'"]/g, char => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", "'": "&#39;", '"': "&quot;" }[char]));
}

function toast(message, error = false) {
  const el = $("#toast"); el.textContent = message; el.className = `toast${error ? " error" : ""}`;
  clearTimeout(toast.timer); toast.timer = setTimeout(() => el.classList.add("hidden"), 2800);
}

function renderProducts() {
  $("#product-count").textContent = `${state.products.length} 件可选商品`;
  $("#product-grid").innerHTML = state.products.map(product => `
    <article class="product-card">
      <div class="product-image"><img src="${escapeHtml(product.image_url)}" alt="${escapeHtml(product.name)}"></div>
      <div class="product-location">
        <span>抓取位置</span>
        <label>行<select data-location-row="${escapeHtml(product.sku_id)}"><option value="">选择</option>${[1,2,3,4,5].map(value => `<option value="L${value}">L${value}</option>`).join("")}</select></label>
        <label>列<select data-location-column="${escapeHtml(product.sku_id)}"><option value="">选择</option><option value="1">1</option><option value="2">2</option></select></label>
      </div>
      <div class="product-meta">
        <span class="product-category">${escapeHtml(product.category)}</span>
        <h3 title="${escapeHtml(product.name)}">${escapeHtml(product.name)}</h3>
        <p>${escapeHtml(product.description)}</p>
        <div class="product-action"><span class="sku">SKU ${escapeHtml(product.sku_id)}</span><button class="add-button" data-sku="${escapeHtml(product.sku_id)}" aria-label="添加 ${escapeHtml(product.name)}">＋</button></div>
      </div>
    </article>`).join("") || '<div class="empty-cart"><strong>暂无可下单商品</strong><p>请检查商品目录与 SKU 配置</p></div>';
  document.querySelectorAll(".add-button").forEach(button => button.onclick = () => changeQuantity(button.dataset.sku, 1));
  document.querySelectorAll("[data-location-row], [data-location-column]").forEach(select => select.onchange = () => {
    const sku = select.dataset.locationRow || select.dataset.locationColumn;
    const row = document.querySelector(`[data-location-row="${sku}"]`).value;
    const column = document.querySelector(`[data-location-column="${sku}"]`).value;
    state.locations.set(sku, { row, column });
    const cartItem = state.cart.get(sku);
    if (cartItem) { cartItem.agv_row = row; cartItem.agv_column = column; renderCart(); }
  });
}

function changeQuantity(sku, delta) {
  const current = state.cart.get(sku);
  const location = state.locations.get(sku) || {};
  if (delta > 0 && (!location.row || !location.column)) { toast("请先选择该商品的 AGV 抓取行列", true); return; }
  const next = (current?.quantity || 0) + delta;
  if (next <= 0) state.cart.delete(sku);
  else state.cart.set(sku, { quantity: Math.min(next, 20), agv_row: location.row, agv_column: location.column });
  renderCart();
}

function renderCart() {
  const entries = [...state.cart.entries()];
  const total = entries.reduce((sum, [, item]) => sum + item.quantity, 0);
  $("#cart-count").textContent = total;
  $("#total-items").textContent = total;
  $("#submit-order").disabled = total === 0;
  $("#empty-cart").classList.toggle("hidden", entries.length > 0);
  $("#cart-items").innerHTML = entries.map(([sku, item]) => {
    const product = state.products.find(item => item.sku_id === sku);
    return `<div class="cart-row"><img src="${escapeHtml(product.image_url)}" alt=""><div><strong>${escapeHtml(product.name)}</strong><small>AGV ${escapeHtml(item.agv_row)} · ${escapeHtml(item.agv_column)}列</small></div><div class="quantity"><button data-delta="-1" data-sku="${escapeHtml(sku)}">−</button><b>${item.quantity}</b><button data-delta="1" data-sku="${escapeHtml(sku)}">＋</button></div></div>`;
  }).join("");
  document.querySelectorAll(".quantity button").forEach(button => button.onclick = () => changeQuantity(button.dataset.sku, Number(button.dataset.delta)));
}

async function submitOrder() {
  const button = $("#submit-order"); button.disabled = true; button.querySelector("span").textContent = "正在创建订单…";
  try {
    const order = await api("/orders/api/orders", { method: "POST", body: JSON.stringify({ items: [...state.cart].map(([sku_id, item]) => ({ sku_id, quantity: item.quantity, agv_row: item.agv_row, agv_column: item.agv_column })), basket_row: $("#basket-row").value, basket_column: $("#basket-column").value, mock: state.mock }) });
    state.order = order; state.builder = false; showOrder(); connectEvents(order.order_id);
  } catch (error) { toast(error.message, true); button.disabled = false; }
  finally { button.querySelector("span").textContent = "确认下单"; }
}

const statusText = { RUNNING: "运行中", PAUSED: "已暂停", CANCELLING: "取消中", CANCELLED: "已取消", SUCCEEDED: "已完成", PENDING: "等待中", RETRYING: "重试中" };
function unitStatus(status) { return statusText[status] || status; }

function formatClock(value) {
  const date = new Date(Number(value) * 1000);
  return Number.isNaN(date.getTime()) ? "" : date.toLocaleTimeString("zh-CN", { hour12: false });
}

function describeStep(record) {
  const payload = record.payload && typeof record.payload === "object" ? record.payload : {};
  const type = record.type;
  if (type === "order.created") return { text: payload.message || "订单已创建", tone: "" };
  if (type === "item.started") return { text: `开始抓取第 ${payload.sequence} 件：${payload.name}`, tone: "" };
  if (type === "item.retrying") return payload.name ? { text: `正在重试第 ${payload.sequence} 件：${payload.name}`, tone: "warn" } : { text: `第 ${payload.sequence} 件执行失败，正在自动重试`, tone: "warn" };
  if (type === "item.succeeded") return { text: `第 ${payload.sequence} 件已放入篮筐`, tone: "ok" };
  if (type === "agent.progress") {
    const info = payload.info && typeof payload.info === "object" ? payload.info : {};
    const progress = info.progress || info.message;
    if (!progress) return null;
    const error = info.error && !String(progress).includes(String(info.error)) ? `：${info.error}` : "";
    return { text: `${progress}${error}`, tone: info.error ? "warn" : "" };
  }
  if (type === "order.succeeded") return { text: payload.message || "订单已完成", tone: "ok" };
  if (type === "order.cancelling") return { text: payload.message || "正在取消订单", tone: "warn" };
  if (type === "order.cancelled") return { text: payload.message || "订单已取消", tone: "" };
  if (type === "order.paused") return { text: payload.message || "订单已暂停", tone: "warn" };
  if (type === "order.retry_requested") return { text: payload.message || "已请求重试", tone: "" };
  return payload.message ? { text: String(payload.message), tone: "" } : null;
}

function renderSteps() {
  const list = $("#step-log");
  if (!list) return;
  const follow = list.scrollHeight - list.scrollTop - list.clientHeight < 80;
  if (!state.steps.length) { list.innerHTML = `<li class="step-empty">等待机器人开始</li>`; return; }
  list.innerHTML = state.steps.map((step, index) => `<li class="step-row ${step.tone}${index === state.steps.length - 1 ? " latest" : ""}"><time>${escapeHtml(formatClock(step.time))}</time><span>${escapeHtml(step.text)}</span></li>`).join("");
  if (follow) list.scrollTop = list.scrollHeight;
}

function appendStep(record) {
  if (state.seenEventIds.has(record.event_id)) return;
  state.seenEventIds.add(record.event_id);
  const described = describeStep(record);
  if (!described) return;
  const last = state.steps[state.steps.length - 1];
  if (last && last.text === described.text) return;
  state.steps.push({ id: record.event_id, text: described.text, tone: described.tone, time: record.created_at });
  renderSteps();
}

function appendNote(text) {
  const last = state.steps[state.steps.length - 1];
  if (last && last.text === text) return;
  state.steps.push({ id: `local-${Date.now()}`, text, tone: "", time: Date.now() / 1000 });
  renderSteps();
}

function showOrder() {
  const order = state.order;
  if (!order || state.builder) { $("#shop-view").classList.remove("hidden"); $("#progress-view").classList.add("hidden"); return; }
  $("#shop-view").classList.add("hidden"); $("#progress-view").classList.remove("hidden");
  const done = terminal.has(order.status);
  const paused = order.status === "PAUSED";
  $("#progress-title").textContent = order.status === "SUCCEEDED" ? "订单已完成" : order.status === "CANCELLED" ? "订单已取消" : paused ? "订单已暂停，等待处理" : "机器人正在处理您的订单";
  $("#order-number").textContent = `订单号 ${order.order_id}`;
  $("#order-mode").textContent = order.mock ? "执行模式 Mock" : "执行模式 真机";
  $("#percent").textContent = order.progress_percent;
  $("#status-orb").style.setProperty("--progress", `${order.progress_percent}%`);
  const status = $("#order-status"); status.textContent = statusText[order.status] || order.status; status.className = `status-pill ${order.status.toLowerCase()}`;
  $("#basket-label").textContent = `篮筐 ${order.basket_row} · ${order.basket_column}列`;
  $("#progress-copy").textContent = `${order.completed_items} / ${order.total_items} 件`;
  $("#progress-bar").style.width = `${order.progress_percent}%`;
  $("#back-to-shop").classList.toggle("hidden", !done);
  $("#cancel-order").classList.toggle("hidden", done || order.status === "CANCELLING");
  $("#retry-order").classList.toggle("hidden", !paused);
  $("#order-actions").classList.toggle("hidden", done);
  $("#live-dot").style.background = paused ? "#df7045" : done ? "#63a174" : "#43a474";
  $("#robot-state-text").textContent = done ? "系统准备就绪" : paused ? "订单等待处理" : "机器人正在执行订单";
  const warning = $("#risk-warning"); warning.classList.toggle("hidden", !paused); warning.querySelector("p").textContent = order.error || "请确认机器人和商品状态后再选择重试或取消。";
  $("#progress-items").innerHTML = order.items.map(item => `<div class="progress-item"><img src="${escapeHtml(item.image_url)}" alt=""><div><strong>${escapeHtml(item.name)} × ${item.quantity}</strong><p>AGV ${escapeHtml(item.agv_row)} · ${escapeHtml(item.agv_column)}列　已完成 ${item.completed_quantity} / ${item.quantity}</p></div><span class="unit-status ${item.status.toLowerCase()}">${unitStatus(item.status)}</span></div>`).join("");
}

function connectEvents(orderId) {
  state.stream?.close();
  clearTimeout(connectEvents.timer);
  if (state.stepsOrderId !== orderId) { state.steps = []; state.seenEventIds = new Set(); state.stepsOrderId = orderId; renderSteps(); }
  if (terminal.has(state.order?.status)) return;
  const stream = new EventSource(`/orders/api/orders/${encodeURIComponent(orderId)}/events`); state.stream = stream;
  stream.addEventListener("order", message => {
    try { appendStep(JSON.parse(message.data)); } catch (_) {}
    clearTimeout(connectEvents.timer);
    connectEvents.timer = setTimeout(async () => {
      try { state.order = await api(`/orders/api/orders/${encodeURIComponent(orderId)}`); showOrder(); if (terminal.has(state.order.status)) stream.close(); } catch (_) {}
    }, 150);
  });
  stream.onerror = () => { if (!terminal.has(state.order?.status)) $("#robot-state-text").textContent = "正在恢复实时连接"; };
}

async function act(path, busyText) {
  const id = state.order.order_id;
  appendNote(busyText);
  try { state.order = await api(`/orders/api/orders/${encodeURIComponent(id)}/${path}`, { method: "POST", body: "{}" }); showOrder(); connectEvents(id); }
  catch (error) { toast(error.message, true); }
}

async function init() {
  try {
    const [catalog, current] = await Promise.all([api("/orders/api/products"), api("/orders/api/current")]);
    state.products = catalog.products; state.order = current; state.builder = !current || terminal.has(current.status);
    renderProducts(); renderCart();
    if (current && !terminal.has(current.status)) { state.builder = false; showOrder(); connectEvents(current.order_id); }
  } catch (error) { toast(error.message, true); }
}

function setMode(mock) {
  state.mock = mock;
  document.querySelectorAll(".mode-option").forEach(button => button.classList.toggle("active", (button.dataset.mock === "true") === mock));
}

$("#submit-order").onclick = submitOrder;
document.querySelectorAll(".mode-option").forEach(button => button.onclick = () => setMode(button.dataset.mock === "true"));
$("#retry-order").onclick = () => act("retry", "正在重新启动当前步骤…");
$("#cancel-order").onclick = () => { if (window.confirm("确认取消当前订单？机器人会在当前能力调用返回后停止。")) act("cancel", "正在停止机器人…"); };
$("#back-to-shop").onclick = () => { state.builder = true; state.order = null; state.cart.clear(); state.locations.clear(); renderProducts(); renderCart(); showOrder(); window.scrollTo({ top: 0, behavior: "smooth" }); };
init();
