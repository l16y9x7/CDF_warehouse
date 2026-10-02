'use strict';
const $ = (id) => document.getElementById(id);
const palette = ['#8b75ef', '#35bb97', '#eeaf46', '#4c9ee8', '#e77eaa', '#70ba58', '#e98764', '#58bbce', '#ba82db', '#b4b743'];
const state = {config: null, image: null, bitmap: null, result: null, masks: [], history: [], view: 'overlay', zoom: 1, focused: null, busy: false, imageVersion: 0, historyId: null};
const canvas = $('preview');
const ctx = canvas.getContext('2d');
const escapeId = () => `${Date.now()}-${Math.random().toString(16).slice(2, 8)}`;

function notice(message, kind = '') { $('notice').textContent = message; $('notice').className = `notice ${kind}`; }
function setBusy(value) {
  state.busy = value;
  $('input-fields').disabled = value;
  $('busy-cover').hidden = !value;
  $('run-button').disabled = value || !state.image;
  document.querySelectorAll('.history-card').forEach((b) => { b.disabled = value; });
}
async function api(url, options) {
  const response = await fetch(url, options);
  let data;
  try { data = await response.json(); } catch { throw new Error(`本地服务返回无效内容（HTTP ${response.status}）。`); }
  if (!response.ok || data.ok === false) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}
function imageElement(src) {
  return new Promise((resolve, reject) => {
    const image = new Image();
    image.onload = () => resolve(image);
    image.onerror = () => reject(new Error('图片加载失败。'));
    image.src = src;
  });
}
function fileDataUrl(file) {
  return new Promise((resolve, reject) => {
    const reader = new FileReader(); reader.onload = () => resolve(reader.result);
    reader.onerror = () => reject(new Error('无法读取图片文件。')); reader.readAsDataURL(file);
  });
}
function currentParameters() {
  return {prompt: $('prompt').value.trim(), threshold: Number($('threshold-number').value), mask_threshold: Number($('mask-threshold-number').value)};
}
function parametersChanged() {
  $('prompt-count').textContent = `${$('prompt').value.length} / 2000`;
  const p = currentParameters(), r = state.result;
  $('stale-badge').hidden = !r || (p.prompt === r.prompt && p.threshold === r.threshold && p.mask_threshold === r.mask_threshold);
}
function fillParameters(prompt, threshold = 0.5, maskThreshold = 0.5) {
  $('prompt').value = prompt;
  $('threshold').value = threshold;
  $('threshold-number').value = Number(threshold).toFixed(2);
  $('mask-threshold').value = maskThreshold;
  $('mask-threshold-number').value = Number(maskThreshold).toFixed(2);
  parametersChanged();
}
function applyProfile() {
  const profile = state.config.profiles.find((p) => p.id === $('profile-select').value);
  if (!profile) return;
  fillParameters(profile.sam3_prompt, profile.target_threshold ?? 0.5);
  notice(`已填入 ${profile.id} 的当前配置。运行时以输入框中的参数为准。`);
}
async function loadConfig(initial = false) {
  const config = await api('/api/config'); state.config = config;
  $('endpoint').textContent = new URL(config.upstream).host;
  $('endpoint').title = config.upstream;
  $('library-revision').textContent = `SKU LIBRARY · ${config.revision}`;
  const previousProfile = $('profile-select').value;
  $('profile-select').replaceChildren(new Option('手动输入提示词', ''));
  config.profiles.forEach((p) => $('profile-select').add(new Option(`${p.id} · ${p.name}`, p.id)));
  $('profile-select').value = previousProfile;
  const previousSample = $('sample-select').value;
  $('sample-select').replaceChildren(new Option('选择日志图片…', ''));
  config.samples.forEach((s) => $('sample-select').add(new Option(`${s.name}${s.sku_id ? ' · ' + s.sku_id : ''}`, s.key)));
  $('sample-select').value = previousSample;
  notice('日志和 SKU 配置已就绪。');
  if (initial && config.samples.length) {
    const desired = new URLSearchParams(location.search).get('sample');
    $('sample-select').value = config.samples.some((s) => s.key === desired) ? desired : config.samples[0].key;
    await loadSample();
  }
}
function clearResult() {
  state.result = null; state.masks = []; state.focused = null; state.historyId = null;
  $('result-meta').hidden = true; $('stale-badge').hidden = true;
  $('count-badge').textContent = '—'; $('result-caption').textContent = '运行后可查看各实例的置信度、面积和位置。';
  $('instance-list').replaceChildren();
  const empty = document.createElement('div'); empty.className = 'list-placeholder';
  empty.textContent = '图片已就绪，运行分割查看结果。'; $('instance-list').append(empty);
  $('export-json').disabled = true; $('export-image').disabled = !state.image;
  renderHistory();
}
async function setImage(image) {
  const version = ++state.imageVersion;
  const bitmap = await imageElement(image.dataUrl);
  if (version !== state.imageVersion) return false;
  if (bitmap.naturalWidth * bitmap.naturalHeight > 24_000_000) throw new Error('图片超过 2400 万像素，请缩小后上传。');
  state.image = image; state.bitmap = bitmap; state.zoom = 1;
  canvas.width = bitmap.naturalWidth; canvas.height = bitmap.naturalHeight;
  $('empty-view').hidden = true; $('canvas-wrap').hidden = false;
  $('image-ready').textContent = '已载入'; $('image-ready').classList.add('ready');
  $('source-label').textContent = image.name;
  $('image-details').textContent = `${canvas.width} × ${canvas.height} px  ·  ${(image.bytes / 1024).toFixed(0)} KB  ·  ${image.name}`;
  $('image-details').title = image.name;
  $('run-button').disabled = state.busy;
  clearResult(); fitCanvas(); draw(); return true;
}
async function loadSample() {
  const key = $('sample-select').value;
  if (!key || state.busy) return;
  setBusy(true); $('busy-text').textContent = '正在载入日志图片…';
  try {
    const data = await api(`/api/sample?key=${encodeURIComponent(key)}`);
    await setImage({name: data.name, bytes: data.bytes, dataUrl: `data:${data.mime};base64,${data.image_base64}`});
    if (data.sku_id && state.config.profiles.some((p) => p.id === data.sku_id)) {
      $('profile-select').value = data.sku_id; applyProfile();
      notice(`日志图片已载入，参数来自 SKU ${data.sku_id} 的当前库配置。`, 'success');
    } else {
      $('profile-select').value = '';
      const defaults = {box: 'the top surfaces of the small boxes', tube: 'the sealed ends of tubes', bottle: 'the main cylindrical body of each individual cosmetic bottle'};
      fillParameters(data.prompt || defaults[data.sku_typ] || 'each individual open cardboard box', data.threshold ?? (data.sku_typ === 'tube' ? 0.2 : 0.5));
      notice('日志图片已载入，可以开始分割。', 'success');
    }
  } catch (error) { notice(error.message, 'error'); }
  finally { setBusy(false); }
}
async function upload(file) {
  if (!file || state.busy) return;
  if (!/^image\/(jpeg|png|webp)$/.test(file.type)) return notice('请选择 JPG、PNG 或 WebP 图片。', 'error');
  if (file.size > 20 * 1024 * 1024) return notice('图片文件不能超过 20 MiB。', 'error');
  setBusy(true); $('busy-text').textContent = '正在读取图片…';
  try {
    await setImage({name: file.name, bytes: file.size, dataUrl: await fileDataUrl(file)});
    $('sample-select').value = ''; notice('图片已载入。确认 prompt 和阈值后运行分割。', 'success');
  } catch (error) { notice(error.message, 'error'); }
  finally { setBusy(false); $('file-input').value = ''; }
}
function fitCanvas() {
  if (!state.bitmap) return;
  const viewport = $('canvas-viewport');
  const padding = innerWidth <= 1000 ? 24 : 40;
  const scale = Math.min((viewport.clientWidth - padding) / canvas.width, (viewport.clientHeight - padding) / canvas.height);
  $('canvas-wrap').style.width = `${Math.max(1, canvas.width * scale * state.zoom)}px`;
  $('canvas-wrap').style.height = `${Math.max(1, canvas.height * scale * state.zoom)}px`;
  $('zoom-reset').textContent = state.zoom === 1 ? '适应' : `${Math.round(scale * state.zoom * 100)}%`;
}
function draw() {
  if (!state.bitmap) return;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (state.view === 'mask') { ctx.fillStyle = '#161925'; ctx.fillRect(0, 0, canvas.width, canvas.height); }
  else ctx.drawImage(state.bitmap, 0, 0);
  if (state.view === 'original' || !state.result) return;
  const opacity = Number($('opacity').value);
  for (const item of state.masks) {
    if (!item.visible) continue;
    const muted = state.focused !== null && state.focused !== item.id;
    ctx.globalAlpha = (state.view === 'mask' ? 1 : opacity) * (muted ? 0.2 : 1);
    ctx.drawImage(item.canvas, 0, 0); ctx.globalAlpha = 1;
    if (!$('show-boxes').checked) continue;
    const [x1, y1, x2, y2] = item.bbox_xyxy;
    ctx.globalAlpha = muted ? 0.3 : 1; ctx.strokeStyle = item.color; ctx.lineWidth = Math.max(2, canvas.width / 550);
    ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
    const fontSize = Math.max(13, Math.round(canvas.width / 80)); ctx.font = `600 ${fontSize}px "Segoe UI", sans-serif`;
    const label = `#${item.id}  ${item.score.toFixed(3)}`, labelWidth = ctx.measureText(label).width + 12;
    const lx = Math.max(0, Math.min(x1, canvas.width - labelWidth)), ly = Math.max(fontSize + 9, y1);
    ctx.fillStyle = item.color; ctx.fillRect(lx, ly - fontSize - 9, labelWidth, fontSize + 7);
    ctx.fillStyle = '#fff'; ctx.fillText(label, lx + 6, ly - 5); ctx.globalAlpha = 1;
  }
}
async function prepareMasks(instances) {
  const masks = [];
  for (let i = 0; i < instances.length; i++) {
    const instance = instances[i], image = await imageElement(`data:image/png;base64,${instance.mask_png_base64}`);
    if (image.width !== canvas.width || image.height !== canvas.height) throw new Error('SAM3 返回的 mask 尺寸与原图不一致。');
    const layer = document.createElement('canvas'); layer.width = image.width; layer.height = image.height;
    const context = layer.getContext('2d'); context.drawImage(image, 0, 0);
    const pixels = context.getImageData(0, 0, layer.width, layer.height), color = palette[i % palette.length];
    const rgb = [1, 3, 5].map((offset) => parseInt(color.slice(offset, offset + 2), 16));
    for (let p = 0; p < pixels.data.length; p += 4) {
      const on = pixels.data[p] > 0 && pixels.data[p + 3] > 0;
      pixels.data[p] = rgb[0]; pixels.data[p + 1] = rgb[1]; pixels.data[p + 2] = rgb[2]; pixels.data[p + 3] = on ? 255 : 0;
    }
    context.putImageData(pixels, 0, 0);
    masks.push({...instance, canvas: layer, color, visible: true});
  }
  return masks;
}
function renderInstances() {
  $('instance-list').replaceChildren();
  if (!state.masks.length) {
    const empty = document.createElement('div'); empty.className = 'list-placeholder';
    empty.textContent = '没有返回实例。可尝试更具体的 prompt，或降低检测阈值。'; $('instance-list').append(empty); return;
  }
  for (const item of state.masks) {
    const card = document.createElement('div'); card.className = `instance-card${item.visible ? '' : ' off'}${state.focused === item.id ? ' focused' : ''}`;
    card.tabIndex = 0; card.setAttribute('role', 'button'); card.setAttribute('aria-label', `高亮实例 ${item.id}，分数 ${item.score.toFixed(3)}`);
    const top = document.createElement('div'); top.className = 'instance-top';
    const checkbox = document.createElement('input'); checkbox.type = 'checkbox'; checkbox.checked = item.visible; checkbox.setAttribute('aria-label', `显示实例 ${item.id}`);
    checkbox.addEventListener('click', (event) => event.stopPropagation());
    checkbox.addEventListener('change', () => { item.visible = checkbox.checked; renderInstances(); draw(); });
    const dot = document.createElement('span'); dot.className = 'instance-dot'; dot.style.background = item.color;
    const name = document.createElement('span'); name.className = 'instance-name'; name.textContent = `实例 ${String(item.id).padStart(2, '0')}`;
    const score = document.createElement('span'); score.className = 'instance-score'; score.textContent = item.score.toFixed(3);
    top.append(checkbox, dot, name, score);
    const detail = document.createElement('div'); detail.className = 'instance-detail';
    detail.textContent = `${item.area_pixels.toLocaleString()} px · ${(item.area_ratio * 100).toFixed(2)}% 全图`;
    const coords = document.createElement('div'); coords.textContent = `[${item.bbox_xyxy.map(Math.round).join(', ')}]`; detail.append(coords);
    const download = document.createElement('button'); download.className = 'mask-download'; download.type = 'button'; download.textContent = '↓ mask'; download.setAttribute('aria-label', `下载实例 ${item.id} mask`);
    download.addEventListener('click', (event) => { event.stopPropagation(); downloadUrl(`data:image/png;base64,${item.mask_png_base64}`, `sam3-mask-${item.id}.png`); });
    const focus = () => { state.focused = state.focused === item.id ? null : item.id; renderInstances(); draw(); };
    card.addEventListener('click', focus); card.addEventListener('keydown', (event) => { if (event.target === card && ['Enter', ' '].includes(event.key)) { event.preventDefault(); focus(); } });
    card.append(top, detail, download); $('instance-list').append(card);
  }
}
async function showResult(result) {
  const masks = await prepareMasks(result.instances);
  state.result = result; state.masks = masks; state.focused = null;
  $('count-badge').textContent = `${result.instance_count} 个实例`;
  $('result-caption').textContent = `本次 prompt：${result.prompt}`;
  $('result-meta').hidden = false;
  $('timing-summary').textContent = `SAM3 往返 ${(result.timing_ms.upstream / 1000).toFixed(2)} s  ·  本地处理合计 ${(result.timing_ms.total / 1000).toFixed(2)} s  ·  阈值 ${result.threshold.toFixed(2)}`;
  $('timing-summary').title = 'SAM3 往返包含图片传输、服务排队和推理；合计还包括本地图片转换和响应解析。';
  $('export-json').disabled = false; $('export-image').disabled = false;
  renderInstances(); parametersChanged(); draw();
}
async function run(event) {
  event?.preventDefault();
  if (state.busy || !state.image) return;
  const params = currentParameters();
  if (!params.prompt) return notice('请先输入 prompt。', 'error');
  if (!['threshold', 'mask_threshold'].every((k) => Number.isFinite(params[k]) && params[k] >= 0 && params[k] <= 1)) return notice('两个阈值都必须在 0～1 之间。', 'error');
  setBusy(true); const started = performance.now();
  const update = () => { $('busy-text').textContent = `SAM3 正在分割… ${((performance.now() - started) / 1000).toFixed(1)} s`; };
  update(); const timer = setInterval(update, 100);
  notice('正在向 SAM3 发送图片，请稍候。');
  try {
    const result = await api('/api/segment', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({...params, image_name: state.image.name, image_base64: state.image.dataUrl.split(',')[1]})});
    await showResult(result);
    state.historyId = escapeId();
    state.history.unshift({id: state.historyId, result, image: state.image, time: new Date().toLocaleTimeString('zh-CN', {hour12: false}), profile: $('profile-select').value});
    state.history = state.history.slice(0, 6); renderHistory();
    notice(result.instance_count ? `分割完成，返回 ${result.instance_count} 个原始实例。点击右侧卡片检查 mask。` : '请求成功，但没有返回实例。可调整 prompt 或阈值后再试。', result.instance_count ? 'success' : '');
  } catch (error) { notice(error.message, 'error'); }
  finally { clearInterval(timer); setBusy(false); }
}
function renderHistory() {
  $('history-list').replaceChildren();
  if (!state.history.length) {
    const empty = document.createElement('div'); empty.className = 'history-empty'; empty.textContent = '换个提示词再试一次，结果会保留在这里。'; $('history-list').append(empty); return;
  }
  for (const entry of state.history) {
    const card = document.createElement('button'); card.type = 'button'; card.className = `history-card${state.historyId === entry.id ? ' active' : ''}`; card.disabled = state.busy;
    const thumb = document.createElement('img'); thumb.src = entry.image.dataUrl; thumb.alt = '';
    const copy = document.createElement('div'); copy.className = 'history-copy';
    const title = document.createElement('strong'); title.textContent = entry.result.prompt; title.title = entry.result.prompt;
    const info = document.createElement('span'); info.textContent = `${entry.time} · ${entry.result.instance_count} 个 · 阈值 ${entry.result.threshold.toFixed(2)}`;
    copy.append(title, info); card.append(thumb, copy);
    card.addEventListener('click', async () => {
      if (state.busy) return;
      setBusy(true); $('busy-text').textContent = '正在恢复调试记录…';
      try {
        await setImage(entry.image); fillParameters(entry.result.prompt, entry.result.threshold, entry.result.mask_threshold);
        $('profile-select').value = entry.profile; $('sample-select').value = '';
        await showResult(entry.result); state.historyId = entry.id; renderHistory(); notice('已恢复历史图片、参数和结果，未重新调用 SAM3。');
      } catch (error) { notice(error.message, 'error'); } finally { setBusy(false); }
    });
    $('history-list').append(card);
  }
}
function downloadUrl(url, name) { const a = document.createElement('a'); a.href = url; a.download = name; a.click(); }
function downloadBlob(blob, name) { const url = URL.createObjectURL(blob); downloadUrl(url, name); setTimeout(() => URL.revokeObjectURL(url), 3000); }
$('run-form').addEventListener('submit', run);
$('prompt').addEventListener('input', parametersChanged);
$('profile-select').addEventListener('change', applyProfile);
$('sample-select').addEventListener('change', loadSample);
$('reload-config').addEventListener('click', async () => { try { await loadConfig(); } catch (error) { notice(error.message, 'error'); } });
$('upload-zone').addEventListener('click', () => $('file-input').click());
$('file-input').addEventListener('change', () => upload($('file-input').files[0]));
for (const event of ['dragenter', 'dragover']) $('upload-zone').addEventListener(event, (e) => { e.preventDefault(); $('upload-zone').classList.add('dragover'); });
for (const event of ['dragleave', 'drop']) $('upload-zone').addEventListener(event, (e) => { e.preventDefault(); $('upload-zone').classList.remove('dragover'); });
$('upload-zone').addEventListener('drop', (e) => upload(e.dataTransfer.files[0]));
for (const key of ['threshold', 'mask-threshold']) {
  $(key).addEventListener('input', () => { $(`${key}-number`).value = Number($(key).value).toFixed(2); parametersChanged(); });
  $(`${key}-number`).addEventListener('input', () => { $(key).value = $(`${key}-number`).value; parametersChanged(); });
}
document.querySelectorAll('[data-prompt]').forEach((b) => b.addEventListener('click', () => { $('prompt').value = b.dataset.prompt; $('profile-select').value = ''; parametersChanged(); }));
document.querySelectorAll('[data-view]').forEach((b) => b.addEventListener('click', () => { state.view = b.dataset.view; document.querySelectorAll('[data-view]').forEach((x) => x.classList.toggle('active', x === b)); draw(); }));
$('show-boxes').addEventListener('change', draw);
$('opacity').addEventListener('input', () => { $('opacity-value').textContent = `${Math.round(Number($('opacity').value) * 100)}%`; draw(); });
$('zoom-in').addEventListener('click', () => { state.zoom = Math.min(4, state.zoom + 0.5); fitCanvas(); });
$('zoom-out').addEventListener('click', () => { state.zoom = Math.max(1, state.zoom - 0.5); fitCanvas(); });
$('zoom-reset').addEventListener('click', () => { state.zoom = 1; fitCanvas(); });
$('select-all').addEventListener('click', () => { state.masks.forEach((m) => { m.visible = true; }); renderInstances(); draw(); });
$('clear-focus').addEventListener('click', () => { state.focused = null; renderInstances(); draw(); });
$('export-image').addEventListener('click', () => canvas.toBlob((blob) => { if (blob) downloadBlob(blob, `sam3-${state.view}-${Date.now()}.png`); }));
$('export-json').addEventListener('click', () => { if (state.result) downloadBlob(new Blob([JSON.stringify(state.result, null, 2)], {type: 'application/json'}), `sam3-result-${Date.now()}.json`); });
document.addEventListener('keydown', (event) => { if ((event.ctrlKey || event.metaKey) && event.key === 'Enter') { event.preventDefault(); run(); } });
new ResizeObserver(fitCanvas).observe($('canvas-viewport'));
parametersChanged(); loadConfig(true).catch((error) => notice(error.message, 'error'));
