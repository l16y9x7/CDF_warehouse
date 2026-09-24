"use strict";
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const root = path.resolve(__dirname, '..');
const html = fs.readFileSync(path.join(root, 'static/index.html'), 'utf8');
assert(html.includes('<h3>摄像头采集</h3>'));
for (const name of ['获取头部当前帧', '获取左手当前帧', '记录头部当前数据', '采集左手 RGB 画面']) assert(html.includes(name));
const ids = new Set([...html.matchAll(/id="([^"]+)"/g)].map(m => m[1]));
const elements = new Map();
const element = id => {
  if (!ids.has(id)) return null;
  if (!elements.has(id)) elements.set(id, { hidden: true, disabled: false, textContent: '',
    removeAttribute(name) { delete this[name]; } });
  return elements.get(id);
};
const requests = [];
const context = vm.createContext({ document: {getElementById: element},
  setTimeout: () => 1, clearTimeout: () => {}, Date, Number, Boolean, Object, Array, JSON });
const source = fs.readFileSync(path.join(root, 'static/app.js'), 'utf8');
vm.runInContext(source.slice(0,source.lastIndexOf('\nrenderCards();')),context);
context.status = {enabled:true,available:true,frame_available:true,ros_domain_id:105,
  rgb_profile:{width:1280,height:720,fps:15}, actual_capture_fps:15};
vm.runInContext("updateCameraStatus('head',status); updateCameraStatus('left_wrist',status)",context);
assert(element('cameraInfo-left_wrist').textContent.includes('仅 RGB，不启用深度'));
assert(!element('cameraFrameButton-left_wrist').disabled);
vm.runInContext("getCameraFrame('head'); getCameraFrame('left_wrist')",context);
element('rgbCameraFrame-left_wrist').onload();
element('rgbCameraFrame-head').onload();
assert(element('rgbCameraFrame-head').hidden);
assert(!element('rgbCameraFrame-left_wrist').hidden);
assert(element('cameraPreviewCaption').textContent.includes('左手'));
vm.runInContext("getCameraFrame('head')",context);
element('rgbCameraFrame-head').onload();
assert(!element('rgbCameraFrame-head').hidden);
assert(element('rgbCameraFrame-left_wrist').hidden);
context.fetch = async (url, options) => {
  requests.push(url);
  return {ok:true,json:async()=>({ok:true,data:{relative_path:'2026-09-19/123456789.jpg',relative_directory:'original-head'}})};
};
(async () => {
  await vm.runInContext("recordCameraRgb('left_wrist')",context);
  await vm.runInContext("recordCameraData('head')",context);
  assert.deepEqual(requests,['/api/cameras/left_wrist/record-rgb','/api/cameras/head/record']);
  assert(!element('cameraRecordButton-left_wrist').disabled);
  console.log('PASS: two preview buttons, latest-selection race, RGB-only save and unchanged head route');
})().catch(error=>{ console.error(error); process.exitCode=1; });
