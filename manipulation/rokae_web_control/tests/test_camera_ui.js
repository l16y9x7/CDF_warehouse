"use strict";
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
const path = require("node:path");
const root = path.resolve(__dirname, "..");
const html = fs.readFileSync(path.join(root, "static/index.html"), "utf8");
assert(!html.includes("data-camera-start"));
assert(!html.includes("data-camera-stop"));
assert(html.includes("data-camera-restart=\"all\""));
assert(html.includes("完整记录需要同步 RGB-D"));
const poseSection = html.match(/<article class="panel camera-card pose-estimation-card">([\s\S]*?)<\/article>/)?.[1];
assert(poseSection);
for (const id of ['poseReferenceCamera', 'poseSavedAt', 'poseWorldGrasp',
  'poseCaptureTrunk', 'poseShoulderGrasp', 'poseShoulderPregrasp', 'poseShoulderTrunk']) {
  assert(poseSection.includes(`id="${id}"`));
}
const elements = new Map();
const element = id => {
  if (!elements.has(id)) elements.set(id, {
    disabled: false, hidden: true, textContent: "", className: "", src: "",
    removeAttribute(name) { delete this[name]; },
  });
  return elements.get(id);
};
const requests = [];
const context = vm.createContext({
  document: { getElementById: element },
  setTimeout: () => 1, clearTimeout: () => {}, Date, Number, Boolean, Object, Array, JSON,
});
const source = fs.readFileSync(path.join(root, "static/app.js"), "utf8");
vm.runInContext(source.slice(0, source.lastIndexOf("\nrenderCards();")), context);
context.poseResult = {
  usable: true, message: "定位成功", relative_directory: "data/test",
  response: { reference_point_camera_mm: [1, 2, 3] },
  world_grasp: {
    grasp_pose_world_mm_deg: [600, -100, 800, 180, -90, 0],
    capture_upper_body_joints_deg: [-54, 23, -59, 0],
  },
};
vm.runInContext("renderPoseResult(poseResult)", context);
assert.equal(element("poseReferenceCamera").textContent, "[1.000, 2.000, 3.000]");
assert.equal(element("poseWorldGrasp").textContent, "[600.000, -100.000, 800.000]");
assert.equal(element("poseSavedAt").textContent, "data/test");
assert(source.includes('pregrasp_pose_${arm}_shoulder_mm_deg'));
context.testStatus = {
  enabled: true, available: true, source: "http8085", externally_managed: true,
  frame_available: true,
  recovery_available: true, recovery_scope: "all", ros_domain_id: 51,
  rgb_profile: { width: 1280, height: 720, fps: 15 }, actual_capture_fps: 15,
};
vm.runInContext("updateCameraStatus('head', testStatus)", context);
assert.equal(element("cameraBadge-head").textContent, "头部 已就绪");
assert.equal(element("cameraFrameButton-head").disabled, false);
assert.equal(element("cameraRecordButton-head").disabled, false);
assert.equal(element("cameraRestartButton-head").disabled, false);
assert(element("cameraInfo-head").textContent.includes("相机 8085"));
vm.runInContext("getCameraFrame('head')", context);
assert(element("rgbCameraFrame-head").src.startsWith("/api/cameras/head/frame/rgb?"));
element("rgbCameraFrame-head").onload();
assert.equal(element("rgbCameraFrame-head").hidden, false);
context.testStatus.enabled = false;
vm.runInContext("updateCameraStatus('head', testStatus)", context);
assert.equal(element("cameraFrameButton-head").disabled, false);
assert.equal(element("cameraRecordButton-head").disabled, true);
assert.equal(element("rgbCameraFrame-head").hidden, false);
context.testStatus.frame_available = false;
vm.runInContext("updateCameraStatus('head', testStatus)", context);
assert.equal(element("rgbCameraFrame-head").hidden, true);
assert.equal(element("rgbCameraFrame-head").src, undefined);
assert.equal(element("cameraFrameButton-head").disabled, true);
assert.equal(element("cameraRecordButton-head").disabled, true);
assert.equal(element("cameraRestartButton-head").disabled, false);
context.testStatus.recovery_running = true;
vm.runInContext("updateCameraStatus('head', testStatus)", context);
assert.equal(element("cameraRestartButton-head").disabled, true);
assert.equal(element("cameraRestartButton-head").textContent, "正在重启三路摄像头…");
context.testStatus.recovery_running = false;
context.fetch = async (url, options) => {
  requests.push({ url, body: JSON.parse(options.body) });
  return { ok: true, json: async () => ({ ok: true, data: { accepted: true, message: "check" } }) };
};
vm.runInContext("refreshStatus = async () => {}", context);
(async () => {
  let release;
  context.fetch = async (url, options) => {
    requests.push({url, body: JSON.parse(options.body)});
    await new Promise(resolve => { release = resolve; });
    return {ok:true, json:async()=>({ok:true,data:{accepted:true,message:"三路摄像头重启中"}})};
  };
  const first=vm.runInContext("restartCamera()", context);
  assert.equal(element("cameraRestartButton-head").disabled,true);
  assert.equal(vm.runInContext("Object.values(cameraBusy).every(item=>item.recovery)",context),true);
  await vm.runInContext("restartCamera()",context);
  assert.equal(requests.length,1);
  release();
  await first;
  assert.equal(requests.length, 1);
  assert.equal(requests[0].url, "/api/cameras/restart");
  assert.deepEqual(requests[0].body, { camera: "all" });
  assert(!source.includes("/api/cameras/enable"));
  context.testStatus.recovery_result={command_ok:true};
  vm.runInContext("updateCameraStatus('head',testStatus)",context);
  assert(element("cameraInfo-head").textContent.includes("三路重启命令已完成"));
  context.testStatus.recovery_result={command_ok:false,error:"右腕重启失败"};
  vm.runInContext("updateCameraStatus('head',testStatus)",context);
  assert(element("cameraInfo-head").textContent.includes("右腕重启失败"));
  delete context.testStatus.recovery_scope;
  vm.runInContext("updateCameraStatus('head',testStatus)",context);
  assert.equal(element("cameraRestartButton-head").disabled,true); // older head-only backend
  assert.equal(vm.runInContext("Object.values(cameraBusy).some(item=>item.recovery)",context),false);
  console.log("Camera UI: ROS readiness, black idle, on-demand frame, all-camera restart request, duplicate prevention, scope gate and results passed");
})().catch(error => { console.error(error); process.exitCode = 1; });
