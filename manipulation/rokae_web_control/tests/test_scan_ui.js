"use strict";
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const root = path.resolve(__dirname, '..');
const html = fs.readFileSync(path.join(root, 'static/index.html'), 'utf8');
assert(html.indexOf('id="scanSequenceButton"') > html.indexOf('id="graspTestButton"'));
const elements = new Map();
const element = id => {
  if (!elements.has(id)) elements.set(id, {disabled: false, textContent: '', value: '', dataset: {}});
  return elements.get(id);
};
const manual = {disabled:false};
const context = vm.createContext({document:{getElementById:element,
  querySelectorAll: selector => selector.includes('data-joint-move') ? [manual] : []},
  setTimeout:()=>1, clearTimeout:()=>{}, Date, Number, Boolean, Object, Array, JSON});
const source = fs.readFileSync(path.join(root, 'static/app.js'), 'utf8');
vm.runInContext(source.slice(0, source.lastIndexOf('\nrenderCards();')), context);
element('poseTargetSelect').value='bottle';
vm.runInContext('serverStatus = {armed:true, scan_sequence:{version:"scan-sequence-v5"}}; updateScanControls()',context);
assert.equal(element('scanSequenceButton').disabled, false);
vm.runInContext('scanExecution = {active:true,phase:"capture",message:"拍照中",completed_points:2,directory:"/rgb/day/scan_time"}; updateMemoryControls()',context);
assert.equal(element('scanSequenceButton').disabled,true);
assert.equal(element('scanSequenceStop').disabled,false);
assert.equal(manual.disabled,true);
assert(element('scanSequenceStatus').textContent.includes('2/5'));
assert(element('scanSequenceStatus').textContent.includes('/rgb/day/scan_time'));
vm.runInContext('scanExecution.active=false; scanExecution.stop_unconfirmed=true; updateScanControls()',context);
assert.equal(element('scanSequenceButton').disabled,true);
assert.equal(element('scanSequenceStop').disabled,false);
const requests=[];
context.fetch=async (url,options)=>{
  requests.push({url,body:JSON.parse(options.body)});
  return {ok:true,json:async()=>({ok:true,data:{active:true,phase:'arms'}})};
};
vm.runInContext('stopChassis = () => {}; scanExecution = {active:false};',context);
(async()=>{
  await vm.runInContext('executeScanSequence()',context);
  await vm.runInContext('stopScanSequence()',context);
  assert.deepEqual(requests,[{url:'/api/scan-sequence/start',body:{sku_typ:'bottle'}},{url:'/api/scan-sequence/stop',body:{}}]);
  console.log('PASS: scan button order, progress, motion interlocks and start/stop routes');
})().catch(error=>{console.error(error);process.exitCode=1});
