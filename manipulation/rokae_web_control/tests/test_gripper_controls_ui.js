const assert = require('node:assert/strict');
const fs = require('node:fs'), path = require('node:path'), vm = require('node:vm');
const root = path.resolve(__dirname, '..');
const html = fs.readFileSync(path.join(root, 'static/index.html'), 'utf8');
const code = fs.readFileSync(path.join(root, 'static/app.js'), 'utf8');
for (const id of ['gripperLockButton', 'gripperUnlockButton']) {
  assert(!html.includes(id));
  assert(!code.includes(id));
}
const ids = new Set([...html.matchAll(/id="([^"]+)"/g)].map(m => m[1]));
const elements = new Map();
const el = id => {
  if (!ids.has(id)) return null;
  if (!elements.has(id)) elements.set(id, {value:'', textContent:'', disabled:false,
    dataset:{}, listeners:{}, addEventListener(name, fn) {this.listeners[name] = fn;}});
  return elements.get(id);
};
const ctx = vm.createContext({document:{getElementById:el, querySelectorAll:()=>[], addEventListener(){}},
  window:{addEventListener(){}}, setTimeout:()=>1, clearTimeout(){}, Date, Number, Object, Array, JSON});
const run = text => vm.runInContext(text, ctx);
run(code.slice(0, code.lastIndexOf('\nrenderCards();')));
run('bindEvents()'); // A dangling listener for a removed node must fail here.
run('serverStatus={armed:false,gripper:{unlocked:false}};gripperState={activation_state:3,fault_code:0};updateGripperControls()');
for (const id of ['gripperActivateButton','gripperOpenButton','gripperCloseButton','gripperPositionSlider']) assert(el(id).disabled);
run('serverStatus={armed:true,gripper:{unlocked:true}};gripperState={activation_state:0,fault_code:0};updateGripperControls()');
assert(!el('gripperActivateButton').disabled);
assert(el('gripperOpenButton').disabled);
run('gripperState.activation_state=3;updateGripperControls()');
assert(el('gripperActivateButton').disabled);
for (const id of ['gripperOpenButton','gripperCloseButton','gripperPositionSlider']) assert(!el(id).disabled);
run('graspExecution.active=true;updateGripperControls()');
assert(el('gripperCloseButton').disabled);
run('graspExecution.active=false;gripperState.fault_code=9;updateGripperControls()');
assert(el('gripperOpenButton').disabled);
run('gripperState.fault_code=0;serverStatus.armed=false;updateGripperControls()');
assert(el('gripperCloseButton').disabled);
const calls=[];ctx.record=(...args)=>calls.push(args);
run('activateGripper=()=>record("activate");moveGripper=p=>record("position",p);bindEvents()');
el('gripperActivateButton').listeners.click();
el('gripperOpenButton').listeners.click();
el('gripperCloseButton').listeners.click();
el('gripperPositionSlider').listeners.change({target:{value:'42'}});
assert.deepEqual(calls,[['activate'],['position',0],['position',255],['position',42]]);
console.log('PASS: no independent lock buttons, global control gate, initialization/fault/busy gates, all remaining click bindings');
