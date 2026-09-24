const assert = require('node:assert/strict');
const fs = require('node:fs'), vm = require('node:vm'), path = require('node:path');
const root = path.resolve(__dirname, '..');
const html = fs.readFileSync(path.join(root, 'static/index.html'), 'utf8');
const code = fs.readFileSync(path.join(root, 'static/app.js'), 'utf8');
const ids = new Set([...html.matchAll(/id="([^"]+)"/g)].map(m => m[1]));
const elements = new Map();
const el = id => {
  assert(ids.has(id), `missing DOM element: ${id}`);
  if (!elements.has(id)) elements.set(id, {textContent:'', hidden:false, disabled:false, className:''});
  return elements.get(id);
};
const requests = [], messages = [];
let respond;
const ctx = vm.createContext({document:{getElementById:el}, Date, Number, Object, Array, JSON,
  fakeApi: async (url, options) => {requests.push({url, options}); return respond();},
  fakeToast:(...args) => messages.push(args)});
const run = text => vm.runInContext(text, ctx);
run(code.slice(0, code.lastIndexOf('\nrenderCards();')));
run('api=fakeApi; toast=fakeToast;');
const fresh = enabled => run(`serverStatus={armed:true,chassis:{ros_ready:true,obstacle_avoidance:{enabled:${enabled},stale:false,service_ready:true}}};updateChassisAvoidance()`);

(async () => {
  fresh(true);
  assert.equal(el('chassisAvoidanceButton').textContent, '关闭遥控避障');
  assert(!el('chassisAvoidanceButton').disabled);
  fresh(false);
  assert.equal(el('chassisAvoidanceButton').textContent, '开启遥控避障');
  assert(el('chassisAvoidanceBadge').className.includes('danger'));
  for (const change of ['serverStatus.armed=false', 'serverStatus.chassis.obstacle_avoidance.enabled=null',
    'serverStatus.chassis.obstacle_avoidance.stale=true', 'serverStatus.chassis.obstacle_avoidance.service_ready=false',
    'graspBusy=true']) {
    fresh(true); run(`${change};updateChassisAvoidance()`);
    assert(el('chassisAvoidanceButton').disabled, change);
    run('graspBusy=false');
  }
  fresh(true);
  let resolve;
  respond = () => new Promise(r => {resolve=r;});
  const pending = run('toggleChassisAvoidance()');
  assert(el('chassisAvoidanceButton').disabled);
  await run('toggleChassisAvoidance()');
  assert.equal(requests.length, 1);
  assert.equal(requests[0].url, '/api/chassis/obstacle-avoidance');
  assert.equal(requests[0].options.method, 'POST');
  assert.equal(JSON.stringify(requests[0].options.body), '{"enabled":false}');
  // No optimistic state change before the service confirms.
  assert.equal(run('serverStatus.chassis.obstacle_avoidance.enabled'), true);
  resolve({obstacle_avoidance:{enabled:false,stale:false,service_ready:true}});
  await pending;
  assert.equal(el('chassisAvoidanceButton').textContent, '开启遥控避障');
  respond = () => {throw new Error('timeout');};
  await run('toggleChassisAvoidance()');
  assert(el('chassisAvoidanceButton').disabled);
  assert.equal(el('chassisAvoidanceBadge').textContent, '遥控避障：状态未知');
  assert.equal(messages.at(-1)[1], true);
  assert(code.includes('document.getElementById("chassisAvoidanceButton").addEventListener("click", toggleChassisAvoidance)'));
  console.log('PASS: avoidance state, lock/offline/busy, confirmed switching, duplicate prevention and failure display');
})().catch(error => {console.error(error); process.exitCode=1;});
