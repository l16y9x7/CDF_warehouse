const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const root=path.resolve(__dirname,'..');
const html=fs.readFileSync(path.join(root,'static/index.html'),'utf8');
const map=new Map();
const el=id=>{
  if(!map.has(id))map.set(id,{textContent:'',disabled:false,hidden:false,value:'',dataset:{},removeAttribute(k){delete this[k]}});
  return map.get(id);
};
assert(!html.includes('data-pose-target'));
assert(html.includes('id="poseEstimateButton" class="primary">获取位姿</button>'));
const select=html.match(/<select id="poseTargetSelect">([\s\S]*?)<\/select>/)[1];
assert.deepEqual([...select.matchAll(/<option value="([^"]+)">([^<]+)<\/option>/g)].map(m=>[m[1],m[2]]),
  [['bottle','白色罐子（bottle）'],['box','方盒（box）'],['tube','软管（tube）'],['basket','篮筐']]);
assert(html.indexOf('id="poseLocationChassis"')<html.indexOf('id="poseLocationShoulder"'));
assert(html.indexOf('id="poseLocationChassis"')<html.indexOf('id="poseLocationLeftShoulder"'));
assert(html.indexOf('id="poseLocationShoulder"')<html.indexOf('id="poseReferenceCamera"'));
assert(html.indexOf('id="poseLocationLeftShoulder"')<html.indexOf('id="poseReferenceCamera"'));
assert(html.indexOf('id="poseLocationShoulder"')<html.indexOf('id="poseLocationLeftShoulder"'));
el('poseEstimateButton').textContent='获取位姿';
el('poseTargetSide').value='LEFT';
const context=vm.createContext({document:{getElementById:el,querySelectorAll:()=>[]},
  setTimeout:()=>1,clearTimeout:()=>{},Date,Number,Boolean,Object,Array,JSON});
const source=fs.readFileSync(path.join(root,'static/app.js'),'utf8');
vm.runInContext(source.slice(0,source.lastIndexOf('\nrenderCards();')),context);
vm.runInContext('refreshStatus=async()=>{};checkPoseService=async()=>{};setPoseServiceBadge=()=>{};',context);
const requests=[];
let release=null,hold=false,fail=false;
context.fetch=async(url,options)=>{
  const body=JSON.parse(options.body);requests.push({url,body});
  if(hold)await new Promise(resolve=>{release=resolve});
  if(fail)throw new Error('test failure');
  return {ok:true,json:async()=>({ok:true,data:{status:'success',usable:true,selected_target:body.sku_typ || body.target,
    localization:{valid:true,label:body.sku_typ || body.target,point_semantics:'test',point_camera_mm:[1,2,3],point_chassis_mm:[4,5,6],
      ...(body.target==='basket'?{point_right_shoulder_mm:[-198,99,200],point_left_shoulder_mm:[202,99,200]}:{})},
    response:{},relative_directory:'test',grasp_test_usable:false}})};
};
const choose=target=>{el('poseTargetSelect').value=target;vm.runInContext('selectPoseTarget()',context)};
(async()=>{
  for(const target of ['bottle','box','tube','basket']){
    choose(target);
    await vm.runInContext('estimateGraspObjectPose()',context);
    assert.equal(el('poseEstimateButton').textContent,'获取位姿');
    assert.equal(el('poseLocationShoulderRow').hidden,target!=='basket');
    assert.equal(el('poseLocationLeftShoulderRow').hidden,target!=='basket');
  }
  assert.deepEqual(requests.map(r=>r.body),[{sku_typ:'bottle',side:'RIGHT'},{sku_typ:'box',side:'LEFT'},
    {sku_typ:'tube',side:'RIGHT'},{target:'basket'}]);
  assert(requests.every(r=>r.url==='/api/pose-estimation/grasp-object'));
  assert.equal(el('poseReprojectButton').disabled,true);
  assert.equal(el('poseTargetSide').disabled,true);
  assert(el('poseLocationShoulder').textContent.includes('-198'));
  assert(el('poseLocationLeftShoulder').textContent.includes('202'));
  choose('bottle');
  assert.equal(el('poseLocationShoulderRow').hidden,true);
  assert.equal(el('poseLocationLeftShoulderRow').hidden,true);
  assert.equal(el('poseLocationShoulder').textContent,'—');
  assert.equal(el('poseLocationLeftShoulder').textContent,'—');
  assert.equal(el('poseResultData').hidden,true);
  hold=true;
  const pending=vm.runInContext('estimateGraspObjectPose()',context);
  const controls=['poseEstimateButton','poseTargetSelect','poseTargetSide'];
  assert(controls.every(id=>el(id).disabled));
  const count=requests.length;
  await vm.runInContext('estimateGraspObjectPose()',context);
  assert.equal(requests.length,count);
  release();await pending;hold=false;
  assert(controls.filter(id=>id!=='poseTargetSide').every(id=>!el(id).disabled));
  assert(el('poseTargetSide').disabled);
  choose('basket');await vm.runInContext('estimateGraspObjectPose()',context);
  vm.runInContext("renderPoseResult({status:'invalid',usable:false,selected_target:'basket',localization:{valid:false},response:{}})",context);
  assert.equal(el('poseLocationShoulder').textContent,'—');
  assert.equal(el('poseLocationLeftShoulder').textContent,'—');
  assert.equal(el('poseReferenceCamera').textContent,'—');
  assert.equal(el('poseLocationChassis').textContent,'—');
  assert.equal(vm.runInContext('graspSourceId',context),null);
  fail=true;await vm.runInContext('estimateGraspObjectPose()',context);
  assert.equal(el('poseLocationShoulderRow').hidden,true);
  assert.equal(el('poseLocationLeftShoulderRow').hidden,true);
  assert.equal(el('poseResultData').hidden,true);
  assert.equal(el('poseEstimateButton').textContent,'获取位姿');
  console.log('PASS: selector routing, busy lock, basket shoulder row, stale and failed result clearing');
})().catch(e=>{console.error(e);process.exitCode=1});
