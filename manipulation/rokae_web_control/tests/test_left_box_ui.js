const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const root=path.resolve(__dirname,'..'), map=new Map();
const el=id=>{if(!map.has(id))map.set(id,{value:'',textContent:'',disabled:false,hidden:false,dataset:{},removeAttribute(){}});return map.get(id)};
const c=vm.createContext({document:{getElementById:el,querySelectorAll:()=>[]},setTimeout:()=>1,clearTimeout:()=>{},Date,Number,Object,Array,JSON});
const code=fs.readFileSync(path.join(root,'static/app.js'),'utf8');
vm.runInContext(code.slice(0,code.lastIndexOf('\nrenderCards();')),c);
vm.runInContext("stopChassis=()=>{}; updateMoveLControls=()=>{updateGraspControls();updateScanControls()}; serverStatus={armed:true,grasp_test:{version:'grasp-test-v20'},scan_sequence:{version:'scan-sequence-v4'}}",c);
const requests=[];
c.fetch=async(url,options)=>{
  requests.push({url,body:JSON.parse(options.body)});
  const left=url.includes('left-shoulder'),right=url.includes('right-shoulder');
  const result=(left||right)?{sku_typ:left?'box':'bottle',source_result_id:'saved-'+(left?'box':'bottle'),
    world_grasp:{sku_typ:left?'box':'bottle',grasp_height_trunk_mm:800,recognition_height_trunk_mm:700,
      grasp_pose_world_mm_deg:[500,100,800,180,-90,0],grasp_offset_trunk_mm:[0,left?0:10,0]},
    shoulder_grasp:{},current_trunk_joints_deg:[0,0,0,0]}:{version:'grasp-test-v20',active:false,phase:'idle'};
  return {ok:true,json:async()=>({ok:true,data:result})};
};
const choose=t=>{el('poseTargetSelect').value=t;vm.runInContext('selectPoseTarget()',c)};
(async()=>{
  choose('box'); assert.equal(el('poseTargetSide').value,'LEFT');assert(el('poseTargetSide').disabled);
  assert(!el('poseReprojectLeftButton').disabled);assert(el('poseReprojectButton').disabled);
  el('poseElbow-left_arm').value='-12';el('poseElbow-right_arm').value='49';
  await vm.runInContext('reprojectGraspToLeftShoulder()',c);
  assert(!el('graspTestButton').disabled);assert(!el('scanSequenceButton').disabled);
  assert(el('poseShoulderGraspLabel').textContent.startsWith('左'));
  assert(el('poseRecognitionHeight').textContent.includes('独立标定'))
  await vm.runInContext('executeScanSequence()',c);
  assert.deepEqual(requests.at(-1).body,{sku_typ:'box'});;
  await vm.runInContext('executeGraspTest()',c);
  assert.deepEqual(requests.at(-1).body,{source_result_id:'saved-box',sku_typ:'box',elbow_deg:-12});
  choose('bottle');assert(el('graspTestButton').disabled);assert.equal(el('poseTargetSide').value,'RIGHT');
  await vm.runInContext('reprojectGraspToRightShoulder()',c);await vm.runInContext('executeGraspTest()',c);
  assert.deepEqual(requests.at(-1).body,{source_result_id:'saved-bottle',sku_typ:'bottle',elbow_deg:49});
  choose('tube');const count=requests.length;await vm.runInContext('executeGraspTest()',c);assert.equal(requests.length,count);
  choose('box');await vm.runInContext('reprojectGraspToLeftShoulder()',c);
  vm.runInContext("serverStatus.grasp_test.version='grasp-test-v11';updateGraspControls()",c);
  assert(el('graspTestButton').disabled);
  console.log('PASS: canonical types, forced box side, left conversion, arm-specific seed, stale-result clearing, unsupported/old-backend lock');
})().catch(e=>{console.error(e);process.exitCode=1});
