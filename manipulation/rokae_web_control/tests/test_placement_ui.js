const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const root=path.resolve(__dirname,'..'), html=fs.readFileSync(path.join(root,'static/index.html'),'utf8');
assert(html.indexOf('id="placementButton"')>html.indexOf('id="scanSequenceStatus"'));
const elements=new Map(),el=id=>{
 if(!elements.has(id))elements.set(id,{hidden:false,disabled:false,textContent:'',value:'',dataset:{},listeners:{},addEventListener(type,fn){(this.listeners[type]??=[]).push(fn)},removeAttribute(k){delete this[k]}});
 return elements.get(id);
};
const context=vm.createContext({document:{getElementById:el,querySelectorAll:()=>[],addEventListener(){}},window:{addEventListener(){}},setTimeout:()=>1,clearTimeout:()=>{},Date,Number,Boolean,Object,Array,JSON});
const source=fs.readFileSync(path.join(root,'static/app.js'),'utf8');
vm.runInContext(source.slice(0,source.lastIndexOf('\nrenderCards();')),context);
const run=s=>vm.runInContext(s,context);
run('bindEvents()');
assert.equal(el('placementButton').listeners.click?.length,1,'placement button must have exactly one click handler');
run("serverStatus={armed:true,placement:{version:'placement-v7'}};gripperState={unlocked:true};updatePlacementControls()");
assert(el('placementButton').disabled);
run("renderPoseResult({usable:true,selected_target:'basket',result_id:'saved-basket',localization:{valid:true,right_shoulder_frame:'right_arm_sdk_world',point_right_shoulder_mm:[1,2,3]},response:{}})");
assert(!el('placementButton').disabled);
run("renderPoseResult({usable:false,selected_target:'box',localization:{valid:false},response:{}})");
assert(!el('placementButton').disabled); // selecting a SKU does not erase the last basket
const requests=[];let release;
context.fetch=async(url,opts)=>{
 requests.push({url,body:JSON.parse(opts.body)});
 await new Promise(r=>release=r);
 return {ok:true,json:async()=>({ok:true,data:{active:true,phase:'moving',message:'test'}})};
};
run('stopChassis=()=>{}');
(async()=>{
 const pending=el('placementButton').listeners.click[0]({type:'click',currentTarget:el('placementButton')});
 assert(el('placementButton').disabled);
 assert(el('scanSequenceButton').disabled);
 assert(el('poseEstimateButton').disabled);
 await run('executePlacement()');assert.equal(requests.length,1);
 assert.deepEqual(requests[0],{url:'/api/placement/start',body:{source_result_id:'saved-basket'}});
 release();await pending;
 assert(!el('placementStop').disabled);
 run("placementExecution={active:false,phase:'idle'};renderPoseResult({usable:false,selected_target:'basket',localization:{valid:false},response:{}})");
 assert(el('placementButton').disabled);
 console.log('PASS: placement basket source, one-shot dispatch, busy controls and failed basket invalidation');
})().catch(e=>{console.error(e);process.exitCode=1});
