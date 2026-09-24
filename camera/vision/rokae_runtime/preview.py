"""Same-origin live view of the cameras already owned by this process."""

PREVIEW_HTML = r'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ROKAE 视觉预览</title><style>
*{box-sizing:border-box}body{margin:0;background:#10151e;color:#edf2f8;font:16px system-ui,sans-serif}
main{max-width:1440px;margin:auto;padding:32px 24px}header{display:flex;justify-content:space-between;gap:20px;align-items:center}
h1{font-size:26px;margin:0 0 8px}p{color:#9eafc5;margin:8px 0}#connection{font-size:14px;color:#b4c2d4}
#views{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,420px),1fr));gap:20px;margin-top:28px}
.card{border:1px solid #304055;border-radius:14px;overflow:hidden;background:#182231}.bar{display:flex;justify-content:space-between;padding:16px}
.stage{aspect-ratio:4/3;position:relative;background:#090e16;display:grid;place-items:center}.stage img{width:100%;height:100%;object-fit:contain;position:absolute}
.stage span{color:#92a3b9;padding:24px;text-align:center}.status{color:#efbb6a}.status.live{color:#66d3aa}
footer{margin-top:24px;color:#9eafc5;font-size:14px}button{background:#20344c;border:1px solid #486381;border-radius:8px;color:white;padding:9px 16px;cursor:pointer}
[hidden]{display:none!important}
</style></head><body><main><header><div><h1>ROKAE 视觉预览</h1><p>现场相机 · 实时画面</p></div><button id="refresh">重新连接</button></header>
<div id="connection" role="status">正在连接相机…</div><section id="views"></section><footer id="disabled"></footer>
<script>
const names={head:'头部',hand_left:'左手',left_wrist:'左手',right_wrist:'右手',hand_right:'右手',right_wrist:'右手'};
const cards=new Map();let busy=false;
function card(id){
 if(cards.has(id))return cards.get(id);
 const el=document.createElement('article');el.className='card';
 const bar=document.createElement('div');bar.className='bar';
 const label=document.createElement('strong');label.textContent=names[id];
 const state=document.createElement('span');state.className='status';bar.append(label,state);
 const stage=document.createElement('div');stage.className='stage';
 const hint=document.createElement('span');hint.textContent='等待画面';
 const img=document.createElement('img');img.alt=names[id]+'实时画面';img.hidden=true;stage.append(hint,img);el.append(bar,stage);
 document.querySelector('#views').append(el);
 const value={el,state,hint,img,failed:false};
 img.onerror=()=>{value.failed=true;img.hidden=true;hint.hidden=false;hint.textContent='连接中断，正在重连…';};
 img.onload=()=>{value.failed=false;img.hidden=false;hint.hidden=true;};
 cards.set(id,value);return value;
}
function stop(c){c.img.removeAttribute('src');c.img.hidden=true;c.hint.hidden=false;c.failed=false;}
async function poll(){
 if(busy)return;busy=true;
 try{
  const response=await fetch('/camera/list',{cache:'no-store',signal:AbortSignal.timeout(4000)});
  if(!response.ok)throw new Error('HTTP '+response.status);
  const body=await response.json();const rows=Array.isArray(body)?body:body.cameras;if(!Array.isArray(rows))throw new Error('invalid listing');
  const disabled=[];let live=0,enabled=0;
  for(const id of Object.keys(names)){
   const row=rows.find(r=>r.id===id);const c=card(id);
   if(!row||!row.enabled){c.el.hidden=true;stop(c);disabled.push(names[id]);continue;}
   c.el.hidden=false;enabled++;
   if(row.ready){live++;c.state.textContent='正在采集';c.state.className='status live';
    if(!c.img.hasAttribute('src')||c.failed){c.failed=false;c.img.hidden=false;c.hint.hidden=true;c.img.src='/camera/stream?camera='+encodeURIComponent(id)+'&t='+Date.now();}
   }else{stop(c);c.state.textContent='暂无画面';c.state.className='status';c.hint.textContent='相机尚未出帧，正在等待恢复';}
  }
  document.querySelector('#connection').textContent=live+' / '+enabled+' 路相机就绪 · '+new Date().toLocaleTimeString();
  document.querySelector('#disabled').textContent=disabled.length?'未启用：'+disabled.join('、'):'';
 }catch(e){document.querySelector('#connection').textContent='相机服务暂时不可达，正在重连…';
  for(const c of cards.values()){stop(c);c.state.textContent='连接中断';c.state.className='status';c.hint.textContent='等待服务恢复';}
 }finally{busy=false;}
}
document.querySelector('#refresh').onclick=()=>{for(const c of cards.values())stop(c);poll();};
poll();setInterval(poll,2000);
</script></main></body></html>'''
