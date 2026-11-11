'use strict';
/*
 * Browser UI/worker contract (one module Worker per sign, -1 then +1):
 * send {id, kind:'init', data:modelData, sign}, {id, kind:'target', goals},
 *      {id, kind:'step', duration:.04}, {id, kind:'reset'}, {id, kind:'export'}.
 * receive {id, frames, record, error}; export may omit frames.
 * init/reset return t=0; target returns the current boundary, never a reset.
 * Frame fields are those of the frozen demo: t (MODEL time), base/line (L),
 * theta/rotor/wheels (unwrapped radians), u, mean_u, tau, distance, stage, waypoint.
 * record is the frozen session record, including the finite-observation fields.
 * Only accepted paired frames advance the shared display; no extrapolation.
 */
const $=id=>document.getElementById(id), canvases=[$('world0'),$('world1')];
const SIGNS=[-1,1], STEP=.04, START_BUFFER_SECONDS=.08, MAX_BUFFER_SECONDS=.24;
const MAX_FRAMES=512, MAX_TRACE_POINTS=12000, RPC_TIMEOUT_MS=30000, EPS=1e-8;
const colors=['#b56b3c','#277b86'];
// Identical orange rotor, three spokes and unique phase mark to the frozen UI.
const rotorPalette={rim:'#b96f3f',face:'#e9c29b',spoke:'#6f4a32',hub:'#213f49',mark:'#fff9e9'};
const stages={ready:'Ready',navigate:'Navigating',fade:'Fading gait',momentum_brake:'Braking',passive:'Motor off'};
let state=null, modelData=null, workers=[], phase='loading';
let frames=[[],[]], traces=[[],[]], mode='goal', route=[];
let displayT=0, lastNow=0, displayRate=0, rate=0, view={cx:0,cy:0,span:6};
let buffering=true, pending=0, stepInFlight=false, operation=Promise.resolve(), requestId=0;
let rateWindow=[], displayWindow=[], computeWindow=[], lastLabels=0;
let lastNotice='', lastNoticeError=false;
const physicalTime=()=>state?.scales.time_s||1;
const pairHead=()=>frames.every(f=>f.length)?Math.min(...frames.map(f=>f.at(-1).t)):0;

function notice(text,error=false){
  if(text===lastNotice&&error===lastNoticeError)return;
  lastNotice=text;lastNoticeError=error;
  $('notice').setAttribute('role',error?'alert':'status');
  $('notice').setAttribute('aria-live',error?'assertive':'polite');
  $('notice').textContent=text;$('notice').classList.toggle('error',error);
}
function refreshControls(){
  const ready=phase==='ready'&&!pending, failed=phase==='failed';
  ['single','waypoints','send','speed'].forEach(id=>$(id).disabled=!ready);
  $('goalfields').disabled=!ready;
  $('pause').disabled=!ready||!state?.goals.length||state.observed_complete;
  $('pause').textContent=state?.observed_complete?'Observed':state?.paused?'Resume':'Pause';
  $('pause').setAttribute('aria-pressed',String(!!state?.paused));
  $('reset').disabled=!modelData||!!pending||phase==='loading';
  $('download').disabled=!state||!!pending||phase==='loading';
  $('retry').hidden=!failed;
  $('retry').disabled=!!pending||stepInFlight;
  $('worlds').setAttribute('aria-busy',String(phase==='loading'));
}
function statusNotice(){
  if(phase!=='ready')return;
  if(state.observed_complete)notice(state.paused
    ?'Both motor-off tails passed the finite observation test. A new target continues from these states.'
    :'Finite observation complete. Displaying the remaining accepted states on the shared clock.');
  else if(!state.goals.length)notice(mode==='goal'
    ?'Click either world or enter coordinates to start both robots from straight rest.'
    :'Add shared waypoints by clicking either world or entering coordinates, then Run route.');
  else if(state.paused)notice('Paused. Resume playback or choose a new target to continue from the current states.');
  else notice('Browser calculation · both robots share the goals and clock. New targets never reset physical state.');
}
function fail(error){
  const message=error instanceof Error?error.message:String(error);
  phase='failed';displayRate=0;rateWindow=[];buffering=true;
  if(state){state.error=message;state.paused=true;}
  notice('Simulation stopped: '+message+' The shared clock is held. Save diagnostics or reload the model to start a new session.',true);
  refreshControls();
}
class WorkerClient{
  constructor(sign){
    this.sign=sign;this.pending=new Map();this.closed=false;
    this.worker=new Worker(new URL('./worker.js',document.baseURI),{type:'module',name:'Beam '+sign});
    this.worker.onmessage=event=>{
      const reply=event.data, entry=this.pending.get(reply?.id);
      if(!entry)return; // A timed-out or superseded reply cannot mutate the UI.
      this.pending.delete(reply.id);clearTimeout(entry.timer);
      if(reply.error)entry.reject(new Error('Worker '+sign+': '+(reply.error.message||reply.error)));
      else entry.resolve(reply);
    };
    this.worker.onerror=event=>{
      if(this.closed)return;
      event.preventDefault();
      const error=new Error('Worker '+sign+' failed to load or execute: '+(event.message||'unknown worker error'));
      this.close(error);fail(error);
    };
    this.worker.onmessageerror=()=>{
      if(this.closed)return;
      const error=new Error('Worker '+sign+' returned an unreadable message');
      this.close(error);fail(error);
    };
  }
  request(kind,payload={}){
    if(this.closed)return Promise.reject(new Error('Worker '+this.sign+' is unavailable'));
    const id=++requestId;
    return new Promise((resolve,reject)=>{
      const timer=setTimeout(()=>{
        const error=new Error('Worker '+this.sign+' timed out during '+kind+' after 30 s');
        this.close(error);
        fail(error);
      },RPC_TIMEOUT_MS);
      this.pending.set(id,{resolve,reject,timer});
      try{this.worker.postMessage({id,kind,...payload});}
      catch(error){clearTimeout(timer);this.pending.delete(id);reject(error);}
    });
  }
  close(error=new Error('Worker session closed')){
    if(this.closed)return;
    this.closed=true;this.worker.terminate();
    for(const entry of this.pending.values()){clearTimeout(entry.timer);entry.reject(error);}
    this.pending.clear();
  }
}
async function exchange(kind,payload={}){
  // Wait for both acknowledgements even if one rejects; commands never overtake steps.
  const results=await Promise.allSettled(workers.map(w=>w.request(kind,
    kind==='init'?{...payload,sign:w.sign}:payload)));
  const rejected=results.find(r=>r.status==='rejected');
  if(rejected)throw rejected.reason;
  if(results.length!==2)throw new Error('Two simulation workers are required');
  return results.map(r=>r.value);
}
function trimFrames(){
  for(const list of frames){
    let trim=0;
    while(trim+1<list.length&&list[trim+1].t<=displayT)trim++;
    if(trim)list.splice(0,trim); // Retain the interpolation bracket, never skip forward.
  }
}
function validFrame(f){
  const point=p=>Array.isArray(p)&&p.length===2&&p.every(Number.isFinite);
  return f&&Number.isFinite(f.t)&&f.t>=0&&point(f.base)&&Array.isArray(f.line)&&f.line.length>=2&&f.line.every(point)
    &&Array.isArray(f.wheels)&&f.wheels.length===2&&f.wheels.every(Number.isFinite)
    &&['theta','rotor','u','mean_u','tau'].every(k=>Number.isFinite(f[k]))
    &&(f.distance===null||Number.isFinite(f.distance))&&typeof f.stage==='string';
}
function acceptPair(replies,kind){
  const oldHeads=frames.map(f=>f.at(-1)?.t??0);
  const batches=replies.map(reply=>reply.frames);
  // Validate the complete pair before committing either side to the display.
  batches.forEach((batch,i)=>{
    if(!Array.isArray(batch)||!batch.length||batch.length>MAX_FRAMES||!batch.every(validFrame))
      throw new Error('Worker '+SIGNS[i]+' returned invalid rendering frames');
    for(let j=1;j<batch.length;j++){
      if(batch[j].t<batch[j-1].t-EPS||batch[j].line.length!==batch[j-1].line.length)
        throw new Error('Worker frames are not an ordered, compatible interpolation sequence');
    }
    const head=batch.at(-1).t;
    const expected=kind==='init'||kind==='reset'?0:oldHeads[i]+(kind==='step'?STEP:0);
    if(Math.abs(head-expected)>EPS||batch[0].t<(kind==='init'||kind==='reset'?0:oldHeads[i])-EPS)
      throw new Error('Worker '+SIGNS[i]+' violated the shared command-boundary clock');
    if(!replies[i].record||!Number.isFinite(replies[i].record.t)||Math.abs(replies[i].record.t-head)>EPS)
      throw new Error('Worker '+SIGNS[i]+' did not return a record at its accepted frame boundary');
    if(kind==='target'&&frames[i].length){
      const before=frames[i].at(-1), after=batch.at(-1);
      for(const key of ['base','line','theta','rotor','wheels','u']){
        if(JSON.stringify(before[key])!==JSON.stringify(after[key]))
          throw new Error('Retarget changed physical geometry or velocity; the clock is held');
      }
    }
  });
  if(Math.abs(batches[0].at(-1).t-batches[1].at(-1).t)>EPS)
    throw new Error('The workers returned different physical clock boundaries');
  if(kind==='init'||kind==='reset')resetDisplay();
  trimFrames();
  const merged=batches.map((batch,i)=>{
    const byTime=new Map(frames[i].map(f=>[f.t,f]));
    batch.forEach(f=>byTime.set(f.t,f));
    const list=[...byTime.values()].sort((a,b)=>a.t-b.t);
    if(list.length>MAX_FRAMES)throw new Error('Accepted display buffer exceeded its frame limit');
    return list;
  });
  frames=merged;
  batches.forEach((batch,i)=>{
    for(const f of batch){
      if(!traces[i].length||f.t>traces[i].at(-1).t)traces[i].push({t:f.t,p:f.base});
    }
    if(traces[i].length>MAX_TRACE_POINTS)traces[i]=traces[i].filter((_,j)=>j%2===0);
  });
  state.records=replies.map(reply=>reply.record);
}
function resetDisplay(){
  frames=[[],[]];traces=[[],[]];displayT=0;displayRate=0;rate=0;
  buffering=true;rateWindow=[];computeWindow=[];displayWindow=[];
}
function completeRecord(r){
  return r&&Number.isFinite(r.stop_end)&&r.t-r.stop_end>=20
    &&r.stable_duration_model>=10&&r.passive_max_distance_L<=.3;
}
async function boot(){
  phase='loading';refreshControls();
  notice('Loading the model and preparing two browser workers. No recorded trajectory is being played.');
  workers.forEach(w=>w.close());workers=[];
  try{
    if(canvases.some(c=>!c.getContext('2d')))throw new Error('This browser does not support 2D canvas');
    if(typeof Worker==='undefined')throw new Error('This browser does not support Web Workers');
    if(location.protocol==='file:')throw new Error('Open this demo through HTTP or HTTPS, not as a local file');
    if(!modelData){
      const controller=new AbortController();
      const timeout=setTimeout(()=>controller.abort(),RPC_TIMEOUT_MS);
      try{
        const response=await fetch(new URL('./model-data.json',document.baseURI),{signal:controller.signal,credentials:'omit'});
        if(!response.ok)throw new Error('Model data could not be loaded (HTTP '+response.status+')');
        modelData=await response.json();
      }finally{clearTimeout(timeout);}
    }
    for(const key of ['time_s','length_m','velocity_m_s','torque_N_m']){
      if(!Number.isFinite(modelData.scales?.[key])||modelData.scales[key]<=0)
        throw new Error('Model data has no valid '+key+' scale');
    }
    state={scales:modelData.scales,goals:[],paused:true,pace:Number($('speed').value)||1,error:null,observed_complete:false,records:[]};
    for(const sign of SIGNS)workers.push(new WorkerClient(sign));
    acceptPair(await exchange('init',{data:modelData}),'init');
    route=[];view={cx:0,cy:0,span:6};setMode(mode);
    phase='ready';lastNow=performance.now();statusNotice();
  }catch(error){
    workers.forEach(w=>w.close());
    fail(error.name==='AbortError'?new Error('Model download timed out after 30 s'):error);
  }finally{refreshControls();}
}
function validateGoals(goals){
  const count=modelData?.settings.target_max_count??8;
  const radius=modelData?.settings.target_max_radius??8;
  if(!Array.isArray(goals)||!goals.length||goals.length>count)throw new Error('Choose 1–'+count+' shared targets');
  if(goals.some(p=>!Array.isArray(p)||p.length!==2||!p.every(Number.isFinite)||Math.hypot(...p)>radius))
    throw new Error('Choose finite goal coordinates within '+radius+' L of the origin');
  return goals.map(p=>p.slice());
}
function queueOperation(task,{fatal=true}={}){
  pending++;displayRate=0;rateWindow=[];refreshControls();
  const queued=operation.then(task);
  operation=queued.catch(()=>{});
  return queued.catch(error=>{if(fatal)fail(error);else notice(error.message,true);return false;})
    .finally(()=>{pending--;lastNow=performance.now();refreshControls();});
}
async function command(payload){
  if(!state||phase==='loading'){notice('The model is still preparing.');return false;}
  if(phase!=='ready'&&payload.kind!=='reset')return false;
  if(payload.kind==='target'){
    try{payload={...payload,goals:validateGoals(payload.goals)};}
    catch(error){notice(error.message,true);return false;}
  }
  // A pause freezes the displayed clock immediately, including an in-flight batch.
  if(payload.kind==='pause'&&payload.paused)state.paused=true;
  return queueOperation(async()=>{
    if(payload.kind!=='reset'&&phase!=='ready')return false;
    if(payload.kind==='target'){
      acceptPair(await exchange('target',{goals:payload.goals}),'target');
      state.goals=payload.goals;state.paused=document.hidden;state.observed_complete=false;
      buffering=true;computeWindow=[];rate=0;
    }else if(payload.kind==='reset'){
      if(phase==='failed'){await boot();return phase==='ready';}
      acceptPair(await exchange('reset'),'reset');
      state.goals=[];state.paused=true;state.error=null;state.observed_complete=false;
      route=[];view={cx:0,cy:0,span:6};setMode(mode);
    }else if(payload.kind==='pause'){
      state.paused=payload.paused||document.hidden;
    }else if(payload.kind==='pace'){
      if(![.5,1,2].includes(payload.value))throw new Error('Playback must be 0.5×, 1× or 2×');
      state.pace=payload.value;buffering=true;
    }else throw new Error('Unknown UI command');
    statusNotice();return true;
  });
}
async function pump(){
  try{
    const allowed=phase==='ready'&&!pending&&!stepInFlight&&!document.hidden
      &&!state.paused&&!state.observed_complete&&state.goals.length;
    // The next pair must fit in the physical-time cap, even at 2× or in a hidden tab.
    if(allowed&&(pairHead()-displayT+STEP)*physicalTime()<=MAX_BUFFER_SECONDS+EPS){
      stepInFlight=true;const before=pairHead(),start=performance.now();
      const task=operation.then(async()=>{
        acceptPair(await exchange('step',{duration:STEP}),'step');
        const end=performance.now();
        computeWindow.push({now:end,wall:(end-start)/1000,advance:(pairHead()-before)*physicalTime()});
        while(computeWindow.length>1&&end-computeWindow[0].now>1000)computeWindow.shift();
        rate=computeWindow.reduce((s,p)=>s+p.advance,0)/(computeWindow.reduce((s,p)=>s+p.wall,0)||1);
        state.observed_complete=state.records.every(completeRecord);
        if(state.observed_complete){statusNotice();refreshControls();}
      });
      operation=task.catch(()=>{});
      try{await task;}catch(error){fail(error);}
      finally{stepInFlight=false;refreshControls();}
    }
  }finally{
    // Only wait when pacing against a full buffer or an inactive UI. A fixed
    // inter-batch delay would cap throughput independently of solver capacity.
    const needsFrames=phase==='ready'&&!pending&&!document.hidden&&!state.paused
      &&!state.observed_complete&&state.goals.length
      &&(pairHead()-displayT+STEP)*physicalTime()<=MAX_BUFFER_SECONDS+EPS;
    setTimeout(pump,needsFrames?0:8);
  }
}
function bounds(canvas){const w=canvas.clientWidth,h=canvas.clientHeight,scale=w/view.span;return{w,h,scale,x:v=>(v-view.cx)*scale+w/2,y:v=>h/2-(v-view.cy)*scale};}
function fit(){const points=[[-.7,-.7],[1.4,.7],...route,...(state?.goals||[])];frames.forEach(f=>{if(f.length)points.push(...f.at(-1).line);});traces.forEach(tr=>tr.forEach(f=>points.push(f.p)));
  const xs=points.map(p=>p[0]),ys=points.map(p=>p[1]);const aspect=canvases[0].clientWidth/canvases[0].clientHeight;
  view={cx:(Math.min(...xs)+Math.max(...xs))/2,cy:(Math.min(...ys)+Math.max(...ys))/2,span:Math.max(4,Math.max(...xs)-Math.min(...xs)+1.2,(Math.max(...ys)-Math.min(...ys)+1.2)*aspect)};
}
function lerpFrame(list,t){
  if(!list.length||t<list[0].t-1e-9||t>list.at(-1).t+1e-9)return null;
  let lo=0,hi=list.length-1;
  while(lo<hi){const mid=Math.ceil((lo+hi)/2);if(list[mid].t<=t)lo=mid;else hi=mid-1;}
  const a=list[lo],b=list[Math.min(lo+1,list.length-1)],f=b.t>a.t?Math.max(0,Math.min(1,(t-a.t)/(b.t-a.t))):0;
  const mix=(x,y)=>x+(y-x)*f;
  return {...a,base:a.base.map((x,j)=>mix(x,b.base[j])),line:a.line.map((p,k)=>p.map((x,j)=>mix(x,b.line[k][j]))),
    theta:mix(a.theta,b.theta),rotor:mix(a.rotor,b.rotor),wheels:a.wheels.map((x,j)=>mix(x,b.wheels[j])),u:mix(a.u,b.u),mean_u:mix(a.mean_u,b.mean_u),tau:mix(a.tau,b.tau),distance:a.distance==null?null:mix(a.distance,b.distance)};
}
function roundRect(c,x,y,w,h,r){c.beginPath();c.roundRect(x,y,w,h,r);c.fill();c.stroke();}
function drawRotor(c,i,R){
  c.fillStyle=rotorPalette.rim;c.strokeStyle=rotorPalette.hub;c.lineWidth=1.7;
  c.beginPath();c.arc(0,0,R,0,Math.PI*2);c.fill();c.stroke();
  c.fillStyle=rotorPalette.face;c.beginPath();c.arc(0,0,.73*R,0,Math.PI*2);c.fill();
  for(let j=0;j<3;j++){
    const a=j*Math.PI*2/3;
    c.strokeStyle=rotorPalette.spoke;c.lineWidth=1.4;c.beginPath();
    c.moveTo(.34*R*Math.cos(a),.34*R*Math.sin(a));c.lineTo(.88*R*Math.cos(a),.88*R*Math.sin(a));c.stroke();
  }
  // A unique high-contrast radial mark breaks the threefold visual ambiguity.
  // Its outline is spatial only: no averaging, trail or altered angular speed.
  c.lineCap='round';c.beginPath();c.moveTo(.44*R,0);c.lineTo(.91*R,0);
  c.strokeStyle=rotorPalette.hub;c.lineWidth=4.8;c.stroke();
  c.strokeStyle=rotorPalette.mark;c.lineWidth=2.8;c.stroke();
  c.fillStyle=rotorPalette.hub;c.beginPath();c.arc(0,0,.22*R,0,Math.PI*2);c.fill();
}
function draw(i,f){
  const canvas=canvases[i],dpr=Math.min(window.devicePixelRatio||1,2),v=bounds(canvas);
  if(canvas.width!==Math.round(v.w*dpr)||canvas.height!==Math.round(v.h*dpr)){canvas.width=Math.round(v.w*dpr);canvas.height=Math.round(v.h*dpr);}
  const c=canvas.getContext('2d');if(!c)return;c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,v.w,v.h);c.fillStyle='#fcfdf9';c.fillRect(0,0,v.w,v.h);
  const step=view.span>10?1:.5;
  c.lineWidth=1;c.strokeStyle='#c4d0c9';c.beginPath();
  for(let x=Math.floor((view.cx-view.span/2)/step)*step;x<view.cx+view.span/2;x+=step){c.moveTo(v.x(x),0);c.lineTo(v.x(x),v.h);}
  for(let y=Math.floor((view.cy-v.h/2/v.scale)/step)*step;y<view.cy+v.h/2/v.scale;y+=step){c.moveTo(0,v.y(y));c.lineTo(v.w,v.y(y));}c.stroke();
  c.strokeStyle='#98afa8';c.beginPath();c.moveTo(v.x(0),0);c.lineTo(v.x(0),v.h);c.moveTo(0,v.y(0));c.lineTo(v.w,v.y(0));c.stroke();
  c.font='10px Segoe UI';c.fillStyle='#718a81';c.fillText('x (L)',v.w-39,Math.max(16,Math.min(v.h-12,v.y(0)-8)));c.fillText('y (L)',Math.max(12,Math.min(v.w-38,v.x(0)+8)),17);
  c.fillText('0',Math.max(5,Math.min(v.w-15,v.x(0)+5)),Math.max(28,Math.min(v.h-7,v.y(0)+14)));
  c.strokeStyle=colors[i]+'90';c.lineWidth=1.4;c.beginPath();let first=true;
  for(const p of traces[i]){if(p.t>displayT)break;if(first){c.moveTo(v.x(p.p[0]),v.y(p.p[1]));first=false;}else c.lineTo(v.x(p.p[0]),v.y(p.p[1]));}c.stroke();
  const goals=route.length?route:(state?.goals||[]);
  if(goals.length>1){c.setLineDash([4,5]);c.strokeStyle='#a4b4a5';c.beginPath();goals.forEach((p,j)=>j?c.lineTo(v.x(p[0]),v.y(p[1])):c.moveTo(v.x(p[0]),v.y(p[1])));c.stroke();c.setLineDash([]);}
  goals.forEach((p,j)=>{const x=v.x(p[0]),y=v.y(p[1]),current=j===(f?.waypoint||0);c.beginPath();c.arc(x,y,.3*v.scale,0,2*Math.PI);c.fillStyle=current?'#d6954a13':'#9daea80a';c.fill();c.strokeStyle=current?'#bb7e41':'#97a9a0';c.setLineDash([3,4]);c.lineWidth=1;c.stroke();c.setLineDash([]);c.beginPath();c.arc(x,y,6,0,Math.PI*2);c.stroke();c.beginPath();c.moveTo(x-10,y);c.lineTo(x+10,y);c.moveTo(x,y-10);c.lineTo(x,y+10);c.stroke();c.fillStyle='#91643c';c.font='11px Segoe UI';c.fillText(goals.length>1?`${j+1}`:'Goal',x+11,y-10);});
  if(!f)return;
  c.lineCap='round';c.lineJoin='round';c.strokeStyle='#576d71';c.lineWidth=5;c.beginPath();f.line.forEach((p,j)=>j?c.lineTo(v.x(p[0]),v.y(p[1])):c.moveTo(v.x(p[0]),v.y(p[1])));c.stroke();c.strokeStyle='#d7dee0';c.lineWidth=2;c.stroke();
  c.save();c.translate(v.x(f.base[0]),v.y(f.base[1]));c.rotate(-f.theta);const sc=v.scale;
  c.strokeStyle='#253f47';c.lineWidth=1.4;c.beginPath();c.moveTo(0,-.265*sc);c.lineTo(0,.265*sc);c.stroke();
  [-1,1].forEach((side,j)=>{const x=-.0775*sc,y=side*.265*sc-.03*sc;c.fillStyle='#263f48';c.strokeStyle='#52686e';roundRect(c,x,y,.155*sc,.06*sc,Math.min(4,.023*sc));if(Math.cos(f.wheels[j])>0){const mark=.06*sc*Math.sin(f.wheels[j]);c.strokeStyle='#f4eee1';c.lineWidth=1.5;c.beginPath();c.moveTo(mark,y+1);c.lineTo(mark,y+.06*sc-1);c.stroke();}});
  c.restore();c.save();c.translate(v.x(f.base[0]),v.y(f.base[1]));c.rotate(-f.rotor);const R=.18*sc;
  drawRotor(c,i,R);c.restore();
}
function updateMetrics(i,f){
  if(!f)return;
  $('stage'+i).textContent=stages[f.stage]||f.stage;
  $('u'+i).textContent=`${(f.mean_u*(state?.scales.velocity_m_s||1)).toFixed(3)} m/s`;
  $('tau'+i).textContent=`${(f.tau*(state?.scales.torque_N_m||1)).toFixed(3)} N·m`;
  $('d'+i).textContent=f.distance==null?'—':`${(f.distance*(state?.scales.length_m||1)).toFixed(3)} m`;
}
function animate(now){
  const previousT=displayT;
  const wallDt=Math.max(0,(now-lastNow)/1000||0);lastNow=now;
  // Browser stalls never accrue simulation-time debt to be caught up later.
  const dt=wallDt>.25?0:Math.min(.05,wallDt), factor=physicalTime();
  const requested=state?.pace||1;
  if(!dt){displayRate=0;rateWindow=[];buffering=true;}
  let advanced=0;
  if(phase==='ready'&&!state.paused&&!document.hidden&&!pending&&frames.every(f=>f.length)){
    const head=pairHead(),buffer=Math.max(0,(head-displayT)*factor);
    if(buffering&&(state.observed_complete||buffer>=START_BUFFER_SECONDS*Math.min(requested,rate||requested)))
      buffering=false;
    if(!buffering&&dt){
      const target=state.observed_complete?requested:Math.min(requested,buffer/.08);
      displayRate+=(target-displayRate)*Math.min(1,dt*8);
      const before=displayT;
      displayT=Math.min(head,displayT+dt*Math.min(requested,displayRate)/factor);
      advanced=(displayT-before)*factor;
      if(state.observed_complete&&head-displayT<=EPS){
        displayT=head;state.paused=true;statusNotice();refreshControls();
      }else if(buffer<=EPS){buffering=true;displayRate=0;}
    }
  }else{displayRate=0;rateWindow=[];}
  if(wallDt>0){
    rateWindow.push({dt:wallDt,advance:advanced});
    let span=rateWindow.reduce((s,p)=>s+p.dt,0);
    while(rateWindow.length>1&&span-rateWindow[0].dt>=.4)span-=rateWindow.shift().dt;
  }
  const actual=rateWindow.reduce((s,p)=>s+p.advance,0)/(rateWindow.reduce((s,p)=>s+p.dt,0)||1);
  const rendered=[];let maxAngle=0;
  for(let i=0;i<2;i++){
    const current=lerpFrame(frames[i],displayT),previous=advanced>0?lerpFrame(frames[i],previousT):null;
    if(current&&previous)maxAngle=Math.max(maxAngle,Math.abs(current.rotor-previous.rotor)*180/Math.PI);
    rendered.push(current);draw(i,current);
  }
  trimFrames();
  if(wallDt>0&&!document.hidden)displayWindow.push({now,dt:wallDt,angle:maxAngle});
  while(displayWindow.length&&now-displayWindow[0].now>1000)displayWindow.shift();
  if(now-lastLabels>=100){
    lastLabels=now;
    $('clock').innerHTML=(displayT*factor).toFixed(2)+' <em>s</em>';
    $('rate').textContent=phase==='loading'?'Preparing the model…':phase==='failed'?'Simulation error · clock held':
      pending?'Applying command · clock held':document.hidden?'Hidden · clock held':state.paused?'Paused':
      actual.toFixed(2)+'× physical playback'+(buffering?' · buffering':actual<.98*requested?' · compute-limited':'');
    $('gridunit').textContent=(view.span>10?'1':'0.5')+' L';
    $('solver').textContent=rate>0?'Paired compute: '+rate.toFixed(2)+'× physical'+
      (state.paused?' (last active)':'')+' · includes worker messaging; excludes pacing waits.':'Waiting for the first live steps.';
    rendered.forEach((f,i)=>updateMetrics(i,f));
    const hz=displayWindow.length/(displayWindow.reduce((s,p)=>s+p.dt,0)||1);
    const angle=Math.max(0,...displayWindow.map(p=>p.angle));
    $('displaystats').textContent='Display: '+hz.toFixed(0)+' frames/s · largest rotor step: '+angle.toFixed(1)+
      '° per frame (last second). Sharp rendering; no motion blur or rotor-speed scaling.';
  }
  requestAnimationFrame(animate);
}
function updateDraft(){
  $('send').hidden=!route.length;$('clear').hidden=!route.length;
  $('hint').textContent=route.length?route.length+' shared waypoint'+(route.length>1?'s':'')+' · add more or Run route':
    mode==='goal'?'Click to start or change the goal.':'Add shared waypoints, then Run route.';
}
async function selectGoal(point){
  if(phase!=='ready'||pending)return;
  try{validateGoals([point]);}catch(error){notice(error.message,true);return;}
  if(mode==='waypoints'){
    const count=modelData.settings.target_max_count??8;
    if(route.length>=count){notice('A route supports up to '+count+' waypoints.');return;}
    route.push(point);updateDraft();
    notice(route.length+' shared waypoint'+(route.length>1?'s':'')+' in the draft. Run route applies them to both workers.');
  }else{
    if(await command({kind:'target',goals:[point]})){route=[];updateDraft();}
  }
}
canvases.forEach(canvas=>canvas.addEventListener('click',event=>{
  const rect=canvas.getBoundingClientRect(),v=bounds(canvas);
  const point=[view.cx+(event.clientX-rect.left-v.w/2)/v.scale,view.cy-(event.clientY-rect.top-v.h/2)/v.scale];
  void selectGoal(point);
}));
function setMode(next){
  mode=next;route=[];updateDraft();
  ['single','waypoints'].forEach((id,j)=>{
    const selected=(mode==='goal')===(j===0);
    $(id).classList.toggle('selected',selected);$(id).setAttribute('aria-pressed',String(selected));
  });
  $('addgoal').textContent=mode==='goal'?'Set shared goal':'Add waypoint';
}
$('single').onclick=()=>{setMode('goal');statusNotice();};
$('waypoints').onclick=()=>{setMode('waypoints');statusNotice();};
$('clear').onclick=()=>{route=[];updateDraft();statusNotice();};
$('goalform').onsubmit=event=>{
  event.preventDefault();
  if(!$('goalform').reportValidity())return;
  void selectGoal([$('goalx').valueAsNumber,$('goaly').valueAsNumber]);
};
$('send').onclick=async()=>{
  if(route.length&&await command({kind:'target',goals:route})){
    route=[];updateDraft();$('hint').textContent='Add waypoints to build a replacement route.';
  }
};
$('pause').onclick=()=>command({kind:'pause',paused:!state.paused});
$('reset').onclick=()=>command({kind:'reset'});
$('retry').onclick=()=>queueOperation(async()=>{modelData=null;await boot();});
$('fit').onclick=fit;
$('speed').onchange=()=>command({kind:'pace',value:Number($('speed').value)});
$('download').onclick=()=>queueOperation(async()=>{
  // Export is a barrier, not a step or reset. Preserve partial diagnostics on failure.
  const exported=await Promise.allSettled(workers.map(w=>w.request('export')));
  const records=SIGNS.map((sign,i)=>{
    const result=exported[i];
    if(result?.status==='fulfilled'&&result.value.record)return result.value.record;
    return {...state.records[i],sign,export_error:result?.reason?.message||'No export record available'};
  });
  const data={
    schema_version:'beam-browser-run-v1',created_at:new Date().toISOString(),
    model_schema:modelData.schema_version,model_provenance:modelData.provenance,
    signs:SIGNS,records,goals:state.goals,scales:state.scales,
    displayed_model_time:displayT,displayed_physical_time_s:displayT*physicalTime(),
    physical_clock_shared:true,requested_pace:state.pace,paused:state.paused,
    observed_complete:state.observed_complete,error:state.error,
    maximum_lookahead_physical_s:MAX_BUFFER_SECONDS,step_batch_model_time:STEP,
    paired_compute_physical_rate:rate,
    scope:'Exploratory browser finite-model demonstration; no general real-time, global-navigation or all-future stopping guarantee.'
  };
  const url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'}));
  const link=document.createElement('a');link.href=url;link.download='beam-browser-diagnostics.json';
  document.body.appendChild(link);link.click();link.remove();
  setTimeout(()=>URL.revokeObjectURL(url),1000);
  const partial=records.some(r=>r.export_error);
  if(phase==='ready')notice(partial?'Partial diagnostics downloaded; one worker could not export.':'Diagnostics downloaded locally. Nothing was uploaded.',partial);
  return true;
},{fatal:false});
window.addEventListener('resize',()=>canvases.forEach(c=>c.width=0));
document.addEventListener('visibilitychange',()=>{
  lastNow=performance.now();displayRate=0;rateWindow=[];buffering=true;
  if(document.hidden&&phase==='ready'&&!state.paused)void command({kind:'pause',paused:true});
});
window.addEventListener('pagehide',event=>{if(!event.persisted)workers.forEach(w=>w.close());});
$('u0').parentElement.title=$('u1').parentElement.title='Rolling mean over one reference period; shorter during startup.';
void boot();void pump();requestAnimationFrame(animate);
