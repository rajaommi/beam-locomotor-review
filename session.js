import {Mechanics,norm,wrap,clip} from './core.js';
const TAU=2*Math.PI;
const copyState=s=>({q:s.q.slice(),v:s.v.slice(),u:s.u,psi:s.psi,pd:s.pd,b:s.b.slice()});
const low=(s,k=1)=>Math.abs(s.u)<=k*.001&&Math.abs(s.v[0])<=k*.001&&norm(s.v.subarray(1))<=k*.005&&Math.abs(s.pd)<=k*.01;
export function validateTargets(g){if(!Array.isArray(g)||g.length<1||g.length>8||g.some(p=>!Array.isArray(p)||p.length!==2||p.some(v=>typeof v!=='number'||!Number.isFinite(v))||Math.hypot(...p)>8))throw Error('Choose 1–8 finite goals within 4 m of the origin.');return g.map(p=>p.slice());}
class Reference {
  constructor(r){this.values=r.values;this.period=r.period;this.amplitude=r.amplitude;this.sha256=r.sha256;this.mean=r.mean;}
  sample(t){const n=this.values.length,x=((t/this.period)%1+1)%1*n,i=Math.floor(x)%n,a=x-Math.floor(x);return this.values[i].map((v,j)=>(1-a)*v+a*this.values[(i+1)%n][j]);}
  feedforward(t){return this.amplitude*Math.sin(TAU*t/this.period);}
}
class Controller {
  constructor(reference,goals,config,sign){Object.assign(this,{reference,goals,config,sign,waypoint:0,amplitude:0,bearing:0,stop_start:null,stop_end:null,release_since:null,stage:'navigate',events:[],momentum:0,tau:0});}
  retarget(goals,s,t){if(this.stage!=='navigate')this.bearing=s.q[0]-this.amplitude*(this.reference.sample(t)[0]-this.reference.mean);this.goals=goals;this.waypoint=0;this.stop_start=this.stop_end=this.release_since=null;this.stage='navigate';this.events.push({kind:'retarget',t,goals,physical_state_reset:false});}
  command(mech,s,t,dt){
    const c=this.config,oldStage=this.stage,oldA=this.amplitude,oldB=this.bearing;let delta=this.goals[this.waypoint].map((v,i)=>v-s.b[i]),distance=Math.hypot(...delta);
    if(this.stage==='navigate'&&this.waypoint<this.goals.length-1&&distance<=c.waypoint_radius){this.events.push({kind:'waypoint_accepted',t,index:this.waypoint,distance});this.waypoint++;delta=this.goals[this.waypoint].map((v,i)=>v-s.b[i]);distance=Math.hypot(...delta);}
    if(this.stage==='navigate'&&this.waypoint===this.goals.length-1&&distance<=c.capture_trigger){this.stop_start=t;this.stage='fade';this.events.push({kind:'capture_trigger',t,distance});}
    if(this.stage==='fade'&&t-this.stop_start>=c.fade_duration-1e-10){this.stage='momentum_brake';this.events.push({kind:'momentum_brake_started',t});}
    if(this.stage==='momentum_brake'){
      if(t-this.stop_start>c.max_brake_time)throw Error('Momentum braking exceeded its bounded horizon.');
      if(low(s,.5)){if(this.release_since===null)this.release_since=t;if(t-this.release_since>=c.release_dwell-1e-10){this.stage='passive';this.stop_end=t;this.events.push({kind:'motor_off',t,distance});}}else this.release_since=null;
    }
    const law=new Float64Array(10);
    if(this.stage==='passive')return {law,stage_changed:this.stage!==oldStage};
    if(this.stage==='momentum_brake'){
      this.amplitude=0;const M=mech.blocks(s.q,s.v,s.u).M,m=mech.m;this.momentum=M[m]*s.u+M[2*m-1]*s.pd;for(let k=0;k<mech.n;k++)this.momentum+=M[m+k+1]*s.v[k];
      law.set([2,c.tau_max,c.rotor_friction,0,c.momentum_kd,0,0,0,c.momentum_gain,c.momentum_rate_max]);
      this.tau=clip(c.rotor_friction*s.pd+c.momentum_kd*(s.v[0]-clip(c.momentum_gain*this.momentum,-c.momentum_rate_max,c.momentum_rate_max)),-c.tau_max,c.tau_max);
    }else{
      const thf=this.reference.sample(t)[0],heading=s.q[0]-this.amplitude*(thf-this.reference.mean);let rate=0,targetAmp=0;
      if(this.stage==='navigate'){const bearing=Math.atan2(delta[1],delta[0])+(this.sign<0?Math.PI:0);rate=clip(c.bearing_gain*wrap(bearing-this.bearing),-c.bearing_rate_max,c.bearing_rate_max);targetAmp=Math.min(1,Math.sqrt(distance/c.amplitude_distance))*Math.max(0,Math.cos(wrap(bearing-heading)))**2;}
      const ampDot=clip((targetAmp-this.amplitude)/(this.stage==='navigate'?c.amplitude_time:c.fade_time),-c.amplitude_slew,c.amplitude_slew);this.amplitude+=dt*ampDot;this.bearing+=dt*rate;
      // Exactly the midpoint-feedback realization, including differences of memories.
      const ampm=.5*(oldA+this.amplitude),ad=(this.amplitude-oldA)/dt,bd=(this.bearing-oldB)/dt,[th,om,pd]=this.reference.sample(t+.5*dt);
      const ampm0=oldA+.5*dt*ampDot,desired0=oldB+.5*dt*rate+ampm0*(th-this.reference.mean),desiredRate0=rate+ampm0*om+ampDot*(th-this.reference.mean);
      let raw=ampm0*this.reference.feedforward(t+.5*dt);raw+=c.rotor_friction*(s.pd-ampm0*pd);raw+=c.kp*wrap(s.q[0]-desired0)+c.kd*(s.v[0]-desiredRate0);this.tau=clip(raw,-c.tau_max,c.tau_max);
      const desired=.5*(oldB+this.bearing)+ampm*(th-this.reference.mean),desiredRate=bd+ampm*om+ad*(th-this.reference.mean),offset=ampm*this.reference.feedforward(t+.5*dt)-c.rotor_friction*ampm*pd;
      law.set([1,c.tau_max,c.rotor_friction,c.kp,c.kd,desired,desiredRate,offset,0,0]);
    }
    return {law,stage_changed:this.stage!==oldStage};
  }
  metadata(){return {identity:'browser-port-of-RO-midpoint-feedback-v1',physical_input_dimension:1,config:this.config,certified:false,reference_period:this.reference.period,propulsion_sign:this.sign,terminal_policy:'momentum_brake',fast_feedback:'implicit midpoint',guidance:'once per step'};}
}

export class Session {
  constructor(data,sign){if(sign!==1&&sign!==-1)throw Error('Invalid propulsion sign');this.data=data;this.sign=sign;this.mech=new Mechanics(data);this.reference=new Reference(data.references[sign===1?'positive':'negative']);this.dt=.004;this.reset();}
  reset(){const n=this.mech.n;this.s={q:new Float64Array(n),v:new Float64Array(n),u:0,psi:0,pd:0,b:new Float64Array(2)};this.H=null;this.cacheDt=null;this.ctrl=null;this.k=0;this.compute=0;this.work=this.loss=this.tau=this.signed_travel=0;this.rolling=[0,0];this.window=[];this.low_since=null;this.stable=this.passive_max=0;this.events=[];this.adaptive_events=[];this.adaptive_last_event=null;this.mission_start=0;this.goals=[];this.samples=[];this.sample_stride=10;this.counters={macrosteps:0,fallback_macrosteps:0,rescued_macrosteps:0,failed_macrosteps:0,exhausted_macrosteps:0,parent_calls:0,newton_failures:0,trial_successful_substeps:0,committed_substeps:0};this.solverStats={calls:0,jcalls:0,iterations:0};}
  get t(){return this.k*this.dt;}
  vector(){const s=this.s;return [...s.q,...s.v,s.u,s.psi,s.pd,...s.b];}
  target(goals){const values=validateTargets(goals);if(!this.ctrl)this.ctrl=new Controller(this.reference,values,{...this.data.config,rotor_inertia:this.data.jr,rotor_friction:this.data.cp},this.sign);else{this.ctrl.retarget(values,this.s,this.t);this.H=null;}this.goals=values;this.low_since=null;this.stable=this.passive_max=0;this.mission_start=this.t;this.events.push({kind:'target',t:this.t,goals:values,physical_state_reset:false});}
  singleStep(dt,law,changed){
    const old=this.s,energyStart=this.mech.energy(old.q,old.v,old.u,old.pd);if(changed)this.H=null;
    let r=this.mech.solve(old,dt,law,this.H,24);this.addStats(r);
    if(!r.ok){this.H=null;r=this.mech.solve(old,dt,law,null,96);this.addStats(r);}
    if(!r.ok){const e=Error('Implicit midpoint did not converge');e.newton=true;throw e;}
    const uMid=.5*(old.u+r.u),omMid=.5*(old.v[0]+r.v[0]),pdMid=.5*(old.pd+r.pd),qm=old.q.map((v,i)=>.5*(v+r.q[i])),vm=old.v.map((v,i)=>.5*(v+r.v[i]));
    const torque=this.mech.torque(qm,vm,uMid,pdMid,law);let dx=uMid*dt,dy=0;
    if(Math.abs(omMid*dt)>=1e-8){dx=uMid/omMid*Math.sin(omMid*dt);dy=uMid/omMid*(1-Math.cos(omMid*dt));}
    const ct=Math.cos(old.q[0]),st=Math.sin(old.q[0]),b=new Float64Array([old.b[0]+ct*dx-st*dy,old.b[1]+st*dx+ct*dy]);
    const next={q:r.q,v:r.v,u:r.u,psi:r.psi,pd:r.pd,b},energyEnd=this.mech.energy(r.q,r.v,r.u,r.pd);
    if(this.vectorOf(next).some(v=>!Number.isFinite(v))||!Number.isFinite(energyEnd))throw Error('Non-finite mechanical state');
    if(Math.abs(r.pd)>500)throw Error('Relative rotor speed limit exceeded');if(energyEnd>1e5)throw Error('Energy limit exceeded');
    this.s=next;this.H=r.H;
    const d=this.data;let qLoss=0;for(let i=1;i<vm.length;i++)qLoss+=vm[i]**2;
    return {u_mid:uMid,omega_mid:omMid,psi_dot_mid:pdMid,motor_power:torque.tau*pdMid,dissipation_power:d.cu*uMid**2+d.cw*omMid**2+d.cq*qLoss+d.cp*pdMid**2,tau_applied:torque.tau,tau_unsaturated:torque.raw,saturated:Number(Math.abs(torque.tau-torque.raw)>1e-14),converged:1,energy_start:energyStart,energy_end:energyEnd,newton_residual:r.res};
  }
  vectorOf(s){return [...s.q,...s.v,s.u,s.psi,s.pd,...s.b];}
  addStats(r){this.solverStats.calls+=r.calls;this.solverStats.jcalls+=r.jcalls;this.solverStats.iterations+=r.iterations;}
  snapshot(){return {s:copyState(this.s),H:this.H?.slice()??null,dt:this.cacheDt};}
  restore(snap){this.s=snap.s;this.H=snap.H;this.cacheDt=snap.dt;}
  adaptive(dt,cmd,depth,event,t=this.t){
    const saved=this.snapshot(),trial={t_start:t,dt,depth};event.attempts.push(trial);event.parent_calls++;event.depth=Math.max(event.depth,depth);
    try{if(this.cacheDt!==null&&this.cacheDt!==dt)this.H=null;this.cacheDt=dt;const r=this.singleStep(dt,cmd.law,cmd.stage_changed);event.trial_successful_substeps++;trial.outcome='success';return [{dt,r}];}
    catch(e){this.restore(saved);trial.outcome=e.newton?'newton_failure':'other_error';trial.error=e.message;if(!e.newton)throw e;event.newton_failures++;event.fallback=true;if(depth>=4){event.exhausted=true;throw e;}this.H=null;this.cacheDt=null;try{return [...this.adaptive(dt/2,cmd,depth+1,event,t),...this.adaptive(dt/2,cmd,depth+1,event,t+dt/2)];}catch(err){this.restore(saved);throw err;}}
  }
  step(cmd){
    const saved=this.snapshot(),event={macro_index:this.counters.macrosteps+1,t_start:this.t,dt:this.dt,parent_calls:0,newton_failures:0,trial_successful_substeps:0,committed_substeps:0,depth:0,fallback:false,exhausted:false,failed:false,attempts:[],accepted_substep_durations:[],controller_recommanded:false,accepted_physical_state_reset:false};
    try{
      const leaves=this.adaptive(this.dt,cmd,0,event);event.committed_substeps=leaves.length;event.accepted_substep_durations=leaves.map(v=>v.dt);event.outcome='committed';
      if(!event.fallback)return leaves[0].r;
      this.H=null;this.cacheDt=null;const out={};for(const key of ['motor_power','dissipation_power','u_mid','omega_mid','psi_dot_mid'])out[key]=leaves.reduce((s,v)=>s+v.dt*v.r[key],0)/this.dt;
      for(const key of ['tau_applied','tau_unsaturated'])out[key]=leaves.map(v=>v.r[key]).reduce((a,b)=>Math.abs(a)>=Math.abs(b)?a:b);
      out.energy_start=leaves[0].r.energy_start;out.energy_end=leaves.at(-1).r.energy_end;out.converged=1;out.newton_residual=Math.max(...leaves.map(v=>v.r.newton_residual));out.saturated=Number(leaves.some(v=>v.r.saturated));out.adaptive_substeps=leaves.length;out.adaptive_max_depth=event.depth;out.mid_velocities_are_duration_weighted_diagnostics=true;out.torque_diagnostics_are_signed_peaks=true;for(const key of ['tau_applied','tau_unsaturated'])out[key+'_abs_max']=Math.abs(out[key]);return out;
    }catch(e){event.failed=true;event.outcome='rolled_back';event.error=e.message;this.restore(saved);throw e;}
    finally{const c=this.counters;c.macrosteps++;for(const key of ['parent_calls','newton_failures','trial_successful_substeps','committed_substeps'])c[key]+=event[key];c.failed_macrosteps+=Number(event.failed);c.fallback_macrosteps+=Number(event.fallback);c.rescued_macrosteps+=Number(event.fallback&&!event.failed);c.exhausted_macrosteps+=Number(event.exhausted);this.adaptive_last_event=null;if(event.fallback||event.failed){this.adaptive_last_event=event;this.adaptive_events.push(event);if(this.adaptive_events.length>64)this.adaptive_events.shift();}}
  }
  advance(duration=.04){
    if(!this.ctrl)return [this.frame()];if(!Number.isFinite(duration)||duration<=0||duration>1)throw Error('Invalid step batch duration');
    const tic=performance.now(),frames=[],count=Math.round(duration/this.dt),stride=2;
    for(let j=0;j<count;j++){
      if(this.t-this.mission_start>1200)throw Error('Mission horizon reached; start a new session');
      const cmd=this.ctrl.command(this.mech,this.s,this.t,this.dt),d=this.step(cmd);this.k++;this.tau=d.tau_applied;this.work+=this.dt*d.motor_power;this.loss+=this.dt*d.dissipation_power;this.signed_travel+=this.dt*d.u_mid;
      this.window.push([this.t,d.u_mid]);while(this.window.length&&this.window[0][0]<this.t-this.reference.period)this.window.shift();
      [-1,1].forEach((side,i)=>this.rolling[i]+=this.dt*(d.u_mid-side*.265*d.omega_mid)/.0775);
      if(this.ctrl.stop_end!==null){const distance=Math.hypot(...this.s.b.map((v,i)=>v-this.goals.at(-1)[i]));this.passive_max=Math.max(this.passive_max,distance);const slow=distance<=.3&&low(this.s);this.low_since=slow?(this.low_since??this.t):null;this.stable=this.low_since===null?0:this.t-this.low_since;}
      if((j+1)%stride===0||j+1===count)frames.push(this.frame());
      if(this.k%this.sample_stride===0)this.samples.push([this.t,...this.vector(),this.tau]);
      if(this.samples.length>30000){this.samples=this.samples.filter((_,i)=>i%2===0);this.sample_stride*=2;}
    }
    this.compute+=(performance.now()-tic)/1000;return frames;
  }
  frame(){const s=this.s,c=this.ctrl;return {t:this.t,physical_t:this.t*this.data.scales.time_s,base:Array.from(s.b),line:this.mech.centerline(s.q,s.b),theta:s.q[0],rotor:s.q[0]+s.psi,wheels:this.rolling.slice(),u:s.u,mean_u:this.window.length?this.window.reduce((a,b)=>a+b[1],0)/this.window.length:0,tau:this.tau,stage:c?.stage??'ready',waypoint:c?.waypoint??0,distance:c?Math.hypot(...s.b.map((v,i)=>v-this.goals[c.waypoint][i])):null,amplitude:c?.amplitude??0,bearing:c?.bearing??0};}
  record(includeSamples=false){const s=this.s,energy=this.mech.energy(s.q,s.v,s.u,s.pd),r={sign:this.sign,t:this.t,dt:this.dt,state:this.vector(),controller:this.ctrl?.metadata()??null,controller_state:this.ctrl?{amplitude:this.ctrl.amplitude,bearing:this.ctrl.bearing,stage:this.ctrl.stage,waypoint:this.ctrl.waypoint,stop_start:this.ctrl.stop_start,stop_end:this.ctrl.stop_end,release_since:this.ctrl.release_since,tau:this.ctrl.tau,momentum:this.ctrl.momentum,goals:this.ctrl.goals}:null,events:this.events,controller_events:this.ctrl?.events??[],reference_sha256:this.reference.sha256,compute_seconds:this.compute,physical_time_per_compute_second:this.compute?this.t*this.data.scales.time_s/this.compute:null,signed_travel:this.signed_travel,stable_duration_model:this.stable,stop_end:this.ctrl?.stop_end??null,passive_max_distance_L:this.passive_max,energy,work:this.work,loss:this.loss,work_residual:(energy-this.work+this.loss)/Math.max(1,Math.abs(this.work),Math.abs(this.loss)),numerical_subdivision:this.counters,adaptive_events:this.adaptive_events,adaptive_last_event:this.adaptive_last_event,solver_stats:this.solverStats,scales:this.data.scales,numerical_method:'Float64 browser midpoint; dt=0.004 model units',claim:'Live finite-model experiment; no global or all-future guarantee'};if(includeSamples){r.samples=this.samples;r.sample_columns=['t','theta0','q1','q2','q3','q4','omega','qdot1','qdot2','qdot3','qdot4','u','psi','psi_dot','x','y','tau'];r.sample_stride_steps=this.sample_stride;}return r;}
}
