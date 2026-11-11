import {Session} from './session.js';
let session=null;
self.onmessage=({data:m})=>{
  try{
    if(m.kind==='init')session=new Session(m.data,m.sign);
    if(!session)throw Error('Simulation not initialized');
    if(m.kind==='target')session.target(m.goals);
    if(m.kind==='reset')session.reset();
    const frames=m.kind==='step'?session.advance(m.duration??.04):[session.frame()];
    self.postMessage({id:m.id,frames,record:session.record(m.kind==='export'),error:null});
  }catch(e){self.postMessage({id:m.id,frames:session?[session.frame()]:[],record:session?session.record():null,error:e.message});}
};
