// Browser port of demo/compiled_midpoint.py. Float64 throughout; no fast-math.
export const dot=(a,b)=>{let s=0;for(let i=0;i<a.length;i++)s+=a[i]*b[i];return s;};
export const norm=a=>Math.sqrt(dot(a,a));
export const wrap=x=>((x+Math.PI)%(2*Math.PI)+2*Math.PI)%(2*Math.PI)-Math.PI;
export const clip=(x,a,b)=>Math.max(a,Math.min(b,x));
const flatten=a=>new Float64Array(a.flat(Infinity));
export function mv(A,x,n=x.length){const r=new Float64Array(A.length/n);for(let i=0;i<r.length;i++)for(let j=0;j<n;j++)r[i]+=A[i*n+j]*x[j];return r;}
function eye(n){const A=new Float64Array(n*n);for(let i=0;i<n;i++)A[i*n+i]=1;return A;}
function invert(A,n){const m=A.slice(),I=eye(n);for(let k=0;k<n;k++){let p=k;for(let i=k+1;i<n;i++)if(Math.abs(m[i*n+k])>Math.abs(m[p*n+k]))p=i;if(m[p*n+k]===0||!Number.isFinite(m[p*n+k]))throw Error('Singular Newton Jacobian');if(p!==k)for(let j=0;j<n;j++){[m[k*n+j],m[p*n+j]]=[m[p*n+j],m[k*n+j]];[I[k*n+j],I[p*n+j]]=[I[p*n+j],I[k*n+j]];}const d=m[k*n+k];for(let j=0;j<n;j++){m[k*n+j]/=d;I[k*n+j]/=d;}for(let i=0;i<n;i++)if(i!==k){const c=m[i*n+k];for(let j=0;j<n;j++){m[i*n+j]-=c*m[k*n+j];I[i*n+j]-=c*I[k*n+j];}}}return I;}
function choleskySolve(M,b,n){const L=new Float64Array(n*n),y=new Float64Array(n),x=new Float64Array(n);for(let i=0;i<n;i++)for(let j=0;j<=i;j++){let s=M[i*n+j];for(let k=0;k<j;k++)s-=L[i*n+k]*L[j*n+k];if(i===j){if(!(s>0))throw Error('Non-positive mechanical mass matrix');L[i*n+j]=Math.sqrt(s);}else L[i*n+j]=s/L[j*n+j];}for(let i=0;i<n;i++){let s=b[i];for(let j=0;j<i;j++)s-=L[i*n+j]*y[j];y[i]=s/L[i*n+i];}for(let i=n-1;i>=0;i--){let s=y[i];for(let j=i+1;j<n;j++)s-=L[j*n+i]*x[j];x[i]=s/L[i*n+i];}return x;}

export class Mechanics {
  constructor(d){
    this.d=d;this.ns=d.phi.length;this.nqp=d.phi[0].length;this.n=d.phi[0][0].length;this.m=this.n+2;
    for(const k of ['phi','weights','prefix','half','stiffness','Sq','SqInv','Sbig'])this[k]=flatten(d[k]);
    this.S=eye(this.m);this.inv=eye(this.m);
    for(let i=0;i<this.n;i++)for(let j=0;j<this.n;j++){this.S[i*this.m+j]=this.Sq[i*this.n+j];this.inv[i*this.m+j]=this.SqInv[i*this.n+j];}
    this.colnorm=new Float64Array(this.m);for(let j=0;j<this.m;j++){let s=0;for(let i=0;i<this.m;i++)s+=this.inv[i*this.m+j]**2;this.colnorm[j]=Math.sqrt(s);}
    // Reused, private scratch buffers: methods are synchronous and not reentrant.
    this.F=new Float64Array(this.nqp*2*this.n);this.G=new Float64Array(this.nqp*2);this.B=new Float64Array(2*(this.n+1));
    this.Jprev=new Float64Array(2*this.n);this.bprev=new Float64Array(2);
    this.visualPhi=flatten(d.visualization.phi);this.visualWeights=flatten(d.visualization.weights);this.visualNqp=d.visualization.phi[0].length;
  }
  blocks(q,v,u){
    const {n,nqp,ns,phi,weights,prefix,half,d,F,G,B,Jprev,bprev}=this,m=n+2,N=n+1;
    const M=new Float64Array(m*m),H=new Float64Array(N);Jprev.fill(0);bprev.fill(0);
    const ct=Math.cos(q[0]),st=Math.sin(q[0]);
    for(let e=0;e<ns;e++){
      for(let j=0;j<nqp;j++){const ix=(e*nqp+j)*n;let th=0,om=0;for(let k=0;k<n;k++){th+=phi[ix+k]*q[k];om+=phi[ix+k]*v[k];}const c=Math.cos(th),s=Math.sin(th);for(let k=0;k<n;k++){F[j*2*n+k]=-s*phi[ix+k];F[j*2*n+n+k]=c*phi[ix+k];}G[j*2]=-c*om*om;G[j*2+1]=-s*om*om;}
      for(let i=0;i<nqp;i++){
        B[0]=ct;B[N]=st;let bx=bprev[0]-u*st*v[0],by=bprev[1]+u*ct*v[0];
        for(let a=0;a<2;a++){
          for(let k=0;k<n;k++){let val=0;for(let j=0;j<nqp;j++)val+=prefix[i*nqp+j]*F[j*2*n+a*n+k];B[a*N+k+1]=Jprev[a*n+k]+half[e]*val;}
          let val=0;for(let j=0;j<nqp;j++)val+=prefix[i*nqp+j]*G[j*2+a];if(a===0)bx+=half[e]*val;else by+=half[e]*val;
        }
        const wt=weights[e*nqp+i];
        for(let a=0;a<N;a++){H[a]+=d.rhoA*wt*(B[a]*bx+B[N+a]*by);for(let b=a;b<N;b++)M[a*m+b]+=d.rhoA*wt*(B[a]*B[b]+B[N+a]*B[N+b]);}
        const ix=(e*nqp+i)*n;for(let a=0;a<n;a++)for(let b=a;b<n;b++)M[(a+1)*m+b+1]+=d.rhoI*wt*phi[ix+a]*phi[ix+b];
      }
      for(let j=0;j<nqp;j++)for(let a=0;a<2;a++){bprev[a]+=weights[e*nqp+j]*G[j*2+a];for(let k=0;k<n;k++)Jprev[a*n+k]+=weights[e*nqp+j]*F[j*2*n+a*n+k];}
    }
    for(let a=0;a<N;a++)for(let b=a+1;b<N;b++)M[b*m+a]=M[a*m+b];
    M[0]+=d.mass;M[m+1]+=d.mass*d.offset**2+d.jc+d.jr;
    M[m+m-1]=d.jr;M[(m-1)*m+1]=d.jr;M[m*m-1]=d.jr;
    H[0]-=d.mass*d.offset*v[0]**2;H[1]+=d.mass*d.offset*u*v[0];
    return {M,H};
  }
  energy(q,v,u,pd){
    const {n,nqp,ns,phi,weights,prefix,half,stiffness,d}=this;
    const F=new Float64Array(2*nqp),om=new Float64Array(nqp),uv=[u*Math.cos(q[0]),u*Math.sin(q[0])],vp=[0,0];let T=0;
    for(let e=0;e<ns;e++){
      for(let j=0;j<nqp;j++){let th=0;om[j]=0;for(let k=0;k<n;k++){const p=phi[(e*nqp+j)*n+k];th+=p*q[k];om[j]+=p*v[k];}F[2*j]=-Math.sin(th)*om[j];F[2*j+1]=Math.cos(th)*om[j];}
      for(let i=0;i<nqp;i++){let vx=uv[0]+vp[0],vy=uv[1]+vp[1];for(let a=0;a<2;a++){let val=0;for(let j=0;j<nqp;j++)val+=prefix[i*nqp+j]*F[2*j+a];if(a===0)vx+=half[e]*val;else vy+=half[e]*val;}T+=.5*weights[e*nqp+i]*(d.rhoA*(vx*vx+vy*vy)+d.rhoI*om[i]**2);}
      for(let j=0;j<nqp;j++){vp[0]+=weights[e*nqp+j]*F[2*j];vp[1]+=weights[e*nqp+j]*F[2*j+1];}
    }
    T+=.5*d.mass*(u*u+d.offset**2*v[0]**2)+.5*d.jc*v[0]**2+.5*d.jr*(v[0]+pd)**2;
    for(let i=1;i<n;i++)for(let j=1;j<n;j++)T+=.5*q[i]*stiffness[(i-1)*(n-1)+j-1]*q[j];return T;
  }
  torque(q,v,u,pd,law,M=null){
    let raw=0;const {m,n}=this;
    if(law[0]===1)raw=law[7]+law[2]*pd+law[3]*wrap(q[0]-law[5])+law[4]*(v[0]-law[6]);
    else if(law[0]===2){if(!M)M=this.blocks(q,v,u).M;let momentum=M[m]*u+M[2*m-1]*pd;for(let k=0;k<n;k++)momentum+=M[m+k+1]*v[k];const rate=clip(law[8]*momentum,-law[9],law[9]);raw=law[2]*pd+law[4]*(v[0]-rate);}
    else if(law[0]===3)raw=law[7];
    return {tau:law[0]===1||law[0]===2?clip(raw,-law[1],law[1]):raw,raw};
  }
  acceleration(q,v,u,pd,law){
    const {M,H}=this.blocks(q,v,u),{d,n,m,stiffness}=this,{tau}=this.torque(q,v,u,pd,law,M),rhs=new Float64Array(m);
    rhs[0]=-d.cu*u-H[0];rhs[1]=-d.cw*v[0]-H[1];
    for(let i=1;i<n;i++){let f=0;for(let j=1;j<n;j++)f+=stiffness[(i-1)*(n-1)+j-1]*q[j];rhs[i+1]=-d.cq*v[i]-H[i+1]-f;}rhs[m-1]=tau-d.cp*pd;
    const sol=choleskySolve(M,rhs,m),out=new Float64Array(m);out.set(sol.subarray(1,n+1));out[n]=sol[0];out[m-1]=sol[m-1];return out;
  }
  residual(x,q,w0,dt,law){const w=mv(this.inv,x),qm=q.map((v,i)=>v+.5*dt*w[i]),a=this.acceleration(qm,w.subarray(0,this.n),w[this.n],w[this.m-1],law);return w.map((v,i)=>v-w0[i]-.5*dt*a[i]);}
  threshold(x,q,w0,psi,dt){const {n,m}=this,w=mv(this.inv,x),full=new Float64Array(2*n+1);for(let i=0;i<n;i++)full[i]=q[i]+dt*w[i];for(let i=0;i<n+1;i++)full[n+i]=2*w[i]-w0[i];const b=mv(this.Sbig,full,full.length);return 1e-11*(1+Math.sqrt(dot(b,b)+(psi+dt*w[m-1])**2+(2*w[m-1]-w0[m-1])**2));}
  solve(s,dt,law,cache,maxIter=24){
    const {n,m}=this,w0=new Float64Array(m);w0.set(s.v);w0[n]=s.u;w0[m-1]=s.pd;
    let x=mv(this.S,w0),R=this.residual(x,s.q,w0,dt,law),H=cache?cache.slice():null,reused=!!cache,iterations=0,calls=1,jcalls=0,ok=false;
    for(let it=1;it<=maxIter;it++){
      if(2*norm(R)<=this.threshold(x,s.q,w0,s.psi,dt)){ok=true;break;}iterations=it;
      if(!H||(it%4===1&&!(reused&&it===1))){const J=new Float64Array(m*m);for(let j=0;j<m;j++){const h=1e-7*(1+Math.abs(x[j]))/(1+this.colnorm[j]),xp=x.slice(),xm=x.slice();xp[j]+=h;xm[j]-=h;const p=this.residual(xp,s.q,w0,dt,law),minus=this.residual(xm,s.q,w0,dt,law);for(let i=0;i<m;i++)J[i*m+j]=(p[i]-minus[i])/(2*h);}for(let i=0;i<m;i++)J[i*m+i]+=1e-12;H=invert(J,m);calls+=2*m;jcalls++;}
      const dx=mv(H,R).map(v=>-v),rn=norm(R);let alpha=1,bestNorm=rn,bestX=x,accepted=false;
      for(let k=0;k<3;k++){const trial=x.map((v,i)=>v+alpha*dx[i]),next=this.residual(trial,s.q,w0,dt,law);calls++;const nr=norm(next);
        if(nr<=(1-1e-4*alpha)*rn){const step=trial.map((v,i)=>v-x[i]),y=next.map((v,i)=>v-R[i]),Hy=mv(H,y),denom=dot(step,Hy);if(Math.abs(denom)>1e-14*(norm(step)*norm(Hy)+1)){const row=new Float64Array(m);for(let j=0;j<m;j++)for(let i=0;i<m;i++)row[j]+=step[i]*H[i*m+j];for(let i=0;i<m;i++)for(let j=0;j<m;j++)H[i*m+j]+=(step[i]-Hy[i])*row[j]/denom;}x=trial;R=next;accepted=true;break;}
        if(nr<bestNorm){bestNorm=nr;bestX=trial;}alpha*=.5;
      }
      if(!accepted){x=bestX;R=this.residual(x,s.q,w0,dt,law);calls++;}reused=false;
    }
    const res=2*norm(R);ok=ok||res<=this.threshold(x,s.q,w0,s.psi,dt);const w=mv(this.inv,x),w1=w.map((v,i)=>2*v-w0[i]);
    return {q:s.q.map((v,i)=>v+dt*w[i]),v:w1.slice(0,n),u:w1[n],psi:s.psi+dt*w[m-1],pd:w1[m-1],H,ok,res,iterations,calls,jcalls};
  }
  centerline(q,b){const {n,ns,visualNqp:nqp,visualPhi:phi,visualWeights:weights}=this;const line=[[b[0],b[1]]];let x=b[0],y=b[1];for(let e=0;e<ns;e++){let dx=0,dy=0;for(let j=0;j<nqp;j++){let th=0;for(let k=0;k<n;k++)th+=phi[(e*nqp+j)*n+k]*q[k];dx+=weights[e*nqp+j]*Math.cos(th);dy+=weights[e*nqp+j]*Math.sin(th);}x+=dx;y+=dy;line.push([x,y]);}return line;}
}
