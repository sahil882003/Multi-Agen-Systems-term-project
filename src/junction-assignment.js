import {NeighborTransport,reportPlanningProgress} from './robot-agent.js';
import {PeerBus} from './peer-protocol.js';

function batterySlotCosts(request,n){
  const reserve=request.minimumBattery??.12,warning=.30;
  if(!Number.isFinite(request.battery)||request.battery<0||request.battery>1)
    throw new Error('Slot assignment requires a measured battery in [0,1]');
  const urgency=Math.max(0,Math.min(1,(warning-request.battery)/(warning-reserve)));
  return Array.from({length:n},(_,s)=>{
    const late=request.deliveryActive===false?0:Math.max(0,s-request.slack);
    return 8*urgency*s*s+20*Number(late>0)+late*late+.1*(1+.05*Math.min(100,request.age??0))*s;
  });
}

function assignmentSimplex(values,allowed){
  const indices=allowed.flatMap((ok,k)=>ok?[k]:[]);if(!indices.length)return null;
  const sorted=indices.map(k=>values[k]).sort((a,b)=>b-a);let sum=0,theta=0;
  for(let j=0;j<sorted.length;j++){
    sum+=sorted[j];const candidate=(sum-1)/(j+1);
    if(j===sorted.length-1||sorted[j+1]<=candidate){theta=candidate;break;}
  }
  return values.map((v,k)=>allowed[k]?Math.max(0,v-theta):0);
}

class BatterySlotAgent{
  #slot=null;#finished=false;#claimEndpoint;#threshold=0;#claims;
  constructor(request,roster,transport,rho){
    this.id=request.id;this.request=structuredClone(request);this.roster=[...roster].sort((a,b)=>a-b);
    this.endpoint=transport.endpoint(this.id);this.rho=rho;this.n=roster.length;
    this.costs=batterySlotCosts(request,this.n);this.z=Array(this.n).fill(0);this.u=Array(this.n).fill(0);
    this.x=Array(this.n).fill(1/this.n);this.inbox=new Map();
  }
  discover(){
    const received=this.endpoint.receive();
    this.requests=[this.request,...received.filter(m=>m.type==='assignment-request').map(m=>m.payload)];
    const clearance=this.requests.filter(p=>p.clearance).length;
    const feasible=this.request.battery-(this.request.remainingEnergy??0)>=(this.request.minimumBattery??.12)-1e-9;
    // Cycle-clearance restrictions limit the slots each robot may claim.

    this.allowed=Array.from({length:this.n},(_,s)=>feasible&&(this.request.clearance?s<clearance:s>=clearance));
    const rows=this.requests.map(p=>batterySlotCosts(p,this.n));
    const floor=Array.from({length:this.n},(_,s)=>Math.min(...rows.map(c=>c[s])));

    // Common column offsets preserve the square-assignment objective.

    const ranges=rows.map(c=>Math.max(...c.map((v,s)=>v-floor[s])));
    // Scale costs relative to rho without changing their minimizer.

    this.scale=Math.max(1,...ranges.map(v=>v/20));
    this.scaled=this.costs.map((c,s)=>(c-floor[s])/this.scale);
    return this.allowed.some(Boolean);
  }
  update(){this.x=assignmentSimplex(this.z.map((z,s)=>z-this.u[s]-this.scaled[s]/this.rho),this.allowed);}
  propose(k){for(const id of this.roster)if(id!==this.id)this.endpoint.send(id,'assignment-vector',
    {shifted:this.x.map((x,s)=>x+this.u[s]),x:[...this.x]},k);}
  project(){
    const messages=this.endpoint.receive().filter(m=>m.type==='assignment-vector');
    const own=this.x.map((x,s)=>x+this.u[s]),old=[...this.z];
    this.violation=0;
    this.z=own.map((v,s)=>{
      const total=v+messages.reduce((sum,m)=>sum+m.payload.shifted[s],0);
      this.violation=Math.max(this.violation,this.x[s]+messages.reduce((sum,m)=>sum+m.payload.x[s],0)-1);
      return v-Math.max(0,(total-1)/this.n);
    });
    this.primal=Math.hypot(...this.x.map((x,s)=>x-this.z[s]));
    this.dual=this.rho*Math.hypot(...this.z.map((z,s)=>z-old[s]));
    this.u=this.u.map((u,s)=>u+this.x[s]-this.z[s]);
    this.ready=this.primal<1e-6&&this.dual<1e-6&&this.violation<1e-6;
  }
  announce(k){for(const id of this.roster)if(id!==this.id)this.endpoint.send(id,'assignment-ready',
    {ready:this.ready,epoch:this.request.epoch},k);}
  agreed(){const messages=this.endpoint.receive();return this.ready&&messages.length===this.n-1&&
    messages.every(m=>m.type==='assignment-ready'&&m.payload.ready&&m.payload.epoch===this.request.epoch);}
  bindClaims(bus){
    this.#claimEndpoint=bus.register(this.id,(type,payload)=>{
      if(type==='slot-state')return {slot:this.#slot,finished:this.#finished};
      if(type==='relocate-slot')return {ok:this.#relocate(payload.visited)};
      if(type==='assignment-verify')return {id:this.id,slot:this.#slot,epoch:this.request.epoch,
        allowed:this.#slot!==null&&this.allowed[this.#slot],finished:this.#finished};
      throw new Error('Unknown peer slot claim message');
    });
  }
  beginClaims(threshold){this.#slot=null;this.#finished=false;this.#threshold=threshold;}
  #relocate(visited){
    if(visited.includes(this.id))return false;
    const path=[...visited,this.id];
    const candidates=this.allowed.flatMap((ok,s)=>ok&&(this.#threshold<0||this.x[s]>this.#threshold)?[s]:[])
      .sort((a,b)=>this.x[b]-this.x[a]||this.costs[a]-this.costs[b]||a-b);
    for(const s of candidates){
      if(s===this.#slot)continue;
      let owner=null;
      for(const id of this.roster)if(id!==this.id&&this.#claimEndpoint.request(id,'slot-state').slot===s){owner=id;break;}
      if(owner===null||!path.includes(owner)&&this.#claimEndpoint.request(owner,'relocate-slot',{visited:path}).ok){
        this.#slot=s;return true;
      }
    }
    return false;
  }
  claimTurn(){
    if(this.#finished)return true;

    if(this.roster.some(id=>id<this.id&&!this.#claimEndpoint.request(id,'slot-state').finished))return false;
    const ok=this.#relocate([]);this.#finished=true;return ok;
  }
  verify(){
    this.#claims=this.roster.map(id=>id===this.id?{id,slot:this.#slot,epoch:this.request.epoch,
      allowed:this.#slot!==null&&this.allowed[this.#slot],finished:this.#finished}:this.#claimEndpoint.request(id,'assignment-verify'));
    return this.#claims.every(p=>p.allowed&&p.finished&&p.epoch===this.request.epoch)&&new Set(this.#claims.map(p=>p.slot)).size===this.n;
  }
  decision(){return {id:this.id,slot:this.#slot,priority:this.#slot,from:this.request.from,to:this.request.to,
    epoch:this.request.epoch,roster:[...this.roster],battery:this.request.battery,
    batteryReported:true,assignment:true,cost:this.costs[this.#slot],fractional:[...this.x]};}
}

export function negotiateBatterySlots(groups,settings,tick){
  const started=performance.now(),transport=new NeighborTransport(),agents=[];
  for(const group of groups){
    const roster=group.requests.map(r=>r.id);
    if(new Set(roster).size!==roster.length)throw new Error('Duplicate assignment endpoint');
    for(const request of group.requests)transport.register(request.id);
    for(const a of group.requests)for(const b of group.requests)if(a.id!==b.id){
      if(Math.abs(a.from.x-b.from.x)+Math.abs(a.from.y-b.from.y)>2)throw new Error('Nonlocal slot assignment');
      transport.send(a.id,b.id,'assignment-request',a);
    }
    agents.push(...group.requests.map(r=>new BatterySlotAgent(r,roster,transport,settings.admmRho)));
  }
  const valid=agents.map(a=>a.discover()).every(Boolean);let status=valid?'converged':'energy-infeasible',iterations=0;
  const history=[];let lastProgress=-Infinity;
  for(let k=1;valid&&agents.length;k++){
    if(!settings.admmUntilConverged&&(k>settings.admmMaxRounds||performance.now()-started>settings.admmBudgetMs)){status='timeout';break;}

    const working=agents.filter(a=>!a.solved);
    working.forEach(a=>a.update());working.forEach(a=>a.propose(k));working.forEach(a=>a.project());
    working.forEach(a=>a.announce(k));iterations=k;
    const agreement=new Map(working.map(a=>[a.id,a.agreed()]));
    for(const a of working)if(a.roster.every(id=>agreement.get(id)))a.solved=true;
    const row={iteration:k,primal:Math.hypot(...agents.map(a=>a.primal)),dual:Math.hypot(...agents.map(a=>a.dual)),
      objective:agents.reduce((v,a)=>v+a.x.reduce((sum,x,s)=>sum+x*a.costs[s],0),0)};
    history.push(row);if(history.length>2000)history.shift();
    if(performance.now()-lastProgress>250){reportPlanningProgress({phase:'assignment',batchTick:tick,iteration:k,
      elapsedMs:performance.now()-started,totalAgents:agents.length,activeAgents:agents.length,readyAgents:agents.filter(a=>a.ready).length,
      maxPrimal:Math.max(...agents.map(a=>a.primal)),maxDual:Math.max(...agents.map(a=>a.dual)),
      violation:Math.max(...agents.map(a=>a.violation)),untilConverged:settings.admmUntilConverged});lastProgress=performance.now();}
    if(agents.every(a=>a.solved))break;
  }
  const bus=new PeerBus();agents.forEach(a=>a.bindClaims(bus));let extracted=false,expanded=false;
  if(status==='converged')for(const threshold of [1e-5,1e-9,-1]){
    agents.forEach(a=>a.beginClaims(threshold));

    for(let round=0;round<=agents.length;round++)if(agents.map(a=>a.claimTurn()).every(Boolean))break;
    if(agents.map(a=>a.verify()).every(Boolean)){extracted=true;expanded=threshold<0;break;}
  }
  if(status==='converged'&&!extracted)status='discrete-unresolved';
  const decisions=status==='converged'?agents.map(a=>a.decision()):[];
  const discreteObjective=decisions.reduce((s,d)=>s+d.cost,0),relaxedObjective=history.at(-1)?.objective??null;
  const solver={status,architecture:'peer-battery-slot-assignment-admm',iterations,history,
    assignmentRows:decisions.map(d=>({id:d.id,slot:d.slot,battery:d.battery,to:d.to,cost:d.cost})),
    objective:relaxedObjective,relaxedObjective,discreteObjective,
    extractionGap:relaxedObjective===null?null:discreteObjective-relaxedObjective,extractionExpanded:expanded,
    messages:transport.count+bus.wire.count,messageBytes:transport.bytes+bus.wire.bytes,
    groupCount:groups.length,untilConverged:settings.admmUntilConverged,
    maxPrimal:Math.max(0,...agents.map(a=>a.primal??0)),maxDual:Math.max(0,...agents.map(a=>a.dual??0)),
    violation:Math.max(0,...agents.map(a=>a.violation??0)),oracleUsed:false,
    ownership:'own measured battery, own assignment vector and duals; peer capacity projection, augmenting claims and bilateral verification',
    reason:'Only slot zero may depart; later assignments are tentative. Waiting consumes no energy; low battery is a preference.'};
  reportPlanningProgress({phase:'finished',batchTick:tick,iteration:iterations,elapsedMs:performance.now()-started,
    status,totalAgents:agents.length,activeAgents:agents.length,readyAgents:status==='converged'?agents.length:0,
    maxPrimal:solver.maxPrimal,maxDual:solver.maxDual,violation:solver.violation,untilConverged:settings.admmUntilConverged});
  return {decisions,solver};
}
