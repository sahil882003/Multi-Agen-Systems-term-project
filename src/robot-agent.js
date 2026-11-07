import { isotonic } from './passage.js';
import { same, movementCount, spatialMission } from './world.js';

let planningProgressObserver=null;
export function setPlanningProgressObserver(observer){planningProgressObserver=observer;}
export function reportPlanningProgress(progress){planningProgressObserver?.(progress);}

export class NeighborTransport {
  constructor() { this.inboxes = new Map(); this.records = []; this.cursor = 0; this.count = 0; this.bytes = 0; }
  get log() { return this.records.length<1000 ? [...this.records] : [...this.records.slice(this.cursor),...this.records.slice(0,this.cursor)]; }
  endpoint(id) {return Object.freeze({send:(to,type,payload,iteration)=>this.send(id,to,type,payload,iteration),receive:()=>this.receive(id)});}
  register(id) { this.inboxes.set(id, []); }
  send(from, to, type, payload, iteration = 0) {

    let copied;
    if(type==='timing')copied={shifted:payload.shifted.slice(),values:payload.values.slice(),localValue:payload.localValue};
    else if(type==='integer')copied={values:payload.values.slice(),localValue:payload.localValue};
    else if(type==='ready')copied={...payload};
    else copied=structuredClone(payload);
    const message = {from,to,type,payload:copied,iteration};
    this.count++;
    // Defined logical wire format: JSON envelope followed by packed float buffers.
    // This is payload accounting, not measured network traffic or latency.
    const envelope={...message,payload:{...message.payload}};let binary=0;
    for(const [name,value] of Object.entries(envelope.payload))if(ArrayBuffer.isView(value)) {
      binary+=value.byteLength;envelope.payload[name]={encoding:'float64',length:value.length};
    }
    this.bytes+=JSON.stringify(envelope).length+binary;
    this.inboxes.get(to).push(message);
    const record={...message,from:`R${from+1}`,to:`R${to+1}`};
    if(this.records.length<1000)this.records.push(record);
    else {this.records[this.cursor]=record;this.cursor=(this.cursor+1)%1000;}
  }
  receive(id) { const messages = this.inboxes.get(id); this.inboxes.set(id, []); return messages; }
}

function visits(plan) {
  const result = [];
  for (let k = 0; k < plan.points.length - 1;) {
    let end = k;
    while (end + 1 < plan.points.length && same(plan.points[k], plan.points[end + 1])) end++;
    if (end + 1 < plan.points.length) result.push({entry:k, exit:end+1, cell:`${plan.points[k].x},${plan.points[k].y}`});
    k = end + 1;
  }
  return result;
}

function routeWithoutWaits(plan) {
  const points = [], arrivals = []; let pickupIndex, deliveryIndex;
  plan.points.forEach((p,k) => {
    if (!points.length || !same(points.at(-1), p) || k === plan.pickupIndex || k === plan.deliveryIndex) {
      points.push({...p}); arrivals.push(k);
      if (k === plan.pickupIndex) pickupIndex = points.length-1;
      if (k === plan.deliveryIndex) deliveryIndex = points.length-1;
    }
  });
  return {...plan, points, pickupIndex, deliveryIndex, seedArrivals:arrivals};
}

class WarehouseAgent {
  #self; #order; #world; #settings; #tick; #inbox = new Map(); #transport;
  constructor({self,order,world,settings,tick,transport}) {
    this.#self=structuredClone(self); this.id=self.id; this.#order=structuredClone(order);
    this.#world=world; this.#settings={...settings}; this.#tick=tick; this.#transport=transport.endpoint(self.id);
    this.ports=[]; this.x=[]; this.phase='new'; this.error=null; this.epoch=tick;
    this.rho=settings.admmRho ?? 1; this.primal=Infinity; this.dual=Infinity; this.violation=Infinity;
  }
  ingest() {
    for(const m of this.#transport.receive()) this.#inbox.set(`${m.type}/${m.from}`,m);
  }
  // A travelling robot publishes its remaining commitment. Keeping every wait
  // as a route event and fixing all delays to zero prevents retiming that robot.
  commit(plan) {
    this.fixed=true;this.seed=structuredClone(plan);
    this.raw={...this.seed,seedArrivals:plan.points.map((_,k)=>k)};
    this.x=plan.points.map(()=>0);this.upper=plan.points.map(()=>0);this.phase='planned';
  }
  plan(routePlanner) {
    this.ingest();
    const statuses=[...this.#inbox.values()].filter(m=>m.type==='status').map(m=>m.payload);
    const proposals=[...this.#inbox.values()].filter(m=>m.type==='route').map(m=>m.payload.plan);
    const result=routePlanner(this.#world,this.#self,this.#order,proposals,statuses,this.#tick,this.#settings,true);
    this.explored=result.explored??0;
    if(!result.ok) {
      const committed=[...this.#inbox.values()].some(m=>m.type==='route'&&m.payload.fixed);
      const lower=committed?spatialMission(this.#world,this.#self,this.#order):null;
      this.phase=lower&&this.#tick+lower.deliveryIndex<=this.#order.deadline?'deferred':'rejected';
      this.error=this.phase==='deferred'?'No insertion found around committed traffic; retry after movement.':result.reason;return;
    }
    this.seed=result.plan;
    if(movementCount(this.seed.points)*this.#settings.drain>this.#self.battery-this.#settings.batteryMin+1e-9) {
      this.phase='rejected';this.error='Local route exceeds available energy.';return;
    }
    this.raw=routeWithoutWaits(this.seed);
    this.upper=this.raw.points.map((_,k)=>k===0?0:Math.min(this.#settings.horizon-this.#tick-this.raw.points.length+1,
      k<=this.raw.deliveryIndex?this.#order.deadline-this.#tick-this.raw.deliveryIndex:Infinity));
    this.x=this.raw.seedArrivals.map((t,k)=>t-k);
    this.phase='planned';
  }
  tryPlan(routePlanner,roster){
    if(this.fixed||this.phase!=='new')return true;
    this.ingest();
    if(roster.some(id=>id<this.id&&!this.#inbox.has(`route-done/${id}`)))return false;
    this.plan(routePlanner);
    for(const peer of roster)if(peer!==this.id){
      if(this.phase==='planned')this.#transport.send(peer,'route',this.routeMessage());
      this.#transport.send(peer,'route-done',{epoch:this.epoch});
    }
    return true;
  }
  routeMessage() { return {id:this.id,raw:this.raw,plan:this.seed,fixed:!!this.fixed}; }
  buildPorts() {
    this.ingest(); this.ports=[];
    const all=[this.routeMessage(), ...[...this.#inbox.values()].filter(m=>m.type==='route'&&m.from!==this.id).map(m=>m.payload)];
    const fixedIds=new Set(all.filter(r=>r.fixed).map(r=>r.id));
    const resources=new Map();
    for(const r of all) {
      for(const v of visits(r.raw)) {
        const resource=`cell:${v.cell}`;
        if(!resources.has(resource))resources.set(resource,[]);
        resources.get(resource).push({id:r.id,entry:v.entry,release:v.exit,time:r.raw.seedArrivals[v.entry],kind:'cell',resource:v.cell});
      }
      for(let k=1;k<r.raw.points.length;k++) {
        const u=r.raw.points[k-1],v=r.raw.points[k];if(same(u,v))continue;
        const resource=[`${u.x},${u.y}`,`${v.x},${v.y}`].sort().join('/');
        const key=`edge:${resource}`;
        if(!resources.has(key))resources.set(key,[]);
        resources.get(key).push({id:r.id,entry:k,release:k,time:r.raw.seedArrivals[k],kind:'edge',resource:key});
      }
    }
    for(const list of resources.values()) {
      if(!list.some(v=>v.id===this.id))continue;
      list.sort((a,b)=>a.time-b.time || a.id-b.id);
      for(let k=1;k<list.length;k++) {
        const u=list[k-1],v=list[k];
        if(u.id===v.id || (u.id!==this.id && v.id!==this.id))continue;
        if(fixedIds.has(u.id) && fixedIds.has(v.id))continue;
        const e={from:u.id,to:v.id,a:u.release,b:v.entry,gap:u.release-v.entry+(u.kind==='edge'?1:0),resource:u.resource,kind:u.kind};
        e.key=`${e.from}/${e.to}/${e.a}/${e.b}/${e.kind}`;
        const left=e.from===this.id,at=left?e.a:e.b;
        this.ports.push({edge:e,peer:left?e.to:e.from,left,at,z:this.x[at],u:0});
      }
    }
  }

  // Substitute the announced fixed endpoint (delay=0) into each shared
  // inequality. These become local box bounds, not consensus variables.
  // The travelling robot's trajectory is neither solved nor changed.
  eliminateCommitted(fixedIds) {
    this.boundaryEdges=this.ports.filter(p=>fixedIds.has(p.peer)).map(p=>p.edge);
    this.lower=this.x.map(()=>0);
    for(const p of this.ports)if(fixedIds.has(p.peer)){
      if(p.left)this.upper[p.at]=Math.min(this.upper[p.at],-p.edge.gap);
      else this.lower[p.at]=Math.max(this.lower[p.at],p.edge.gap);
    }
    this.ports=this.ports.filter(p=>!fixedIds.has(p.peer));
    for(let k=1;k<this.lower.length;k++)this.lower[k]=Math.max(this.lower[k],this.lower[k-1]);
    for(let k=this.upper.length-2;k>=0;k--)this.upper[k]=Math.min(this.upper[k],this.upper[k+1]);
    if(this.lower.some((v,k)=>v>this.upper[k]+1e-9))throw new Error('Committed trajectory bounds are inconsistent with the negotiated seed.');
  }

  prepareGroups() {
    this.peerGroups=new Map();
    for(const p of this.ports){if(!this.peerGroups.has(p.peer))this.peerGroups.set(p.peer,[]);this.peerGroups.get(p.peer).push(p);}
    for(const [peer,ports] of this.peerGroups){
      ports.sort((a,b)=>a.edge.key.localeCompare(b.edge.key));ports.forEach((p,k)=>p.peerIndex=k);
      this.#transport.send(peer,'constraints',{keys:ports.map(p=>p.edge.key),localValue:this.x[this.raw.deliveryIndex]});
    }
  }
  verifyGroups() {
    this.ingest();
    for(const [peer,ports] of this.peerGroups){const keys=this.#inbox.get(`constraints/${peer}`)?.payload.keys;
      if(!keys || keys.length!==ports.length || ports.some((p,k)=>p.edge.key!==keys[k]))throw new Error('Neighbor constraint agreement mismatch');}
  }
  update() {
    const weights=this.x.map(()=>1),sums=this.x.map(()=>0);
    for(const p of this.ports){weights[p.at]+=this.rho;sums[p.at]+=this.rho*(p.z-p.u);}
    this.x=isotonic(sums.map((v,k)=>v/weights[k]),weights,this.upper,this.lower);
  }
  propose(round) {
    for(const [peer,ports] of this.peerGroups) {
      const shifted=Float64Array.from(ports,p=>this.x[p.at]+p.u),values=Float64Array.from(ports,p=>this.x[p.at]);
      this.#transport.send(peer,'timing',{shifted,values,localValue:this.x[this.raw.deliveryIndex]},round);
    }
  }
  project() {
    this.ingest();let primal=0,dual=0,violation=0;const changes=this.x.map(()=>0);

    for(const [peer,ports] of this.peerGroups){
      const payload=this.#inbox.get(`timing/${peer}`)?.payload;
      if(!payload)throw new Error('Missing neighbor timing message');
      for(const p of ports)p.received=payload;
    }
    for(const p of this.ports) {
      const otherValue=p.received.shifted[p.peerIndex],otherX=p.received.values[p.peerIndex];
      const own=this.x[p.at]+p.u,left=p.left?own:otherValue,right=p.left?otherValue:own;
      const shift=Math.max(0,(p.edge.gap-right+left)/2),old=p.z;
      p.z=p.left?left-shift:right+shift;p.u+=this.x[p.at]-p.z;
      primal+=(this.x[p.at]-p.z)**2;changes[p.at]+=p.z-old;
      violation=Math.max(violation,p.edge.gap-(p.left?otherX-this.x[p.at]:this.x[p.at]-otherX));
    }
    dual=changes.reduce((s,v)=>s+v*v,0);
    this.primal=Math.sqrt(primal);this.dual=this.rho*Math.sqrt(dual);this.violation=violation;
    this.ready=this.primal<1e-3 && this.dual<1e-3 && violation<1e-3;
  }
  announceReady(roster,round) {
    if(this.lastAnnouncedReady === this.ready) return;
    this.lastAnnouncedReady = this.ready;
    for(const id of roster) if(id!==this.id) this.#transport.send(id,'ready',{ready:this.ready,epoch:this.epoch,localValue:this.x[this.raw.deliveryIndex]},round);
  }
  hasReadyPeers(roster) {
    this.ingest();return this.ready && roster.every(id=>id===this.id || this.#inbox.get(`ready/${id}`)?.payload.ready===true && this.#inbox.get(`ready/${id}`)?.payload.epoch===this.epoch);
  }
  roundIntegers() {this.integer=this.x.map(v=>Math.max(0,Math.abs(v-Math.round(v))<1e-3?Math.round(v):Math.ceil(v)));}
  exchangeIntegers(round) {
    const peers=new Set(this.ports.map(p=>p.peer));
    for(const peer of peers)this.#transport.send(peer,'integer',{values:this.integer,localValue:this.integer[this.raw.deliveryIndex]},round);
  }
  repairIntegers() {
    this.ingest();const old=[...this.integer];
    for(let k=0;k<this.integer.length;k++) {
      this.integer[k]=Math.max(this.integer[k],this.lower?.[k]??0);
      if(k)this.integer[k]=Math.max(this.integer[k],this.integer[k-1]);
      for(const p of this.ports) if(!p.left && p.at===k) {
        const values=this.#inbox.get(`integer/${p.peer}`).payload.values;
        this.integer[k]=Math.max(this.integer[k],values[p.edge.a]+p.edge.gap);
      }
    }
    this.ready=this.integer.every((v,k)=>v<=this.upper[k]+1e-9 && v===old[k]);
    return this.integer.every((v,k)=>v<=this.upper[k]+1e-9);
  }
  executionPlan() {
    const arrivals=this.integer.map((d,k)=>d+k),points=[{...this.raw.points[0]}];
    for(let k=1;k<arrivals.length;k++){while(points.length<arrivals[k])points.push({...this.raw.points[k-1]});points.push({...this.raw.points[k]});}
    return {...this.raw,points,pickupIndex:arrivals[this.raw.pickupIndex],deliveryIndex:arrivals[this.raw.deliveryIndex],arrivals,
      waypointWaits:this.integer.map((d,k)=>k?d-this.integer[k-1]:0)};
  }
  diagnostic() {return {id:this.id,raw:this.raw,upper:this.upper,lower:this.lower,x:[...this.x],edges:[...this.ports.map(p=>p.edge),...(this.boundaryEdges??[])],primal:this.primal,dual:this.dual};}
}

export function planAgentBatch(world,robots,jobs,tick,settings,routePlanner,{coordinate=true,maxRounds=3000,commitments=[],peerRouting=false}={}) {
  const started=performance.now();let lastProgress=-Infinity,lastPhase='';
  const progress=(phase,values={},force=false)=>{
    if(!planningProgressObserver)return;
    const now=performance.now();if(!force&&phase===lastPhase&&now-lastProgress<250)return;
    lastProgress=now;lastPhase=phase;
    planningProgressObserver({phase,batchTick:tick,elapsedMs:now-started,totalAgents:jobs.length,committedAgents:commitments.length,untilConverged:!!settings.admmUntilConverged,...values});
  };
  progress('routes',{plannedAgents:0},true);
  const transport=new NeighborTransport();const agents=jobs.map(({robot,order})=>new WarehouseAgent({self:robot,order,world,
    settings:peerRouting?{...settings,peerSearchIndex:true}:settings,tick,transport}));
  const fixed=commitments.map(plan=>{
    const a=new WarehouseAgent({self:robots.find(r=>r.id===plan.robotId),order:null,world,settings,tick,transport});
    a.commit(plan);return a;
  });
  agents.push(...fixed);
  agents.forEach(a=>transport.register(a.id));

  for(const r of robots)for(const a of agents)if(!a.fixed)transport.send(r.id,a.id,'status',{id:r.id,pos:r.pos,home:r.home,battery:r.battery});
  for(const a of fixed)for(const b of agents)if(!b.fixed)transport.send(a.id,b.id,'route',a.routeMessage());
  const routingRoster=agents.filter(a=>!a.fixed).map(a=>a.id);
  if(peerRouting)for(let round=0;round<=routingRoster.length;round++){
      const done=agents.filter(a=>!a.fixed).map(a=>a.tryPlan(routePlanner,routingRoster)).every(Boolean);
      progress('routes',{plannedAgents:agents.filter(r=>!r.fixed&&r.phase!=='new').length});
      if(done)break;
    }
  else for(const a of agents.filter(a=>!a.fixed).sort((a,b)=>a.id-b.id)){
    a.plan(routePlanner);
    progress('routes',{plannedAgents:agents.filter(r=>!r.fixed&&r.phase!=='new').length});
    if(a.phase==='planned')for(const b of agents)if(!b.fixed&&b.id!==a.id)transport.send(a.id,b.id,'route',a.routeMessage());
  }
  const active=agents.filter(a=>a.phase==='planned'&&!a.fixed),ids=active.map(a=>a.id),rejected=agents.filter(a=>a.phase==='rejected').map(a=>({orderId:jobs.find(j=>j.robot.id===a.id).order.id,reason:a.error}));
  const history=[];let status=active.length?'converged':'infeasible',iterations=0;
  if(coordinate && active.length) {
    progress('constraints',{activeAgents:active.length},true);
    active.forEach(a=>a.buildPorts());
    if(fixed.length){const fixedIds=new Set(fixed.map(a=>a.id));active.forEach(a=>a.eliminateCommitted(fixedIds));}
    active.forEach(a=>a.prepareGroups());active.forEach(a=>a.verifyGroups());status='timeout';
    const solveStart=performance.now();
    const untilConverged=!!settings.admmUntilConverged;
    progress('admm',{iteration:0,activeAgents:active.length},true);
    for(let k=1;k<=(untilConverged?Infinity:maxRounds);k++) {
      if(!untilConverged&&performance.now()-solveStart>(settings.admmBudgetMs??2000))break;
      active.forEach(a=>a.update());active.forEach(a=>a.propose(k));active.forEach(a=>a.project());
      active.forEach(a=>a.announceReady(ids,k));iterations=k;
      history.push({iteration:k,primal:Math.hypot(...active.map(a=>a.primal)),dual:Math.hypot(...active.map(a=>a.dual)),
        objective:active.reduce((s,a)=>s+a.x.reduce((v,x)=>v+.5*x*x,0),0),proposals:active.map(a=>a.x[a.raw.deliveryIndex])});
      if(untilConverged&&history.length>2000)history.shift();
      progress('admm',{...history.at(-1),activeAgents:active.length,readyAgents:active.filter(a=>a.ready).length,
        maxPrimal:Math.max(...active.map(a=>a.primal)),maxDual:Math.max(...active.map(a=>a.dual)),violation:Math.max(...active.map(a=>a.violation))});

      if(active.map(a=>a.hasReadyPeers(ids)).every(Boolean)){status='converged';break;}
    }
    if(status==='converged') {
      progress('integer-repair',{iteration:iterations,activeAgents:active.length},true);
      active.forEach(a=>a.roundIntegers());let repaired=false;
      for(let k=0;k<(untilConverged?Infinity:1000);k++) {
        active.forEach(a=>a.exchangeIntegers(iterations+k+1));
        if(!active.map(a=>a.repairIntegers()).every(Boolean))break;
        active.forEach(a=>a.announceReady(ids,iterations+k+1));
        progress('integer-repair',{iteration:iterations,repairRound:k+1,activeAgents:active.length,readyAgents:active.filter(a=>a.ready).length});
        if(active.map(a=>a.hasReadyPeers(ids)).every(Boolean)){repaired=true;break;}
      }
      if(!repaired)status='repair-failed';
    }
  }
  const plans=status==='converged'?active.filter(a=>!a.fixed).map(a=>coordinate?a.executionPlan():a.seed):[];

  const deferred=agents.filter(a=>a.phase==='deferred').map(a=>({orderId:jobs.find(j=>j.robot.id===a.id).order.id,reason:a.error}));
  if(status==='timeout'||status==='repair-failed')deferred.push(...active.filter(a=>!a.fixed).map(a=>({orderId:a.raw.orderId,reason:`Local coordination ${status}; safe hold at private home.`})));

  const diagnostics=coordinate?[...active,...fixed].map(a=>a.diagnostic()):[];
  const diagnosticIds=diagnostics.map(a=>a.id);
  const unique=new Map();diagnostics.forEach(a=>a.edges.forEach(e=>unique.set(e.key,e)));
  const edges=[...unique.values()].map(e=>({...e,i:diagnosticIds.indexOf(e.from),j:diagnosticIds.indexOf(e.to)}));
  progress('finished',{status,iteration:iterations,activeAgents:active.length,readyAgents:active.filter(a=>a.ready).length,
    maxPrimal:active.length?Math.max(...active.map(a=>a.primal)):undefined,maxDual:active.length?Math.max(...active.map(a=>a.dual)):undefined,
    violation:active.length?Math.max(...active.map(a=>a.violation)):undefined},true);
  return {plans,rejected,deferred,explored:agents.reduce((total,a)=>total+(a.explored??0),0),solver:{status,architecture:coordinate?'decentralized-passage':'local-guided',iterations,
    committedRobotIds:fixed.map(a=>a.id),newRobotIds:active.filter(a=>!a.fixed).map(a=>a.id),
    eliminatedFixedAgents:fixed.length,numericalAgents:active.length,
    reason:status==='converged'?null:`Local protocol ${status}; no centralized fallback executed.`,history,
    untilConverged:!!settings.admmUntilConverged,historyTruncated:iterations>history.length,elapsedMs:performance.now()-started,
    objective:history.at(-1)?.objective??0,robotIds:coordinate?diagnosticIds:ids,messages:transport.count,messageBytes:transport.bytes,
    retainedMessages:transport.log.length,messageLogTruncated:transport.count>transport.log.length,messageLog:transport.log,
    edges,
    model:{agents:diagnostics.map(a=>({id:a.id,length:a.x.length,upper:a.upper,lower:a.lower,delivery:a.raw.deliveryIndex})),edges},
    localValues:diagnostics.map(a=>a.x),arrivalSchedules:[...active,...fixed].filter(a=>a.fixed||plans.some(p=>p.robotId===a.id)).map(a=>{const p=plans.find(p=>p.robotId===a.id);return {robotId:a.id,committed:!!a.fixed,points:a.raw.points,arrivals:a.fixed?a.raw.seedArrivals:p.arrivals,waits:a.fixed?a.x:p.waypointWaits};}),
    ownership:'robot-local state; copied neighbor messages; common external task assignment; synchronous rounds',oracleUsed:false}};
}
