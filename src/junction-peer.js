import {PeerBus} from './peer-protocol.js';
import {same,cellKey,neighbours,movementCount} from './world.js';
import {junctionIntent,junctionPriority,junctionLaneStep,junctionSpatialMission} from './junction.js';

class JunctionRobot {
  #self;#order;#world;#settings;#tick;#endpoint;#seen=[];#held;#override;#prepared=new Map();#movement;
  #immutable;#advertised;#localAnnouncements;
  #lastDeparture;
  #openExit=false;
  constructor({self,order,world,settings,tick,bus,held=false,intent}){
    this.id=self.id;this.#self=self;this.#order=order;this.#world=world;
    this.#settings=settings;this.#tick=tick;this.#held=held;this.#override=intent;
    this.#immutable=!world;
    this.#endpoint=bus.register(this.id,(type,payload,from)=>this.#receive(type,payload,from));
  }
  position(){return {id:this.id,pos:{...this.#self.pos},active:!!(this.#override||this.#self.plan&&this.#self.index+1<this.#self.plan.points.length)};}
  sense(nearby){this.#seen=structuredClone(nearby);}
  #intent(){return this.#override??(this.#self.plan&&this.#self.index+1<this.#self.plan.points.length?
    junctionIntent(this.#self,this.#order,this.#tick):null);}
  #owner(point){return this.#seen.find(p=>same(p.pos,point));}
  #ask(id,type,payload={}){
    if(!this.#seen.some(p=>p.id===id))throw new Error('Junction request outside sensed neighbourhood');
    return this.#endpoint.request(id,type,payload);
  }
  #receive(type,payload,from){
    if(type==='intent')return {intent:this.discover()};
    if(type==='probe'){
      if(payload.origin===this.id)return {clearance:payload.visited.length>=4};
      if(this.#immutable&&this.#openExit)return {clearance:false,open:true};
      const own=this.#intent();
      if(payload.visited.includes(this.id)||payload.visited.length>=50)return {clearance:false};
      if(!own||same(own.from,own.to))return {clearance:false,open:true};
      const owner=this.#owner(own.to);if(!owner)return {clearance:false,open:true};
      const reply=this.#ask(owner.id,'probe',{...payload,visited:[...payload.visited,this.id]});
      if(this.#immutable&&reply.open)this.#openExit=true;return reply;
    }
    if(type==='yield-prepare')return this.#vacate(payload,from);
    if(type==='yield-commit')return this.#commit(payload,from);
    if(type==='yield-cancel'){
      const staged=this.#prepared.get(payload.transaction);
      if(staged?.child!==undefined)this.#ask(staged.child,'yield-cancel',payload);
      this.#prepared.delete(payload.transaction);return {ok:true};
    }
    if(type==='departure-notice'){
      const own=this.#movement;
      if(payload.epoch===this.#tick&&own?.ready&&own.owner===from&&!same(own.own.from,own.own.to)&&
        (!payload.departure||same(payload.from,payload.to))){own.ready=false;this.announceDeparture();}
      return;
    }
    throw new Error(`Unknown junction message ${type}`);
  }
  discover(){
    if(this.#override)return this.#override;
    if(this.#immutable&&this.#advertised!==undefined)return this.#advertised;
    const own=this.#intent();if(!own)return null;
    if(!same(own.from,own.to)){
      const owner=this.#owner(own.to);
      if(owner){const reply=this.#ask(owner.id,'probe',{origin:this.id,visited:[this.id]});
        if(reply.clearance)own.clearance=true;if(this.#immutable&&reply.open)this.#openExit=true;}
    }
    if(this.#immutable)this.#advertised=own;return own;
  }
  #localIntents(){
    if(this.#immutable&&this.#localAnnouncements)return this.#localAnnouncements;
    const announcements=this.#seen.filter(p=>p.active!==false&&Math.abs(p.pos.x-this.#self.pos.x)+Math.abs(p.pos.y-this.#self.pos.y)<=2)
      .map(p=>this.#ask(p.id,'intent').intent).filter(Boolean);
    if(this.#immutable)this.#localAnnouncements=announcements;return announcements;
  }
  contenders(own){return [own,...this.#localIntents()].filter(p=>!same(p.from,p.to)&&same(p.to,own.to));}
  group(){
    const own=this.discover();if(!own||same(own.from,own.to))return null;
    const requests=this.contenders(own).map(p=>p.id===this.id?own:{...p});
    return requests.length>1?{resource:cellKey(own.to),requests}:null;
  }
  acceptGrant(decision,epoch,status){
    const own=this.#intent();if(status!=='converged'||decision.slot!==0||!own)return null;
    if(decision.assignment&&(decision.epoch!==this.#tick||!Number.isFinite(decision.battery)||
      Math.abs(decision.battery-this.#self.battery)>1e-9||
      own.battery-own.remainingEnergy<own.minimumBattery-1e-9))return null;
    const roster=this.contenders(own).map(p=>p.id).sort((a,b)=>a-b).join('/');
    return decision.epoch===epoch&&same(own.from,decision.from)&&same(own.to,decision.to)&&
      roster===[...decision.roster].sort((a,b)=>a-b).join('/')?{...decision,issuedTick:this.#tick}:null;
  }
  #makeYieldPlan(point){
    const plan=junctionSpatialMission(this.#world,{...this.#self,pos:point},this.#order);if(!plan)return null;
    plan.points.unshift({...this.#self.pos});if(plan.pickupIndex!==null)plan.pickupIndex++;if(plan.deliveryIndex!==null)plan.deliveryIndex++;
    return movementCount(plan.points)*this.#settings.drain+this.#settings.batteryMin<=this.#self.battery+1e-9?plan:null;
  }
  #vacate(payload,caller){
    const r=this.#self;let spent=1;const budget=payload.budget??100;
    if(budget<1||payload.epoch!==this.#tick||payload.chain.length>12||payload.chain.includes(this.id)||this.#held||!r.plan||same(r.pos,r.plan.points[r.index+1]))return {ok:false,count:0,spent};
    const occupied=this.#seen.filter(p=>Math.abs(p.pos.x-r.pos.x)+Math.abs(p.pos.y-r.pos.y)===1);
    const choices=neighbours(this.#world,r.pos,r.home).filter(p=>junctionLaneStep(r.pos,p,r.home)&&!payload.forbidden.includes(cellKey(p)));
    choices.sort((a,b)=>Number(occupied.some(p=>same(p.pos,a)))-Number(occupied.some(p=>same(p.pos,b)))||
      (this.#tick+this.id)%2*(a.x-b.x)||a.y-b.y||a.x-b.x);
    for(const point of choices){
      if(spent>=budget)break;
      const owner=occupied.find(p=>same(p.pos,point));
      if(owner&&(payload.chain.includes(owner.id)||owner.id===payload.root))continue;
      const plan=this.#makeYieldPlan(point);if(!plan)continue;
      const child=owner?this.#ask(owner.id,'yield-prepare',{...payload,budget:budget-spent,chain:[...payload.chain,this.id],forbidden:[...payload.forbidden,cellKey(r.pos)]}):{ok:true,count:0,spent:0};
      spent+=child.spent;
      if(!child.ok)continue;
      this.#prepared.set(payload.transaction,{plan,child:owner?.id,epoch:this.#tick,caller});
      return {ok:true,count:1+child.count,spent};
    }
    return {ok:false,count:0,spent};
  }
  #commit(payload,caller){
    const staged=this.#prepared.get(payload.transaction);
    if(!staged||payload.epoch!==this.#tick||staged.epoch!==this.#tick||
      caller!==undefined&&caller!==staged.caller||!same(staged.plan.points[0],this.#self.pos))return {ok:false};
    if(staged.child!==undefined&&!this.#ask(staged.child,'yield-commit',payload).ok)return {ok:false};
    this.#self.plan=staged.plan;this.#self.index=0;
    this.#self.junctionRecoveryAfter=this.#tick+8;this.#self.junctionYieldAfter=this.#tick+8;
    this.#prepared.delete(payload.transaction);return {ok:true};
  }
  yield(){
    const r=this.#self;if(!r.plan||r.blocked<8||this.#held||this.#tick<(r.junctionYieldAfter??0))return 0;
    const blocker=this.#owner(r.plan.points[r.index+1]);if(!blocker)return 0;
    const a=this.#intent(),b=this.#ask(blocker.id,'intent').intent;
    if(b&&same(b.to,r.pos)&&junctionPriority(a,b)<0)return 0;
    const payload={epoch:this.#tick,transaction:`${this.#tick}/${this.id}`,root:this.id,budget:100,chain:[],forbidden:[cellKey(blocker.pos)]};
    const prepared=this.#vacate(payload,blocker.id);
    if(prepared.ok&&this.#commit(payload).ok)return prepared.count;
    this.#receive('yield-cancel',payload,this.id);r.junctionYieldAfter=this.#tick+4;return 0;
  }
  recover(){
    const r=this.#self;if(!r.plan||r.blocked<4||this.#held||this.#tick<(r.junctionRecoveryAfter??0))return false;
    const next=r.plan.points[r.index+1];if(!next||same(next,r.pos))return false;
    const temporary={...this.#world,walls:new Set([...this.#world.walls,...this.#seen.map(p=>cellKey(p.pos))])};
    let plan=junctionSpatialMission(temporary,r,this.#order);
    if(plan&&plan.points.length>1&&!same(plan.points[1],next)&&movementCount(plan.points)*this.#settings.drain+this.#settings.batteryMin<=r.battery+1e-9){
      r.plan=plan;r.index=0;r.junctionRecoveryAfter=this.#tick+4;return true;
    }
    const blocker=this.#owner(next);if(!blocker)return false;
    const b=this.#ask(blocker.id,'intent').intent;if(!b)return false;
    if(junctionPriority(this.#intent(),b)<0&&r.blocked<12)return false;
    const choices=neighbours(this.#world,r.pos,r.home).filter(p=>junctionLaneStep(r.pos,p,r.home)&&!same(p,next)&&!this.#seen.some(q=>same(q.pos,p)));
    choices.sort((p,q)=>(Math.abs(q.x-next.x)+Math.abs(q.y-next.y))-(Math.abs(p.x-next.x)+Math.abs(p.y-next.y))||p.y-q.y||p.x-q.x);
    for(const point of choices){plan=this.#makeYieldPlan(point);if(plan){r.plan=plan;r.index=0;r.junctionRecoveryAfter=this.#tick+6;return true;}}
    return false;
  }
  beginMovement(grants){
    const own=this.#intent();if(!own)return;
    const contenders=this.contenders(own),owner=this.#owner(own.to);
    const grant=grants.find(g=>g.id===this.id&&g.slot===0&&g.issuedTick===this.#tick&&same(g.from,own.from)&&same(g.to,own.to)&&
      [...g.roster].sort((a,b)=>a-b).join('/')===contenders.map(p=>p.id).sort((a,b)=>a-b).join('/'));
    const allowed=!this.#held&&(same(own.from,own.to)||contenders.length<=1||!!grant);
    const opposite=owner?this.#ask(owner.id,'intent').intent:null;
    this.#movement={own,owner:owner?.id,ready:allowed&&!(opposite&&same(opposite.to,own.from)&&!same(own.from,own.to))};
  }
  departure(){return !!this.#movement?.ready;}
  announceDeparture(){
    const own=this.#movement?.own??{from:this.#self.pos,to:this.#self.pos};
    const departure=this.departure();if(this.#lastDeparture===departure)return;
    this.#lastDeparture=departure;
    for(const peer of this.#seen)if(peer.active!==false&&Math.abs(peer.pos.x-this.#self.pos.x)+Math.abs(peer.pos.y-this.#self.pos.y)<=2)
      this.#endpoint.notify(peer.id,'departure-notice',{id:this.id,from:own.from,to:own.to,departure,epoch:this.#tick});
  }
}

function junctionEndpoints(robots,orders,tick,{world,settings,held=[],intents}={}){
  const bus=new PeerBus();
  const agents=robots.map(self=>new JunctionRobot({self,order:orders.find(o=>o.id===self.orderId),world,settings,tick,bus,
    held:held.includes(self.id),intent:intents?.find(p=>p.id===self.id)}));
  const positions=agents.map(a=>a.position());
  for(const a of agents){const own=a.position().pos;a.sense(positions.filter(p=>p.id!==a.id&&Math.abs(p.pos.x-own.x)+Math.abs(p.pos.y-own.y)<=4));}
  return {agents,bus};
}

export function peerJunctionFrame(robots,orders,tick){
  const {agents,bus}=junctionEndpoints(robots,orders,tick),intents=agents.map(a=>a.discover()).filter(Boolean),groups=new Map();
  for(const a of agents){const g=a.group();if(g&&!groups.has(g.resource))groups.set(g.resource,g);}
  return {tick,intents,groups:[...groups.values()],probeMessages:bus.wire.count,probeBytes:bus.wire.bytes};
}
export function peerJunctionRecovery(world,robots,orders,tick,settings,held=[]){
  const {agents,bus}=junctionEndpoints(robots,orders,tick,{world,settings,held});
  let count=0;for(const a of agents)count+=a.yield();for(const a of agents)count+=Number(a.recover());
  return {count,messages:bus.wire.count,bytes:bus.wire.bytes};
}
export function peerJunctionGrants(robots,orders,tick,decisions,epoch,status){
  const {agents}=junctionEndpoints(robots,orders,tick);
  return agents.flatMap(a=>{const d=decisions.find(d=>d.id===a.id);const grant=d?a.acceptGrant(d,epoch,status):null;return grant?[grant]:[];});
}
export function peerJunctionMoves(frame,robots,grants=[],held=[]){
  const {agents,bus}=junctionEndpoints(robots,[],frame.tick,{held,intents:frame.intents});
  agents.forEach(a=>a.beginMovement(grants));

  agents.forEach(a=>a.announceDeparture());
  const result=new Set(agents.filter(a=>a.departure()).map(a=>a.id));
  result.messages=bus.wire.count+(frame.probeMessages??0);result.bytes=bus.wire.bytes+(frame.probeBytes??0);result.messageLog=bus.wire.log;return result;
}
