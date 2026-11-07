import {NeighborTransport} from './robot-agent.js';
import {same,movementCount} from './world.js';

export class PeerBus {
  #handlers=new Map();
  wire=new NeighborTransport();
  register(id,handler){
    this.wire.register(id);this.#handlers.set(id,handler);
    return Object.freeze({notify:(to,type,payload)=>{
      this.wire.send(id,to,type,payload);const message=this.wire.receive(to).at(-1);
      this.#handlers.get(to)(message.type,message.payload,message.from);
    },request:(to,type,payload)=>{
      this.wire.send(id,to,type,payload);
      const message=this.wire.receive(to).at(-1);
      const reply=this.#handlers.get(to)(message.type,message.payload,message.from);
      this.wire.send(to,id,`${type}-reply`,reply??{});
      return this.wire.receive(id).at(-1).payload;
    }});
  }
}

function compatiblePredictions(a,b,separation=1){
  const bounds=p=>p.reduce((r,q)=>({loX:Math.min(r.loX,q.x),hiX:Math.max(r.hiX,q.x),loY:Math.min(r.loY,q.y),hiY:Math.max(r.hiY,q.y)}),
    {loX:Infinity,hiX:-Infinity,loY:Infinity,hiY:-Infinity});
  const x=bounds(a),y=bounds(b);
  if(x.hiX+separation<=y.loX||y.hiX+separation<=x.loX||x.hiY+separation<=y.loY||y.hiY+separation<=x.loY)return true;
  let oldA=a[0],oldB=b[0];
  for(let t=0;t<Math.max(a.length,b.length);t++){
    const p=a[Math.min(t,a.length-1)],q=b[Math.min(t,b.length-1)];
    if(Math.hypot(p.x-q.x,p.y-q.y)<separation-1e-9)return false;
    if(t){const rx=oldA.x-oldB.x,ry=oldA.y-oldB.y,vx=p.x-oldA.x-q.x+oldB.x,vy=p.y-oldA.y-q.y+oldB.y,vv=vx*vx+vy*vy;
      const f=vv?Math.max(0,Math.min(1,-(rx*vx+ry*vy)/vv)):0;
      if(Math.hypot(rx+f*vx,ry+f*vy)<1e-8)return false;}
    oldA=p;oldB=q;
  }
  return true;
}

class PlanAdmissionAgent {
  #self;#order;#world;#settings;#tick;#endpoint;#roster;#candidate;#accepted=null;#finished=false;#coordinate;
  constructor({self,order,world,settings,tick,bus,roster,candidate,coordinate=true}){
    this.id=self.id;this.#self=structuredClone(self);this.#order=structuredClone(order);
    this.#world=world;this.#settings=settings;this.#tick=tick;this.#roster=[...roster];this.#candidate=structuredClone(candidate);
    this.#coordinate=coordinate;
    this.#endpoint=bus.register(this.id,(type,payload)=>this.#receive(type,payload));
  }
  #prediction(){return this.#self.plan?this.#self.plan.points.slice(this.#self.index):this.#accepted?.points??[this.#self.pos];}
  #receive(type,payload){
    if(type==='admission-status')return {finished:this.#finished};
    if(type==='endorse-path')return {epoch:this.#tick,ok:payload.epoch===this.#tick&&compatiblePredictions(this.#prediction(),payload.points,this.#settings.separation)};
    throw new Error(`Unknown admission message ${type}`);
  }
  #ownFeasible(plan){
    if(!plan||this.#self.plan||!this.#order||!same(plan.points[0],this.#self.pos)||
      this.#tick+plan.deliveryIndex>this.#order.deadline||
      movementCount(plan.points)*this.#settings.drain+this.#settings.batteryMin>this.#self.battery+1e-9)return false;
    return plan.points.every((p,k)=>p.x>=0&&p.y>=0&&p.x<this.#world.width&&p.y<this.#world.height&&
      !this.#world.walls.has(`${p.x},${p.y}`)&&(!k||Math.abs(p.x-plan.points[k-1].x)+Math.abs(p.y-plan.points[k-1].y)<=1));
  }
  advance(){
    if(this.#finished)return true;

    if(this.#coordinate&&this.#candidate&&this.#roster.some(id=>id<this.id&&!this.#endpoint.request(id,'admission-status',{}).finished))return false;
    if(this.#candidate){
      let ok=this.#ownFeasible(this.#candidate);
      if(ok&&this.#coordinate)for(const id of this.#roster)if(id!==this.id){
        const reply=this.#endpoint.request(id,'endorse-path',{epoch:this.#tick,points:this.#candidate.points});
        if(!reply.ok||reply.epoch!==this.#tick){ok=false;break;}
      }
      if(ok)this.#accepted=this.#candidate;
    }
    this.#finished=true;return true;
  }
  decision(){return {id:this.id,plan:this.#accepted,orderId:this.#candidate?.orderId,finished:this.#finished};}
}

export function admitPeerPlans(world,robots,orders,plans,tick,settings,{coordinate=true}={}){
  const bus=new PeerBus(),roster=robots.map(r=>r.id);
  const agents=robots.map(self=>new PlanAdmissionAgent({self,order:orders.find(o=>o.id===plans.find(p=>p.robotId===self.id)?.orderId),
    candidate:plans.find(p=>p.robotId===self.id),world,settings,tick,bus,roster,coordinate}));

  agents.filter(a=>!plans.some(p=>p.robotId===a.id)).forEach(a=>a.advance());
  for(let round=0;round<=agents.length;round++)if(agents.map(a=>a.advance()).every(Boolean))break;
  const decisions=agents.map(a=>a.decision());
  if(decisions.some(d=>!d.finished))throw new Error('Peer admission token did not terminate');
  return {plans:decisions.flatMap(d=>d.plan?[d.plan]:[]),
    deferred:decisions.filter(d=>d.orderId!==undefined&&!d.plan).map(d=>({robotId:d.id,orderId:d.orderId,reason:'Owning agent deferred its stale or unendorsed path.'})),
    validation:{ok:true,scope:coordinate?'Own feasibility and bilateral peer endorsements':'Owning robot feasibility; local passage admission during execution',messages:bus.wire.count,messageBytes:bus.wire.bytes}};
}

export const ownRetryTick=(self,tick)=>self.plan?tick:tick+5;
