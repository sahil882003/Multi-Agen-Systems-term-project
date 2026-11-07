import { same, cellKey, neighbours, movementCount } from "./world.js";
import {peerJunctionFrame,peerJunctionMoves} from './junction-peer.js';

export function junctionLaneStep(from,to,home) {
  return same(from,to)||same(from,home)||same(to,home)||(from.y>=home.y-2&&to.y>=home.y-2)||
    (from.y===to.y?Math.sign(to.x-from.x)===(from.y%2?1:-1):Math.sign(to.y-from.y)===(from.x%2?-1:1));
}
export function junctionSpatialMission(world,robot,order) {
  const points=[{...robot.pos}],actions=[];let pickupIndex=null,deliveryIndex=null;
  if(robot.state!=='return'){
    if(!robot.carrying)actions.push({point:order.pickup,kind:'pickup'});
    actions.push({point:order.dropoff,kind:'delivery'});
  }
  actions.push({point:robot.home,kind:'home'});
  for(const action of actions){
    const start=points.at(-1),queue=[start],parents=new Map([[cellKey(start),null]]);let found=false;
    for(let head=0;head<queue.length;head++){
      const p=queue[head];if(same(p,action.point)){found=true;break;}
      for(const q of neighbours(world,p,robot.home))if(junctionLaneStep(p,q,robot.home)&&!parents.has(cellKey(q))){parents.set(cellKey(q),p);queue.push(q);}
    }
    if(!found)return null;
    const leg=[];for(let p=action.point;p!==null;p=parents.get(cellKey(p)))leg.push({...p});
    points.push(...leg.reverse().slice(1));
    if(action.kind!=='home'){points.push({...action.point});if(action.kind==='pickup')pickupIndex=points.length-1;else deliveryIndex=points.length-1;}
  }
  return {robotId:robot.id,orderId:order.id,points,pickupIndex,deliveryIndex};
}
const junctionEnergyCache=new WeakMap();
export function junctionAdmissionEnergy(world,robot,order,settings) {
  if(!junctionEnergyCache.has(world))junctionEnergyCache.set(world,new Map());
  const cache=junctionEnergyCache.get(world),key=JSON.stringify([robot.pos,robot.home,order.pickup,order.dropoff]);
  if(!cache.has(key)){
    const plan=junctionSpatialMission(world,robot,order);
    cache.set(key,plan?movementCount(plan.points):null);
  }
  const moves=cache.get(key);
  return moves===null?null:moves*settings.drain+settings.batteryMin+.015;
}

export function junctionPriority(a,b) {
  return Number(!!b.clearance)-Number(!!a.clearance) || (a.slack-2*a.age)-(b.slack-2*b.age) || a.id-b.id;
}
export function junctionIntent(robot,order,tick) {
  const remaining=robot.state==='return'?Infinity:Math.max(0,robot.plan.deliveryIndex-robot.index);
  const slack=Number.isFinite(remaining)?order.deadline-tick-remaining:0;
  return {id:robot.id,from:{...robot.pos},to:{...robot.plan.points[robot.index+1]},
    slack,age:robot.blocked,weight:1+4/(1+Math.max(0,slack)/20),epoch:tick,
    ...(robot.slotAssignment?{battery:robot.battery,minimumBattery:robot.slotAssignment.batteryMin,
      remainingEnergy:movementCount(robot.plan.points.slice(robot.index))*robot.slotAssignment.drain,
      deliveryActive:robot.state!=='return'}:{})};
}
export function planJunctionRoutes(world,jobs,tick,settings,routePlanner) {
  const plans=[],rejected=[];
  for(const {robot,order} of jobs) {

    const candidate=routePlanner(world,structuredClone(robot),structuredClone(order),[],[structuredClone(robot)],tick,{...settings,junctionLanes:true},true);
    if(candidate.ok)plans.push(candidate.plan);
    else rejected.push({orderId:order.id,reason:candidate.reason});
  }
  return {plans,rejected,deferred:[],solver:null,explored:0};
}

export function junctionFrame(...args){return peerJunctionFrame(...args);}
export function junctionMoves(...args){return peerJunctionMoves(...args);}

