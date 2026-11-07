import { planAgentBatch } from './robot-agent.js';
import { admitPeerPlans } from './peer-protocol.js';
import { spatialMission, movementCount } from './world.js';

const admissionCache=new WeakMap();
export function admissionEnergy(world,robot,order,settings){
  if(!admissionCache.has(world))admissionCache.set(world,new Map());
  const cache=admissionCache.get(world);
  const key=JSON.stringify([robot.pos,robot.home,!!robot.carrying,robot.state==='return',order.pickup,order.dropoff]);
  if(!cache.has(key)){
    const plan=spatialMission(world,robot,order);
    cache.set(key,plan?movementCount(plan.points):null);
  }
  const moves=cache.get(key);
  return moves===null?null:moves*settings.drain+settings.batteryMin+0.015;
}

export function remainingCommitments(robots) {
  return robots.filter(r=>r.plan).map(r=>({...r.plan,
    points:r.plan.points.slice(r.index).map(p=>({...p})),
    pickupIndex:Math.max(0,r.plan.pickupIndex-r.index),
    deliveryIndex:Math.max(0,r.plan.deliveryIndex-r.index)}));
}

export function planIncremental(world,robots,jobs,tick,settings,routePlanner) {
  const commitments=remainingCommitments(robots);
  const result=planAgentBatch(world,robots,jobs,tick,settings,routePlanner,
    {coordinate:true,maxRounds:settings.admmMaxRounds??3000,commitments,peerRouting:true});

  const admission=admitPeerPlans(world,robots,jobs.map(j=>j.order),result.plans,tick,settings);
  result.plans=admission.plans;result.deferred.push(...admission.deferred);
  result.solver.architecture='decentralized-passage';
  result.solver.dispatchPolicy='event-driven';result.solver.commitmentsPreserved=true;
  result.solver.validation=admission.validation;
  result.solver.admissionMessages=admission.validation.messages;
  result.solver.messages+=admission.validation.messages;
  result.solver.messageBytes+=admission.validation.messageBytes;
  return result;
}
