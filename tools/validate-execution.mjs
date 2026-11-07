import {makeWorld,same} from '../src/world.js';
export function validateExecution(data) {
  const {configuration:c,trace,orders,summary}=data,world=makeWorld(c.fleetSize,{layout:c.warehouseLayout});
  const errors=[];
  const fail=message=>{if(errors.length<30)errors.push(message);};
  if(trace.length!==summary.ticks+1)fail('Trace length/tick mismatch');
  for(let t=0;t<trace.length;t++){
    const row=trace[t],old=trace[Math.max(0,t-1)];if(row.tick!==t)fail(`Noncontiguous tick ${t}`);
    for(let i=0;i<row.positions.length;i++){
      const p=row.positions[i],a=old.positions[i];
      if(p.x<0||p.y<0||p.x>=world.width||p.y>=world.height||world.walls.has(`${p.x},${p.y}`))fail(`Illegal cell R${i} t${t}`);
      if(world.homes.some((h,id)=>id!==i&&same(h,p)))fail(`Foreign private bay R${i} t${t}`);
      const moves=Math.abs(p.x-a.x)+Math.abs(p.y-a.y);if(moves>1)fail(`Nonunit movement R${i} t${t}`);
      if(row.batteries[i]<c.settings.batteryMin-1e-9)fail(`Reserve R${i} t${t}`);
      if(t&&moves&&Math.abs(row.batteries[i]-(old.batteries[i]-c.settings.drain))>1e-8)fail(`Movement energy R${i} t${t}`);
      for(let j=0;j<i;j++){
        const q=row.positions[j],b=old.positions[j];
        if(Math.hypot(p.x-q.x,p.y-q.y)<c.settings.separation-1e-9)fail(`Vertex conflict t${t}`);
        if(t){const rx=a.x-b.x,ry=a.y-b.y,vx=p.x-a.x-q.x+b.x,vy=p.y-a.y-q.y+b.y,vv=vx*vx+vy*vy;
          const k=vv?Math.max(0,Math.min(1,-(rx*vx+ry*vy)/vv)):0;
          if(Math.hypot(rx+k*vx,ry+k*vy)<1e-8)fail(`Swept conflict t${t}`);}
      }
    }
  }
  for(const o of orders)if(o.done!==null){
    if(trace.find(r=>r.completedOrderIds.includes(o.id))?.tick!==o.done)fail(`Completion timestamp order ${o.id}`);
    if(o.robot===null||!same(trace[o.done]?.positions[o.robot]??{},o.dropoff))fail(`Delivery location order ${o.id}`);
  }
  if(summary.delivered+summary.rejected+summary.incomplete!==orders.length)fail('Order accounting');
  if(summary.waiting!==summary.blockedWaiting+summary.scheduledWaiting+summary.serviceWaiting)fail('Waiting partition');
  if(summary.guardInterventions)fail('Runtime physical guard intervened');
  if(summary.planningTicks!==Math.min(Math.floor(summary.planningClockMs/(1000*c.settings.secondsPerTick)),summary.ticks)&&!summary.done)fail('Planning clock');
  for(const batch of data.batches??[])if(batch.kind==='local-passage'){
    for(const g of batch.acceptedGrants){
      if(g.slot!==0||g.epoch!==batch.planningStartedTick||g.issuedTick!==batch.tick)fail('Invalid grant epoch/slot');
      if(!batch.decisions.some(d=>d.id===g.id&&d.slot===0))fail('Unsolved grant');
    }
    if(batch.solver.status==='converged')for(const d of batch.decisions){
      if(d.slot!==d.priority)fail('Integer passage chain disagrees with analytic earliest slots');
      if(Math.abs(d.value-d.priority)>.02)fail('ADMM differs from analytic local reference');
    }
  }
  return {ok:errors.length===0,errors,scope:'Actual execution, energy, delivery timestamps, accounting and local grant records'};
}
