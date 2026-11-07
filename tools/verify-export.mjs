import {gunzipSync} from 'node:zlib';
import {validateExecution} from './validate-execution.mjs';
import fs from 'node:fs';
import {makeWorld} from '../src/world.js';
import {trajectoryChecks} from '../src/stl.js';
import {referenceTiming} from './reference-timing.mjs';
if(!process.argv[2])throw new Error('Usage: node tools/verify-export.mjs path/to/warehouse-export.json');
const raw=fs.readFileSync(process.argv[2]);
const data=JSON.parse((process.argv[2].endsWith('.gz')?gunzipSync(raw):raw).toString('utf8').replace(/^\uFEFF/,''));
const {configuration:c,trace,orders,summary}=data;
const world=makeWorld(c.fleetSize,{layout:c.warehouseLayout});
const robots=world.homes.map((home,id)=>({id,home,pos:home}));
let vertices=0,swaps=0,invalidSteps=0,batteryViolations=0;
for(let t=0;t<trace.length;t++){
  const current=trace[t].positions,old=trace[Math.max(0,t-1)].positions;
  for(let i=0;i<current.length;i++){
    if(trace[t].batteries[i]<c.settings.batteryMin-1e-9)batteryViolations++;
    if(t&&Math.abs(current[i].x-old[i].x)+Math.abs(current[i].y-old[i].y)>1)invalidSteps++;
    for(let j=0;j<i;j++){
      if(current[i].x===current[j].x&&current[i].y===current[j].y)vertices++;
      if(t&&current[i].x===old[j].x&&current[i].y===old[j].y&&current[j].x===old[i].x&&current[j].y===old[i].y)swaps++;
    }
  }
}
const latest=new Map();
const plannedChecks=['baseline','junction-assignment'].includes(c.mode)?null:data.batches.map(b=>{
  const current=robots.map((r,id)=>({...r,pos:trace.find(t=>t.tick===b.tick)?.positions[id]??r.home}));
  const remaining=[...latest.values()].filter(p=>p.start+p.points.length-1>b.tick).map(p=>({...p,points:p.points.slice(b.tick-p.start)}));
  const valid=trajectoryChecks(world,current,[...remaining,...b.plans],c.settings).ok;
  for(const p of b.plans)latest.set(p.robotId,{...p,start:b.tick});
  return valid;
});
const oracle=c.mode==='incremental'&&data.solver?.model?referenceTiming(data.solver.model):null;
const objectiveGap=oracle?.status==='converged'&&data.solver.status==='converged'?Math.abs(oracle.objective-data.solver.objective):null;
const execution=validateExecution(data);
const result={execution,mode:c.mode,agents:c.fleetSize,orders:orders.length,traceTicks:trace.length,vertices,swaps,invalidSteps,batteryViolations,
  plannedBatchesValid:plannedChecks?.every(Boolean)??null,centralStatus:oracle?.status??null,latestContinuousObjectiveGap:objectiveGap,
  oracleUsedInRuntime:data.solver?.oracleUsed??null,guardInterventions:summary.guardInterventions,
  orderAccounting:orders.filter(o=>o.done!==null).length+orders.filter(o=>o.done===null&&o.status==='rejected').length+orders.filter(o=>o.done===null&&o.status!=='rejected').length===orders.length};
console.log(JSON.stringify(result,null,2));
if(!execution.ok||vertices||swaps||invalidSteps||batteryViolations||result.plannedBatchesValid===false||!result.orderAccounting)process.exitCode=1;
