import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import {gunzipSync} from 'node:zlib';
import {fileURLToPath} from 'node:url';
const root=fileURLToPath(new URL('../',import.meta.url));
const dir=path.resolve(root,process.argv[2]||'benchmarks/latest');
const rows=JSON.parse(fs.readFileSync(path.join(dir,'runs.json'),'utf8'));
const meta=JSON.parse(fs.readFileSync(path.join(dir,'metadata.json'),'utf8'));
const modes=['baseline','guided-local','incremental','junction-assignment'];
const labels=['BFS baseline','STL-guided search','Route-timing ADMM','Crossing-allocation ADMM'];
if(rows.length!==80||rows.some(r=>r.status!=='measured'||!r.validation?.ok||!r.done))throw Error('Campaign incomplete or contains invalid/unmeasured executions.');
const sourceHashesAfter=Object.fromEntries(Object.keys(meta.sources).map(n=>[n,crypto.createHash('sha256').update(fs.readFileSync(path.join(root,'src',n))).digest('hex')]));
if(Object.entries(meta.sources).some(([n,h])=>sourceHashesAfter[n]!==h))throw Error('Source changed during campaign');
for(const fleet of [30,50])for(const seed of meta.design.seeds){
 const group=rows.filter(r=>r.fleetSize===fleet&&r.seed===seed);
 if(group.length!==4||new Set(group.map(r=>r.mode)).size!==4||new Set(group.map(r=>r.workloadHash)).size!==1)throw Error('Unpaired workload');
}
function quantile(a,q){const p=(a.length-1)*q,l=Math.floor(p),u=Math.ceil(p);return a[l]+(a[u]-a[l])*(p-l);}
const mean=a=>a.reduce((s,x)=>s+x,0)/a.length;
const summary=[];
for(const fleet of [30,50])for(const [i,mode]of modes.entries()){
 const group=rows.filter(r=>r.fleetSize===fleet&&r.mode===mode),times=[];
 for(const r of group){
  const raw=JSON.parse(gunzipSync(fs.readFileSync(path.join(dir,r.rawFile))));
  times.push(...raw.orders.filter(o=>o.done!==null).map(o=>o.done-o.created));
 }
 times.sort((a,b)=>a-b);
 const sum=k=>group.reduce((s,r)=>s+r[k],0),rates=group.map(r=>100*r.onTimeRate),avg=mean(rates);
 summary.push({fleet,method:labels[i],mode,completed:group.length,expected:10,generatedInCompleted:sum('total'),delivered:sum('delivered'),onTime:sum('onTime'),
  dispatchPolicy:group[0].dispatchPolicy,
  meanTicks:mean(times),medianTicks:quantile(times,.5),p90Ticks:quantile(times,.9),waiting:sum('waiting'),blockedWaiting:sum('blockedWaiting'),
  scheduledWaiting:sum('scheduledWaiting'),serviceWaiting:sum('serviceWaiting'),planningMeanMs:mean(group.map(r=>r.planningMs)),
  planningMedianMs:quantile(group.map(r=>r.planningMs).sort((a,b)=>a-b),.5),planningP90Ms:quantile(group.map(r=>r.planningMs).sort((a,b)=>a-b),.9),
  stalls:group.filter(r=>r.stopReason?.includes('stalled')).length,earlyStops:group.filter(r=>r.stopReason).length,rejected:sum('rejected'),incomplete:sum('incomplete'),
  onTimeRate:sum('onTime')/sum('total'),deliveryRate:sum('delivered')/sum('total'),waitingMean:sum('waiting')/10,
  onTimeSd:Math.sqrt(rates.reduce((s,x)=>s+(x-avg)**2,0)/9),onTimeMin:Math.min(...rates),onTimeMax:Math.max(...rates),
  collisions:sum('collisions')+sum('edgeConflicts'),guardInterventions:sum('guardInterventions'),overlappingDispatches:sum('overlappingDispatches'),
  partialChargeDispatches:sum('partialChargeDispatches'),planningTicksMean:mean(group.map(r=>r.planningTicks)),
  throughputPer100TicksMean:mean(group.map(r=>r.throughputPer100Ticks)),
  waitingPerDeliveredOrder:sum('waiting')/sum('delivered'),
  minimumSafetyMargin:Math.min(...group.map(r=>r.safetyRobustness)),
  minimumBatteryMargin:Math.min(...group.map(r=>r.batteryRobustness))});
}
const csvEscape=x=>/[",\n]/.test(String(x))?'"'+String(x).replaceAll('"','""')+'"':String(x);
function csv(file,data,keys=Object.keys(data[0])){fs.writeFileSync(path.join(dir,file),keys.join(',')+'\n'+data.map(r=>keys.map(k=>csvEscape(r[k]??'')).join(',')).join('\n')+'\n');}
csv('summary.csv',summary);
csv('runs.csv',rows,['fleetSize','seed','mode','status','total','delivered','onTime','onTimeRate','meanDeliveryTime','medianDeliveryTime','p90DeliveryTime','waiting','blockedWaiting','scheduledWaiting','serviceWaiting','planningMs','planningTicks','overlappingDispatches','partialChargeDispatches','collisions','edgeConflicts','guardInterventions','rejected','incomplete','stopReason','wallMs']);
fs.writeFileSync(path.join(dir,'summary.json'),JSON.stringify(summary,null,2));
meta.sourceHashesAfter=sourceHashesAfter;meta.sourcesUnchanged=true;meta.pairedWorkloadsVerified=true;
meta.validatedRuns=rows.filter(r=>r.validation.ok).length;
fs.writeFileSync(path.join(dir,'metadata.json'),JSON.stringify(meta,null,2));
const batchBfs=rows.filter(r=>r.mode==='baseline').every(r=>r.dispatchPolicy==='fleet-batch');
let report='# Current-controller benchmark\n\n7 November 2026. '+(batchBfs?'BFS uses fleet-batch dispatch. STL-guided search and both ADMM methods use individual readiness.':'All four methods use individual readiness.')+' Every method uses one mission per robot and mission-energy admission.\n\n';
report+='80 serial runs: 4 methods × 2 fleets (30 and 50 robots) × 10 paired seeds. Two orders per robot, medium shared-resource demand, mixed challenges, fixed 54 × 22 map. Seeds: '+meta.design.seeds.join(', ')+'. ADMM rho = 5, until convergence.\n\n';
report+='Execution uses the simulation worker and nested planning worker, without animation delays. Measured planning latency, including planning-worker overhead, consumes mission ticks. The previous presentation used a synchronous benchmark harness, so old/new wall-time figures are not a controlled dispatch-only comparison. These fresh results compare all four current controllers under the same worker-based harness.\n\n';
for(const fleet of [30,50]){
 report+=`## ${fleet} robots\n\n| Method | Delivered | On time | Mean / median ticks | P90 ticks | Waiting/run | Planning s/run | Stalled runs |\n|---|---:|---:|---:|---:|---:|---:|---:|\n`;
 for(const r of summary.filter(r=>r.fleet===fleet))report+=`| ${r.method} | ${r.delivered}/${r.generatedInCompleted} | ${(100*r.onTimeRate).toFixed(1)}% | ${r.meanTicks.toFixed(1)} / ${r.medianTicks.toFixed(1)} | ${r.p90Ticks.toFixed(1)} | ${r.waitingMean.toFixed(1)} | ${(r.planningMeanMs/1000).toFixed(2)} | ${r.stalls}/10 |\n`;
 report+='\n';
}
report+='## Interpretation and limits\n\nDelivery-time statistics pool delivered orders across the ten seeds; rejected and incomplete orders remain in delivery-rate denominators. Waiting includes blocked, scheduled and service ticks. Seed variability uses sample standard deviation. Stalled runs remain in all aggregates. All 80 exports passed recorded execution checks; source hashes remained unchanged and paired workload hashes matched.\n\n';
report+='## Additional website aggregates\n\n| Fleet | Method | Throughput / 100 ticks | Blocked/run | Scheduled/run | Service/run | Rejected | Incomplete | Collisions | Guard interventions |\n|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|\n';
for(const r of summary)report+=`| ${r.fleet} | ${r.method} | ${r.throughputPer100TicksMean.toFixed(2)} | ${(r.blockedWaiting/10).toFixed(1)} | ${(r.scheduledWaiting/10).toFixed(1)} | ${(r.serviceWaiting/10).toFixed(1)} | ${r.rejected} | ${r.incomplete} | ${r.collisions} | ${r.guardInterventions} |\n`;
report+='\nThroughput is the mean of per-run delivered orders per 100 mission ticks. Waiting components are mean robot-ticks/run. Rejected, incomplete, collision and guard counts sum ten runs.\n\n## Planning variability\n\n| Fleet | Method | Mean seconds/run | Median seconds/run | P90 seconds/run | On-time rate SD (percentage points) |\n|---:|---|---:|---:|---:|---:|\n';
for(const r of summary)report+=`| ${r.fleet} | ${r.method} | ${(r.planningMeanMs/1000).toFixed(2)} | ${(r.planningMedianMs/1000).toFixed(2)} | ${(r.planningP90Ms/1000).toFixed(2)} | ${r.onTimeSd.toFixed(2)} |\n`;
report+='\nAll recorded timing outliers remain included. Medians and P90 values expose scheduling variability. This campaign does not measure animation smoothness in the browser.\n\n';
for(const fleet of [30,50]){
 const group=summary.filter(r=>r.fleet===fleet),best=key=>group.reduce((a,b)=>a[key]<=b[key]?a:b);
 const guided=group.filter(r=>r.mode!=='baseline');
 if(guided.every(r=>Math.abs(r.onTimeRate-guided[0].onTimeRate)<1e-9))report+=`At ${fleet} robots, STL and both ADMM methods tie at ${(100*guided[0].onTimeRate).toFixed(1)}% on-time delivery. `;
 report+=`Lowest pooled mean delivery time: ${best('meanTicks').method} (${best('meanTicks').meanTicks.toFixed(1)} ticks). Lowest waiting: ${best('waitingMean').method} (${best('waitingMean').waitingMean.toFixed(1)} robot-ticks/run). Lowest planning cost: ${best('planningMeanMs').method} (${(best('planningMeanMs').planningMeanMs/1000).toFixed(2)} s/run).\n\n`;
}
report+=(batchBfs?'Dispatch policy differs: comparisons against BFS measure the complete batch-BFS controller against individual-dispatch controllers. They do not isolate routing, STL or ADMM gains from dispatch effects.':'Common individual dispatch removes the fleet-barrier difference, but does not isolate ADMM alone.')+' Routing, recovery, directed-lane and crossing policies still differ. Ten percent of generated orders deliberately have impossible deadlines. Zero observed collisions is an experimental result, not a general safety proof. Computation time depends on hardware and runtime scheduling. Raw compressed exports, per-run CSV, source snapshot and metadata accompany this report.\n';
fs.writeFileSync(path.join(dir,'report.md'),report);
console.log(JSON.stringify(summary.map(r=>({fleet:r.fleet,method:r.method,onTime:100*r.onTimeRate,mean:r.meanTicks,waiting:r.waitingMean,planningS:r.planningMeanMs/1000,stalls:r.stalls})),null,2));
