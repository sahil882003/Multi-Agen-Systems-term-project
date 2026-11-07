import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import os from 'node:os';
import {fileURLToPath,pathToFileURL} from 'node:url';
import {gzipSync} from 'node:zlib';
import {makeBrowserWorker} from './browser-worker-adapter.mjs';
import {validateExecution} from './validate-execution.mjs';
import {makeWorld} from '../src/world.js';
const root=fileURLToPath(new URL('../',import.meta.url));
const out=path.resolve(root,process.env.BENCHMARK_OUT||'benchmarks/current-runs');
fs.mkdirSync(out,{recursive:true});
const lock=path.join(out,'active.lock');
const fd=fs.openSync(lock,'wx');fs.writeFileSync(fd,String(process.pid));
process.on('exit',()=>{try{fs.closeSync(fd);fs.unlinkSync(lock);}catch{}});
const hash=b=>crypto.createHash('sha256').update(b).digest('hex');
const snapshot=path.join(out,'source');fs.mkdirSync(snapshot,{recursive:true});
if(!fs.existsSync(path.join(snapshot,'src'))){
 fs.cpSync(path.join(root,'src'),path.join(snapshot,'src'),{recursive:true});
 fs.writeFileSync(path.join(snapshot,'package.json'),'{"type":"module"}');
}
const sources=Object.fromEntries(fs.readdirSync(path.join(snapshot,'src')).filter(x=>x.endsWith('.js')).map(n=>[n,hash(fs.readFileSync(path.join(snapshot,'src',n)))]));
for(const [n,h] of Object.entries(sources))if(hash(fs.readFileSync(path.join(root,'src',n)))!==h)throw Error('Source differs from frozen benchmark: '+n);
const modes=['baseline','guided-local','incremental','junction-assignment'];
const labels={'baseline':'BFS baseline','guided-local':'STL-guided search','incremental':'Route-timing ADMM','junction-assignment':'Crossing-allocation ADMM'};
const seeds=[7,101,3001,3002,3003,3004,3005];
let cases=[];
for(const fleetSize of [5,10,20,30,50])for(const ordersPerRobot of [1,3])for(const congestion of ['low','medium','high'])for(const seed of seeds){
 const shift=seed%4;
 for(const mode of [...modes.slice(shift),...modes.slice(0,shift)])cases.push({mode,fleetSize,ordersPerRobot,congestion,seed,scenario:'mixed',warehouseLayout:'standard',admmRho:5,admmUntilConverged:true});
}
const pilot=process.argv.includes('--pilot');
const presentation=process.argv.includes('--presentation');
if(presentation)cases=[];
if(presentation)for(const fleetSize of [30,50])for(const [i,seed] of [7,11,23,42,71,101,211,509,2026,3001].entries()){
 const shift=i%4;
 for(const mode of [...modes.slice(shift),...modes.slice(0,shift)])cases.push({mode,fleetSize,ordersPerRobot:2,congestion:'medium',seed,scenario:'mixed',warehouseLayout:'fixed-large',admmRho:5,admmUntilConverged:true,secondsPerTick:1});
}
if(pilot)cases=[5,30].flatMap(fleetSize=>modes.map(mode=>({mode,fleetSize,ordersPerRobot:3,congestion:'high',seed:3001,scenario:'mixed',warehouseLayout:'standard',admmRho:5,admmUntilConverged:true})));
const filename=pilot?'pilot.json':'runs.json';
const results=fs.existsSync(path.join(out,filename))?JSON.parse(fs.readFileSync(path.join(out,filename),'utf8')):[];
const key=c=>[c.mode,c.fleetSize,c.ordersPerRobot,c.congestion,c.seed].join('-');
const watchdogMs=120000;
const meta={started:new Date().toISOString(),runtime:process.version,cpu:os.cpus()[0].model,platform:os.platform(),sources,labels,design:{fleets:[5,10,20,30,50],ordersPerRobot:[1,3],congestion:['low','medium','high'],seeds,modes,scenario:'mixed',layout:'app standard; automatic scaled warehouse for 30 and 50 robots',rho:5,untilConverged:true,evaluationHorizon:'app defaults; recorded per raw export',watchdogMs,serial:true,execution:'Actual simulation-worker.js + nested planning-worker.js via Node browser-worker API adapter; no rendering or playback delay',plannedRuns:cases.length}};
fs.writeFileSync(path.join(out,pilot?'pilot-metadata.json':'metadata.json'),JSON.stringify(meta,null,2));
if(presentation){Object.assign(meta.design,{fleets:[30,50],ordersPerRobot:[2],congestion:['medium'],seeds:[7,11,23,42,71,101,211,509,2026,3001],layout:'fixed-large 54x22 for every run',dispatchPolicy:{baseline:'fleet-batch','guided-local':'event-driven',incremental:'event-driven','junction-assignment':'event-driven'},plannedRuns:cases.length});fs.writeFileSync(path.join(out,'metadata.json'),JSON.stringify(meta,null,2));}
const raw=path.join(out,pilot?'pilot-raw':'raw');fs.mkdirSync(raw,{recursive:true});
async function run(config){
 const worker=makeBrowserWorker(pathToFileURL(path.join(snapshot,'src','simulation-worker.js')).href);
 const start=performance.now();let phase='init',partial=null;
 return await new Promise(resolve=>{
  let settled=false;
  const finish=async row=>{if(settled)return;settled=true;clearTimeout(timer);await worker.terminate();resolve({...config,...row,wallMs:performance.now()-start});};
  const timer=setTimeout(()=>finish({status:'watchdog-timeout',budgetMs:watchdogMs,total:config.fleetSize*config.ordersPerRobot,lastObserved:partial}),watchdogMs);
  worker.on('error',error=>finish({status:'error',error:error.stack}));
  worker.on('message',m=>{
   if(m.type==='error')return finish({status:'error',error:m.message,stack:m.stack});
   if(m.type==='state'){
    partial=m.summary;
    if(phase==='init'&&!m.busy){phase='run';worker.postMessage({type:'step',steps:1000000});}
    else if(phase==='run'&&!m.busy){phase='export';worker.postMessage({type:'export'});}
   }else if(m.type==='export'){
    try{
     const d=m.data;
     const world=makeWorld(config.fleetSize,{layout:d.configuration.warehouseLayout});
     const workload={world:{width:world.width,height:world.height,walls:[...world.walls].sort(),homes:world.homes},orders:d.orders.map(({id,pickup,dropoff,deadline,created,challenge})=>({id,pickup,dropoff,deadline,created,challenge})),positions:d.trace[0].positions,batteries:d.trace[0].batteries};
     const data=JSON.stringify(d);fs.writeFileSync(path.join(raw,key(config)+'.json.gz'),gzipSync(data));
     finish({status:'measured',...d.summary,workloadHash:hash(JSON.stringify(workload)),validation:validateExecution(d),rawFile:path.relative(out,path.join(raw,key(config)+'.json.gz'))});
    }catch(error){finish({status:'error',error:error.stack});}
   }
  });
  worker.postMessage({type:'init',config});
 });
}
for(const [i,c] of cases.entries()){
 if(results.some(r=>key(r)===key(c)))continue;
 const row=await run(c);results.push(row);
 fs.writeFileSync(path.join(out,filename),JSON.stringify(results,null,2));
 console.log(`${i+1}/${cases.length} ${key(c)}: ${row.status} onTime=${row.onTime??'?'} total=${row.total??'?'} waiting=${row.waiting??'?'} wall=${(row.wallMs/1000).toFixed(2)}s`);
}
meta.finished=new Date().toISOString();meta.count=results.length;
fs.writeFileSync(path.join(out,pilot?'pilot-metadata.json':'metadata.json'),JSON.stringify(meta,null,2));
console.log('COMPLETE '+out);
