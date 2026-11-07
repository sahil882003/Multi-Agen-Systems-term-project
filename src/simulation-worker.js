import { Simulation } from './sim.js';
import { setPlanningProgressObserver } from './robot-agent.js';
import {stlLogPage} from './stl-log.js';
let workerSimulation;
let sentWorld, sentSolver;
let junctionPlanner;
let guidedPlanner;
function workerSnapshot(busy=false) {
  const summary=workerSimulation.summary();

  const state={...workerSimulation,trace:[],batches:[]};
  delete state.stlObservation; 
  if(state.world===sentWorld)delete state.world;
  else sentWorld=state.world;
  if(state.latestSolver===sentSolver)delete state.latestSolver;
  else sentSolver=state.latestSolver;
  return {type:'state',state,summary,busy};
}
function solveInWorker(request,signal){
  return new Promise((resolve,reject)=>{

    const guided=request.mode==='guided-local';
    const persistent=guided||request.type==='junction-negotiate'||request.mode==='junction-assignment';
    let holder=guided?guidedPlanner:persistent?junctionPlanner:null;
    if(!holder){
      const source=globalThis.__warehousePlannerSource;
      const url=source?URL.createObjectURL(new Blob([source],{type:'text/javascript'})):new URL('./planning-worker.js',import.meta.url);
      holder={planner:source?new Worker(url):new Worker(url,{type:'module'}),url,blob:!!source};
      if(guided)guidedPlanner=holder;
      else if(persistent)junctionPlanner=holder;
    }
    const {planner}=holder;let settled=false;
    const finish=(error,result)=>{
      if(settled)return;settled=true;
      signal.removeEventListener('abort',abort);planner.onmessage=planner.onerror=null;
      if(!persistent||error||signal.aborted){
        planner.terminate();if(holder.blob)URL.revokeObjectURL(holder.url);
        if(junctionPlanner===holder)junctionPlanner=null;
        if(guidedPlanner===holder)guidedPlanner=null;
      }
      error?reject(error):resolve(result);
    };
    const abort=()=>finish(null,null);
    signal.addEventListener('abort',abort,{once:true});
    planner.onmessage=({data})=>{
      if(data.type==='planning-progress')postMessage(data);
      else if(data.type==='error')finish(new Error(data.message));
      else if(data.type==='result')finish(null,data.result);
    };
    planner.onerror=e=>finish(new Error(e.message||'Planning worker could not complete the request.'));
    planner.postMessage(request);
  });
}
let handlingStep=false;
self.onmessage=async({data})=>{
  try {
    if(data.type==='init'){setPlanningProgressObserver(p=>postMessage({type:'planning-progress',progress:p}));sentWorld=sentSolver=undefined;workerSimulation=new Simulation(data.config);postMessage(workerSnapshot());}
    else if(data.type==='step'){
      if(handlingStep)return;handlingStep=true;
      try {
        for(let i=0;i<(data.steps??1)&&!workerSimulation.done;i++){
          if(typeof Worker==='function')await workerSimulation.stepAsync(solveInWorker,()=>postMessage(workerSnapshot(true)));
          else workerSimulation.step(); 
        }
        postMessage(workerSnapshot());
      }finally{handlingStep=false;}
    } else if(data.type==='stl-log')postMessage({type:'stl-log',requestId:data.requestId,data:stlLogPage(workerSimulation,data.options)});
    else if(data.type==='export')postMessage({type:'export',data:workerSimulation.export()});
  } catch(error){postMessage({type:'error',message:error.message,stack:error.stack});}
};
