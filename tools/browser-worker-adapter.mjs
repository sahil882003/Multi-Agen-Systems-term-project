import {Worker as Thread} from 'node:worker_threads';
async function browserWorkerBootstrap(target,delayPlanningMs=0){
 const {parentPort,Worker:Thread}=await import('node:worker_threads');
 const {resolveObjectURL}=await import('node:buffer');
 const bootstrap=browserWorkerBootstrap;
 class BrowserWorker {
  constructor(url){
   this.stopped=false;this.queue=[];
   const source=String(url).startsWith('blob:')?resolveObjectURL(String(url)).text().then(source=>({source})) : Promise.resolve({url:String(url)});
   source.then(target=>{
    if(this.stopped)return;
    this.thread=new Thread(`(${bootstrap.toString()})(${JSON.stringify(target)},${delayPlanningMs})`,{eval:true});
    this.thread.on('message',data=>this.onmessage?.({data}));
    this.thread.on('error',error=>this.onerror?.(error));
    for(const data of this.queue)this.thread.postMessage(data);this.queue=[];
   }).catch(error=>this.onerror?.(error));
  }
  postMessage(data){if(this.thread)this.thread.postMessage(data);else this.queue.push(data);}
  terminate(){this.stopped=true;return this.thread?.terminate();}
 }
 globalThis.self=globalThis;globalThis.Worker=BrowserWorker;
 globalThis.postMessage=data=>parentPort.postMessage(data);
 let ready=false;const queue=[];
 const deliver=data=>{
  if(delayPlanningMs&&data.world&&data.jobs)setTimeout(()=>self.onmessage({data}),delayPlanningMs);
  else self.onmessage({data});
 };
 parentPort.on('message',data=>ready?deliver(data):queue.push(data));
 if(target.source)await import('data:text/javascript;base64,'+Buffer.from(target.source).toString('base64'));
 else await import(target.url);
 ready=true;queue.forEach(deliver);
}
export function makeBrowserWorker(target,delayPlanningMs=0){
 return new Thread(`(${browserWorkerBootstrap.toString()})(${JSON.stringify(typeof target==='string'?{url:target}:target)},${delayPlanningMs})`,{eval:true});
}
