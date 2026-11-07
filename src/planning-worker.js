import {negotiateBatterySlots} from './junction-assignment.js';
import { planBatch } from './planner.js';
import { setPlanningProgressObserver } from './robot-agent.js';
let cachedWorld,cachedWorldKey;
self.onmessage=({data:q})=>{
  try {
    if(q.mode==='guided-local'){
      const key=JSON.stringify([q.world.width,q.world.height,[...q.world.walls],q.world.homes,q.world.pickups,q.world.docks]);
      if(key!==cachedWorldKey){cachedWorld=q.world;cachedWorldKey=key;}
      else q.world=cachedWorld;
    }
    setPlanningProgressObserver(progress=>postMessage({type:'planning-progress',progress}));
    const result=q.type==='junction-negotiate'?negotiateBatterySlots(q.groups,q.settings,q.tick):planBatch(q.world,q.robots,q.jobs,q.mode,q.tick,q.settings);
    postMessage({type:'result',result});
  }catch(error){postMessage({type:'error',message:error.message,stack:error.stack});}
};
