export function observeStl(sim, sample, closestRobots) {
  const state=sim.stlObservation;
  if (!state.safetyWitness || sample.separation<state.safetyWitness.margin)
    state.safetyWitness={tick:sample.tick,robotIds:closestRobots,margin:sample.separation,distance:sample.separation+sim.settings.separation};
  if (!state.batteryWitness || sample.battery<state.batteryWitness.margin) {
    const minimum=Math.min(...sample.batteries);
    state.batteryWitness={tick:sample.tick,robotId:sample.batteries.indexOf(minimum),margin:sample.battery,battery:minimum};
  }
  if(sample.separation<0 && !state.safetyViolated){
    state.safetyViolated=true;
    sim.log('stl',`STL safety violated: ${closestRobots.map(id=>`R${id+1}`).join(' / ')}; distance ${(sample.separation+sim.settings.separation).toFixed(2)} cells, required ${sim.settings.separation.toFixed(2)}.`,{specification:'separation',robotIds:closestRobots,margin:sample.separation});
  }
  if(sample.battery<0 && !state.batteryViolated){
    state.batteryViolated=true;
    const w=state.batteryWitness;
    sim.log('stl',`STL battery reserve violated: R${w.robotId+1}; ${(w.battery*100).toFixed(1)}% charge, ${(sim.settings.batteryMin*100).toFixed(0)}% required.`,{specification:'battery',robotId:w.robotId,margin:sample.battery});
  }
  for(const order of sim.orders){
    const missed=order.done===null?sample.tick>=order.deadline:order.done>order.deadline;
    if(!missed || state.deadlineSeen.has(order.id))continue;
    state.deadlineSeen.add(order.id);
    const label=`ORD-${String(order.id).padStart(2,'0')}`;
    sim.log('stl',`STL deadline violated: ${label}, due tick ${order.deadline}; ${order.done===null?'not delivered by deadline':`delivered late at tick ${order.done}`}.${order.status==='rejected'?' Planning outcome: rejected.':''}`,
      {specification:'deadline',orderId:order.id,robotId:order.robot??null,dueTick:order.deadline});
  }
}
