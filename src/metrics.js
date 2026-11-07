// Read-only operational aggregates. Times use simulation ticks; planning wall
// time is reported separately. Undelivered orders never enter timing averages.
function deliveryQuantile(sorted, fraction) {
  if (!sorted.length) return null;
  const position=(sorted.length-1)*fraction,lower=Math.floor(position),upper=Math.ceil(position);
  return sorted[lower]+(sorted[upper]-sorted[lower])*(position-lower);
}
export function deliveryAggregates(orders, robots, tick) {
  const delivered=orders.filter(o=>o.done!==null),total=orders.length;
  const times=delivered.map(o=>o.done-o.created).sort((a,b)=>a-b);
  const late=delivered.filter(o=>o.done>o.deadline).map(o=>o.done-o.deadline);
  const onTime=delivered.filter(o=>o.done<=o.deadline).length;
  return {
    onTimeRate:total?onTime/total:0,
    deliveryRate:total?delivered.length/total:0,
    meanDeliveryTime:times.length?times.reduce((a,b)=>a+b,0)/times.length:null,
    medianDeliveryTime:deliveryQuantile(times,.5),
    p90DeliveryTime:deliveryQuantile(times,.9),
    lateDelivered:late.length,
    meanTardiness:late.length?late.reduce((a,b)=>a+b,0)/late.length:null,
    maxTardiness:late.length?Math.max(...late):null,
    workloadCompletionTime:total&&delivered.length===total?Math.max(...delivered.map(o=>o.done)):null,
    throughputPer100Ticks:tick>0?100*delivered.length/tick:0,
    waitingPerDeliveredOrder:delivered.length?robots.reduce((n,r)=>n+r.wait,0)/delivered.length:null,
    blockedWaiting:robots.reduce((n,r)=>n+(r.blockedWaiting??0),0),
    scheduledWaiting:robots.reduce((n,r)=>n+(r.scheduledWaiting??0),0),
    serviceWaiting:robots.reduce((n,r)=>n+(r.serviceWaiting??0),0),
  };
}
