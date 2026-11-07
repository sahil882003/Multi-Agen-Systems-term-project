const stlPrefixCache = new WeakMap();

function stlPrefixes(trace) {
  let cached = stlPrefixCache.get(trace);
  if (!cached || cached.length > trace.length) {
    cached = [];
    stlPrefixCache.set(trace, cached);
  }
  for (let i = cached.length; i < trace.length; i++) {
    const row = trace[i], previous = cached[i-1];
    cached.push({
      safety: previous && previous.safety <= row.separation ? previous.safety : row.separation,
      safetyAt: previous && previous.safety <= row.separation ? previous.safetyAt : i,
      battery: previous && previous.battery <= row.battery ? previous.battery : row.battery,
      batteryAt: previous && previous.battery <= row.battery ? previous.batteryAt : i,
    });
  }
  return cached;
}

const stlSigned = (value, digits=2) => Number.isFinite(value) ? `${value>=0?'+':''}${value.toFixed(digits)}` : '—';
const stlMarginVerdict = value => value < 0 ? 'Violated' : value === 0 ? 'At limit' : 'Satisfied';

export function stlLogPage(sim, options={}) {
  const {trace, orders, settings} = sim;
  const specifications = ['separation','battery','deadline'];
  const selected = specifications.includes(options.specification) ? [options.specification] : specifications;
  const size = Math.max(1, Math.min(100, Math.floor(Number(options.pageSize)||30)));
  const asOf = !trace.length ? -1 : options.asOfTick == null ? trace.length-1 : Math.max(0, Math.min(trace.length-1, Math.floor(Number(options.asOfTick)||0)));
  const total = Math.max(0, asOf+1)*selected.length;
  const pages = Math.max(1, Math.ceil(total/size));
  const page = Math.max(0, Math.min(pages-1, Math.floor(Number(options.page)||0)));
  const prefix = stlPrefixes(trace), rows=[];
  for (let offset=page*size; offset<Math.min(total,(page+1)*size); offset++) {
    const index = asOf-Math.floor(offset/selected.length), specification=selected[offset%selected.length];
    const sample=trace[index], historical=prefix[index];
    if (specification==='separation') {
      const witness=trace[historical.safetyAt];
      let closest=Infinity, pair=[];
      for(let i=0;i<witness.positions.length;i++)for(let j=i+1;j<witness.positions.length;j++){
        const distance=Math.hypot(witness.positions[i].x-witness.positions[j].x,witness.positions[i].y-witness.positions[j].y);
        if(distance<closest){closest=distance;pair=[i,j];}
      }
      rows.push({tick:sample.tick,specification,title:'Safe separation',verdict:stlMarginVerdict(historical.safety),
        formula:`G[0,${sample.tick}] distance ≥ ${settings.separation}`,
        observed:Number.isFinite(sample.separation)?`${(sample.separation+settings.separation).toFixed(2)} cells; required ${settings.separation.toFixed(2)}`:'No robot pair',
        margin:`${stlSigned(sample.separation)} cells`,robustness:`${stlSigned(historical.safety)} cells`,
        evidence:pair.length?`${pair.map(id=>`R${id+1}`).join(' / ')} at tick ${witness.tick}; this is the worst sampled separation so far.`:'No pair to compare.'});
    } else if (specification==='battery') {
      const witness=trace[historical.batteryAt],minimum=Math.min(...witness.batteries),id=witness.batteries.indexOf(minimum);
      rows.push({tick:sample.tick,specification,title:'Battery reserve',verdict:stlMarginVerdict(historical.battery),
        formula:`G[0,${sample.tick}] battery ≥ ${(settings.batteryMin*100).toFixed(0)}%`,
        observed:`${((sample.battery+settings.batteryMin)*100).toFixed(1)}% lowest; required ${(settings.batteryMin*100).toFixed(0)}%`,
        margin:`${stlSigned(sample.battery*100,1)} pp`,robustness:`${stlSigned(historical.battery*100,1)} pp`,
        evidence:`R${id+1} at tick ${witness.tick}; lowest recorded charge ${(minimum*100).toFixed(1)}%.`});
    } else {
      let onTime=0,late=0,overdue=0,pending=0;
      const misses=[];
      for(const order of orders){
        if(order.done!==null&&order.done<=sample.tick){
          if(order.done<=order.deadline)onTime++;
          else {late++;misses.push(`ORD-${String(order.id).padStart(2,'0')} delivered at ${order.done}, due ${order.deadline}`);}
        } else if(sample.tick>=order.deadline){
          overdue++;misses.push(`ORD-${String(order.id).padStart(2,'0')} undelivered at tick ${sample.tick}, due ${order.deadline}`);
        } else pending++;
      }
      rows.push({tick:sample.tick,specification,title:'Delivery deadlines',verdict:late+overdue?'Violated':pending?'Pending':'Satisfied',
        formula:'For every order: F[created, deadline] delivered',
        observed:`${onTime} on time · ${late} late · ${overdue} overdue · ${pending} pending`,
        margin:'Per-order completion slack',robustness:'Timestamp verdict',
        evidence:misses.length?misses.join('; '):pending?'Unexpired obligations remain pending; future delivery is not assumed.':'Every order completed by its deadline.'});
    }
  }
  return {rows,page,pages,pageSize:size,total,asOfTick:trace[asOf]?.tick??0,latestTick:sim.tick,
    first:total?page*size+1:0,last:Math.min(total,(page+1)*size)};
}
