export class PlaybackClock {
  constructor(speed=5){this.speed=speed;this.started=null;this.duration=500/speed;}
  accept(now,animate=true){this.started=animate?now:null;this.duration=500/this.speed;}
  fraction(now){return this.started===null?1:Math.max(0,Math.min(1,(now-this.started)/this.duration));}
  canStep(now,busy){return !busy&&this.fraction(now)>=1;}
}

export function renderStlResults(s,settings,orders,done) {
  const verdict=margin=>margin<0?['Violated','violated']:margin===0?['At limit','limit']:['Satisfied','satisfied'];
  const badge=([label,kind])=>`<span class="stl-verdict ${kind}">${label}</span>`;
  const signed=(v,digits)=>`${v>=0?'+':''}${v.toFixed(digits)}`;
  const sw=s.stlSafetyWitness,bw=s.stlBatteryWitness;
  const safetyEvidence=sw?.robotIds?.length?`${sw.robotIds.map(id=>`R${id+1}`).join(' / ')} · tick ${sw.tick}`:'No robot pair observed';
  const batteryEvidence=bw?`R${bw.robotId+1} · tick ${bw.tick}`:'No battery observation';
  const late=s.lateDelivered??orders.filter(o=>o.done!==null&&o.done>o.deadline).length;
  const overdue=s.deadlineViolations-late;
  const example=orders.find(o=>o.done===null?s.ticks>=o.deadline:o.done>o.deadline);
  const deadlineVerdict=s.deadlineViolations?['Violated','violated']:s.pending?['Pending','pending']:['Satisfied','satisfied'];
  return `<div class="stl-card" data-spec="separation">
    <div class="stl-title"><strong>Safe separation</strong>${badge(verdict(s.safetyRobustness))}</div>
    <p><b>${(s.safetyRobustness+settings.separation).toFixed(2)} cells</b> minimum · ${settings.separation.toFixed(2)} required</p>
    <p>Margin ${signed(s.safetyRobustness,2)} cells</p><small>${safetyEvidence}</small>
    <code title="G means always: sampled robot distances must stay at or above the required separation throughout the observed interval.">G[0,t] (distance ≥ ${settings.separation})</code>
  </div><div class="stl-card" data-spec="battery">
    <div class="stl-title"><strong>Battery reserve</strong>${badge(verdict(s.batteryRobustness))}</div>
    <p><b>${((s.batteryRobustness+settings.batteryMin)*100).toFixed(1)}%</b> lowest charge · ${(settings.batteryMin*100).toFixed(0)}% reserve</p>
    <p>Margin ${signed(s.batteryRobustness*100,1)} percentage points</p><small>${batteryEvidence}</small>
    <code title="G means always: every sampled battery level must remain at or above the reserve during the observed interval.">G[0,t] (battery ≥ ${(settings.batteryMin*100).toFixed(0)}%)</code>
  </div><div class="stl-card" data-spec="deadline">
    <div class="stl-title"><strong>Delivery deadlines</strong>${badge(deadlineVerdict)}</div>
    <p><b>${s.deadlineViolations} total violations</b> = late + overdue undelivered</p>
    <p>${s.onTime} on time · ${late} delivered late</p><p>${overdue} overdue undelivered · ${s.pending} pending</p>
    ${example?`<small>Example miss: ORD-${String(example.id).padStart(2,'0')} · due tick ${example.deadline}${example.robot!==null?` · R${example.robot+1}`:''}</small>`:''}
    ${s.rejected?`<small>Planner rejected ${s.rejected}; included above by deadline status.</small>`:''}
    ${done&&s.pending?`<p class="stl-unresolved">Run ended with ${s.pending} unexpired delivery obligations unresolved.</p>`:''}
    <code title="F means eventually: each order must be delivered by its own deadline D, inclusive. Pending means its deadline has not passed and it has not been delivered.">For each order: F[0,D] delivered</code>
  </div>`;
}

// Deadlines are absolute mission ticks. Completion freezes the final slack.
export function orderTimeLabel(order,tick) {
  return order.done===null
    ? `${Math.max(0,order.deadline-tick)} ticks left`
    : `${order.deadline-order.done} ticks slack`;
}

export function renderStlLogRows(rows) {
  const escape = value => String(value).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  return rows.map(row=>{
    const kind={'At limit':'limit',Satisfied:'satisfied',Violated:'violated',Pending:'pending'}[row.verdict];
    const evidence=row.evidence.length>160?`<details><summary>${escape(row.evidence.slice(0,110))}…</summary>${escape(row.evidence)}</details>`:escape(row.evidence);
    return `<tr><td>${row.tick}</td><td><strong>${escape(row.title)}</strong><span class="stl-verdict ${kind}">${escape(row.verdict)}</span><code>${escape(row.formula)}</code></td><td>${escape(row.observed)}</td><td>${escape(row.margin)}</td><td>${escape(row.robustness)}</td><td>${evidence}</td></tr>`;
  }).join('');
}
