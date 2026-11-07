import { Simulation, DEFAULT_ADMM_RHO } from "./sim.js";
import { same } from "./world.js";
import { PlaybackClock, renderStlResults, orderTimeLabel, renderStlLogRows } from './presentation.js';

const $ = (id) => document.getElementById(id);

const labels = {
  "baseline": "BFS baseline",
  "guided-local": "STL-guided search",
  "incremental": "Route-timing ADMM",
  "junction-assignment": "Crossing-allocation ADMM"
};
let sim = new Simulation({ scenario: "mixed", mode: "guided-local" }),
  running = false,
  selectedRobot = null,
  speed = 5;
const playback = new PlaybackClock(speed);
let renderedSolver, renderedSolverMode;
let lastListPaint=-Infinity;
let orderPage=0;
let pickupBadges=[];
let activePage='simulation',stlLogPageIndex=0,stlLogAnchor=null,stlLogPending=false,stlLogDirty=false;
let stlLogRequestId=0,lastStlLogTick=-1,lastStlLogRequest=-Infinity;
const ordersPerPage=50;
let planningProgress=null,progressReceivedAt=0,lastProgressSeconds=-1;
function renderPlanningProgress() {
  const admm=['incremental','junction-assignment'].includes(sim.mode);
  $("admm-settings").hidden=!admm;$("admm-progress").hidden=!admm;
  $("admm-fixed-rho").textContent=DEFAULT_ADMM_RHO;
  const p=planningProgress;
  const phases={assignment:'Optimizing crossing-slot allocation',junction:'Negotiating local passage slots',routes:'Preparing robot-local routes',constraints:'Building neighbor constraints',admm:'ADMM solving',
    'integer-repair':'Checking integer passage times',finished:sim.latestSolver?.status==='converged'?'Schedule converged and checked':'Coordination finished'};
  $("admm-phase").textContent=p?`${phases[p.phase]??p.phase} · ${(sim.mode==='junction-assignment')?'local event':'batch'} at tick ${p.batchTick}`:(sim.mode==='junction-assignment')?'Local ADMM ready · runs only on contention':'ADMM ready';
  $("cancel-planning").hidden=!(workerBusy&&p&&p.phase!=='finished');
  $("admm-iteration").textContent=p?.iteration??'—';
  const number=v=>Number.isFinite(v)?v.toExponential(3):'—';
  $("admm-primal").textContent=number(p?.maxPrimal);$("admm-dual").textContent=number(p?.maxDual);
  $("admm-violation").textContent=number(p?.violation);
  $("admm-ready").textContent=p?.readyAgents===undefined?'—':`${p.readyAgents} / ${p.activeAgents}`;
  $("admm-progress-note").textContent=p?.phase==='finished'
    ?`Solver ${p.status}. Planning latency consumes mission time. Robots check their own deadlines and obtain current peer permission before execution.`
    :p?.phase==='routes'?`Routes prepared: ${p.plannedAgents??0} / ${p.totalAgents}. Cancel & reset stops this run.`
    :`Until convergence; no numerical time or iteration cap. ${sim.mode==='junction-assignment'?'Assignment residual tolerances: 0.000001.':'All local tolerances: 0.001.'} Residuals can fluctuate; convergence time is unknown. Cancel & reset stops this run.`;
  if(p&&p.phase!=='finished')$("admm-progress-note").textContent+=` Mission clock continues: 1 tick = ${sim.settings.secondsPerTick} s; ${(sim.mode==='junction-assignment')?'negotiating robots hold; unrelated safe traffic continues':'committed robots keep executing'}.`;
  if(sim.mode==='incremental'&&p&&p.phase!=='finished')$("run-note").textContent=`Planning insertion for ${p.totalAgents} new missions around ${p.committedAgents} committed trajectories at tick ${p.batchTick}. Mission time and committed trajectories continue during this solve; Cancel & reset remains available.`;
  $("admm-progress-note").textContent+=` Penalty ρ = ${sim.settings.admmRho}.`;
  lastProgressSeconds=-1;renderPlanningClock(performance.now());
}
function renderPlanningClock(time) {
  const p=planningProgress;
  const seconds=p?Math.floor((p.elapsedMs+(workerBusy&&p.phase!=='finished'?Math.max(0,time-progressReceivedAt):0))/1000):0;
  if(seconds!==lastProgressSeconds){$("admm-elapsed").textContent=`${seconds} s`;lastProgressSeconds=seconds;}
}
let simulationWorker = null, workerBusy = false, exportResolve = null;
function makeSimulationWorker() {
  if(globalThis.__warehouseWorkerSource) return new Worker(URL.createObjectURL(new Blob([globalThis.__warehouseWorkerSource],{type:"text/javascript"})));
  return new Worker(new URL('./simulation-worker.js',import.meta.url),{type:'module'});
}
function startWorker(config) {
  simulationWorker?.terminate(); workerBusy=true;
  planningProgress=null;
  const worker=makeSimulationWorker();simulationWorker=worker;
  worker.onmessage=({data})=>{
    if(worker!==simulationWorker)return;
    if(data.type==='state'){
      globalThis.__warehouseStartupReady?.();
      const previousTick=sim.tick;
      sim=Object.assign(sim,data.state,{summary:()=>data.summary});
      playback.accept(performance.now(),running&&!sim.done&&sim.tick>previousTick);
      workerBusy=!!data.busy;update();
    }else if(data.type==='stl-log'){
      if(data.requestId!==stlLogRequestId)return;
      stlLogPending=false;
      if(stlLogDirty){stlLogDirty=false;requestStlLogs(true);return;}
      const page=data.data;
      stlLogPageIndex=page.page;lastStlLogTick=page.latestTick;
      $("stl-log-rows").innerHTML=renderStlLogRows(page.rows);
      $("stl-log-range").textContent=`RECORDS ${page.first}–${page.last} / ${page.total}`;
      $("stl-log-page").textContent=`Page ${page.page+1} / ${page.pages}`;
      $("stl-log-state").textContent=`${stlLogAnchor===null?'Live history':'Frozen history'} through tick ${page.asOfTick}`;
      $("stl-log-newer").disabled=page.page===0;$("stl-log-older").disabled=page.page>=page.pages-1;
      if(stlLogAnchor===null&&page.latestTick<sim.tick)requestStlLogs(sim.done);
    }else if(data.type==='planning-progress'){
      planningProgress=data.progress;progressReceivedAt=performance.now();renderPlanningProgress();
    }else if(data.type==='export'){exportResolve?.(data.data);exportResolve=null;}
    else if(data.type==='error'){workerBusy=false;running=false;toast(data.message);console.error(data.stack);update();globalThis.__warehouseStartupError?.(data.message);}
  };
  worker.onerror=e=>{workerBusy=false;running=false;const message=e.message||'The browser could not start the simulation worker.';toast(message);globalThis.__warehouseStartupError?.(message);};
  worker.postMessage({type:'init',config});
}
function requestWorkerSteps(steps=1){
  if(workerBusy||sim.done)return;workerBusy=true;
  simulationWorker.postMessage({type:'step',steps});

  $("step").disabled=true;$("export").disabled=true;
}
function workerExport(){return new Promise(resolve=>{exportResolve=resolve;simulationWorker.postMessage({type:'export'});});}

function requestStlLogs(force=false){
  if(activePage!=='results'||!simulationWorker)return;
  if(stlLogPending){if(force)stlLogDirty=true;return;}
  if(!force&&(stlLogAnchor!==null||lastStlLogTick===sim.tick||!sim.done&&performance.now()-lastStlLogRequest<500))return;
  stlLogPending=true;lastStlLogRequest=performance.now();
  simulationWorker.postMessage({type:'stl-log',requestId:++stlLogRequestId,options:{page:stlLogPageIndex,pageSize:30,
    specification:$("stl-log-filter").value,asOfTick:stlLogAnchor}});
}
function showPage(){
  activePage=globalThis.location?.hash==='#results'?'results':'simulation';
  $("overview").hidden=activePage!=='simulation';$("results-view").hidden=activePage!=='results';
  $("simulation-link").setAttribute('aria-current',activePage==='simulation'?'page':'false');
  $("results-link").setAttribute('aria-current',activePage==='results'?'page':'false');
  update(true);drawResiduals();requestStlLogs(true);
}
$("stl-log-filter").onchange=()=>{stlLogPageIndex=0;stlLogAnchor=null;requestStlLogs(true);};
$("stl-log-latest").onclick=()=>{stlLogPageIndex=0;stlLogAnchor=null;requestStlLogs(true);};
$("stl-log-newer").onclick=()=>{stlLogPageIndex=Math.max(0,stlLogPageIndex-1);requestStlLogs(true);};
$("stl-log-older").onclick=()=>{stlLogAnchor??=sim.tick;stlLogPageIndex++;requestStlLogs(true);};
window.addEventListener('hashchange',showPage);

function toast(message) {
  $("toast").textContent = message;
  $("toast").style.display = "block";
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => ($("toast").style.display = "none"), 3200);
}

function reset() {
  running = false;
  stlLogPageIndex=0;stlLogAnchor=null;stlLogPending=false;stlLogDirty=false;lastStlLogTick=-1;
  $("stl-log-rows").innerHTML='';
  if (Number($("fleet-size").value) > 20 && $("warehouse-layout").value === "standard") $("warehouse-layout").value = "scaled";
  const config = {
    fleetSize: Number($("fleet-size").value),
    ordersPerRobot: Number($("orders-per-robot").value),
    warehouseLayout: $("warehouse-layout").value,
    mode: sim.mode,
    scenario: "mixed",
    congestion: $("congestion").value,
    admmUntilConverged:true,
    admmRho:DEFAULT_ADMM_RHO,
    seed: Math.max(0, Math.min(4294967295, Number($("seed").value) || 0)),
  };
  sim = new Simulation(config);
  startWorker(config);
  selectedRobot = null;
  orderPage=0;
  playback.accept(performance.now(),false);
  update();
}

function setRunning(value) {
  if (sim.done && value) reset();
  running = value;
  if(!running)playback.accept(performance.now(),false);
  update();
}
$("play").onclick = () => setRunning(!running);
$("step").onclick = () => {
  running = false;
  requestWorkerSteps();
  update();
};
$("reset").onclick = reset;
$("fleet-size").onchange = reset;
$("orders-per-robot").onchange = reset;
$("warehouse-layout").onchange = reset;
$("congestion").onchange = reset;

$("seed").onchange = reset;
$("cancel-planning").onclick=()=>{reset();toast('Planning cancelled. This run has been reset.');};
$("orders-previous").onclick=()=>{orderPage=Math.max(0,orderPage-1);update(true);};
$("orders-next").onclick=()=>{orderPage=Math.min(Math.ceil(sim.orders.length/ordersPerPage)-1,orderPage+1);update(true);};
$("speed").oninput = (e) => {
  speed = Number(e.target.value);
  playback.speed = speed;
  $("speed-value").textContent = `${speed}×`;
};
$("paths").onchange = () => drawMap(playback.fraction(performance.now()));
document.querySelectorAll("[data-mode]").forEach(
  (button) =>
    (button.onclick = () => {
      sim.mode = button.dataset.mode;
      reset();
    }),
);
let exportUrl = null,
  exportText = "";

function download(name, text, type) {
  exportText = text;
  if (exportUrl) URL.revokeObjectURL(exportUrl);
  exportUrl = URL.createObjectURL(new Blob([text], { type }));
  $("download-export").href = exportUrl;
  $("download-export").download = name;
  $("export-title").textContent = name;
  $("export-content").value = text.slice(0, 100000);
  $("export-hint").textContent =
    text.length > 100000
      ? "Preview shortened for responsiveness. Download file and Copy contents both include the complete data."
      : "Your data is ready. Download the file or copy its contents.";
  $("export-dialog").showModal();
}
$("close-export").onclick = () => $("export-dialog").close();
$("copy-export").onclick = async () => {
  try {
    await navigator.clipboard.writeText(exportText);
    toast("Contents copied.");
  } catch {
    $("export-content").value = exportText;
    $("export-hint").textContent =
      "Full contents selected. Use your browser’s copy command.";
    $("export-content").focus();
    $("export-content").select();
    toast("Text selected. Use your browser’s copy command.");
  }
};
$("export").onclick = async () => {
  running = false;
  update();
  download(
    `warehouse-${sim.mode}-seed${sim.seed}-tick${sim.tick}.json`,
    JSON.stringify(await workerExport(), null, 2),
    "application/json",
  );
};
function update(forceLists=false) {
  renderPlanningProgress();
  $("export").disabled=workerBusy;
  const s = sim.summary();
  $("results-context").textContent=`${labels[sim.mode]} · seed ${sim.seed} · ${sim.fleetSize} robots · ${sim.orders.length} orders · tick ${sim.tick}${running&&!sim.done?' · simulation running':''}`;
  const carrying=new Map(sim.robots.map(r=>[r.id,r.carrying])),pickupCounts=new Map();
  for(const order of sim.orders) {
    if(order.status!=='queued'&&!(order.status==='assigned'&&!carrying.get(order.robot)))continue;
    const key=`${order.pickup.x},${order.pickup.y}`;
    if(!pickupCounts.has(key))pickupCounts.set(key,{point:order.pickup,count:0});
    pickupCounts.get(key).count++;
  }
  pickupBadges=[...pickupCounts.values()];
  $("map-size-tag").textContent = `${sim.world.width} × ${sim.world.height} GRID`;
  $("warehouse").setAttribute(
    "aria-label",
    `Animated warehouse with ${sim.fleetSize} robots, shelves, four dispatch stations and private charging bays. Numbered pickup badges show orders awaiting collection at ${pickupBadges.length} pickup points. ${pickupBadges.map(b=>`${b.count} orders at (${b.point.x}, ${b.point.y})`).join('; ')}.`,
  );
  $("play").textContent = sim.done
    ? "↻ Run again"
    : running
      ? "Ⅱ Pause"
      : "▶ Start run";
  $("step").disabled = sim.done || workerBusy;
  const planning = workerBusy && sim.robots.every(r=>!r.plan);
  $("run-badge").textContent = planning ? "● PLANNING" : sim.done
    ? sim.stopReason
      ? "● STOPPED"
      : "● COMPLETE"
    : running
      ? "● RUNNING"
      : sim.tick
        ? "● PAUSED"
        : "● READY";
  $("run-badge").className =
    "run-badge" +
    (running ? " running" : sim.done && sim.stopReason ? " stopped" : "");
  document
    .querySelectorAll("[data-mode]")
    .forEach((b) =>
      b.classList.toggle("selected", b.dataset.mode === sim.mode),
    );
  $("metric-delivered").innerHTML =
    `${s.delivered} <small>/ ${s.total}</small>`;
  $("metric-delivered-sub").textContent = s.rejected
    ? `${s.rejected} rejected · all orders still accounted for`
    : `${s.total - s.delivered} not yet delivered`;
  $("metric-ontime").textContent = s.onTime;
  $("metric-deadline").textContent =
    `${s.deadlineViolations} missed · ${s.pending} obligations pending`;
  $("metric-wait").innerHTML = `${s.waiting} <small>ticks</small>`;
  $("metric-collision").textContent = s.collisions + s.edgeConflicts;
  $("metric-safety").textContent =
    `${s.collisions} cell · ${s.edgeConflicts} swept-motion`;
  $("tick").textContent = String(sim.tick).padStart(3, "0");
  const value=n=>n===null||n===undefined?'—':Number(n).toFixed(1);
  $("benchmark-state").textContent=sim.done?'FINAL RUN AGGREGATES':'LIVE RUN AGGREGATES';
  $("benchmark-ontime").textContent=`${value(s.onTimeRate*100)}%`;
  $("benchmark-ontime-note").textContent=`${s.onTime} / ${s.total} generated orders`;
  $("benchmark-delivered").textContent=`${s.delivered} / ${s.total}`;
  $("benchmark-outcomes").textContent=`${s.rejected} rejected · ${s.incomplete} incomplete · ${s.lateDelivered} delivered late`;
  $("benchmark-mean").textContent=value(s.meanDeliveryTime);
  $("benchmark-median").textContent=value(s.medianDeliveryTime);
  $("benchmark-p90").textContent=value(s.p90DeliveryTime);
  $("benchmark-wait").textContent=s.waiting;
  $("benchmark-wait-note").textContent=`${s.blockedWaiting} blocked + ${s.scheduledWaiting} scheduled + ${s.serviceWaiting} service`;
  $("benchmark-throughput").textContent=value(s.throughputPer100Ticks);
  $("benchmark-planning").textContent=`${(s.planningMs/1000).toFixed(2)} s`;

  const modelNotes = {
    baseline: "Fleet-batch dispatch: new missions wait until active robots finish their delivery and return. Spatial BFS routes use reactive occupancy checks and rerouting after blockage. No consensus algorithm.",
    "guided-local": "Individual dispatch with timed search around committed traffic. Cached distances around shelves, separation-margin guidance, and deadline/return-energy bounds guide each new route. Travelling routes stay fixed.",
    incremental: "Available robots accept missions without a fleet barrier. Committed routes stay fixed; new missions use local ADMM timing. Charging ends once a mission plus reserve is affordable.",
    "junction-assignment": "Peer ADMM chooses robot-to-slot assignments using reported battery, deadline slack and blocking age. Peers extract and verify unique slots. No fixed urgency order; cycle-clearance constraints remain. Only the current slot-zero grant executes. Waiting consumes zero battery, so low charge is a preference, not predicted depletion.",
  };
  $("run-note").textContent =
    (sim.stopReason ? sim.stopReason + (['incremental','junction-assignment'].includes(sim.mode)&&sim.latestSolver
      ? ` Solver: ${sim.latestSolver.status}, ${sim.latestSolver.iterations} numerical rounds. This is a coordination failure, not a frozen display.`:'') : null) ??
    (planning ? `Planning missions for ready robots around travelling commitments. Planning latency advances mission time and deadlines; Reset cancels this run.` : sim.done
      ? `Run finished. ${s.delivered} delivered; ${s.rejected} rejected; ${s.pending} deadline obligations still pending at termination.`
      : modelNotes[sim.mode]);
  if((sim.mode==='junction-assignment')){const j=s.junction;$("run-note").textContent+=` Local contention groups: ${j.groups}; route/yield repairs: ${j.recoveries}; stale grants: ${j.staleGrants}; local approach/yield messages: ${j.approachMessages}. Later slots are tentative; each accepted grant lasts one tick.`;}
  if(sim.mode==='junction-assignment'&&sim.latestSolver?.assignmentRows?.length){const rows=sim.latestSolver.assignmentRows.slice(0,6);$("run-note").textContent+=` Latest peer slots: ${rows.map(r=>`R${r.id+1}: slot ${r.slot}, battery ${(100*r.battery).toFixed(1)}%`).join('; ')}. Discrete extraction gap: ${(sim.latestSolver.extractionGap??0).toFixed(5)}.`;}

  $("run-note").textContent+=` Overlapping dispatches: ${s.overlappingDispatches}; missions started before full charge: ${s.partialChargeDispatches}.`;

  if(forceLists||!running||sim.done||performance.now()-lastListPaint>=125){
  lastListPaint=performance.now();
  $("specs").innerHTML=renderStlResults(s,sim.settings,sim.orders,sim.done);
  $("stl-scope").textContent=`Observed ticks 0–${sim.tick}${sim.done?' · run ended':''}. Safety and battery values are historical minima, not current readings.`;
  orderPage=Math.min(orderPage,Math.max(0,Math.ceil(sim.orders.length/ordersPerPage)-1));
  const orderStart=orderPage*ordersPerPage,orderEnd=Math.min(sim.orders.length,orderStart+ordersPerPage);
  $("order-range").textContent=`ORDERS ${orderStart+1}–${orderEnd} / ${sim.orders.length}`;
  $("queue-pages").hidden=sim.orders.length<=ordersPerPage;
  $("orders-previous").disabled=orderPage===0;
  $("orders-next").disabled=orderEnd>=sim.orders.length;
  $("orders").innerHTML = sim.orders.slice(orderStart,orderEnd)
    .map(
      (o) =>
        `<tr title="${o.failure ?? ""}"><td>ORD-${String(o.id).padStart(2, "0")}</td><td>${o.robot === null ? "—" : `R${o.robot + 1}`}</td><td>${String.fromCharCode(65 + sim.world.docks.findIndex((d) => same(d, o.dropoff)))}</td><td>t = ${o.deadline}</td><td><span class="status-pill ${o.done !== null && o.done > o.deadline ? "late" : o.status}">${o.done !== null && o.done > o.deadline ? "late" : o.status}</span></td><td>${orderTimeLabel(o,sim.tick)}</td></tr>`,
    )
    .join("");
  $("events").innerHTML = sim.events
    .slice(0, 35)
    .map(
      (e) =>
        `<div class="event ${e.kind}"><time>${String(e.tick).padStart(3, "0")}</time><span>${e.text}</span></div>`,
    )
    .join("");
  $("event-retention").textContent=`Showing newest ${Math.min(35,sim.events.length)} of ${sim.eventCount} events. JSON retains the latest ${sim.events.length} (limit 150); the execution trace remains available.`;
  }

  Array.from($("orders").rows).forEach((row,i)=>{
    const order=sim.orders[orderPage*ordersPerPage+i];
    if(!order)return;
    const label=orderTimeLabel(order,sim.tick),cell=row.cells[5];
    if(cell.textContent!==label)cell.textContent=label;
  });
  $("convergence-panel").hidden=!['incremental','junction-assignment'].includes(sim.mode);
  $("diagnostics-row").classList.toggle('single-column',$("convergence-panel").hidden);
  if(renderedSolver!==sim.latestSolver || renderedSolverMode!==sim.mode){
    drawResiduals();renderedSolver=sim.latestSolver;renderedSolverMode=sim.mode;
  }
  $("warehouse").style.aspectRatio = `${sim.world.width} / ${sim.world.height}`;
  drawMap(playback.fraction(performance.now()));
  requestStlLogs();
}

function prep(canvas) {
  const rect = canvas.getBoundingClientRect();
  if (!rect.width || !rect.height) return null;
  const dpr = devicePixelRatio || 1;
  if (
    canvas.width !== Math.round(rect.width * dpr) ||
    canvas.height !== Math.round(rect.height * dpr)
  ) {
    canvas.width = Math.round(rect.width * dpr);
    canvas.height = Math.round(rect.height * dpr);
  }
  const c = canvas.getContext("2d");
  c.setTransform(dpr, 0, 0, dpr, 0, 0);
  c.clearRect(0, 0, rect.width, rect.height);
  return { c, w: rect.width, h: rect.height };
}

function round(c, x, y, w, h, r, fill, stroke) {
  c.beginPath();
  c.roundRect(x, y, w, h, r);
  if (fill) {
    c.fillStyle = fill;
    c.fill();
  }
  if (stroke) {
    c.strokeStyle = stroke;
    c.stroke();
  }
}

function drawMap(alpha = 1) {
  const info = prep($("warehouse"));
  if (!info) return;
  const { c, w, h } = info,
    world = sim.world;
  const size = Math.min(w / world.width, h / world.height),
    ox = (w - size * world.width) / 2,
    oy = (h - size * world.height) / 2;
  const xy = (p) => ({
    x: ox + (p.x + 0.5) * size,
    y: oy + (p.y + 0.5) * size,
  });
  c.strokeStyle = "#303c50";
  c.lineWidth = 0.5;
  for (let x = 1; x < world.width; x++) {
    c.beginPath();
    c.moveTo(ox + x * size, oy + size);
    c.lineTo(ox + x * size, oy + (world.height - 1) * size);
    c.stroke();
  }
  for (let y = 1; y < world.height; y++) {
    c.beginPath();
    c.moveTo(ox + size, oy + y * size);
    c.lineTo(ox + (world.width - 1) * size, oy + y * size);
    c.stroke();
  }
  c.fillStyle = "#8998af";
  c.font = `${Math.max(6, size * 0.24)}px ui-monospace,monospace`;
  c.textAlign = "center";
  for (let x = 2; x < world.width - 1; x += 2)
    c.fillText(
      String(x).padStart(2, "0"),
      ox + (x + 0.5) * size,
      oy + size * 0.58,
    );
  for (const s of world.shelves) {
    round(
      c,
      ox + s.x * size + 1,
      oy + s.y * size + 4,
      s.w * size - 2,
      s.h * size - 2,
      4,
      "#182131",
    );
    round(
      c,
      ox + s.x * size + 1,
      oy + s.y * size + 1,
      s.w * size - 2,
      s.h * size - 2,
      4,
      "#364359",
      "#56657c",
    );
    for (let row = 0; row < 3; row++)
      for (let col = 0; col < 3; col++)
        round(
          c,
          ox + (s.x + col) * size + size * 0.13,
          oy + (s.y + row) * size + size * 0.16,
          size * 0.72,
          size * 0.62,
          2,
          (row + col) % 2 ? "#3e4d64" : "#46566d",
        );
    c.fillStyle = "#d0d9e7";
    c.font = `600 ${size * 0.28}px ui-monospace,monospace`;
    c.fillText(s.label, ox + (s.x + 1.5) * size, oy + (s.y + 1.58) * size);
  }
  for (let i = 0; i < world.docks.length; i++) {
    const p = world.docks[i],
      q = xy(p);
    round(
      c,
      q.x - size * 0.48,
      q.y - size * 0.4,
      size * 0.96,
      size * 0.8,
      3,
      "#293f5d",
      "#7598c7",
    );
    c.fillStyle = "#c0d6f3";
    c.font = `600 ${size * 0.3}px ui-monospace,monospace`;
    c.fillText(String.fromCharCode(65 + i), q.x, q.y + size * 0.12);
    c.fillStyle = "#9eafc8";
    c.font = `${size * 0.2}px ui-monospace,monospace`;
    c.fillText(
      "DISPATCH",
      q.x + size * (p.x > world.width / 2 ? -1.65 : 1.65),
      q.y + size * 0.1,
    );
  }
  for (let i = 0; i < world.homes.length; i++) {
    const p = xy(world.homes[i]);
    round(
      c,
      p.x - size * 0.45,
      p.y - size * 0.42,
      size * 0.9,
      size * 0.84,
      3,
      "#403d37",
      "#9f8b64",
    );
    c.fillStyle = "#e1c78f";
    c.font = `${size * 0.4}px sans-serif`;
    c.fillText("ϟ", p.x, p.y + size * 0.13);
    c.fillStyle = "#9eafc8";
    c.font = `${size * 0.2}px ui-monospace,monospace`;
    c.fillText(`BAY ${i + 1}`, p.x, p.y - size * 0.65);
  }
  if ($("paths").checked)
    for (const r of sim.robots)
      if (r.plan && (selectedRobot === null || selectedRobot === r.id)) {
        c.beginPath();
        r.plan.points.slice(r.index).forEach((p, i) => {
          const q = xy(p);
          if (i) c.lineTo(q.x, q.y);
          else c.moveTo(q.x, q.y);
        });
        c.strokeStyle = r.color + (selectedRobot === r.id ? "b0" : "40");
        c.lineWidth = Math.max(1, size * 0.06);
        c.setLineDash([3, 4]);
        c.stroke();
        c.setLineDash([]);
      }
  for (const r of sim.robots) {
    const start = xy(r.previous),
      end = xy(r.pos),
      p = {
        x: start.x + (end.x - start.x) * alpha,
        y: start.y + (end.y - start.y) * alpha,
      };
    if (r.id === selectedRobot) {
      c.beginPath();
      c.arc(p.x, p.y, size * 0.61, 0, Math.PI * 2);
      c.strokeStyle = r.color + "88";
      c.lineWidth = 1.5;
      c.stroke();
    }
    c.shadowColor = r.color + "30";
    c.shadowBlur = 0;
    round(
      c,
      p.x - size * 0.36,
      p.y - size * 0.36,
      size * 0.72,
      size * 0.72,
      size * 0.19,
      r.color,
    );
    c.shadowBlur = 0;
    c.fillStyle = "#1d293b";
    c.textAlign = "center";
    c.font = `700 ${size * 0.31}px ui-monospace,monospace`;
    c.fillText(String(r.id + 1), p.x, p.y + size * 0.11);
    if (r.carrying)
      round(
        c,
        p.x + size * 0.16,
        p.y - size * 0.48,
        size * 0.27,
        size * 0.27,
        2,
        "#f9e2ae",
        "#997949",
      );
  }

  for(const badge of pickupBadges) {
    const p=xy(badge.point),radius=Math.max(7,Math.min(12,size*.27));
    const x=p.x+size*.24,y=p.y-size*.24,label=String(badge.count);
    c.font=`700 ${Math.max(9,Math.min(13,size*.34))}px ui-monospace,monospace`;
    const width=Math.max(radius*2,c.measureText(label).width+8);
    round(c,x-width/2,y-radius,width,radius*2,radius,'#e3bb7d','#f4dfba');
    c.fillStyle='#273244';c.textAlign='center';c.textBaseline='middle';
    c.fillText(label,x,y+.5);
  }
  c.textBaseline='alphabetic';c.textAlign = "left";
}

function drawResiduals() {
  const info = prep($("residuals"));
  if (!info) return;
  const { c, w, h } = info,
    solver = sim.latestSolver,
    history = solver?.history ?? [];
  $("solver-status").textContent = solver
    ? solver.status.toUpperCase()
    : "NOT RUN";
  c.font = "10px -apple-system,sans-serif";
  c.fillStyle = "#9aa79e";
  if (!history.length) {
    c.textAlign = "center";
    c.fillText(
      ['incremental','junction-assignment'].includes(sim.mode)
        ? "Start the run to see numerical convergence."
        : "Select an ADMM mode to inspect the solver.",
      w / 2,
      h / 2,
    );
    c.textAlign = "left";
    $("solver-meta").textContent =
      solver?.reason ??
      "Solver history comes from actual numerical iterations, not simulated chart data.";
    return;
  }

  const left = 42,
    right = w - 20,
    top = 25,
    bottom = h - 27,
    max = 3,
    min = -8;
  c.strokeStyle = "#edf1eb";
  c.font = "8px ui-monospace,monospace";
  for (const power of [2, 0, -2, -4, -6, -8]) {
    const y = top + ((max - power) / (max - min)) * (bottom - top);
    c.beginPath();
    c.moveTo(left, y);
    c.lineTo(right, y);
    c.stroke();
    c.fillStyle = "#9eaaa1";
    c.fillText(`1e${power}`, 8, y + 3);
  }
  for (const [field, color] of [
    ["primal", "#2e9c79"],
    ["dual", "#9296d7"],
  ]) {
    c.beginPath();
    history.forEach((v, i) => {
      const x = left + (i / Math.max(1, history.length - 1)) * (right - left),
        y =
          top +
          ((max -
            Math.max(
              min,
              Math.min(max, Math.log10(Math.max(v[field], 1e-12))),
            )) /
            (max - min)) *
            (bottom - top);
      if (i) c.lineTo(x, y);
      else c.moveTo(x, y);
    });
    c.strokeStyle = color;
    c.lineWidth = 1.7;
    c.stroke();
  }
  c.fillStyle = "#2e9c79";
  c.fillText("— primal", left, 13);
  c.fillStyle = "#9296d7";
  c.fillText("— dual", left + 70, 13);
  c.fillStyle = "#94a195";
  c.fillText(String(history[0]?.iteration??1), left, h - 10);
  c.fillText(
    `${solver.iterations??history.length} iterations`,
    Math.max(left, right - 77),
    h - 10,
  );
  const central = solver.central,
    delta = central ? Math.abs(solver.objective - central.objective) : null;
  $("solver-meta").textContent =
    `Latest ${(sim.mode==='junction-assignment')?'local negotiation':'batch'} ${solver.batchIndex ?? "—"}: ${solver.iterations??history.length} iterations${solver.historyTruncated?' (latest 2000 plotted)':''} · ${solver.messages} messages${delta === null ? "" : ` · objective difference vs centralized: ${delta.toExponential(1)}`} · Run total: ${sim.summary().admmMessages} messages across ${sim.summary().solverBatches} batches. Log scale.`;
}

function frame(time) {
  if(workerBusy&&planningProgress)renderPlanningClock(time);
  if (running && !sim.done) {
    if(playback.canStep(time,workerBusy))requestWorkerSteps(1);
    drawMap(playback.fraction(time));
  }
  requestAnimationFrame(frame);
}
window.addEventListener("resize", () => {
  drawMap(playback.fraction(performance.now()));
  drawResiduals();
});
reset();
showPage();
requestAnimationFrame(frame);
