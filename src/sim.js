import {negotiateBatterySlots} from './junction-assignment.js';
import {peerJunctionRecovery,peerJunctionGrants} from './junction-peer.js';
import {admitPeerPlans,ownRetryTick} from './peer-protocol.js';
import { junctionFrame, junctionMoves, junctionAdmissionEnergy } from './junction.js';
import { observeStl } from "./stl-observer.js";
import { deliveryAggregates } from "./metrics.js";
import { makeWorld, makeOrders, rng, ROBOT_COLORS, SETTINGS, same, spatialMission, movementCount, distance } from "./world.js";
import { remainingCommitments } from "./incremental.js";
import { trajectoryChecks } from "./stl.js";
import { admissionEnergy } from "./incremental.js";
import { planBatch, sweptCollision } from "./planner.js";
import { always, predicate, evaluate } from "./stl.js";

export const DEFAULT_ADMM_RHO = 5;

export class Simulation {

  constructor({
    mode = "guided-local",
    scenario = "normal",
    seed = 7,
    fleetSize = 5,
    ordersPerRobot = 2,
    warehouseLayout = "standard",
    congestion = "medium",
    evaluationHorizon = null,
    secondsPerTick = 1,
    admmMaxRounds = 3000,
    admmRho = DEFAULT_ADMM_RHO,
    admmBudgetMs = 5000,
    admmUntilConverged = true,
  } = {}) {
    if(!["baseline","guided-local","incremental","junction-assignment"].includes(mode))throw new Error("Unsupported submission stage.");
    this.mode = mode;
    this.scenario = scenario;
    this.seed = Number(seed);
    if (!(Number(secondsPerTick)>0) || !Number.isFinite(Number(secondsPerTick))) throw new Error("secondsPerTick must be finite and positive");
    this.planningClockMs=0;this.reactivePlanningMs=0;this.planningTicks=0;this.stalePlanningAttempts=0;
    this.settings = { secondsPerTick:Number(secondsPerTick), timingModel:"latency-aware-v1", ...SETTINGS, admmMaxRounds, admmRho, admmBudgetMs, admmUntilConverged };
    if(mode==='guided-local')this.settings.stlGuidance='mission-bounds-v1';
    this.congestion = congestion;
    this.guardInterventions = 0;
    this.fleetSize = Number(fleetSize);
    this.ordersPerRobot = Number(ordersPerRobot);
    this.warehouseLayout = this.fleetSize > 20 && warehouseLayout === "standard" ? "scaled" : warehouseLayout;
    this.world = makeWorld(this.fleetSize, { layout: this.warehouseLayout });
    this.settings.horizon =
      600 *
      Math.max(1, this.fleetSize / 5) *
      (this.warehouseLayout === "scaled" ? this.world.scaleFactor : 1) *
      Math.max(1, this.ordersPerRobot / 2);
    if(evaluationHorizon!==null) this.settings.horizon=Number(evaluationHorizon);
    this.orders = makeOrders(
      this.world,
      this.seed,
      scenario,
      this.ordersPerRobot,
      congestion,
    );
    this.batchLimit = this.fleetSize;
    this.nextDispatchTick=0;this.overlappingDispatches=0;this.partialChargeDispatches=0;
    this.tick = 0;
    this.done = false;
    this.stopReason = null;
    const random = rng(seed);
    this.robots = this.world.homes.map((home, id) => ({
      id,
      ...(mode==='junction-assignment'?{slotAssignment:{drain:this.settings.drain,batteryMin:this.settings.batteryMin}}:{}),
      home: { ...home },
      pos: { ...home },
      previous: { ...home },
      battery:
        scenario === "mixed" && id % 5 === (Number(seed) >>> 0) % 5
          ? 0.15 + random() * 0.03
          : scenario === "battery" ? Math.min(0.95, 0.15 + id * 0.035) : 0.8 + random() * 0.18,
      color: ROBOT_COLORS[id],
      state: "idle",
      orderId: null,
      plan: null,
      index: 0,
      wait: 0,
      blockedWaiting: 0,
      scheduledWaiting: 0,
      serviceWaiting: 0,
      moves: 0,
      blocked: 0,
      delivered: 0,
      carrying: false,
    }));
    this.trace = [];
    this.events = [];
    this.stlObservation={safetyWitness:null,batteryWitness:null,safetyViolated:false,batteryViolated:false,deadlineSeen:new Set()};
    this.batches = [];
    this.eventCount = 0;
    this.latestSolver = null;
    this.junctionPending=[];this.junctionGrants=[];
    this.junctionStats={groups:0,recoveries:0,staleGrants:0,approachMessages:0,approachBytes:0};
    this.planningMs = 0;
    this.monitoringMs = 0;
    this.edgeConflicts = 0;
    this.collisions = 0;
    this.planningFailures = 0;
    this.baselineNoProgressTicks=0;
    this.baselineStall=null;
    this.log(
      "system",
      `Simulation ready. ${this.fleetSize} robots; common external task dispatch. Primary stages use local routing and message-based timing.`,
    );
    this.record();
  }
  get junctionMode(){return this.mode==='junction-assignment';}
  get baselineMode(){return this.mode==='baseline';}
  get eventDispatch(){return !this.baselineMode;}

  log(kind, text, details = {}) {
    this.eventCount++;
    this.events.unshift({ tick: this.tick, kind, text, ...details });
    if (this.events.length > 150) this.events.pop();
  }

  prepareDispatch() {
    if(this.baselineMode&&this.robots.some(r=>r.plan))return null;
    const jobs = [],
      available = this.robots.filter((r) => !r.plan && this.tick>=(r.peerRetryAfter??0) && (r.state === "idle" || r.state === "charging")),
      queue = this.orders.filter((o) => o.status === "queued");
    for (const order of queue) {
      if (!available.length || jobs.length >= this.batchLimit) break;
      available.sort(
        (a, b) =>
          Math.abs(a.pos.x - order.pickup.x) +
            Math.abs(a.pos.y - order.pickup.y) -
            (Math.abs(b.pos.x - order.pickup.x) +
              Math.abs(b.pos.y - order.pickup.y)) || a.id - b.id,
      );
      const chosen=available.findIndex(r=>{const required=(this.junctionMode?junctionAdmissionEnergy:admissionEnergy)(this.world,r,order,this.settings);return required!==null && r.battery+1e-9>=required;});
      if(chosen<0){
        for(const r of available)if(r.state==='idle')r.state='charging';
        continue;
      }
      const [robot]=available.splice(chosen,1);
      jobs.push({ robot, order });
    }
    if (!jobs.length) return;
    const committedRobotIds=this.robots.filter(r=>r.plan).map(r=>r.id);
    const dispatchBatteries=jobs.map(j=>({robotId:j.robot.id,battery:j.robot.battery,wasCharging:j.robot.state==='charging'}));

    return { tick:this.tick, committedRobotIds, dispatchBatteries, jobs,
      request:structuredClone({world:this.world,robots:this.junctionMode?jobs.map(j=>j.robot):this.robots,
        jobs,mode:this.mode,tick:this.tick,settings:this.settings}) };
  }

  assign() {
    const dispatch=this.prepareDispatch();if(!dispatch)return;
    const started=performance.now(),q=dispatch.request;
    const result=planBatch(q.world,q.robots,q.jobs,q.mode,q.tick,q.settings);
    result.solverMs=result.ms;result.ms=performance.now()-started;
    this.consumePlanningTime(result.ms);
    this.acceptDispatch(dispatch,result);
  }
  consumePlanningTime(ms,onTick=()=>{}) {
    this.planningClockMs+=Math.max(0,ms);
    const due=Math.floor(this.planningClockMs/(1000*this.settings.secondsPerTick));
    while(this.planningTicks<due&&!this.done){
      if(this.tick>=this.settings.horizon){this.stop("Evaluation horizon reached.");break;}
      this.planningTicks++;this.step(false);onTick();
    }
  }
  async stepAsync(solve,onTick=()=>{},now=()=>performance.now()) {
    if(this.done)return;
    const dispatch=this.prepareDispatch();
    if(!dispatch){if(this.junctionMode)await this.coordinateJunctionAsync(solve,onTick,now);if(!this.done)this.step(false);return;}
    const controller=new AbortController();const started=now();let last=started;
    const advance=()=>{
      const current=now();this.consumePlanningTime(current-last,onTick);last=current;
      if(this.done)controller.abort();
    };

    const timer=setInterval(advance,50);
    try {
      const result=await solve(dispatch.request,controller.signal);
      advance();if(result){result.solverMs=result.ms;result.ms=now()-started;}
      if(this.done){this.planningMs+=now()-started;this.log("plan","Planning cancelled at mission termination; elapsed computation remains charged.");}
      else this.acceptDispatch(dispatch,result);
    } finally {clearInterval(timer);controller.abort();}

    if(!this.done&&this.junctionMode)await this.coordinateJunctionAsync(solve,onTick,now);
    if(!this.done)this.step(false);
  }

  prepareJunction() {
    const started=performance.now();
    const recovery=peerJunctionRecovery(this.world,this.robots,this.orders,this.tick,this.settings,this.junctionPending);
    this.junctionStats.approachMessages+=recovery.messages;this.junctionStats.approachBytes+=recovery.bytes;
    this.junctionStats.recoveries+=recovery.count;
    if(recovery.count)this.log('yield',`${recovery.count} robot-owned routes repaired through local requests.`);
    let frame=junctionFrame(this.robots,this.orders,this.tick);
    const ms=performance.now()-started;
    this.planningMs+=ms;this.consumePlanningTime(ms);
    if(frame.tick!==this.tick)frame=junctionFrame(this.robots,this.orders,this.tick);
    this.junctionStats.approachMessages+=frame.probeMessages;this.junctionStats.approachBytes+=frame.probeBytes;
    return frame;
  }
  finishJunction(frame,result,ms) {
    this.junctionPending=[];this.planningMs+=ms;if(!result||this.done)return;
    const live=junctionFrame(this.robots,this.orders,this.tick);
    this.junctionStats.approachMessages+=live.probeMessages;this.junctionStats.approachBytes+=live.probeBytes;
    const grants=peerJunctionGrants(this.robots,this.orders,this.tick,result.decisions,frame.tick,result.solver.status);
    if(result.solver.status==='converged')this.junctionStats.staleGrants+=result.decisions.filter(d=>d.slot===0&&!grants.some(g=>g.id===d.id)).length;
    this.junctionGrants=grants;this.junctionStats.groups+=frame.groups.length;
    const solver={...result.solver,batchTick:frame.tick,batchIndex:this.batches.length+1};
    this.latestSolver=solver;
    this.batches.push({tick:this.tick,planningStartedTick:frame.tick,planningTicks:this.tick-frame.tick,
      ms,plans:[],solver,kind:'local-passage',decisions:result.decisions,acceptedGrants:grants});
  }
  async coordinateJunctionAsync(solve,onTick,now) {
    const frame=this.prepareJunction();if(this.done||!frame.groups.length)return;
    this.junctionPending=frame.groups.flatMap(g=>g.requests.map(p=>p.id));
    const controller=new AbortController(),started=now();let last=started;
    const advance=()=>{const current=now();this.consumePlanningTime(current-last,onTick);last=current;if(this.done)controller.abort();};
    const timer=setInterval(advance,50);
    try {
      const result=await solve({type:'junction-negotiate',groups:frame.groups,settings:this.settings,tick:frame.tick},controller.signal);
      advance();this.finishJunction(frame,result,now()-started);
    }finally{clearInterval(timer);controller.abort();this.junctionPending=[];}
  }
  activatePeerPlans(plans,coordinate){
    // Charge actual admission work as planning. If it crosses a mission tick,
    // owners recheck against a fresh epoch; no stale endorsement is installed.
    let admission;
    do{
      const epoch=this.tick,started=performance.now();
      admission=admitPeerPlans(this.world,this.robots,this.orders,plans,epoch,this.settings,{coordinate});
      const elapsed=performance.now()-started;this.planningMs+=elapsed;this.consumePlanningTime(elapsed);
      if(this.tick===epoch)return admission;
    }while(!this.done);
    return {...admission,plans:[],deferred:plans.map(p=>({orderId:p.orderId,reason:'Mission ended before activation.'}))};
  }
  acceptDispatch(dispatch,result) {
    if(this.done){this.planningMs+=result?.ms??0;return;}
    if(!result)return;
    const {committedRobotIds,dispatchBatteries,jobs}=dispatch;
    const incremental=this.eventDispatch;
    let elapsedTicks=this.tick-dispatch.tick;
    result.deferred??=[];
    // Never execute an old absolute schedule blindly. New robots have held at
    // their snapshot positions; travelling commitments have advanced unchanged.
    const commitments=remainingCommitments(this.robots),deferredAtActivation=[];
    let validation={ok:true,errors:[]};
    if(this.junctionMode){
      const admission=this.activatePeerPlans(result.plans,false);
      result.plans=admission.plans;deferredAtActivation.push(...admission.deferred);validation=admission.validation;
    }else if(this.mode==='incremental'){
      const admission=this.activatePeerPlans(result.plans,true);
      result.plans=admission.plans;deferredAtActivation.push(...admission.deferred);validation=admission.validation;
    }else{
      const expired=result.plans.some(p=>!this.baselineMode&&
        this.tick+p.deliveryIndex>this.orders.find(o=>o.id===p.orderId).deadline);
      const startsChanged=result.plans.some(p=>!same(p.points[0],this.robots.find(r=>r.id===p.robotId).pos));
      validation=this.baselineMode?{ok:true,errors:[]}:
        trajectoryChecks(this.world,this.robots,[...commitments,...result.plans],this.settings);
      if(expired||startsChanged||!validation.ok){
        deferredAtActivation.push(...result.plans.map(p=>({orderId:p.orderId,reason:'Schedule became stale during computation; retry from current tick.'})));
        result.plans=[];
      }
    }
    elapsedTicks=this.tick-dispatch.tick;
    if(deferredAtActivation.length){
      this.stalePlanningAttempts++;result.deferred.push(...deferredAtActivation);
      this.log('plan',`Activation accepted ${result.plans.length} safe missions; ${deferredAtActivation.length} stale missions remain queued.`);
    }
    if(this.mode==='incremental'&&result.solver){
      result.solver.admissionMessages=(result.solver.admissionMessages??0)+(validation.messages??0);
      result.solver.messages+=validation.messages??0;result.solver.messageBytes+=validation.messageBytes??0;
    }
    const attemptedBatchLimit = this.batchLimit;
    if(incremental){
      this.nextDispatchTick=this.tick+(result.deferred?.length?5:1);
      if(result.plans.length && committedRobotIds.length)this.overlappingDispatches++;
      this.partialChargeDispatches+=dispatchBatteries.filter(b=>b.wasCharging && b.battery<1 && result.plans.some(p=>p.robotId===b.robotId)).length;

      if(['timeout','repair-failed','validation-failed'].includes(result.solver?.status))
        this.log('warning','Insertion unresolved; new missions remain queued while committed robots continue.');
    }
    this.planningMs += result.ms;
    this.batches.push({
      tick: this.tick, planningStartedTick:dispatch.tick, planningTicks:elapsedTicks,
      deferredAtActivation, activationValidation:validation, activationStatus:result.plans.length?"accepted":"no-new-plans",
      attemptedBatchLimit,
      committedRobotIds,dispatchBatteries,
      nextBatchLimit: this.batchLimit,
      ms: result.ms, solverMs:result.solverMs??result.ms,
      explored: result.explored,
      plans: result.plans.map((p) => ({ ...p })),
      rejected: result.rejected,
      deferred: result.deferred ?? [],
      solver: result.solver
        ? {
            status: result.solver.status,
            failureKind: result.solver.failureKind ?? null,
            reason: result.solver.reason ?? null,
            iterations: result.solver.iterations ?? result.solver.history.length,
            messages: result.solver.messages,
            messageBytes: result.solver.messageBytes ?? 0,
            untilConverged:!!result.solver.untilConverged,
            elapsedMs:result.solver.elapsedMs??null,
            historyTruncated:!!result.solver.historyTruncated,
            historySamples:result.solver.history.length,
            retainedMessages: result.solver.messageLog?.length ?? 0,
            objective: result.solver.objective ?? null,
            centralStatus: result.solver.central?.status ?? null,
            centralObjective: result.solver.central?.objective ?? null,
          }
        : null,
    });
    if (result.solver) {
      this.latestSolver = {
        ...result.solver,
        batchIndex: this.batches.length,
        batchTick: this.tick,
      };
      this.log(
        "admm",
        `ADMM ${result.solver.status}; ${result.solver.iterations??result.solver.history.length} rounds, ${result.solver.messages} numerical messages.`,
      );
    }
    for (const rejected of result.rejected) {
      const order = this.orders.find((o) => o.id === rejected.orderId);
      order.status = "rejected";
      order.failure = rejected.reason;
      this.planningFailures++;
      this.log("warning", `Order ${order.id}: ${order.failure}`);
    }
    for (const item of result.deferred ?? []) {
      const order = this.orders.find(o => o.id === item.orderId);
      if(incremental){const owner=jobs.find(j=>j.order.id===item.orderId)?.robot;
        if(owner){const live=this.robots.find(r=>r.id===owner.id);live.peerRetryAfter=ownRetryTick(live,this.tick);}}
      order.deferrals = (order.deferrals ?? 0) + 1;
      this.log("plan", `Order ${order.id} deferred until the next dispatch attempt.`);
    }
    for (const plan of result.plans) {
      const robot = this.robots.find((r) => r.id === plan.robotId),
        order = this.orders.find((o) => o.id === plan.orderId);
      robot.plan = plan;
      robot.index = 0;
      robot.orderId = order.id;
      robot.state = "pickup";
      robot.blocked = 0;
      order.status = "assigned";
      order.robot = robot.id;
    }
    if(incremental && result.plans.length)this.log("dispatch",`Event dispatch: ${result.plans.length} new missions; ${committedRobotIds.length} travelling commitments unchanged.`);
    this.log(
      "plan",
      `Dispatch planned: ${result.plans.length} missions, ${result.explored} search states, ${result.ms.toFixed(1)} ms.`,
    );
  }

  step(dispatch=true) {
    if (this.done) return;
    if(dispatch)this.assign();

    if(dispatch&&this.junctionMode&&!this.done){
      const frame=this.prepareJunction();
      if(frame.groups.length){
        this.junctionPending=frame.groups.flatMap(g=>g.requests.map(p=>p.id));
        const started=performance.now(),result=negotiateBatterySlots(frame.groups,this.settings,frame.tick);
        const ms=performance.now()-started;this.consumePlanningTime(ms);this.finishJunction(frame,result,ms);
      }
    }
    if(this.done)return;
    for (const r of this.robots) r.previous = { ...r.pos };
    for (const r of this.robots)
      if (r.state === "charging") {
        r.battery = Math.min(1, r.battery + this.settings.chargeRate);
        if (r.battery >= 1) {
          r.state = "idle";
          this.log("battery", `R${r.id + 1} fully charged.`);
        }
      }
    if (
      this.robots.some(
        (r) =>
          r.plan &&
          r.index < r.plan.points.length - 1 &&
          !same(r.pos, r.plan.points[r.index + 1]) &&
          r.battery - this.settings.drain < this.settings.batteryMin - 1e-9,
      )
    ) {
      this.stop(
        "Next move would breach the battery reserve; all robots stopped.",
      );
      return;
    }
    let reactiveMs=0;
    if(this.junctionMode)this.junctionGrants=this.junctionGrants.filter(g=>g.issuedTick===this.tick);
    const advancing = new Set();
    const completedThisStep=[];
    if (this.baselineMode) {
      const remaining = this.robots.filter(
        (r) => r.plan && r.index < r.plan.points.length - 1,
      );
      // Fixed-point occupancy resolution supports safe convoys but forbids head-on swaps.
      let progress = true;
      while (progress) {
        progress = false;
        for (const r of remaining) {
          if (advancing.has(r.id)) continue;
          const q = r.plan.points[r.index + 1];
          if (
            !this.robots.some(
              (other) => other.id !== r.id && same(other.pos, q),
            )
          ) {
            r.pos = { ...q };
            advancing.add(r.id);
            progress = true;
          }
        }
      }
    } else {
      const local=this.junctionMode?junctionMoves(junctionFrame(this.robots,this.orders,this.tick),this.robots,this.junctionGrants,this.junctionPending):null;
      if(local){this.junctionStats.approachMessages+=local.messages??0;this.junctionStats.approachBytes+=local.bytes??0;}
      const proposals = this.robots.map((r) =>
        r.plan && r.index < r.plan.points.length - 1 && (!local||local.has(r.id))
          ? r.plan.points[r.index + 1]
          : r.pos,
      );
      let invalid = false;
      for (let i = 0; i < this.robots.length; i++)
        for (let j = i + 1; j < this.robots.length; j++)
          if (
            distance(proposals[i], proposals[j]) <
              this.settings.separation - 1e-9 ||
            sweptCollision(
              this.robots[i].pos,
              proposals[i],
              this.robots[j].pos,
              proposals[j],
            )
          )
            invalid = true;
      if (invalid) {
        this.guardInterventions++;
        this.stop("Runtime reservation inconsistency: all robots stopped.");
        return;
      }
      this.robots.forEach((r, i) => {
        if (r.plan && (!local||local.has(r.id))) {
          r.pos = { ...proposals[i] };
          advancing.add(r.id);
        }
      });
    }
    for (const r of this.robots) {
      if (!same(r.previous, r.pos)) {
        if (r.battery < this.settings.drain) {
          r.pos = { ...r.previous };
          this.stop(`R${r.id + 1} cannot move with an empty battery.`);
          return;
        }
        r.battery = Math.max(0, r.battery - this.settings.drain);
        r.moves++;
      }
      if (r.plan) {
        if (same(r.previous, r.pos)) {
          r.wait++;

          if (!advancing.has(r.id)) r.blockedWaiting++;
          else if (r.index+1===r.plan.pickupIndex || r.index+1===r.plan.deliveryIndex) r.serviceWaiting++;
          else r.scheduledWaiting++;
        }
        if (advancing.has(r.id)) {
          r.index++;
          r.blocked = 0;
        } else r.blocked++;
        const order = this.orders.find((o) => o.id === r.orderId);
        if (r.index === r.plan.pickupIndex) {
          r.carrying = true;
          r.state = "delivery";
        }
        if (r.index === r.plan.deliveryIndex && order.done === null) {
          order.done = this.tick + 1;
          order.status = "done";
          r.delivered++;
          r.carrying = false;
          r.state = "return";
          completedThisStep.push({robotId:r.id,orderId:order.id,late:order.done>order.deadline});
        }
        if (r.index >= r.plan.points.length - 1) {
          r.plan = null;
          r.orderId = null;
          r.state = r.battery < 0.4 ? "charging" : "idle";
        }
      }
    }

    const rerouteClaims = new Map(), opposing = new Map();
    if (this.baselineMode)for(const r of this.robots){
      const next=r.plan?.points[r.index+1];
      if(!next||same(next,r.pos))continue;
      const other=this.robots.find(o=>o.id!==r.id&&same(o.pos,next)&&o.plan&&same(o.plan.points[o.index+1]??o.pos,r.pos));
      if(other)opposing.set(r.id,other.id);
    }
    if (this.baselineMode)
      for (const r of this.robots)
        if (r.plan && r.blocked >= 6 + (r.id % 3)) {
          const temporary = {
            ...this.world,
            walls: new Set([
              ...this.world.walls,
              ...(rerouteClaims.get(opposing.get(r.id))??[]),
              ...this.robots
                .filter((o) => o.id !== r.id)
                .map((o) => `${o.pos.x},${o.pos.y}`),
            ]),
          };
          const rerouteStarted=performance.now();
          const alternative = spatialMission(
            temporary,
            r,
            this.orders.find((o) => o.id === r.orderId),
          );
          reactiveMs+=performance.now()-rerouteStarted;
          if (
            alternative?.points.length > 1 &&
            !same(alternative.points[1], r.pos) &&
            r.battery -
              movementCount(alternative.points) * this.settings.drain >=
              this.settings.batteryMin - 1e-9
          ) {
            r.plan = alternative;
            r.index = 0;
            r.blocked = 0;
            rerouteClaims.set(r.id,alternative.points.slice(1,3).map(p=>`${p.x},${p.y}`));
            this.log("plan", `R${r.id + 1} rerouted around occupied cells.`);
          }
        }
    if(this.baselineMode){
      const hasActive=this.robots.some(r=>r.plan);
      this.baselineNoProgressTicks=hasActive&&advancing.size===0?this.baselineNoProgressTicks+1:0;
    }
    this.planningMs+=reactiveMs;this.reactivePlanningMs+=reactiveMs;
    this.tick++;
    for(const event of completedThisStep)this.log('delivery',`R${event.robotId+1} delivered order ${event.orderId}${event.late?' late':''}.`,{robotId:event.robotId,orderId:event.orderId});
    this.record();
    // Record first, then charge computation: recursive execution sees a complete tick.
    if(reactiveMs)this.consumePlanningTime(reactiveMs);
    if(this.done)return;
    if (this.baselineNoProgressTicks >= 30) {
      this.baselineStall=this.describeBaselineStall();
      for(const cycle of this.baselineStall.cycles)this.log('blocking',`Circular wait: ${[...cycle,cycle[0]].map(id=>`R${id+1}`).join(' → ')}.`);
      this.stop(
        `Baseline stalled: no robot advanced a movement or service step for 30 ticks. ${this.baselineStall.cycles.length} circular-wait cycle(s) detected. Remaining missions reported incomplete.`,
      );
      return;
    }
    if (this.tick >= this.settings.horizon) {
      this.stop("Evaluation horizon reached.");
      return;
    }
    if (
      this.orders.every(
        (o) => o.status === "done" || o.status === "rejected",
      ) &&
      !this.robots.some((r) => r.plan)
    ) {
      this.done = true;
      this.log(
        "system",
        "Run complete. All missions delivered or explicitly rejected.",
      );
    }
  }

  // A cycle diagnoses the chosen next moves, not global warehouse infeasibility.
  describeBaselineStall(){
    const blockedRobots=this.robots.filter(r=>r.plan).map(r=>{
      const next=r.plan.points[r.index+1]??r.pos;
      const blocker=this.robots.find(o=>o.id!==r.id&&same(o.pos,next));
      return {robotId:r.id,orderId:r.orderId,state:r.state,position:{...r.pos},next:{...next},blockedTicks:r.blocked,blockedBy:blocker?.id??null};
    });
    const follows=new Map(blockedRobots.map(r=>[r.robotId,r.blockedBy])),unique=new Map();
    for(const robot of blockedRobots){
      const path=[],seen=new Map();let id=robot.robotId;
      while(id!==null&&follows.has(id)&&!seen.has(id)){seen.set(id,path.length);path.push(id);id=follows.get(id);}
      if(id!==null&&seen.has(id)){
        const cycle=path.slice(seen.get(id)),start=cycle.indexOf(Math.min(...cycle));
        const canonical=[...cycle.slice(start),...cycle.slice(0,start)];unique.set(canonical.join('/'),canonical);
      }
    }
    return {tick:this.tick,noProgressTicks:this.baselineNoProgressTicks,blockedRobots,cycles:[...unique.values()]};
  }

  stop(reason) {
    this.done = true;
    this.stopReason = reason;
    this.log("warning", reason);
  }

  record() {
    let separation = Infinity, closestRobots=null;
    for (let i = 0; i < this.robots.length; i++)
      for (let j = i + 1; j < this.robots.length; j++) {
        const a = this.robots[i],
          b = this.robots[j];
        const margin=distance(a.pos,b.pos)-this.settings.separation;
        if(margin<separation){separation=margin;closestRobots=[a.id,b.id];}
        if (same(a.pos, b.pos)) this.collisions++;
        if (this.tick && sweptCollision(a.previous, a.pos, b.previous, b.pos))
          this.edgeConflicts++;
      }
    this.trace.push({
      tick: this.tick,
      separation,
      battery:
        Math.min(...this.robots.map((r) => r.battery)) -
        this.settings.batteryMin,
      positions: this.robots.map((r) => ({ ...r.pos })),
      batteries: this.robots.map((r) => r.battery),
      waiting: this.robots.map((r) => r.wait),
      blocked: this.robots.map((r) => r.blocked),
      states: this.robots.map((r) => r.state),
      orderIds: this.robots.map((r) => r.orderId),
      completedOrderIds: this.orders
        .filter((o) => o.done !== null)
        .map((o) => o.id),
      delivered: this.orders.filter((o) => o.done !== null).length,
    });
    observeStl(this,this.trace.at(-1),closestRobots);
  }

  summary() {
    const delivered = this.orders.filter((o) => o.done !== null),
      onTime = delivered.filter((o) => o.done <= o.deadline).length;

    const violations = this.orders.filter((o) =>
      o.done === null ? this.tick >= o.deadline : o.done > o.deadline,
    ).length;
    const pending = this.orders.filter(
      (o) => o.done === null && this.tick < o.deadline,
    ).length;
    const monitoringBegin = performance.now();
    const safety = evaluate(
      always(
        0,
        this.trace.length - 1,
        predicate("separation", (s) => s.separation),
      ),
      this.trace,
    );
    const battery = evaluate(
      always(
        0,
        this.trace.length - 1,
        predicate("battery", (s) => s.battery),
      ),
      this.trace,
    );
    this.monitoringMs += performance.now()-monitoringBegin;
    return {
      fleetSize: this.fleetSize,
      congestion: this.congestion,
      guardInterventions: this.guardInterventions,
      baselineNoProgressTicks:this.baselineNoProgressTicks,
      baselineStall:this.baselineStall,
      ordersPerRobot: this.ordersPerRobot,
      currentBatchLimit: this.batchLimit,
      warehouseLayout: this.warehouseLayout,
      mode: this.mode,
      scenario: this.scenario,
      seed: this.seed,
      overlappingDispatches:this.overlappingDispatches,
      partialChargeDispatches:this.partialChargeDispatches,
      dispatchPolicy:this.eventDispatch?'event-driven':'fleet-batch',
      junction:this.junctionMode?{...this.junctionStats}:null,
      admmRho:this.settings.admmRho,
      admmUntilConverged:!!this.settings.admmUntilConverged,
      timingModel:this.settings.timingModel, secondsPerTick:this.settings.secondsPerTick,
      planningTicks:this.planningTicks, planningClockMs:this.planningClockMs, reactivePlanningMs:this.reactivePlanningMs,
      stalePlanningAttempts:this.stalePlanningAttempts,
      ticks: this.tick,
      delivered: delivered.length,
      total: this.orders.length,
      queued: this.orders.filter(o=>o.status === "queued").length,
      assigned: this.orders.filter(o=>o.status === "assigned").length,
      incomplete: this.orders.filter(o=>o.done === null && o.status !== "rejected").length,
      throughputPerTick: delivered.length / Math.max(1,this.tick),
      onTime,
      deadlineViolations: violations,
      pending,
      rejected: this.orders.filter((o) => o.status === "rejected").length,
      waiting: this.robots.reduce((s, r) => s + r.wait, 0),
      moves: this.robots.reduce((s, r) => s + r.moves, 0),
      collisions: this.collisions,
      edgeConflicts: this.edgeConflicts,
      ...deliveryAggregates(this.orders,this.robots,this.tick),
      stlSafetyWitness:this.stlObservation.safetyWitness,
      stlBatteryWitness:this.stlObservation.batteryWitness,
      safetyRobustness: safety.robustness,
      batteryRobustness: battery.robustness,
      planningMs: this.planningMs,
      monitoringMs: this.monitoringMs,
      coordinationBytes: this.batches.reduce((sum,b)=>sum+(b.solver?.messageBytes??0),0)+(this.junctionMode?this.junctionStats.approachBytes:0),
      coordinationRounds: this.batches.reduce(
        (n, b) => n + (b.solver?.iterations ?? 0),
        0,
      ),
      coordinationMessages: (this.junctionMode?this.junctionStats.approachMessages:0)+this.batches.reduce(
        (n, b) => n + (b.solver?.messages ?? 0),
        0,
      ),
      admmIterations: this.batches.reduce(
        (n, b) => n + (b.solver?.iterations ?? 0),
        0,
      ),
      admmMessages: this.batches.reduce(
        (n, b) => n + (b.solver?.messages ?? 0),
        0,
      ),
      fixedModelInfeasibleBatches: this.batches.filter(b => b.solver?.failureKind === "fixed-model-infeasible").length,
      numericalTimeoutBatches: this.batches.filter(b => b.solver?.failureKind === "numerical-timeout").length,
      deferredAttempts: this.orders.reduce((sum, o) => sum + (o.deferrals ?? 0), 0),
      solverBatches: this.batches.filter((b) => b.solver).length,
      solverFailures: this.batches.filter(
        (b) => b.solver && b.solver.status !== "converged",
      ).length,
      done: this.done,
      stopReason: this.stopReason,
    };
  }

  export() {
    return {
      schema: "warehouse-mas-lab/v2",
      batches: this.batches,
      eventCount: this.eventCount,
      eventsTruncated: this.eventCount > this.events.length,
      configuration: {
        fleetSize: this.fleetSize,
      congestion: this.congestion,
      guardInterventions: this.guardInterventions,
        ordersPerRobot: this.ordersPerRobot,
      currentBatchLimit: this.batchLimit,
        warehouseLayout: this.warehouseLayout,
        mode: this.mode,
        dispatchPolicy:this.eventDispatch?'event-driven':'fleet-batch',
        controllerOwnership:['incremental','junction-assignment'].includes(this.mode)?'peer-owned-v2':this.eventDispatch?'shared-individual-dispatch':'shared-batch-dispatch',
        scenario: this.scenario,
        seed: this.seed,
        settings: this.settings,
      },
      summary: this.summary(),
      orders: this.orders,
      trace: this.trace,
      events: [...this.events].reverse(),
      solver: this.latestSolver,
    };
  }
}

