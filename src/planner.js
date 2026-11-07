import { planJunctionRoutes, junctionLaneStep } from './junction.js';
import { planIncremental, remainingCommitments } from "./incremental.js";
import {planAgentBatch} from "./robot-agent.js";
import {indexRoutePredictions} from './route-predictions.js';
import {missionBounds, keepSearchLabel} from './search-guidance.js';
import { neighbours, same, spatialMission, movementCount } from "./world.js";
class Heap {

  before(a, b) {
    return a.f < b.f || a.f === b.f && (a.tieMargin ?? -Infinity) > (b.tieMargin ?? -Infinity);
  }

  constructor() {
    this.a = [];
  }

  push(v) {
    const a = this.a;
    a.push(v);
    let i = a.length - 1;
    while (i) {
      const p = (i - 1) >> 1;
      if (!this.before(v, a[p])) break;
      a[i] = a[p];
      i = p;
    }
    a[i] = v;
  }

  pop() {
    const a = this.a,
      first = a[0],
      last = a.pop();
    if (a.length) {
      let i = 0;
      while (i * 2 + 1 < a.length) {
        let j = i * 2 + 1;
        if (j + 1 < a.length && this.before(a[j + 1], a[j])) j++;
        if (!this.before(a[j], last)) break;
        a[i] = a[j];
        i = j;
      }
      a[i] = last;
    }
    return first;
  }
  get length() {
    return this.a.length;
  }
}

export function sweptCollision(a, b, c, d) {
  const rx = a.x - c.x,
    ry = a.y - c.y,
    vx = b.x - a.x - (d.x - c.x),
    vy = b.y - a.y - (d.y - c.y),
    vv = vx * vx + vy * vy;
  const s = vv ? Math.max(0, Math.min(1, -(rx * vx + ry * vy) / vv)) : 0;
  return Math.hypot(rx + s * vx, ry + s * vy) < 1e-8;
}

function occupancy(plans, robots, exclude, t) {
  return robots
    .filter((r) => r.id !== exclude)
    .map((r) => {
      const p = plans.find((p) => p.robotId === r.id);
      return p ? p.points[Math.min(t, p.points.length - 1)] : r.pos;
    });
}

function allowed(from, to, t, plans, robots, exclude, separation = 1) {
  const old = occupancy(plans, robots, exclude, t - 1),
    next = occupancy(plans, robots, exclude, t);
  return next.every(
    (p, i) =>
      Math.hypot(to.x - p.x, to.y - p.y) >= separation - 1e-9 &&
      !sweptCollision(from, to, old[i], p),
  );
}

function searchTimedLeg(
  world,
  robot,
  from,
  target,
  start,
  maxTime,
  plans,
  robots,
  settings,
  guided,
  holdAtTarget = false,
  guidance = null,
) {

  const byId=new Map(plans.map(p=>[p.robotId,p]));
  const others=robots.filter(r=>r.id!==robot.id),positions=new Map();
  const at=t=>{if(!positions.has(t))positions.set(t,others.map(r=>{const p=byId.get(r.id);return p?p.points[Math.min(t,p.points.length-1)]:r.pos;}));return positions.get(t);};
  const predictionIndex=(settings.peerSearchIndex||guidance)?indexRoutePredictions(at,settings,sweptCollision):null;
  const canMove=predictionIndex?predictionIndex.canMove:(a,b,t)=>{const old=at(t-1),next=at(t);return next.every((p,i)=>Math.hypot(b.x-p.x,b.y-p.y)>=settings.separation-1e-9&&!sweptCollision(a,b,old[i],p));};
  const initialBounds = guidance?.bounds(from, start, 0);
  const feasible = bounds => {
    if (!Number.isFinite(bounds.distance)) { guidance.stats.unreachable++; return false; }
    if (bounds.deadlineSlack < -1e-9) { guidance.stats.deadlinePruned++; return false; }
    if (bounds.batteryMargin < -1e-9) { guidance.stats.energyPruned++; return false; }
    return true;
  };
  if (guidance && !feasible(initialBounds)) return {
    path: null, explored: 0,
    reason: initialBounds.batteryMargin < -1e-9 ? 'Insufficient energy for delivery and return.' :
      'No admissible timed route within the search/deadline bound.',
  };
  const heap = new Heap(),
    key = (p, t) => `${p.x},${p.y},${t}`,
    root = { p: from, t: start, g: 0, f: guidance ? initialBounds.distance : 0, parent: null,
      ...(guidance ? {moves: 0, minMargin: Math.min(5, ...at(start).map(p => Math.hypot(from.x-p.x,from.y-p.y)-settings.separation))} : {}) };
  const best = new Map([[key(from, start), 0]]);
  const labels = new Map();
  const energyAware = guidance?.preserveEnergy;
  const fastLabels = new Map([[key(from, start), root]]);
  let energyAlternatives = false;
  if (energyAware) keepSearchLabel(labels, key(from, start), root);
  heap.push(root);
  let explored = 0;
  while (heap.length && explored < 45000) {
    const node = heap.pop();
    if (guidance ? node.discarded : node.g > best.get(key(node.p, node.t)) + 1e-9) continue;
    explored++;
    if (
      same(node.p, target) &&
      (!holdAtTarget ||
        canMove(node.p,node.p,node.t+1))
    ) {
      const path = [];
      for (let n = node; n; n = n.parent) path.push(n.p);
      return { path: path.reverse(), explored, minMargin: node.minMargin };
    }
    if (node.t >= maxTime) continue;
    for (const q of neighbours(world, node.p, robot.home, true)) {
      if(settings.junctionLanes&&!junctionLaneStep(node.p,q,robot.home))continue;
      const t = node.t + 1;
      const moves = guidance ? node.moves + Number(!same(q, node.p)) : 0;
      const bounds = guidance?.bounds(q, t, moves);
      if (guidance && !feasible(bounds)) continue;
      if (!canMove(node.p,q,t))
        continue;
      const others = at(t),
        margin = predictionIndex?predictionIndex.margin(q,t):Math.min(
          5,
          ...others.map(
            (p) => Math.hypot(q.x - p.x, q.y - p.y) - settings.separation,
          ),
        );

      const g =
        node.g +
        1 +
        (guided ? 0.14 / (0.2 + Math.max(0, margin)) : 0) +
        (same(q, node.p) ? 0.03 : 0);
      const k = key(q, t);
      const child = {
        p: q,
        t,
        g,
        f: g + (guidance ? bounds.distance : Math.abs(q.x - target.x) + Math.abs(q.y - target.y)),
        parent: node,
        ...(guidance ? {moves, minMargin: Math.min(node.minMargin, margin)} : {}),
        ...(guidance && guided ? {tieMargin: Math.min(node.minMargin, margin)} : {}),
      };
      if (energyAware) {
        if (!keepSearchLabel(labels, k, child)) continue;
      } else if (guidance) {
        const old = fastLabels.get(k);
        if (old && old.moves !== child.moves) energyAlternatives = true;
        if (old && (old.g < child.g - 1e-9 || Math.abs(old.g-child.g)<=1e-9 &&
          (old.tieMargin??-Infinity)>=(child.tieMargin??-Infinity))) continue;
        if (old) old.discarded = true;
        fastLabels.set(k, child);
      } else {
        if (g >= (best.get(k) ?? Infinity) - 1e-9) continue;
        best.set(k, g);
      }
      heap.push(child);
    }
  }
  return { path: null, explored, energyAlternatives };
}

function timedLeg(...args) {
  const result = searchTimedLeg(...args), guidance = args[11];
  if (!guidance || result.path || !result.energyAlternatives) return result;
  guidance.stats.energyAwareRetries++;
  args[11] = {...guidance, preserveEnergy: true};
  const retried = searchTimedLeg(...args);
  return {...retried, explored: result.explored + retried.explored};
}

function timedMission(
  world,
  robot,
  order,
  plans,
  robots,
  tick,
  settings,
  guided = true,
) {
  const points = [{ ...robot.pos }],
    targets = [order.pickup, order.dropoff, robot.home];
  let pickupIndex,
    deliveryIndex,
    explored = 0;
  const enhanced = guided && settings.stlGuidance === 'mission-bounds-v1';
  const stats = {deadlinePruned: 0, energyPruned: 0, unreachable: 0, energyAwareRetries: 0};
  let minimumSafetyMargin = Infinity;
  for (let i = 0; i < targets.length; i++) {
    const bound =
      guided && i < 2 ? Math.min(450, order.deadline - tick - 1) : 450;
    const leg = timedLeg(
      world,
      robot,
      points.at(-1),
      targets[i],
      points.length - 1,
      bound,
      plans,
      robots,
      settings,
      guided,
      i < 2,
      enhanced ? {bounds: missionBounds(world, robot, order, i, tick, settings, movementCount(points)), stats} : null,
    );
    explored += leg.explored;
    if (!leg.path)
      return {
        ok: false,
        reason: leg.reason ?? "No admissible timed route within the search/deadline bound.",
        explored,
        ...(enhanced ? {searchGuidance: {...stats}} : {}),
      };
    if (enhanced) minimumSafetyMargin = Math.min(minimumSafetyMargin, leg.minMargin);
    points.push(...leg.path.slice(1));
    if (i < 2) {
      if (
        !allowed(
          points.at(-1),
          points.at(-1),
          points.length,
          plans,
          robots,
          robot.id,
          settings.separation,
        )
      )
        return {
          ok: false,
          reason: "Service-cell reservation conflict.",
          explored,
        };
      points.push({ ...points.at(-1) });
      if (enhanced) minimumSafetyMargin = Math.min(minimumSafetyMargin, 5, ...occupancy(plans, robots, robot.id, points.length-1)
        .map(p => Math.hypot(points.at(-1).x-p.x, points.at(-1).y-p.y)-settings.separation));
      if (i === 0) pickupIndex = points.length - 1;
      else deliveryIndex = points.length - 1;
    }
  }
  const batteryMargin =
    robot.battery -
    movementCount(points) * settings.drain -
    settings.batteryMin;
  if (guided && batteryMargin < -1e-9)
    return {
      ok: false,
      reason: "Insufficient energy for delivery and return.",
      explored,
    };
  if (guided && tick + deliveryIndex > order.deadline)
    return { ok: false, reason: "Delivery deadline infeasible.", explored };
  return {
    ok: true,
    plan: {
      robotId: robot.id,
      orderId: order.id,
      points,
      pickupIndex,
      deliveryIndex,
      batteryMargin,
      ...(enhanced ? {searchGuidance: {version: settings.stlGuidance, ...stats,
        minimumSafetyMargin, safetyMarginCap: 5, deliverySlack: order.deadline-tick-deliveryIndex}} : {}),
    },
    explored,
  };
}

export function planBatch(world,robots,jobs,mode,tick,settings){
  if(mode==='junction-assignment'){const start=performance.now();return {...planJunctionRoutes(world,jobs,tick,settings,timedMission),ms:performance.now()-start};}
  if(mode==='incremental'){const begin=performance.now();return {...planIncremental(world,robots,jobs,tick,settings,timedMission),ms:performance.now()-begin};}
 const start=performance.now();
 if(mode==='baseline'){
  const plans=[],rejected=[];
  for(const {robot,order} of [...jobs].sort((a,b)=>a.robot.id-b.robot.id)){
   const p=spatialMission(world,robot,order);if(p)plans.push(p);else rejected.push({orderId:order.id,reason:'No spatial route.'});
  }
  return {plans,rejected,deferred:[],explored:0,solver:null,ms:performance.now()-start};
 }
 if(mode!=='guided-local')throw new Error('Unsupported planning method.');
 const result=planAgentBatch(world,robots,jobs,tick,{...settings,stlGuidance:'mission-bounds-v1'},timedMission,
   {coordinate:false,maxRounds:settings.admmMaxRounds??3000,commitments:remainingCommitments(robots)});
 return {...result,ms:performance.now()-start};
}

