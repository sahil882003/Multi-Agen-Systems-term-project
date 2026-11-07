import {cellKey, neighbours} from './world.js';

// Maps are immutable during a run. Keep robot-specific private bays in the key.
const distanceCache = new WeakMap();
function distanceMap(world, target, home) {
  let maps = distanceCache.get(world);
  if (!maps) distanceCache.set(world, maps = new Map());
  const key = `${cellKey(target)}/${cellKey(home)}`;
  if (maps.has(key)) return maps.get(key);
  const distances = new Map();
  if (neighbours(world, target, home, true).some(p => cellKey(p) === cellKey(target))) {
    const queue = [target];
    distances.set(cellKey(target), 0);
    for (let head = 0; head < queue.length; head++) {
      const p = queue[head], nextDistance = distances.get(cellKey(p)) + 1;
      for (const q of neighbours(world, p, home)) {
        const k = cellKey(q);
        if (!distances.has(k)) { distances.set(k, nextDistance); queue.push(q); }
      }
    }
  }
  maps.set(key, distances);
  return distances;
}

export function missionBounds(world, robot, order, leg, tick, settings, movesBefore) {
  const targets = [order.pickup, order.dropoff, robot.home];
  const maps = targets.map(target => distanceMap(world, target, robot.home));
  const distance = (i, p) => maps[i].get(cellKey(p)) ?? Infinity;
  const pickupToDropoff = distance(1, order.pickup);
  const dropoffToHome = distance(2, order.dropoff);
  const energyTail = leg === 0 ? pickupToDropoff + dropoffToHome : leg === 1 ? dropoffToHome : 0;
  return (p, t, moves) => {
    const toTarget = distance(leg, p);
    const deliveryRemaining = leg === 0 ? toTarget + pickupToDropoff + 2 : leg === 1 ? toTarget + 1 : 0;
    return {
      distance: toTarget,
      deadlineSlack: leg < 2 ? order.deadline - tick - t - deliveryRemaining : Infinity,
      batteryMargin: robot.battery - (movesBefore + moves + toTarget + energyTail) * settings.drain - settings.batteryMin,
    };
  };
}

// Same position/time can have multiple useful cost/energy tradeoffs.
export function keepSearchLabel(labels, key, candidate) {
  const previous = labels.get(key) ?? [];
  const dominates = (a, b) => a.g <= b.g + 1e-9 && a.moves <= b.moves &&
    (a.g < b.g - 1e-9 || a.minMargin >= b.minMargin);
  if (previous.some(old => dominates(old, candidate))) return false;
  const remaining = previous.filter(old => {
    if (!dominates(candidate, old)) return true;
    old.discarded = true;
    return false;
  });
  remaining.push(candidate);
  labels.set(key, remaining);
  return true;
}
