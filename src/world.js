export const ROBOT_COLORS = [
  "#4cd6b4",
  "#ffb568",
  "#a4a3ff",
  "#60b9f5",
  "#ff8496",
  ...Array.from(
    { length: 45 },
    (_, i) =>
      "#" +
      Math.floor(0x668888 + ((i * 0x174b29) % 0x997777))
        .toString(16)
        .padStart(6, "0"),
  ),
];
export const SETTINGS = {
  drain: 0.0035,
  idleDrain: 0,
  chargeRate: 0.035,
  batteryMin: 0.12,
  separation: 1,
  horizon: 600,
};

export function rng(seed = 7) {
  let a = Number(seed) >>> 0;
  return () => {
    a += 0x6d2b79f5;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export const cellKey = (p) => `${p.x},${p.y}`;

export const same = (a, b) => a.x === b.x && a.y === b.y;
export const distance = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

export function makeWorld(fleetSize = 5, { layout = "standard" } = {}) {
  if (!Number.isInteger(fleetSize) || fleetSize < 1 || fleetSize > 50)
    throw new Error("Fleet must be an integer from 1 to 50.");
  if (!['standard', 'scaled', 'fixed-large'].includes(layout))
    throw new Error("Layout must be standard, scaled or fixed-large.");
  const scaled = layout === "scaled" || layout === "fixed-large" || fleetSize > 20,
    width = layout === "fixed-large" ? 54 : scaled ? Math.max(22, fleetSize + 4) : 22,
    height = layout === "fixed-large" ? 22 : scaled ? Math.max(16, 12 + 2 * Math.ceil(fleetSize / 10)) : 16,
    shelfXs = scaled
      ? Array.from(
          { length: Math.max(1, Math.floor((width - 5) / 6)) },
          (_, i) => 3 + i * 6,
        ).filter((x) => x + 2 < width - 1)
      : [3, 9, 15],
    shelfYs = scaled
      ? Array.from(
          { length: Math.max(1, Math.floor((height - 5) / 5)) },
          (_, i) => 4 + i * 5,
        ).filter((y) => y + 2 < height - 1)
      : [4, 9],
    walls = new Set(),
    shelves = [],
    pickups = [];
  for (let x = 0; x < width; x++) {
    walls.add(`${x},0`);
    walls.add(`${x},${height - 1}`);
  }
  for (let y = 0; y < height; y++) {
    walls.add(`0,${y}`);
    walls.add(`${width - 1},${y}`);
  }
  for (const x of shelfXs)
    for (const y of shelfYs) {
      shelves.push({ x, y, w: 3, h: 3, label: `S${shelves.length + 1}` });
      for (let dx = 0; dx < 3; dx++)
        for (let dy = 0; dy < 3; dy++) walls.add(`${x + dx},${y + dy}`);
      pickups.push({ x: x - 1, y: y + 1 });
    }
  const homeXs =
    !scaled && fleetSize === 5
      ? [2, 6, 10, 14, 18]
      : !scaled && fleetSize === 10
        ? Array.from({ length: fleetSize }, (_, i) => 1 + i * 2)
        : !scaled && fleetSize === 20
          ? Array.from({ length: fleetSize }, (_, i) => 1 + i)
          : Array.from(
              { length: fleetSize },
              (_, i) =>
                1 +
                Math.round((i * (width - 3)) / Math.max(1, fleetSize - 1)),
            );
  const homes = homeXs.map((x) => ({ x, y: height - 2 }));
  const middle = Math.floor(height / 2);
  const docks = [
    { x: 3, y: 1 },
    { x: width - 4, y: 1 },
    { x: 1, y: middle },
    { x: width - 2, y: middle },
  ];
  return {
    width,
    height,
    walls,
    shelves,
    pickups,
    homes,
    docks,
    layout,
    scaleFactor: Math.max(width / 22, height / 16),
  };
}

// A robot may use its own home bay, but other robots' private bays remain
// blocked even though they are inside the warehouse boundary.
export function neighbours(world, p, home, includeWait = false) {
  const dirs = [
    [0, -1],
    [-1, 0],
    [1, 0],
    [0, 1],
  ];
  if (includeWait) dirs.push([0, 0]);
  return dirs
    .map(([dx, dy]) => ({ x: p.x + dx, y: p.y + dy }))
    .filter(
      (q) =>
        q.x >= 0 &&
        q.y >= 0 &&
        q.x < world.width &&
        q.y < world.height &&
        !world.walls.has(cellKey(q)) &&
        !world.homes.some((h) => same(q, h) && !same(h, home)),
    );
}

function bfs(world, from, to, home) {
  const queue = [from],
    parent = new Map([[cellKey(from), null]]);
  for (let head = 0; head < queue.length; head++) {
    const p = queue[head];
    if (same(p, to)) {
      const out = [];
      let k = cellKey(p);
      while (k !== null) {
        const [x, y] = k.split(",").map(Number);
        out.push({ x, y });
        k = parent.get(k);
      }
      return out.reverse();
    }
    for (const q of neighbours(world, p, home))
      if (!parent.has(cellKey(q))) {
        parent.set(cellKey(q), cellKey(p));
        queue.push(q);
      }
  }
  return null;
}

export function makeOrders(world, seed, scenario, ordersPerRobot = 2, congestion = "medium") {
  if (!Number.isInteger(ordersPerRobot) || ordersPerRobot < 1 || ordersPerRobot > 30)
    throw new Error("Orders per robot must be an integer from 1 to 30.");
  const random = rng(seed),
    count = world.homes.length * ordersPerRobot;
  if (scenario === "mixed") {

    const hotspot = world.pickups[Math.floor(random() * world.pickups.length)];
    const dock = world.docks[Math.floor(random() * world.docks.length)];
    return Array.from({ length: count }, (_, i) => {
      const slot = i % 10;
      const congested = ({low:[],medium:[5,6],high:[0,5,6,7,8,9]}[congestion] ?? [5,6]).includes(slot);
      const challenge = slot === 3 ? "impossible-deadline" : slot === 4 ? "tight-deadline"
        : (slot === 1 || slot === 2 || congested) ? "congestion" : "ordinary";
      const pickup = { ...(challenge === "congestion" ? hotspot
        : world.pickups[Math.floor(random() * world.pickups.length)]) };
      const dropoff = { ...(challenge === "congestion" ? dock
        : world.docks[Math.floor(random() * world.docks.length)]) };
      const manhattan = (a,b) => Math.abs(a.x-b.x)+Math.abs(a.y-b.y);
      const lowerBound = Math.min(...world.homes.map(h=>manhattan(h,pickup)))
        + manhattan(pickup,dropoff) + 2;
      const normalDeadline = (210 + i * 18) * Math.max(1, world.homes.length / 5);
      const deadline = challenge === "impossible-deadline" ? Math.max(1, lowerBound - 1)
        : challenge === "tight-deadline" ? Math.ceil(lowerBound * 1.5 + i * 5)
        : normalDeadline;
      return {id:i+1, pickup, dropoff, created:0, deadline, challenge,
        status:"queued", robot:null, done:null, failure:null};
    });
  }

  return Array.from({ length: count }, (_, i) => ({
    id: i + 1,
    pickup: {
      ...world.pickups[
        scenario === "congestion"
          ? 1
          : Math.floor(random() * world.pickups.length)
      ],
    },
    dropoff: {
      ...world.docks[
        scenario === "congestion"
          ? 0
          : Math.floor(random() * world.docks.length)
      ],
    },
    created: 0,
    deadline:
      scenario === "tight"
        ? i < 3
          ? 8
          : 75
        : (210 + i * 18) * Math.max(1, world.homes.length / 5),
    status: "queued",
    robot: null,
    done: null,
    failure: null,
  }));
}

export function spatialMission(world, robot, order) {
  const points = [{ ...robot.pos }];
  let pickupIndex = null,
    deliveryIndex = null;
  const actions = [];
  if (order && robot.state !== "return") {
    if (!robot.carrying) actions.push({ target: order.pickup, type: "pickup" });
    actions.push({ target: order.dropoff, type: "delivery" });
  }
  actions.push({ target: robot.home, type: "home" });
  const targets = actions.map((a) => a.target);
  for (let i = 0; i < targets.length; i++) {
    const leg = bfs(world, points.at(-1), targets[i], robot.home);
    if (!leg) return null;
    points.push(...leg.slice(1));
    if (actions[i].type !== "home") {
      points.push({ ...targets[i] });
      if (actions[i].type === "pickup") pickupIndex = points.length - 1;
      else deliveryIndex = points.length - 1;
    }
  }
  return {
    robotId: robot.id,
    points,
    pickupIndex,
    deliveryIndex,
    orderId: order?.id ?? null,
  };
}

export function movementCount(points) {
  return points
    .slice(1)
    .reduce((n, p, i) => n + Number(!same(p, points[i])), 0);
}
