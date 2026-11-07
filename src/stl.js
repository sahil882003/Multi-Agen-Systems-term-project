export const predicate = (name, fn) => ({ type: "predicate", name, fn });

export const always = (from, to, child) => ({
  type: "always",
  from,
  to,
  child,
});

export function evaluate(formula, trace, at = 0) {
  if (formula.type === "predicate") {
    const sample = trace[at];
    if (sample === undefined)
      return {
        lo: -Infinity,
        hi: Infinity,
        status: "pending",
        robustness: null,
      };
    const value = formula.fn(sample);
    return {
      lo: value,
      hi: value,
      status: value >= 0 ? "satisfied" : "violated",
      robustness: value,
    };
  }
  const children =
    formula.type === "and"
      ? formula.children.map((f) => evaluate(f, trace, at))
      : Array.from(
          { length: Math.max(0, formula.to - formula.from + 1) },
          (_, i) => evaluate(formula.child, trace, at + formula.from + i),
        );
  const combine = formula.type === "eventually" ? Math.max : Math.min;
  const lo = combine(...children.map((c) => c.lo)),
    hi = combine(...children.map((c) => c.hi));
  return {
    lo,
    hi,
    status: lo >= 0 ? "satisfied" : hi < 0 ? "violated" : "pending",
    robustness: lo === hi ? lo : null,
  };
}

export function trajectoryChecks(world, robots, plans, settings) {
  const maxTime = Math.max(0, ...plans.map((p) => p.points.length - 1)),
    errors = [];

  const byRobot=robots.map(r=>plans.find(p=>p.robotId===r.id));
  for (const p of plans)
    for (let t = 0; t < p.points.length; t++) {
      const q = p.points[t];
      if (
        q.x < 0 ||
        q.y < 0 ||
        q.x >= world.width ||
        q.y >= world.height ||
        world.walls.has(`${q.x},${q.y}`)
      )
        errors.push(`R${p.robotId + 1}: obstacle at ${t}`);
      if (
        t &&
        Math.abs(q.x - p.points[t - 1].x) + Math.abs(q.y - p.points[t - 1].y) >
          1
      )
        errors.push(`R${p.robotId + 1}: invalid step at ${t}`);
    }
  let previous;
  for (let t = 0; t <= maxTime; t++) {
    const current=robots.map((r,i)=>{const p=byRobot[i];return p?p.points[Math.min(t,p.points.length-1)]:r.pos;});
    for (let i = 0; i < robots.length; i++)
      for (let j = i + 1; j < robots.length; j++) {
        const a = current[i],
          b = current[j];
        if (Math.hypot(a.x - b.x, a.y - b.y) < settings.separation - 1e-9)
          errors.push(`R${i + 1}/R${j + 1}: separation at ${t}`);
        if (t) {
          const ap = previous[i],
            bp = previous[j];
          // Minimum relative separation anywhere along the two linear moves in this tick.
          const rx = ap.x - bp.x,
            ry = ap.y - bp.y,
            vx = a.x - ap.x - (b.x - bp.x),
            vy = a.y - ap.y - (b.y - bp.y);
          const vv = vx * vx + vy * vy,
            s = vv ? Math.max(0, Math.min(1, -(rx * vx + ry * vy) / vv)) : 0;
          if (Math.hypot(rx + s * vx, ry + s * vy) < 1e-8)
            errors.push(`R${i + 1}/R${j + 1}: swept collision at ${t}`);
        }
      }
    previous=current;
  }
  return { ok: errors.length === 0, errors: [...new Set(errors)].slice(0, 12) };
}
