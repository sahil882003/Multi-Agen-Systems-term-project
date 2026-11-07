export function isotonic(values, weights, upper, lower = null) {
  const blocks = [];
  for (let i = 0; i < values.length; i++) {
    blocks.push({
      start: i,
      end: i,
      sum: weights[i] * values[i],
      weight: weights[i],
      cap: upper[i],
      floor: lower?.[i] ?? 0,
    });
    const value = (b) => Math.max(b.floor, Math.min(b.cap, b.sum / b.weight));
    while (blocks.length > 1 && value(blocks.at(-2)) > value(blocks.at(-1))) {
      const b = blocks.pop(),
        a = blocks.pop();
      blocks.push({
        start: a.start,
        end: b.end,
        sum: a.sum + b.sum,
        weight: a.weight + b.weight,
        cap: Math.min(a.cap, b.cap),
        floor: Math.max(a.floor, b.floor),
      });
    }
  }
  const out = Array(values.length);
  for (const b of blocks)
    for (let i = b.start; i <= b.end; i++)
      out[i] = Math.max(b.floor, Math.min(b.cap, b.sum / b.weight));
  return out;
}

