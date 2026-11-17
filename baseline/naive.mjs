// Baseline: the discrete scan a 240 Hz engine performs (one gap evaluation per frame).
// This is the realistic alternative the certificate is compared against.
import { frames, gap, MARGIN } from '../src/core.mjs';

export function discreteScan(chain, hz = 240) {
  const steps = Math.round(chain.T * hz);
  const dt = chain.T / steps;
  const pairs = [];
  for (let i = 0; i < chain.n; i++) for (let j = i + 2; j < chain.n; j++) pairs.push([i, j]);
  const hit = new Array(pairs.length).fill(false);
  const firstT = new Array(pairs.length).fill(null);
  let evals = 0, deepest = 0;
  for (let s = 0; s <= steps; s++) {
    const t = s * dt;
    const F = frames(chain, t, null);
    for (let p = 0; p < pairs.length; p++) {
      const [i, j] = pairs[p];
      const g = gap(chain, t, i, j, null, F); evals++;
      if (g < deepest) deepest = g;
      if (g < MARGIN && !hit[p]) { hit[p] = true; firstT[p] = t; }
    }
  }
  return { hits: hit.filter(Boolean).length, deepestMm: Math.round(deepest * 1e5) / 1e2, evals, steps, firstT };
}