#!/usr/bin/env node
// Independent recomputation: does not import src/core.mjs and shares no constant with it.
// Forward kinematics and the segment-segment distance are re-derived from the corpus, then the
// deepest gap per chain is compared with the value baked into data/chains.json.
import { readFileSync } from 'node:fs';

const data = JSON.parse(readFileSync(new URL('../data/chains.json', import.meta.url)));

// nested bisection on both segment parameters (a different algorithm from the analytic clamp)
function segDist(a, b, c, e) {
  const f = (s, t) => Math.hypot(
    a[0] + (b[0] - a[0]) * s - (c[0] + (e[0] - c[0]) * t),
    a[1] + (b[1] - a[1]) * s - (c[1] + (e[1] - c[1]) * t));
  let lo1 = 0, hi1 = 1, lo2 = 0, hi2 = 1;
  for (let it = 0; it < 40; it++) {
    const m1 = (lo1 + hi1) / 2, m2 = (lo2 + hi2) / 2;
    if (f((lo1 + m1) / 2, m2) < f((m1 + hi1) / 2, m2)) hi1 = (m1 + hi1) / 2; else lo1 = (lo1 + m1) / 2;
    if (f(m1, (lo2 + m2) / 2) < f(m1, (m2 + hi2) / 2)) hi2 = (m2 + hi2) / 2; else lo2 = (lo2 + m2) / 2;
  }
  return f((lo1 + hi1) / 2, (lo2 + hi2) / 2);
}

let pass = 0, fail = 0;
for (const ch of data.chains) {
  const n = ch.L.length;
  const angle = (k, t) => { let a = 0; for (let m = 0; m <= k; m++) a += ch.q0[m] + ch.amp[m] * Math.sin(2 * Math.PI * ch.freq[m] * t + ch.phase[m]); return a; };
  const points = (t) => {
    const p = [[0, 0]]; let x = 0, y = 0;
    for (let k = 0; k < n; k++) { const a = angle(k, t); x += ch.L[k] * Math.cos(a); y += ch.L[k] * Math.sin(a); p.push([x, y]); }
    return p;
  };
  let best = Infinity;
  for (let s = 0; s <= 2000; s++) {
    const t = (ch.T * s) / 2000, p = points(t);
    for (let i = 0; i < n; i++) for (let j = i + 2; j < n; j++) {
      const g = segDist(p[i], p[i + 1], p[j], p[j + 1]) - ch.r[i] - ch.r[j];
      if (g < best) best = g;
    }
  }
  const baked = data.baked.chains.find((b) => b.id === ch.id).nominal.deepestMm;
  const mine = Math.round(best * 1e5) / 1e2;
  const ok = Math.abs(mine - baked) <= 0.01;
  console.log(`${ok ? 'PASS' : 'FAIL'} ${ch.id}: independent ${mine} mm vs certificate ${baked} mm`);
  ok ? pass++ : fail++;
}
console.log(`\n${pass} passed / ${fail} failed (independent implementation, 2000-frame dense scan, worst pair)`);
process.exit(fail === 0 ? 0 : 1);
