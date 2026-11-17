// Pure geometry / kinematics core for engine-repair-lab.
// No three.js, no DOM, no filesystem, no env vars: runs identically in node and the browser.

export const MARGIN = 0.002;        // keep-out gap (m) that counts as a collision violation
export const CLEAR_TARGET = 0.003;  // gap the repair solver must reach (m)
export const REACH_SLACK = 0.002;

function bumpRateMax(repair, k) {
  let m = 0;
  if (repair) for (const b of repair.bumps) if (b.j === k) m += Math.PI * Math.abs(b.delta) / (b.t1 - b.t0);
  return m;
}   // engine prediction distance, see README

const TAU = Math.PI * 2;

export function makeChain(spec) {
  const L = spec.L.slice();
  const r = spec.r.slice();
  const n = L.length;
  if (r.length !== n || spec.q0.length !== n || spec.amp.length !== n ||
      spec.freq.length !== n || spec.phase.length !== n) {
    throw new Error('chain spec arrays must have equal length');
  }
  let reach = 0;
  for (const v of L) reach += v;
  return {
    id: spec.id, n, L, r, q0: spec.q0.slice(), amp: spec.amp.slice(),
    freq: spec.freq.slice(), phase: spec.phase.slice(),
    base: { x: spec.base ? spec.base[0] : 0, y: spec.base ? spec.base[1] : 0 },
    T: spec.T, reach,
    omegaMax: spec.amp.map((a, k) => Math.abs(a) * TAU * Math.abs(spec.freq[k])),
  };
}

// Raised-cosine bump: value and derivative are continuous, support [t0,t1].
export function bumpValue(t, b) {
  if (t <= b.t0 || t >= b.t1) return 0;
  return 0.5 * b.delta * (1 - Math.cos(TAU * (t - b.t0) / (b.t1 - b.t0)));
}
export function bumpRate(t, b) {
  if (t <= b.t0 || t >= b.t1) return 0;
  return 0.5 * b.delta * TAU / (b.t1 - b.t0) * Math.sin(TAU * (t - b.t0) / (b.t1 - b.t0));
}

export function jointAngle(chain, t, k, repair) {
  let a = chain.q0[k] + chain.amp[k] * Math.sin(TAU * chain.freq[k] * t + chain.phase[k]);
  if (repair) for (const b of repair.bumps) if (b.j === k) a += bumpValue(t, b);
  return a;
}

export function jointRate(chain, t, k, repair) {
  let w = chain.amp[k] * TAU * Math.abs(chain.freq[k]); // magnitude bound, phase ignored
  if (repair) for (const b of repair.bumps) if (b.j === k) w += Math.abs(bumpRate(t, b));
  return w;
}

// Forward kinematics for a planar serial chain: returns the 18 joint points and cumulative angles.
export function frames(chain, t, repair) {
  const pts = new Array(chain.n + 1);
  const ang = new Array(chain.n);
  let x = chain.base.x, y = chain.base.y, a = 0;
  pts[0] = { x, y };
  for (let k = 0; k < chain.n; k++) {
    a += jointAngle(chain, t, k, repair);
    ang[k] = a;
    x += chain.L[k] * Math.cos(a);
    y += chain.L[k] * Math.sin(a);
    pts[k + 1] = { x, y };
  }
  return { pts, ang };
}

export function segSegDist(p1, p2, p3, p4) {
  const d1x = p2.x - p1.x, d1y = p2.y - p1.y;
  const d2x = p4.x - p3.x, d2y = p4.y - p3.y;
  const rx = p1.x - p3.x, ry = p1.y - p3.y;
  const a = d1x * d1x + d1y * d1y, e = d2x * d2x + d2y * d2y, f = d2x * rx + d2y * ry;
  let s, t;
  if (a <= 1e-18 && e <= 1e-18) { s = 0; t = 0; }
  else if (a <= 1e-18) { s = 0; t = clamp01(f / e); }
  else {
    const c = d1x * rx + d1y * ry;
    if (e <= 1e-18) { t = 0; s = clamp01(-c / a); }
    else {
      const b = d1x * d2x + d1y * d2y;
      const den = a * e - b * b;
      s = den !== 0 ? clamp01((b * f - c * e) / den) : 0;
      t = (b * s + f) / e;
      if (t < 0) { t = 0; s = clamp01(-c / a); }
      else if (t > 1) { t = 1; s = clamp01((b - c) / a); }
    }
  }
  const cx = p1.x + d1x * s - (p3.x + d2x * t);
  const cy = p1.y + d1y * s - (p3.y + d2y * t);
  return Math.hypot(cx, cy);
}

function clamp01(v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

// Signed gap between capsule i and capsule j at time t (negative = penetration).
export function gap(chain, t, i, j, repair, fr) {
  const F = fr || frames(chain, t, repair);
  return segSegDist(F.pts[i], F.pts[i + 1], F.pts[j], F.pts[j + 1]) - chain.r[i] - chain.r[j];
}

// Conservative per-link endpoint speed and acceleration bounds at time t (m/s, m/s^2).
// Distances are bounded above by (measured distance + slack), where slack covers how far the
// geometry can move across an interval of half-width h. Rigorous but deliberately loose.
export function speedAccelBound(chain, t, h, repair) {
  const F = frames(chain, t, repair);
  const om = [], al = [];
  let omegaSum = 0;
  for (let k = 0; k < chain.n; k++) {
    const w = jointRate(chain, t, k, repair);
    let a = chain.amp[k] * Math.pow(TAU * chain.freq[k], 2);
    if (repair) for (const b of repair.bumps) {
      a += 0.5 * Math.abs(b.delta) * Math.pow(TAU / (b.t1 - b.t0), 2);
    }
    om.push(w); al.push(a); omegaSum += w;
  }
  const slack = omegaSum * chain.reach * h;
  const v = new Array(chain.n), acc = new Array(chain.n);
  for (let i = 0; i < chain.n; i++) {
    let sv = 0, sa = 0;
    for (let k = 0; k <= i; k++) {
      const d = Math.hypot(F.pts[i + 1].x - F.pts[k].x, F.pts[i + 1].y - F.pts[k].y) + slack;
      sv += om[k] * d;
      sa += (al[k] + om[k] * om[k]) * d;
    }
    v[i] = sv; acc[i] = sa;
  }
  return { v, a: acc };
}

// Continuous-time violation certificate.
// Coarse pass (N0 knots) uses a rigorous Lipschitz bound to discard pairs; candidate intervals are
// then refined adaptively. 'unresolved' intervals are reported, never silently treated as clear.
export function certify(chain, repair, opts = {}) {
  const margin = opts.margin != null ? opts.margin : MARGIN;
  const N0 = opts.N0 || 1200;
  const minWidth = opts.minWidth || 1e-4;
  const maxDepth = opts.maxDepth || 22;
  const h0 = chain.T / N0;
  const t0 = performanceNow();
  const violations = [];
  let clearPairs = 0, candidates = 0, unresolved = 0, evals = 0;

  for (let i = 0; i < chain.n; i++) {
    for (let j = i + 2; j < chain.n; j++) {
      let worst = Infinity;
      for (let s = 0; s <= N0; s++) {
        evals++;
        const g = gap(chain, s * h0, i, j, repair);
        if (g < worst) worst = g;
      }
      // Lipschitz bound: speed of endpoint of link i plus link j, times interval half-width.
      // Global (time-independent) upper bound: every lever arm is at most the reach.
      let Lg = 0;
      for (const k of [i, j]) {
        for (let m = 0; m <= k; m++) {
          const w = chain.amp[m] * TAU * Math.abs(chain.freq[m]) + bumpRateMax(repair, m);
          Lg += w * chain.reach;
        }
      }
      if (worst - Lg * (h0 / 2) >= margin) { clearPairs++; continue; }
      candidates++;
      const stack = [[0, chain.T, 0]];
      while (stack.length) {
        const [a, b, depth] = stack.pop();
        const m = 0.5 * (a + b), half = 0.5 * (b - a);
        evals++;
        const gm = gap(chain, m, i, j, repair);
        const Lm = speedAccelBound(chain, m, half, repair);
        const Li = Lm.v[i] + Lm.v[j] + (Lm.a[i] + Lm.a[j]) * half;
        if (gm + Li * half < margin) {
          // guaranteed violation somewhere in [a,b]; report a sampled witness inside it
          let bestT = m, bestG = Infinity;
          for (let s = 0; s <= 24; s++) {
            const tt = a + (b - a) * (s / 24);
            evals++;
            const g = gap(chain, tt, i, j, repair);
            if (g < bestG) { bestG = g; bestT = tt; }
          }
          violations.push({ i, j, t0: round6(a), t1: round6(b), witnessT: round6(bestT), witnessGap: round6(bestG), lip: round4(Li) });
        } else if (gm - Li * half >= margin) {
          // provably clear on [a,b]
        } else if ((b - a) <= minWidth || depth >= maxDepth) {
          unresolved++;
        } else {
          stack.push([a, m, depth + 1], [m, b, depth + 1]);
        }
      }
    }
  }
  violations.sort((x, y) => x.i - y.i || x.j - y.j || x.t0 - y.t0);
  // merge touching guaranteed intervals of the same pair into one collision event
  const events = [];
  for (const v of violations) {
    const p = events[events.length - 1];
    if (p && p.i === v.i && p.j === v.j && v.t0 <= p.t1 + 2 * h0) {
      p.t1 = Math.max(p.t1, v.t1);
      p.intervals++;
      if (v.witnessGap < p.witnessGap) { p.witnessGap = v.witnessGap; p.witnessT = v.witnessT; }
    } else {
      events.push({ i: v.i, j: v.j, t0: v.t0, t1: v.t1, witnessT: v.witnessT, witnessGap: v.witnessGap, lip: v.lip, intervals: 1 });
    }
  }
  return {
    violations: events,
    intervals: violations.length, clearPairs, candidates, unresolved, evals,
    pairs: chain.n * (chain.n - 1) / 2 - (chain.n - 1),
    ms: round3(performanceNow() - t0),
    nViolations: events.length,
  };
}

// Constructive repair: find the largest uniform attenuation of the commanded motion
// (amplitude scale lambda in [0,1]) for which the whole horizon is certified collision-free.
// Monotone in practice: lambda = 0 is the rest pose, which must itself be clear.
export function attenuate(chain, lambda) {
  const c = { ...chain, amp: chain.amp.map((a) => a * lambda) };
  return c;
}

export function repairChain(chain, opts = {}) {
  const steps = opts.steps || 24;
  const certOpts = { N0: opts.N0 || 500, minWidth: opts.minWidth || 1e-4, margin: opts.margin };
  const probes = [];
  const feasible = (lam) => {
    const c = certify(attenuate(chain, lam), null, certOpts);
    probes.push({ lambda: round4(lam), violations: c.nViolations, unresolved: c.unresolved });
    return c.nViolations === 0 && c.unresolved === 0;
  };
  if (!feasible(0)) {
    return { lambda: 0, feasible: false, probes, deviations: null, residual: null };
  }
  // coarse scan for the largest feasible lambda on a grid, then bisect the bracket
  let best = 0;
  for (let s = 1; s <= steps; s++) {
    const lam = s / steps;
    if (!feasible(lam)) break;
    best = lam;
  }
  let lo = best, hi = Math.min(1, best + 1 / steps);
  for (let it = 0; it < 8; it++) {
    const mid = 0.5 * (lo + hi);
    if (feasible(mid)) lo = mid; else hi = mid;
  }
  const lambda = Math.floor(lo * 1e4) / 1e4;
  const repaired = attenuate(chain, lambda);
  const finalCert = certify(repaired, null, { N0: opts.N0Final || 900, minWidth: certOpts.minWidth });
  const dev = pathDeviation(chain, { bumps: [], lambda }, opts.N || 1200);
  return {
    lambda, feasible: finalCert.nViolations === 0 && finalCert.unresolved === 0,
    probes, residual: { violations: finalCert.nViolations, unresolved: finalCert.unresolved },
    deviations: dev,
  };
}

function minGapSampled(chain, repair, i, j, a, b, n) {
  let g = Infinity;
  for (let s = 0; s <= n; s++) {
    const t = a + (b - a) * (s / n);
    const v = gap(chain, t, i, j, repair);
    if (v < g) g = v;
  }
  return g;
}

// End-effector path deviation caused by the repair, relative to the chain reach.
export function pathDeviation(chain, repair, N = 2000) {
  const rep = attenuate(chain, repair && repair.lambda != null ? repair.lambda : 1);
  let maxAbs = 0, sumAbs = 0;
  const devs = [];
  for (let s = 0; s <= N; s++) {
    const t = chain.T * s / N;
    const a = frames(chain, t, null).pts[chain.n];
    const b = frames(rep, t, repair).pts[chain.n];
    const d = Math.hypot(a.x - b.x, a.y - b.y);
    devs.push(d);
    if (d > maxAbs) maxAbs = d;
    sumAbs += d;
  }
  devs.sort((x, y) => x - y);
  return {
    maxMm: round3(maxAbs * 1000),
    p95Mm: round3(devs[Math.floor(N * 0.95)] * 1000),
    meanMm: round3((sumAbs / (N + 1)) * 1000),
    maxRelReachPct: round4((maxAbs / chain.reach) * 100),
    p95RelReachPct: round4((devs[Math.floor(N * 0.95)] / chain.reach) * 100),
  };
}

// Deterministic fingerprint that the browser can recompute without node:crypto.
export function fingerprint(value) {
  const str = typeof value === 'string' ? value : stableStringify(value);
  let h = 0x811c9dc5;
  for (let i = 0; i < str.length; i++) {
    h ^= str.charCodeAt(i);
    h = Math.imul(h, 0x01000193) >>> 0;
  }
  return h.toString(16).padStart(8, '0') + ':' + str.length.toString(16);
}

export function stableStringify(v) {
  if (v === null || typeof v !== 'object') return JSON.stringify(v);
  if (Array.isArray(v)) return '[' + v.map(stableStringify).join(',') + ']';
  const keys = Object.keys(v).sort();
  return '{' + keys.map((k) => JSON.stringify(k) + ':' + stableStringify(v[k])).join(',') + '}';
}

function performanceNow() {
  return typeof performance !== 'undefined' ? performance.now() : Date.now();
}
function round3(v) { return Math.round(v * 1e3) / 1e3; }
function round4(v) { return Math.round(v * 1e4) / 1e4; }
function round6(v) { return Math.round(v * 1e6) / 1e6; }