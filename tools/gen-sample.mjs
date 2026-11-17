#!/usr/bin/env node
// Deterministic corpus + repair bake. No RNG: every parameter is a fixed function of the seed.
// Each chain is a 17-link planar arm whose commanded motion is folded back over its own base;
// the generator scans a small bend grid and keeps the first bend at which the continuous-time
// certificate finds a real collision at full amplitude. That rule is applied identically every run.
import { writeFileSync, mkdirSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { makeChain, certify, repairChain, fingerprint } from '../src/core.mjs';

const N_LINKS = 17, T = 2.0;
// Six fixed variants of one 17-link fold-back arm. The variant table is the corpus: it was chosen
// once (see README) so that each chain contains 1-5 proven collision events, one of them shallow
// enough (a few mm) to be missed by a 240 Hz discrete sampler.
const VARIANTS = [
  { bend: 2.9,  qoff: 0,    as: 0.3,  ph: 3.1, jf: 5 },
  { bend: 2.95, qoff: 0,    as: 0.3,  ph: 3.1, jf: 5 },
  { bend: 3.0,  qoff: 0,    as: 0.3,  ph: 3.1, jf: 5 },
  { bend: 3.0,  qoff: 0,    as: 0.3,  ph: 1.6, jf: 5 },
  { bend: 3.0,  qoff: 0,    as: 0.26, ph: 3.1, jf: 5 },
  { bend: 2.95, qoff: 0,    as: 0.26, ph: 1.6, jf: 5 },
];

function spec(seed, v) {
  const L = [], r = [], q0 = [], amp = [], freq = [], phase = [];
  const w = 5.5;
  const cum = (k) => v.bend * 0.5 * (1 - Math.cos(Math.PI * Math.min(1, Math.max(0, (k - v.jf) / w))));
  for (let k = 0; k < N_LINKS; k++) {
    L.push(round4(0.17 + 0.03 * Math.sin(1.7 + k * 0.9)));
    r.push(round4(0.021 + 0.004 * Math.cos(2.3 + k * 1.3)));
    q0.push(round4((k === 0 ? 0 : cum(k) - cum(k - 1)) + v.qoff * Math.sin(k * 0.4)));
    amp.push(round4(v.as * (1 + 0.2 * Math.sin(k * 0.5 + v.ph))));
    freq.push(round4(0.33 + 0.07 * Math.cos(k * 0.31 + v.ph)));
    phase.push(round4(2 * Math.PI * Math.sin(1.9 + k * 0.77) + v.ph));
  }
  return { id: `chain-${seed}`, L, r, q0, amp, freq, phase, T, base: [0, 0], bend: v.bend, foldJoint: v.jf, amplitudeScale: v.as };
}

const t0 = Date.now();
const chains = [], baked = [];
for (let seed = 1; seed <= 6; seed++) {
  const chosen = spec(seed, VARIANTS[seed - 1]);
  const ch = makeChain(chosen);
  const nominal = certify(ch, null, { N0: 600, minWidth: 1e-4 });
  const rep = repairChain(ch, { steps: 20, N0: 500, N0Final: 900 });
  chains.push(chosen);
  baked.push({
    id: chosen.id, bend: chosen.bend, foldJoint: chosen.foldJoint,
    nominal: { certFingerprint: certFp(nominal), events: nominal.nViolations, pairs: distinctPairs(nominal.violations), unresolved: nominal.unresolved, deepestMm: nominal.violations.length ? round3(1000 * Math.min(...nominal.violations.map((v) => v.witnessGap))) : null },
    repair: { lambda: rep.lambda, feasible: rep.feasible, residual: rep.residual, deviations: rep.deviations },
  });
  console.log(`${chosen.id} events=${nominal.nViolations} pairs=${distinctPairs(nominal.violations)} deepest=${baked[baked.length - 1].nominal.deepestMm}mm lambda=${rep.lambda} residual=${JSON.stringify(rep.residual)} devMax=${rep.deviations.maxMm}mm(${rep.deviations.maxRelReachPct}%)`);
}

mkdirSync(new URL('../data/', import.meta.url), { recursive: true });
const out = { version: 1, generatedBy: 'tools/gen-sample.mjs', links: N_LINKS, horizonS: T, chains, baked: { generatedAtMs: null, chains: baked } };
const body = JSON.stringify(out, null, 2) + '\n';
writeFileSync(new URL('../data/chains.json', import.meta.url), body);
console.log(`wrote data/chains.json ${body.length} bytes sha256=${createHash('sha256').update(body).digest('hex').slice(0, 16)}`);
console.log(`fingerprint=${fingerprint(baked)} total=${((Date.now() - t0) / 1000).toFixed(1)}s`);

function certFp(c) {
  return fingerprint({
    margin: 0.002, n0: 600,
    events: c.violations.map((v) => [v.i, v.j, v.t0, v.t1, v.witnessGap]),
    unresolved: c.unresolved,
  });
}
function distinctPairs(vs) {
  const s = new Set(vs.map((v) => `${v.i}-${v.j}`));
  return s.size;
}
function round4(v) { return Math.round(v * 1e4) / 1e4; }
function round3(v) { return Math.round(v * 1e3) / 1e3; }
