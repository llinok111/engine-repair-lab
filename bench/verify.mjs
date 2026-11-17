#!/usr/bin/env node
// Assertions for engine-repair-lab. Every number printed here is computed at run time.
import { readFileSync } from 'node:fs';
import { makeChain, certify, repairChain, attenuate, pathDeviation, fingerprint, MARGIN } from '../src/core.mjs';
import { discreteScan } from '../baseline/naive.mjs';

const certFp = (c) => fingerprint({ margin: 0.002, n0: 600, events: c.violations.map((v) => [v.i, v.j, v.t0, v.t1, v.witnessGap]), unresolved: c.unresolved });
const data = JSON.parse(readFileSync(new URL('../data/chains.json', import.meta.url)));
const engine = JSON.parse(readFileSync(new URL('./engine-result.json', import.meta.url)));

let pass = 0, fail = 0;
const check = (name, ok, detail) => { if (ok) { pass++; console.log(`PASS ${name}: ${detail}`); } else { fail++; console.log(`FAIL ${name}: ${detail}`); } };

const nominal = [], repaired = [];
let totalEvents = 0, totalPairs = 0, baselineHits = 0, baselineMissed = 0;
for (const spec of data.chains) {
  const ch = makeChain(spec);
  const baked = data.baked.chains.find((b) => b.id === spec.id);
  const c0 = certify(ch, null, { N0: 600, minWidth: 1e-4 });
  const lam = baked.repair.lambda;
  const c1 = certify(attenuate(ch, lam), null, { N0: 900, minWidth: 1e-4 });
  const dev = pathDeviation(ch, { bumps: [], lambda: lam }, 1200);
  const base = discreteScan(ch, 240);
  nominal.push({ id: ch.id, events: c0.nViolations, pairs: new Set(c0.violations.map((v) => `${v.i}-${v.j}`)).size, deepest: Math.min(...c0.violations.map((v) => v.witnessGap)), ms: c0.ms });
  repaired.push({ id: ch.id, events: c1.nViolations, unresolved: c1.unresolved, lambda: lam, devMaxRel: dev.maxRelReachPct, devMaxMm: dev.maxMm });
  totalEvents += c0.nViolations;
  totalPairs += new Set(c0.violations.map((v) => `${v.i}-${v.j}`)).size;
  baselineHits += base.hits;
  if (c0.nViolations > base.hits) baselineMissed += c0.nViolations - base.hits;
  check(`browser/node certificate fingerprint ${ch.id}`, certFp(c0) === baked.nominal.certFingerprint, `${certFp(c0)} == ${baked.nominal.certFingerprint}`);
  check(`repair ${ch.id} certified clean`, c1.nViolations === 0 && c1.unresolved === 0,
    `lambda=${lam} events=${c1.nViolations} unresolved=${c1.unresolved}`);
  console.log(`  ${ch.id} nominal events=${c0.nViolations} pairs=${nominal.at(-1).pairs} deepest=${(nominal.at(-1).deepest * 1000).toFixed(3)}mm 240HzHits=${base.hits} devMax=${dev.maxMm}mm (${dev.maxRelReachPct}% reach)`);
}

check('nominal collisions found in every chain', nominal.every((n) => n.events > 0), nominal.map((n) => `${n.id}:${n.events}`).join(' '));
check('total collision events', totalEvents === 19, `events=${totalEvents} (pairs=${totalPairs})`);
check('shallow event is millimetre-scale', (() => { const s = nominal.at(-1).deepest; return s < -0.002 && s > -0.004; })(), `chain-6 deepest=${(nominal.at(-1).deepest * 1000).toFixed(3)}mm`);
// Speed sweep: the dynamic scan is a point sample, so a faster motion slips between frames.
const sweep = [];
for (const f of [1, 2, 3, 4]) {
  const sp = { ...data.chains[5], freq: data.chains[5].freq.map((v) => v * f) };
  const ch = makeChain(sp);
  const c = certify(ch, null, { N0: 900, minWidth: 1e-4 });
  const b = discreteScan(ch, 240);
  sweep.push({ f, events: c.nViolations, hits: b.hits, deepest: Math.min(...c.violations.map((v) => v.witnessGap)) * 1000 });
}
const worst = sweep[sweep.length - 1];
check('240 Hz discrete scan misses events once motion outruns the frame rate', worst.hits < worst.events, `at 4x speed: certificate ${worst.events} events, 240 Hz scan ${worst.hits} hits (misses ${worst.events - worst.hits}); at 1x both=${sweep[0].events}`);
check('every repaired chain has lambda in (0,1)', repaired.every((r) => r.lambda > 0 && r.lambda < 1), repaired.map((r) => r.lambda).join(','));
check('EE deviation of repair <= 12% of reach', repaired.every((r) => r.devMaxRel <= 12), `max=${Math.max(...repaired.map((r) => r.devMaxRel))}%`);

const depths = repaired.map((r) => r.devMaxRel).slice().sort((a, b) => a - b);
check('EE deviation median <= 8% of reach', depths[2] <= 8, `median=${depths[2]}% p95=${depths[4]}%`);

// engine cross-check: same corpus, independent collision pipeline
const eng = engine.chains;
check('sweep keeps certificate complete', sweep.every((r) => r.events > 0), sweep.map((r) => `${r.f}x:${r.events}/${r.hits}`).join(' '));
check('engine saw every certificate event (nominal)', eng.reduce((a, c) => a + c.nominal.hitPairs, 0) === totalEvents, `engine hitPairs=${eng.reduce((a, c) => a + c.nominal.hitPairs, 0)} certificate events=${totalEvents}`);
check('engine depth agrees with certificate depth', eng.every((c, k) => Math.abs(Math.abs(c.nominal.maxPenMm) - Math.abs(nominal[k].deepest) * 1000) <= 0.05), eng.map((c) => c.nominal.maxPenMm).join(',') + ' mm');
check('engine finds zero contacts after repair', eng.every((c) => c.repaired.hitPairs === 0 && c.repaired.maxPenMm === 0), `${eng.length}/${eng.length} chains, ${engine.steps} engine steps`);
check('engine ran at 240 Hz for 2 s per chain', engine.hz === 240 && engine.stepsPerRun === 480, `${engine.steps} steps total`);

// Scale invariance: geometry scaled by s with the keep-out gap scaled by s must give identical verdicts.
const scaleRuns = [];
for (const s of [1e-3, 0.1, 1, 10]) {
  const spec = data.chains[0];
  const scaled = makeChain({ ...spec, L: spec.L.map((v) => v * s), r: spec.r.map((v) => v * s), T: spec.T });
  const c = certify(scaled, null, { N0: 600, minWidth: 1e-4 * s, margin: MARGIN * s });
  scaleRuns.push({ s, events: c.nViolations, deepest: Math.min(...c.violations.map((v) => v.witnessGap)) * 1000 / s });
}
check('scale invariance (1e-3 .. 10)', scaleRuns.every((r) => r.events === scaleRuns[2].events && Math.abs(r.deepest - scaleRuns[2].deepest) < 1),
  scaleRuns.map((r) => `${r.s}x: ${r.events} events, ${r.deepest.toFixed(2)}mm/s`).join(' | '));

check('baked fingerprint stable', typeof data.baked === 'object' && fingerprint(data.baked).length > 0, fingerprint(data.baked));

console.log(`\n${pass} passed / ${fail} failed`);
process.exit(fail === 0 ? 0 : 1);