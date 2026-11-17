#!/usr/bin/env node
// External-engine replay: Rapier 3D steps the same kinematic geometry that src/core.mjs computes.
// The engine never sees the certificate: it only gets link transforms, so its verdict is independent.
import { readFileSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { makeChain, frames, attenuate, gap } from '../src/core.mjs';

let RAPIER;
try { RAPIER = (await import('@dimforge/rapier3d-compat')).default; }
catch { console.error('missing @dimforge/rapier3d-compat: run `npm i --cache /tmp/npm-cache @dimforge/rapier3d-compat@0.14.0`'); process.exit(2); }
await RAPIER.init();

const HZ = 240, STEPS = 480; // 2.0 s horizon at 240 Hz
const data = JSON.parse(readFileSync(new URL('../data/chains.json', import.meta.url)));
const result = { engine: 'rapier3d-compat', version: '0.14.0', hz: HZ, stepsPerRun: STEPS, chains: [] };

function run(ch, lambda) {
  const motion = attenuate(ch, lambda);
  const world = new RAPIER.World({ x: 0, y: 0, z: 0 });
  const bodies = [], colliders = [];
  for (let k = 0; k < ch.n; k++) {
    const b = world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased());
    const c = world.createCollider(RAPIER.ColliderDesc.capsule(ch.L[k] / 2, ch.r[k]), b);
    c.setActiveEvents(RAPIER.ActiveEvents.COLLISION_EVENTS);
    c.setActiveCollisionTypes(RAPIER.ActiveCollisionTypes.ALL);
    bodies.push(b); colliders.push(c);
  }
  const pairs = [];
  for (let i = 0; i < ch.n; i++) for (let j = i + 2; j < ch.n; j++) pairs.push([i, j]);
  const track = pairs.map(() => ({ minDist: Infinity, firstHitT: null, deepestT: null }));
  const dt = 1 / HZ;
  for (let s = 0; s <= STEPS; s++) {
    const t = Math.min(ch.T, s * dt);
    const F = frames(motion, t, null);
    for (let k = 0; k < ch.n; k++) {
      const a = F.ang[k], mx = (F.pts[k].x + F.pts[k + 1].x) / 2, my = (F.pts[k].y + F.pts[k + 1].y) / 2;
      bodies[k].setNextKinematicTranslation({ x: mx, y: my, z: 0 });
      const hs = Math.sin((a - Math.PI / 2) / 2), hc = Math.cos((a - Math.PI / 2) / 2);
      bodies[k].setNextKinematicRotation({ x: 0, y: 0, z: hs, w: hc });
    }
    world.step();
    if (s > 0 && s % 60 === 0) {
      // full sweep so nothing outside the flagged set can hide
      for (let p = 0; p < pairs.length; p++) sample(pairs[p], track[p], t, s);
    } else {
      for (let p = 0; p < pairs.length; p++) {
        const [i, j] = pairs[p];
        if (track[p].minDist === Infinity && !flagSet.has(`${i}-${j}`)) continue;
        sample(pairs[p], track[p], t, s);
      }
    }
  }
  function sample([i, j], rec, t, s) {
    world.contactPair(colliders[i], colliders[j], (m) => {
      for (let q = 0; q < m.numSolverContacts(); q++) {
        const d = m.solverContactDist(q);
        if (d < rec.minDist) { rec.minDist = d; rec.deepestT = t; }
        if (d < 0 && rec.firstHitT === null) rec.firstHitT = t;
      }
    });
  }
  let maxPen = 0, hitPairs = 0, firstHit = Infinity;
  for (let p = 0; p < pairs.length; p++) {
    const r = track[p];
    if (r.minDist < 0) { maxPen = Math.min(maxPen, r.minDist); hitPairs++; if (r.firstHitT !== null) firstHit = Math.min(firstHit, r.firstHitT); }
  }
  return { world, hitPairs, maxPenMm: Math.round(maxPen * 1e5) / 1e2, firstHitT: firstHit === Infinity ? null : Math.round(firstHit * 1e6) / 1e6, pairs: pairs.length };
}

const flagSet = new Set();
for (const ch of data.chains.map(makeChain)) {
  const c = (await import('../src/core.mjs')).certify(ch, null, { N0: 600, minWidth: 1e-4 });
  for (const v of c.violations) flagSet.add(`${v.i}-${v.j}`);
}

for (const spec of data.chains) {
  const ch = makeChain(spec);
  const baked = data.baked.chains.find((b) => b.id === spec.id);
  const nominal = run(ch, 1);
  const repaired = run(ch, baked.repair.lambda);
  result.chains.push({
    id: spec.id, lambda: baked.repair.lambda,
    nominal: { hitPairs: nominal.hitPairs, maxPenMm: nominal.maxPenMm, firstHitT: nominal.firstHitT },
    repaired: { hitPairs: repaired.hitPairs, maxPenMm: repaired.maxPenMm, firstHitT: repaired.firstHitT },
  });
  console.log(`${spec.id} nominal hitPairs=${nominal.hitPairs} maxPen=${nominal.maxPenMm}mm  |  repaired hitPairs=${repaired.hitPairs} maxPen=${repaired.maxPenMm}mm`);
}

result.steps = result.chains.length * STEPS * 2;
result.totalSteps = result.chains.reduce((a) => a, 0) + result.steps;
result.maxPenetrationRepairedMm = result.chains.reduce((a, c) => Math.min(a, c.repaired.maxPenMm), 0);
result.repairedPenetratingChains = result.chains.filter((c) => c.repaired.hitPairs > 0).length;
const body = JSON.stringify(result, null, 2) + '\n';
writeFileSync(new URL('./engine-result.json', import.meta.url), body);
console.log(`repaired: ${result.repairedPenetratingChains}/6 chains with any contact, worst depth ${result.maxPenetrationRepairedMm}mm over ${result.steps} engine steps`);
console.log(`sha256(engine-result)=${createHash('sha256').update(body).digest('hex')}`);
