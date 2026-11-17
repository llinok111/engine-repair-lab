#!/usr/bin/env node
// Timing: continuous-time certificate vs the 240 Hz discrete scan, 1 warmup + 7 measured reps.
import { readFileSync, writeFileSync } from 'node:fs';
import { makeChain, certify } from '../src/core.mjs';
import { discreteScan } from '../baseline/naive.mjs';

const data = JSON.parse(readFileSync(new URL('../data/chains.json', import.meta.url)));
const chains = data.chains.map(makeChain);
const REPS = 7, WARM = 1;
const t = () => Number(process.hrtime.bigint()) / 1e6;

function stats(xs) {
  const s = xs.slice().sort((a, b) => a - b);
  const med = s[Math.floor(s.length / 2)];
  const mean = xs.reduce((a, b) => a + b, 0) / xs.length;
  const sd = Math.sqrt(xs.reduce((a, b) => a + (b - mean) ** 2, 0) / xs.length);
  return { min: r(s[0]), median: r(med), p95: r(s[Math.min(s.length - 1, Math.floor(s.length * 0.95))]), stddev: r(sd), relStddevPct: r((sd / mean) * 100) };
}
const r = (v) => Math.round(v * 1e3) / 1e3;

const out = { tool: 'engine-repair-lab', generatedBy: 'bench/bench.mjs', reps: REPS, warmup: WARM, note: 'times in ms, one full horizon (2 s, 6 chains) per rep', chains: [] };
let coldCert = null, coldBase = null;
const certAll = [], baseAll = [], eventsAll = [], hitsAll = [];
for (let c = 0; c < chains.length; c++) {
  const ch = chains[c];
  const cert = [], base = [];
  for (let i = 0; i < WARM + REPS; i++) {
    const t0 = t(); const cv = certify(ch, null, { N0: 600, minWidth: 1e-4 }); const dt = t() - t0;
    if (i === 0) { if (coldCert === null) coldCert = dt; } else cert.push(dt);
    if (i === 0) eventsAll.push(cv.nViolations);
    const t1 = t(); const bv = discreteScan(ch, 240); const db = t() - t1;
    if (i === 0) { if (coldBase === null) coldBase = db; } else base.push(db);
    if (i === 0) hitsAll.push(bv.hits);
  }
  certAll.push(...cert); baseAll.push(...base);
  out.chains.push({ id: ch.id, certificate: stats(cert), discrete240: stats(base), coldCertificateMs: r(cert[0] ?? 0) });
}
out.aggregate = { certificate: stats(certAll), discrete240: stats(baseAll), coldCertificateMs: r(coldCert), coldDiscrete240Ms: r(coldBase), speedRatioDiscreteVsCertificate: r(stats(baseAll).median / stats(certAll).median) };
out.sweep = { certificateEvents: eventsAll.reduce((a, b) => a + b, 0), discrete240Hits: hitsAll.reduce((a, b) => a + b, 0) };
const body = JSON.stringify(out, null, 2) + '\n';
writeFileSync(new URL('./result.json', import.meta.url), body);
console.log(JSON.stringify(out.aggregate, null, 1));
console.log(JSON.stringify(out.sweep));
