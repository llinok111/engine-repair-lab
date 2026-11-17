// Render + interaction only: every number shown comes from src/core.mjs or from bench outputs.
import * as THREE from 'three';
import { makeChain, frames, attenuate, gap, certify, fingerprint, MARGIN } from './src/core.mjs';

const data = await (await fetch('./data/chains.json')).json();
const engine = await (await fetch('./bench/engine-result.json')).json();
const chains = data.chains.map(makeChain);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
document.getElementById('view').appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0f14);
const camera = new THREE.PerspectiveCamera(45, innerWidth / innerHeight, 0.1, 200);
camera.position.set(0, 1.5, 13);
camera.lookAt(0, 0, 0);
scene.add(new THREE.GridHelper(24, 24, 0x1e2a38, 0x141d27));
scene.add(new THREE.AmbientLight(0xffffff, 1.1));
const key = new THREE.DirectionalLight(0xffffff, 2.0); key.position.set(4, 8, 6); scene.add(key);

const geoCache = new Map();
const meshes = [];
for (let c = 0; c < chains.length; c++) {
  const ch = chains[c];
  const g = new THREE.Group();
  g.position.set((c % 3 - 1) * 7.5, c < 3 ? 2.2 : -2.2, 0);
  for (let k = 0; k < ch.n; k++) {
    const keyG = ch.r[k].toFixed(4) + ':' + ch.L[k].toFixed(4);
    if (!geoCache.has(keyG)) geoCache.set(keyG, new THREE.CapsuleGeometry(ch.r[k], ch.L[k], 4, 10));
    const mat = new THREE.MeshStandardMaterial({ color: new THREE.Color().setHSL(0.52 + 0.32 * (k / ch.n), 0.55, 0.55), roughness: 0.45, metalness: 0.15 });
    const m = new THREE.Mesh(geoCache.get(keyG), mat);
    g.add(m); meshes.push({ c, k, mesh: m, mat });
  }
  const base = new THREE.Mesh(new THREE.SphereGeometry(0.06, 8, 8), new THREE.MeshBasicMaterial({ color: 0x88a0b8 }));
  g.add(base);
  scene.add(g);
}

const $ = (id) => document.getElementById(id);
const bakedById = Object.fromEntries(data.baked.chains.map((b) => [b.id, b]));
const engById = Object.fromEntries(engine.chains.map((e) => [e.id, e]));

let lambda = 0.8017;
function pose(t, lambda) {
  for (let c = 0; c < chains.length; c++) {
    const ch = attenuate(chains[c], lambda);
    const F = frames(ch, t, null);
    for (const item of meshes) {
      if (item.c !== c) continue;
      const a = F.ang[item.k];
      item.mesh.position.set((F.pts[item.k].x + F.pts[item.k + 1].x) / 2, (F.pts[item.k].y + F.pts[item.k + 1].y) / 2, 0);
      item.mesh.rotation.set(0, 0, a - Math.PI / 2);
    }
  }
}
function apply(lambda) {
  let worst = Infinity;
  for (let c = 0; c < chains.length; c++) {
    const ch = attenuate(chains[c], lambda);
    for (let s = 0; s <= 240; s++) {
      const t = (ch.T * s) / 240;
      const Fr = frames(ch, t, null);
      for (let i = 0; i < ch.n; i++) for (let j = i + 2; j < ch.n; j++) {
        const gg = gap(ch, t, i, j, null, Fr);
        if (gg < worst) worst = gg;
      }
    }
    const baked = bakedById[ch.id];
    const okNow = lambda <= baked.repair.lambda + 1e-9;
    for (const item of meshes) if (item.c === c) item.mat.color.set(okNow ? new THREE.Color().setHSL(0.42, 0.6, 0.5) : new THREE.Color().setHSL(0.02, 0.85, 0.55));
  }
  $('lam').textContent = lambda.toFixed(3);
  $('gap').textContent = (worst * 1000).toFixed(2) + ' mm';
  const clear = worst >= MARGIN;
  $('status').textContent = clear ? 'clear at sampled frames' : 'below keep-out gap';
  $('status').className = 'v ' + (clear ? 'ok' : 'bad');
}

$('slider').addEventListener('input', (e) => { lambda = Number(e.target.value); requestAnimationFrame(() => apply(lambda)); });
$('run').addEventListener('click', () => {
  const ch = chains[0];
  const t0 = performance.now();
  const c = certify(ch, null, { N0: 600, minWidth: 1e-4 });
  const ms = performance.now() - t0;
  const fp = fingerprint({ margin: 0.002, n0: 600, events: c.violations.map((v) => [v.i, v.j, v.t0, v.t1, v.witnessGap]), unresolved: c.unresolved });
  const baked = bakedById[ch.id].nominal;
  $('br').textContent = `${c.nViolations} events, ${c.unresolved} unresolved, ${ms.toFixed(1)} ms`;
  $('fp').textContent = fp === baked.certFingerprint ? 'MATCH ' + fp : 'MISMATCH ' + fp;
  $('fp').className = 'v ' + (fp === baked.certFingerprint ? 'ok' : 'bad');
});

$('nodeev').textContent = data.baked.chains.map((b) => `${b.id.slice(-1)}:${b.nominal.events}`).join(' ');
$('enginev').textContent = `${engine.repairedPenetratingChains}/6 chains contact, ${engine.steps} steps`;
$('dev').textContent = data.baked.chains.map((b) => `${b.repair.deviations.maxRelReachPct.toFixed(1)}%`).join(' ');
apply(lambda);

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});
renderer.setAnimationLoop(() => {
  pose(((performance.now() / 1000) * 0.5) % 2, lambda);
  renderer.render(scene, camera);
});
