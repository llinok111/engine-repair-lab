# engine-repair-lab

**Promise (measured, one command):** on 6 chains × 17 capsule links (720 non-adjacent pairs), a
continuous-time certificate locates **19/19 real self-collision events** (`-38.1 mm` worst) in
**85.5 ms median**, a uniform motion-derating repair clears **19/19** (residual 0 events,
0 unresolved), and **Rapier 0.14.0 at 240 Hz replays 5,760 steps with 0 contacts, 0.00 mm
penetration in 6/6 chains** — engine depth agrees with the certificate to **0.05 mm** on every
event. Baseline: the 240 Hz discrete scan (`baseline/naive.mjs`), which is **24× cheaper**
(3.53 ms) but is a point sample: at 4× motion speed it sees **5 of 7** events.

Upstream: none

This is an original tool. It is not a fork and does not copy another repository.

## What it does

1. `src/core.mjs` computes planar serial-chain forward kinematics and a capsule-pair gap, then
   **certifies** collisions over the whole 2 s horizon: a coarse Lipschitz pass discards provably
   clear intervals, and the remaining intervals are bisected until each is provably clear, provably
   violating, or reported as `unresolved` (never silently treated as clear).
2. The certificate is demoted to a **locator**. It says *which pair* and *when*, not "safe".
3. `bench/engine.mjs` rebuilds the same geometry as Rapier kinematic capsule bodies and lets the
   engine's own narrow phase decide: per-pair contact distances every step, full 720-pair sweep
   every 60 steps.
4. The repair is constructive: the largest uniform attenuation λ of the commanded motion for which
   the whole horizon certifies clean, found by grid scan + bisection. λ is bounded by the engine
   verdict, not by the certificate.

## Run it

```bash
node tools/gen-sample.mjs          # deterministic corpus + baked repair (10 s)
node bench/verify.mjs              # 26 assertions, no dependencies
node bench/bench.mjs               # 1 warmup + 7 reps -> bench/result.json
node bench/independent.mjs         # re-derives FK + segment distance without importing core
```

Engine replay needs one pinned package (the only dependency; not vendored):

```bash
npm i --cache /tmp/npm-cache @dimforge/rapier3d-compat@0.14.0
node bench/engine.mjs              # -> bench/engine-result.json
```

Browser: `python3 -m http.server 4173` in this directory, open <http://localhost:4173>. The page
imports `three@0.170.0` from jsDelivr via an import map, animates all six chains, and the slider
changes λ live. "Run the certificate in this browser" recomputes `certify()` with the same
`N0=600` as node and compares an FNV-1a fingerprint with the value baked into `data/chains.json`.

## Evidence (`bench/result.json`, `bench/engine-result.json`)

| measurement | value |
| --- | --- |
| chains × links × non-adjacent pairs | 6 × 17 × 120 = 720 |
| certificate, full horizon | median **85.482 ms**, min 73.455, p95 99.610, RSD **8.99 %** |
| cold start (first call) | **108.031 ms** |
| 240 Hz discrete scan baseline | median **3.532 ms** (RSD 36.98 %, see limits) |
| events found / pairs | **19 / 19**, worst **-38.1 mm** at 240 Hz sampling |
| repaired residual | **0 events, 0 unresolved**, λ ∈ [0.6695, 0.9917] |
| end-effector deviation after repair | median **6.83 %**, p95 9.29 %, max 11.47 %, min 0.19 % of reach |
| Rapier replay | 240 Hz × 480 steps × 6 chains × 2 runs = **5,760 steps** |
| engine on nominal motion | **19 hit pairs** (3/4/5/3/3/1), depth match ≤ 0.05 mm |
| engine after repair | **0/6 chains with any contact, 0.00 mm** |
| frame-rate sweep | at 4× speed: certificate **7** events, 240 Hz scan **5** hits (misses 2) |
| independent reimplementation (no shared code) | **6/6 chains match ≤ 0.01 mm**; chain-6 −3.13 mm |
| scale sweep | identical verdict at 1e-3×, 0.1×, 1×, 10× (6 orders of magnitude) |

## 本轮创新

This closes the loop that the previous rounds left open: earlier certificates only re-proved a
continuous-time safety property, and the "fix" was a number in a report. Here the certificate is
**only a locator** — the final verdict on every repair comes from Rapier's own narrow phase, which
never sees the certificate, and the two independently agree on **19/19 event pairs** and on the
penetration depth to **0.05 mm** while the repair drives the engine's worst contact to **0.00 mm
over 5,760 steps**. What was previously impossible to check offline is now an executable claim:
`node bench/engine.mjs` turns "provably clear" into "the physics engine found no contact".

## Known limits (read before trusting a number)

- **Keep-out gap is an absolute 2 mm** (`MARGIN`), valid for chains with 0.1–5 m reach. Scaling the
  geometry without scaling the margin changes the verdict by construction. The certificate is
  scale-consistent only if you pass `margin` proportional to the scale (bench/verify.mjs does this
  for 1e-3×…10×). At 100× and 1000× the conservative slack term in the Lipschitz bound leaves
  intervals **unresolved** — they are reported, not counted as clear.
- The Lipschitz constant is deliberately loose (it uses each link's reach and a slack term for
  distance drift across the interval). That is why 19 events cost 85 ms rather than 3.5 ms: the
  discrete scan is 24× faster but has no bound between frames, and at 4× motion speed it misses 2
  of 7 events.
- The discrete baseline's RSD is 36.98 % because its absolute cost is ~3.5 ms, dominated by timer
  and GC noise; the certificate's 8.99 % is the meaningful figure.
- The repair is **uniform attenuation only** (no joint-space detour, no time warp). It is minimal
  in the single scalar λ, not in path deviation: chain-3 needs λ = 0.6695, i.e. 11.47 % end-effector
  deviation. A joint-space detour repair is future work.
- The engine measures penetration only through contact manifolds, which exist inside Rapier's
  prediction distance. It cannot report a positive gap of, say, 5 mm; the certificate supplies that
  bound, the engine supplies the independent penetration verdict.
- No static obstacles, no ground plane, no dynamics: bodies are kinematic and follow the core's
  kinematics exactly. The engine is an oracle for collision, not for dynamics.
- Planar chains only (revolute joints about z). A 3D chain would need quaternion kinematics in
  `frames()`; the certificate itself is dimension-agnostic.

## Layout

```
index.html / main.js     renderer + slider; consumes core, never recomputes its own numbers
src/core.mjs             kinematics, capsule gap, certificate, attenuation repair, fingerprint
baseline/naive.mjs       240 Hz discrete scan (the baseline the promise is measured against)
bench/verify.mjs         26 assertions, exit code 0/1
bench/bench.mjs          timing -> bench/result.json
bench/engine.mjs         Rapier replay -> bench/engine-result.json
bench/independent.mjs    second implementation (nested bisection, no core import)
tools/gen-sample.mjs     deterministic corpus + baked λ and certificate fingerprints
data/chains.json         generated; byte-identical on every run (no RNG)
```

MIT licensed, see `LICENSE`.
