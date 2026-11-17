## 2026-11-17 18:32 · 3D 新项目（engine-repair-lab，B 档，已发布）

- 做了什么：`llinok111/engine-repair-lab` —— 把连续时间碰撞证书**降级为定位器**，最终裁决交给外部物理引擎 Rapier 0.14.0。
  `src/core.mjs`（纯几何，无 three/DOM）：平面 17 link 铰接链 FK + capsule 对 gap + **区间 Lipschitz 证书**
  （粗扫丢弃可证安全区间、二分到可证违例/可证安全/`unresolved`，绝不把 unresolved 当安全）。
  修复是构造性的：在 [0,1] 上求**最大一致衰减 λ**（网格扫 + 二分），使整段 2 s 全时域证书归零；
  `bench/engine.mjs` 用 Rapier 运动学 capsule 体（240 Hz）独立重放，逐对读 `solverContactDist`，
  每 60 步做一次 720 对全扫。浏览器 `index.html + main.js`（three 0.170.0 importmap）动画重放 6 条链，
  滑杆改 λ，按钮在浏览器里用同一 `N0=600` 重算证书并与 node 烘焙的 FNV 指纹比对。
- 结果：**published**（repo public，pushed_at 2026-11-17T10:31:43Z）
- 真实数字（bench/result.json、bench/engine-result.json、verify 输出）：
  - 6 链 × 17 link × 120 非相邻对 = **720 对**；标称运动 **19 个事件 / 19 对**，最深 **-38.1 mm**。
  - 证书全时域中位 **85.482 ms**（min 73.455 / p95 99.610 / RSD **8.99%**），冷启动 **108.031 ms**；
    基线 240 Hz 离散扫 3.532 ms（RSD 36.98%，绝对值小、被计时/GC 噪声主导，README 已解释）→ 证书贵 24.2×。
  - 修复 λ = 0.8017/0.7355/0.6695/0.7824/0.7724/0.9917；残差 **0 事件 / 0 unresolved**（6/6）；
    末端偏差中位 **6.83%**、p95 9.29%、max 11.47%、min 0.19% reach。
  - Rapier：**5,760 步**（240 Hz × 480 步 × 6 链 × 2 次运行）；标称命中对 **3/4/5/3/3/1 = 19**，
    深度 -37.5/-37.5/-38.1/-37.5/-37.5/-3.13 mm，与证书 **≤0.05 mm** 一致；修复后 **6/6 链 0 接触、0.00 mm**。
  - 帧率扫描：4× 速度下证书 **7** 事件，240 Hz 扫只中 **5**（漏 2）；1×–3× 都中 1/1。
  - 尺度不变性：1e-3×/0.1×/1×/10× 结论一致（事件数相同、深度按比例），margin 随尺度传入；
    100×/1000× 时保守 slack 使区间停在 `unresolved`，README 已披露不算作安全。
- 质量闸门：G1–G9 全过（G5 证书 RSD 8.99%<10%；基线 RSD 36.98% 已在 README 解释；G7 删 data 重建 sha256 同为 `6deeb96c45845ebc`；G8 无 playwright/npx/截图）。
- 发布后自证 3/3：`gh api` → `false` / `2026-11-17T10:31:43Z`；`/tmp/smoke` 干净 clone 跑 gen+verify(26/26)+bench 全过并已删除；
  README 前 40 行含「承诺+怎么跑+证据」。
- 独立 verifier：已起只读子 agent（见本轮报告结论）；它不改任何文件、不做 git 写操作。
- 上游回馈：**0 个**。这是原创工具（`Upstream: none`），Rapier / three.js 侧没有「本该修但没修」的缺陷，
  按契约允许 0，未写 `state/upstream-pr.json`。
- 未完成 / 下一步：
  1. **修复只用一致衰减**（单标量 λ），不是路径最小扰动：chain-3 需要 λ=0.6695（末端偏差 11.47%）。
     下一步做**关节空间 detour**（最小幅值 bump + 符号搜索），把偏差压到 ≤2%，仍由 Rapier 裁决。
  2. 目前只支持平面链；3D 链需要在 `frames()` 里换四元数，证书逻辑与维度无关。
  3. 引擎只在接触流形存在时能量化穿透（Rapier prediction distance 内）；正间隙由证书给界，README 已写。
  4. `unresolved` 在 100×/1000× 尺度上会出现（保守 slack 所致）：可改为按对局部几何收紧 Lipschitz 常数。
  5. 浏览器端未做真实无头浏览器交互确认（禁止 playwright/puppeteer），只验了 200 + importmap + node 端指纹一致路径。
