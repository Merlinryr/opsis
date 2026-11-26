# M02: SDK

**State:** In progress
**Goal:** Ship the M01 tracker as an npm SDK: a TypeScript API over the Rust/WASM core, with thin adapters for Three.js and Babylon.

## Relations
- Depends on: [M01](M01_SLAM.md)
- Enables: [M00](M00_WEBAR_LIBRARY.md)

## Target flow
```
 Browser                 TS layer (@opsis/core)                  Rust/WASM core
 ───────                 ──────────────────────                  ──────────────
 getUserMedia  ──video──▶ camera.ts ──frame (canvas)───────────▶ GPU front end
 devicemotion  ──imu────▶ sensors.ts ─gravity, accel, time─────▶   (WebGPU | WebGL,
 orientation   ─────────▶ (permissions, iOS prompt)               private context)
                          loop.ts (rAF / video frame callback)        │
                                 ▲                                    ▼
                                 └──── pose, state, scale ◀──── SLAM tracker (CPU)
                                 │
                                 ▼
                 onFrame({ pose, projection, state, video })
                                 │
             ┌───────────────────┼────────────────────┐
             ▼                   ▼                    ▼
   @opsis/three          @opsis/babylon         plain WebGL/WebGPU
   (camera matrix +      (same)                 (use core directly)
    video background)
```
- Rust sees pixels, timestamps and sensor numbers only. Camera, permissions, sensors and orientation are TS.
- The SDK runs vision on its own private GPU context. Apps draw scene and video background in their own engine.
- `@opsis/core` owns the frame loop and timing (one source of truth). Adapters only copy matrices.
- No worker in v0.1: `devicemotion` fires on the main thread only, and the frame budget already fits.

## Architecture review (2026-11-26)
Verdict: not spaghetti. 3.7k lines, dependencies point one way (`slam` → `geom`, `orb`, `fast`; nothing pure imports web code). The mess is in placement, not coupling:
1. Pure and web code share one flat `src/`, split only by `cfg(target_arch)`.
2. `web.rs` mixes the public `Tracker` with parity-test helpers (`test_frame`, `max_diff`).
3. The SDK's real logic (camera, sensors, orientation, loop) lives in the demo page `camera.html`.
4. `slam.rs` is 880 lines of code plus 425 of tests: tracking, keyframes, epipolar, BA, scale and relocalization in one file.

Decision: a Cargo + pnpm monorepo, the layout used by wgpu, rapier and most wasm SDKs. Two crates, because the pure core must build and test without web deps, and that boundary should be enforced by the compiler, not `cfg`.
```
crates/
  core/      opsis-core: image, fast, orb, readback (unpack), geom, imu, slam/  — host tests, replay
    src/slam/  mod.rs (state + update), ba.rs, epipolar.rs, reloc.rs, scale.rs, tests.rs
    tests/replay.rs
  wasm/      opsis (wasm-bindgen): tracker.rs (Tracker), parity.rs, gpu.rs, webgl.rs, webgpu.rs
packages/    (next: R2-R4)
  core/      @opsis/core   TS: camera, sensors, orientation, loop, typed onFrame
  three/     @opsis/three
  babylon/   @opsis/babylon
examples/    camera.html (HUD, record, ?flags), index.html (parity); camera.html moves onto @opsis/core with R3
tools/       serve.mjs, phone.ps1, build-web.ps1, replay-seeds.ps1
recordings/  docs/   (unchanged)
```
- Move, don't rewrite: each step must keep replays byte-identical.
- No new abstractions: `Gpu` stays the one backend interface; `Slam` keeps its API.

## Requirements
- [x] R1: Repo follows the layout above; `crates/core` has no wasm or web-sys dependency.
- [ ] R2: `@opsis/core` exposes `start(options)`, `stop()`, `onFrame(cb)` with `{ pose, projection, state, scale, video }`; no Rust type leaks into the public TS API.
- [ ] R3: Camera, permissions (incl. iOS motion prompt), sensors and orientation handling live in `@opsis/core`, not in example pages.
- [ ] R4: `@opsis/three` and `@opsis/babylon` each stay under ~100 lines and only map the frame to the engine camera and video background.
- [ ] R5: Works on both WebGPU and WebGL backends from the same API.
- [ ] R6: Ships as ESM + `.d.ts` with the wasm and no Rust source (M00 R8).

## Verification (built right)
- [x] V1: `cargo test -p opsis-core` passes on the host with replays identical to before the move.
- [x] V2: `cargo tree -p opsis-core` shows no wasm-bindgen, web-sys or wgpu.
- [ ] V3: `pnpm -r build` and `tsc --noEmit` pass; published tarball (`pnpm pack`) contains no `.rs` file.
- [x] V4: `examples/index.html` (parity) PASS on desktop WebGL2 and WebGPU.
- [ ] V5: Release wasm gzip ≤ the M00 R9 budget (today 127 KB).

## Validation (built the right thing)
- [ ] VA1: Owner builds a fresh Three.js page and a fresh Babylon page from the README only, on iPhone; the cube stays anchored while walking, as in M01 VA1.

## Log
- 2026-11-26: Created. Flow from the SDK design discussion; architecture review above. Not started.
- 2026-11-26: Library named Opsis. R1 done: Cargo workspace `crates/core` (`opsis-core`, no dependencies) + `crates/wasm` (`opsis`, output `pkg/opsis.js`); `slam.rs` split into `slam/{mod,ba,epipolar,reloc,scale,tests}.rs`; parity helpers out of the tracker (`parity.rs`, `tracker.rs`); `web/` → `examples/`, `scripts/` + `serve.mjs` → `tools/`; env vars `WEBAR_*` → `OPSIS_*`. Self-explanatory comments removed; tuning evidence, shortcuts and conventions kept.
  - Replay paths now resolve from the repo root (cargo runs tests from the crate dir).
  - Checked: 38 host tests pass; all 12 recordings replay identical to before (frame lines and metrics, timing excluded); parity PASS on desktop WebGL2 + WebGPU; wasm 126.4 KB gzip; `camera.html` loads without console errors (not run on a phone).
