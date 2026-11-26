# M02: SDK

**State:** In progress
**Goal:** Ship the M01 tracker as an npm SDK: a TypeScript API over the Rust/WASM core, with thin adapters for Three.js and Babylon.

## Relations
- Depends on: [M01](M01_SLAM.md)
- Enables: [M00](M00_WEBAR_LIBRARY.md)

## Target flow
```
 Browser                 TS layer (opsis)                        Rust/WASM core
 ───────                 ──────────────────────                  ──────────────
 getUserMedia  ──video──▶ camera.ts ──frame (canvas)───────────▶ GPU front end
 devicemotion  ──imu────▶ sensors.ts ─gravity, accel, time─────▶   (WebGPU | WebGL,
 orientation   ─────────▶ (permissions, iOS prompt)               private context)
                          loop.ts (rAF / video frame callback)        │
                                 ▲                                    ▼
                                 └──── pose, state, scale ◀──── SLAM tracker (CPU)
                                 │
                                 ▼
                 onFrame(frame: pose, fovY/projectionMatrix, hitTest, state, scale)
                                 │
             ┌───────────────────┼────────────────────┐
             ▼                   ▼                    ▼
   opsis/three           opsis/babylon,         plain WebGL/WebGPU/2D
   (+ A-Frame; camera    opsis/playcanvas       (matrices directly)
    only)                (same)
```
- Rust sees pixels, timestamps and sensor numbers only. Camera, permissions, sensors and orientation are TS.
- The SDK runs vision on its own private GPU context. The camera image (live `<video>`, or with `background: "tracked"` the tracked frame in sync with the pose) sits under a transparent engine canvas with `object-fit: cover`; `fovY`, `projectionMatrix` and `hitTest` match that crop.
- Push (`onFrame`) and pull (`session.frame`) both work, so engines with their own render loop (Babylon, PlayCanvas) need no callback.
- `opsis` owns the frame loop and timing (one source of truth). Adapters only copy the pose and field of view.
- One zero-dependency package with subpath entries (`opsis/three`, `opsis/babylon`) instead of one package per engine: each adapter is ~20 lines, typed structurally (no engine import).
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
packages/
  opsis/     npm `opsis`: src/index.ts (camera, sensors, loop, Frame), three.ts, babylon.ts; wasm/ from build-web
examples/    vanilla, webgl, webgpu, three, babylon (.html), camera.html (diagnostic HUD, record), index.html (parity)
tools/       serve.mjs, phone.ps1, build-web.ps1, replay-seeds.ps1
recordings/  docs/   (unchanged)
```
- Move, don't rewrite: each step must keep replays byte-identical.
- No new abstractions: `Gpu` stays the one backend interface; `Slam` keeps its API.

## Requirements
- [x] R1: Repo follows the layout above; `crates/core` has no wasm or web-sys dependency.
- [x] R2: `opsis` exposes `start(options)` → session with `onFrame`, `onStatus`, `onError`, `frame` (pull), `pause`, `resume`, `recenter`, `stop`; frames carry `pose`, `fovY`, `projectionMatrix`, typed `hitTest`, `state`, `scale`. No Rust type in the public API.
- [x] R3: Camera, permissions (incl. iOS motion prompt), sensors and orientation handling live in `opsis`, not in example pages.
- [x] R4: `opsis/three`, `opsis/babylon` and `opsis/playcanvas` each stay under ~100 lines and only map the frame to the engine camera; A-Frame uses `opsis/three`.
- [ ] R5: Works on both WebGPU and WebGL backends from the same API.
- [x] R6: Ships as ESM + `.d.ts` with the wasm and no Rust source (M00 R8).

## Verification (built right)
- [x] V1: `cargo test -p opsis-core` passes on the host with replays identical to before the move.
- [x] V2: `cargo tree -p opsis-core` shows no wasm-bindgen, web-sys or wgpu.
- [x] V3: `tools/build-web.ps1` (cargo + wasm-bindgen + tsc) and `pnpm test` pass; `pnpm pack --dry-run` lists no `.rs` file.
- [x] V4: `examples/index.html` (parity) PASS on desktop WebGL2 and WebGPU.
- [x] V5: Release wasm gzip ≤ the M00 R9 budget (103 KB, parity left out).

## Validation (built the right thing)
- [ ] VA1: Owner builds a fresh Three.js page and a fresh Babylon page from `packages/opsis/README.md` only, on iPhone; the cube stays anchored while walking, as in M01 VA1.

## Log
- 2026-11-26: Created. Flow from the SDK design discussion; architecture review above. Not started.
- 2026-11-26: Library named Opsis. R1 done: Cargo workspace `crates/core` (`opsis-core`, no dependencies) + `crates/wasm` (`opsis`, output `pkg/opsis.js`); `slam.rs` split into `slam/{mod,ba,epipolar,reloc,scale,tests}.rs`; parity helpers out of the tracker (`parity.rs`, `tracker.rs`); `web/` → `examples/`, `scripts/` + `serve.mjs` → `tools/`; env vars `WEBAR_*` → `OPSIS_*`. Self-explanatory comments removed; tuning evidence, shortcuts and conventions kept.
  - Replay paths now resolve from the repo root (cargo runs tests from the crate dir).
  - Checked: 38 host tests pass; all 12 recordings replay identical to before (frame lines and metrics, timing excluded); parity PASS on desktop WebGL2 + WebGPU; wasm 126.4 KB gzip; `camera.html` loads without console errors (not run on a phone).
- 2026-11-27: SDK built (R2–R4, R6). One zero-dependency package `packages/opsis` (subpaths `opsis/three`, `opsis/babylon`) instead of three packages: the adapters are ~20 lines. Camera shown as a `<video>` with `object-fit: cover` under a transparent engine canvas, so no engine needs texture code; `fovY`, `projectionMatrix` (GL or WebGPU depth) and `hitTest` (floor) match that crop. World is Y-up, right-handed (Babylon: right-handed scene).
  - Weight, gzip: wasm 103 KB (parity test now behind the `parity` cargo feature, `build-web.ps1 -Parity`: −22 KB), wasm-bindgen glue 12.7 KB, SDK 4 KB, adapters < 0.5 KB each. Not minified: bundlers do that.
  - Examples: Canvas 2D, WebGL2, WebGPU, three.js, Babylon.js (engines from CDN), tap to place a cube. `camera.html` now runs on the SDK (HUD, recording, `?flags` unchanged; positions print in Y-up).
  - A denied motion permission no longer aborts: frames carry no pose (the old page behaved so).
  - Checked: `pnpm test` (pose is a proper rotation, quaternion = matrix, view = inverse, `hitTest` reprojects to the tap within 1e-5 at 3 aspects, both depth ranges); in the desktop pane with a fake camera and sensor the SDK reaches Normal. Not checked: the engine pages rendering (the pane was hidden, so frames were throttled), and anything on a phone.
- 2026-11-27: From the [8th Wall API comparison](../research/2026-11-27_8THWALL_API_COMPARISON.md), everything but scale modes:
  - `background: "tracked"` draws the tracked frame on a canvas in `mount`, in sync with the pose (fixes the ~10 ms swim of the live video, at `maxSide` resolution). `"video"` stays the default.
  - `session.frame` (latest frame) for apps with their own render loop; the Babylon and PlayCanvas examples use it.
  - `pause()` / `resume()` (also automatic on `visibilitychange`), `recenter()` (new wasm `Tracker.reset`: drops the map), `status` + `onStatus`; a camera track that ends reports an error and stops the session.
  - `hitTest` returns `[{ type: "floor", position, quaternion, distance }]`, nearest first, so feature points or walls can be added without an API break.
  - `opsis/playcanvas` adapter (structural, ~20 lines); A-Frame runs on `opsis/three`. Examples: `playcanvas.html`, `aframe.html`; `three.html` has a Recenter button and `?background=tracked`.
  - Weight: wasm 104.5 KB gzip (+0.05 for `reset`).
  - Checked: `pnpm test` (4 tests, adds hit type and distance, no hit looking up); in the pane with a fake camera: tracked background canvas 640×360, no frames while paused and video paused, frames again after resume, stop empties the mount. Not checked: `recenter` (a static fake scene re-seeds to the same map), the PlayCanvas and A-Frame pages rendering, anything on a phone.
- 2026-11-27: Root `README.md` (goals, quick start, architecture flow, repository map, dev commands, status). Redundant comments removed from the SDK and examples; public API TSDoc kept (units and semantics show in IDEs).
