# M00: WebAR library

**State:** In progress
**Goal:** Opsis: a lightweight Rust/WASM WebAR package whose one job is highly stable, reliable SLAM in the browser.

## Relations
- Depends on: [M01](M01_SLAM.md), [M02](M02_SDK.md)
- Enables: none

## Requirements
- [x] R1: Core is written in Rust and compiled to WASM (`crates/core`, `crates/wasm`).
- [x] R2: Renders and runs GPU vision passes on WebGPU when available (front-end passes; rendering is left to the app's engine, [M02](M02_SDK.md)).
- [x] R3: Falls back to WebGL when WebGPU is unavailable (`backend: "auto"`; WebGL2 passes match the CPU byte-exact).
- [ ] R4: SLAM: delivered by [M01](M01_SLAM.md) (_in progress: 10 of 12 checks met there; open: fps on other devices_).
- [x] R5: Lightweight: release WASM ≤ 1 MB gzipped (104.5 KB, 2026-11-27; whole package ~122 KB with JS).
- [ ] R6: Optimized: tracking at ≥ 30 fps on all supported devices, ≥ 60 fps on recent flagships (_partial: 60 fps on iPhone 16e only_).

## Verification (built right)
- [x] V1: `cargo build --target wasm32-unknown-unknown --release` succeeds (inside `tools/build-web.ps1`).
- [x] V2: Size check on the release `.wasm` (gzip) is ≤ the R5 budget (`tools/build-web.ps1` prints it: 104,479 B).
- [x] V3: Forced-fallback test (WebGPU disabled) runs on WebGL with the same pose output (parity page: WebGL2 and WebGPU front ends match the CPU; owner, 2026-11-26: `?backend=webgl` tracks as well on iPhone, [M01](M01_SLAM.md) V2).
- [x] V4: SLAM passes a recorded-sequence test within a pose-error threshold (0.15 units, [M01](M01_SLAM.md) V1).
- [ ] V5: Frame-time benchmark on target devices meets the R6 fps (_partial: iPhone 16e only_).

## Validation (built the right thing)
- [ ] VA1: The owner runs the SLAM demo on target phones; content stays anchored while walking around (_iPhone 16e passed, [M01](M01_SLAM.md) VA1; other phones not tried_).

## Log
- 2026-11-27: Scope narrowed to SLAM only: dropped WebXR fallback (old R4, V4), image tracking (R6), VPS (R7) and the closed-source requirement (R8); the rest renumbered (R9 → R5, R10 → R6, V5 → V4, V6 → V5). Met: R1–R3, R5, V1–V4. Open: R4 (via M01), R6, V5, VA1 on other phones.
- 2026-11-27: Status from M01 and M02. Met: R1–R3, R9, V1–V3. Partial: R5, R8, R10, V5, V6. Not started: R4 (WebXR fallback), R6 (image tracking), R7 (VPS), V4. Library named Opsis; the SDK is [M02](M02_SDK.md).
- 2026-11-24: SLAM split into [M01](M01_SLAM.md), shipped first.
- 2026-11-24: Prior art, licenses and open decisions in [research](../research/2026-11-24_WEBAR_LIBRARY_DESIGN.md).
- 2026-11-24: Set budgets (1 MB, 30/60 fps) and on-device VPS.
- 2026-11-24: Created. Rust + WebGPU-first supersedes the C++/MIT-engine, WebGL2-first stack in [research](../research/SLAM/2026-11-24_MIT_ENGINE_AND_WEBGPU_STACK.md).
