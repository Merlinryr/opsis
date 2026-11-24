# M00: WebAR library

**State:** Planned
**Goal:** A closed-source, lightweight Rust/WASM WebAR package that provides SLAM, image tracking and VPS in the browser.

## Relations
- Depends on: [M01](M01_SLAM.md)
- Enables: none

## Requirements
- [ ] R1: Core is written in Rust and compiled to WASM.
- [ ] R2: Renders and runs GPU vision passes on WebGPU when available.
- [ ] R3: Falls back to WebGL when WebGPU is unavailable.
- [ ] R4: Falls back to WebXR when the custom pipeline cannot run.
- [ ] R5: SLAM: delivered by [M01](M01_SLAM.md).
- [ ] R6: Image tracking: detects and tracks registered image targets.
- [ ] R7: VPS: localizes the device against a prebuilt map shipped to the client, with no cloud call.
- [ ] R8: Package is closed source: shipped as compiled WASM + JS, no source published.
- [ ] R9: Lightweight: release WASM ≤ 1 MB gzipped.
- [ ] R10: Optimized: tracking at ≥ 30 fps on all supported devices, ≥ 60 fps on recent flagships.

## Verification (built right)
- [ ] V1: `cargo build --target wasm32-unknown-unknown --release` succeeds.
- [ ] V2: Size check on the release `.wasm` (gzip) is ≤ the R9 budget.
- [ ] V3: Forced-fallback test (WebGPU disabled) runs on WebGL with the same pose output.
- [ ] V4: Forced-fallback test (custom pipeline disabled) hands off to a WebXR `immersive-ar` session.
- [ ] V5: SLAM, image tracking and VPS each pass a recorded-sequence test within a pose-error threshold (_open_).
- [ ] V6: Frame-time benchmark on target devices meets the R10 fps.

## Validation (built the right thing)
- [ ] VA1: The owner runs a demo of each capability on target phones; content stays anchored while walking around.

## Log
- 2026-11-24: SLAM split into [M01](M01_SLAM.md), shipped first.
- 2026-11-24: Prior art, licenses and open decisions in [research](../research/2026-11-24_WEBAR_LIBRARY_DESIGN.md).
- 2026-11-24: Set budgets (1 MB, 30/60 fps) and on-device VPS.
- 2026-11-24: Created. Rust + WebGPU-first supersedes the C++/MIT-engine, WebGL2-first stack in [research](../research/SLAM/2026-11-24_MIT_ENGINE_AND_WEBGPU_STACK.md).
