# M01: SLAM

**State:** In progress
**Goal:** Markerless world tracking (SLAM) in the browser, shipped first as the core of the [M00](M00_WEBAR_LIBRARY.md) library.

## Relations
- Depends on: none
- Enables: [M00](M00_WEBAR_LIBRARY.md)

## Requirements
- [ ] R1: Returns a 6-DoF camera pose every frame from camera + motion sensors.
- [ ] R2: Runs in the Rust/WASM core; GPU passes work on both WebGPU and WebGL.
- [ ] R3: Reports tracking state (normal, limited, lost) each frame.
- [ ] R4: Recovers tracking after loss when the camera returns to a mapped area.
- [ ] R5: Meets the M00 fps budget: ≥ 30 fps on all supported devices, ≥ 60 fps on recent flagships.
- [ ] R6: Uses no GPL or non-commercial code ([licenses](../research/2026-11-24_WEBAR_LIBRARY_DESIGN.md#open-source-and-licenses)).

## Verification (built right)
- [ ] V1: Recorded-sequence test: pose error under a threshold (_open_).
- [ ] V2: Same sequence gives matching poses with WebGPU forced off (WebGL path).
- [ ] V3: Occlude-then-uncover test: state goes lost → normal and pose snaps back.
- [ ] V4: Frame-time benchmark on target devices meets R5.
- [ ] V5: Dependency license audit (e.g. `cargo deny check licenses`) passes.

## Validation (built the right thing)
- [ ] VA1: The owner places content on target phones, walks around it, and it stays anchored.

## Log
- 2026-11-24: Corner overlay confirmed on iPhone 16e. Phase 1 closed; next: phase 2 (ground-plane init, tracking, tracking state: R1–R3).
- 2026-11-24: First frame-time baseline, iPhone 16e, WebGPU, 360x640: detect 14.5 ms, 54 fps (rAF-capped at 60), 280 keypoints. Fixed camera overlay never showing circles (canvas presented during the detect await).
- 2026-11-24: Live camera view (`web/camera.html`): back camera + detected corners + fps/ms HUD, works on iPhone 16e. Phone testing via `scripts/phone.ps1` (Cloudflare quick tunnel + QR). wasm 75 KB gzip. Frame time not yet recorded; known slow paths (canvas copy, per-pass textures, sync readback) are `shortcut:` items for R5.
- 2026-11-24: Phase 1 built: CPU front end + BRIEF (13 tests); WebGL2 and WebGPU match the CPU per pass and end to end down to descriptors (`web/index.html`); wasm 69 KB gzip; deps MIT/Apache/Zlib only. Verified on desktop Chrome 156 and iPhone 16e (both backends PASS; WebGPU downscale ±1 there vs 0 on desktop). Android and frame time not verified.
- 2026-11-24: Pipeline and phases in [SLAM design](../research/2026-11-24_SLAM_DESIGN.md).
- 2026-11-24: Created. SLAM split out of M00 as the first deliverable.
