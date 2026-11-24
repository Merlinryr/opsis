# M01: SLAM

**State:** In progress
**Goal:** Markerless world tracking (SLAM) in the browser, shipped first as the core of the [M00](M00_WEBAR_LIBRARY.md) library.

## Relations
- Depends on: none
- Enables: [M00](M00_WEBAR_LIBRARY.md)

## Requirements
- [ ] R1: Returns a 6-DoF camera pose every frame from camera + motion sensors.
- [x] R2: Runs in the Rust/WASM core; GPU passes work on both WebGPU and WebGL.
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
- 2026-11-24: First phone recording replayed (corridor walk, white tiles, 300 frames, fov 60). Old build: started on a blurry frame with 50 points, lost ~3 s, tracked ~5 s, then lost when the map hit its 1500 cap. Fixes: start needs 100 floor points; full map evicts least-recently-seen points. Same replay now: Normal 298/300, never lost. Height drifts with FOV (final z: fov 50 → 0.33, 60 → 0.57, 70 → 1.05, 75 → 1.41), so the iPhone 16e video FOV is about 70, not 60; the replay sweep (`WEBAR_REPLAY_FOV`) is an offline FOV calibration. Replays use the CPU front end (GPU differs by ±1 in downscale only).
- 2026-11-24: Phase 3 (part 1). Record/replay harness: `camera.html?record=1` + `scripts/phone.ps1 -Record` upload luma + sensor angles to `recordings/`, `tests/replay.rs` replays them through the shared `track()` (groundwork for V1; no ground-truth pose yet). Seeded floor points re-triangulated from two rays when seen with ≥ 4° parallax (stool fix; raised-platform test: 0/177 → ≥ 80% at true height). Relocalization: global ratio-tested matching + 128-sample RANSAC from the last pose; re-seed only after ~3 s; Limited needs ≥ 15 inliers and ≥ 25% agreement. Replay of a pane recording: a camera jump now costs 1 lost frame then relocalizes onto the map (before: 15 frames of wrong limited poses). 26 host tests. R2 ticked (parity PASS on desktop + iPhone). Not yet on the phone; keyframe triangulation and BA still to do.
- 2026-11-24: Fixes from the first vision run: adaptive FAST threshold (down 2/frame below 80% of the budget, min 7; back up 1/frame to the default 20 when full), map grows whenever Normal (was 40 inliers), re-seed from the floor after 30 lost frames keeping x/y (stand-in for R4), cube hidden unless normal/limited, threshold shown in the HUD. 22 host tests. Pane check on a low-texture synthetic floor: threshold settles at 14, Normal throughout, map 149 -> ~358. Known: on scenes whose corners all share one contrast the threshold can flip and drop all corners for a frame. Not yet re-run on the phone.
- 2026-11-24: First vision run on the phone (grey tile floor, stools, shelves): mostly lost, brief normal at 26-28 inliers, map frozen at 83 points. Causes: (1) almost no FAST corners on the low-texture floor at threshold 20 (2-7 keypoints looking down); (2) map growth gated at 40 inliers, never reached; (3) stool/shelf corners seeded as floor points give wrong 3D (pos z jumped to 0.76-1.08); (4) no recovery once lost. Detect rose to ~40 ms / 17-19 fps in this run (screen recording on). Fixed: HUD position formatting (Float32Array.map).
- 2026-11-24: Phone, sensor-only run (`?vision=0`, iPhone 16e): floor grid level and in perspective, axis conventions confirmed; ~30 fps, detect 20–25 ms at 360x640 WebGPU. Dense corner rows on tile grout seen: FAST verified not to fire on synthetic thin lines (new test), so it is real grout texture; watch for sliding matches along lines in vision runs.
- 2026-11-24: Phase 2 built (`src/geom.rs`, `src/slam.rs`): ground-plane init from floor rays, sensor pitch/roll + vision-solved position and yaw (2-point RANSAC + Gauss-Newton), guided matching, initializing/normal/limited/lost states, map growth on an 8 px occupancy grid with culling of in-view misses on well-tracked frames only. 19 host tests incl. 3 deg yaw error + outliers and a 3.2-unit walk off the initial map and back (Normal throughout, drift < 0.05). Camera page: floor grid + anchored cube, `?fov=` `?delay=` `?vision=0` (sensor only), sensor age + frame driver in the HUD; full loop checked in the pane with a fake camera and sensor. wasm 81 KB gzip. Not yet on the phone: R1–R3 stay unticked until the cube holds after walking ~1 m sideways and back. Lost has no recovery until phase 3 (R4).
- 2026-11-24: Corner overlay confirmed on iPhone 16e. Phase 1 closed; next: phase 2 (ground-plane init, tracking, tracking state: R1–R3).
- 2026-11-24: First frame-time baseline, iPhone 16e, WebGPU, 360x640: detect 14.5 ms, 54 fps (rAF-capped at 60), 280 keypoints. Fixed camera overlay never showing circles (canvas presented during the detect await).
- 2026-11-24: Live camera view (`web/camera.html`): back camera + detected corners + fps/ms HUD, works on iPhone 16e. Phone testing via `scripts/phone.ps1` (Cloudflare quick tunnel + QR). wasm 75 KB gzip. Frame time not yet recorded; known slow paths (canvas copy, per-pass textures, sync readback) are `shortcut:` items for R5.
- 2026-11-24: Phase 1 built: CPU front end + BRIEF (13 tests); WebGL2 and WebGPU match the CPU per pass and end to end down to descriptors (`web/index.html`); wasm 69 KB gzip; deps MIT/Apache/Zlib only. Verified on desktop Chrome 156 and iPhone 16e (both backends PASS; WebGPU downscale ±1 there vs 0 on desktop). Android and frame time not verified.
- 2026-11-24: Pipeline and phases in [SLAM design](../research/2026-11-24_SLAM_DESIGN.md).
- 2026-11-24: Created. SLAM split out of M00 as the first deliverable.
