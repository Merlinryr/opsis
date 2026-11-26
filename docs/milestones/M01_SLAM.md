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
- [x] R6: Uses no GPL or non-commercial code ([licenses](../research/2026-11-24_WEBAR_LIBRARY_DESIGN.md#open-source-and-licenses)).

## Verification (built right)
- [x] V1: Recorded-sequence test (`tests/replay.rs` on `recordings/`): pose error under a threshold (`WEBAR_REPLAY_MAX_ERR=0.15` on the `10-43-54` walk: closure 0.066, z spread 0.14, 2026-11-26).
- [ ] V2: Same sequence gives matching poses with WebGPU forced off (WebGL path).
- [x] V3: Occlude-then-uncover test: state goes lost → normal and pose snaps back (replays of 2 phone recordings, 2026-11-25).
- [ ] V4: Frame-time benchmark on target devices meets R5.
- [x] V5: Dependency license audit (e.g. `cargo deny check licenses`) passes.

## Validation (built the right thing)
- [ ] VA1: The owner places content on target phones, walks around it, and it stays anchored.

## Log
- 2026-11-26: Narrower readback looked at, not built. The CPU needs per pixel: blurred luma (8 bit, all pixels, for descriptor patches), the NMS flag, and the u16 FAST score at survivors and their 4 neighbors (sub-pixel parabola, ranking). Scores often exceed 255 (sum of arc differences), so a 2-byte format must quantize them, which changes keypoint positions and ranking (no longer exact, replays shift). WebGPU has no 3-byte color format, so exact packing stays at 4 bytes. Compacting survivors on the GPU needs a second round trip or compute-shader compaction. The best case would save ~0.7 of the 1.4 ms unpack, not worth that cost at 60 fps.
- 2026-11-26: CPU per frame. Phone: copy 2.5 (page `drawImage`) + upload ~4 (`copyExternalImageToTexture`) + unpack 1.4 + track ~3 ms.
  - Native profile of track's front-end half: descriptors 1.16, keypoint gathering 0.31, retain 0.17, blurred copy 0.15 ms; SLAM ~0.8 ms.
  - Cuts, exact (same replay output under the same seed): descriptors skip per-sample clamping when the patch is inside the level, and keypoint extraction reads scores straight from the readback instead of building a full-image score map. Native track mean 2.57 → 2.38 ms (12-06-23). Parity PASS on desktop WebGL2 + WebGPU. Not timed on the phone.
  - The rest of the CPU frame is browser work (copy + upload ~6.5 ms). The remaining code-side lever is a narrower readback, which cuts unpack.
- 2026-11-26: Height path dependence. `WEBAR_REPLAY_REVERSE=1` starts the first pass from the last frame. The SUMMARY `zratio` is end/start z of the first pass, so forward × reverse = 1 for a real height change (5 seeds each).
  - `12-43-04`: 1.18–1.23 forward × 0.74–1.02 reverse ≈ 0.91–1.22 (median ~1.04). The rise is mostly real: the phone was lifted.
  - Most other walks drift upward in both directions: products 1.04–1.24 (`14-02-*`, `10-43-54`, `10-44-12`, `10-51-41`, `12-43-29`). `11-55-10` and `12-06-23` read 0.88–0.98. That is a systematic ~5–10% height (scale) creep per 10 s pass, and it is the target.
  - Tried: unconfirmed floor seeds stop voting when ≥ 30 confirmed points match. Rejected: it fixed the `12-43-04` diverging seed (closures 0.012–0.022), but products got worse on `10-44-12`/`10-51-41` and it made wrong poses (gap max 1.6 and 3.9). Reverted.
- 2026-11-26: Tried holding scale with the IMU before writing code, and changed nothing. Sliding 3 s IMU scale windows (0.4 s low-pass, va/vv) vary 1.46–1.96 m/unit on `12-06-23`, whose height holds within 0.09 the whole walk. That is ±15–25% noise at the 3 s scale the `12-43-04` drift happens on (1.44 → 0.88 there, but `10-43-54` falls 1.57 → 0.94 with z flat too: late windows correlate less). 5 s windows are calmer but slower than the drift. A correction from this signal would add as much error as it removes.
  - Open options: an anchor that survives new floor (keyframe-based height: re-observe the original floor patch's scale through keyframe chains), or accept per-walk height drift of this size.
- 2026-11-26: `12-43-04` height drift (z 0.93 → 1.23 forward, correlation 0.77 with pitch) has no anchor.
  - `WEBAR_REPLAY_BORN=n` prints inliers by map point age. Points from before frame 100 carry the pose until ~130, then leave the view (≤ 20 matched from frame 160, 0 by 280), while z climbs on points seeded during the walk. On new floor the height is set by fresh seeds only, so height (= scale, since the unit is the camera height) drifts.
  - Not FOV: 64° cuts this take's spread 0.35 → 0.25 but doubles it on `10-43-54`. Not pitch bias: ±2–4° makes no consistent change. The IMU vertical motion correlates only 0.39 with the visual height, so part may be a real lift.
  - Next candidate: hold scale with the IMU metric estimate (now within 8% on walks), since scale drift and height drift are the same error here.
- 2026-11-26: Replay yardstick, then two measured changes.
  - Tooling:
    - `WEBAR_REPLAY_SEED`, plus `scripts/replay-seeds.ps1`: 5 seeds × every recording, median [min..max]. A change counts only if a median clears the other run's range.
    - Forward-vs-backward pose gap per frame on frames Normal both ways (median, p95, max): consistency, which catches wrong poses.
    - "Too far" split by whether a keypoint lies within 3 px of the projection.
  - Finding: under 1% of "too far" outcomes have their best-descriptor candidate within 3 px. That suggests the corner was not re-detected, but the test only checked the best candidate, not any keypoint, so it is not yet conclusive.
  - Frame-count thresholds (`REINIT_AFTER_LOST`, `KF_COOLDOWN`, miss counting) scale with the measured frame rate, so they keep their duration at 60 fps. 30 fps replays are identical under the same seed.
  - `MAX_MAP` 1500 → 3000: 1500 filled by frame 60 on the new 1.8 m walk `12-43-04`, which evicted the start area and drifted 0.5 on the way back.
    - Closure medians fell 0.004–0.041 → 0.003–0.009 on the other takes, most outside the old range; `12-43-04` itself unchanged. Lost on `10-51-41` fell 102 [87..110] → 73 [64..89]. Others unchanged.
    - Track max +0–40% (`12-43-04` 5.0 → 6.9 ms).
  - Rejected: `max_keypoints` 1000 (lost −10–25% on two takes, but wrong poses: gap max 0.13 → 1.2 on `10-44-12`, closure median 0.58 on `12-43-04`, track +50%). FAST minimum 5/4: no consistent change, more cost.
  - Still open: `12-43-04` height wanders 0.35 on every seed (0.93 → 1.20 forward), and 1 seed of 5 diverges (closure 0.47).
- 2026-11-26: Metric scale against the owner's phone height (~1.4 m):
  - Low-pass 0.1 s → 0.4 s twice. Correlation on walks rose 0.43–0.57 → 0.9.
  - Estimator aa/va → va/vv, with gate r > 0.8.
  - Window and lag search are now in seconds (5 s, ±0.1 s), so 60 fps matches 30 fps. A best lag on the search edge is rejected.
  - Replays: `12-06-23` 1.48 (+6%), `10-43-54` 1.52 (+8%), `10-51-41` 0.91 (−35%, lag 0; that take correlates at only 0.63 overall). `11-55-10` and `10-44-12`: none.
  - 2 of 3 within 8%. Not live-checked. Scale is still reported only, not used by tracking.
- 2026-11-26: Metric scale never converged (HUD `scale -`): visual and IMU accelerations correlated at r -0.4..0.03. An offline search over axis permutations and signs on the walk replays found the iPhone reports `devicemotion.acceleration` negated (r 0.57 / 0.43 at lag 0–1 frame).
  - Fix: one shared `geom::accel_in_camera`, used by the page and the replay, negates it.
  - Scale now converges on 3 of 4 IMU recordings: `12-06-23` 1.78, `10-43-54` 1.46, `10-51-41` 1.72 m/unit (unit = camera height at seed), IMU lag 0–33 ms. `11-55-10` (cover + pans) has no estimate.
  - Tracking unchanged: scale is reported only. Not validated against a true camera height. The estimators disagree on this data (aa/va 1.5–2.0 vs va/vv 0.3–0.65), so aa/va may read high.
  - shortcut: sign checked on iOS only.
- 2026-11-26: 60 fps target.
  - Phone frame work: copy 2.5 + upload ~4 + GPU ~6 + unpack 1.4 + track ~3 ms, run serially, about 17 ms.
  - The page now keeps two frames in flight. The next camera frame is drawn, uploaded and submitted while the previous one waits on the GPU, so frame time becomes max(CPU ~11, GPU ~6) instead of the sum. Each slot has its own canvas, so a frame shows the pixels it tracked. A frame is dropped when both slots are busy. The camera is asked for 60 fps (`frameRate: { ideal: 60 }`).
  - Pose delay stays one frame. The FAST threshold reads one frame stale.
  - Desktop: 4 overlapping `detect` calls on WebGPU run without error (500 keypoints each, Normal). Phone (iPhone 16e, WebGPU): HUD fps 60.0, was 29.9. R5 60 fps on this device.
- 2026-11-26: Phone, `?src=video` against the canvas path: upload 2.9–3.1 vs ~4.0 ms, but GPU 7.0–7.8 vs 5.0–6.0 ms (the luma pass now runs on 4x the pixels, plus a scale pass). Net ~1 ms worse. The image was also laggy, offset and inverted: the video element copy ignores the camera orientation that `drawImage` applies. Reverted to 8a2e540 (canvas path, gpu/map/unpack HUD).
  - The ~4 ms upload stays: only a raw `importExternalTexture` pass outside wgpu could remove it, and the same orientation would have to be handled there. Not worth it at 29.9 fps, which the camera's 30 fps caps anyway.
- 2026-11-26: Diagnostic build for the frame upload.
  - wgpu 30 has no `importExternalTexture` on web (`create_external_texture` is `unimplemented!`), so the first try stays inside wgpu. `?src=video` copies the video element at native size (`copy_external_image_to_texture` / `texImage2D`, both backends), and the existing `Down` pass scales the luma to 360x640 (a 2x2 box at ratio 2).
  - The HUD now times `upload` apart from `wait`, and splits wait into submit / gpu / map / unpack.
  - `?fps=60` asks the camera for 60 fps.
  - The display `drawImage` stays (the visible canvas needs the frame), so the possible saving is the ~4 ms upload, not 6.5 ms.
  - Parity PASS on desktop WebGL2 + WebGPU, and both sources run on both backends with a 720x1280 test stream. **The recorder still reads the canvas:** don't record with `?src=video` until it reads the GPU luma.
- 2026-11-26: Phone wait split (iPhone 16e, WebGPU 360x640), reading wait = gpu + map + unpack + rest:
  - `?levels=1`: 8.1 = 3.5 + 0.0 + 0.5 + 4.1.
  - `?levels=4`: 11.9 = 6.5 + 0.0 + 1.4 + 4.0.
  - The ~4 ms rest is fixed: `wait` also times `upload_canvas` (`copyExternalImageToTexture` from the canvas, `web.rs` t0 sits before it). With the page's `drawImage` (copy 2.5), getting the frame in costs ~6.5 ms, as much as all GPU passes at 4 levels. Levels 1–3 add 3 ms of GPU. Map is free.
  - Next candidate: import the video frame directly (`importExternalTexture`, scaled in the luma pass), dropping both copies. That touches the recorder (it reads the canvas) and needs a scaling that matches `drawImage`, or the recordings stop matching the live input.
- 2026-11-26: Phone HUD after batching: copy 2.0, gpu enc 0.2, wait 10.1, track 2.5 ms. The old build's `gpu` was 10.9, so batching saved under 1 ms: detect is wait-bound.
  - Added a diagnostic: the HUD splits `wait` into `gpu` (submit → work done), `map` (→ buffers mapped) and `unpack`, plus a `?levels=n` param to see what scales with passes and readback size (~1.7 MB at 4 levels).
  - The row unpack now uses per-row `extend_from_slice`. Parity PASS on desktop WebGL2 + WebGPU.
  - Next: the HUD at `?levels=4` and `?levels=1`, then pick: GPU-bound → fewer/cheaper passes; map-bound → narrower readback; fixed floor → only overlapping frames, which adds a frame of lag.
- 2026-11-26: Replay of `12-06-23` on ccc589e: 593 Normal of 599, 0 Limited, 0 lost, closure 0.003 (0.1% of a 3.7 m path), z 0.94–1.03, track mean 2.3 / max 4.4 ms. Passes V1 at 0.15. Best run so far. Old-point inlier share 37% (walk `10-43-54`: 24%).
- 2026-11-26: `camera.html` keeps drawing the cube for up to 1 s after tracking is lost, from the last position with the live gyro rotation, in orange like Limited. Lost longer than that hides it as before. Page loads without errors on desktop; not tried on the phone.
- 2026-11-26: Looked into "hold tracking through fast pans" on `11-55-10` before writing code, and changed nothing. Gyro prediction is already in: `orient()` applies the current sensor rotation before guided matching, and floor descriptors are world-oriented.
  - Loss 1 (frames 78–90, under 21°/s): fingers over the lens, with keypoints at 400 → 90 at the minimum FAST threshold.
  - Loss 2 (149–165, up to ~110°/s): heavy motion blur (keypoints 360 → 70), then an unmapped wall area. Inlier share stayed at 60–80% while matches fell, so it lacks corners and has no bad poses.
  - Both losses are honest. Widening the radius or the gates would bring back wrong poses.
  - Open product choice: show content while Lost for a short time using the gyro rotation, as `camera.html` now hides it while lost.
- 2026-11-26: Replay of the new recording `11-55-10` on 7b3f243: 372 Normal / 9 Limited / 208 Lost, closure 0.011, z 0.87–1.18 (fails V1 at 0.15 on z spread 0.30). There are two forward lost runs, ~85–135 and ~160–225, with 100–250 global matches but under 20 inliers: the scene is visible and relocalization fails. Both runs are under 90 frames, so there was no re-seed or merge. Cause (owner): fast pans, tilts and pointing at an unmapped spot, so the losses are expected, and recovery afterwards was correct. It relocalized at z 1.12 (was 0.96) and drifted back to ~1.0. `10-51-41`: 491/10/91, closure 0.021, z 0.87–1.02.
- 2026-11-26: Second map with merge. After 90 lost frames the tracker re-seeds again, but only on a frame that can seed, so a covered lens just stays Lost. The old map is kept as the stash, and it is tried every 5 tracked frames and on every lost frame, with the same Normal-quality gate as leaving Lost. On a hit while tracking, the new map is carried into the old frame (translation + yaw + scale, since each seed assumes camera height 1) and appended. On a hit while lost, the stash replaces the current map. Pose, `v`, `motion`, keyframes and metric scale reset on merge.
  - Replays: walk `10-43-54` identical (573/8/7, closure 0.066, 0 re-seeds). Covered-lens `10-44-12`: 1 re-seed, 1 merge, lost 299 → 267, closure 0.003. `14-02-15`/`14-02-35` identical. 38 host tests (re-seed then merge, at true height 1.0 and 1.2).
  - Between the re-seed and the merge, placed content sits offset (new map frame), then snaps back.
  - shortcut: one stash, always the oldest map; a second re-seed drops the current map. A stash hit while lost drops the current map.
- 2026-11-26: WebGPU front end records all passes into one command encoder and submits once with the readback copies; it used to make one submit per pass, ~21 per frame at 4 levels. Parity PASS byte-exact on desktop WebGL2 + WebGPU. wasm 128.5 KB gzip. **Not timed on the phone:** the earlier screenshot HUD read `gpu 10.9` with no `enc`/`wait` split, so it ran a build older than the split. Next: phone run reading `gpu enc` vs `wait`, to decide between shader work (wait-bound) and less per-pass setup (enc-bound).
- 2026-11-26: V1 closed. `WEBAR_REPLAY_MAX_ERR=m` fails the replay if the closure gap or forward z spread exceeds m (units: camera height = 1). Walk `10-43-54` passes at 0.15. Covered-lens `10-44-12` fails it on z spread 0.21 (not a constant-height take). Recordings are gitignored, so this runs locally only.
- 2026-11-26: Replays of the two new IMU recordings on 422e6fb.
  - `10-43-54` (walk): 574/599 Normal, lost 4, z 0.94–1.09, closure 0.044 (0.9% of a 5.1 m path), track mean 3.9 / max 7.3 ms.
  - `10-44-12` (covered lens): forward pass gets lost at ~70–160, then relocalizes back to the origin (R4/V3 hold). A second cover at ~230 runs across the turnaround: about 100 lost frames, which triggers the floor re-seed (map 1148 → 322). That moves the origin by ~1 m in y, giving closure 1.016. The Limited frames before the re-seed report garbage poses (up to y −7.8).
  - Per-frame trace: every pose jump came from a Lost → Limited relocalization (1–8 units off), which then became the Lost "last pose". Two weak Limited jumps chained into a wrong Normal.
  - Fix: leaving Lost now takes Normal quality, and a long loss re-seeds only if the map has under `MIN_INIT_POINTS` points (otherwise it keeps relocalizing). Trade-off: in a genuinely new place it stays Lost until the mapped area is back in view. Upgrade: a second map, merged on relocalization.
  - After: `10-44-12` closure 1.016 → 0.002, lost 260 → 299 (the wrong poses are now Lost), z 0.90–1.10. `10-43-54` closure 0.044 → 0.066, lost 4 → 7. `14-02-15`/`14-02-35` within noise (0.007 → 0.004, 0.034 → 0.036). 37 host tests. Not run on the phone yet.
- 2026-11-26: Replays of 422e6fb vs f0b8583, 3 seeds each. Only the two covered-lens recordings are still on disk (`14-02-15`, `14-02-35`); the corridor, room, grey and tilt runs are gone. They have no IMU columns, so scale isn't exercised.

  | | f0b8583 | 422e6fb |
  |---|---|---|
  | Lost frames `14-02-15` (fwd + back, incl. cover) | 170 / 235 / 172 | 169 / 168 / 168 |
  | Lost frames `14-02-35` | 154 / 155 / 154 | 150 / 150 / 150 |
  | Uncover → Normal (`14-02-15` / `14-02-35`) | frame 203 / 198 (via Limited) | frame 199 / 192 (direct) |
  | `track()` mean / max (native) | 3.1–4.0 / 6.1–11.3 ms | 2.0–2.4 / 3.9–5.6 ms |
  | Old-point inliers | 44–66k | 53–79k (+20–27%) |
  | Old-point ambiguous | 24–83k | 11–14k |
  | Old-point "no keypoint" | 14–37k | 37–116k |
  | Forward z range | 0.84–1.06 | 0.91–1.06 |
  | Closure gap `14-02-35` | 0.003–0.012 | 0.021–0.034 |

  - **Reading:**
    - The seed no longer moves the outcome: the 235-lost / 0.32-closure seed of the old build is gone.
    - Recovery is faster and skips Limited.
    - `track()` is faster despite keyframes and BA, because the level filter cuts candidates. No keyframe spike shows in the max.
  - **To watch:** "no keypoint" tripled, because the level filter now counts wrong-level candidates as none. Closure on `14-02-35` is slightly worse but still ≤ 1.1% of path.
  - **Next:** new phone recordings with IMU (walk at constant height, sway for scale, covered lens), plus a grey-floor run to re-measure old-point re-matching.
- 2026-11-25: Second build round, all in code. **Not validated: no replay or phone run (test later).**
  - **Time and IMU end to end:**
    - `update` takes the capture time and gravity-free acceleration. The page sends `devicemotion` and requests its permission on iOS.
    - The recorder writes `ax, ay, az` after the time column. Old recordings still replay, but their results will change with this build. The frame interval is taken as an absolute value, because replays also run backward.
    - Turn gating is now per second (70 °/s).
  - **Metric scale and IMU offset** (`src/imu.rs`, loose like 8th Wall, not preintegration):
    - They come from a 150-frame window: low-passed visual vs IMU acceleration at the lag of best correlation, over the window's mean frame interval. Scale = aa/va: with ~4 mm tracking jitter, va/vv read 1.390 and aa/va 1.401 for a true 1.4.
    - They are reported in `pose()` and on the HUD; nothing in the solve uses them.
  - **Epipolar points:** each keyframe stores keypoints with no map point. A new keyframe matches them against the last 3 (Hamming ≤ 32, ratio 0.8), and the rays must meet with ≥ 4° parallax, above the floor. These are the first off-floor points that don't start as floor seeds.
  - **Joint local BA:** Gauss-Newton with a Cauchy loss, points eliminated by Schur complement, 12×12 camera system. It replaces the alternation. The oldest keyframe and floor points stay fixed, and a point is solved only if its window rays have ≥ 4° parallax.
  - **Keyframes:** the low-map rule now counts map points, as in 8th Wall (was inliers). Only the 4-keyframe window is kept, because nothing reads older ones. A redundant keyframe (≥ 70% shared, < half the baseline) is skipped.
  - **Landscape:** camera axes turn by `screen.orientation.angle`, and a rotation starts a new map. The sign is from the W3C definition, not yet checked on a phone.
  - **GPU:**
    - Textures are pooled in both backends; uploads into reused textures use `texSubImage2D`.
    - The HUD splits `gpu` into encode and wait.
    - Parity PASS on desktop WebGL2 and WebGPU. wasm 127 KB gzip (was 97.5).
  - **Self-checks:** the scale and lag estimator recovers 1.4 m/unit and 3 frames from a noisy synthetic sway. BA recovers keyframes nudged ~1 cm / 0.3° and wall points nudged ~1 cm, within 5 mm and 0.1°; points seen by ≥ 3 keyframes average < 4 mm. Epipolar creation puts synthetic wall corners within 1 mm. 37 host tests.
  - **Not exercised by host tests:** level prediction, east-oriented descriptors end to end, keyframe views, epipolar creation, the IMU path from page to tracker, landscape, and texture pool reuse across frames.
  - **Watch in the test round:**
    - If the HUD scale stays "-" on the iPhone, check the sign of the acceleration correlation first.
    - Keyframe frames (epipolar + BA) will spike `track()`: judge on the replay's max ms, not the mean.
  - **Still out:**
    - Patch warp, GPU descriptors with keypoint-only readback, and threshold retuning: each needs measurements first.
    - WebGL PBO readback (changes frame timing) and per-device intrinsics.
- 2026-11-25: Phase 3 in a lightweight form, in code, from the [gap analysis](../research/2026-11-25_8THWALL_GAP_ANALYSIS.md). **Not validated: no replay or phone run yet (test after).**
  - Floor keypoints are described with the image direction of world east, which stays fixed as the camera moves. Gravity was rejected: on the floor it points to the camera's foot. Other keypoints keep the centroid angle.
  - Guided matching only accepts keypoints within ±1 of the pyramid level predicted from distance. It tries the point's latest descriptor plus up to 3 keyframe descriptors; global matching uses the latest only.
  - Constant-velocity prediction for guided matching, zeroed on Lost and on relocalization.
  - No map upkeep above 0.04 rad/frame of sensor turn (~70 °/s at 30 fps).
  - Keyframes: cooldown 5 frames, then on ≥ 20 new points, < 120 inliers, or 0.1 camera heights moved. Capped at 64.
  - Local BA over 4 keyframes by alternation. Points are re-triangulated from all rays if they fit every view; keyframe poses get Gauss-Newton steps on still-fitting observations. The first version without that filter dropped confirmed platform points 26 → 19.
  - Points that move (re-triangulation, BA) rescale their stored distance, or level prediction would reject their own corner. BA never moves a point to within 0.05 of the floor (z = 0 pins scale).
  - The host tests use level 0 and exact geometry, so they don't exercise level prediction, the canonical angle end to end, or keyframe views. V3's tick predates this build; the canonical angle drifts with yaw while Lost.
  - First things to check in the test round: whether `inliers < 120` makes a keyframe every 5 frames, and `track()` time (up to 4 Hamming distances per candidate).
  - 34 host tests, wasm release builds. Left out: phase 4 (tight IMU) and phase 5 (fps), epipolar search for new points, redundant-keyframe culling.
- 2026-11-25: 8th Wall gap analysis ([note](../research/2026-11-25_8THWALL_GAP_ANALYSIS.md)). Architecture already matches, including what 8th Wall leaves off (loop closure, global BA). Found: live tracking uses centroid-angle descriptors (`lib.rs:40`), while 8th Wall tracks with gravity-oriented ORB. That is the top suspect for the 42% "descriptor too far" loss. Next: gravity descriptors, then level-predicted matching, gyro gating and velocity prediction, each judged on 3 seeds. Not run.
- 2026-11-25: Phone, canvas straight to GPU: copy 5.0 → ~2.1 ms, gpu 7–9 → 10–12 ms, track ~2.5. Net frame work unchanged (~15.5 ms): the cost moved from the readback into the GPU copy/wait, as feared. Kept (no worse, simpler path). Next: time the `gpu` part internally (encode + allocation vs wait for the GPU) before changing more.
- 2026-11-25: Camera frame straight to the GPU: the page still scales the video with `drawImage` on a hidden canvas (same pixels as the recordings), but no longer calls `getImageData`; `Tracker.detect` takes the canvas and both backends copy it into a texture (`copyExternalImageToTexture` on WebGPU, `texImage2D` on WebGL). The canvas drops `willReadFrequently`, which kept it on the CPU. Recording reads pixels back only when `?record=1`. Parity now feeds its frame through a canvas too: PASS byte-exact on desktop WebGL2 + WebGPU. wasm 97.5 KB gzip, 33 host tests. Not yet on the phone.
- 2026-11-25: Phone split after the first pass (iPhone 16e, WebGPU 360x640): copy ~5.0 ms, gpu 7–9 ms, track 2–2.5 ms. Detect (gpu + track) ~10 ms, was 19–21. Frame work ~15 ms incl. the copy: under the 16.7 ms of 60 fps with little margin. Next: copy the video straight into a GPU texture (removes canvas copy and RGBA upload), then pool textures.
- 2026-11-25: Detect time (R5), first pass. WebGPU reads all 4 pyramid levels back in one submit and one wait (was 4 sequential round trips). Release builds at `opt-level = 3` instead of `"z"`: replay `track()` 5.41 → 2.38 ms (tilt) and 3.44 → 2.23 ms (grey), identical output, wasm 89.9 → 96.4 KB gzip (`"s"`: 3.69 ms, same size, rejected). HUD shows the split `copy / gpu / track` ms (`copy` = canvas drawImage + getImageData, outside `detect`). Parity PASS on desktop WebGL2 + WebGPU, 33 host tests. Not yet on the phone: the split decides the next step (texture pooling, canvas copy, or tracker).
- 2026-11-25: Live phone run of f0b8583 (iPhone 16e, WebGPU 360x640, screen recording on, 12 s): Normal while walking, z 0.99–1.04, lost on a cover and Normal again within ~1 s, map 995–1463. fps 30.0–30.6 (paced by the 30 fps video frames), detect 19.2–21.0 ms: the R5 30 fps floor holds on this device, but 60 fps needs detect well under 16 ms. Recording upload showed `REC 0/300 FAILED 3` in this clip (server not in record mode).
- 2026-11-25: Three phone recordings of the f0b8583 build, replayed (3 seeds not run, default only). Tilt at constant height (`14-01-54`, pitch 64–89°): Normal 585/599, never lost, z 0.91–1.05: height no longer follows pitch, so the earlier rises were real lifts. Two covered-lens runs (`14-02-15`, `14-02-35`, cover ≈ frames 115–195): Normal until the cover, Lost under it, then Limited → Normal 2–6 frames after uncover by relocalization onto the old map (map size unchanged, no re-seed). First run snaps back within 0.05; the second comes back 0.37 off in y (unknown whether the phone moved under the cover). `track()` mean 4.0–5.3 ms native. V3 ticked. Not measured: live fps/detect on the phone during these runs.
- 2026-11-25: License audit (R6, V5): every dependency crate in `cargo metadata` is MIT, Apache-2.0, Zlib, ISC, BSD-2-Clause, Unicode-3.0 or Unlicense; no GPL or non-commercial. The web side uses Node built-ins only. cargo-deny is not installed; re-run: `cargo metadata --format-version 1` and list each package's `license`.
- 2026-11-25: Relocalization guards and descriptor refresh, judged on 3 RANSAC seeds (the seed alone moves room 103–130 lost, grey 0–7) and sensor shifts ±0.25. One seed relocalized 26–30 units away, below the floor, as Normal and culled the map with that pose. Fixes: a relocalized pose is confirmed by guided matching from it in the same frame; a pose below the floor is Lost; a relocalization may not change height by more than half the camera height; Normal needs ≥ 1/8 agreement (good Normal frames agree ≥ 16% across seeds and shifts). New replay metric, outcome of in-image points ≥ 50 frames old: descriptor too far is the main loss (24/34/48% corridor/room/grey), then ratio-rejected (30/21/13%); no keypoint near only 4/4/22%. Refreshing a point's descriptor on each inlier: old inliers 13k → 26k / 18k → 25k / 9k → 23k, same `track()` time. Up to 4 views per point: little more (room +10%, grey none) and let a relocalization 20 units off through, so rejected. Final, 3 seeds + shifts ±0.25: grey and corridor 0 lost, room 100–118 (was 92–130), no pose jumps, z ranges no wider. 32 host tests. `raised_points_get_their_true_height` failed under one seed: it counted unconfirmed floor hypotheses (no 4° parallax yet) as wrong heights; it now judges confirmed points and passes on 5 seeds. Next: not settled by replays alone (descriptor distance still 42% of grey's old points). Not run on the phone.
- 2026-11-25: Keyframes on hold. Height follows phone pitch in both new recordings, carried only by fresh points, with no pitch bias found: likely a real lift, not drift keyframes would fix. Needs a constant-height recording (also V1's z reference). Next: keyframe store with multi-view descriptors, judged by re-matched old points on the grey replay. Details: [replay analysis](../research/2026-11-25_PHONE_REPLAY_ANALYSIS.md).
- 2026-11-25: Grey-floor recording (scene of the first failed run): Normal every frame. Fix: map points count a miss only near a detected corner (a blurred turn culled the map 1500 → 806). Found: on texture-poor floors old points are rarely re-matched, which limits R4 there. 29 host tests.
- 2026-11-25: Room recording: lost while looking at the ceiling, then relocalized onto the old map without re-seed (R4 evidence). Fix: guided matching ratio test (0.8, skips same-corner twins within 3 px): drifting poses now report Lost instead of Normal. Lost run now 79 frames, 11 short of the floor re-seed.
- 2026-11-25: Recorder fixed: capture index, capture time and session id per frame; HUD counts acks (`REC n/300`, `FAILED n`); server logs rejections and sends `no-store`. The 2026-11-24 corridor tail is likely out of order: use frames 0–269.
- 2026-11-25: Replay metrics: forward-then-backward pass, matches, `track()` time, closure, z range; knobs `WEBAR_REPLAY_END`, `_SHIFT`, `_PITCH`. Closure is noise at this level: judge on lost frames and z range. CPU half 6.04 → ~4.0 ms native (identical output). FOV 69–70 and delay 0 confirmed offline. Nothing from 2026-11-25 has run on the phone yet.
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
