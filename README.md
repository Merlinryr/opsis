# Opsis

Markerless augmented reality for the mobile web. Opsis tracks where the phone is in the room from its camera and motion sensors, so 3D content stays fixed in the real world while the user walks around it. It runs in any mobile browser, with no app install and no cloud, and works with any 3D engine.

## Why
- **WebXR is not everywhere.** iPhones have no WebXR AR in Safari. Opsis runs on plain web APIs (camera, motion sensors, WebGPU or WebGL2).
- **Light.** About 120 KB gzip in total, no dependencies. The heavy work runs in Rust compiled to WebAssembly, with image processing on the GPU.
- **Any engine.** Opsis does not own your scene. It gives you a camera pose; three.js, Babylon.js, PlayCanvas, A-Frame or raw WebGL/WebGPU draw with it.
- **Simple.** One function to start, one callback per frame.

## Use it

```js
import { start } from "opsis";
import { updateCamera } from "opsis/three";

startButton.onclick = async () => {                  // must be a user tap: iOS asks for sensor permission
  const session = await start({ mount: document.getElementById("app") });
  session.onFrame((frame) => {
    const aspect = innerWidth / innerHeight;
    if (!updateCamera(camera, frame, aspect)) return; // no pose yet
    const hit = frame.hitTest(0.5, 0.5, aspect)[0];   // floor under the screen centre
    if (hit && !cube.visible) (cube.position.fromArray(hit.position), (cube.visible = true));
    renderer.render(scene, camera);
  });
};
```

```html
<div id="app" style="position:fixed;inset:0">
  <canvas style="position:absolute;inset:0;width:100%;height:100%"></canvas> <!-- transparent engine canvas -->
</div>
```

Opsis puts the camera image in `#app` behind your canvas. Point the phone at the floor; tracking starts once enough floor is in view. The full API is in [packages/opsis/README.md](packages/opsis/README.md).

## How it works

```
 Phone                     opsis (TypeScript)                   Rust → WebAssembly
 ─────                     ──────────────────                   ──────────────────
 camera ───── video ─────▶ copies each frame to a small canvas ─▶ GPU: corners + descriptors
 motion sensors ─────────▶ orientation + acceleration samples ─▶ CPU: SLAM (map + pose)
                                     │                                   │
                                     ▼                                   │
 your app ◀── onFrame(frame) ◀── pose, state, hitTest ◀──────────────────┘
   │
   └─ adapter (opsis/three, /babylon, /playcanvas) sets your engine's camera
```

1. **Camera and sensors (TypeScript).** Opsis opens the back camera, asks for motion permission and keeps the latest orientation and acceleration readings.
2. **Front end (GPU).** Each frame is shrunk to 640 px and turned into an image pyramid. Corners (FAST) and their descriptors (ORB) are computed on the GPU, with WebGPU or WebGL2.
3. **SLAM (CPU, WebAssembly).** The phone's sensors give the camera's tilt. Vision finds its position and heading by matching corners against a map of floor and wall points. The map grows as the user moves, is refined over recent keyframes, and is found again after tracking is lost.
4. **Frame (TypeScript).** The pose is converted to engine conventions and handed to your callback, with a projection matching the camera image on screen and a floor hit test.

**World:** Y-up, right-handed, floor at y = 0, origin under the starting phone. The unit is the starting camera height; `frame.scale` gives metres once the motion sensor has seen enough movement.

## Repository

| Path | |
|---|---|
| `crates/core` | Pure Rust: image processing, corners, descriptors, SLAM. No web dependency; host tests and recorded-sequence replays. |
| `crates/wasm` | The browser side: WebGPU and WebGL2 passes, and the `Tracker` the SDK calls. |
| `packages/opsis` | The npm package: TypeScript SDK and engine adapters. |
| `examples/` | One page per engine, plus a diagnostic camera page with a live HUD and recording. |
| `tools/` | Build, local server, phone testing, replay scripts. |
| `docs/` | Milestones, research notes, reviews. |

## Develop
Needs Rust with the `wasm32-unknown-unknown` target, `wasm-bindgen`, `wasm-opt`, Node and pnpm.

```powershell
.\tools\build-web.ps1            # wasm + SDK into packages/opsis (add -Parity for the GPU/CPU check page)
cargo test --release             # Rust tests
pnpm -C packages/opsis test      # SDK tests
node tools/serve.mjs             # http://localhost:8081/examples/
.\tools\phone.ps1                # HTTPS tunnel + QR code to test on a phone
```

Phone recordings (`camera.html?record=1` with `.\tools\phone.ps1 -Record`) replay offline through the same tracker: `$env:OPSIS_REPLAY="recordings/<session>"; cargo test --release --test replay -- --ignored --nocapture`.

## Status
World tracking works on an iPhone 16e at 60 fps (milestone [M01](docs/milestones/M01_SLAM.md)); the SDK is in progress ([M02](docs/milestones/M02_SDK.md)). Known limits: height can creep a few percent on longer walks, metric scale is not yet reliable, and only one device has been measured. Image tracking and VPS are planned ([M00](docs/milestones/M00_WEBAR_LIBRARY.md)). Proprietary.
