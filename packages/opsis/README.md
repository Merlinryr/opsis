# Opsis

Markerless world tracking (SLAM) for the mobile web, on WebGPU with a WebGL2 fallback. Engine-agnostic, zero dependencies: ~120 KB gzip in total (wasm 103, glue 13, SDK 4).

```js
import { start } from "opsis";
import { updateCamera } from "opsis/three"; // or "opsis/babylon", "opsis/playcanvas", or the matrices directly

button.onclick = async () => {
  const session = await start({ mount: document.getElementById("app") }); // must run in a user gesture (iOS)
  session.onFrame((frame) => {
    const aspect = innerWidth / innerHeight;
    if (!updateCamera(camera, frame, aspect)) return; // no pose yet
    if (!placed && frame.state === "normal") placed = frame.hitTest(0.5, 0.5, aspect)[0]?.position; // floor under the centre
    renderer.render(scene, camera);
  });
};
```

The camera image fills `mount` (`object-fit: cover`); put a transparent engine canvas on top of it, in the same element.

Apps that render in their own loop read `session.frame` (the latest frame) instead of subscribing:

```js
engine.runRenderLoop(() => session.frame && updateCamera(camera, session.frame, aspect));
```

## World
- Y-up, right-handed, floor at y = 0, origin under the starting camera (glTF / three.js convention). Babylon: `scene.useRightHandedSystem = true`.
- Unit: the camera height at start. `frame.scale` gives metres per unit once the motion sensor has seen enough movement, `null` before.

## API
`start(options?) → Promise<Session>`: asks for motion sensor permission (iOS), opens the back camera, loads the wasm. Rejects if the camera is unavailable. Denied sensors are not fatal: frames then have no pose.

| Option | Default | |
|---|---|---|
| `mount` | none | Element the camera image fills. |
| `background` | `"tracked"` | `"tracked"`: the camera frame the pose was computed from, at camera resolution, so content stays locked to the image. `"video"`: the live `<video>`, 1–2 frames newer than the pose, so content lags the image while the phone moves (one less full-size copy per frame). |
| `backend` | `"auto"` | `"webgpu"`, `"webgl"`. |
| `fov` | `69` | Degrees across the long side (browsers expose no intrinsics). |
| `sensorDelay` | `0` | ms the motion sensor leads the camera. |
| `maxSide` | `640` | Processing size of the long side, px. |
| `wasm` | bundled | URL of `opsis_bg.wasm` if your bundler does not resolve it. |

`load(wasm?)`: fetch and compile the wasm early (optional).

`Session`:

| Member | |
|---|---|
| `onFrame(cb)`, `onStatus(cb)`, `onError(cb)` | Each returns an unsubscribe function. |
| `frame` | Latest frame (pull model). |
| `status` | `"running"`, `"paused"`, `"stopped"`. |
| `pause()`, `resume()` | Stop and restart processing and the video; the map is kept. Automatic while the page is hidden. |
| `recenter()` | Drops the map; a new one starts with its origin under the current camera. |
| `stop()` | Releases the camera, sensors and GPU resources. |
| `video`, `backend` | The camera `<video>`; `"webgpu"` or `"webgl2"`. |

`Frame`, once per processed camera frame:

| Member | |
|---|---|
| `state` | `"initializing"` (point at the floor), `"normal"`, `"limited"`, `"lost"` (last position, live sensor rotation). |
| `pose` | `{ position, quaternion, matrix, viewMatrix }` or `null` before the first sensor reading. `matrix`: camera to world, column-major, GL camera axes. |
| `fovY(aspect)` | Vertical field of view (radians) for a viewport of `aspect`. |
| `projectionMatrix(aspect, near?, far?, zeroToOne?)` | Column-major; `zeroToOne` for WebGPU depth. |
| `hitTest(u, v, aspect)` | Surfaces under viewport point (u, v) in [0, 1], top-left origin, nearest first: `[{ type, position, quaternion, distance }]`. Today the only type is `"floor"`. |
| `scale`, `inliers`, `mapSize`, `time` | |
| `keypoints`, `image`, `intrinsics`, `sensor`, `motion`, `stats` | Diagnostics. `image` is reused: read it in the callback only. |

## Adapters
- `opsis/three`: `updateCamera(perspectiveCamera, frame, aspect)`.
- `opsis/babylon`: `updateCamera(camera, frame, aspect)`; needs a right-handed scene and `camera.rotationQuaternion`.
- `opsis/playcanvas`: `updateCamera(cameraEntity, frame, aspect)`.
- A-Frame: `opsis/three` on `cameraEl.getObject3D("camera")`, with `camera="userHeight: 0"` and look-controls off.

Adapters take structural types, so they work with any engine version and add no dependency.

## Debug HUD
`opsis/debug` (3.4 KB gzip, loaded only if you import it) draws tracking diagnostics over your scene and a benchmark panel.

```js
import { debug } from "opsis/debug";
const hud = debug(session); // or debug(session, { corners: false, grid: true, panel: true, window: 120 })
```

- **Overlay:** detected corners coloured by pyramid level (cyan, purple, yellow, pink = fine to coarse), the floor grid (y = 0) tinted by tracking state (green normal, amber limited, red lost, grey initializing) and the world axes at the origin (x red, y green, z blue).
- **Panel:** state, fps, detect time (mean, p95, max over the last `window` frames), the copy / GPU / track split, corners, inliers, map size, position, scale, sensor age. Tap it to collapse; buttons toggle the corners and the grid.
- **Copy report:** the benchmark as JSON (device, backend, sizes, fps, per-stage ms, state shares) to the clipboard, and to the console.
- `hud.note(text)` adds a line, `hud.report()` returns the JSON, `hud.toggle()` shows or hides it, `hud.dispose()` removes it.

The examples load it lazily with `?debug` in the URL.

## Requirements
HTTPS, a back camera, and `deviceorientation`. WebGPU when available, else WebGL2.
