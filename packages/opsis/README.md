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
| `background` | `"video"` | `"video"`: live `<video>`, full resolution, ~10 ms newer than the pose. `"tracked"`: the tracked frame, in sync with the pose, at `maxSide` resolution. |
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

## Requirements
HTTPS, a back camera, and `deviceorientation`. WebGPU when available, else WebGL2.
