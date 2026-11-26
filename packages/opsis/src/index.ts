import init, { Tracker, type InitInput } from "../wasm/opsis.js";

export type TrackingState = "initializing" | "normal" | "limited" | "lost";
export type Vec3 = [number, number, number];

export type SessionStatus = "running" | "paused" | "stopped";

export interface Hit {
  type: "floor";
  position: Vec3;
  /** Surface orientation: +y along the surface normal. */
  quaternion: [number, number, number, number];
  /** From the camera, world units. */
  distance: number;
}

export interface StartOptions {
  /** Element the camera image fills (`object-fit: cover`), under a transparent engine canvas. */
  mount?: HTMLElement;
  /**
   * `"video"`: the live `<video>`, full resolution, ~one detect (~10 ms) newer than the pose.
   * `"tracked"`: the tracked frame, drawn in sync with the pose, at `maxSide` resolution.
   */
  background?: "video" | "tracked";
  backend?: "auto" | "webgpu" | "webgl";
  /** Degrees across the long image side. Browsers expose no intrinsics. */
  fov?: number;
  /** ms the motion sensor leads the camera. */
  sensorDelay?: number;
  /** Processing size of the long side (px). */
  maxSide?: number;
  /** URL of `opsis_bg.wasm`, if a bundler does not resolve it. */
  wasm?: InitInput;
  /** false: sensor rotation only, no map (to tune `fov` and `sensorDelay`). */
  vision?: boolean;
  /** Diagnostic: pyramid levels (default 4). */
  levels?: number;
}

export interface Pose {
  /** World units: the camera height at start, until `Frame.scale` is known. */
  position: Vec3;
  quaternion: [number, number, number, number];
  /** Camera to world, column-major, GL camera axes (x right, y up, looking down -z). */
  matrix: Float32Array;
  viewMatrix: Float32Array;
}

export interface Session {
  readonly video: HTMLVideoElement;
  readonly backend: "webgpu" | "webgl2";
  readonly status: SessionStatus;
  /** Latest frame, for apps that render in their own loop. `image` is not valid outside `onFrame`. */
  readonly frame: Frame | null;
  onFrame(cb: (frame: Frame) => void): () => void;
  onStatus(cb: (status: SessionStatus) => void): () => void;
  onError(cb: (error: unknown) => void): () => void;
  /** Stops processing and the video; the map is kept. Also automatic while the page is hidden. */
  pause(): void;
  resume(): void;
  /** Drops the map: a new one starts with its origin under the current camera. */
  recenter(): void;
  stop(): void;
}

const STATES: TrackingState[] = ["initializing", "normal", "limited", "lost"];

let ready: Promise<unknown> | undefined;
export const load = (wasm?: InitInput) => (ready ??= init(wasm === undefined ? undefined : { module_or_path: wasm }));

/** World is Y-up, right-handed, floor at y = 0 (glTF / three.js convention). */
export class Frame {
  readonly state: TrackingState;
  readonly pose: Pose | null;
  /** Metres per world unit, null until the IMU estimate converges. */
  readonly scale: number | null;
  readonly inliers: number;
  readonly mapSize: number;
  readonly intrinsics: { f: number; cx: number; cy: number; width: number; height: number };
  private readonly r: Float32Array;
  private readonly c: Float32Array;

  constructor(
    raw: Float32Array,
    readonly time: number,
    /** Flat [x, y, level, score] per keypoint, in `image` pixels. */
    readonly keypoints: Float32Array,
    /** The tracked pixels. Valid only during the callback: the canvas is reused. */
    readonly image: OffscreenCanvas,
    /** Device orientation (W3C degrees) and gravity-free acceleration (m/s^2, device axes) used, `t`: sample time. */
    readonly sensor: { alpha: number; beta: number; gamma: number; t: number } | null,
    readonly motion: { x: number; y: number; z: number; t: number } | null,
    /** ms: `copy` (video to canvas), `detect`, and its `split` [GPU encode, GPU wait + readback, track], then on WebGPU [GPU done, mapped, unpacked]. */
    readonly stats: { copy: number; detect: number; split: Float64Array; threshold: number },
  ) {
    const { width, height } = image;
    this.state = STATES[raw[0] ?? 0];
    this.inliers = raw[1] ?? 0;
    this.mapSize = raw[2] ?? 0;
    this.c = raw.subarray(3, 6);
    this.r = raw.subarray(6, 15);
    this.intrinsics = raw.length ? { f: raw[15], cx: raw[16], cy: raw[17], width, height } : { f: width / 2 / Math.tan(0.6), cx: width / 2, cy: height / 2, width, height };
    this.scale = raw.length && !Number.isNaN(raw[18]) ? raw[18] : null;
    this.pose = raw.length ? pose(this.r, this.c) : null;
  }

  /** Image height visible in a viewport of this aspect, video drawn with `object-fit: cover`. */
  private visible(aspect: number) {
    const { width, height } = this.intrinsics;
    const h = Math.min(height, width / aspect);
    return [h * aspect, h];
  }

  /** Vertical field of view (radians) for a viewport of `aspect` (width / height). */
  fovY(aspect: number) {
    return 2 * Math.atan(this.visible(aspect)[1] / 2 / this.intrinsics.f);
  }

  /** Column-major perspective matrix. `zeroToOne`: WebGPU depth range. */
  projectionMatrix(aspect: number, near = 0.01, far = 100, zeroToOne = false) {
    const f = 1 / Math.tan(this.fovY(aspect) / 2), nf = 1 / (near - far);
    const m = new Float32Array(16);
    m[0] = f / aspect;
    m[5] = f;
    m[10] = zeroToOne ? far * nf : (far + near) * nf;
    m[11] = -1;
    m[14] = (zeroToOne ? 1 : 2) * far * near * nf;
    return m;
  }

  /** Surfaces under viewport point (u, v) in [0, 1], top-left origin, nearest first. Today: the floor. */
  hitTest(u: number, v: number, aspect: number): Hit[] {
    if (!this.pose) return [];
    const [w, h] = this.visible(aspect), f = this.intrinsics.f, r = this.r, c = this.c;
    const d = [(u - 0.5) * w / f, (v - 0.5) * h / f, 1];
    const dz = r[6] * d[0] + r[7] * d[1] + r[8] * d[2];
    const t = -c[2] / dz;
    if (!(t > 0) || t > 50) return [];
    const x = c[0] + t * (r[0] * d[0] + r[1] * d[1] + r[2] * d[2]);
    const y = c[1] + t * (r[3] * d[0] + r[4] * d[1] + r[5] * d[2]);
    return [{ type: "floor", position: [x, 0, -y], quaternion: [0, 0, 0, 1], distance: t * Math.hypot(d[0], d[1], d[2]) }];
  }
}

// Tracker world: x east, y north, z up; camera: x right, y down, z forward. Here: Y-up world, GL camera.
function pose(r: Float32Array, c: Float32Array): Pose {
  const col = (j: number, s: number) => [s * r[j], s * r[6 + j], -s * r[3 + j]];
  const [x, y, z] = [col(0, 1), col(1, -1), col(2, -1)];
  const p: Vec3 = [c[0], c[2], -c[1]];
  const matrix = new Float32Array([...x, 0, ...y, 0, ...z, 0, ...p, 1]);
  const dot = (a: number[]) => a[0] * p[0] + a[1] * p[1] + a[2] * p[2];
  const viewMatrix = new Float32Array([x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0, -dot(x), -dot(y), -dot(z), 1]);
  return { position: p, quaternion: quaternion(x, y, z), matrix, viewMatrix };
}

function quaternion(x: number[], y: number[], z: number[]): [number, number, number, number] {
  const tr = x[0] + y[1] + z[2];
  if (tr > 0) {
    const s = 0.5 / Math.sqrt(tr + 1);
    return [(y[2] - z[1]) * s, (z[0] - x[2]) * s, (x[1] - y[0]) * s, 0.25 / s];
  }
  if (x[0] > y[1] && x[0] > z[2]) {
    const s = 2 * Math.sqrt(1 + x[0] - y[1] - z[2]);
    return [0.25 * s, (y[0] + x[1]) / s, (z[0] + x[2]) / s, (y[2] - z[1]) / s];
  }
  if (y[1] > z[2]) {
    const s = 2 * Math.sqrt(1 + y[1] - x[0] - z[2]);
    return [(y[0] + x[1]) / s, 0.25 * s, (z[1] + y[2]) / s, (z[0] - x[2]) / s];
  }
  const s = 2 * Math.sqrt(1 + z[2] - x[0] - y[1]);
  return [(z[0] + x[2]) / s, (z[1] + y[2]) / s, 0.25 * s, (x[1] - y[0]) / s];
}

type Sample<T> = T & { t: number };
const nearest = <T>(list: Sample<T>[], t: number) =>
  list.reduce<Sample<T> | null>((best, s) => (!best || Math.abs(s.t - t) < Math.abs(best.t - t) ? s : best), null);

/** Call from a user gesture: iOS asks for motion permission on the first call. Rejects if the camera is unavailable. */
export function start(options: StartOptions = {}): Promise<Session> {
  const DOE = DeviceOrientationEvent as unknown as { requestPermission?: () => Promise<string> };
  const DME = DeviceMotionEvent as unknown as { requestPermission?: () => Promise<string> };
  // iOS: both prompts must be requested synchronously in the gesture, before any await.
  const orientation = DOE.requestPermission?.() ?? Promise.resolve("granted");
  DME.requestPermission?.().catch(() => {});
  return run(options, orientation);
}

async function run(o: StartOptions, permission: Promise<string>): Promise<Session> {
  const { fov = 69, sensorDelay = 0, maxSide = 640, vision = true } = o;
  // Denied or absent sensors are not fatal: frames then carry no pose (`Frame.sensor` null).
  await permission.catch(() => "denied");
  await load(o.wasm);
  const gpu = (navigator as { gpu?: { requestAdapter(): Promise<unknown> } }).gpu;
  const webgpu = o.backend !== "webgl" && !!gpu && !!(await gpu.requestAdapter());
  if (o.backend === "webgpu" && !webgpu) throw new Error("WebGPU unavailable");

  // Euler angles are singular in upright portrait, so frames take the nearest sample, not an interpolation.
  const orient: Sample<{ alpha: number; beta: number; gamma: number }>[] = [];
  const motion: Sample<{ x: number; y: number; z: number }>[] = [];
  const onOrient = (e: DeviceOrientationEvent) => {
    if (e.alpha === null || e.beta === null || e.gamma === null) return;
    orient.push({ t: performance.now(), alpha: e.alpha, beta: e.beta, gamma: e.gamma });
    if (orient.length > 60) orient.shift();
  };
  const onMotion = (e: DeviceMotionEvent) => {
    const a = e.acceleration;
    if (a?.x == null || a.y == null || a.z == null) return;
    motion.push({ t: performance.now(), x: a.x, y: a.y, z: a.z });
    if (motion.length > 120) motion.shift();
  };
  addEventListener("deviceorientation", onOrient);
  addEventListener("devicemotion", onMotion);

  const video = document.createElement("video");
  Object.assign(video, { playsInline: true, muted: true });
  const stream = await navigator.mediaDevices.getUserMedia({
    video: { facingMode: "environment", width: { ideal: 1280 }, height: { ideal: 720 }, frameRate: { ideal: 60 } },
    audio: false,
  });
  video.srcObject = stream;
  await video.play();
  const cover = "position:absolute;inset:0;width:100%;height:100%;object-fit:cover";
  const tracked = o.background === "tracked" ? document.createElement("canvas") : null;
  const bg = tracked ?? video;
  bg.style.cssText = cover;
  o.mount?.prepend(bg);

  const tracker = await Tracker.create(document.createElement("canvas"), webgpu, fov);
  if (o.levels) tracker.set_levels(o.levels);
  const k = maxSide / Math.max(video.videoWidth, video.videoHeight);
  const w = Math.round(video.videoWidth * k), h = Math.round(video.videoHeight * k);
  // Two frames in flight, one canvas each: the next frame is copied and submitted while this one waits
  // on the GPU. No willReadFrequently: it keeps the canvas on the CPU.
  const slots = [0, 1].map(() => new OffscreenCanvas(w, h).getContext("2d")!);
  const bgCtx = tracked && Object.assign(tracked, { width: w, height: h }).getContext("2d")!;
  const frameCbs = new Set<(f: Frame) => void>(), statusCbs = new Set<(s: SessionStatus) => void>(), errorCbs = new Set<(e: unknown) => void>();
  let inFlight = 0, slot = 0, gotVfc = false, vfcGaveUp = false, latest: Frame | null = null;
  let status: SessionStatus = "running", userPaused = false;
  const setStatus = (next: SessionStatus) => {
    if (status === next) return;
    status = next;
    for (const cb of statusCbs) cb(next);
  };

  const frame = async (now: number, meta?: VideoFrameCallbackMetadata) => {
    if (status !== "running") return;
    if (meta) gotVfc = true;
    next();
    if (inFlight >= 2) return;
    inFlight++;
    const ctx = slots[slot++ & 1];
    try {
      const tc = performance.now();
      ctx.drawImage(video, 0, 0, w, h);
      const time = meta?.captureTime ?? meta?.expectedDisplayTime ?? now;
      const s = nearest(orient, time - sensorDelay), m = nearest(motion, time - sensorDelay);
      const t0 = performance.now();
      const kps = await tracker.detect(ctx.canvas, s?.alpha ?? NaN, s?.beta ?? NaN, s?.gamma ?? NaN, screen.orientation?.angle ?? 0, time, m?.x ?? NaN, m?.y ?? NaN, m?.z ?? NaN, vision);
      if (session.status === "stopped") return;
      const f = new Frame(tracker.pose(), time, kps, ctx.canvas, s, m, { copy: t0 - tc, detect: performance.now() - t0, split: tracker.split_ms(), threshold: tracker.threshold() });
      latest = f;
      bgCtx?.drawImage(ctx.canvas, 0, 0);
      for (const cb of frameCbs) cb(f);
    } catch (e) {
      for (const cb of errorCbs) cb(e);
      if (!errorCbs.size) console.error(e);
    } finally {
      inFlight--;
    }
  };
  // requestVideoFrameCallback never fires for some hidden videos: fall back to rAF.
  const next = () => ("requestVideoFrameCallback" in video && !vfcGaveUp ? video.requestVideoFrameCallback(frame) : requestAnimationFrame(frame));
  setTimeout(() => (vfcGaveUp = !gotVfc) && requestAnimationFrame(frame), 500);
  next();

  const pause = () => {
    if (status !== "running") return;
    video.pause();
    setStatus("paused");
  };
  const resume = () => {
    if (status !== "paused") return;
    setStatus("running");
    video.play().then(next, (e) => errorCbs.forEach((cb) => cb(e)));
  };
  const onVisibility = () => (document.hidden ? pause() : !userPaused && resume());
  document.addEventListener("visibilitychange", onVisibility);
  const fail = () => {
    for (const cb of errorCbs) cb(new Error("camera stream ended"));
    session.stop();
  };
  for (const t of stream.getTracks()) t.addEventListener("ended", fail);

  const sub = <T>(set: Set<T>) => (cb: T) => (set.add(cb), () => void set.delete(cb));
  const session: Session = {
    video,
    backend: webgpu ? "webgpu" : "webgl2",
    get status() { return status; },
    get frame() { return latest; },
    onFrame: sub(frameCbs),
    onStatus: sub(statusCbs),
    onError: sub(errorCbs),
    pause() { userPaused = true; pause(); },
    resume() { userPaused = false; resume(); },
    recenter() { tracker.reset(); },
    stop() {
      if (status === "stopped") return;
      setStatus("stopped");
      removeEventListener("deviceorientation", onOrient);
      removeEventListener("devicemotion", onMotion);
      document.removeEventListener("visibilitychange", onVisibility);
      for (const t of stream.getTracks()) (t.removeEventListener("ended", fail), t.stop());
      bg.remove();
      // Frees after the in-flight detects resolve: wasm-bindgen objects must not be freed mid-call.
      const free = () => (inFlight ? setTimeout(free, 50) : tracker.free());
      free();
    },
  };
  return session;
}
