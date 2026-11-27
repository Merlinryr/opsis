import type { Frame, Session, TrackingState } from "./index.js";

export interface DebugOptions {
  /** Defaults to `session.mount`, else the page (fixed, full screen). */
  mount?: HTMLElement;
  /** Detected corners, coloured by pyramid level. */
  corners?: boolean;
  /** Floor wireframe (y = 0) tinted by tracking state, and world axes at the origin. */
  grid?: boolean;
  /** Stats panel. */
  panel?: boolean;
  /** Frames in the rolling benchmark window. */
  window?: number;
}

export interface Debug {
  /** Extra line in the panel (e.g. a recording counter). */
  note(text: string): void;
  /** Benchmark over the rolling window, as shown by the Copy button. */
  report(): Report;
  /** Shows or hides everything. */
  toggle(visible?: boolean): void;
  dispose(): void;
}

export interface Stat { mean: number; p95: number; max: number }

export interface Report {
  userAgent: string;
  backend: string;
  image: string;
  viewport: string;
  frames: number;
  fps: number;
  ms: Record<string, Stat>;
  keypoints: number;
  /** Share of frames per tracking state. */
  states: Record<TrackingState, number>;
  mapSize: number;
  scale: number | null;
}

export const STATE_COLORS: Record<TrackingState, string> = { initializing: "#8e8e93", normal: "#34c759", limited: "#ff9f0a", lost: "#ff3b30" };
export const LEVEL_COLORS = ["#64d2ff", "#bf5af2", "#ffd60a", "#ff375f"];
const SPLIT = ["gpu encode", "gpu wait", "track", "gpu done", "mapped", "unpacked"];

export function summarize(values: number[]): Stat {
  if (!values.length) return { mean: NaN, p95: NaN, max: NaN };
  const sorted = [...values].sort((a, b) => a - b);
  return { mean: values.reduce((a, b) => a + b, 0) / values.length, p95: sorted[Math.min(sorted.length - 1, Math.floor(0.95 * sorted.length))], max: sorted[sorted.length - 1] };
}

interface Sample { t: number; state: TrackingState; ms: number[]; keypoints: number }

/** On-screen tracking diagnostics and a benchmark panel. Zero cost unless imported. */
export function debug(session: Session, options: DebugOptions = {}): Debug {
  const { corners = true, grid = true, panel = true, window: size = 120 } = options;
  const mount = options.mount ?? session.mount ?? document.body;
  const fixed = !options.mount && !session.mount;
  const layer = (el: HTMLElement, css: string) => (el.style.cssText = `${fixed ? "position:fixed" : "position:absolute"};${css}`, mount.append(el), el);

  const overlay = layer(document.createElement("canvas"), "inset:0;width:100%;height:100%;pointer-events:none;z-index:9998") as HTMLCanvasElement;
  const ctx = overlay.getContext("2d")!;
  const box = layer(document.createElement("div"), [
    "top:max(8px,env(safe-area-inset-top));left:max(8px,env(safe-area-inset-left));z-index:9999",
    "max-width:calc(100% - 16px);padding:6px 8px;border-radius:8px;background:#000a;backdrop-filter:blur(6px)",
    "color:#fff;font:11px/1.45 ui-monospace,Menlo,monospace;white-space:pre;user-select:none;-webkit-user-select:none",
  ].join(";"));
  box.hidden = !panel;
  const head = box.appendChild(document.createElement("div"));
  const dot = head.appendChild(document.createElement("span"));
  const title = head.appendChild(document.createElement("span"));
  dot.textContent = "● ";
  title.textContent = "waiting for the first frame…";
  const text = box.appendChild(document.createElement("div"));
  const bar = box.appendChild(document.createElement("div"));
  bar.style.cssText = "display:flex;gap:6px;margin-top:4px";
  const button = (label: string, onclick: () => void) => {
    const b = bar.appendChild(document.createElement("button"));
    b.textContent = label;
    b.style.cssText = "font:inherit;color:#fff;background:#fff2;border:0;border-radius:5px;padding:3px 7px";
    b.onclick = (e) => (e.stopPropagation(), onclick());
    return b;
  };

  const layers = { corners, grid };
  let collapsed = false, visible = true, note = "", last: Frame | null = null;
  const samples: Sample[] = [];
  const copy = button("Copy report", () => {
    const json = JSON.stringify(report(), null, 2);
    console.log(json);
    navigator.clipboard?.writeText(json).then(() => flash("Copied"), () => flash("Logged to console"));
  });
  const flash = (msg: string) => ((copy.textContent = msg), setTimeout(() => (copy.textContent = "Copy report"), 1200));
  const toggleLayer = (k: keyof typeof layers, label: string) => {
    const b = button(label, () => ((layers[k] = !layers[k]), (b.style.opacity = layers[k] ? "1" : "0.45")));
    b.style.opacity = layers[k] ? "1" : "0.45";
  };
  toggleLayer("corners", "Corners");
  toggleLayer("grid", "Grid");
  box.onclick = () => ((collapsed = !collapsed), (bar.hidden = text.hidden = collapsed), last && draw(last));

  function report(): Report {
    const n = samples.length, f = last;
    const span = n > 1 ? (samples[n - 1].t - samples[0].t) / 1000 : 0;
    const ms: Record<string, Stat> = {};
    ["copy", "detect", ...SPLIT].forEach((k, i) => {
      const v = samples.map((s) => s.ms[i]).filter((x) => x !== undefined);
      if (v.length) ms[k] = summarize(v);
    });
    const states = { initializing: 0, normal: 0, limited: 0, lost: 0 };
    for (const s of samples) states[s.state] += 1 / n;
    return {
      userAgent: navigator.userAgent,
      backend: session.backend,
      image: f ? `${f.image.width}x${f.image.height}` : "",
      viewport: `${innerWidth}x${innerHeight}@${devicePixelRatio}`,
      frames: n,
      fps: span ? (n - 1) / span : NaN,
      ms,
      keypoints: summarize(samples.map((s) => s.keypoints)).mean,
      states,
      mapSize: f?.mapSize ?? 0,
      scale: f?.scale ?? null,
    };
  }

  function draw(f: Frame) {
    const r = overlay.getBoundingClientRect(), dpr = devicePixelRatio;
    const W = Math.round(r.width * dpr), H = Math.round(r.height * dpr);
    if (overlay.width !== W || overlay.height !== H) (overlay.width = W), (overlay.height = H);
    ctx.clearRect(0, 0, W, H);
    // Corners live in tracked-image pixels; the image is shown with object-fit: cover.
    const { width: w, height: h } = f.image, s = Math.max(W / w, H / h), ox = (W - w * s) / 2, oy = (H - h * s) / 2;
    if (layers.corners) {
      ctx.lineWidth = 1.5 * dpr;
      const k = f.keypoints;
      for (let level = 0; level < LEVEL_COLORS.length; level++) {
        ctx.strokeStyle = LEVEL_COLORS[level];
        ctx.beginPath();
        for (let i = 0; i < k.length; i += 4) {
          if (k[i + 2] !== level) continue;
          const x = ox + k[i] * s, y = oy + k[i + 1] * s, rad = 2.5 * dpr * 1.44 ** level;
          ctx.moveTo(x + rad, y);
          ctx.arc(x, y, rad, 0, 2 * Math.PI);
        }
        ctx.stroke();
      }
    }
    if (layers.grid && f.pose) {
      const P = f.projectionMatrix(W / H), V = f.pose.viewMatrix;
      const clip = ([x, y, z]: number[]) => [0, 1, 3].map((i) => {
        const v = [0, 1, 2, 3].map((j) => V[j] * x + V[4 + j] * y + V[8 + j] * z + V[12 + j]);
        return P[i] * v[0] + P[4 + i] * v[1] + P[8 + i] * v[2] + P[12 + i] * v[3];
      });
      const px = (c: number[]) => (c[2] > 0.02 ? [(c[0] / c[2] + 1) * W / 2, (1 - c[1] / c[2]) * H / 2] : null);
      const line = (a: number[], b: number[], n: number) => {
        for (let i = 0; i < n; i++) {
          const p = px(clip(a.map((v, j) => v + (b[j] - v) * i / n))), q = px(clip(a.map((v, j) => v + (b[j] - v) * (i + 1) / n)));
          if (p && q) (ctx.moveTo(p[0], p[1]), ctx.lineTo(q[0], q[1]));
        }
      };
      ctx.lineWidth = dpr;
      ctx.strokeStyle = STATE_COLORS[f.state] + "99";
      ctx.beginPath();
      for (let g = -3; g <= 3; g += 0.5) (line([g, 0, -3], [g, 0, 3], 24), line([-3, 0, g], [3, 0, g], 24));
      ctx.stroke();
      ctx.lineWidth = 3 * dpr;
      ["#ff3b30", "#34c759", "#0a84ff"].forEach((color, a) => {
        ctx.strokeStyle = color;
        ctx.beginPath();
        line([0, 0, 0], [0, 1, 2].map((j) => (j === a ? 0.3 : 0)), 1);
        ctx.stroke();
      });
    }
    if (box.hidden) return;
    const rep = report();
    dot.style.color = STATE_COLORS[f.state];
    title.textContent = `${f.state.toUpperCase()}  ${Number.isFinite(rep.fps) ? rep.fps.toFixed(1) : "-"} fps${collapsed ? "" : `  ${session.backend} ${f.image.width}×${f.image.height}`}`;
    if (!collapsed) text.textContent = panelText(f, rep);
  }

  function panelText(f: Frame, rep: Report) {
    const m = rep.ms, fx = (v: number | undefined, d = 1) => (v !== undefined && Number.isFinite(v) ? v.toFixed(d) : "-");
    const split = f.stats.split;
    return [
      `detect ${fx(m.detect?.mean)} ms  p95 ${fx(m.detect?.p95)}  max ${fx(m.detect?.max)}`,
      `copy ${fx(m.copy?.mean)} · gpu ${fx(m["gpu encode"]?.mean)}+${fx(m["gpu wait"]?.mean)} · track ${fx(m.track?.mean)} ms`,
      split.length > 3 ? `wait: done ${fx(m["gpu done"]?.mean)} · mapped ${fx(m.mapped?.mean)} · unpack ${fx(m.unpacked?.mean)}` : "",
      `corners ${f.keypoints.length / 4} (t ${f.stats.threshold}) · inliers ${f.inliers} · map ${f.mapSize}`,
      f.pose ? `pos ${f.pose.position.map((v) => fx(v, 2)).join(" ")} · scale ${f.scale ? `${fx(f.scale, 2)} m/u` : "-"}` : "no pose: " + (f.sensor ? "point at the floor" : "no orientation sensor"),
      `sensor ${f.sensor ? `${fx(f.time - f.sensor.t, 0)} ms` : "none"} · motion ${f.motion ? "yes" : "none"}`,
      note,
    ].filter(Boolean).join("\n");
  }

  const off = session.onFrame((f) => {
    last = f;
    const st = f.stats;
    samples.push({ t: performance.now(), state: f.state, ms: [st.copy, st.detect, ...st.split], keypoints: f.keypoints.length / 4 });
    if (samples.length > size) samples.shift();
    if (visible) draw(f);
  });

  return {
    note: (t) => void (note = t),
    report,
    toggle(v = !visible) {
      visible = v;
      overlay.hidden = !v;
      box.hidden = !v || !panel;
    },
    dispose() {
      off();
      overlay.remove();
      box.remove();
    },
  };
}
