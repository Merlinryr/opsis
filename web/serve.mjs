// Dev static server: only /web and /pkg (it may be exposed through a public tunnel). Sends COOP/COEP so wasm threads can be tested later.
// With WEBAR_RECORD=1 it also accepts POST /rec frames from camera.html?record=1 into recordings/<session>/ (see M01 V1 replay).
import { createServer } from "node:http";
import { readFile, writeFile, appendFile, mkdir } from "node:fs/promises";
import { extname, join, normalize, sep } from "node:path";

const root = normalize(join(import.meta.dirname, ".."));
const allowed = ["web", "pkg"].map((d) => join(root, d) + sep);
const types = { ".html": "text/html", ".js": "text/javascript", ".mjs": "text/javascript", ".wasm": "application/wasm" };
const port = Number(process.env.PORT ?? 8081);

const RECORD = process.env.WEBAR_RECORD === "1";
const MAX_FRAMES = 300; // per recording: ~10 s at 30 fps
const MAX_TOTAL = 3 * MAX_FRAMES; // per server run (public tunnel: bounds disk use)
const MAX_PIXELS = 1280 * 1280;
let frames = 0;

// Untrusted input (public tunnel): numbers only, server-chosen file names, size-capped body.
async function record(req, res) {
  const q = new URL(req.url, "http://x").searchParams;
  const [w, h, s, i] = ["w", "h", "s", "i"].map((k) => Number(q.get(k)));
  const nums = ["a", "b", "g", "fov", "t"].map((k) => Number(q.get(k)));
  const ok = [w, h, s, i].every(Number.isSafeInteger) && w > 0 && h > 0 && w * h <= MAX_PIXELS && s > 0 && s < 1e14 && i >= 0 && i < MAX_FRAMES;
  if (!ok || nums.some((v) => !Number.isFinite(v))) return reject(res, 400, `bad query (stale camera.html?) ${q}`);
  if (frames >= MAX_TOTAL) return reject(res, 429, "frame cap reached");
  const chunks = [];
  let size = 0;
  for await (const c of req) {
    size += c.length;
    if (size > w * h) return res.writeHead(413).end();
    chunks.push(c);
  }
  if (size !== w * h) return reject(res, 400, `body ${size} B, expected ${w * h}`);
  // Named by the client's capture index (uploads land out of order); one dir per client recording.
  const session = join(root, "recordings", new Date(s).toISOString().replace(/[:.]/g, "-"));
  await mkdir(session, { recursive: true });
  await writeFile(join(session, `${String(i).padStart(4, "0")}.luma`), Buffer.concat(chunks), { flag: "wx" }); // a repeated index fails
  frames++;
  await appendFile(join(session, "meta.csv"), `${i},${w},${h},${nums.join(",")}\n`);
  res.writeHead(204).end();
}

// Rejected uploads are logged once per reason: the phone HUD is the only other place they show.
const seen = new Set();
function reject(res, code, why) {
  if (!seen.has(code)) (seen.add(code), console.warn(`rec ${code}: ${why}`));
  res.writeHead(code).end();
}

createServer(async (req, res) => {
  if (req.method === "POST" && new URL(req.url, "http://x").pathname === "/rec") {
    return RECORD ? record(req, res).catch((e) => reject(res, 500, e.message)) : res.writeHead(403).end();
  }
  const path = normalize(join(root, decodeURIComponent(new URL(req.url, "http://x").pathname)));
  if (!allowed.some((dir) => (path + sep).startsWith(dir))) return res.writeHead(403).end();
  try {
    const file = path.endsWith("\\") || path.endsWith("/") ? join(path, "index.html") : path;
    const body = await readFile(file);
    res.writeHead(200, {
      "Content-Type": types[extname(file)] ?? "application/octet-stream",
      "Cross-Origin-Opener-Policy": "same-origin",
      "Cross-Origin-Embedder-Policy": "require-corp",
      "Cache-Control": "no-store", // phones kept a stale camera.html/wasm across server restarts
    }).end(body);
  } catch {
    res.writeHead(404).end();
  }
}).listen(port, "127.0.0.1", () => console.log(`serving ${root} on http://localhost:${port}/web/${RECORD ? ` (recording to recordings/)` : ""}`));
