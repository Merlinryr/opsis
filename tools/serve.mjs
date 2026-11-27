// Dev server for /examples and /packages only, since it may be exposed through a public tunnel.
// With OPSIS_RECORD=1 it also saves frames posted by camera.html?record=1 into recordings/<session>/.
import { createServer } from "node:http";
import { readFile, writeFile, appendFile, mkdir } from "node:fs/promises";
import { extname, join, normalize, sep } from "node:path";

const root = normalize(join(import.meta.dirname, ".."));
const allowed = ["examples", "packages"].map((d) => join(root, d) + sep);
const types = { ".html": "text/html", ".css": "text/css", ".js": "text/javascript", ".mjs": "text/javascript", ".wasm": "application/wasm" };
const port = Number(process.env.PORT ?? 8081);

const RECORD = process.env.OPSIS_RECORD === "1";
const MAX_FRAMES = 300; // Per recording (~10 s at 30 fps).
const MAX_TOTAL = 3 * MAX_FRAMES; // Per server run, to cap disk use behind a public tunnel.
const MAX_PIXELS = 1280 * 1280;
let frames = 0;

// Untrusted input: numbers only, file names chosen by the server, body size capped.
async function record(req, res) {
  const q = new URL(req.url, "http://x").searchParams;
  const [w, h, s, i] = ["w", "h", "s", "i"].map((k) => Number(q.get(k)));
  // ax, ay, az are optional (no motion sensor): written as NaN.
  const nums = ["a", "b", "g", "fov", "t"].map((k) => Number(q.get(k)));
  const accel = ["ax", "ay", "az"].map((k) => (q.has(k) ? Number(q.get(k)) : NaN));
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
  // One folder per recording; files named by capture index, since uploads arrive out of order.
  const session = join(root, "recordings", new Date(s).toISOString().replace(/[:.]/g, "-"));
  await mkdir(session, { recursive: true });
  await writeFile(join(session, `${String(i).padStart(4, "0")}.luma`), Buffer.concat(chunks), { flag: "wx" }); // Fails on a repeated index.
  frames++;
  await appendFile(join(session, "meta.csv"), `${i},${w},${h},${[...nums, ...accel].join(",")}\n`);
  res.writeHead(204).end();
}

// Log each rejection reason once (the phone HUD is the only other place it shows).
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
      "Cache-Control": "no-store", // Phones kept stale pages and wasm across server restarts.
    }).end(body);
  } catch {
    res.writeHead(404).end();
  }
}).listen(port, "127.0.0.1", () => console.log(`serving ${root} on http://localhost:${port}/examples/${RECORD ? ` (recording to recordings/)` : ""}`));
