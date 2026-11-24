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
const MAX_FRAMES = 300; // per server run: ~10 s at 30 fps
const MAX_PIXELS = 1280 * 1280;
const session = join(root, "recordings", new Date().toISOString().replace(/[:.]/g, "-"));
let frames = 0;

// Untrusted input (public tunnel): numbers only, server-chosen file names, size-capped body.
async function record(req, res) {
  const q = new URL(req.url, "http://x").searchParams;
  const [w, h] = ["w", "h"].map((k) => Number.parseInt(q.get(k), 10));
  const angles = ["a", "b", "g", "fov"].map((k) => Number(q.get(k)));
  if (!(w > 0 && h > 0 && w * h <= MAX_PIXELS) || angles.some((v) => !Number.isFinite(v))) return res.writeHead(400).end();
  if (frames >= MAX_FRAMES) return res.writeHead(429).end();
  const chunks = [];
  let size = 0;
  for await (const c of req) {
    size += c.length;
    if (size > w * h) return res.writeHead(413).end();
    chunks.push(c);
  }
  if (size !== w * h) return res.writeHead(400).end();
  const n = frames++;
  await mkdir(session, { recursive: true });
  await writeFile(join(session, `${String(n).padStart(4, "0")}.luma`), Buffer.concat(chunks));
  await appendFile(join(session, "meta.csv"), `${n},${w},${h},${angles.join(",")}\n`);
  res.writeHead(204).end();
}

createServer(async (req, res) => {
  if (req.method === "POST" && new URL(req.url, "http://x").pathname === "/rec") {
    return RECORD ? record(req, res).catch(() => res.writeHead(500).end()) : res.writeHead(403).end();
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
    }).end(body);
  } catch {
    res.writeHead(404).end();
  }
}).listen(port, "127.0.0.1", () => console.log(`serving ${root} on http://localhost:${port}/web/${RECORD ? ` (recording to ${session})` : ""}`));
