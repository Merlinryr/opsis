// Dev static server: only /web and /pkg (it may be exposed through a public tunnel). Sends COOP/COEP so wasm threads can be tested later.
import { createServer } from "node:http";
import { readFile } from "node:fs/promises";
import { extname, join, normalize, sep } from "node:path";

const root = normalize(join(import.meta.dirname, ".."));
const allowed = ["web", "pkg"].map((d) => join(root, d) + sep);
const types = { ".html": "text/html", ".js": "text/javascript", ".mjs": "text/javascript", ".wasm": "application/wasm" };
const port = Number(process.env.PORT ?? 8081);

createServer(async (req, res) => {
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
}).listen(port, "127.0.0.1", () => console.log(`serving ${root} on http://localhost:${port}/web/`));
