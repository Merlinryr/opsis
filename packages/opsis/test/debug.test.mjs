import { test } from "node:test";
import assert from "node:assert/strict";
import { summarize } from "../dist/debug.js";

test("summarize: mean, p95, max over the window", () => {
  const v = Array.from({ length: 100 }, (_, i) => i + 1);
  assert.deepEqual(summarize(v), { mean: 50.5, p95: 96, max: 100 });
  assert.deepEqual(summarize([7]), { mean: 7, p95: 7, max: 7 });
  assert.ok(Number.isNaN(summarize([]).mean));
});
