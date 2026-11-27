import { test } from "node:test";
import assert from "node:assert/strict";
import { Frame } from "../dist/index.js";

const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const mul = (M, v) => [0, 1, 2, 3].map((i) => M[i] * v[0] + M[4 + i] * v[1] + M[8 + i] * v[2] + M[12 + i] * v[3]);
const close = (a, b, eps = 1e-5) => assert.ok(Math.abs(a - b) < eps, `${a} vs ${b}`);

// Fake tracker output: a camera 1 unit above the floor, in tracker axes (world z up, camera y down).
function frame(yaw, pitch, roll) {
  const fwd = [Math.sin(yaw) * Math.cos(pitch), Math.cos(yaw) * Math.cos(pitch), -Math.sin(pitch)];
  const r0 = [Math.cos(yaw), -Math.sin(yaw), 0], d0 = cross(fwd, r0);
  const right = r0.map((v, i) => Math.cos(roll) * v + Math.sin(roll) * d0[i]), down = cross(fwd, right);
  const R = [right[0], down[0], fwd[0], right[1], down[1], fwd[1], right[2], down[2], fwd[2]];
  const raw = new Float32Array([1, 100, 500, 0.3, -0.2, 1, ...R, 460, 179.5, 319.5, NaN]);
  return new Frame(raw, 0, new Float32Array(), { width: 360, height: 640 }, null, null, {});
}
const poses = [[0.5, 0.8, 0], [2.5, 0.3, 0.2], [-2.0, 1.4, -0.4], [3.1, 0.05, 0.1]];

test("pose: proper rotation, Y-up position, quaternion and view matrix agree", () => {
  for (const p of poses) {
    const { pose } = frame(...p), m = pose.matrix, [x, y, z, w] = pose.quaternion;
    assert.deepEqual(pose.position.map((v) => +v.toFixed(5)), [0.3, 1, 0.2]);
    const q = [1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w), 2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w), 2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)];
    [0, 1, 2, 4, 5, 6, 8, 9, 10].forEach((i, k) => close(q[k], m[i]));
    for (let c = 0; c < 4; c++) {
      const back = mul(pose.viewMatrix, [m[4 * c], m[4 * c + 1], m[4 * c + 2], m[4 * c + 3]]);
      back.forEach((v, r) => close(v, r === c ? 1 : 0));
    }
    assert.ok(m[5] > 0, "camera up points up");
  }
});

test("looking up: no floor hit", () => {
  assert.deepEqual(frame(0, -0.5, 0).hitTest(0.5, 0.5, 1), []);
});

test("hitTest lands on the floor and projects back to the tapped point", () => {
  for (const p of poses.slice(0, 2)) for (const aspect of [9 / 16, 1, 16 / 9]) for (const zeroToOne of [false, true]) {
    const f = frame(...p);
    for (const [u, v] of [[0.5, 0.5], [0.2, 0.8], [0.9, 0.6]]) {
      const [h] = f.hitTest(u, v, aspect);
      assert.ok(h, `hit at ${u},${v}`);
      assert.equal(h.type, "floor");
      const hit = h.position;
      close(hit[1], 0);
      close(h.distance, Math.hypot(...hit.map((x, i) => x - f.pose.position[i])));
      const c = mul(f.projectionMatrix(aspect, 0.01, 100, zeroToOne), mul(f.pose.viewMatrix, [...hit, 1]));
      close((c[0] / c[3] + 1) / 2, u);
      close((1 - c[1] / c[3]) / 2, v);
      assert.ok(c[2] / c[3] >= (zeroToOne ? 0 : -1) && c[2] / c[3] <= 1, "inside the depth range");
    }
  }
});

test("no pose before the first sensor frame; scale null until known", () => {
  const f = new Frame(new Float32Array(), 0, new Float32Array(), { width: 360, height: 640 }, null, null, {});
  assert.equal(f.pose, null);
  assert.equal(f.state, "initializing");
  assert.deepEqual(f.hitTest(0.5, 0.5, 1), []);
  assert.equal(frame(0, 0.5, 0).scale, null);
});
