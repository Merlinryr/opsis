//! Browser entry points. Phase 1: GPU-vs-CPU parity harness on a fixed image.

use crate::FrontEnd;
use crate::fast;
use crate::gpu::{Gpu, Pass};
use crate::image::{self, Gray};
use crate::orb::{BORDER, hamming};
use crate::geom::camera_from_orientation;
use crate::readback::{self, Level};
use crate::slam::{Intrinsics, Slam};
use std::cell::{Cell, RefCell};
use crate::{webgl::Gl, webgpu::Wgpu};
use wasm_bindgen::prelude::*;
use web_sys::HtmlCanvasElement;

/// Deterministic RGBA test frame: colour value noise plus solid squares (sharp corners).
pub fn test_frame(w: usize, h: usize) -> Vec<u8> {
    let mut s: u32 = 7;
    let noise: Vec<u8> = (0..(w / 8 + 2) * (h / 8 + 2) * 3)
        .map(|_| {
            s = s.wrapping_mul(1_103_515_245).wrapping_add(12345);
            (s >> 24) as u8
        })
        .collect();
    let mut out = vec![255u8; w * h * 4];
    for y in 0..h {
        for x in 0..w {
            let cell = ((y / 8) * (w / 8 + 2) + x / 8) * 3;
            let sq = (x / 40 + y / 40) % 3 == 0 && x % 40 > 10 && y % 40 > 10;
            for c in 0..3 {
                out[(y * w + x) * 4 + c] = if sq { 30 + 60 * c as u8 } else { noise[cell + c] };
            }
        }
    }
    out
}

fn max_diff(gpu_rgba: &[u8], cpu: &Gray) -> u8 {
    cpu.data.iter().zip(gpu_rgba.chunks_exact(4)).map(|(c, g)| c.abs_diff(g[0])).max().unwrap_or(0)
}

fn set_mismatches(mut a: Vec<(usize, usize)>, mut b: Vec<(usize, usize)>) -> usize {
    a.sort();
    b.sort();
    a.len().abs_diff(b.len()) + a.iter().filter(|p| b.binary_search(p).is_err()).count()
}

#[wasm_bindgen]
pub async fn parity_webgl(canvas: HtmlCanvasElement) -> Result<String, JsValue> {
    parity(&Gl::new(&canvas)?, "webgl2").await
}

#[wasm_bindgen]
pub async fn parity_webgpu() -> Result<String, JsValue> {
    parity(&Wgpu::new().await?, "webgpu").await
}

/// Per-pass parity: each GPU pass gets the CPU output of the previous stage, so one
/// rounding difference cannot cascade. Then the full chain down to descriptors.
/// Returns a report; first line is PASS or FAIL.
async fn parity<G: Gpu>(gl: &G, name: &str) -> Result<String, JsValue> {
    let (w, h, cfg) = (320, 240, FrontEnd::default());
    let rgba = test_frame(w, h);
    let cpu_levels = image::pyramid(image::luma(&rgba, w, h), cfg.levels, cfg.scale);
    let mut lines = Vec::new();
    let mut ok = true;
    let mut check = |name: String, bad: u64, limit: u64| {
        ok &= bad <= limit;
        lines.push(format!("{} {name}: {bad} (limit {limit})", if bad <= limit { "ok  " } else { "FAIL" }));
    };

    let frame = gl.upload(&rgba, w, h)?;
    let luma = gl.read(&gl.run(Pass::Luma, &frame, None, w, h)?).await?;
    check("luma max diff".into(), max_diff(&luma, &cpu_levels[0]) as u64, 0);

    for (l, lvl) in cpu_levels.iter().enumerate() {
        let tex = gl.upload_gray(lvl)?;
        let cpu_blur = image::blur(lvl);
        check(format!("L{l} blur max diff"), max_diff(&gl.read(&gl.blur(&tex)?).await?, &cpu_blur) as u64, 0);

        if l + 1 < cfg.levels {
            let cpu_down = image::downscale(&cpu_blur, cfg.scale);
            let t = gl.upload_gray(&cpu_blur)?;
            let gpu = gl.read(&gl.run(Pass::Down(cfg.scale), &t, None, cpu_down.w, cpu_down.h)?).await?;
            check(format!("L{l} downscale max diff"), max_diff(&gpu, &cpu_down) as u64, 1);
        }

        let cpu_score = fast::score_map(lvl, cfg.fast_threshold, BORDER);
        let gpu = gl.read(&gl.run(Pass::Fast(cfg.fast_threshold, BORDER), &tex, None, lvl.w, lvl.h)?).await?;
        let gpu_score = Level { w: lvl.w, h: lvl.h, rgba: gpu }.score();
        let bad = cpu_score.iter().zip(&gpu_score).filter(|(a, b)| a != b).count();
        check(format!("L{l} FAST score mismatches"), bad as u64, 0);

        let cpu_nms = fast::nms(&cpu_score, lvl.w, lvl.h);
        let packed = Level::from_cpu(&cpu_score, &[], &cpu_blur);
        let st = gl.upload(&packed.rgba, lvl.w, lvl.h)?;
        let bt = gl.upload_gray(&cpu_blur)?;
        let gpu = Level { w: lvl.w, h: lvl.h, rgba: gl.read(&gl.run(Pass::Nms, &st, Some(&bt), lvl.w, lvl.h)?).await? };
        check(format!("L{l} NMS mismatches ({} kps)", cpu_nms.len()), set_mismatches(cpu_nms, gpu.survivors()) as u64, 0);
        let alpha = gpu.blurred().data.iter().zip(&cpu_blur.data).filter(|(a, b)| a != b).count();
        check(format!("L{l} NMS alpha (blurred) mismatches"), alpha as u64, 0);
    }

    // Full chain: GPU readback -> keypoints -> retain -> descriptors, vs the same chain on CPU.
    // A +-1 downscale may move a few upper-level corners, so allow 1% of the keypoints.
    let gravity = |_: &fast::Keypoint| Some(std::f32::consts::FRAC_PI_2);
    let (gk, gd) = readback::features(&gl.front_end(&rgba, w, h, &cfg, BORDER).await?, &cfg, gravity);
    let (ck, cd) = readback::features(&readback::cpu_levels(&cpu_levels, &cfg, BORDER), &cfg, gravity);
    let key = |k: &fast::Keypoint| (k.level, (k.x * 64.0).round() as i32, (k.y * 64.0).round() as i32);
    let mut cpu_map: Vec<_> = ck.iter().map(key).zip(&cd).collect();
    cpu_map.sort_by_key(|e| e.0);
    let (mut kp_bad, mut desc_bad) = (ck.len().abs_diff(gk.len()), 0);
    for (k, d) in gk.iter().zip(&gd) {
        match cpu_map.binary_search_by_key(&key(k), |e| e.0) {
            Ok(i) => desc_bad += (hamming(d, cpu_map[i].1) != 0) as usize,
            Err(_) => kp_bad += 1,
        }
    }
    let limit = (ck.len() / 100) as u64;
    check(format!("end-to-end keypoint mismatches ({} kps)", ck.len()), kp_bad as u64, limit);
    check("end-to-end descriptor mismatches".into(), desc_bad as u64, limit);

    Ok(format!("{} {name}\n{}", if ok { "PASS" } else { "FAIL" }, lines.join("\n")))
}

enum Backend {
    Gl(Gl),
    Gpu(Wgpu),
}

/// Live tracker for the camera page: one RGBA frame + sensor angles in, keypoints out;
/// the pose is read with `pose()`.
#[wasm_bindgen]
pub struct Tracker {
    backend: Backend,
    /// Its FAST threshold adapts per frame to the scene's texture.
    cfg: Cell<FrontEnd>,
    fov: f32,
    /// Created on the first frame, once the image size is known.
    slam: RefCell<Option<Slam>>,
}

#[wasm_bindgen]
impl Tracker {
    /// WebGPU when `webgpu` is true, else WebGL2 on `canvas`. `fov_deg`: field of view across the long side.
    pub async fn create(canvas: HtmlCanvasElement, webgpu: bool, fov_deg: f32) -> Result<Tracker, JsValue> {
        let backend = if webgpu { Backend::Gpu(Wgpu::new().await?) } else { Backend::Gl(Gl::new(&canvas)?) };
        Ok(Tracker { backend, cfg: Cell::new(FrontEnd::default()), fov: fov_deg, slam: RefCell::new(None) })
    }

    /// Current adaptive FAST threshold.
    pub fn threshold(&self) -> u8 {
        self.cfg.get().fast_threshold
    }

    pub fn backend(&self) -> String {
        match self.backend {
            Backend::Gl(_) => "webgl2".into(),
            Backend::Gpu(_) => "webgpu".into(),
        }
    }

    /// Flat [x, y, level, score] per kept keypoint, level-0 pixels. `alpha`, `beta`, `gamma`:
    /// W3C deviceorientation degrees for this frame, or NaN when there is no sensor (no tracking then).
    /// `vision` false: sensor rotation only, no map (to tune FOV and sensor delay).
    pub async fn detect(&self, rgba: Vec<u8>, w: usize, h: usize, alpha: f32, beta: f32, gamma: f32, vision: bool) -> Result<Vec<f32>, JsValue> {
        let mut cfg = self.cfg.get();
        let levels = match &self.backend {
            Backend::Gl(g) => g.front_end(&rgba, w, h, &cfg, BORDER).await?,
            Backend::Gpu(g) => g.front_end(&rgba, w, h, &cfg, BORDER).await?,
        };
        let kps = {
            let mut slam = self.slam.borrow_mut(); // after the GPU await: never held across it
            let sensor = camera_from_orientation(alpha, beta, gamma);
            let slam = (!alpha.is_nan()).then(|| slam.get_or_insert_with(|| Slam::new(Intrinsics::from_fov(w, h, self.fov))));
            crate::track(&levels, &mut cfg, slam, &sensor, vision)
        };
        self.cfg.set(cfg);
        Ok(kps.iter().flat_map(|k| [k.x, k.y, k.level as f32, k.score as f32]).collect())
    }

    /// [state, inliers, map size, C(3), R camera-to-world row-major (9), f, cx, cy];
    /// empty before the first sensor frame. State: 0 initializing, 1 normal, 2 limited, 3 lost.
    pub fn pose(&self) -> Vec<f32> {
        let slam = self.slam.borrow();
        let Some(s) = slam.as_ref() else { return Vec::new() };
        let mut out = vec![s.state as u8 as f32, s.inliers as f32, s.map_len() as f32];
        out.extend(s.c);
        out.extend(s.r.iter().flatten());
        out.extend([s.k.f, s.k.cx, s.k.cy]);
        out
    }
}
