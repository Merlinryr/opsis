use opsis_core::FrontEnd;
use crate::gpu::Gpu;
use opsis_core::orb::BORDER;
use opsis_core::geom::{accel_in_camera, camera_from_orientation};
use opsis_core::slam::{Intrinsics, Slam};
use std::cell::{Cell, RefCell};
use crate::{webgl::Gl, webgpu::Wgpu};
use wasm_bindgen::prelude::*;
use web_sys::{HtmlCanvasElement, OffscreenCanvas};

enum Backend {
    Gl(Gl),
    Gpu(Wgpu),
}

#[wasm_bindgen]
pub struct Tracker {
    backend: Backend,
    cfg: Cell<FrontEnd>,
    fov: f32,
    slam: RefCell<Option<Slam>>,
    /// Last `detect` timings in ms: [front end incl. read-back, track].
    split_ms: Cell<[f64; 3]>,
}

#[wasm_bindgen]
impl Tracker {
    /// WebGPU when `webgpu` is true, else WebGL2 on `canvas`. `fov_deg`: field of view across the long image side.
    pub async fn create(canvas: HtmlCanvasElement, webgpu: bool, fov_deg: f32) -> Result<Tracker, JsValue> {
        let backend = if webgpu { Backend::Gpu(Wgpu::new().await?) } else { Backend::Gl(Gl::new(&canvas)?) };
        Ok(Tracker { backend, cfg: Cell::new(FrontEnd::default()), fov: fov_deg, slam: RefCell::new(None), split_ms: Cell::new([0.0; 3]) })
    }

    /// Last `detect` timings in ms: [GPU encode + submit, GPU wait + read-back, track]. On WebGPU, then the wait split: [GPU done, buffers mapped, unpacked].
    pub fn split_ms(&self) -> Vec<f64> {
        let mut v = self.split_ms.get().to_vec();
        if let Backend::Gpu(g) = &self.backend {
            v.extend(g.wait_ms.get());
        }
        v
    }

    /// Diagnostic: number of pyramid levels (to see how GPU time scales with it).
    pub fn set_levels(&self, n: usize) {
        let mut cfg = self.cfg.get();
        cfg.levels = n.clamp(1, 8);
        self.cfg.set(cfg);
    }

    /// Drops the map: the next frames start a new one under the current camera.
    pub fn reset(&self) {
        *self.slam.borrow_mut() = None;
    }

    pub fn threshold(&self) -> u8 {
        self.cfg.get().fast_threshold
    }

    pub fn backend(&self) -> String {
        match self.backend {
            Backend::Gl(_) => "webgl2".into(),
            Backend::Gpu(_) => "webgpu".into(),
        }
    }

    /// Tracks one camera frame. Returns [x, y, level, score] per kept keypoint, in full-size pixels. `alpha`, `beta`, `gamma`: device orientation in degrees, NaN without a sensor (then no tracking). `ax`, `ay`, `az`: acceleration (m/s², device axes), NaN if absent; `t`: capture time (ms); `screen`: screen rotation (degrees); `vision` false: sensor rotation only.
    #[allow(clippy::too_many_arguments)]
    pub async fn detect(&self, frame: OffscreenCanvas, alpha: f32, beta: f32, gamma: f32, screen: f32, t: f64, ax: f32, ay: f32, az: f32, vision: bool) -> Result<Vec<f32>, JsValue> {
        let (w, h) = (frame.width() as usize, frame.height() as usize);
        let mut cfg = self.cfg.get();
        let t0 = js_sys::Date::now();
        let (levels, encode) = match &self.backend {
            Backend::Gl(g) => g.front_end(&g.upload_canvas(&frame)?, &cfg, BORDER).await?,
            Backend::Gpu(g) => g.front_end(&g.upload_canvas(&frame)?, &cfg, BORDER).await?,
        };
        let t1 = js_sys::Date::now();
        let kps = {
            let mut slam = self.slam.borrow_mut(); // Borrowed after the GPU await, never across it.
            let sensor = camera_from_orientation(alpha, beta, gamma, screen);
            let k = Intrinsics::from_fov(w, h, self.fov);
            // shortcut: rotating the screen (new image size) starts a new map. Upgrade: rotate the map instead.
            if slam.as_ref().is_some_and(|s| s.k.cx != k.cx) {
                *slam = None;
            }
            let slam = (!alpha.is_nan()).then(|| slam.get_or_insert_with(|| Slam::new(k)));
            let accel = (!ax.is_nan()).then(|| accel_in_camera([ax, ay, az], screen));
            opsis_core::track(&levels, &mut cfg, slam, &sensor, t, accel, vision)
        };
        self.split_ms.set([encode, t1 - t0 - encode, js_sys::Date::now() - t1]);
        self.cfg.set(cfg);
        Ok(kps.iter().flat_map(|k| [k.x, k.y, k.level as f32, k.score as f32]).collect())
    }

    /// [state, inliers, map size, camera position (3), camera-to-world rotation row-major (9), f, cx, cy, metres per unit (NaN until known)]. Empty before the first sensor reading. State: 0 initializing, 1 normal, 2 limited, 3 lost.
    pub fn pose(&self) -> Vec<f32> {
        let slam = self.slam.borrow();
        let Some(s) = slam.as_ref() else { return Vec::new() };
        let mut out = vec![s.state as u8 as f32, s.inliers as f32, s.map_len() as f32];
        out.extend(s.c);
        out.extend(s.r.iter().flatten());
        out.extend([s.k.f, s.k.cx, s.k.cy, s.metres_per_unit.unwrap_or(f32::NAN)]);
        out
    }
}
