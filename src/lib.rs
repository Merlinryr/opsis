//! WebAR core. M01: GPU front end (phase 1) and ground-plane SLAM (phase 2).

pub mod fast;
pub mod geom;
pub mod image;
pub mod orb;
pub mod readback;
pub mod slam;
#[cfg(target_arch = "wasm32")]
pub mod gpu;
#[cfg(target_arch = "wasm32")]
pub mod web;
#[cfg(target_arch = "wasm32")]
pub mod webgl;
#[cfg(target_arch = "wasm32")]
pub mod webgpu;

/// Front-end tunables: the single source for the CPU reference and the GPU passes.
#[derive(Clone, Copy, Debug)]
pub struct FrontEnd {
    pub levels: usize,
    pub scale: f32,
    pub fast_threshold: u8,
    pub max_keypoints: usize,
    pub grid: (usize, usize),
}

impl Default for FrontEnd {
    fn default() -> Self {
        // shortcut: starting values, not tuned on real sequences; revisit in M01 phase 5.
        Self { levels: 4, scale: 1.44, fast_threshold: Self::DEFAULT_THRESHOLD, max_keypoints: 500, grid: (6, 8) }
    }
}

/// CPU half of one tracked frame, shared by the browser `Tracker` and the replay test:
/// keypoints + descriptors from the GPU (or CPU) readback, threshold adaptation, SLAM update.
/// `slam` is None without a sensor; `vision` false applies the sensor rotation only.
pub fn track(levels: &[readback::Level], cfg: &mut FrontEnd, slam: Option<&mut slam::Slam>, sensor: &geom::M3, vision: bool) -> Vec<fast::Keypoint> {
    // Phase 2/3 descriptors use the intensity centroid everywhere (open decision in the SLAM design note).
    let (kps, desc) = readback::features(levels, cfg, |_| None);
    cfg.fast_threshold = cfg.adapt(kps.len());
    if let Some(slam) = slam {
        if vision { slam.update(&kps, &desc, sensor) } else { slam.orient(sensor) }
    }
    kps
}

impl FrontEnd {
    const MIN_THRESHOLD: u8 = 7;
    const DEFAULT_THRESHOLD: u8 = 20;

    /// Next frame's FAST threshold from this frame's kept keypoint count: lower it fast on
    /// low-texture scenes (grey floors), drift back up to the default while the budget is full.
    /// Never above the default: on scenes whose corners share one contrast, a step up can drop
    /// every corner at once.
    pub fn adapt(&self, kept: usize) -> u8 {
        let t = self.fast_threshold;
        if kept < self.max_keypoints * 4 / 5 {
            t.saturating_sub(2).max(Self::MIN_THRESHOLD)
        } else if kept >= self.max_keypoints {
            (t + 1).min(Self::DEFAULT_THRESHOLD.max(t))
        } else {
            t
        }
    }
}

#[cfg(test)]
mod tests {
    use super::FrontEnd;

    #[test]
    fn threshold_adapts_within_bounds() {
        let mut cfg = FrontEnd::default();
        for _ in 0..20 {
            cfg.fast_threshold = cfg.adapt(10);
        }
        assert_eq!(cfg.fast_threshold, 7, "floors at the minimum on low texture");
        for _ in 0..50 {
            cfg.fast_threshold = cfg.adapt(cfg.max_keypoints);
        }
        assert_eq!(cfg.fast_threshold, 20, "returns to the default while saturated, no higher");
        assert_eq!(cfg.adapt(450), 20, "steady inside the band");
    }
}
