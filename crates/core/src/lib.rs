pub mod fast;
pub mod geom;
pub mod image;
pub mod imu;
pub mod orb;
pub mod readback;
pub mod slam;

/// Front-end settings, shared by the CPU reference and the GPU passes.
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
        // shortcut: starting values, not tuned on real recordings yet.
        Self { levels: 4, scale: 1.44, fast_threshold: Self::DEFAULT_THRESHOLD, max_keypoints: 500, grid: (6, 8) }
    }
}

/// CPU half of one tracked frame, shared by the browser `Tracker` and the replay test. `slam` is None without a sensor; `vision` false applies only the sensor rotation. `t`: capture time (ms, NaN if unknown); `accel`: acceleration without gravity in camera axes (m/s²), if available.
pub fn track(levels: &[readback::Level], cfg: &mut FrontEnd, slam: Option<&mut slam::Slam>, sensor: &geom::M3, t: f64, accel: Option<geom::V3>, vision: bool) -> Vec<fast::Keypoint> {
    let Some(slam) = slam else {
        let (kps, _) = readback::features(levels, cfg, |_| None);
        cfg.fast_threshold = cfg.adapt(kps.len());
        return kps;
    };
    // Floor keypoints are oriented by world east (stable as the camera moves); others by the image centroid.
    slam.orient(sensor);
    let (kps, desc) = readback::features(levels, cfg, |k| slam.canonical_angle(k.x, k.y));
    cfg.fast_threshold = cfg.adapt(kps.len());
    if vision {
        slam.update(&kps, &desc, sensor, t, accel);
    }
    kps
}

impl FrontEnd {
    const MIN_THRESHOLD: u8 = 7;
    const DEFAULT_THRESHOLD: u8 = 20;

    /// Next frame's FAST threshold from how many keypoints were kept: drop it fast on plain floors, creep back up when there are plenty. Never above the default: on scenes where all corners have the same contrast, one step up can lose them all.
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
