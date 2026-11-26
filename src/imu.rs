//! Loose IMU use (8th Wall style, not preintegration): camera-IMU time offset and metric scale,
//! from the visual trajectory and the gravity-free accelerometer, matched after time alignment.
//! Method: scale = |IMU acceleration|^2 / (IMU . visual acceleration), low-passed, at the
//! lag of best correlation (8th Wall `Alignment` + `MetricScaleEstimator`, method inferred).

use crate::geom::{V3, add, dot, scale, sub};

/// Moving average over `2r + 1` samples (edges shrink): the low-pass before differencing.
fn smooth(x: &[V3], r: usize) -> Vec<V3> {
    (0..x.len())
        .map(|i| {
            let s = &x[i.saturating_sub(r)..(i + r + 1).min(x.len())];
            scale(s.iter().fold([0.0; 3], |a, &b| add(a, b)), 1.0 / s.len() as f32)
        })
        .collect()
}

/// Central second difference of uniformly sampled positions (`dt` seconds apart).
fn accel(p: &[V3], dt: f32) -> Vec<V3> {
    (1..p.len().saturating_sub(1)).map(|i| scale(add(sub(p[i + 1], p[i]), sub(p[i - 1], p[i])), 1.0 / (dt * dt))).collect()
}

/// (lag in samples, metres per map unit) aligning `imu` (world-frame, gravity-free, m/s^2) to the visual
/// positions `p` (map units), both sampled every `dt` s. Positive lag: the IMU sample `lag` steps later
/// matches the frame. None without enough motion to tell, or with a non-positive fit.
// shortcut: assumes uniform frame spacing; resample on timestamps if frame drops are common.
pub fn scale_and_lag(p: &[V3], imu: &[V3], dt: f32, max_lag: usize) -> Option<(isize, f32)> {
    const SMOOTH: usize = 3; // ~5 Hz cut at 30 fps (8th Wall `maxFreqForAlignment` 5)
    if p.len() != imu.len() || p.len() < 4 * max_lag + 8 {
        return None;
    }
    let v = accel(&smooth(&smooth(p, SMOOTH), SMOOTH), dt);
    let a: Vec<V3> = smooth(&smooth(imu, SMOOTH), SMOOTH)[1..imu.len() - 1].to_vec();
    let pairs = |lag: isize| {
        let n = v.len() as isize;
        (max_lag as isize..n - max_lag as isize).map(move |i| (i as usize, (i + lag) as usize))
    };
    let corr = |lag: isize| {
        let (mut va, mut vv, mut aa) = (0.0, 0.0, 0.0);
        for (i, j) in pairs(lag) {
            (va, vv, aa) = (va + dot(v[i], a[j]), vv + dot(v[i], v[i]), aa + dot(a[j], a[j]));
        }
        (va / (vv * aa).sqrt().max(1e-12), va, vv, aa)
    };
    let lag = (-(max_lag as isize)..=max_lag as isize).max_by(|&x, &y| corr(x).0.total_cmp(&corr(y).0))?;
    let (r, va, _, aa) = corr(lag);
    let n = pairs(lag).count() as f32;
    // Needs real motion (mean |a| over ~0.3 m/s^2) and a clear correlation.
    // aa/va, not va/vv: tracking jitter, twice differenced, inflates vv and biases va/vv low (synthetic test,
    // ~4 mm jitter: va/vv 1.390, aa/va 1.401 for 1.4); the low-passed IMU noise biases aa/va far less.
    (aa / n > 0.1 && r > 0.5 && va > 0.0).then(|| (lag, aa / va))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Side-to-side sway in map units; the IMU sees it at 1.4 m per unit, 3 frames late, with noise.
    #[test]
    fn recovers_scale_and_lag() {
        let (dt, s_true, lag_true) = (1.0 / 30.0, 1.4f32, 3isize);
        let pos = |t: f32| [0.3 * (2.0 * t).sin(), 0.2 * (3.1 * t).sin(), 0.05 * (1.3 * t).sin()];
        let acc = |t: f32| [-1.2 * (2.0 * t).sin(), -1.922 * (3.1 * t).sin(), -0.0845 * (1.3 * t).sin()];
        let n = 240;
        let mut noise = 1u32;
        let mut rnd = || {
            noise ^= noise << 13;
            noise ^= noise >> 17;
            noise ^= noise << 5;
            (noise % 1000) as f32 / 1000.0 - 0.5
        };
        // ~4 mm of tracking jitter on the vision track (in map units, 1 unit ~ 1.4 m), noise on the IMU too.
        let p: Vec<V3> = (0..n).map(|i| pos(i as f32 * dt)).map(|x| [x[0] + 0.006 * rnd(), x[1] + 0.006 * rnd(), x[2] + 0.006 * rnd()]).collect();
        let imu: Vec<V3> = (0..n).map(|i| scale(acc((i as isize - lag_true) as f32 * dt), s_true)).map(|a| [a[0] + 0.2 * rnd(), a[1] + 0.2 * rnd(), a[2]]).collect();
        let (lag, s) = scale_and_lag(&p, &imu, dt, 10).expect("estimate");
        assert_eq!(lag, lag_true);
        assert!((s - s_true).abs() < 0.07, "scale {s}");
        let still = vec![[0.0; 3]; n];
        assert!(scale_and_lag(&still, &still, dt, 10).is_none(), "no motion, no scale");
    }
}
