//! Metric scale from the accelerometer, the simple way (like 8th Wall): compare how fast the camera accelerates in the map with what the sensor measures. The best time offset between the two is searched first.

use crate::geom::{V3, add, dot, scale, sub};

fn smooth(x: &[V3], r: usize) -> Vec<V3> {
    (0..x.len())
        .map(|i| {
            let s = &x[i.saturating_sub(r)..(i + r + 1).min(x.len())];
            scale(s.iter().fold([0.0; 3], |a, &b| add(a, b)), 1.0 / s.len() as f32)
        })
        .collect()
}

fn accel(p: &[V3], dt: f32) -> Vec<V3> {
    (1..p.len().saturating_sub(1)).map(|i| scale(add(sub(p[i + 1], p[i]), sub(p[i - 1], p[i])), 1.0 / (dt * dt))).collect()
}

/// Returns (offset in samples, metres per map unit) matching the sensor acceleration `imu` (world axes, no gravity, m/s²) to camera positions `p` (map units), both sampled every `dt` s. None without enough motion or a clear match.
// shortcut: assumes evenly spaced frames; resample on timestamps if frames drop often.
pub fn scale_and_lag(p: &[V3], imu: &[V3], dt: f32, max_lag: usize) -> Option<(isize, f32)> {
    // Smooth both signals with a 0.4 s moving average, twice. Shorter windows (5 Hz) gave scattered results on phone walks; 0.4 s agrees within ~15% (replays).
    let r = ((0.4 / dt).round() as usize).max(1);
    let edge = max_lag.max(2 * r); // Smoothing is biased at the edges.
    if p.len() != imu.len() || p.len() < 2 * edge + 4 * max_lag + 8 {
        return None;
    }
    let v = accel(&smooth(&smooth(p, r), r), dt);
    let a: Vec<V3> = smooth(&smooth(imu, r), r)[1..imu.len() - 1].to_vec();
    let pairs = |lag: isize| {
        let n = v.len() as isize;
        (edge as isize..n - edge as isize).map(move |i| (i as usize, (i + lag) as usize))
    };
    let corr = |lag: isize| {
        let (mut va, mut vv, mut aa) = (0.0, 0.0, 0.0);
        for (i, j) in pairs(lag) {
            (va, vv, aa) = (va + dot(v[i], a[j]), vv + dot(v[i], v[i]), aa + dot(a[j], a[j]));
        }
        (va / (vv * aa).sqrt().max(1e-12), va, vv, aa)
    };
    let lag = (-(max_lag as isize)..=max_lag as isize).max_by(|&x, &y| corr(x).0.total_cmp(&corr(y).0))?;
    let (c, va, vv, aa) = corr(lag);
    let n = pairs(lag).count() as f32;
    // Needs real motion (~0.3 m/s² on average) and a clear correlation. The velocity-based estimate is used: the acceleration-based one kept too much sensor noise.
    // A best offset at the edge of the search range is not a real peak.
    (aa / n > 0.1 && c > 0.8 && va > 0.0 && lag.unsigned_abs() < max_lag).then(|| (lag, va / vv))
}

#[cfg(test)]
mod tests {
    use super::*;

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
        // ~4 mm of tracking noise on the camera path, and noise on the sensor too.
        let p: Vec<V3> = (0..n).map(|i| pos(i as f32 * dt)).map(|x| [x[0] + 0.006 * rnd(), x[1] + 0.006 * rnd(), x[2] + 0.006 * rnd()]).collect();
        let imu: Vec<V3> = (0..n).map(|i| scale(acc((i as isize - lag_true) as f32 * dt), s_true)).map(|a| [a[0] + 0.2 * rnd(), a[1] + 0.2 * rnd(), a[2]]).collect();
        let (lag, s) = scale_and_lag(&p, &imu, dt, 10).expect("estimate");
        assert_eq!(lag, lag_true);
        assert!((s - s_true).abs() < 0.07, "scale {s}");
        let still = vec![[0.0; 3]; n];
        assert!(scale_and_lag(&still, &still, dt, 10).is_none(), "no motion, no scale");
    }
}
