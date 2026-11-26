//! Replays a phone recording (camera.html?record=1, server started with WEBAR_RECORD=1)
//! through the CPU front end and the same `track()` the browser uses. M01 V1.
//!   $env:WEBAR_REPLAY="recordings/<session>"; cargo test --release --test replay -- --ignored --nocapture
//! WEBAR_REPLAY_FOV overrides the recorded field of view (to compare FOV guesses offline).
//! WEBAR_REPLAY_END keeps only the first N frames (to cut a bad tail).
//! WEBAR_REPLAY_OCCLUDE=a..b blacks out forward frames a..b-1 (M01 V3: lost, then recovery on uncover).
//! WEBAR_REPLAY_MAX_ERR=m fails the test if the closure gap or the forward z spread (max - min, true height
//! is constant on a walk) exceeds m units (M01 V1 pass/fail).
//! The frames play forward then backward, and the pose at the first tracked frame is compared on both
//! passes. Closure only: errors symmetric in both directions (FOV, scale) cancel, and the way back
//! re-tracks the forward map. The forward z range (true height is constant) catches some of those.
use std::time::Instant;
use webar::geom::{camera_from_orientation, camera_in_device, mulv, transpose};
use webar::image::{Gray, pyramid};
use webar::orb::BORDER;
use webar::slam::{Intrinsics, Slam, State};
use webar::{FrontEnd, readback, track};

#[test]
#[ignore = "needs WEBAR_REPLAY=<recording dir>"]
fn replay() {
    let dir = std::path::PathBuf::from(std::env::var("WEBAR_REPLAY").expect("set WEBAR_REPLAY to a recordings/<session> dir"));
    let meta = std::fs::read_to_string(dir.join("meta.csv")).expect("meta.csv");
    // Columns: index, w, h, alpha, beta, gamma, fov[, capture time ms[, ax, ay, az]]. Recordings made before the
    // capture index was sent are in upload order, which can differ from capture order.
    let mut rows: Vec<Vec<f32>> = meta.lines().map(|l| l.split(',').map(|v| v.parse().unwrap()).collect()).collect();
    rows.sort_by(|a, b| a[0].total_cmp(&b[0]));
    let env = |k: &str| std::env::var(k).ok().map(|v| v.parse::<f32>().expect(k));
    rows.truncate(env("WEBAR_REPLAY_END").map_or(rows.len(), |n| n as usize));
    let fov = env("WEBAR_REPLAY_FOV");
    // Constant offset added to beta (degrees): offline sensor-pitch bias sweep.
    for r in rows.iter_mut() {
        r[4] += env("WEBAR_REPLAY_PITCH").unwrap_or(0.0);
    }
    // Sensor angles taken `shift` frames later (interpolated; negative = earlier): offline sensor-delay sweep.
    if let Some(shift) = env("WEBAR_REPLAY_SHIFT") {
        let src = rows.clone();
        let wrap = |d: f32| (d + 540.0).rem_euclid(360.0) - 180.0; // alpha crosses 0/360
        for (n, r) in rows.iter_mut().enumerate() {
            let p = (n as f32 + shift).clamp(0.0, (src.len() - 1) as f32);
            let (i, f) = (p.floor() as usize, p.fract());
            let (a, b) = (&src[i], &src[(i + 1).min(src.len() - 1)]);
            r[3] = a[3] + f * wrap(b[3] - a[3]);
            r[4] = a[4] + f * (b[4] - a[4]);
            r[5] = a[5] + f * (b[5] - a[5]);
        }
    }
    let occlude = std::env::var("WEBAR_REPLAY_OCCLUDE").ok().map(|v| {
        let (a, b) = v.split_once("..").expect("WEBAR_REPLAY_OCCLUDE=a..b");
        a.parse::<usize>().unwrap()..b.parse::<usize>().unwrap()
    });
    assert!(occlude.as_ref().is_none_or(|o| rows.iter().any(|r| r[0] as usize >= o.end)), "occlusion window past the last frame");
    let mut reseeded = false;
    let mut recovered: Option<(usize, [f32; 3])> = None;
    let mut cfg = FrontEnd::default();
    let mut slam: Option<Slam> = None;
    let (mut start, mut back, mut last, mut path, mut zr) = (None, None, None, 0.0f32, (f32::MAX, f32::MIN));
    let (mut states, mut ms_sum, mut ms_max) = ([0usize; 4], 0.0f64, 0.0f64);
    println!("frame dir state inl/match  map thr  kps  track_ms   x     y     z");
    let order: Vec<(&Vec<f32>, char)> = rows.iter().map(|r| (r, '>')).chain(rows.iter().rev().skip(1).map(|r| (r, '<'))).collect();
    for &(r, dir_mark) in &order {
        let (n, w, h) = (r[0] as usize, r[1] as usize, r[2] as usize);
        let mut data = std::fs::read(dir.join(format!("{n:04}.luma"))).expect("frame");
        if dir_mark == '>' && occlude.as_ref().is_some_and(|o| o.contains(&n)) {
            data.fill(0);
        }
        let levels = pyramid(Gray { w, h, data }, cfg.levels, cfg.scale);
        let rb = readback::cpu_levels(&levels, &cfg, BORDER);
        let s = slam.get_or_insert_with(|| Slam::new(Intrinsics::from_fov(w, h, fov.unwrap_or(r[6]))));
        let threshold = cfg.fast_threshold;
        let t0 = Instant::now();
        // Optional columns: capture time (ms), then gravity-free device acceleration (recordings since IMU capture).
        let t = r.get(7).map_or(f64::NAN, |&t| t as f64);
        // shortcut: recordings are portrait (screen angle 0); record the angle if landscape replays are needed.
        let accel = r.get(8..11).map(|a| mulv(&transpose(&camera_in_device(0.0)), [a[0], a[1], a[2]])).filter(|a| a.iter().all(|v| v.is_finite()));
        let kps = track(&rb, &mut cfg, Some(s), &camera_from_orientation(r[3], r[4], r[5], 0.0), t, accel, true);
        let ms = t0.elapsed().as_secs_f64() * 1e3;
        (ms_sum, ms_max) = (ms_sum + ms, ms_max.max(ms));
        states[s.state as usize] += 1;
        if let Some(o) = occlude.as_ref().filter(|o| dir_mark == '>' && n >= o.start && recovered.is_none()) {
            reseeded = s.reseeds > 0;
            if n >= o.end && (s.state == State::Normal || s.state == State::Limited) {
                recovered = Some((n - o.end, s.c));
            }
        }
        if s.state == State::Normal || s.state == State::Limited {
            let (sn, sc) = *start.get_or_insert((n, s.c));
            if dir_mark == '<' && n == sn {
                back = Some((sc, s.c));
            }
            if dir_mark == '>' {
                zr = (zr.0.min(s.c[2]), zr.1.max(s.c[2]));
                path += last.map_or(0.0, |l: [f32; 3]| (0..3).map(|i| (s.c[i] - l[i]).powi(2)).sum::<f32>().sqrt());
                last = Some(s.c);
            }
        }
        println!(
            "{n:5}   {dir_mark} {:5} {:4}/{:<4} {:4} {threshold:3} {:4} {ms:8.2} {:5.2} {:5.2} {:5.2}",
            s.state as u8, s.inliers, s.matches, s.map_len(), kps.len(), s.c[0], s.c[1], s.c[2]
        );
    }
    let (a, b) = back.expect("start frame not tracked on the way back");
    let gap = (0..3).map(|i| (a[i] - b[i]).powi(2)).sum::<f32>().sqrt();
    println!("\nframes {} | init {} normal {} limited {} lost {}", order.len(), states[0], states[1], states[2], states[3]);
    if let Some(s) = &slam { println!("re-seeds {} | stash merges {}", s.reseeds, s.merges); }
    println!("track (front-end CPU half + SLAM) mean {:.2} ms, max {ms_max:.2} ms", ms_sum / order.len() as f64);
    let o = slam.as_ref().map_or([0; 5], |s| s.old);
    println!("old points in view (>= 50 frames): {} | no keypoint {} | too far {} | ambiguous {} | outlier {} | inlier {}", o.iter().sum::<u32>(), o[0], o[1], o[2], o[3], o[4]);
    if let Some(o) = &occlude {
        match recovered {
            Some((k, c)) => println!(
                "occluded {}..{} | {} {k} frames after uncover at ({:.2}, {:.2}, {:.2})",
                o.start,
                o.end,
                if reseeded { "re-seeded (new map)" } else { "relocalized" },
                c[0],
                c[1],
                c[2]
            ),
            None => println!("occluded {}..{} | never tracked again", o.start, o.end),
        }
    }
    println!("forward path {path:.2} | closure gap {gap:.3} ({:.1}% of path) | forward z {:.2}..{:.2}", 100.0 * gap / path.max(1e-6), zr.0, zr.1);
    if let Some(m) = env("WEBAR_REPLAY_MAX_ERR") {
        assert!(gap <= m && zr.1 - zr.0 <= m, "pose error over {m}: closure {gap:.3}, z spread {:.3}", zr.1 - zr.0);
    }
}
