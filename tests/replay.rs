//! Replays a phone recording (camera.html?record=1, server started with WEBAR_RECORD=1)
//! through the CPU front end and the same `track()` the browser uses. M01 V1.
//!   $env:WEBAR_REPLAY="recordings/<session>"; cargo test --release --test replay -- --ignored --nocapture
//! WEBAR_REPLAY_FOV overrides the recorded field of view (to compare FOV guesses offline).
use webar::geom::camera_from_orientation;
use webar::image::{Gray, pyramid};
use webar::orb::BORDER;
use webar::slam::{Intrinsics, Slam};
use webar::{FrontEnd, readback, track};

#[test]
#[ignore = "needs WEBAR_REPLAY=<recording dir>"]
fn replay() {
    let dir = std::path::PathBuf::from(std::env::var("WEBAR_REPLAY").expect("set WEBAR_REPLAY to a recordings/<session> dir"));
    let meta = std::fs::read_to_string(dir.join("meta.csv")).expect("meta.csv");
    let mut rows: Vec<Vec<f32>> = meta.lines().map(|l| l.split(',').map(|v| v.parse().unwrap()).collect()).collect();
    rows.sort_by(|a, b| a[0].total_cmp(&b[0])); // uploads can land out of order
    let fov: Option<f32> = std::env::var("WEBAR_REPLAY_FOV").ok().map(|v| v.parse().expect("WEBAR_REPLAY_FOV"));
    let mut cfg = FrontEnd::default();
    let mut slam: Option<Slam> = None;
    println!("frame state inliers map threshold kps   x     y     z");
    for r in &rows {
        let (n, w, h) = (r[0] as usize, r[1] as usize, r[2] as usize);
        let data = std::fs::read(dir.join(format!("{n:04}.luma"))).expect("frame");
        let levels = pyramid(Gray { w, h, data }, cfg.levels, cfg.scale);
        let rb = readback::cpu_levels(&levels, &cfg, BORDER);
        let s = slam.get_or_insert_with(|| Slam::new(Intrinsics::from_fov(w, h, fov.unwrap_or(r[6]))));
        let threshold = cfg.fast_threshold;
        let kps = track(&rb, &mut cfg, Some(s), &camera_from_orientation(r[3], r[4], r[5]), true);
        println!("{n:5} {:5} {:7} {:4} {threshold:9} {:4} {:5.2} {:5.2} {:5.2}", s.state as u8, s.inliers, s.map_len(), kps.len(), s.c[0], s.c[1], s.c[2]);
    }
}
