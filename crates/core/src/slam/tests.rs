use super::*;
use crate::geom::camera_from_orientation;

const K: Intrinsics = Intrinsics { f: 460.0, cx: 179.5, cy: 319.5 };

fn rand_desc(s: &mut u32) -> Descriptor {
    std::array::from_fn(|_| {
        *s ^= *s << 13;
        *s ^= *s >> 17;
        *s ^= *s << 5;
        (*s as u64) << 32 | (s.rotate_left(7) as u64)
    })
}

fn world(s: &mut u32) -> Vec<(V3, Descriptor)> {
    (0..60 * 40).map(|i| ([(i % 60) as f32 * 0.1 - 2.0, (i / 60) as f32 * 0.1 - 0.5, 0.0], rand_desc(s))).collect()
}

fn observe(slam: &Slam, r: &M3, c: V3, w: &[(V3, Descriptor)]) -> (Vec<Keypoint>, Vec<Descriptor>) {
    w.iter()
        .filter_map(|(x, d)| {
            let (u, v) = slam.k.project(r, c, *x)?;
            ((0.0..360.0).contains(&u) && (0.0..640.0).contains(&v)).then_some((Keypoint { x: u, y: v, level: 0, score: 100 }, *d))
        })
        .take(500)
        .unzip()
}

#[test]
fn step_recovers_yaw_and_position() {
    let mut s = 1;
    let w = world(&mut s);
    let sensor = camera_from_orientation(0.0, 45.0, 0.0, 0.0);
    let (c_true, yaw_true) = ([0.3, -0.2, 1.1], 3f32.to_radians());
    let r_true = mul(&rot_z(yaw_true), &sensor);
    let slam = Slam::new(K);
    let (kps, _) = observe(&slam, &r_true, c_true, &w);
    let pairs: Vec<Pair> = kps
        .iter()
        .map(|k| {
            let x = w.iter().find(|(x, _)| slam.k.project(&r_true, c_true, *x).is_some_and(|p| p == (k.x, k.y))).unwrap().0;
            (x, K.ray(k.x, k.y))
        })
        .collect();
    let (mut c, mut yaw) = ([0.0, 0.0, 1.0], 0.0);
    for _ in 0..5 {
        (c, yaw) = step(&pairs, &sensor, c, yaw).unwrap();
    }
    assert!(norm(sub(c, c_true)) < 1e-3, "c {c:?}");
    assert!((yaw - yaw_true).abs() < 0.1f32.to_radians(), "yaw {}", yaw.to_degrees());
}

#[test]
fn initializes_then_tracks_through_outliers() {
    let mut s = 7;
    let w = world(&mut s);
    let sensor = camera_from_orientation(0.0, 45.0, 0.0, 0.0);
    let mut slam = Slam::new(K);

    // Looking at the horizon: no floor rays, stays initializing.
    let level = camera_from_orientation(0.0, 90.0, 0.0, 0.0);
    slam.update(&[], &[], &level, f64::NAN, None);
    assert_eq!(slam.state, State::Initializing);

    let (kps, desc) = observe(&slam, &sensor, [0.0, 0.0, 1.0], &w);
    slam.update(&kps, &desc, &sensor, f64::NAN, None);
    assert_eq!(slam.state, State::Normal);
    assert!(slam.map_len() >= MIN_INIT_POINTS);

    // Move, and let the sensor heading be 3 deg off; 30% of keypoints jump 50 px.
    let (c_true, yaw_true) = ([0.15, 0.1, 1.0], 3f32.to_radians());
    let (mut kps, desc) = observe(&slam, &mul(&rot_z(yaw_true), &sensor), c_true, &w);
    for k in kps.iter_mut().step_by(3) {
        k.x += 20.0; // inside the match radius, so these reach RANSAC as outliers
    }
    slam.update(&kps, &desc, &sensor, f64::NAN, None);
    assert_eq!(slam.state, State::Normal, "inliers {}", slam.inliers);
    assert!(norm(sub(slam.c, c_true)) < 0.02, "c {:?}", slam.c);
    assert!((slam.yaw - yaw_true).abs() < 0.3f32.to_radians(), "yaw {}", slam.yaw.to_degrees());
}

#[test]
fn walks_off_the_initial_map_and_back() {
    let mut s = 3;
    let w = world(&mut s);
    let sensor = camera_from_orientation(0.0, 45.0, 0.0, 0.0);
    let mut slam = Slam::new(K);
    let start = [0.0, 0.0, 1.0];
    let (kps, desc) = observe(&slam, &sensor, start, &w);
    slam.update(&kps, &desc, &sensor, f64::NAN, None);
    let initial: Vec<V3> = slam.map.iter().map(|m| m.x).collect();
    let xs: Vec<f32> = (0..=64).chain((0..64).rev()).map(|i| i as f32 * 0.05).collect();
    for (frame, &x) in xs.iter().enumerate() {
        let c = [x, 0.0, 1.0];
        let (kps, desc) = observe(&slam, &sensor, c, &w);
        slam.update(&kps, &desc, &sensor, f64::NAN, None);
        assert_eq!(slam.state, State::Normal, "frame {frame} x {x} inliers {}", slam.inliers);
        if frame == 64 {
            let seen = initial.iter().filter(|p| slam.k.project(&sensor, c, **p).is_some_and(|(u, v)| (0.0..360.0).contains(&u) && (0.0..640.0).contains(&v))).count();
            assert_eq!(seen, 0, "initial map still in view at the far end");
        }
    }
    assert!(norm(sub(slam.c, start)) < 0.05, "drift {:?}", slam.c);
    assert!(slam.map_len() < MAX_MAP, "map {}", slam.map_len());
    assert!(slam.keyframes() >= 2, "keyframes {}", slam.keyframes());
}

/// Joint BA: 4 keyframes see floor points (fixed, they pin scale) and wall points. Keyframes 1-3 are
/// nudged ~1 cm and 0.3 deg, wall points ~1 cm (a few px, what tracking leaves); BA brings both back.
#[test]
fn local_ba_recovers_perturbed_poses_and_points() {
    let mut s = 9;
    let sensor = camera_from_orientation(0.0, 60.0, 0.0, 0.0);
    let mut slam = Slam::new(K);
    let truth: Vec<V3> = (0..120).map(|i| [(i % 12) as f32 * 0.25 - 1.4, 1.5 + (i / 12) as f32 * 0.2, if i % 3 == 0 { 0.6 + (i % 7) as f32 * 0.1 } else { 0.0 }]).collect();
    for (i, &x) in truth.iter().enumerate() {
        slam.map.push(MapPoint { x, desc: rand_desc(&mut s), misses: 0, seed: None, seen: 0, born: 0, id: i as u32, level: 0, dist: 1.0, views: Vec::new() });
    }
    let poses: Vec<V3> = (0..4).map(|k| [k as f32 * 0.15, 0.0, 1.0]).collect(); // >= KF_BASELINE apart, as real keyframes are
    for &c in &poses {
        let r = slam.rotation(&sensor, 0.0);
        let obs = truth.iter().enumerate().filter_map(|(i, &x)| slam.k.project(&r, c, x).filter(|p| (0.0..360.0).contains(&p.0) && (0.0..640.0).contains(&p.1)).map(|_| (i as u32, mulv(&transpose(&r), normalize(sub(x, c)))))).collect();
        slam.keyframes.push(Keyframe { c, yaw: 0.0, sensor, obs, free: Vec::new() });
    }
    for (k, kf) in slam.keyframes.iter_mut().enumerate().skip(1) {
        kf.c = add(kf.c, [0.008, -0.003 * k as f32, 0.004]);
        kf.yaw = 0.3f32.to_radians();
    }
    for mp in slam.map.iter_mut().filter(|m| m.x[2] > 0.0) {
        mp.x = add(mp.x, [0.008, -0.006, 0.004]); // ~3 px: BA only takes observations that still fit (4 px)
    }
    slam.local_ba();
    for (kf, c) in slam.keyframes.iter().zip(&poses) {
        assert!(norm(sub(kf.c, *c)) < 0.005 && kf.yaw.abs() < 0.1f32.to_radians(), "keyframe {:?} yaw {} vs {c:?}", kf.c, kf.yaw.to_degrees());
    }
    // Wall points seen by >= 3 keyframes (2 views 0.12 apart leave depth weak; unseen ones can't move).
    let errs: Vec<f32> = slam.map.iter().zip(&truth).filter(|(m, _)| m.x[2] > 0.0 && slam.keyframes.iter().filter(|k| k.obs.iter().any(|o| o.0 == m.id)).count() >= 3).map(|(m, x)| norm(sub(m.x, *x))).collect();
    let mean = errs.iter().sum::<f32>() / errs.len() as f32;
    assert!(errs.len() >= 15 && mean < 0.004, "{} points, mean error {mean} (nudged 0.0108)", errs.len());
}

/// Epipolar creation: two keyframes 0.3 apart see the same wall corners as free keypoints (plus one
/// corner only the second sees); the matched ones become map points at their true positions.
#[test]
fn epipolar_matches_become_wall_points() {
    let mut s = 21;
    let sensor = camera_from_orientation(0.0, 80.0, 0.0, 0.0);
    let mut slam = Slam::new(K);
    let wall: Vec<(V3, Descriptor)> = (0..30).map(|i| ([(i % 6) as f32 * 0.3 - 0.75, 2.5, 0.6 + (i / 6) as f32 * 0.2], rand_desc(&mut s))).collect();
    let r = slam.rotation(&sensor, 0.0);
    for (k, c) in [[0.0, 0.0, 1.0], [0.3, 0.0, 1.0]].into_iter().enumerate() {
        let n = if k == 0 { wall.len() - 1 } else { wall.len() };
        let free = wall[..n].iter().map(|&(x, d)| (mulv(&transpose(&r), normalize(sub(x, c))), d, 0)).collect();
        slam.keyframes.push(Keyframe { c, yaw: 0.0, sensor, obs: Vec::new(), free });
    }
    slam.epipolar_points();
    assert_eq!(slam.map_len(), wall.len() - 1, "the corner seen once is not a point");
    for mp in &slam.map {
        let (x, _) = wall.iter().find(|(_, d)| *d == mp.desc).unwrap();
        assert!(norm(sub(mp.x, *x)) < 1e-3, "point {:?} vs {x:?}", mp.x);
        assert!(mp.seed.is_none() && mp.misses == MAX_MISSES - 3, "confirmed, on probation");
    }
}

/// The descriptor angle of a floor point names the same world direction (east) from two cameras on
/// opposite sides of it, where gravity's image direction would differ by 180 deg.
#[test]
fn canonical_angle_is_a_fixed_floor_direction() {
    let target = [0.3, 1.2, 0.0];
    for (c, alpha) in [([0.0, 0.0, 1.0], 0.0), ([0.6, 2.4, 1.0], 180.0)] {
        let mut slam = Slam::new(K);
        slam.c = c;
        slam.orient(&camera_from_orientation(alpha, 45.0, 0.0, 0.0));
        let (u, v) = slam.k.project(&slam.r, c, target).unwrap();
        let a = slam.canonical_angle(u, v).expect("floor pixel");
        let floor = |u: f32, v: f32| {
            let d = mulv(&slam.r, slam.k.ray(u, v));
            add(c, scale(d, -c[2] / d[2]))
        };
        let step = sub(floor(u + a.cos(), v + a.sin()), floor(u, v));
        assert!(step[1].abs() < 0.05 * step[0].abs(), "camera {c:?}: floor step {step:?} is not along x");
    }
}

/// Long lost, then a scene the map does not know (same floor, new descriptors), seen from 0.5 to the side
/// at height `h`: re-seeds at the last x/y, keeping the old map as the stash. A view of both merges the
/// new map back into the old frame: the pose snaps to truth and merged floor points sit at z = 0.
fn reseed_then_merge(h: f32) {
    let (mut s, mut s2) = (11, 99);
    let a = world(&mut s);
    let b: Vec<_> = world(&mut s2).into_iter().zip(&a).map(|((_, d), (x, _))| (*x, d)).collect();
    let both: Vec<_> = a.iter().zip(&b).enumerate().map(|(i, (pa, pb))| if i % 2 == 0 { *pa } else { *pb }).collect();
    let sensor = camera_from_orientation(0.0, 45.0, 0.0, 0.0);
    let mut slam = Slam::new(K);
    let (kps, desc) = observe(&slam, &sensor, [0.0, 0.0, 1.0], &a);
    slam.update(&kps, &desc, &sensor, f64::NAN, None);
    for _ in 0..REINIT_AFTER_LOST {
        slam.update(&[], &[], &sensor, f64::NAN, None); // covered lens: cannot seed, stays Lost
    }
    assert_eq!((slam.state, slam.reseeds), (State::Lost, 0));
    let truth = [0.5, 0.0, h];
    let (kps, desc) = observe(&slam, &sensor, truth, &b);
    slam.update(&kps, &desc, &sensor, f64::NAN, None);
    assert_eq!((slam.state, slam.reseeds), (State::Normal, 1));
    assert_eq!(slam.c, [0.0, 0.0, CAMERA_HEIGHT], "new map seeded at the last x/y");
    let (kps, desc) = observe(&slam, &sensor, truth, &both);
    for _ in 0..STASH_EVERY {
        slam.update(&kps, &desc, &sensor, f64::NAN, None);
    }
    assert_eq!((slam.state, slam.merges), (State::Normal, 1));
    assert!(norm(sub(slam.c, truth)) < 0.02, "merged pose {:?}", slam.c);
    let floor: Vec<f32> = slam.map.iter().filter(|m| m.seed.is_some()).map(|m| m.x[2]).collect();
    assert!(floor.iter().all(|z| z.abs() < 0.02), "floor z after merge {:?}", floor.iter().fold(0f32, |m, z| m.max(z.abs())));
}

#[test]
fn reseeds_then_merges_back() {
    reseed_then_merge(CAMERA_HEIGHT);
}

#[test]
fn merge_corrects_the_seed_scale() {
    reseed_then_merge(1.2);
}

/// A third of the corners sit on a 0.4-high platform (a stool top, a box). They are seeded as
/// floor at first; walking must move them to their true height without losing the pose.
#[test]
fn raised_points_get_their_true_height() {
    let mut s = 5;
    let platform = |x: f32, y: f32| (-0.6..0.6).contains(&x) && (1.0..1.8).contains(&y);
    let w: Vec<(V3, Descriptor)> = world(&mut s)
        .into_iter()
        .map(|(x, d)| (if platform(x[0], x[1]) { [x[0], x[1], 0.4] } else { x }, d))
        .collect();
    let sensor = camera_from_orientation(0.0, 45.0, 0.0, 0.0);
    let mut slam = Slam::new(K);
    let xs: Vec<f32> = (0..=20).chain((0..20).rev()).map(|i| i as f32 * 0.03).collect();
    for &x in &xs {
        let (kps, desc) = observe(&slam, &sensor, [x, 0.0, 1.0], &w);
        slam.update(&kps, &desc, &sensor, f64::NAN, None);
        assert_ne!(slam.state, State::Lost, "x {x}");
    }
    assert!(norm(sub(slam.c, [0.0, 0.0, 1.0])) < 0.05, "drift {:?}", slam.c);
    // Confirmed points only: a floor hypothesis waits for 4 deg of parallax (late ones never get it here).
    let on_platform: Vec<f32> = slam
        .map
        .iter()
        .filter(|m| m.seed.is_none())
        .filter_map(|m| w.iter().find(|(_, d)| *d == m.desc).filter(|(x, _)| x[2] > 0.0).map(|_| m.x[2]))
        .collect();
    let fresh = slam.map.iter().filter(|m| m.seed.is_none() && w.iter().any(|(x, d)| *d == m.desc && x[2] > 0.0) && slam.frame - m.seen <= 5).count();
    assert!(fresh * 2 >= on_platform.len(), "{fresh}/{} confirmed platform points still matched", on_platform.len());
    let right = on_platform.iter().filter(|z| (**z - 0.4).abs() < 0.05).count();
    assert!(on_platform.len() >= 20 && right * 10 >= on_platform.len() * 8, "{right}/{} platform points at true height", on_platform.len());
}

#[test]
fn relocalizes_after_a_jump() {
    let mut s = 21;
    let w = world(&mut s);
    let sensor = camera_from_orientation(0.0, 45.0, 0.0, 0.0);
    let mut slam = Slam::new(K);
    let (kps, desc) = observe(&slam, &sensor, [0.0, 0.0, 1.0], &w);
    slam.update(&kps, &desc, &sensor, f64::NAN, None);
    for _ in 0..3 {
        slam.update(&[], &[], &sensor, f64::NAN, None);
    }
    assert_eq!(slam.state, State::Lost);
    let target = [0.5, 0.3, 1.0];
    let (kps, desc) = observe(&slam, &sensor, target, &w);
    slam.update(&kps, &desc, &sensor, f64::NAN, None);
    assert_eq!(slam.state, State::Normal, "inliers {}", slam.inliers);
    assert!(norm(sub(slam.c, target)) < 0.02, "c {:?}", slam.c);
}

/// Most keypoints have a twin far away (repeating texture): globally ambiguous, so only a few
/// match globally, but guided matching from that weak pose confirms it.
#[test]
fn relocalizes_from_a_weak_global_match() {
    let mut s = 21;
    let w = world(&mut s);
    let sensor = camera_from_orientation(0.0, 45.0, 0.0, 0.0);
    let mut slam = Slam::new(K);
    let (kps, desc) = observe(&slam, &sensor, [0.0, 0.0, 1.0], &w);
    slam.update(&kps, &desc, &sensor, f64::NAN, None);
    slam.update(&[], &[], &sensor, f64::NAN, None);
    let target = [0.5, 0.3, 1.0];
    let (mut kps, mut desc) = observe(&slam, &sensor, target, &w);
    for i in 24..kps.len() {
        kps.push(Keypoint { y: (kps[i].y + 320.0) % 640.0, ..kps[i] });
        desc.push(desc[i]);
    }
    slam.update(&kps, &desc, &sensor, f64::NAN, None);
    assert_eq!(slam.state, State::Normal, "inliers {}/{}", slam.inliers, slam.matches);
    assert!(norm(sub(slam.c, target)) < 0.02, "c {:?}", slam.c);
}

#[test]
fn never_relocalizes_to_another_height() {
    let mut s = 21;
    let w = world(&mut s);
    let sensor = camera_from_orientation(0.0, 45.0, 0.0, 0.0);
    let mut slam = Slam::new(K);
    let (kps, desc) = observe(&slam, &sensor, [0.0, 0.0, 1.0], &w);
    slam.update(&kps, &desc, &sensor, f64::NAN, None);
    slam.update(&[], &[], &sensor, f64::NAN, None);
    let (kps, desc) = observe(&slam, &sensor, [0.0, 0.3, 1.7], &w);
    slam.update(&kps, &desc, &sensor, f64::NAN, None);
    assert_eq!(slam.state, State::Lost, "c {:?} inliers {}", slam.c, slam.inliers);
}

#[test]
fn few_agreeing_matches_are_not_normal() {
    let mut s = 21;
    let w = world(&mut s);
    let sensor = camera_from_orientation(0.0, 45.0, 0.0, 0.0);
    let mut slam = Slam::new(K);
    let (mut kps, desc) = observe(&slam, &sensor, [0.0, 0.0, 1.0], &w);
    slam.update(&kps, &desc, &sensor, f64::NAN, None);
    // All but 30 keypoints moved 20 px in scattered directions: they still match, but agree on no pose.
    for (i, k) in kps.iter_mut().enumerate().skip(30) {
        let a = i as f32 * 2.4;
        (k.x, k.y) = (k.x + 20.0 * a.cos(), k.y + 20.0 * a.sin());
    }
    slam.update(&kps, &desc, &sensor, f64::NAN, None);
    assert!(slam.inliers >= NORMAL_INLIERS && slam.inliers * 8 < slam.matches, "{}/{}", slam.inliers, slam.matches);
    assert_ne!(slam.state, State::Normal);
}

#[test]
fn never_relocalizes_below_the_floor() {
    let mut s = 21;
    let w = world(&mut s);
    let down = camera_from_orientation(0.0, 45.0, 0.0, 0.0);
    let mut slam = Slam::new(K);
    let (kps, desc) = observe(&slam, &down, [0.0, 0.0, 1.0], &w);
    slam.update(&kps, &desc, &down, f64::NAN, None);
    slam.update(&[], &[], &down, f64::NAN, None);
    let up = camera_from_orientation(0.0, 135.0, 0.0, 0.0);
    let (kps, desc) = observe(&slam, &up, [0.0, 0.0, -1.0], &w);
    assert!(kps.len() > 100, "{}", kps.len());
    slam.update(&kps, &desc, &up, f64::NAN, None);
    assert_eq!(slam.state, State::Lost, "c {:?} inl {} m {}", slam.c, slam.inliers, slam.matches);
    assert!(slam.c[2] > 0.0);
}

/// Tiles repeat every 0.4: a one-tile shift explains every match. Relocalization must end
/// Lost or at the true pose, never shifted.
#[test]
fn repeating_tiles_never_snap_one_tile_over() {
    let mut s = 31;
    let tiles: Vec<Descriptor> = (0..16).map(|_| rand_desc(&mut s)).collect();
    let w: Vec<(V3, Descriptor)> = world(&mut s)
        .into_iter()
        .enumerate()
        .map(|(i, (x, _))| (x, tiles[(i % 60 % 4) + 4 * (i / 60 % 4)]))
        .collect();
    let sensor = camera_from_orientation(0.0, 45.0, 0.0, 0.0);
    let mut slam = Slam::new(K);
    let (kps, desc) = observe(&slam, &sensor, [0.0, 0.0, 1.0], &w);
    slam.update(&kps, &desc, &sensor, f64::NAN, None);
    for _ in 0..3 {
        slam.update(&[], &[], &sensor, f64::NAN, None);
    }
    let target = [0.4, 0.0, 1.0];
    let (kps, desc) = observe(&slam, &sensor, target, &w);
    slam.update(&kps, &desc, &sensor, f64::NAN, None);
    let at_truth = norm(sub(slam.c, target)) < 0.05;
    assert!(slam.state == State::Lost || at_truth, "state {:?} c {:?}", slam.state, slam.c);
}

#[test]
fn blurred_frames_keep_the_map() {
    let mut s = 13;
    let w = world(&mut s);
    let sensor = camera_from_orientation(0.0, 45.0, 0.0, 0.0);
    let mut slam = Slam::new(K);
    let (kps, desc) = observe(&slam, &sensor, [0.0, 0.0, 1.0], &w);
    slam.update(&kps, &desc, &sensor, f64::NAN, None);
    slam.update(&kps, &desc, &sensor, f64::NAN, None);
    let before = slam.map_len();
    // Blur: only the corners in a band of the image survive, enough to stay Normal for longer than MAX_MISSES.
    let (few, fd): (Vec<_>, Vec<_>) = kps.iter().zip(&desc).filter(|(k, _)| k.y > 420.0).map(|(k, d)| (*k, *d)).unzip();
    for _ in 0..2 * MAX_MISSES {
        slam.update(&few, &fd, &sensor, f64::NAN, None);
        assert_eq!(slam.state, State::Normal, "{} inliers", slam.inliers);
    }
    assert!(slam.map_len() >= before, "map {before} -> {}", slam.map_len());
}

#[test]
fn guided_matching_skips_ambiguous_neighbours() {
    let mut s = 11;
    let w = world(&mut s);
    let sensor = camera_from_orientation(0.0, 45.0, 0.0, 0.0);
    let mut slam = Slam::new(K);
    let (mut kps, mut desc) = observe(&slam, &sensor, [0.0, 0.0, 1.0], &w);
    slam.update(&kps, &desc, &sensor, f64::NAN, None);
    let matched = |slam: &Slam, kps: &[Keypoint], desc: &[Descriptor]| -> Vec<usize> {
        slam.match_guided(kps, desc).into_iter().filter_map(|(_, g)| if let Guided::Match(ki) = g { Some(ki) } else { None }).collect()
    };
    let unique = matched(&slam, &kps, &desc).len();
    assert!(unique > 100, "{unique}");
    // A twin of keypoint 0 a few pixels away (a repeating tile corner): that map point must not match.
    kps.push(Keypoint { x: kps[0].x + 6.0, ..kps[0] });
    desc.push(desc[0]);
    let m = matched(&slam, &kps, &desc);
    assert_eq!(m.len(), unique - 1);
    assert!(m.iter().all(|&ki| ki != 0 && ki != kps.len() - 1));
    // The same corner 1 px away (another pyramid level) is not a rival: keypoint 1 still matches.
    kps.push(Keypoint { x: kps[1].x + 1.0, ..kps[1] });
    desc.push(desc[1]);
    assert_eq!(matched(&slam, &kps, &desc).len(), unique - 1);
}
