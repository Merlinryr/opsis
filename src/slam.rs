//! M01 phase 2: ground-plane SLAM. Pitch and roll come from the device sensor; vision
//! solves camera position and a heading (yaw) correction from 2D-3D matches.
//! The map starts as keypoint rays hitting the floor one unit below the first camera
//! (scale is arbitrary until M01 adds metric scale).

use crate::fast::Keypoint;
use crate::geom::{self, M3, V3, add, cross, dot, mul, mulv, norm, normalize, rot_z, scale, sub, transpose};
use crate::orb::{Descriptor, hamming};

#[derive(Clone, Copy, Debug)]
pub struct Intrinsics {
    pub f: f32,
    pub cx: f32,
    pub cy: f32,
}

impl Intrinsics {
    /// Pinhole guess from the field of view across the long image side.
    // shortcut: browsers expose no intrinsics; per-device table or calibration later (M01 risk).
    pub fn from_fov(w: usize, h: usize, fov_deg: f32) -> Self {
        let f = 0.5 * w.max(h) as f32 / (0.5 * fov_deg.to_radians()).tan();
        Self { f, cx: 0.5 * w as f32 - 0.5, cy: 0.5 * h as f32 - 0.5 }
    }
    /// Pixel of world point `x` for camera (r, c), or None if behind it.
    fn project(&self, r: &M3, c: V3, x: V3) -> Option<(f32, f32)> {
        let p = mulv(&transpose(r), sub(x, c));
        (p[2] > 1e-3).then(|| (self.f * p[0] / p[2] + self.cx, self.f * p[1] / p[2] + self.cy))
    }
    fn ray(&self, x: f32, y: f32) -> V3 {
        normalize([(x - self.cx) / self.f, (y - self.cy) / self.f, 1.0])
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum State {
    /// Not enough floor in view yet ("tilt down").
    Initializing = 0,
    Normal = 1,
    Limited = 2,
    Lost = 3,
}

struct MapPoint {
    x: V3,
    desc: Descriptor,
    /// Consecutive frames matched but rejected: off-plane objects get culled.
    misses: u8,
}

// shortcut: starting values, not tuned on real sequences; revisit in M01 phase 5.
const CAMERA_HEIGHT: f32 = 1.0;
const MIN_BELOW_HORIZON: f32 = 0.26; // sin(15 deg): depth error explodes near the horizon
const MAX_DEPTH: f32 = 4.0 * CAMERA_HEIGHT;
const MIN_INIT_POINTS: usize = 40;
const MAX_MAP: usize = 1500;
const MAX_ADD: usize = 50;
const MATCH_DIST: u32 = 64;
const INLIER_PX: f32 = 4.0;
const NORMAL_INLIERS: usize = 25;
const LIMITED_INLIERS: usize = 10;
const REINIT_AFTER_LOST: u32 = 30; // frames (~1 s): then re-seed the map from the floor
const MAX_MISSES: u8 = 10;
const CELL: usize = 8; // px: one map point per cell when growing the map
const RANSAC_ITERS: usize = 64;

pub struct Slam {
    pub k: Intrinsics,
    map: Vec<MapPoint>,
    /// Camera center (world).
    pub c: V3,
    /// Heading correction applied on top of the sensor rotation.
    pub yaw: f32,
    /// Last camera-to-world rotation (sensor + yaw).
    pub r: M3,
    pub state: State,
    pub inliers: usize,
    lost_frames: u32,
    rng: u32,
}

/// One 2D-3D correspondence: world point and camera-frame ray.
type Pair = (V3, V3);

impl Slam {
    pub fn new(k: Intrinsics) -> Self {
        let r = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        Self { k, map: Vec::new(), c: [0.0, 0.0, CAMERA_HEIGHT], yaw: 0.0, r, state: State::Initializing, inliers: 0, lost_frames: 0, rng: 0x2545_F491 }
    }

    pub fn map_len(&self) -> usize {
        self.map.len()
    }

    fn rotation(&self, sensor: &M3, yaw: f32) -> M3 {
        mul(&rot_z(yaw), sensor)
    }

    /// Floor point hit by the ray through (x, y), if well below the horizon and not too far.
    fn floor_point(&self, r: &M3, c: V3, x: f32, y: f32) -> Option<V3> {
        let d = mulv(r, self.k.ray(x, y));
        let t = -c[2] / d[2];
        (-d[2] > MIN_BELOW_HORIZON && t > 0.0 && t <= MAX_DEPTH).then(|| add(c, scale(d, t)))
    }

    /// One frame: keypoints + descriptors and the sensor camera-to-world rotation.
    /// Apply the sensor rotation only (sensor-only mode, for tuning FOV and delay).
    pub fn orient(&mut self, sensor: &M3) {
        self.r = self.rotation(sensor, self.yaw);
    }

    pub fn update(&mut self, kps: &[Keypoint], desc: &[Descriptor], sensor: &M3) {
        self.orient(sensor);
        if self.state == State::Initializing {
            return self.initialize(kps, desc);
        }
        let radius = if self.state == State::Normal { 40.0 } else { 80.0 };
        let (r, c) = (self.r, self.c);
        // Guided matching: best descriptor among keypoints near each projected map point.
        let mut matches = Vec::new(); // (map index, keypoint index)
        for (mi, mp) in self.map.iter().enumerate() {
            let Some((u, v)) = self.k.project(&r, c, mp.x) else { continue };
            let best = kps
                .iter()
                .enumerate()
                .filter(|(_, k)| (k.x - u).powi(2) + (k.y - v).powi(2) <= radius * radius)
                .map(|(ki, _)| (hamming(&mp.desc, &desc[ki]), ki))
                .min();
            if let Some((_, ki)) = best.filter(|b| b.0 <= MATCH_DIST) {
                matches.push((mi, ki));
            }
        }
        let pairs: Vec<Pair> = matches.iter().map(|&(mi, ki)| (self.map[mi].x, self.k.ray(kps[ki].x, kps[ki].y))).collect();
        let (c, yaw, inl) = self.ransac(&pairs, sensor);
        self.inliers = inl.iter().filter(|&&b| b).count();
        self.state = match self.inliers {
            n if n >= NORMAL_INLIERS => State::Normal,
            n if n >= LIMITED_INLIERS => State::Limited,
            _ => State::Lost,
        };
        if self.state == State::Lost {
            // Keep the last pose. shortcut: re-seeding from the floor stands in for relocalization
            // (phase 3, R4); it keeps x/y, so anchors stay roughly in place but can jump.
            self.lost_frames += 1;
            if self.lost_frames >= REINIT_AFTER_LOST {
                self.state = State::Initializing;
            }
            return;
        }
        self.lost_frames = 0;
        (self.c, self.yaw) = (c, yaw);
        self.r = self.rotation(sensor, yaw);

        // Map upkeep, only on well-tracked frames (one blurry frame must not wipe the map):
        // in-view points that are not inliers accumulate misses and get culled; unmatched keypoints
        // become new floor points where no map point already projects.
        if self.state != State::Normal {
            return;
        }
        let (r, c) = (self.r, self.c);
        let (gw, gh) = (((2.0 * self.k.cx) as usize) / CELL + 1, ((2.0 * self.k.cy) as usize) / CELL + 1);
        let cell = |u: f32, v: f32| (u >= 0.0 && v >= 0.0).then(|| (u as usize / CELL, v as usize / CELL)).filter(|&(x, y)| x < gw && y < gh);
        let mut is_inlier = vec![false; self.map.len()];
        for (&(mi, _), &ok) in matches.iter().zip(&inl) {
            is_inlier[mi] = ok;
        }
        let mut occupied = vec![false; gw * gh];
        for (mp, &ok) in self.map.iter_mut().zip(&is_inlier) {
            let Some((x, y)) = self.k.project(&r, c, mp.x).and_then(|(u, v)| cell(u, v)) else { continue };
            occupied[y * gw + x] = true;
            mp.misses = if ok { 0 } else { mp.misses.saturating_add(1) };
        }
        self.map.retain(|mp| mp.misses < MAX_MISSES);
        let room = MAX_MAP.saturating_sub(self.map.len()).min(MAX_ADD);
        let mut added = 0;
        for (k, d) in kps.iter().zip(desc) {
            let Some((x, y)) = cell(k.x, k.y) else { continue };
            if added == room || occupied[y * gw + x] {
                continue;
            }
            if let Some(p) = self.floor_point(&r, c, k.x, k.y) {
                occupied[y * gw + x] = true;
                self.map.push(MapPoint { x: p, desc: *d, misses: 0 });
                added += 1;
            }
        }
    }

    fn initialize(&mut self, kps: &[Keypoint], desc: &[Descriptor]) {
        let (r, c) = (self.r, [self.c[0], self.c[1], CAMERA_HEIGHT]);
        let map: Vec<MapPoint> = kps
            .iter()
            .zip(desc)
            .filter_map(|(k, d)| Some(MapPoint { x: self.floor_point(&r, c, k.x, k.y)?, desc: *d, misses: 0 }))
            .collect();
        if map.len() >= MIN_INIT_POINTS {
            (self.map, self.c, self.state, self.inliers, self.lost_frames) = (map, c, State::Normal, 0, 0);
        }
    }

    fn next_rand(&mut self, n: usize) -> usize {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 17;
        self.rng ^= self.rng << 5;
        self.rng as usize % n
    }

    fn inliers(&self, pairs: &[Pair], sensor: &M3, c: V3, yaw: f32) -> Vec<bool> {
        let r = self.rotation(sensor, yaw);
        pairs
            .iter()
            .map(|&(x, ray)| {
                let p = (ray[0] / ray[2] * self.k.f + self.k.cx, ray[1] / ray[2] * self.k.f + self.k.cy);
                self.k.project(&r, c, x).is_some_and(|q| (q.0 - p.0).powi(2) + (q.1 - p.1).powi(2) <= INLIER_PX * INLIER_PX)
            })
            .collect()
    }

    /// 2-point RANSAC over (position, yaw), then refinement on all inliers.
    fn ransac(&mut self, pairs: &[Pair], sensor: &M3) -> (V3, f32, Vec<bool>) {
        let (c0, yaw0) = (self.c, self.yaw);
        let mut best = (c0, yaw0, self.inliers(pairs, sensor, c0, yaw0));
        let count = |v: &[bool]| v.iter().filter(|&&b| b).count();
        if pairs.len() < 2 {
            return best;
        }
        for _ in 0..RANSAC_ITERS {
            let (i, j) = (self.next_rand(pairs.len()), self.next_rand(pairs.len()));
            let r = self.rotation(sensor, yaw0);
            let (di, dj) = (mulv(&r, pairs[i].1), mulv(&r, pairs[j].1));
            if i == j || dot(di, dj) > 0.9994 {
                continue; // same or near-parallel rays (< 2 deg) constrain nothing
            }
            let Some((c, yaw)) = step(&[pairs[i], pairs[j]], sensor, c0, yaw0) else { continue };
            let inl = self.inliers(pairs, sensor, c, yaw);
            if count(&inl) > count(&best.2) {
                best = (c, yaw, inl);
            }
        }
        // Gauss-Newton on the inliers.
        for _ in 0..3 {
            let set: Vec<Pair> = pairs.iter().zip(&best.2).filter(|(_, b)| **b).map(|(p, _)| *p).collect();
            let Some((c, yaw)) = step(&set, sensor, best.0, best.1) else { break };
            let inl = self.inliers(pairs, sensor, c, yaw);
            if count(&inl) < count(&best.2) {
                break;
            }
            best = (c, yaw, inl);
        }
        best
    }
}

/// One linearized least-squares step on (camera center, yaw) from (c0, yaw0).
/// Residual per pair: world ray x (X - C) = 0, scaled by 1/|X - C| so every pair weighs as an angle.
pub fn step(pairs: &[Pair], sensor: &M3, c0: V3, yaw0: f32) -> Option<(V3, f32)> {
    if pairs.len() < 2 {
        return None;
    }
    let r = mul(&rot_z(yaw0), sensor);
    let (mut a, mut b) = ([[0.0f32; 4]; 4], [0.0f32; 4]);
    for &(x, ray) in pairs {
        let d = mulv(&r, ray);
        let e = sub(x, c0);
        let w = 1.0 / norm(e).max(1e-6);
        let res = scale(cross(d, e), w);
        let dyaw = scale(cross(cross([0.0, 0.0, 1.0], d), e), w);
        // d/dC of d x (X - C) is -[d]x; rows of the 3x4 Jacobian.
        let jc = [[0.0, d[2], -d[1]], [-d[2], 0.0, d[0]], [d[1], -d[0], 0.0]];
        for row in 0..3 {
            let j = [jc[row][0] * w, jc[row][1] * w, jc[row][2] * w, dyaw[row]];
            for p in 0..4 {
                for q in 0..4 {
                    a[p][q] += j[p] * j[q];
                }
                b[p] -= j[p] * res[row];
            }
        }
    }
    for (i, row) in a.iter_mut().enumerate() {
        row[i] += 1e-6; // keeps degenerate samples solvable
    }
    let dx = geom::solve(a, b)?;
    Some((add(c0, [dx[0], dx[1], dx[2]]), yaw0 + dx[3]))
}

#[cfg(test)]
mod tests {
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

    /// Floor grid with random descriptors.
    fn world(s: &mut u32) -> Vec<(V3, Descriptor)> {
        (0..60 * 40).map(|i| ([(i % 60) as f32 * 0.1 - 2.0, (i / 60) as f32 * 0.1 - 0.5, 0.0], rand_desc(s))).collect()
    }

    /// Keypoints a camera (r, c) sees of the world, inside a 360x640 image.
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
        let sensor = camera_from_orientation(0.0, 45.0, 0.0);
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
        let sensor = camera_from_orientation(0.0, 45.0, 0.0);
        let mut slam = Slam::new(K);

        // Looking at the horizon: no floor rays, stays initializing.
        let level = camera_from_orientation(0.0, 90.0, 0.0);
        slam.update(&[], &[], &level);
        assert_eq!(slam.state, State::Initializing);

        let (kps, desc) = observe(&slam, &sensor, [0.0, 0.0, 1.0], &w);
        slam.update(&kps, &desc, &sensor);
        assert_eq!(slam.state, State::Normal);
        assert!(slam.map_len() >= MIN_INIT_POINTS);

        // Move, and let the sensor heading be 3 deg off; 30% of keypoints jump 50 px.
        let (c_true, yaw_true) = ([0.15, 0.1, 1.0], 3f32.to_radians());
        let (mut kps, desc) = observe(&slam, &mul(&rot_z(yaw_true), &sensor), c_true, &w);
        for k in kps.iter_mut().step_by(3) {
            k.x += 20.0; // inside the match radius, so these reach RANSAC as outliers
        }
        slam.update(&kps, &desc, &sensor);
        assert_eq!(slam.state, State::Normal, "inliers {}", slam.inliers);
        assert!(norm(sub(slam.c, c_true)) < 0.02, "c {:?}", slam.c);
        assert!((slam.yaw - yaw_true).abs() < 0.3f32.to_radians(), "yaw {}", slam.yaw.to_degrees());
    }

    /// Walk sideways until none of the initial map is in view, then come back.
    #[test]
    fn walks_off_the_initial_map_and_back() {
        let mut s = 3;
        let w = world(&mut s);
        let sensor = camera_from_orientation(0.0, 45.0, 0.0);
        let mut slam = Slam::new(K);
        let start = [0.0, 0.0, 1.0];
        let (kps, desc) = observe(&slam, &sensor, start, &w);
        slam.update(&kps, &desc, &sensor);
        let initial: Vec<V3> = slam.map.iter().map(|m| m.x).collect();
        let xs: Vec<f32> = (0..=64).chain((0..64).rev()).map(|i| i as f32 * 0.05).collect();
        for (frame, &x) in xs.iter().enumerate() {
            let c = [x, 0.0, 1.0];
            let (kps, desc) = observe(&slam, &sensor, c, &w);
            slam.update(&kps, &desc, &sensor);
            assert_eq!(slam.state, State::Normal, "frame {frame} x {x} inliers {}", slam.inliers);
            if frame == 64 {
                let seen = initial.iter().filter(|p| slam.k.project(&sensor, c, **p).is_some_and(|(u, v)| (0.0..360.0).contains(&u) && (0.0..640.0).contains(&v))).count();
                assert_eq!(seen, 0, "initial map still in view at the far end");
            }
        }
        assert!(norm(sub(slam.c, start)) < 0.05, "drift {:?}", slam.c);
        assert!(slam.map_len() < MAX_MAP, "map {}", slam.map_len());
    }

    /// After ~1 s lost, the tracker re-seeds from the floor and keeps its last x/y.
    #[test]
    fn reinitializes_after_being_lost() {
        let mut s = 11;
        let w = world(&mut s);
        let sensor = camera_from_orientation(0.0, 45.0, 0.0);
        let mut slam = Slam::new(K);
        let (kps, desc) = observe(&slam, &sensor, [0.0, 0.0, 1.0], &w);
        slam.update(&kps, &desc, &sensor);
        assert_eq!(slam.state, State::Normal);
        for _ in 0..REINIT_AFTER_LOST {
            slam.update(&[], &[], &sensor); // e.g. covered lens
        }
        assert_eq!(slam.state, State::Initializing);
        // Floor visible again from elsewhere: re-seeds at the last known x/y.
        let (kps, desc) = observe(&slam, &sensor, [0.5, 0.0, 1.0], &w);
        slam.update(&kps, &desc, &sensor);
        assert_eq!(slam.state, State::Normal);
        assert_eq!(slam.c, [0.0, 0.0, CAMERA_HEIGHT]);
    }
}
