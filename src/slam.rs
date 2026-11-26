//! M01: ground-plane SLAM with keyframes and local BA. Pitch and roll come from the device sensor; vision
//! solves camera position and a heading (yaw) correction from 2D-3D matches.
//! The map starts as keypoint rays hitting the floor one unit below the first camera
//! (scale is arbitrary until M01 adds metric scale).

use std::collections::{HashMap, HashSet};

use crate::fast::Keypoint;
use crate::geom::{self, M3, V3, add, cross, dot, mul, mulv, norm, normalize, rot_z, scale, sub, transpose};
use crate::orb::{BORDER, Descriptor, hamming};

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
    /// Consecutive well-tracked frames in view without being an inlier: stale points get culled.
    misses: u8,
    /// Floor hypothesis not yet confirmed: seeding camera centre and world ray. Seen again with
    /// parallax, the point is confirmed (inlier) or re-triangulated from the two rays (outlier).
    seed: Option<(V3, V3)>,
    /// Frame it was last an inlier (or created): the oldest go first when the map is full.
    seen: u32,
    born: u32,
    /// Stable id: keyframes refer to points by it (map indices shift on cull and eviction).
    id: u32,
    /// Pyramid level and distance of the `desc` view: predicts the level it appears at now.
    level: u8,
    dist: f32,
    /// Descriptors from keyframes (oldest first), tried after `desc` in guided matching.
    views: Vec<Descriptor>,
}

/// Camera pose and inlier observations (point id, camera ray) when the keyframe was taken.
struct Keyframe {
    c: V3,
    yaw: f32,
    sensor: M3,
    obs: Vec<(u32, V3)>,
    /// Keypoints with no map point (camera ray, descriptor, level): epipolar candidates.
    free: Vec<(V3, Descriptor, u8)>,
}

/// Guided matching outcome for one projected map point.
#[derive(Clone, Copy, PartialEq)]
enum Guided {
    NoKeypoint,
    TooFar,
    Ambiguous,
    Match(usize),
}

// shortcut: starting values, not tuned on real sequences; revisit in M01 phase 5.
const CAMERA_HEIGHT: f32 = 1.0;
const MIN_BELOW_HORIZON: f32 = 0.26; // sin(15 deg): depth error explodes near the horizon
const MAX_DEPTH: f32 = 4.0 * CAMERA_HEIGHT;
const MIN_INIT_POINTS: usize = 100; // 40 started on blurry frames with a map too thin to track (phone replay)
const MAX_MAP: usize = 1500;
const MAX_ADD: usize = 50;
const MATCH_DIST: u32 = 64;
const INLIER_PX: f32 = 4.0;
const NORMAL_INLIERS: usize = 25;
const LIMITED_INLIERS: usize = 15;
const REINIT_AFTER_LOST: u32 = 90; // frames (~3 s) of failed relocalization: then re-seed, the old map kept as the stash
const STASH_EVERY: u32 = 5; // tracked frames between relocalization tries against the stash (each is a global match)
const RATIO: f32 = 0.8; // Lowe ratio, guided and global matching
const MAX_MISSES: u8 = 10;
const MIN_PARALLAX_COS: f32 = 0.99756; // cos(4 deg)
const CELL: usize = 8; // px: one map point per cell when growing the map
const RANSAC_ITERS: usize = 64;
const MAX_RELOC_DZ: f32 = 0.5 * CAMERA_HEIGHT;
const OLD: u32 = 50; // frames: a point this old counts in `Slam::old`
const MAX_TURN_RATE: f32 = 1.22; // rad/s (70 deg/s, 8th Wall `maxGyroMagnitude`): no map upkeep above
const FRAME_S: f32 = 1.0 / 30.0; // frame interval assumed when no capture time is given (unit tests, old recordings)
const SCALE_WINDOW_S: f32 = 5.0; // seconds of tracked motion per metric-scale estimate (150 frames at 30 fps, 300 at 60)
const MAX_LAG_S: f32 = 0.1; // camera-IMU offset searched within +-0.1 s (8th Wall 0.5 s): the phone walks fit at 0-33 ms, and a 167 ms fit read 0.90 for ~1.4
const SCALE_RANGE: std::ops::RangeInclusive<f32> = 0.3..=3.0; // metres per unit: the unit is the camera height
// Keyframes, from 8th Wall `MapBuilder`: cooldown 5 frames, >= 20 new points or < 120 map points, BA over 4.
const KF_COOLDOWN: u32 = 5;
const KF_NEW_POINTS: usize = 20;
const KF_LOW_MAP: usize = 120; // map points
const KF_BASELINE: f32 = 0.1 * CAMERA_HEIGHT; // or moved this far since the last keyframe (PTAM's distance rule)
const BA_WINDOW: usize = 4;
const KF_PAIRS: usize = 3; // earlier keyframes paired with a new one (8th Wall `keyframePairProcessCount`)
const EPI_DIST: u32 = 32; // Hamming, epipolar matches (8th Wall `EpipolarMatcher`)
const MAX_VIEWS: usize = 3; // keyframe descriptors per point, for guided matching only (global: frame 191 jumps)

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
    /// Matches fed to RANSAC this frame (inliers / matches = agreement).
    pub matches: usize,
    /// Replay metric, cumulative: guided outcomes of in-view points older than `OLD` frames
    /// (no keypoint near, best over `MATCH_DIST`, ratio-rejected, RANSAC outlier, inlier).
    pub old: [u32; 5],
    lost_frames: u32,
    frame: u32,
    rng: u32,
    /// Smoothed position change per tracked frame: guided matching projects from `c + v`.
    v: V3,
    /// Last frame's sensor rotation: the turn between frames gates map upkeep.
    last_sensor: M3,
    keyframes: Vec<Keyframe>,
    last_kf: u32,
    next_id: u32,
    /// Capture time (ms) of the last frame, NaN if unknown.
    last_t: f64,
    /// Tracked frames since the last gap: (camera centre, world acceleration m/s^2, time s), for metric scale.
    motion: Vec<(V3, V3, f32)>,
    /// Metres per map unit and camera-to-IMU offset (ms), once the estimator has converged. Reported only:
    /// nothing in the solve uses them.
    pub metres_per_unit: Option<f32>,
    pub imu_lag_ms: Option<f32>,
    /// The map from before the last re-seed, whose frame placed content lives in. Tried every `STASH_EVERY`
    /// frames (and every lost frame); on a confirmed hit the current map is merged into it.
    stash: Vec<MapPoint>,
    /// Replay metrics, cumulative.
    pub reseeds: u32,
    pub merges: u32,
}

/// One 2D-3D correspondence: world point and camera-frame ray.
type Pair = (V3, V3);

impl Slam {
    pub fn new(k: Intrinsics) -> Self {
        let r = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        Self { k, map: Vec::new(), c: [0.0, 0.0, CAMERA_HEIGHT], yaw: 0.0, r, state: State::Initializing, inliers: 0, matches: 0, old: [0; 5], lost_frames: 0, frame: 0, rng: 0x2545_F491, v: [0.0; 3], last_sensor: r, keyframes: Vec::new(), last_kf: 0, next_id: 0, last_t: f64::NAN, motion: Vec::new(), metres_per_unit: None, imu_lag_ms: None, stash: Vec::new(), reseeds: 0, merges: 0 }
    }

    pub fn keyframes(&self) -> usize {
        self.keyframes.len()
    }

    /// Descriptor angle for a keypoint: the image direction of world +x (east, corrected by yaw) at that
    /// pixel. Unlike gravity, which on the floor points to the camera's foot and turns as the camera walks
    /// around a point, this direction depends only on the ray and the rotation. None above the floor band
    /// (centroid there). Call after `orient` for this frame.
    // shortcut: a point whose ray crosses the 15 deg band switches descriptor type and stops matching, and
    // the angle follows the yaw estimate (drifts while Lost); give such points both descriptors if replays show it.
    pub fn canonical_angle(&self, x: f32, y: f32) -> Option<f32> {
        let below = -mulv(&self.r, self.k.ray(x, y))[2] > MIN_BELOW_HORIZON;
        let ax = mulv(&transpose(&self.r), [1.0, 0.0, 0.0]);
        below.then(|| crate::orb::gravity_angle(x, y, ax, self.k.f, self.k.f, self.k.cx, self.k.cy))
    }

    pub fn map_len(&self) -> usize {
        self.map.len()
    }

    fn rotation(&self, sensor: &M3, yaw: f32) -> M3 {
        mul(&rot_z(yaw), sensor)
    }

    /// Map point on the floor hit by the ray through (x, y), if well below the horizon and not too far.
    fn floor_point(&mut self, r: &M3, c: V3, k: &Keypoint, desc: Descriptor) -> Option<MapPoint> {
        let d = mulv(r, self.k.ray(k.x, k.y));
        let t = -c[2] / d[2];
        if !(-d[2] > MIN_BELOW_HORIZON && t > 0.0 && t <= MAX_DEPTH) {
            return None;
        }
        self.next_id += 1;
        Some(MapPoint { x: add(c, scale(d, t)), desc, misses: 0, seed: Some((c, d)), seen: self.frame, born: self.frame, id: self.next_id, level: k.level, dist: t, views: Vec::new() })
    }

    /// One frame: keypoints + descriptors and the sensor camera-to-world rotation.
    /// Apply the sensor rotation only (sensor-only mode, for tuning FOV and delay).
    pub fn orient(&mut self, sensor: &M3) {
        self.r = self.rotation(sensor, self.yaw);
    }

    /// `t`: capture time in ms (NaN if unknown). `accel`: gravity-free acceleration in camera axes (m/s^2,
    /// `devicemotion.acceleration` through `geom::camera_in_device`), None without it.
    pub fn update(&mut self, kps: &[Keypoint], desc: &[Descriptor], sensor: &M3, t: f64, accel: Option<V3>) {
        let dt = ((t - self.last_t).abs() as f32 * 1e-3).clamp(0.005, 0.2); // abs: replays also run backward
        let dt = if dt.is_nan() { FRAME_S } else { dt };
        self.last_t = t;
        // Turn since last frame (rotation angle of last^T * now): blurred fast pans must not edit the map.
        let tr: f32 = (0..3).map(|i| dot([self.last_sensor[0][i], self.last_sensor[1][i], self.last_sensor[2][i]], [sensor[0][i], sensor[1][i], sensor[2][i]])).sum();
        let fast = ((tr - 1.0) * 0.5).clamp(-1.0, 1.0).acos() > MAX_TURN_RATE * dt;
        self.last_sensor = *sensor;
        self.orient(sensor);
        self.frame += 1;
        if self.state == State::Initializing {
            return self.initialize(kps, desc);
        }
        let lost = self.state == State::Lost;
        let (guided, matches, c, yaw, inl) = if lost { self.relocalize(kps, desc, sensor) } else { self.solve(kps, desc, sensor, false) };
        self.inliers = inl.iter().filter(|&&b| b).count();
        self.matches = matches.len();
        let mut is_inlier = vec![false; self.map.len()];
        for (&(mi, _), &ok) in matches.iter().zip(&inl) {
            is_inlier[mi] = ok;
        }
        for &(mi, g) in guided.iter().filter(|&&(mi, _)| self.frame - self.map[mi].born >= OLD) {
            let b = match g {
                Guided::NoKeypoint => 0,
                Guided::TooFar => 1,
                Guided::Ambiguous => 2,
                Guided::Match(_) => 3 + is_inlier[mi] as usize,
            };
            self.old[b] += 1;
        }
        // Limited also needs a quarter of the matches to agree: a few lucky inliers among many
        // matches is a wrong pose (seen in the first replay). Normal needs an eighth: good Normal frames
        // agree >= 16% on all replays, seeds and shifts; a relocalization 14 units off confirmed with 27/503.
        // The map is seeded on the floor (z = 0): a camera under it is a wrong pose, however many inliers
        // (room replay: relocalized 26 units away at z -2.56 with 43 guided inliers). Nor does the height
        // jump while lost (room replay, frame 191: relocalized 20 units away at z 4-7, confirmed 13%).
        self.state = match self.inliers {
            n if lost && self.confirmed(n, matches.len(), c) => State::Normal,
            _ if lost || c[2] <= 0.0 => State::Lost,
            n if n >= NORMAL_INLIERS && n * 8 >= matches.len() => State::Normal,
            // Leaving Lost takes Normal quality: Lost -> Limited relocalizations were 1-8 units off on the
            // covered-lens replay (2026-11-26) and then became the Lost "last pose".
            n if !lost && n >= LIMITED_INLIERS && n * 4 >= matches.len() => State::Limited,
            _ => State::Lost,
        };
        if self.state == State::Lost {
            // Keep the last pose and keep relocalizing, against the stash too. shortcut: a stash hit while lost
            // drops the current map (no pose in it to align by); upgrade by aligning through older frames.
            self.v = [0.0; 3];
            self.motion.clear();
            self.lost_frames += 1;
            if self.try_stash(kps, desc, sensor, false) {
                return;
            }
            // Long lost in a new place: re-seed from the floor, keeping the old map as the stash so returning
            // to it merges back (a plain re-seed moved the origin ~1 m, covered-lens replay 2026-11-26). Only
            // on frames that can seed: a covered lens just stays Lost. shortcut: one stash, always the oldest
            // map (placed content lives in its frame); a second re-seed drops the current map.
            if self.lost_frames >= REINIT_AFTER_LOST {
                let old = std::mem::take(&mut self.map);
                self.initialize(kps, desc);
                if self.state != State::Normal {
                    self.map = old;
                } else {
                    self.reseeds += 1;
                    if self.stash.is_empty() && old.len() >= MIN_INIT_POINTS {
                        self.stash = old;
                    }
                }
            }
            return;
        }
        self.lost_frames = 0;
        // Velocity only between tracked frames: a relocalization jump is not motion.
        self.v = if lost { [0.0; 3] } else { scale(add(self.v, sub(c, self.c)), 0.5) };
        (self.c, self.yaw) = (c, yaw);
        self.r = self.rotation(sensor, yaw);
        self.estimate_scale(accel, dt);
        if self.frame % STASH_EVERY == 0 && self.try_stash(kps, desc, sensor, true) {
            return;
        }

        // Map upkeep, only on well-tracked frames (one blurry frame must not wipe the map):
        // in-view points that are not inliers accumulate misses and get culled; unmatched keypoints
        // become new floor points where no map point already projects.
        if self.state != State::Normal || fast {
            return;
        }
        let mut new_obs = Vec::new(); // keyframe observations: inliers, then new points
        let (r, c) = (self.r, self.c);
        let (gw, gh) = (((2.0 * self.k.cx) as usize) / CELL + 1, ((2.0 * self.k.cy) as usize) / CELL + 1);
        let cell = |u: f32, v: f32| (u >= 0.0 && v >= 0.0).then(|| (u as usize / CELL, v as usize / CELL)).filter(|&(x, y)| x < gw && y < gh);
        for (&(mi, ki), &ok) in matches.iter().zip(&inl) {
            let mp = &mut self.map[mi];
            if ok {
                // Latest view: descriptors drift with viewpoint; doubled re-matched old points on the replays
                // (keeping up to 4 views gained little more and let a wrong relocalization through).
                (mp.desc, mp.level, mp.dist) = (desc[ki], kps[ki].level, norm(sub(mp.x, c)));
                new_obs.push((mp.id, self.k.ray(kps[ki].x, kps[ki].y)));
            }
            let Some((c1, d1)) = mp.seed else { continue };
            let d2 = mulv(&r, self.k.ray(kps[ki].x, kps[ki].y));
            if dot(d1, d2) > MIN_PARALLAX_COS {
                continue; // not enough parallax to tell floor from off-floor yet
            }
            mp.seed = None;
            if !ok {
                // Off the floor (stool, shelf): move it to where the two rays meet, on probation.
                match geom::triangulate(c1, d1, c, d2) {
                    Some(x) => (mp.x, mp.dist, mp.misses) = (x, mp.dist * norm(sub(x, c)) / norm(sub(mp.x, c)), MAX_MISSES - 3),
                    None => mp.misses = MAX_MISSES,
                }
            }
        }
        // A miss counts only where a corner was detected nearby: blurred frames detect few corners and
        // used to cull most of the map while still Normal (grey-floor replay: 1500 -> 806 points in 16 frames).
        let mut detected = vec![false; gw * gh];
        for (x, y) in kps.iter().filter_map(|k| cell(k.x, k.y)) {
            detected[y * gw + x] = true;
        }
        let near_corner = |x: usize, y: usize| (y.saturating_sub(1)..(y + 2).min(gh)).any(|j| (x.saturating_sub(1)..(x + 2).min(gw)).any(|i| detected[j * gw + i]));
        let mut occupied = vec![false; gw * gh];
        for (mp, &ok) in self.map.iter_mut().zip(&is_inlier) {
            let Some((x, y)) = self.k.project(&r, c, mp.x).and_then(|(u, v)| cell(u, v)) else { continue };
            occupied[y * gw + x] = true;
            if !ok && !near_corner(x, y) {
                continue;
            }
            mp.misses = if ok { 0 } else { mp.misses.saturating_add(1) };
            if ok {
                mp.seen = self.frame;
            }
        }
        self.map.retain(|mp| mp.misses < MAX_MISSES);
        if self.map.len() > MAX_MAP - MAX_ADD {
            // Full: evict the points seen longest ago so the map follows the camera (it used to freeze here).
            self.map.sort_by(|a, b| b.seen.cmp(&a.seen)); // stable: unstable order shifted with struct size
            self.map.truncate(MAX_MAP - MAX_ADD);
        }
        let room = MAX_MAP.saturating_sub(self.map.len()).min(MAX_ADD);
        let mut added = 0;
        for (k, d) in kps.iter().zip(desc) {
            let Some((x, y)) = cell(k.x, k.y) else { continue };
            if added == room || occupied[y * gw + x] {
                continue;
            }
            if let Some(mp) = self.floor_point(&r, c, k, *d) {
                occupied[y * gw + x] = true;
                new_obs.push((mp.id, self.k.ray(k.x, k.y)));
                self.map.push(mp);
                added += 1;
            }
        }
        if self.frame - self.last_kf >= KF_COOLDOWN && (added >= KF_NEW_POINTS || self.map.len() < KF_LOW_MAP || self.keyframes.last().is_none_or(|kf| norm(sub(kf.c, c)) > KF_BASELINE)) {
            // Keypoints with no map point near them: candidates for epipolar matching with the next keyframes.
            let free = kps.iter().zip(desc).filter(|(k, _)| cell(k.x, k.y).is_some_and(|(x, y)| !occupied[y * gw + x])).map(|(k, d)| (self.k.ray(k.x, k.y), *d, k.level)).collect();
            self.add_keyframe(sensor, new_obs, free);
        }
    }

    /// Keyframe from this frame's inliers and new points; its descriptors join the points' views. New
    /// points come from epipolar matches with the previous keyframes, then local BA runs over the window.
    fn add_keyframe(&mut self, sensor: &M3, obs: Vec<(u32, V3)>, free: Vec<(V3, Descriptor, u8)>) {
        self.last_kf = self.frame;
        // Redundant (8th Wall: >= 70% shared points) and close to the last one: keep the old one.
        if let Some(last) = self.keyframes.last() {
            let ids: HashSet<u32> = last.obs.iter().map(|o| o.0).collect();
            let shared = obs.iter().filter(|o| ids.contains(&o.0)).count();
            if shared * 10 >= obs.len() * 7 && norm(sub(last.c, self.c)) < 0.5 * KF_BASELINE {
                return;
            }
        }
        let ids: HashSet<u32> = obs.iter().map(|o| o.0).collect();
        for mp in self.map.iter_mut().filter(|mp| ids.contains(&mp.id)) {
            if mp.views.len() == MAX_VIEWS {
                mp.views.remove(0);
            }
            mp.views.push(mp.desc);
        }
        // Only the window is ever read (BA, epipolar pairing): older keyframes are dropped, not culled.
        if self.keyframes.len() == BA_WINDOW {
            self.keyframes.remove(0);
        }
        self.keyframes.push(Keyframe { c: self.c, yaw: self.yaw, sensor: *sensor, obs, free });
        self.epipolar_points();
        self.local_ba();
        if let Some(kf) = self.keyframes.last() {
            (self.c, self.yaw) = (kf.c, kf.yaw);
            self.r = self.rotation(sensor, kf.yaw);
        }
    }

    /// Off-floor points (walls, furniture) that floor seeding never makes: the newest keyframe's free
    /// keypoints matched against earlier keyframes' (Hamming <= `EPI_DIST`, ratio test), kept if the two
    /// rays meet (the epipolar check: `triangulate` rejects rays that miss by > 2% of depth), with enough
    /// parallax, above the floor and within reach. They start on probation, like re-triangulated points.
    fn epipolar_points(&mut self) {
        let Some((new, old)) = self.keyframes.split_last() else { return };
        let r2 = self.rotation(&new.sensor, new.yaw);
        let mut points = Vec::new();
        let mut used = vec![false; new.free.len()];
        for kf in old.iter().rev().take(KF_PAIRS).filter(|kf| norm(sub(kf.c, new.c)) >= 0.5 * KF_BASELINE) {
            let r1 = self.rotation(&kf.sensor, kf.yaw);
            for (i, &(ray2, d2, level)) in new.free.iter().enumerate() {
                if used[i] {
                    continue;
                }
                let (mut best, mut second, mut bj) = (u32::MAX, u32::MAX, 0);
                for (j, (_, d1, _)) in kf.free.iter().enumerate() {
                    let h = hamming(d1, &d2);
                    if h < best {
                        (second, best, bj) = (best, h, j);
                    } else if h < second {
                        second = h;
                    }
                }
                if best > EPI_DIST || (best as f32) >= RATIO * second as f32 {
                    continue;
                }
                let (w1, w2) = (mulv(&r1, kf.free[bj].0), mulv(&r2, ray2));
                if dot(w1, w2) > MIN_PARALLAX_COS {
                    continue;
                }
                let Some(x) = geom::triangulate(kf.c, w1, new.c, w2) else { continue };
                let dist = norm(sub(x, new.c));
                if x[2] > 0.05 * CAMERA_HEIGHT && dist <= 2.0 * MAX_DEPTH {
                    used[i] = true;
                    points.push((x, d2, level, dist));
                }
            }
        }
        let room = MAX_MAP.saturating_sub(self.map.len());
        for (x, desc, level, dist) in points.into_iter().take(room) {
            self.next_id += 1;
            self.map.push(MapPoint { x, desc, misses: MAX_MISSES - 3, seed: None, seen: self.frame, born: self.frame, id: self.next_id, level, dist, views: Vec::new() });
        }
    }

    /// Joint local bundle adjustment over the keyframe window: Gauss-Newton with a Cauchy loss, points
    /// eliminated by Schur complement. Unknowns: (centre, yaw) of every keyframe but the oldest, and every
    /// off-floor point seen by >= 2 of them. The oldest keyframe and floor points (z = 0) stay fixed: they
    /// pin gauge and scale. Residual per observation as in `step`: world ray x (X - C), over |X - C|.
    /// A moved point that no longer reprojects within `INLIER_PX` in every view goes back.
    fn local_ba(&mut self) {
        const N: usize = 4 * (BA_WINDOW - 1);
        let index: HashMap<u32, usize> = self.map.iter().enumerate().map(|(i, mp)| (mp.id, i)).collect();
        // Observations (keyframe, map index, camera ray) that fit their keyframe now: outliers stay out.
        let mut obs: Vec<(usize, usize, V3)> = Vec::new();
        for (ki, kf) in self.keyframes.iter().enumerate() {
            for &(id, ray) in &kf.obs {
                if let Some(&mi) = index.get(&id).filter(|&&mi| self.inliers(&[(self.map[mi].x, ray)], &kf.sensor, kf.c, kf.yaw)[0]) {
                    obs.push((ki, mi, ray));
                }
            }
        }
        let mut views: HashMap<usize, usize> = HashMap::new();
        for &(_, mi, _) in &obs {
            *views.entry(mi).or_default() += 1;
        }
        // Free points also need parallax between their window rays: near-parallel rays let BA slide a point
        // along them, which the fit-every-view check cannot see.
        let world_ray = |&(ki, _, ray): &(usize, usize, V3)| mulv(&self.rotation(&self.keyframes[ki].sensor, self.keyframes[ki].yaw), ray);
        let parallax = |mi: usize| {
            let rays: Vec<V3> = obs.iter().filter(|o| o.1 == mi).map(world_ray).collect();
            rays.iter().any(|a| rays.iter().any(|b| dot(*a, *b) < MIN_PARALLAX_COS))
        };
        let mut pts: Vec<usize> = views.iter().filter(|&(&mi, &n)| n >= 2 && self.map[mi].seed.is_none() && self.map[mi].x[2] > 0.05 * CAMERA_HEIGHT && parallax(mi)).map(|(&mi, _)| mi).collect();
        pts.sort_unstable();
        if self.keyframes.len() < 2 {
            return;
        }
        let slot: HashMap<usize, usize> = pts.iter().enumerate().map(|(i, &mi)| (mi, i)).collect();
        let before: Vec<V3> = pts.iter().map(|&mi| self.map[mi].x).collect();
        let cauchy = (INLIER_PX / self.k.f).powi(2); // squared angle at which the loss starts to flatten
        for _ in 0..5 {
            let (mut s, mut g) = ([[0.0f32; N]; N], [0.0f32; N]);
            let mut hpp = vec![[[0.0f32; 3]; 3]; pts.len()];
            let mut gp = vec![[0.0f32; 3]; pts.len()];
            let mut hcp = vec![[[0.0f32; 3]; N]; pts.len()];
            for &(ki, mi, ray) in &obs {
                let kf = &self.keyframes[ki];
                let d = mulv(&self.rotation(&kf.sensor, kf.yaw), ray);
                let e = sub(self.map[mi].x, kf.c);
                let w = 1.0 / norm(e).max(1e-6);
                let res = scale(cross(d, e), w);
                let rw = 1.0 / (1.0 + dot(res, res) / cauchy); // Cauchy weight (IRLS)
                let jyaw = scale(cross(cross([0.0, 0.0, 1.0], d), e), w);
                let jc = [[0.0, d[2], -d[1]], [-d[2], 0.0, d[0]], [d[1], -d[0], 0.0]]; // d/dC; d/dX = -d/dC
                let o = ki.checked_sub(1).map(|k| 4 * k); // keyframe 0 is fixed
                let pk = slot.get(&mi).copied();
                for row in 0..3 {
                    let jcam = [jc[row][0] * w, jc[row][1] * w, jc[row][2] * w, jyaw[row]];
                    if let Some(o) = o {
                        for p in 0..4 {
                            g[o + p] -= rw * jcam[p] * res[row];
                            for q in 0..4 {
                                s[o + p][o + q] += rw * jcam[p] * jcam[q];
                            }
                        }
                    }
                    let Some(i) = pk else { continue };
                    for p in 0..3 {
                        gp[i][p] += rw * jcam[p] * res[row]; // jpt = -jcam
                        for q in 0..3 {
                            hpp[i][p][q] += rw * jcam[p] * jcam[q];
                        }
                        if let Some(o) = o {
                            for q in 0..4 {
                                hcp[i][o + q][p] -= rw * jcam[q] * jcam[p];
                            }
                        }
                    }
                }
            }
            // Schur complement: S = Hcc - W V^-1 W^T, g = gc - W V^-1 gp; solve cameras, back-substitute points.
            let mut inv = vec![[[0.0f32; 3]; 3]; pts.len()];
            for i in 0..pts.len() {
                for j in 0..3 {
                    hpp[i][j][j] += 1e-6 + 1e-3 * hpp[i][j][j]; // LM damping
                }
                for col in 0..3 {
                    if let Some(v) = geom::solve(hpp[i], std::array::from_fn(|r| (r == col) as u8 as f32)) {
                        (0..3).for_each(|r| inv[i][r][col] = v[r]);
                    }
                }
                let wv: [[f32; 3]; N] = std::array::from_fn(|a| std::array::from_fn(|c| (0..3).map(|b| hcp[i][a][b] * inv[i][b][c]).sum()));
                for a in 0..N {
                    g[a] -= (0..3).map(|c| wv[a][c] * gp[i][c]).sum::<f32>();
                    for b in 0..N {
                        s[a][b] -= (0..3).map(|c| wv[a][c] * hcp[i][b][c]).sum::<f32>();
                    }
                }
            }
            for j in 0..N {
                s[j][j] += 1e-6 + 1e-3 * s[j][j]; // also keeps absent keyframes' rows solvable
            }
            let Some(dc) = geom::solve(s, g) else { break };
            for (k, kf) in self.keyframes.iter_mut().enumerate().skip(1) {
                let o = 4 * (k - 1);
                let c = add(kf.c, [dc[o], dc[o + 1], dc[o + 2]]);
                // A step that moves a keyframe far is a bad linearization, not a correction.
                if norm(sub(c, kf.c)) < 0.1 * CAMERA_HEIGHT && c[2] > 0.0 {
                    (kf.c, kf.yaw) = (c, kf.yaw + dc[o + 3]);
                }
            }
            for (i, &mi) in pts.iter().enumerate() {
                let rhs: [f32; 3] = std::array::from_fn(|p| gp[i][p] - (0..N).map(|a| hcp[i][a][p] * dc[a]).sum::<f32>());
                let dx: V3 = std::array::from_fn(|r| (0..3).map(|c| inv[i][r][c] * rhs[c]).sum());
                self.map[mi].x = add(self.map[mi].x, dx);
            }
        }
        // Moved points must fit every view and stay off the floor; `dist` follows the move (level prediction).
        let c = self.c;
        for (i, &mi) in pts.iter().enumerate() {
            let x = self.map[mi].x;
            let fits = x[2] > 0.05 * CAMERA_HEIGHT && obs.iter().filter(|o| o.1 == mi).all(|&(ki, _, ray)| self.inliers(&[(x, ray)], &self.keyframes[ki].sensor, self.keyframes[ki].c, self.keyframes[ki].yaw)[0]);
            let mp = &mut self.map[mi];
            if fits {
                mp.dist *= norm(sub(x, c)) / norm(sub(before[i], c));
            } else {
                mp.x = before[i];
            }
        }
    }

    /// Metric scale and IMU offset from the last `SCALE_WINDOW_S` of tracked frames (device acceleration rotated
    /// to world by the camera rotation).
    fn estimate_scale(&mut self, accel: Option<V3>, dt: f32) {
        let Some(a) = accel else { return self.motion.clear() };
        let a = mulv(&self.r, a);
        let t = self.motion.last().map_or(0.0, |m| m.2) + dt;
        self.motion.push((self.c, a, t));
        while t - self.motion[0].2 > SCALE_WINDOW_S {
            self.motion.remove(0);
        }
        let n = self.motion.len();
        if t - self.motion[0].2 >= SCALE_WINDOW_S - 0.1 && self.frame % 30 == 0 {
            // Mean interval over the window: scale goes with dt^2, so one jittery frame must not set it.
            let dt = (t - self.motion[0].2) / (n - 1) as f32;
            let (p, imu): (Vec<V3>, Vec<V3>) = self.motion.iter().map(|m| (m.0, m.1)).unzip();
            if let Some((lag, s)) = crate::imu::scale_and_lag(&p, &imu, dt, ((MAX_LAG_S / dt).round() as usize).max(1)).filter(|e| SCALE_RANGE.contains(&e.1)) {
                (self.metres_per_unit, self.imu_lag_ms) = (Some(s), Some(lag as f32 * dt * 1e3));
            }
        }
    }

    fn initialize(&mut self, kps: &[Keypoint], desc: &[Descriptor]) {
        let (r, c) = (self.r, [self.c[0], self.c[1], CAMERA_HEIGHT]);
        let map: Vec<MapPoint> = kps
            .iter()
            .zip(desc)
            .filter_map(|(k, d)| self.floor_point(&r, c, k, *d))
            .collect();
        if map.len() >= MIN_INIT_POINTS {
            (self.map, self.c, self.state, self.inliers, self.lost_frames, self.v) = (map, c, State::Normal, 0, 0, [0.0; 3]);
            self.keyframes.clear(); // they refer to the old map's points
        }
    }

    /// Lost: match against the whole map (ratio test, so repeating tiles cannot snap the pose one tile over),
    /// then confirm with guided matching from the candidate pose, whose result alone decides the state.
    /// Global matches agree only 13-21% even when right, so a weak global result is enough to try the
    /// confirmation: after a blackout on tiles they gave 6-15 inliers of ~200, some near the true pose.
    fn relocalize(&mut self, kps: &[Keypoint], desc: &[Descriptor], sensor: &M3) -> (Vec<(usize, Guided)>, Vec<(usize, usize)>, V3, f32, Vec<bool>) {
        let (guided, matches, c, yaw, inl) = self.solve(kps, desc, sensor, true);
        if inl.iter().filter(|&&b| b).count() < LIMITED_INLIERS / 2 {
            return (guided, matches, c, yaw, inl);
        }
        let last = (self.c, self.yaw, self.r, self.state, self.v);
        (self.c, self.yaw, self.r, self.state, self.v) = (c, yaw, self.rotation(sensor, yaw), State::Limited, [0.0; 3]);
        let out = self.solve(kps, desc, sensor, false);
        (self.c, self.yaw, self.r, self.state, self.v) = last;
        out
    }

    /// Normal quality, from `self.c`: the bar to leave Lost or to merge into the stash. A camera under the
    /// floor (z = 0) or a height jump is a wrong pose however many inliers (room replay: 26 units away at
    /// z -2.56 with 43 inliers; 20 units away at z 4-7 confirmed 13%). The height bound also bounds the
    /// stash merge scale.
    fn confirmed(&self, inliers: usize, matches: usize, c: V3) -> bool {
        c[2] > 0.0 && (c[2] - self.c[2]).abs() <= MAX_RELOC_DZ && inliers >= NORMAL_INLIERS && inliers * 8 >= matches
    }

    /// Relocalize against the stash; on a confirmed hit it becomes the map again. `keep`: the current pose is
    /// good, so the current map is carried into the stash frame (similarity: every seed assumed
    /// `CAMERA_HEIGHT`, so the maps differ in scale by the height ratio) and appended.
    fn try_stash(&mut self, kps: &[Keypoint], desc: &[Descriptor], sensor: &M3, keep: bool) -> bool {
        if self.stash.is_empty() {
            return false;
        }
        std::mem::swap(&mut self.map, &mut self.stash);
        let (_, matches, c, yaw, inl) = self.relocalize(kps, desc, sensor);
        let n = inl.iter().filter(|&&b| b).count();
        if !self.confirmed(n, matches.len(), c) {
            std::mem::swap(&mut self.map, &mut self.stash);
            return false;
        }
        for (&(mi, _), _) in matches.iter().zip(&inl).filter(|(_, ok)| **ok) {
            self.map[mi].seen = self.frame; // the restored points must not be first to go on eviction
        }
        let mut current = std::mem::take(&mut self.stash);
        if keep {
            let (s, rz) = (c[2] / self.c[2], rot_z(yaw - self.yaw));
            let to_old = |x: V3| add(c, scale(mulv(&rz, sub(x, self.c)), s));
            for p in &mut current {
                p.x = to_old(p.x);
                p.seed = p.seed.map(|(c1, d1)| (to_old(c1), mulv(&rz, d1)));
                p.dist *= s;
            }
            self.map.append(&mut current);
            self.metres_per_unit = None; // map units changed
        }
        (self.c, self.yaw, self.r, self.state, self.inliers, self.v) = (c, yaw, self.rotation(sensor, yaw), State::Normal, n, [0.0; 3]);
        self.motion.clear();
        self.keyframes.clear(); // they refer to poses in the other frame
        self.merges += 1;
        true
    }

    /// Matches (guided from the current pose, or global) and the RANSAC pose from them.
    fn solve(&mut self, kps: &[Keypoint], desc: &[Descriptor], sensor: &M3, global: bool) -> (Vec<(usize, Guided)>, Vec<(usize, usize)>, V3, f32, Vec<bool>) {
        let guided = if global { Vec::new() } else { self.match_guided(kps, desc) };
        let matches: Vec<(usize, usize)> = if global {
            self.match_global(desc)
        } else {
            guided.iter().filter_map(|&(mi, g)| if let Guided::Match(ki) = g { Some((mi, ki)) } else { None }).collect()
        };
        let pairs: Vec<Pair> = matches.iter().map(|&(mi, ki)| (self.map[mi].x, self.k.ray(kps[ki].x, kps[ki].y))).collect();
        let (c, yaw, inl) = if global { self.ransac(&pairs, sensor, 128, 3) } else { self.ransac(&pairs, sensor, RANSAC_ITERS, 1) };
        (guided, matches, c, yaw, inl)
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
    /// Best descriptor among keypoints near each projected map point, if clearly better than the runner-up:
    /// on tile grids the nearest wrong corner is often a close second (replays: wrong Normal poses).
    fn match_guided(&self, kps: &[Keypoint], desc: &[Descriptor]) -> Vec<(usize, Guided)> {
        let radius: f32 = if self.state == State::Normal { 40.0 } else { 80.0 };
        let (r, c) = (self.r, add(self.c, self.v)); // constant-velocity prediction
        let fe = crate::FrontEnd::default();
        let (ln_s, top) = (fe.scale.ln(), (fe.levels - 1) as f32);
        // Keypoints bucketed in radius-sized cells: each map point scans 3x3 cells, not every keypoint.
        let (gw, gh) = ((2.0 * self.k.cx / radius) as usize + 1, (2.0 * self.k.cy / radius) as usize + 1);
        let bucket = |p: f32, n: usize| ((p / radius).max(0.0) as usize).min(n - 1);
        let mut grid = vec![Vec::new(); gw * gh];
        for (ki, k) in kps.iter().enumerate() {
            grid[bucket(k.y, gh) * gw + bucket(k.x, gw)].push(ki);
        }
        let grid = &grid;
        self.map
            .iter()
            .enumerate()
            .filter_map(|(mi, mp)| {
                let (u, v) = self.k.project(&r, c, mp.x)?;
                // Farther than when described -> smaller in the image -> found on a finer level (ORB-SLAM's
                // scale prediction): other levels are skipped, they are the same corner at the wrong scale.
                let lvl = (mp.level as f32 - (norm(sub(mp.x, c)) / mp.dist).ln() / ln_s).clamp(0.0, top);
                let (xs, ys) = (bucket(u - radius, gw)..=bucket(u + radius, gw), bucket(v - radius, gh)..=bucket(v + radius, gh));
                let near = |x: f32, y: f32, d: f32| move |&ki: &usize| (kps[ki].x - x).powi(2) + (kps[ki].y - y).powi(2) <= d * d;
                let candidates = || {
                    ys.clone()
                        .flat_map(|y| xs.clone().flat_map(move |x| grid[y * gw + x].iter().copied()))
                        .filter(near(u, v, radius))
                        .filter(move |&ki| (kps[ki].level as f32 - lvl).abs() <= 1.0)
                        .map(|ki| (mp.views.iter().chain([&mp.desc]).map(|d| hamming(d, &desc[ki])).min().unwrap_or(u32::MAX), ki))
                };
                // No keypoint counts only where one could be detected: inside the image, off the border.
                let b = BORDER as f32;
                let detectable = (b..2.0 * self.k.cx - b).contains(&u) && (b..2.0 * self.k.cy - b).contains(&v);
                let Some((h, ki)) = candidates().min() else { return detectable.then_some((mi, Guided::NoKeypoint)) };
                if h > MATCH_DIST {
                    return Some((mi, Guided::TooFar));
                }
                // The runner-up skips the same corner found again (another pyramid level, a pixel or two away).
                let twin = near(kps[ki].x, kps[ki].y, 3.0);
                let second = candidates().filter(|c| !twin(&c.1)).map(|c| c.0).min().unwrap_or(u32::MAX);
                Some((mi, if (h as f32) < RATIO * second as f32 { Guided::Match(ki) } else { Guided::Ambiguous }))
            })
            .collect()
    }

    /// Best descriptor among all keypoints, kept only if clearly better than the runner-up.
    fn match_global(&self, desc: &[Descriptor]) -> Vec<(usize, usize)> {
        self.map
            .iter()
            .enumerate()
            .filter_map(|(mi, mp)| {
                let (mut best, mut second, mut bk) = (u32::MAX, u32::MAX, 0);
                for (ki, d) in desc.iter().enumerate() {
                    let h = hamming(&mp.desc, d);
                    if h < best {
                        (second, best, bk) = (best, h, ki);
                    } else if h < second {
                        second = h;
                    }
                }
                (best <= MATCH_DIST && (best as f32) < RATIO * second as f32).then_some((mi, bk))
            })
            .collect()
    }

    /// RANSAC over (position, yaw) from the current pose: `iters` 2-point samples, each solved with
    /// `steps` Gauss-Newton steps (several when the start pose is far, e.g. relocalizing).
    fn ransac(&mut self, pairs: &[Pair], sensor: &M3, iters: usize, steps: usize) -> (V3, f32, Vec<bool>) {
        let (c0, yaw0) = (self.c, self.yaw);
        let mut best = (c0, yaw0, self.inliers(pairs, sensor, c0, yaw0));
        let count = |v: &[bool]| v.iter().filter(|&&b| b).count();
        if pairs.len() < 2 {
            return best;
        }
        for _ in 0..iters {
            let (i, j) = (self.next_rand(pairs.len()), self.next_rand(pairs.len()));
            let r = self.rotation(sensor, yaw0);
            let (di, dj) = (mulv(&r, pairs[i].1), mulv(&r, pairs[j].1));
            if i == j || dot(di, dj) > 0.9994 {
                continue; // same or near-parallel rays (< 2 deg) constrain nothing
            }
            let mut hyp = Some((c0, yaw0));
            for _ in 0..steps {
                hyp = hyp.and_then(|(c, yaw)| step(&[pairs[i], pairs[j]], sensor, c, yaw));
            }
            let Some((c, yaw)) = hyp else { continue };
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

    /// Walk sideways until none of the initial map is in view, then come back.
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

    /// Lost, then the camera reappears 0.5 away (beyond the guided-match radius): relocalizes onto
    /// the existing map instead of re-seeding a new one.
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

    /// Nobody changes height by half while lost: such a relocalization is a wrong pose.
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

    /// Enough inliers but few of many matches agreeing is a wrong pose, never Normal.
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

    /// The floor seen from below (looking up) matches like a ceiling; a pose under the floor is impossible.
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
}
