use std::collections::{HashMap, HashSet};

use crate::fast::Keypoint;
use crate::geom::{self, M3, V3, add, cross, dot, mul, mulv, norm, normalize, rot_z, scale, sub, transpose};
use crate::orb::{BORDER, Descriptor, hamming};

mod ba;
mod epipolar;
mod reloc;
mod scale;
#[cfg(test)]
mod tests;
pub use reloc::step;

#[derive(Clone, Copy, Debug)]
pub struct Intrinsics {
    pub f: f32,
    pub cx: f32,
    pub cy: f32,
}

impl Intrinsics {
    // shortcut: browsers don't give camera intrinsics, so they are guessed from the field of view. Upgrade with a per-device table or calibration.
    pub fn from_fov(w: usize, h: usize, fov_deg: f32) -> Self {
        let f = 0.5 * w.max(h) as f32 / (0.5 * fov_deg.to_radians()).tan();
        Self { f, cx: 0.5 * w as f32 - 0.5, cy: 0.5 * h as f32 - 0.5 }
    }
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
    Initializing = 0,
    Normal = 1,
    Limited = 2,
    Lost = 3,
}

struct MapPoint {
    x: V3,
    desc: Descriptor,
    misses: u8,
    /// Unconfirmed floor point: (camera centre, world ray) when first seen. Seen again from another angle, it is either confirmed or moved to where the two rays meet.
    seed: Option<(V3, V3)>,
    /// Frame it was last matched: the oldest go first when the map is full.
    seen: u32,
    born: u32,
    /// Stable id used by keyframes (map indices change when points are removed).
    id: u32,
    /// Pyramid level and distance when described: predicts which level it shows up on now.
    level: u8,
    dist: f32,
    views: Vec<Descriptor>,
}

struct Keyframe {
    c: V3,
    yaw: f32,
    sensor: M3,
    obs: Vec<(u32, V3)>,
    free: Vec<(V3, Descriptor, u8)>,
}

#[derive(Clone, Copy, PartialEq)]
enum Guided {
    NoKeypoint,
    /// Best match was too different. `true`: a corner was found within 3 px (it just looks different now); `false`: no corner there.
    TooFar(bool, u8),
    Ambiguous,
    Match(usize),
}

// shortcut: starting values, not tuned on real sequences; revisit in M01 phase 5.
const CAMERA_HEIGHT: f32 = 1.0;
const MIN_BELOW_HORIZON: f32 = 0.26; // sin(15°): closer to the horizon, floor depth is too unreliable.
const MAX_DEPTH: f32 = 4.0 * CAMERA_HEIGHT;
const MIN_INIT_POINTS: usize = 100; // Fewer points made a map too thin to track when the first frames were blurry.
const MAX_MAP: usize = 3000; // 1500 filled up within 60 frames of a 1.8 m walk; 3000 tracks longer walks better (replays).
const MAX_ADD: usize = 50;
const MATCH_DIST: u32 = 64;
const INLIER_PX: f32 = 4.0;
const NORMAL_INLIERS: usize = 25;
const LIMITED_INLIERS: usize = 15;
const REINIT_AFTER_LOST: u32 = 90; // ~3 s of failed recovery, then start a new map (the old one is kept as the stash).
const STASH_EVERY: u32 = 5; // Frames between attempts to find the stashed map again.
const RATIO: f32 = 0.8;
const MAX_MISSES: u8 = 10;
const MIN_PARALLAX_COS: f32 = 0.99756; // cos(4 deg)
const CELL: usize = 8; // px: one map point per cell when growing the map
const RANSAC_ITERS: usize = 64;
const MAX_RELOC_DZ: f32 = 0.5 * CAMERA_HEIGHT;
const OLD: u32 = 50;
const MAX_TURN_RATE: f32 = 1.22; // 70°/s (as in 8th Wall): faster turns blur the image, so the map is not edited.
const FRAME_S: f32 = 1.0 / 30.0; // Frame time assumed when none is given (unit tests, old recordings).
// Frame-count limits are tuned at 30 fps and scaled by `Slam::rate` (2 at 60 fps), so they last the same time.
const SCALE_WINDOW_S: f32 = 5.0; // Seconds of motion used for each metric-scale estimate.
const MAX_LAG_S: f32 = 0.1; // Camera-to-motion-sensor delay searched within ±0.1 s (phone walks fit at 0–33 ms).
const SCALE_RANGE: std::ops::RangeInclusive<f32> = 0.3..=3.0; // Metres per map unit (1 unit = the starting camera height).
// Keyframe rules, after 8th Wall: wait 5 frames, then add one on ≥ 20 new points, < 120 map points, or enough movement.
const KF_COOLDOWN: u32 = 5;
const KF_NEW_POINTS: usize = 20;
const KF_LOW_MAP: usize = 120;
const KF_BASELINE: f32 = 0.1 * CAMERA_HEIGHT; // ...or after moving this far (PTAM's rule).
const BA_WINDOW: usize = 4;
const KF_PAIRS: usize = 3; // Earlier keyframes a new keyframe is matched against.
const EPI_DIST: u32 = 32; // Max descriptor distance (Hamming) for matches between keyframes.
const MAX_VIEWS: usize = 3; // Extra keyframe descriptors kept per point, for guided matching only (more let wrong recoveries through).

pub struct Slam {
    pub k: Intrinsics,
    map: Vec<MapPoint>,
    pub c: V3,
    /// Heading correction applied on top of the sensor rotation.
    pub yaw: f32,
    pub r: M3,
    pub state: State,
    pub inliers: usize,
    pub matches: usize,
    /// Replay statistics for points older than `OLD` frames, cumulative: [0] no corner near, [1] best match too different, [2] ambiguous, [3] RANSAC outlier, [4] inlier. [5]..[9] break [1] down further (printed by `replay.rs`).
    pub old: [u32; 10],
    lost_frames: u32,
    frame: u32,
    rng: u32,
    v: V3,
    last_sensor: M3,
    keyframes: Vec<Keyframe>,
    last_kf: u32,
    next_id: u32,
    last_t: f64,
    frame_dt: f32,
    /// Recent tracked frames (camera centre, world acceleration m/s², time s) for the scale estimate.
    motion: Vec<(V3, V3, f32)>,
    /// Metres per map unit and camera-to-sensor delay (ms), once estimated. Reported only: tracking does not use them.
    pub metres_per_unit: Option<f32>,
    pub imu_lag_ms: Option<f32>,
    /// The previous map, kept after starting a new one, because placed content lives in its frame. Tracking keeps trying to find it again; on success the new map is merged into it.
    stash: Vec<MapPoint>,
    pub reseeds: u32,
    pub merges: u32,
    /// Replay diagnostic: (birth frame, inlier?) for each match this frame.
    pub matched_born: Vec<(u32, bool)>,
}

type Pair = (V3, V3);

impl Slam {
    pub fn new(k: Intrinsics) -> Self {
        let r = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        Self { k, map: Vec::new(), c: [0.0, 0.0, CAMERA_HEIGHT], yaw: 0.0, r, state: State::Initializing, inliers: 0, matches: 0, old: [0; 10], lost_frames: 0, frame: 0, rng: 0x2545_F491, v: [0.0; 3], last_sensor: r, keyframes: Vec::new(), last_kf: 0, next_id: 0, last_t: f64::NAN, frame_dt: FRAME_S, motion: Vec::new(), metres_per_unit: None, imu_lag_ms: None, stash: Vec::new(), reseeds: 0, merges: 0, matched_born: Vec::new() }
    }

    fn rate(&self) -> u32 {
        (FRAME_S / self.frame_dt).round().max(1.0) as u32
    }

    /// RANSAC random seed. Replays compare changes over several seeds, since the seed alone moves results.
    pub fn seed(&mut self, s: u32) {
        self.rng = s.max(1); // xorshift gets stuck at 0.
    }

    pub fn keyframes(&self) -> usize {
        self.keyframes.len()
    }

    /// Angle used to orient a floor keypoint's descriptor: the direction of world east in the image at that pixel. Unlike gravity, it stays the same as the camera walks around the point, so matches survive. None above the floor band (those use the image-based angle). Call after `orient`.
    // shortcut: a point crossing the 15° band switches angle type and stops matching. Give such points both descriptors if replays show it matters.
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

    fn floor_point(&mut self, r: &M3, c: V3, k: &Keypoint, desc: Descriptor) -> Option<MapPoint> {
        let d = mulv(r, self.k.ray(k.x, k.y));
        let t = -c[2] / d[2];
        if !(-d[2] > MIN_BELOW_HORIZON && t > 0.0 && t <= MAX_DEPTH) {
            return None;
        }
        self.next_id += 1;
        Some(MapPoint { x: add(c, scale(d, t)), desc, misses: 0, seed: Some((c, d)), seen: self.frame, born: self.frame, id: self.next_id, level: k.level, dist: t, views: Vec::new() })
    }

    pub fn orient(&mut self, sensor: &M3) {
        self.r = self.rotation(sensor, self.yaw);
    }

    /// `t`: capture time in ms (NaN if unknown). `accel`: acceleration without gravity, in camera axes (m/s²), if the device has it.
    pub fn update(&mut self, kps: &[Keypoint], desc: &[Descriptor], sensor: &M3, t: f64, accel: Option<V3>) {
        let dt = ((t - self.last_t).abs() as f32 * 1e-3).clamp(0.005, 0.2); // abs: replays also run backward.
        let dt = if dt.is_nan() { FRAME_S } else { dt };
        self.frame_dt = 0.9 * self.frame_dt + 0.1 * dt;
        self.last_t = t;
        // Blurred fast pans must not edit the map.
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
        self.matched_born = matches.iter().zip(&inl).map(|(&(mi, _), &ok)| (self.map[mi].born, ok)).collect();
        for &(mi, g) in guided.iter().filter(|&&(mi, _)| self.frame - self.map[mi].born >= OLD) {
            let b = match g {
                Guided::NoKeypoint => 0,
                Guided::TooFar(near, any) => {
                    self.old[5] += near as u32;
                    self.old[6] += (any == 3) as u32;
                    self.old[7] += (any >= 6) as u32;
                    let confirmed = self.map[mi].seed.is_none();
                    self.old[8] += confirmed as u32;
                    self.old[9] += (confirmed && any == 0) as u32;
                    1
                }
                Guided::Ambiguous => 2,
                Guided::Match(_) => 3 + is_inlier[mi] as usize,
            };
            self.old[b] += 1;
        }
        // Sanity checks. Limited needs a quarter of the matches to agree and Normal an eighth: a few lucky inliers among many matches means a wrong pose. A camera below the floor, or a big height jump while lost, is also wrong (both seen in replays).
        self.state = match self.inliers {
            n if lost && self.confirmed(n, matches.len(), c) => State::Normal,
            _ if lost || c[2] <= 0.0 => State::Lost,
            n if n >= NORMAL_INLIERS && n * 8 >= matches.len() => State::Normal,
            // Leaving Lost needs Normal quality: weaker recoveries were often 1–8 units off (replay).
            n if !lost && n >= LIMITED_INLIERS && n * 4 >= matches.len() => State::Limited,
            _ => State::Lost,
        };
        if self.state == State::Lost {
            // Keep the last pose and keep trying to recover, against the stash too. shortcut: a stash hit while lost drops the current map.
            self.v = [0.0; 3];
            self.motion.clear();
            self.lost_frames += 1;
            if self.try_stash(kps, desc, sensor, false) {
                return;
            }
            // Lost for long in a new place: start a new map, keeping the old one as the stash so coming back merges them. A covered lens can't start a map, so it stays Lost. shortcut: only one stash is kept.
            if self.lost_frames >= REINIT_AFTER_LOST * self.rate() {
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

        // Only on well-tracked frames: one blurry frame must not wipe the map.
        if self.state != State::Normal || fast {
            return;
        }
        let mut new_obs = Vec::new();
        let (r, c) = (self.r, self.c);
        let (gw, gh) = (((2.0 * self.k.cx) as usize) / CELL + 1, ((2.0 * self.k.cy) as usize) / CELL + 1);
        let cell = |u: f32, v: f32| (u >= 0.0 && v >= 0.0).then(|| (u as usize / CELL, v as usize / CELL)).filter(|&(x, y)| x < gw && y < gh);
        for (&(mi, ki), &ok) in matches.iter().zip(&inl) {
            let mp = &mut self.map[mi];
            if ok {
                // Keep only the latest descriptor: appearance changes with viewpoint, and this doubled re-matched old points (replays).
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
                match geom::triangulate(c1, d1, c, d2) {
                    Some(x) => (mp.x, mp.dist, mp.misses) = (x, mp.dist * norm(sub(x, c)) / norm(sub(mp.x, c)), MAX_MISSES - 3),
                    None => mp.misses = MAX_MISSES,
                }
            }
        }
        // Count a miss only where a corner was detected nearby. Otherwise blurry frames wiped most of the map (replay).
        let mut detected = vec![false; gw * gh];
        for (x, y) in kps.iter().filter_map(|k| cell(k.x, k.y)) {
            detected[y * gw + x] = true;
        }
        let near_corner = |x: usize, y: usize| (y.saturating_sub(1)..(y + 2).min(gh)).any(|j| (x.saturating_sub(1)..(x + 2).min(gw)).any(|i| detected[j * gw + i]));
        let mut occupied = vec![false; gw * gh];
        let tick = self.frame % self.rate() == 0;
        for (mp, &ok) in self.map.iter_mut().zip(&is_inlier) {
            let Some((x, y)) = self.k.project(&r, c, mp.x).and_then(|(u, v)| cell(u, v)) else { continue };
            occupied[y * gw + x] = true;
            if !ok && !near_corner(x, y) {
                continue;
            }
            // Misses count at 30 Hz whatever the frame rate, so points get the same time to come back.
            mp.misses = if ok { 0 } else { mp.misses.saturating_add(tick as u8) };
            if ok {
                mp.seen = self.frame;
            }
        }
        self.map.retain(|mp| mp.misses < MAX_MISSES);
        if self.map.len() > MAX_MAP - MAX_ADD {
            // Map full: drop the points seen longest ago, so the map follows the camera.
            self.map.sort_by(|a, b| b.seen.cmp(&a.seen)); // stable sort: keeps results identical across builds.
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
        if self.frame - self.last_kf >= KF_COOLDOWN * self.rate() && (added >= KF_NEW_POINTS || self.map.len() < KF_LOW_MAP || self.keyframes.last().is_none_or(|kf| norm(sub(kf.c, c)) > KF_BASELINE)) {
            let free = kps.iter().zip(desc).filter(|(k, _)| cell(k.x, k.y).is_some_and(|(x, y)| !occupied[y * gw + x])).map(|(k, d)| (self.k.ray(k.x, k.y), *d, k.level)).collect();
            self.add_keyframe(sensor, new_obs, free);
        }
    }

    fn add_keyframe(&mut self, sensor: &M3, obs: Vec<(u32, V3)>, free: Vec<(V3, Descriptor, u8)>) {
        self.last_kf = self.frame;
        // Skip a keyframe that shares ≥ 70% of points with the last one and is close to it (8th Wall rule).
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
        // Only the last few keyframes are used, so older ones are dropped.
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

    fn initialize(&mut self, kps: &[Keypoint], desc: &[Descriptor]) {
        let (r, c) = (self.r, [self.c[0], self.c[1], CAMERA_HEIGHT]);
        let map: Vec<MapPoint> = kps
            .iter()
            .zip(desc)
            .filter_map(|(k, d)| self.floor_point(&r, c, k, *d))
            .collect();
        if map.len() >= MIN_INIT_POINTS {
            (self.map, self.c, self.state, self.inliers, self.lost_frames, self.v) = (map, c, State::Normal, 0, 0, [0.0; 3]);
            self.keyframes.clear(); // Keyframes point to the old map's points.
        }
    }

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

    /// Keeps the best descriptor only if clearly better than the second best: on tiles the wrong neighbouring corner is often close.
    fn match_guided(&self, kps: &[Keypoint], desc: &[Descriptor]) -> Vec<(usize, Guided)> {
        let radius: f32 = if self.state == State::Normal { 40.0 } else { 80.0 };
        let (r, c) = (self.r, add(self.c, self.v));
        let fe = crate::FrontEnd::default();
        let (ln_s, top) = (fe.scale.ln(), (fe.levels - 1) as f32);
        // Bucket keypoints in cells so each map point only checks nearby ones.
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
                // A point farther than when described looks smaller, so it appears on a finer pyramid level. Other levels are skipped.
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
                // "No corner" only counts where one could be detected: inside the image, away from the border.
                let b = BORDER as f32;
                let detectable = (b..2.0 * self.k.cx - b).contains(&u) && (b..2.0 * self.k.cy - b).contains(&v);
                let Some((h, ki)) = candidates().min() else { return detectable.then_some((mi, Guided::NoKeypoint)) };
                if h > MATCH_DIST {
                    let d2 = kps.iter().map(|k| (k.x - u).powi(2) + (k.y - v).powi(2)).fold(f32::MAX, f32::min);
                    let any = if d2 <= 9.0 { 3 } else if d2 <= 36.0 { 6 } else { 0 };
                    return Some((mi, Guided::TooFar((kps[ki].x - u).powi(2) + (kps[ki].y - v).powi(2) <= 9.0, any)));
                }
                // Ignore the same corner found twice (another level, a pixel or two away) when picking the second best.
                let twin = near(kps[ki].x, kps[ki].y, 3.0);
                let second = candidates().filter(|c| !twin(&c.1)).map(|c| c.0).min().unwrap_or(u32::MAX);
                Some((mi, if (h as f32) < RATIO * second as f32 { Guided::Match(ki) } else { Guided::Ambiguous }))
            })
            .collect()
    }

}
