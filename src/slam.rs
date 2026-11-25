//! M01 phase 2: ground-plane SLAM. Pitch and roll come from the device sensor; vision
//! solves camera position and a heading (yaw) correction from 2D-3D matches.
//! The map starts as keypoint rays hitting the floor one unit below the first camera
//! (scale is arbitrary until M01 adds metric scale).

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
const REINIT_AFTER_LOST: u32 = 90; // frames (~3 s) of failed relocalization: then re-seed from the floor
const RATIO: f32 = 0.8; // Lowe ratio, guided and global matching
const MAX_MISSES: u8 = 10;
const MIN_PARALLAX_COS: f32 = 0.99756; // cos(4 deg)
const CELL: usize = 8; // px: one map point per cell when growing the map
const RANSAC_ITERS: usize = 64;
const MAX_RELOC_DZ: f32 = 0.5 * CAMERA_HEIGHT;
const OLD: u32 = 50; // frames: a point this old counts in `Slam::old`

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
}

/// One 2D-3D correspondence: world point and camera-frame ray.
type Pair = (V3, V3);

impl Slam {
    pub fn new(k: Intrinsics) -> Self {
        let r = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        Self { k, map: Vec::new(), c: [0.0, 0.0, CAMERA_HEIGHT], yaw: 0.0, r, state: State::Initializing, inliers: 0, matches: 0, old: [0; 5], lost_frames: 0, frame: 0, rng: 0x1234_5678 }
    }

    pub fn map_len(&self) -> usize {
        self.map.len()
    }

    fn rotation(&self, sensor: &M3, yaw: f32) -> M3 {
        mul(&rot_z(yaw), sensor)
    }

    /// Map point on the floor hit by the ray through (x, y), if well below the horizon and not too far.
    fn floor_point(&self, r: &M3, c: V3, x: f32, y: f32, desc: Descriptor) -> Option<MapPoint> {
        let d = mulv(r, self.k.ray(x, y));
        let t = -c[2] / d[2];
        (-d[2] > MIN_BELOW_HORIZON && t > 0.0 && t <= MAX_DEPTH).then(|| MapPoint { x: add(c, scale(d, t)), desc, misses: 0, seed: Some((c, d)), seen: self.frame, born: self.frame })
    }

    /// One frame: keypoints + descriptors and the sensor camera-to-world rotation.
    /// Apply the sensor rotation only (sensor-only mode, for tuning FOV and delay).
    pub fn orient(&mut self, sensor: &M3) {
        self.r = self.rotation(sensor, self.yaw);
    }

    pub fn update(&mut self, kps: &[Keypoint], desc: &[Descriptor], sensor: &M3) {
        self.orient(sensor);
        self.frame += 1;
        if self.state == State::Initializing {
            return self.initialize(kps, desc);
        }
        // Lost: relocalize by matching against the whole map (ratio test, so repeating tiles cannot
        // snap the pose one tile over), then confirm with guided matching from the candidate pose, which
        // alone decides the state. Global matches agree only 13-21% even when right, and the room replay
        // relocalized 30 units away, below the floor, as Normal with 26/185 inliers (map culled with it).
        let lost = self.state == State::Lost;
        let (mut guided, mut matches, mut c, mut yaw, mut inl) = self.solve(kps, desc, sensor, lost);
        // Confirmation is the gate, so a weak global result is enough to try it: after a blackout on tiles,
        // global matches gave 6-15 inliers of ~200, some near the true pose (corridor replay).
        if lost && inl.iter().filter(|&&b| b).count() >= LIMITED_INLIERS / 2 {
            let last = (self.c, self.yaw, self.r, self.state);
            (self.c, self.yaw, self.r, self.state) = (c, yaw, self.rotation(sensor, yaw), State::Limited);
            (guided, matches, c, yaw, inl) = self.solve(kps, desc, sensor, false);
            (self.c, self.yaw, self.r, self.state) = last;
        }
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
            _ if c[2] <= 0.0 || (lost && (c[2] - self.c[2]).abs() > MAX_RELOC_DZ) => State::Lost,
            n if n >= NORMAL_INLIERS && n * 8 >= matches.len() => State::Normal,
            n if n >= LIMITED_INLIERS && n * 4 >= matches.len() => State::Limited,
            _ => State::Lost,
        };
        if self.state == State::Lost {
            // Keep the last pose and keep relocalizing. shortcut: after REINIT_AFTER_LOST frames the map is
            // re-seeded from the floor at the last x/y (new place); anchors can jump.
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
        for (&(mi, ki), &ok) in matches.iter().zip(&inl) {
            let mp = &mut self.map[mi];
            if ok {
                // Latest view: descriptors drift with viewpoint; doubled re-matched old points on the replays
                // (keeping up to 4 views gained little more and let a wrong relocalization through).
                mp.desc = desc[ki];
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
                    Some(x) => (mp.x, mp.misses) = (x, MAX_MISSES - 3),
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
            if let Some(mp) = self.floor_point(&r, c, k.x, k.y, *d) {
                occupied[y * gw + x] = true;
                self.map.push(mp);
                added += 1;
            }
        }
    }

    fn initialize(&mut self, kps: &[Keypoint], desc: &[Descriptor]) {
        let (r, c) = (self.r, [self.c[0], self.c[1], CAMERA_HEIGHT]);
        let map: Vec<MapPoint> = kps
            .iter()
            .zip(desc)
            .filter_map(|(k, d)| self.floor_point(&r, c, k.x, k.y, *d))
            .collect();
        if map.len() >= MIN_INIT_POINTS {
            (self.map, self.c, self.state, self.inliers, self.lost_frames) = (map, c, State::Normal, 0, 0);
        }
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
        let (r, c) = (self.r, self.c);
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
                let (xs, ys) = (bucket(u - radius, gw)..=bucket(u + radius, gw), bucket(v - radius, gh)..=bucket(v + radius, gh));
                let near = |x: f32, y: f32, d: f32| move |&ki: &usize| (kps[ki].x - x).powi(2) + (kps[ki].y - y).powi(2) <= d * d;
                let candidates = || {
                    ys.clone()
                        .flat_map(|y| xs.clone().flat_map(move |x| grid[y * gw + x].iter().copied()))
                        .filter(near(u, v, radius))
                        .map(|ki| (hamming(&mp.desc, &desc[ki]), ki))
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
        let sensor = camera_from_orientation(0.0, 45.0, 0.0);
        let mut slam = Slam::new(K);
        let xs: Vec<f32> = (0..=20).chain((0..20).rev()).map(|i| i as f32 * 0.03).collect();
        for &x in &xs {
            let (kps, desc) = observe(&slam, &sensor, [x, 0.0, 1.0], &w);
            slam.update(&kps, &desc, &sensor);
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
        let right = on_platform.iter().filter(|z| (**z - 0.4).abs() < 0.05).count();
        assert!(on_platform.len() >= 20 && right * 10 >= on_platform.len() * 8, "{right}/{} platform points at true height", on_platform.len());
    }

    /// Lost, then the camera reappears 0.5 away (beyond the guided-match radius): relocalizes onto
    /// the existing map instead of re-seeding a new one.
    #[test]
    fn relocalizes_after_a_jump() {
        let mut s = 21;
        let w = world(&mut s);
        let sensor = camera_from_orientation(0.0, 45.0, 0.0);
        let mut slam = Slam::new(K);
        let (kps, desc) = observe(&slam, &sensor, [0.0, 0.0, 1.0], &w);
        slam.update(&kps, &desc, &sensor);
        for _ in 0..3 {
            slam.update(&[], &[], &sensor);
        }
        assert_eq!(slam.state, State::Lost);
        let target = [0.5, 0.3, 1.0];
        let (kps, desc) = observe(&slam, &sensor, target, &w);
        slam.update(&kps, &desc, &sensor);
        assert_eq!(slam.state, State::Normal, "inliers {}", slam.inliers);
        assert!(norm(sub(slam.c, target)) < 0.02, "c {:?}", slam.c);
    }

    /// Most keypoints have a twin far away (repeating texture): globally ambiguous, so only a few
    /// match globally, but guided matching from that weak pose confirms it.
    #[test]
    fn relocalizes_from_a_weak_global_match() {
        let mut s = 21;
        let w = world(&mut s);
        let sensor = camera_from_orientation(0.0, 45.0, 0.0);
        let mut slam = Slam::new(K);
        let (kps, desc) = observe(&slam, &sensor, [0.0, 0.0, 1.0], &w);
        slam.update(&kps, &desc, &sensor);
        slam.update(&[], &[], &sensor);
        let target = [0.5, 0.3, 1.0];
        let (mut kps, mut desc) = observe(&slam, &sensor, target, &w);
        for i in 24..kps.len() {
            kps.push(Keypoint { y: (kps[i].y + 320.0) % 640.0, ..kps[i] });
            desc.push(desc[i]);
        }
        slam.update(&kps, &desc, &sensor);
        assert_eq!(slam.state, State::Normal, "inliers {}/{}", slam.inliers, slam.matches);
        assert!(norm(sub(slam.c, target)) < 0.02, "c {:?}", slam.c);
    }

    /// Nobody changes height by half while lost: such a relocalization is a wrong pose.
    #[test]
    fn never_relocalizes_to_another_height() {
        let mut s = 21;
        let w = world(&mut s);
        let sensor = camera_from_orientation(0.0, 45.0, 0.0);
        let mut slam = Slam::new(K);
        let (kps, desc) = observe(&slam, &sensor, [0.0, 0.0, 1.0], &w);
        slam.update(&kps, &desc, &sensor);
        slam.update(&[], &[], &sensor);
        let (kps, desc) = observe(&slam, &sensor, [0.0, 0.3, 1.7], &w);
        slam.update(&kps, &desc, &sensor);
        assert_eq!(slam.state, State::Lost, "c {:?} inliers {}", slam.c, slam.inliers);
    }

    /// Enough inliers but few of many matches agreeing is a wrong pose, never Normal.
    #[test]
    fn few_agreeing_matches_are_not_normal() {
        let mut s = 21;
        let w = world(&mut s);
        let sensor = camera_from_orientation(0.0, 45.0, 0.0);
        let mut slam = Slam::new(K);
        let (mut kps, desc) = observe(&slam, &sensor, [0.0, 0.0, 1.0], &w);
        slam.update(&kps, &desc, &sensor);
        // All but 30 keypoints moved 20 px in scattered directions: they still match, but agree on no pose.
        for (i, k) in kps.iter_mut().enumerate().skip(30) {
            let a = i as f32 * 2.4;
            (k.x, k.y) = (k.x + 20.0 * a.cos(), k.y + 20.0 * a.sin());
        }
        slam.update(&kps, &desc, &sensor);
        assert!(slam.inliers >= NORMAL_INLIERS && slam.inliers * 8 < slam.matches, "{}/{}", slam.inliers, slam.matches);
        assert_ne!(slam.state, State::Normal);
    }

    /// The floor seen from below (looking up) matches like a ceiling; a pose under the floor is impossible.
    #[test]
    fn never_relocalizes_below_the_floor() {
        let mut s = 21;
        let w = world(&mut s);
        let down = camera_from_orientation(0.0, 45.0, 0.0);
        let mut slam = Slam::new(K);
        let (kps, desc) = observe(&slam, &down, [0.0, 0.0, 1.0], &w);
        slam.update(&kps, &desc, &down);
        slam.update(&[], &[], &down);
        let up = camera_from_orientation(0.0, 135.0, 0.0);
        let (kps, desc) = observe(&slam, &up, [0.0, 0.0, -1.0], &w);
        assert!(kps.len() > 100, "{}", kps.len());
        slam.update(&kps, &desc, &up);
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
        let sensor = camera_from_orientation(0.0, 45.0, 0.0);
        let mut slam = Slam::new(K);
        let (kps, desc) = observe(&slam, &sensor, [0.0, 0.0, 1.0], &w);
        slam.update(&kps, &desc, &sensor);
        for _ in 0..3 {
            slam.update(&[], &[], &sensor);
        }
        let target = [0.4, 0.0, 1.0];
        let (kps, desc) = observe(&slam, &sensor, target, &w);
        slam.update(&kps, &desc, &sensor);
        let at_truth = norm(sub(slam.c, target)) < 0.05;
        assert!(slam.state == State::Lost || at_truth, "state {:?} c {:?}", slam.state, slam.c);
    }

    #[test]
    fn blurred_frames_keep_the_map() {
        let mut s = 13;
        let w = world(&mut s);
        let sensor = camera_from_orientation(0.0, 45.0, 0.0);
        let mut slam = Slam::new(K);
        let (kps, desc) = observe(&slam, &sensor, [0.0, 0.0, 1.0], &w);
        slam.update(&kps, &desc, &sensor);
        slam.update(&kps, &desc, &sensor);
        let before = slam.map_len();
        // Blur: only the corners in a band of the image survive, enough to stay Normal for longer than MAX_MISSES.
        let (few, fd): (Vec<_>, Vec<_>) = kps.iter().zip(&desc).filter(|(k, _)| k.y > 420.0).map(|(k, d)| (*k, *d)).unzip();
        for _ in 0..2 * MAX_MISSES {
            slam.update(&few, &fd, &sensor);
            assert_eq!(slam.state, State::Normal, "{} inliers", slam.inliers);
        }
        assert!(slam.map_len() >= before, "map {before} -> {}", slam.map_len());
    }

    #[test]
    fn guided_matching_skips_ambiguous_neighbours() {
        let mut s = 11;
        let w = world(&mut s);
        let sensor = camera_from_orientation(0.0, 45.0, 0.0);
        let mut slam = Slam::new(K);
        let (mut kps, mut desc) = observe(&slam, &sensor, [0.0, 0.0, 1.0], &w);
        slam.update(&kps, &desc, &sensor);
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
