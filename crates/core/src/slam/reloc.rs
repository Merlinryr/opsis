use super::*;

impl Slam {
    /// Lost: match against the whole map (ratio test, so repeating tiles cannot snap the pose one tile over),
    /// then confirm with guided matching from the candidate pose, whose result alone decides the state.
    /// Global matches agree only 13-21% even when right, so a weak global result is enough to try the
    /// confirmation: after a blackout on tiles they gave 6-15 inliers of ~200, some near the true pose.
    pub(super) fn relocalize(&mut self, kps: &[Keypoint], desc: &[Descriptor], sensor: &M3) -> (Vec<(usize, Guided)>, Vec<(usize, usize)>, V3, f32, Vec<bool>) {
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
    pub(super) fn confirmed(&self, inliers: usize, matches: usize, c: V3) -> bool {
        c[2] > 0.0 && (c[2] - self.c[2]).abs() <= MAX_RELOC_DZ && inliers >= NORMAL_INLIERS && inliers * 8 >= matches
    }

    /// Relocalize against the stash; on a confirmed hit it becomes the map again. `keep`: the current pose is
    /// good, so the current map is carried into the stash frame (similarity: every seed assumed
    /// `CAMERA_HEIGHT`, so the maps differ in scale by the height ratio) and appended.
    pub(super) fn try_stash(&mut self, kps: &[Keypoint], desc: &[Descriptor], sensor: &M3, keep: bool) -> bool {
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

    pub(super) fn match_global(&self, desc: &[Descriptor]) -> Vec<(usize, usize)> {
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
    pub(super) fn ransac(&mut self, pairs: &[Pair], sensor: &M3, iters: usize, steps: usize) -> (V3, f32, Vec<bool>) {
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
