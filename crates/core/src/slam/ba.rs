use super::*;

impl Slam {
    /// Joint local bundle adjustment over the keyframe window: Gauss-Newton with a Cauchy loss, points
    /// eliminated by Schur complement. Unknowns: (centre, yaw) of every keyframe but the oldest, and every
    /// off-floor point seen by >= 2 of them. The oldest keyframe and floor points (z = 0) stay fixed: they
    /// pin gauge and scale. Residual per observation as in `step`: world ray x (X - C), over |X - C|.
    /// A moved point that no longer reprojects within `INLIER_PX` in every view goes back.
    pub(super) fn local_ba(&mut self) {
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
}
