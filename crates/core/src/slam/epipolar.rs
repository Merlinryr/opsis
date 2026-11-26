use super::*;

impl Slam {
    /// Off-floor points (walls, furniture) that floor seeding never makes: the newest keyframe's free
    /// keypoints matched against earlier keyframes' (Hamming <= `EPI_DIST`, ratio test), kept if the two
    /// rays meet (the epipolar check: `triangulate` rejects rays that miss by > 2% of depth), with enough
    /// parallax, above the floor and within reach. They start on probation, like re-triangulated points.
    pub(super) fn epipolar_points(&mut self) {
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
}
