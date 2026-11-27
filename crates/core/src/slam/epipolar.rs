use super::*;

impl Slam {
    /// Creates off-floor points (walls, furniture), which floor seeding never makes. Unmatched keypoints of the newest keyframe are matched with earlier keyframes, and kept if the two rays really meet. They also need enough angle between the views, a height above the floor and a reasonable distance.
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
