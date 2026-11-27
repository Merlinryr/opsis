//! ORB descriptors (Rublee et al., ICCV 2011). The caller gives each keypoint's angle (world east for floor points); the image centroid is the fallback.

use crate::fast::Keypoint;
use crate::image::Gray;

pub type Descriptor = [u64; 4];

pub const PATCH_RADIUS: f32 = 15.0;
/// Keypoints this close to the edge are skipped, so every rotated test stays inside the image.
pub const BORDER: usize = 16;

// shortcut: random fixed test pattern, not a learned one; tune it if matching needs more distinctiveness.
pub fn pattern() -> [[i8; 4]; 256] {
    let mut s: u32 = 0x9E37_79B9;
    let mut uni = || {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        s as f32 / u32::MAX as f32
    };
    let mut gauss = || {
        let sigma = PATCH_RADIUS / 2.5;
        let r = loop {
            let v = ((0..4).map(|_| uni()).sum::<f32>() - 2.0) * sigma * 1.732;
            if v.abs() <= PATCH_RADIUS / 1.4142 {
                break v;
            }
        };
        r.round() as i8
    };
    std::array::from_fn(|_| [gauss(), gauss(), gauss(), gauss()])
}

pub fn gravity_angle(x: f32, y: f32, g: [f32; 3], fx: f32, fy: f32, cx: f32, cy: f32) -> f32 {
    let dx = fx * g[0] + (cx - x) * g[2];
    let dy = fy * g[1] + (cy - y) * g[2];
    dy.atan2(dx)
}

/// Angle from the patch's brightness centroid (Rublee 2011). Used when no world direction is available.
pub fn centroid_angle(level: &Gray, x: f32, y: f32) -> f32 {
    let r = PATCH_RADIUS as isize;
    let (cx, cy) = (x.round() as isize, y.round() as isize);
    let inside = cx >= r && cy >= r && cx + r < level.w as isize && cy + r < level.h as isize;
    let (mut m10, mut m01) = (0i64, 0i64);
    // Row by row over the disc: direct slices inside the image (~4x faster), clamped reads at the border.
    for v in -r..=r {
        let du = ((r * r - v * v) as f32).sqrt() as isize;
        let (mut sum, mut usum) = (0i64, 0i64);
        if inside {
            let start = ((cy + v) as usize) * level.w + (cx - du) as usize;
            for (u, &i) in (-du..=du).zip(&level.data[start..=start + 2 * du as usize]) {
                (sum, usum) = (sum + i as i64, usum + u as i64 * i as i64);
            }
        } else {
            for u in -du..=du {
                let i = level.at(cx + u, cy + v) as i64;
                (sum, usum) = (sum + i, usum + u as i64 * i);
            }
        }
        (m10, m01) = (m10 + usum, m01 + v as i64 * sum);
    }
    (m01 as f32).atan2(m10 as f32)
}

pub fn describe(level: &Gray, pat: &[[i8; 4]; 256], x: f32, y: f32, angle: f32) -> Descriptor {
    let rot = angle - std::f32::consts::FRAC_PI_2;
    let (s, c) = rot.sin_cos();
    // Patches far enough from the edge skip the bounds check.
    let r = PATCH_RADIUS + 1.0;
    let inside = x >= r && y >= r && x + r < level.w as f32 && y + r < level.h as f32;
    let px = |u: i8, v: i8| {
        let (u, v) = (u as f32, v as f32);
        // Truncating v + 0.5 rounds here; round() is a slow library call on wasm.
        let (xi, yi) = ((x + c * u - s * v + 0.5) as isize, (y + s * u + c * v + 0.5) as isize);
        if inside { level.data[yi as usize * level.w + xi as usize] } else { level.at(xi, yi) }
    };
    let mut d = [0u64; 4];
    for (i, p) in pat.iter().enumerate() {
        if px(p[0], p[1]) < px(p[2], p[3]) {
            d[i / 64] |= 1 << (i % 64);
        }
    }
    d
}

pub fn describe_all(blurred: &[Gray], scale: f32, kps: &[Keypoint], gravity_of: impl Fn(&Keypoint) -> Option<f32>) -> Vec<Descriptor> {
    let pat = pattern();
    kps.iter()
        .map(|k| {
            let img = &blurred[k.level as usize];
            let inv = scale.powi(-(k.level as i32));
            let (lx, ly) = ((k.x + 0.5) * inv - 0.5, (k.y + 0.5) * inv - 0.5);
            let angle = gravity_of(k).unwrap_or_else(|| centroid_angle(img, lx, ly));
            describe(img, &pat, lx, ly, angle)
        })
        .collect()
}

pub fn hamming(a: &Descriptor, b: &Descriptor) -> u32 {
    a.iter().zip(b).map(|(x, y)| (x ^ y).count_ones()).sum()
}

pub fn match_bf(a: &[Descriptor], b: &[Descriptor], max_dist: u32, ratio: f32) -> Vec<(usize, usize)> {
    a.iter()
        .enumerate()
        .filter_map(|(i, da)| {
            let (mut best, mut second, mut bj) = (u32::MAX, u32::MAX, 0);
            for (j, db) in b.iter().enumerate() {
                let d = hamming(da, db);
                if d < best {
                    (second, best, bj) = (best, d, j);
                } else if d < second {
                    second = d;
                }
            }
            (best <= max_dist && (best as f32) < ratio * second as f32).then_some((i, bj))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::image::blur;

    fn texture(n: usize) -> Gray {
        let mut s: u32 = 12345;
        let mut g = Gray::new(n, n);
        for v in g.data.iter_mut() {
            s = s.wrapping_mul(1_103_515_245).wrapping_add(12345);
            *v = (s >> 24) as u8;
        }
        blur(&blur(&g))
    }

    fn rot90(g: &Gray) -> Gray {
        let n = g.w;
        let mut out = Gray::new(n, n);
        for y in 0..n {
            for x in 0..n {
                out.data[x * n + (n - 1 - y)] = g.data[y * n + x];
            }
        }
        out
    }

    #[test]
    fn pattern_inside_patch() {
        for p in pattern() {
            for v in p {
                assert!((v as f32).abs() <= PATCH_RADIUS / 1.4142 + 0.5);
            }
        }
    }

    #[test]
    fn gravity_angle_straight_down_is_plus_y() {
        let a = gravity_angle(10.0, 10.0, [0.0, 1.0, 0.0], 500.0, 500.0, 320.0, 240.0);
        assert!((a - std::f32::consts::FRAC_PI_2).abs() < 1e-6);
    }

    #[test]
    fn rotation_invariant_with_gravity() {
        let (n, pat) = (96, pattern());
        let img = texture(n);
        let rimg = rot90(&img);
        let (x, y) = (40.0, 50.0);
        let down = std::f32::consts::FRAC_PI_2;
        let a = describe(&img, &pat, x, y, down);
        // Turning the image 90° clockwise turns gravity from +y to -x.
        let b = describe(&rimg, &pat, n as f32 - 1.0 - y, x, std::f32::consts::PI);
        let other = describe(&img, &pat, 60.0, 30.0, down);
        assert!(hamming(&a, &b) <= 16, "rotated distance {}", hamming(&a, &b));
        assert!(hamming(&a, &other) >= 64, "unrelated distance {}", hamming(&a, &other));
    }

    /// Looking straight down, gravity points in opposite directions on either side of the image centre, so it can't orient floor points.
    #[test]
    fn ground_rays_need_centroid() {
        let g = [0.0, 0.0, 1.0];
        let left = gravity_angle(200.0, 240.0, g, 500.0, 500.0, 320.0, 240.0);
        let right = gravity_angle(440.0, 240.0, g, 500.0, 500.0, 320.0, 240.0);
        assert!(((left - right).abs() - std::f32::consts::PI).abs() < 1e-4);

        let (n, pat) = (96, pattern());
        let img = texture(n);
        let rimg = rot90(&rot90(&img)); // Seen from the other side: the patch is rotated 180°.
        let (x, y) = (40.0, 50.0);
        let (rx, ry) = (n as f32 - 1.0 - x, n as f32 - 1.0 - y);
        let a = describe(&img, &pat, x, y, centroid_angle(&img, x, y));
        let b = describe(&rimg, &pat, rx, ry, centroid_angle(&rimg, rx, ry));
        assert!(hamming(&a, &b) <= 16, "centroid distance {}", hamming(&a, &b));
    }

    #[test]
    fn centroid_rows_match_per_pixel_disc() {
        let img = texture(96);
        let naive = |x: f32, y: f32| {
            let (cx, cy, r) = (x.round() as isize, y.round() as isize, PATCH_RADIUS as isize);
            let (mut m10, mut m01) = (0i64, 0i64);
            for v in -r..=r {
                for u in (-r..=r).filter(|u| u * u + v * v <= r * r) {
                    let i = img.at(cx + u, cy + v) as i64;
                    (m10, m01) = (m10 + u as i64 * i, m01 + v as i64 * i);
                }
            }
            (m01 as f32).atan2(m10 as f32)
        };
        for (x, y) in [(40.0, 50.0), (15.0, 15.0), (80.4, 80.6), (3.0, 90.0), (95.0, 0.0)] {
            assert_eq!(centroid_angle(&img, x, y), naive(x, y), "at {x},{y}");
        }
    }

    #[test]
    fn match_finds_identity() {
        let (img, pat) = (texture(96), pattern());
        let pts = [(30.0, 30.0), (60.0, 40.0), (40.0, 65.0)];
        let d: Vec<_> = pts.iter().map(|&(x, y)| describe(&img, &pat, x, y, 1.57)).collect();
        assert_eq!(match_bf(&d, &d, 40, 0.8), vec![(0, 0), (1, 1), (2, 2)]);
    }
}
