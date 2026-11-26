//! FAST-9 corner score, 3x3 NMS, quadratic subpixel and spatial binning.
//! Method: Rosten & Drummond, "Machine learning for high-speed corner detection", ECCV 2006.

use crate::image::Gray;

/// Bresenham circle of radius 3, clockwise from 12 o'clock.
pub const RING: [(isize, isize); 16] = [
    (0, -3), (1, -3), (2, -2), (3, -1), (3, 0), (3, 1), (2, 2), (1, 3),
    (0, 3), (-1, 3), (-2, 2), (-3, 1), (-3, 0), (-3, -1), (-2, -2), (-1, -3),
];

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Keypoint {
    /// Level-0 pixel coordinates.
    pub x: f32,
    pub y: f32,
    pub level: u8,
    pub score: u16,
}

/// True if `mask` (16 ring bits) holds 9 contiguous set bits, wrapping around.
fn has_arc9(mask: u32) -> bool {
    let m = mask | (mask << 16);
    (0..9).fold(m, |acc, i| acc & (m >> i)) != 0
}

/// Score at (x, y): 0 if not a FAST-9 corner, else the summed excess over `t`
/// of the ring pixels on the winning polarity. Cheap and identical on the GPU.
pub fn score(img: &Gray, x: isize, y: isize, t: u8) -> u16 {
    let c = img.at(x, y) as i32;
    let t = t as i32;
    let (mut bright, mut dark, mut sb, mut sd) = (0u32, 0u32, 0i32, 0i32);
    for (i, (dx, dy)) in RING.iter().enumerate() {
        let d = img.at(x + dx, y + dy) as i32 - c;
        if d > t {
            bright |= 1 << i;
            sb += d - t;
        } else if d < -t {
            dark |= 1 << i;
            sd += -d - t;
        }
    }
    let s = match (has_arc9(bright), has_arc9(dark)) {
        (true, true) => sb.max(sd),
        (true, false) => sb,
        (false, true) => sd,
        _ => 0,
    };
    s.min(u16::MAX as i32) as u16
}

/// Full score map; pixels within `border` stay 0.
pub fn score_map(img: &Gray, t: u8, border: usize) -> Vec<u16> {
    let mut out = vec![0; img.w * img.h];
    for y in border..img.h.saturating_sub(border) {
        for x in border..img.w.saturating_sub(border) {
            out[y * img.w + x] = score(img, x as isize, y as isize, t);
        }
    }
    out
}

/// 3x3 non-maximum suppression. Ties go to the later pixel in scan order, so exactly one survives.
pub fn nms(map: &[u16], w: usize, h: usize) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    for y in 1..h.saturating_sub(1) {
        for x in 1..w.saturating_sub(1) {
            let s = map[y * w + x];
            if s == 0 {
                continue;
            }
            let wins = (-1..=1isize).all(|dy| {
                (-1..=1isize).all(|dx| {
                    let n = map[(y as isize + dy) as usize * w + (x as isize + dx) as usize];
                    let before = (dy, dx) < (0, 0);
                    (dy == 0 && dx == 0) || if before { s >= n } else { s > n }
                })
            });
            if wins {
                out.push((x, y));
            }
        }
    }
    out
}

/// Vertex offset of a parabola through (-1, a), (0, b), (1, c), in [-0.5, 0.5].
fn parabola(a: f32, b: f32, c: f32) -> f32 {
    let den = a - 2.0 * b + c;
    if den >= 0.0 { 0.0 } else { (0.5 * (a - c) / den).clamp(-0.5, 0.5) }
}

/// Subpixel keypoints of one level from its score map and NMS survivors (CPU or GPU readback).
/// `score(x, y)`: the FAST score map, read only around survivors.
pub fn level_keypoints(score: impl Fn(usize, usize) -> u16, survivors: &[(usize, usize)], level: usize, scale: f32) -> Vec<Keypoint> {
    let s = |x: usize, y: usize| score(x, y) as f32;
    let k = scale.powi(level as i32);
    survivors
        .iter()
        .map(|&(x, y)| {
            let ox = parabola(s(x - 1, y), s(x, y), s(x + 1, y));
            let oy = parabola(s(x, y - 1), s(x, y), s(x, y + 1));
            Keypoint {
                x: (x as f32 + ox + 0.5) * k - 0.5,
                y: (y as f32 + oy + 0.5) * k - 0.5,
                level: level as u8,
                score: score(x, y),
            }
        })
        .collect()
}

/// Detect on every pyramid level; coordinates are mapped to level 0.
pub fn detect(levels: &[Gray], scale: f32, t: u8, border: usize) -> Vec<Keypoint> {
    levels
        .iter()
        .enumerate()
        .flat_map(|(l, img)| {
            let map = score_map(img, t, border);
            level_keypoints(|x, y| map[y * img.w + x], &nms(&map, img.w, img.h), l, scale)
        })
        .collect()
}

/// Keep at most `max` keypoints, spread over a `gx` x `gy` grid of the `w` x `h` level-0 image.
pub fn retain_best(mut kps: Vec<Keypoint>, w: usize, h: usize, gx: usize, gy: usize, max: usize) -> Vec<Keypoint> {
    let cell = |k: &Keypoint| {
        let cx = ((k.x.max(0.0) as usize) * gx / w).min(gx - 1);
        let cy = ((k.y.max(0.0) as usize) * gy / h).min(gy - 1);
        cy * gx + cx
    };
    kps.sort_by(|a, b| cell(a).cmp(&cell(b)).then(b.score.cmp(&a.score)));
    let per_cell = max.div_ceil(gx * gy);
    let mut out: Vec<Keypoint> = kps
        .chunk_by(|a, b| cell(a) == cell(b))
        .flat_map(|c| c.iter().take(per_cell).copied())
        .collect();
    out.sort_by(|a, b| b.score.cmp(&a.score));
    out.truncate(max);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Black image with a white square; its corners are FAST corners.
    fn square() -> Gray {
        let mut g = Gray::new(64, 64);
        for y in 20..44 {
            for x in 20..44 {
                g.data[y * 64 + x] = 255;
            }
        }
        g
    }

    #[test]
    fn arc9() {
        assert!(has_arc9(0b1_1111_1111));
        assert!(has_arc9(0xF01F)); // bits 12..15 + 0..4 wrap around
        assert!(!has_arc9(0xF00F));
        assert!(!has_arc9(0b1111_1111));
        assert!(!has_arc9(0b0101_0101_0101_0101));
    }

    #[test]
    fn flat_image_has_no_corners() {
        let mut g = Gray::new(32, 32);
        g.data.fill(128);
        assert!(detect(&[g], 1.44, 20, 3).is_empty());
    }

    #[test]
    fn finds_square_corners_only() {
        let kps = detect(&[square()], 1.44, 20, 3);
        let corners = [(20.0, 20.0), (43.0, 20.0), (20.0, 43.0), (43.0, 43.0)];
        assert_eq!(kps.len(), 4, "{kps:?}");
        for (cx, cy) in corners {
            assert!(
                kps.iter().any(|k| (k.x - cx).abs() <= 1.5 && (k.y - cy).abs() <= 1.5),
                "missing corner near ({cx}, {cy}) in {kps:?}"
            );
        }
    }

    #[test]
    fn retain_spreads_and_caps() {
        let kps: Vec<_> = (0..100)
            .map(|i| Keypoint { x: (i % 10) as f32 * 10.0, y: (i / 10) as f32 * 10.0, level: 0, score: i as u16 })
            .collect();
        let kept = retain_best(kps, 100, 100, 2, 2, 8);
        assert_eq!(kept.len(), 8);
        for c in 0..4 {
            let n = kept.iter().filter(|k| (k.x as usize / 50) + 2 * (k.y as usize / 50) == c).count();
            assert_eq!(n, 2, "cell {c}");
        }
    }
}

#[cfg(test)]
mod line_tests {
    use super::*;

    /// FAST-9 must not fire on thin straight lines (tile grout), whatever their slant.
    #[test]
    fn no_corners_on_thin_lines() {
        for slope in [0.0f32, 0.05, 0.1, 0.2, 0.4] {
            for contrast in [40.0f32, 80.0] {
                let mut g = Gray::new(96, 64);
                for y in 0..64 {
                    for x in 0..96 {
                        // anti-aliased ~1.5 px dark line through (0, 30)
                        let d = ((y as f32 - 30.0 - slope * x as f32) / (1.0 + slope * slope).sqrt()).abs();
                        g.data[y * 96 + x] = (200.0 - contrast * (1.25 - d).clamp(0.0, 1.0)) as u8;
                    }
                }
                assert!(detect(&[g], 1.44, 20, 3).is_empty(), "slope {slope} contrast {contrast}");
            }
        }
    }
}
