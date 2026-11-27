//! No web APIs here, so it is unit-tested and shared by both GPU backends.

use crate::FrontEnd;
use crate::fast::{self, Keypoint};
use crate::image::Gray;
use crate::orb::{self, Descriptor};

/// RGBA per pixel = (score low byte, score high byte, corner kept flag, blurred grey).
pub struct Level {
    pub w: usize,
    pub h: usize,
    pub rgba: Vec<u8>,
}

impl Level {
    pub fn from_cpu(score: &[u16], survivors: &[(usize, usize)], blurred: &Gray) -> Self {
        let mut rgba: Vec<u8> = score.iter().zip(&blurred.data).flat_map(|(s, b)| [*s as u8, (*s >> 8) as u8, 0, *b]).collect();
        for &(x, y) in survivors {
            rgba[(y * blurred.w + x) * 4 + 2] = 255;
        }
        Self { w: blurred.w, h: blurred.h, rgba }
    }

    pub fn score_at(&self, x: usize, y: usize) -> u16 {
        let p = &self.rgba[(y * self.w + x) * 4..];
        p[0] as u16 | (p[1] as u16) << 8
    }

    pub fn score(&self) -> Vec<u16> {
        self.rgba.chunks_exact(4).map(|p| p[0] as u16 | (p[1] as u16) << 8).collect()
    }

    pub fn survivors(&self) -> Vec<(usize, usize)> {
        self.rgba.chunks_exact(4).enumerate().filter(|(_, p)| p[2] > 127).map(|(i, _)| (i % self.w, i / self.w)).collect()
    }

    pub fn blurred(&self) -> Gray {
        Gray { w: self.w, h: self.h, data: self.rgba.chunks_exact(4).map(|p| p[3]).collect() }
    }
}

pub fn features(levels: &[Level], cfg: &FrontEnd, gravity_of: impl Fn(&Keypoint) -> Option<f32>) -> (Vec<Keypoint>, Vec<Descriptor>) {
    let kps = levels
        .iter()
        .enumerate()
        .flat_map(|(l, lv)| fast::level_keypoints(|x, y| lv.score_at(x, y), &lv.survivors(), l, cfg.scale))
        .collect();
    let (w, h) = (levels[0].w, levels[0].h);
    let kps = fast::retain_best(kps, w, h, cfg.grid.0, cfg.grid.1, cfg.max_keypoints);
    let blurred: Vec<Gray> = levels.iter().map(Level::blurred).collect();
    let desc = orb::describe_all(&blurred, cfg.scale, &kps, gravity_of);
    (kps, desc)
}

pub fn cpu_levels(levels: &[Gray], cfg: &FrontEnd, border: usize) -> Vec<Level> {
    levels
        .iter()
        .map(|lvl| {
            let score = fast::score_map(lvl, cfg.fast_threshold, border);
            Level::from_cpu(&score, &fast::nms(&score, lvl.w, lvl.h), &crate::image::blur(lvl))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::image;

    #[test]
    fn readback_round_trip_and_features() {
        let mut g = Gray::new(96, 96);
        for y in 30..60 {
            for x in 30..60 {
                g.data[y * 96 + x] = 200;
            }
        }
        let cfg = FrontEnd { levels: 2, ..Default::default() };
        let levels = image::pyramid(g, cfg.levels, cfg.scale);
        let rb = cpu_levels(&levels, &cfg, orb::BORDER);
        let score = fast::score_map(&levels[0], cfg.fast_threshold, orb::BORDER);
        assert_eq!(rb[0].score(), score);
        assert_eq!(rb[0].survivors(), fast::nms(&score, 96, 96));
        assert_eq!(rb[0].blurred(), image::blur(&levels[0]));

        let (kps, desc) = features(&rb, &cfg, |_| Some(std::f32::consts::FRAC_PI_2));
        assert_eq!(kps.len(), desc.len());
        assert!(kps.len() >= 4, "square corners: {kps:?}");
    }
}
