#[derive(Clone, Debug, PartialEq)]
pub struct Gray {
    pub w: usize,
    pub h: usize,
    pub data: Vec<u8>,
}

impl Gray {
    pub fn new(w: usize, h: usize) -> Self {
        Self { w, h, data: vec![0; w * h] }
    }

    /// Clamped read, so filters need no border cases.
    pub fn at(&self, x: isize, y: isize) -> u8 {
        let x = x.clamp(0, self.w as isize - 1) as usize;
        let y = y.clamp(0, self.h as isize - 1) as usize;
        self.data[y * self.w + x]
    }
}

/// Grey level (BT.601) in 8-bit integer maths, from RGBA bytes.
pub fn luma(rgba: &[u8], w: usize, h: usize) -> Gray {
    assert_eq!(rgba.len(), w * h * 4, "rgba size mismatch");
    let data = rgba
        .chunks_exact(4)
        .map(|p| ((77 * p[0] as u32 + 150 * p[1] as u32 + 29 * p[2] as u32) >> 8) as u8)
        .collect();
    Gray { w, h, data }
}

/// Gaussian blur [1 4 6 4 1]/16, horizontal then vertical.
pub fn blur(img: &Gray) -> Gray {
    const K: [u32; 5] = [1, 4, 6, 4, 1];
    let pass = |src: &Gray, dx: isize, dy: isize| {
        let mut out = Gray::new(src.w, src.h);
        for y in 0..src.h as isize {
            for x in 0..src.w as isize {
                let s: u32 = (0..5)
                    .map(|i| K[i] * src.at(x + (i as isize - 2) * dx, y + (i as isize - 2) * dy) as u32)
                    .sum();
                out.data[y as usize * src.w + x as usize] = ((s + 8) >> 4) as u8;
            }
        }
        out
    };
    pass(&pass(img, 1, 0), 0, 1)
}

pub fn next_size(w: usize, h: usize, scale: f32) -> (usize, usize) {
    (((w as f32 / scale).round() as usize).max(1), ((h as f32 / scale).round() as usize).max(1))
}

pub fn downscale(img: &Gray, scale: f32) -> Gray {
    let (w, h) = next_size(img.w, img.h, scale);
    let mut out = Gray::new(w, h);
    for y in 0..h {
        for x in 0..w {
            let sx = (x as f32 + 0.5) * scale - 0.5;
            let sy = (y as f32 + 0.5) * scale - 0.5;
            let (x0, y0) = (sx.floor(), sy.floor());
            let (fx, fy) = (sx - x0, sy - y0);
            let (x0, y0) = (x0 as isize, y0 as isize);
            let p = |dx, dy| img.at(x0 + dx, y0 + dy) as f32;
            let v = p(0, 0) * (1.0 - fx) * (1.0 - fy)
                + p(1, 0) * fx * (1.0 - fy)
                + p(0, 1) * (1.0 - fx) * fy
                + p(1, 1) * fx * fy;
            out.data[y * w + x] = (v + 0.5) as u8;
        }
    }
    out
}

pub fn pyramid(img: Gray, levels: usize, scale: f32) -> Vec<Gray> {
    let mut out = vec![img];
    for _ in 1..levels {
        let next = downscale(&blur(out.last().unwrap()), scale);
        out.push(next);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn luma_white_and_black() {
        let g = luma(&[255, 255, 255, 255, 0, 0, 0, 255], 2, 1);
        assert_eq!(g.data, vec![255, 0]);
    }

    #[test]
    fn blur_keeps_flat_image() {
        let mut g = Gray::new(8, 8);
        g.data.fill(100);
        assert_eq!(blur(&g).data, g.data);
    }

    #[test]
    fn pyramid_sizes() {
        let p = pyramid(Gray::new(720, 960), 4, 1.44);
        let sizes: Vec<_> = p.iter().map(|g| (g.w, g.h)).collect();
        assert_eq!(sizes, vec![(720, 960), (500, 667), (347, 463), (241, 322)]);
    }
}
