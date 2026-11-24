//! Backend seam for the front-end passes. Pass order and level sizes live here once;
//! `webgl` and `webgpu` only know how to run one pass and read a texture back.

use crate::FrontEnd;
use crate::image::{Gray, next_size};
use crate::readback::Level;
use wasm_bindgen::JsValue;

#[derive(Clone, Copy)]
pub enum Pass {
    Luma,
    Blur(i32, i32),
    Down(f32),
    Fast(u8, usize),
    /// Second input: blurred level, copied to alpha for descriptors.
    Nms,
}

#[allow(async_fn_in_trait)] // single-threaded wasm only
pub trait Gpu {
    type Tex;
    fn size(t: &Self::Tex) -> (usize, usize);
    /// Tightly packed RGBA8.
    fn upload(&self, rgba: &[u8], w: usize, h: usize) -> Result<Self::Tex, JsValue>;
    fn upload_gray(&self, g: &Gray) -> Result<Self::Tex, JsValue>;
    /// Run one pass into a new `w` x `h` RGBA8 texture.
    fn run(&self, pass: Pass, src: &Self::Tex, src2: Option<&Self::Tex>, w: usize, h: usize) -> Result<Self::Tex, JsValue>;
    /// Tightly packed RGBA8.
    async fn read(&self, t: &Self::Tex) -> Result<Vec<u8>, JsValue>;

    fn blur(&self, src: &Self::Tex) -> Result<Self::Tex, JsValue> {
        let (w, h) = Self::size(src);
        let tmp = self.run(Pass::Blur(1, 0), src, None, w, h)?;
        self.run(Pass::Blur(0, 1), &tmp, None, w, h)
    }

    /// Full front end on an RGBA frame.
    async fn front_end(&self, rgba: &[u8], w: usize, h: usize, cfg: &FrontEnd, border: usize) -> Result<Vec<Level>, JsValue> {
        let frame = self.upload(rgba, w, h)?;
        let mut level = self.run(Pass::Luma, &frame, None, w, h)?;
        let mut out = Vec::new();
        for l in 0..cfg.levels {
            let (lw, lh) = Self::size(&level);
            let blurred = self.blur(&level)?;
            let score = self.run(Pass::Fast(cfg.fast_threshold, border), &level, None, lw, lh)?;
            let nms = self.run(Pass::Nms, &score, Some(&blurred), lw, lh)?;
            out.push(Level { w: lw, h: lh, rgba: self.read(&nms).await? });
            if l + 1 < cfg.levels {
                let (nw, nh) = next_size(lw, lh, cfg.scale);
                level = self.run(Pass::Down(cfg.scale), &blurred, None, nw, nh)?;
            }
        }
        Ok(out)
    }
}
