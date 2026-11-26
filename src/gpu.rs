//! Backend seam for the front-end passes. Pass order and level sizes live here once;
//! `webgl` and `webgpu` only know how to run one pass and read a texture back.

use crate::FrontEnd;
use crate::image::{Gray, next_size};
use crate::readback::Level;
use wasm_bindgen::JsValue;
use web_sys::OffscreenCanvas;

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
    /// The canvas pixels as RGBA8, copied on the GPU (no getImageData round trip through the CPU).
    fn upload_canvas(&self, c: &OffscreenCanvas) -> Result<Self::Tex, JsValue>;
    /// Run one pass into a new `w` x `h` RGBA8 texture.
    fn run(&self, pass: Pass, src: &Self::Tex, src2: Option<&Self::Tex>, w: usize, h: usize) -> Result<Self::Tex, JsValue>;
    /// Tightly packed RGBA8.
    async fn read(&self, t: &Self::Tex) -> Result<Vec<u8>, JsValue>;
    /// Several textures in one GPU round trip where the backend can (WebGPU); WebGL reads are synchronous anyway.
    async fn read_all(&self, ts: &[Self::Tex]) -> Result<Vec<Vec<u8>>, JsValue> {
        let mut out = Vec::new();
        for t in ts {
            out.push(self.read(t).await?);
        }
        Ok(out)
    }

    fn blur(&self, src: &Self::Tex) -> Result<Self::Tex, JsValue> {
        let (w, h) = Self::size(src);
        let tmp = self.run(Pass::Blur(1, 0), src, None, w, h)?;
        self.run(Pass::Blur(0, 1), &tmp, None, w, h)
    }

    /// Full front end on an RGBA frame texture, and the ms spent encoding and submitting the passes
    /// (the rest of the call is waiting for the GPU and the readback).
    async fn front_end(&self, frame: &Self::Tex, cfg: &FrontEnd, border: usize) -> Result<(Vec<Level>, f64), JsValue> {
        let t0 = js_sys::Date::now();
        let (w, h) = Self::size(frame);
        let mut level = self.run(Pass::Luma, frame, None, w, h)?;
        let mut nms = Vec::new();
        for l in 0..cfg.levels {
            let (lw, lh) = Self::size(&level);
            let blurred = self.blur(&level)?;
            let score = self.run(Pass::Fast(cfg.fast_threshold, border), &level, None, lw, lh)?;
            nms.push(self.run(Pass::Nms, &score, Some(&blurred), lw, lh)?);
            if l + 1 < cfg.levels {
                let (nw, nh) = next_size(lw, lh, cfg.scale);
                level = self.run(Pass::Down(cfg.scale), &blurred, None, nw, nh)?;
            }
        }
        let encode = js_sys::Date::now() - t0;
        let rgba = self.read_all(&nms).await?;
        Ok((nms.iter().zip(rgba).map(|(t, rgba)| { let (w, h) = Self::size(t); Level { w, h, rgba } }).collect(), encode))
    }
}
