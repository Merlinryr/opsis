//! WebAR core. Phase 1 of M01: CPU reference front end (ground truth for the GPU passes).

pub mod fast;
pub mod image;
pub mod orb;
pub mod readback;
#[cfg(target_arch = "wasm32")]
pub mod gpu;
#[cfg(target_arch = "wasm32")]
pub mod web;
#[cfg(target_arch = "wasm32")]
pub mod webgl;
#[cfg(target_arch = "wasm32")]
pub mod webgpu;

/// Front-end tunables: the single source for the CPU reference and the GPU passes.
#[derive(Clone, Copy, Debug)]
pub struct FrontEnd {
    pub levels: usize,
    pub scale: f32,
    pub fast_threshold: u8,
    pub max_keypoints: usize,
    pub grid: (usize, usize),
}

impl Default for FrontEnd {
    fn default() -> Self {
        // shortcut: starting values, not tuned on real sequences; revisit in M01 phase 5.
        Self { levels: 4, scale: 1.44, fast_threshold: 20, max_keypoints: 500, grid: (6, 8) }
    }
}
