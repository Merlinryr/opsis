use opsis_core::FrontEnd;
use opsis_core::fast;
use crate::gpu::{Gpu, Pass};
use opsis_core::image::{self, Gray};
use opsis_core::orb::{BORDER, hamming};
use opsis_core::readback::{self, Level};
use crate::{webgl::Gl, webgpu::Wgpu};
use wasm_bindgen::prelude::*;
use web_sys::{HtmlCanvasElement, OffscreenCanvas};

pub fn test_frame(w: usize, h: usize) -> Vec<u8> {
    let mut s: u32 = 7;
    let noise: Vec<u8> = (0..(w / 8 + 2) * (h / 8 + 2) * 3)
        .map(|_| {
            s = s.wrapping_mul(1_103_515_245).wrapping_add(12345);
            (s >> 24) as u8
        })
        .collect();
    let mut out = vec![255u8; w * h * 4];
    for y in 0..h {
        for x in 0..w {
            let cell = ((y / 8) * (w / 8 + 2) + x / 8) * 3;
            let sq = (x / 40 + y / 40) % 3 == 0 && x % 40 > 10 && y % 40 > 10;
            for c in 0..3 {
                out[(y * w + x) * 4 + c] = if sq { 30 + 60 * c as u8 } else { noise[cell + c] };
            }
        }
    }
    out
}

fn max_diff(gpu_rgba: &[u8], cpu: &Gray) -> u8 {
    cpu.data.iter().zip(gpu_rgba.chunks_exact(4)).map(|(c, g)| c.abs_diff(g[0])).max().unwrap_or(0)
}

fn set_mismatches(mut a: Vec<(usize, usize)>, mut b: Vec<(usize, usize)>) -> usize {
    a.sort();
    b.sort();
    a.len().abs_diff(b.len()) + a.iter().filter(|p| b.binary_search(p).is_err()).count()
}

#[wasm_bindgen]
pub async fn parity_webgl(canvas: HtmlCanvasElement) -> Result<String, JsValue> {
    parity(&Gl::new(&canvas)?, "webgl2").await
}

#[wasm_bindgen]
pub async fn parity_webgpu() -> Result<String, JsValue> {
    parity(&Wgpu::new().await?, "webgpu").await
}

/// Checks each GPU pass against the CPU version. Each pass gets the CPU result of the previous one, so one rounding difference can't snowball. Then the full chain is compared down to descriptors. The report's first line is PASS or FAIL.
async fn parity<G: Gpu>(gl: &G, name: &str) -> Result<String, JsValue> {
    let (w, h, cfg) = (320, 240, FrontEnd::default());
    let rgba = test_frame(w, h);
    let cpu_levels = image::pyramid(image::luma(&rgba, w, h), cfg.levels, cfg.scale);
    let mut lines = Vec::new();
    let mut ok = true;
    let mut check = |name: String, bad: u64, limit: u64| {
        ok &= bad <= limit;
        lines.push(format!("{} {name}: {bad} (limit {limit})", if bad <= limit { "ok  " } else { "FAIL" }));
    };

    // Goes through a canvas like the camera page does, to test the canvas upload too.
    let canvas = OffscreenCanvas::new(w as u32, h as u32)?;
    let ctx: web_sys::OffscreenCanvasRenderingContext2d = canvas.get_context("2d")?.ok_or("no 2d context")?.dyn_into()?;
    ctx.put_image_data(&web_sys::ImageData::new_with_u8_clamped_array(wasm_bindgen::Clamped(&rgba), w as u32)?, 0.0, 0.0)?;
    let frame = gl.upload_canvas(&canvas)?;
    let luma = gl.read(&gl.run(Pass::Luma, &frame, None, w, h)?).await?;
    check("luma max diff".into(), max_diff(&luma, &cpu_levels[0]) as u64, 0);

    for (l, lvl) in cpu_levels.iter().enumerate() {
        let tex = gl.upload_gray(lvl)?;
        let cpu_blur = image::blur(lvl);
        check(format!("L{l} blur max diff"), max_diff(&gl.read(&gl.blur(&tex)?).await?, &cpu_blur) as u64, 0);

        if l + 1 < cfg.levels {
            let cpu_down = image::downscale(&cpu_blur, cfg.scale);
            let t = gl.upload_gray(&cpu_blur)?;
            let gpu = gl.read(&gl.run(Pass::Down(cfg.scale), &t, None, cpu_down.w, cpu_down.h)?).await?;
            check(format!("L{l} downscale max diff"), max_diff(&gpu, &cpu_down) as u64, 1);
        }

        let cpu_score = fast::score_map(lvl, cfg.fast_threshold, BORDER);
        let gpu = gl.read(&gl.run(Pass::Fast(cfg.fast_threshold, BORDER), &tex, None, lvl.w, lvl.h)?).await?;
        let gpu_score = Level { w: lvl.w, h: lvl.h, rgba: gpu }.score();
        let bad = cpu_score.iter().zip(&gpu_score).filter(|(a, b)| a != b).count();
        check(format!("L{l} FAST score mismatches"), bad as u64, 0);

        let cpu_nms = fast::nms(&cpu_score, lvl.w, lvl.h);
        let packed = Level::from_cpu(&cpu_score, &[], &cpu_blur);
        let st = gl.upload(&packed.rgba, lvl.w, lvl.h)?;
        let bt = gl.upload_gray(&cpu_blur)?;
        let gpu = Level { w: lvl.w, h: lvl.h, rgba: gl.read(&gl.run(Pass::Nms, &st, Some(&bt), lvl.w, lvl.h)?).await? };
        check(format!("L{l} NMS mismatches ({} kps)", cpu_nms.len()), set_mismatches(cpu_nms, gpu.survivors()) as u64, 0);
        let alpha = gpu.blurred().data.iter().zip(&cpu_blur.data).filter(|(a, b)| a != b).count();
        check(format!("L{l} NMS alpha (blurred) mismatches"), alpha as u64, 0);
    }

    // A ±1 difference in downscaling can move a few coarse-level corners, so allow 1% of keypoints.
    let gravity = |_: &fast::Keypoint| Some(std::f32::consts::FRAC_PI_2);
    let (gk, gd) = readback::features(&gl.front_end(&frame, &cfg, BORDER).await?.0, &cfg, gravity);
    let (ck, cd) = readback::features(&readback::cpu_levels(&cpu_levels, &cfg, BORDER), &cfg, gravity);
    let key = |k: &fast::Keypoint| (k.level, (k.x * 64.0).round() as i32, (k.y * 64.0).round() as i32);
    let mut cpu_map: Vec<_> = ck.iter().map(key).zip(&cd).collect();
    cpu_map.sort_by_key(|e| e.0);
    let (mut kp_bad, mut desc_bad) = (ck.len().abs_diff(gk.len()), 0);
    for (k, d) in gk.iter().zip(&gd) {
        match cpu_map.binary_search_by_key(&key(k), |e| e.0) {
            Ok(i) => desc_bad += (hamming(d, cpu_map[i].1) != 0) as usize,
            Err(_) => kp_bad += 1,
        }
    }
    let limit = (ck.len() / 100) as u64;
    check(format!("end-to-end keypoint mismatches ({} kps)", ck.len()), kp_bad as u64, limit);
    check("end-to-end descriptor mismatches".into(), desc_bad as u64, limit);

    Ok(format!("{} {name}\n{}", if ok { "PASS" } else { "FAIL" }, lines.join("\n")))
}

