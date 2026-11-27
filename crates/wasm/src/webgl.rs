//! WebGL2 version of the front-end passes. Fragment shaders only (WebGL2 has no compute shaders), and RGBA8 textures only (float textures need an extension some phones lack).

use crate::gpu::{Gpu, Pass};
use opsis_core::image::Gray;
use std::cell::RefCell;
use std::rc::Rc;
use wasm_bindgen::{JsCast, JsValue};
use web_sys::{HtmlCanvasElement, OffscreenCanvas, WebGl2RenderingContext as GL, WebGlProgram, WebGlTexture};

const VS: &str = "#version 300 es
void main() {
    vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}";

const HEAD: &str = "#version 300 es
precision highp float; precision highp int; precision highp sampler2D;
uniform sampler2D u_src; uniform sampler2D u_src2; uniform ivec2 u_size;
uniform ivec2 u_dir; uniform float u_scale; uniform int u_t; uniform int u_border;
out vec4 o;
ivec2 here() { return ivec2(gl_FragCoord.xy); }
int px(ivec2 p) { return int(texelFetch(u_src, clamp(p, ivec2(0), u_size - 1), 0).r * 255.0 + 0.5); }
vec4 gray(int v) { return vec4(float(v) / 255.0, 0.0, 0.0, 1.0); }
";

const LUMA: &str = "void main() {
    ivec3 c = ivec3(texelFetch(u_src, here(), 0).rgb * 255.0 + 0.5);
    o = gray((77 * c.r + 150 * c.g + 29 * c.b) >> 8);
}";

const BLUR: &str = "void main() {
    ivec2 p = here();
    int s = px(p - 2 * u_dir) + 4 * px(p - u_dir) + 6 * px(p) + 4 * px(p + u_dir) + px(p + 2 * u_dir);
    o = gray((s + 8) >> 4);
}";

const DOWN: &str = "void main() {
    vec2 s = gl_FragCoord.xy * u_scale - 0.5;
    vec2 b = floor(s); vec2 f = s - b; ivec2 p = ivec2(b);
    float v = float(px(p)) * (1.0 - f.x) * (1.0 - f.y) + float(px(p + ivec2(1, 0))) * f.x * (1.0 - f.y)
            + float(px(p + ivec2(0, 1))) * (1.0 - f.x) * f.y + float(px(p + ivec2(1, 1))) * f.x * f.y;
    o = gray(int(v + 0.5));
}";

const FAST: &str = "const ivec2 R[16] = ivec2[16](
    ivec2(0,-3), ivec2(1,-3), ivec2(2,-2), ivec2(3,-1), ivec2(3,0), ivec2(3,1), ivec2(2,2), ivec2(1,3),
    ivec2(0,3), ivec2(-1,3), ivec2(-2,2), ivec2(-3,1), ivec2(-3,0), ivec2(-3,-1), ivec2(-2,-2), ivec2(-1,-3));
bool arc9(uint m) {
    uint a = m | (m << 16u); uint acc = a;
    for (uint i = 1u; i < 9u; i++) acc &= a >> i;
    return acc != 0u;
}
void main() {
    ivec2 p = here(); int s = 0;
    if (all(greaterThanEqual(p, ivec2(u_border))) && all(lessThan(p, u_size - u_border))) {
        int c = px(p); uint b = 0u; uint d = 0u; int sb = 0; int sd = 0;
        for (int i = 0; i < 16; i++) {
            int v = px(p + R[i]) - c;
            if (v > u_t) { b |= 1u << uint(i); sb += v - u_t; }
            else if (v < -u_t) { d |= 1u << uint(i); sd += -v - u_t; }
        }
        bool ab = arc9(b); bool ad = arc9(d);
        s = ab && ad ? max(sb, sd) : ab ? sb : ad ? sd : 0;
    }
    o = vec4(float(s & 255), float(s >> 8), 0.0, 255.0) / 255.0;
}";

const NMS: &str = "int sc(ivec2 p) {
    vec4 t = texelFetch(u_src, clamp(p, ivec2(0), u_size - 1), 0);
    return int(t.r * 255.0 + 0.5) + 256 * int(t.g * 255.0 + 0.5);
}
void main() {
    ivec2 p = here(); int s = sc(p);
    bool keep = s > 0 && all(greaterThan(p, ivec2(0))) && all(lessThan(p, u_size - 1));
    for (int dy = -1; dy <= 1; dy++) for (int dx = -1; dx <= 1; dx++) {
        if (dx == 0 && dy == 0) continue;
        int n = sc(p + ivec2(dx, dy));
        bool before = dy < 0 || (dy == 0 && dx < 0);
        keep = keep && (before ? s >= n : s > n);
    }
    vec4 t = texelFetch(u_src, p, 0);
    o = vec4(t.r, t.g, keep ? 1.0 : 0.0, texelFetch(u_src2, p, 0).r);
}";

pub struct Tex {
    t: WebGlTexture,
    gl: GL,
    pub w: usize,
    pub h: usize,
    internal: u32,
    pool: Pool,
}

type Pool = Rc<RefCell<Vec<(WebGlTexture, usize, usize, u32)>>>;
const POOL_MAX: usize = 64; // Beyond this many (e.g. sizes from an old screen rotation), the oldest are freed.

impl Drop for Tex {
    fn drop(&mut self) {
        let mut pool = self.pool.borrow_mut();
        pool.push((self.t.clone(), self.w, self.h, self.internal));
        if pool.len() > POOL_MAX {
            self.gl.delete_texture(Some(&pool.remove(0).0)); // WebGL never frees textures by itself.
        }
    }
}

pub struct Gl {
    gl: GL,
    progs: [WebGlProgram; 5],
    pool: Pool,
}

fn err(m: impl Into<String>) -> JsValue {
    JsValue::from_str(&m.into())
}

impl Gl {
    pub fn new(canvas: &HtmlCanvasElement) -> Result<Self, JsValue> {
        let gl: GL = canvas.get_context("webgl2")?.ok_or_else(|| err("no webgl2"))?.dyn_into()?;
        let prog = |fs: &str| -> Result<WebGlProgram, JsValue> {
            let p = gl.create_program().ok_or_else(|| err("program"))?;
            for (kind, src) in [(GL::VERTEX_SHADER, VS.to_string()), (GL::FRAGMENT_SHADER, format!("{HEAD}{fs}"))] {
                let s = gl.create_shader(kind).ok_or_else(|| err("shader"))?;
                gl.shader_source(&s, &src);
                gl.compile_shader(&s);
                if !gl.get_shader_parameter(&s, GL::COMPILE_STATUS).as_bool().unwrap_or(false) {
                    return Err(err(gl.get_shader_info_log(&s).unwrap_or_default()));
                }
                gl.attach_shader(&p, &s);
            }
            gl.link_program(&p);
            if !gl.get_program_parameter(&p, GL::LINK_STATUS).as_bool().unwrap_or(false) {
                return Err(err(gl.get_program_info_log(&p).unwrap_or_default()));
            }
            Ok(p)
        };
        let progs = [prog(LUMA)?, prog(BLUR)?, prog(DOWN)?, prog(FAST)?, prog(NMS)?];
        let fbo = gl.create_framebuffer().ok_or_else(|| err("fbo"))?;
        gl.bind_framebuffer(GL::FRAMEBUFFER, Some(&fbo));
        gl.pixel_storei(GL::UNPACK_ALIGNMENT, 1);
        Ok(Self { gl, progs, pool: Pool::default() })
    }

    /// A `w`×`h` texture, left bound. Reused from the pool when one matches; its contents are then undefined unless `data` is given.
    fn texture(&self, w: usize, h: usize, internal: u32, format: u32, data: Option<&[u8]>) -> Result<Tex, JsValue> {
        let gl = &self.gl;
        let reuse = {
            let mut pool = self.pool.borrow_mut();
            pool.iter().position(|p| (p.1, p.2, p.3) == (w, h, internal)).map(|i| pool.swap_remove(i).0)
        };
        if let Some(t) = reuse {
            gl.bind_texture(GL::TEXTURE_2D, Some(&t));
            if data.is_some() {
                gl.tex_sub_image_2d_with_i32_and_i32_and_u32_and_type_and_opt_u8_array(GL::TEXTURE_2D, 0, 0, 0, w as i32, h as i32, format, GL::UNSIGNED_BYTE, data)?;
            }
            return Ok(Tex { t, gl: gl.clone(), w, h, internal, pool: self.pool.clone() });
        }
        let t = gl.create_texture().ok_or_else(|| err("texture"))?;
        gl.bind_texture(GL::TEXTURE_2D, Some(&t));
        for p in [GL::TEXTURE_MIN_FILTER, GL::TEXTURE_MAG_FILTER] {
            gl.tex_parameteri(GL::TEXTURE_2D, p, GL::NEAREST as i32);
        }
        gl.tex_image_2d_with_i32_and_i32_and_i32_and_format_and_type_and_opt_u8_array(
            GL::TEXTURE_2D, 0, internal as i32, w as i32, h as i32, 0, format, GL::UNSIGNED_BYTE, data,
        )?;
        Ok(Tex { t, gl: gl.clone(), w, h, internal, pool: self.pool.clone() })
    }
}

impl Gpu for Gl {
    type Tex = Tex;

    fn size(t: &Tex) -> (usize, usize) {
        (t.w, t.h)
    }

    fn upload(&self, rgba: &[u8], w: usize, h: usize) -> Result<Tex, JsValue> {
        self.texture(w, h, GL::RGBA8, GL::RGBA, Some(rgba))
    }

    fn upload_gray(&self, g: &Gray) -> Result<Tex, JsValue> {
        self.texture(g.w, g.h, GL::R8, GL::RED, Some(&g.data))
    }

    fn upload_canvas(&self, c: &OffscreenCanvas) -> Result<Tex, JsValue> {
        let t = self.texture(c.width() as usize, c.height() as usize, GL::RGBA8, GL::RGBA, None)?;
        self.gl.tex_sub_image_2d_with_u32_and_u32_and_offscreen_canvas(GL::TEXTURE_2D, 0, 0, 0, GL::RGBA, GL::UNSIGNED_BYTE, c)?; // Same size: no reallocation.
        Ok(t)
    }

    fn run(&self, pass: Pass, src: &Tex, src2: Option<&Tex>, w: usize, h: usize) -> Result<Tex, JsValue> {
        let gl = &self.gl;
        let out = self.texture(w, h, GL::RGBA8, GL::RGBA, None)?;
        gl.framebuffer_texture_2d(GL::FRAMEBUFFER, GL::COLOR_ATTACHMENT0, GL::TEXTURE_2D, Some(&out.t), 0);
        let prog = &self.progs[match pass {
            Pass::Luma => 0,
            Pass::Blur(..) => 1,
            Pass::Down(_) => 2,
            Pass::Fast(..) => 3,
            Pass::Nms => 4,
        }];
        gl.use_program(Some(prog));
        let loc = |n: &str| gl.get_uniform_location(prog, n);
        for (unit, tex) in [(0, Some(src)), (1, src2)] {
            if let Some(tex) = tex {
                gl.active_texture(GL::TEXTURE0 + unit as u32);
                gl.bind_texture(GL::TEXTURE_2D, Some(&tex.t));
            }
            gl.uniform1i(loc(if unit == 0 { "u_src" } else { "u_src2" }).as_ref(), unit);
        }
        gl.uniform2i(loc("u_size").as_ref(), src.w as i32, src.h as i32);
        match pass {
            Pass::Blur(dx, dy) => gl.uniform2i(loc("u_dir").as_ref(), dx, dy),
            Pass::Down(s) => gl.uniform1f(loc("u_scale").as_ref(), s),
            Pass::Fast(t, b) => {
                gl.uniform1i(loc("u_t").as_ref(), t as i32);
                gl.uniform1i(loc("u_border").as_ref(), b as i32);
            }
            Pass::Luma | Pass::Nms => {}
        }
        gl.viewport(0, 0, w as i32, h as i32);
        gl.draw_arrays(GL::TRIANGLES, 0, 3);
        Ok(out)
    }

    // shortcut: readPixels waits for the GPU. Upgrade: async read with a pixel buffer and fence (one frame of lag).
    async fn read(&self, t: &Tex) -> Result<Vec<u8>, JsValue> {
        let gl = &self.gl;
        gl.framebuffer_texture_2d(GL::FRAMEBUFFER, GL::COLOR_ATTACHMENT0, GL::TEXTURE_2D, Some(&t.t), 0);
        let mut out = vec![0u8; t.w * t.h * 4];
        gl.read_pixels_with_opt_u8_array(0, 0, t.w as i32, t.h as i32, GL::RGBA, GL::UNSIGNED_BYTE, Some(&mut out))?;
        Ok(out)
    }
}
