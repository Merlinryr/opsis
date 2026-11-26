//! WebGPU driver (wgpu, `webgpu` backend only: ~26 KB gzip vs ~670 KB with its WebGL backend).
//! Same fragment passes as `webgl`, written in WGSL; parity tests guard drift between the two.

use crate::gpu::{Gpu, Pass};
use crate::image::Gray;
use wasm_bindgen::JsValue;
use std::cell::RefCell;
use std::rc::Rc;
use web_sys::OffscreenCanvas;

const WGSL: &str = r#"
@group(0) @binding(0) var src: texture_2d<f32>;
@group(0) @binding(1) var src2: texture_2d<f32>;
struct U { size: vec2i, dir: vec2i, scale: f32, t: i32, border: i32, pad: i32 };
@group(0) @binding(2) var<uniform> u: U;

@vertex fn vs(@builtin(vertex_index) i: u32) -> @builtin(position) vec4f {
    let p = vec2f(f32((i << 1u) & 2u), f32(i & 2u));
    return vec4f(p * 2.0 - 1.0, 0.0, 1.0);
}
fn px(p: vec2i) -> i32 { return i32(textureLoad(src, clamp(p, vec2i(0), u.size - 1), 0).r * 255.0 + 0.5); }
fn gray(v: i32) -> vec4f { return vec4f(f32(v) / 255.0, 0.0, 0.0, 1.0); }

@fragment fn luma(@builtin(position) pos: vec4f) -> @location(0) vec4f {
    let c = vec3i(textureLoad(src, vec2i(pos.xy), 0).rgb * 255.0 + 0.5);
    return gray((77 * c.r + 150 * c.g + 29 * c.b) >> 8u);
}

@fragment fn blur(@builtin(position) pos: vec4f) -> @location(0) vec4f {
    let p = vec2i(pos.xy);
    let s = px(p - 2 * u.dir) + 4 * px(p - u.dir) + 6 * px(p) + 4 * px(p + u.dir) + px(p + 2 * u.dir);
    return gray((s + 8) >> 4u);
}

@fragment fn down(@builtin(position) pos: vec4f) -> @location(0) vec4f {
    let s = pos.xy * u.scale - 0.5;
    let b = floor(s); let f = s - b; let p = vec2i(b);
    let v = f32(px(p)) * (1.0 - f.x) * (1.0 - f.y) + f32(px(p + vec2i(1, 0))) * f.x * (1.0 - f.y)
          + f32(px(p + vec2i(0, 1))) * (1.0 - f.x) * f.y + f32(px(p + vec2i(1, 1))) * f.x * f.y;
    return gray(i32(v + 0.5));
}

var<private> R: array<vec2i, 16> = array<vec2i, 16>(
    vec2i(0,-3), vec2i(1,-3), vec2i(2,-2), vec2i(3,-1), vec2i(3,0), vec2i(3,1), vec2i(2,2), vec2i(1,3),
    vec2i(0,3), vec2i(-1,3), vec2i(-2,2), vec2i(-3,1), vec2i(-3,0), vec2i(-3,-1), vec2i(-2,-2), vec2i(-1,-3));
fn arc9(m: u32) -> bool {
    let a = m | (m << 16u); var acc = a;
    for (var i = 1u; i < 9u; i++) { acc &= a >> i; }
    return acc != 0u;
}
@fragment fn fast(@builtin(position) pos: vec4f) -> @location(0) vec4f {
    let p = vec2i(pos.xy); var s = 0;
    if (all(p >= vec2i(u.border)) && all(p < u.size - u.border)) {
        let c = px(p); var b = 0u; var d = 0u; var sb = 0; var sd = 0;
        for (var i = 0; i < 16; i++) {
            let v = px(p + R[i]) - c;
            if (v > u.t) { b |= 1u << u32(i); sb += v - u.t; }
            else if (v < -u.t) { d |= 1u << u32(i); sd += -v - u.t; }
        }
        let ab = arc9(b); let ad = arc9(d);
        if (ab && ad) { s = max(sb, sd); } else if (ab) { s = sb; } else if (ad) { s = sd; }
    }
    return vec4f(f32(s & 255), f32(s >> 8u), 0.0, 255.0) / 255.0;
}

fn sc(p: vec2i) -> i32 {
    let t = textureLoad(src, clamp(p, vec2i(0), u.size - 1), 0);
    return i32(t.r * 255.0 + 0.5) + 256 * i32(t.g * 255.0 + 0.5);
}
@fragment fn nms(@builtin(position) pos: vec4f) -> @location(0) vec4f {
    let p = vec2i(pos.xy); let s = sc(p);
    var keep = s > 0 && all(p > vec2i(0)) && all(p < u.size - 1);
    for (var dy = -1; dy <= 1; dy++) {
        for (var dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) { continue; }
            let n = sc(p + vec2i(dx, dy));
            let before = dy < 0 || (dy == 0 && dx < 0);
            keep = keep && select(s > n, s >= n, before);
        }
    }
    let t = textureLoad(src, p, 0);
    return vec4f(t.r, t.g, select(0.0, 1.0, keep), textureLoad(src2, p, 0).r);
}
"#;

pub struct Tex {
    t: wgpu::Texture,
    w: usize,
    h: usize,
    pool: Pool,
}

/// Textures freed by dropped `Tex`es, reused by size and format: the front end makes ~20 per frame.
type Pool = Rc<RefCell<Vec<wgpu::Texture>>>;

impl Drop for Tex {
    fn drop(&mut self) {
        let mut pool = self.pool.borrow_mut();
        pool.push(self.t.clone());
        if pool.len() > 64 {
            pool.remove(0); // sizes from an old screen orientation: let the oldest go
        }
    }
}

pub struct Wgpu {
    device: wgpu::Device,
    queue: wgpu::Queue,
    layout: wgpu::BindGroupLayout,
    pipes: [wgpu::RenderPipeline; 5],
    pool: Pool,
    /// Passes recorded since the last read, submitted together: one submit per pass was ~26 per frame.
    enc: RefCell<Option<wgpu::CommandEncoder>>,
    /// Last `read_all` wait split, ms: [submit -> GPU work done, -> buffers mapped, -> unpacked].
    pub wait_ms: std::cell::Cell<[f64; 3]>,
}

fn err(e: impl std::fmt::Display) -> JsValue {
    JsValue::from_str(&e.to_string())
}

const OUT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;

impl Wgpu {
    pub async fn new() -> Result<Self, JsValue> {
        let instance = wgpu::Instance::default();
        let adapter = instance.request_adapter(&Default::default()).await.map_err(err)?;
        let (device, queue) = adapter.request_device(&Default::default()).await.map_err(err)?;
        let tex_entry = |binding| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::FRAGMENT,
            ty: wgpu::BindingType::Texture {
                sample_type: wgpu::TextureSampleType::Float { filterable: false },
                view_dimension: wgpu::TextureViewDimension::D2,
                multisampled: false,
            },
            count: None,
        };
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: None,
            entries: &[
                tex_entry(0),
                tex_entry(1),
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
                    count: None,
                },
            ],
        });
        let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor { bind_group_layouts: &[Some(&layout)], ..Default::default() });
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: None, source: wgpu::ShaderSource::Wgsl(WGSL.into()) });
        let pipe = |fs: &str| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: None,
                layout: Some(&pl),
                vertex: wgpu::VertexState { module: &module, entry_point: Some("vs"), buffers: &[], compilation_options: Default::default() },
                fragment: Some(wgpu::FragmentState {
                    module: &module,
                    entry_point: Some(fs),
                    targets: &[Some(OUT.into())],
                    compilation_options: Default::default(),
                }),
                primitive: Default::default(),
                depth_stencil: None,
                multisample: Default::default(),
                multiview_mask: None,
                cache: None,
            })
        };
        let pipes = [pipe("luma"), pipe("blur"), pipe("down"), pipe("fast"), pipe("nms")];
        Ok(Self { device, queue, layout, pipes, pool: Pool::default(), enc: RefCell::default(), wait_ms: Default::default() })
    }

    fn texture(&self, w: usize, h: usize, format: wgpu::TextureFormat, data: Option<&[u8]>) -> Tex {
        let size = wgpu::Extent3d { width: w as u32, height: h as u32, depth_or_array_layers: 1 };
        let reuse = {
            let mut pool = self.pool.borrow_mut();
            pool.iter().position(|t| t.size() == size && t.format() == format).map(|i| pool.swap_remove(i))
        };
        let t = reuse.unwrap_or_else(|| self.device.create_texture(&wgpu::TextureDescriptor {
            label: None,
            size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage: wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_DST
                | wgpu::TextureUsages::COPY_SRC
                | wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        }));
        if let Some(data) = data {
            let bpp = (data.len() / (w * h)) as u32;
            self.queue.write_texture(
                t.as_image_copy(),
                data,
                wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(w as u32 * bpp), rows_per_image: None },
                size,
            );
        }
        Tex { t, w, h, pool: self.pool.clone() }
    }
}

impl Gpu for Wgpu {
    type Tex = Tex;

    fn size(t: &Tex) -> (usize, usize) {
        (t.w, t.h)
    }

    fn upload(&self, rgba: &[u8], w: usize, h: usize) -> Result<Tex, JsValue> {
        Ok(self.texture(w, h, wgpu::TextureFormat::Rgba8Unorm, Some(rgba)))
    }

    fn upload_gray(&self, g: &Gray) -> Result<Tex, JsValue> {
        Ok(self.texture(g.w, g.h, wgpu::TextureFormat::R8Unorm, Some(&g.data)))
    }

    fn upload_canvas(&self, c: &OffscreenCanvas) -> Result<Tex, JsValue> {
        let (w, h) = (c.width() as usize, c.height() as usize);
        let t = self.texture(w, h, wgpu::TextureFormat::Rgba8Unorm, None);
        self.queue.copy_external_image_to_texture(
            &wgpu::CopyExternalImageSourceInfo { source: wgpu::ExternalImageSource::OffscreenCanvas(c.clone()), origin: wgpu::Origin2d::ZERO, flip_y: false },
            wgpu::CopyExternalImageDestInfo {
                texture: &t.t,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
                color_space: wgpu::PredefinedColorSpace::Srgb,
                premultiplied_alpha: false,
            },
            wgpu::Extent3d { width: w as u32, height: h as u32, depth_or_array_layers: 1 },
        );
        Ok(t)
    }

    fn run(&self, pass: Pass, src: &Tex, src2: Option<&Tex>, w: usize, h: usize) -> Result<Tex, JsValue> {
        let out = self.texture(w, h, OUT, None);
        let (idx, dir, scale, t, border) = match pass {
            Pass::Luma => (0, [0, 0], 0.0, 0, 0),
            Pass::Blur(dx, dy) => (1, [dx, dy], 0.0, 0, 0),
            Pass::Down(s) => (2, [0, 0], s, 0, 0),
            Pass::Fast(t, b) => (3, [0, 0], 0.0, t as i32, b as i32),
            Pass::Nms => (4, [0, 0], 0.0, 0, 0),
        };
        // Matches WGSL struct U: size, dir, scale, t, border, pad.
        let words: [u32; 8] = [src.w as u32, src.h as u32, dir[0] as u32, dir[1] as u32, scale.to_bits(), t as u32, border as u32, 0];
        let ubuf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: 32,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let bytes: Vec<u8> = words.iter().flat_map(|w| w.to_le_bytes()).collect();
        self.queue.write_buffer(&ubuf, 0, &bytes);
        let v1 = src.t.create_view(&Default::default());
        let v2 = src2.unwrap_or(src).t.create_view(&Default::default());
        let group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &self.layout,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&v1) },
                wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::TextureView(&v2) },
                wgpu::BindGroupEntry { binding: 2, resource: ubuf.as_entire_binding() },
            ],
        });
        let view = out.t.create_view(&Default::default());
        let mut enc = self.enc.borrow_mut();
        let enc = enc.get_or_insert_with(|| self.device.create_command_encoder(&Default::default()));
        {
            let mut rp = enc.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: None,
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::BLACK), store: wgpu::StoreOp::Store },
                })],
                ..Default::default()
            });
            rp.set_pipeline(&self.pipes[idx]);
            rp.set_bind_group(0, &group, &[]);
            rp.draw(0..3, 0..1);
        }
        Ok(out)
    }

    async fn read(&self, t: &Tex) -> Result<Vec<u8>, JsValue> {
        Ok(self.read_all(std::slice::from_ref(t)).await?.remove(0))
    }

    /// One submit and one wait for all textures: each separate map_async is a full GPU round trip.
    async fn read_all(&self, ts: &[Tex]) -> Result<Vec<Vec<u8>>, JsValue> {
        let row = |t: &Tex| (t.w * 4).next_multiple_of(256); // WebGPU copy rows must be 256-byte aligned
        let mut enc = self.enc.take().unwrap_or_else(|| self.device.create_command_encoder(&Default::default()));
        let bufs: Vec<_> = ts
            .iter()
            .map(|t| {
                let buf = self.device.create_buffer(&wgpu::BufferDescriptor {
                    label: None,
                    size: (row(t) * t.h) as u64,
                    usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                    mapped_at_creation: false,
                });
                enc.copy_texture_to_buffer(
                    t.t.as_image_copy(),
                    wgpu::TexelCopyBufferInfo {
                        buffer: &buf,
                        layout: wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(row(t) as u32), rows_per_image: None },
                    },
                    wgpu::Extent3d { width: t.w as u32, height: t.h as u32, depth_or_array_layers: 1 },
                );
                buf
            })
            .collect();
        self.queue.submit([enc.finish()]);
        let t0 = js_sys::Date::now();
        let done = js_sys::Promise::new(&mut |resolve, _| {
            self.queue.on_submitted_work_done(move || {
                let _ = resolve.call0(&JsValue::NULL);
            })
        });
        let maps: js_sys::Array = bufs
            .iter()
            .map(|buf| {
                js_sys::Promise::new(&mut |resolve, reject| {
                    buf.map_async(wgpu::MapMode::Read, .., move |r| {
                        let f = if r.is_ok() { resolve } else { reject };
                        let _ = f.call0(&JsValue::NULL);
                    });
                })
            })
            .collect();
        wasm_bindgen_futures::JsFuture::from(done).await?;
        let t1 = js_sys::Date::now();
        wasm_bindgen_futures::JsFuture::from(js_sys::Promise::all(&maps)).await?;
        let t2 = js_sys::Date::now();
        let out = ts
            .iter()
            .zip(&bufs)
            .map(|(t, buf)| {
                let mapped = buf.get_mapped_range(..).map_err(err)?;
                let mut v = Vec::with_capacity(t.w * 4 * t.h);
                for r in mapped.chunks(row(t)) {
                    v.extend_from_slice(&r[..t.w * 4]);
                }
                Ok(v)
            })
            .collect();
        self.wait_ms.set([t1 - t0, t2 - t1, js_sys::Date::now() - t2]);
        out
    }
}
