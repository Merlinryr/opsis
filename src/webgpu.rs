//! WebGPU driver (wgpu, `webgpu` backend only: ~26 KB gzip vs ~670 KB with its WebGL backend).
//! Same fragment passes as `webgl`, written in WGSL; parity tests guard drift between the two.

use crate::gpu::{Gpu, Pass};
use crate::image::Gray;
use wasm_bindgen::JsValue;

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
}

pub struct Wgpu {
    device: wgpu::Device,
    queue: wgpu::Queue,
    layout: wgpu::BindGroupLayout,
    pipes: [wgpu::RenderPipeline; 5],
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
        Ok(Self { device, queue, layout, pipes })
    }

    // shortcut: allocates a texture per call; pool and reuse per level before per-frame use.
    fn texture(&self, w: usize, h: usize, format: wgpu::TextureFormat, data: Option<&[u8]>) -> Tex {
        let size = wgpu::Extent3d { width: w as u32, height: h as u32, depth_or_array_layers: 1 };
        let t = self.device.create_texture(&wgpu::TextureDescriptor {
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
        });
        if let Some(data) = data {
            let bpp = (data.len() / (w * h)) as u32;
            self.queue.write_texture(
                t.as_image_copy(),
                data,
                wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(w as u32 * bpp), rows_per_image: None },
                size,
            );
        }
        Tex { t, w, h }
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
        let mut enc = self.device.create_command_encoder(&Default::default());
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
        self.queue.submit([enc.finish()]);
        Ok(out)
    }

    async fn read(&self, t: &Tex) -> Result<Vec<u8>, JsValue> {
        let row = (t.w * 4).next_multiple_of(256); // WebGPU copy rows must be 256-byte aligned
        let buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: (row * t.h) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut enc = self.device.create_command_encoder(&Default::default());
        enc.copy_texture_to_buffer(
            t.t.as_image_copy(),
            wgpu::TexelCopyBufferInfo {
                buffer: &buf,
                layout: wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(row as u32), rows_per_image: None },
            },
            wgpu::Extent3d { width: t.w as u32, height: t.h as u32, depth_or_array_layers: 1 },
        );
        self.queue.submit([enc.finish()]);
        let done = js_sys::Promise::new(&mut |resolve, reject| {
            buf.map_async(wgpu::MapMode::Read, .., move |r| {
                let f = if r.is_ok() { resolve } else { reject };
                let _ = f.call0(&JsValue::NULL);
            });
        });
        wasm_bindgen_futures::JsFuture::from(done).await?;
        let mapped = buf.get_mapped_range(..).map_err(err)?;
        Ok(mapped.chunks(row).flat_map(|r| &r[..t.w * 4]).copied().collect())
    }
}
