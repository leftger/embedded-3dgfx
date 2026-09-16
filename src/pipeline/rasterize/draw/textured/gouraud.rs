//! Gouraud-shaded textured triangles (per-vertex colour interpolation).

use core::fmt::Debug;
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
use embedded_graphics_core::prelude::Point;

use super::{interpolate_uv, should_skip_stipple};
use crate::pipeline::effects::{DitherConfig, FogConfig};
#[cfg(feature = "lighting")]
use crate::pipeline::rasterize::draw::fill::interpolate_color;
use crate::pipeline::shade::retro::palette::PaletteMode;
use crate::pipeline::shade::retro::stipple::StippleMode;
use crate::pipeline::shade::retro::texture_lod::TextureMapping;
use crate::pipeline::shade::retro::tint::ScreenTint;

pub fn fill_triangle_zbuffered_textured_gouraud<D: DrawTarget<Color = Rgb565>>(
    p1: nalgebra::Point2<i32>,
    p2: nalgebra::Point2<i32>,
    p3: nalgebra::Point2<i32>,
    z1: f32,
    z2: f32,
    z3: f32,
    w1: f32,
    w2: f32,
    w3: f32,
    uv1: [f32; 2],
    uv2: [f32; 2],
    uv3: [f32; 2],
    c1: Rgb565,
    c2: Rgb565,
    c3: Rgb565,
    texture: &crate::pipeline::rasterize::texture::Texture,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    width: usize,
    fog_config: Option<&FogConfig>,
    dither_config: Option<&DitherConfig>,
    texture_mapping: TextureMapping,
    stipple_mode: StippleMode,
    screen_tint: Option<ScreenTint>,
    palette_mode: PaletteMode,
) where
    <D as DrawTarget>::Error: Debug,
{
    let p1_eg = Point::new(p1.x, p1.y);
    let p2_eg = Point::new(p2.x, p2.y);
    let p3_eg = Point::new(p3.x, p3.y);

    let z1_int = (z1 * 65536.0) as u32;
    let z2_int = (z2 * 65536.0) as u32;
    let z3_int = (z3 * 65536.0) as u32;

    if p2_eg.y == p3_eg.y {
        fill_bottom_flat_triangle_zbuffered_textured_gouraud(
            p1_eg,
            p2_eg,
            p3_eg,
            z1_int,
            z2_int,
            z3_int,
            w1,
            w2,
            w3,
            uv1,
            uv2,
            uv3,
            c1,
            c2,
            c3,
            texture,
            fb,
            zbuffer,
            width,
            fog_config,
            dither_config,
            texture_mapping,
            stipple_mode,
            screen_tint,
            palette_mode,
        );
    } else if p1_eg.y == p2_eg.y {
        fill_top_flat_triangle_zbuffered_textured_gouraud(
            p1_eg,
            p2_eg,
            p3_eg,
            z1_int,
            z2_int,
            z3_int,
            w1,
            w2,
            w3,
            uv1,
            uv2,
            uv3,
            c1,
            c2,
            c3,
            texture,
            fb,
            zbuffer,
            width,
            fog_config,
            dither_config,
            texture_mapping,
            stipple_mode,
            screen_tint,
            palette_mode,
        );
    } else {
        let t = (p2_eg.y - p1_eg.y) as f32 / (p3_eg.y - p1_eg.y) as f32;
        let split_x = (p1_eg.x as f32 + t * (p3_eg.x - p1_eg.x) as f32) as i32;
        let p_split = Point::new(split_x, p2_eg.y);

        let z_split = (z1_int as f64 + (z3_int as f64 - z1_int as f64) * t as f64) as u32;
        let w_split = w1 + t * (w3 - w1);
        let uv_split = [
            uv1[0] + t * (uv3[0] - uv1[0]),
            uv1[1] + t * (uv3[1] - uv1[1]),
        ];
        let color_split = interpolate_color(c1, c3, t);

        fill_bottom_flat_triangle_zbuffered_textured_gouraud(
            p1_eg,
            p2_eg,
            p_split,
            z1_int,
            z2_int,
            z_split,
            w1,
            w2,
            w_split,
            uv1,
            uv2,
            uv_split,
            c1,
            c2,
            color_split,
            texture,
            fb,
            zbuffer,
            width,
            fog_config,
            dither_config,
            texture_mapping,
            stipple_mode,
            screen_tint,
            palette_mode,
        );

        fill_top_flat_triangle_zbuffered_textured_gouraud(
            p2_eg,
            p_split,
            p3_eg,
            z2_int,
            z_split,
            z3_int,
            w2,
            w_split,
            w3,
            uv2,
            uv_split,
            uv3,
            c2,
            color_split,
            c3,
            texture,
            fb,
            zbuffer,
            width,
            fog_config,
            dither_config,
            texture_mapping,
            stipple_mode,
            screen_tint,
            palette_mode,
        );
    }
}

fn fill_bottom_flat_triangle_zbuffered_textured_gouraud<D: DrawTarget<Color = Rgb565>>(
    p1: Point,
    p2: Point,
    p3: Point,
    z1: u32,
    z2: u32,
    z3: u32,
    w1: f32,
    w2: f32,
    w3: f32,
    uv1: [f32; 2],
    uv2: [f32; 2],
    uv3: [f32; 2],
    c1: Rgb565,
    c2: Rgb565,
    c3: Rgb565,
    texture: &crate::pipeline::rasterize::texture::Texture,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    width: usize,
    fog_config: Option<&FogConfig>,
    dither_config: Option<&DitherConfig>,
    texture_mapping: TextureMapping,
    stipple_mode: StippleMode,
    screen_tint: Option<ScreenTint>,
    palette_mode: PaletteMode,
) where
    <D as DrawTarget>::Error: Debug,
{
    let height = p2.y - p1.y;
    if height == 0 {
        return;
    }

    let invslope1 = ((p2.x - p1.x) << 16) / height;
    let invslope2 = ((p3.x - p1.x) << 16) / height;

    let mut curx1 = p1.x << 16;
    let mut curx2 = p1.x << 16;

    for scanline_y in p1.y..=p2.y {
        let dy = scanline_y - p1.y;
        let t = dy as f32 / height as f32;

        let z_left = if height > 0 {
            (z1 as i64 + ((z2 as i64 - z1 as i64) * dy as i64 / height as i64)) as u32
        } else {
            z1
        };
        let z_right = if height > 0 {
            (z1 as i64 + ((z3 as i64 - z1 as i64) * dy as i64 / height as i64)) as u32
        } else {
            z1
        };

        let w_left = w1 + t * (w2 - w1);
        let w_right = w1 + t * (w3 - w1);

        let uv_left = [
            uv1[0] + t * (uv2[0] - uv1[0]),
            uv1[1] + t * (uv2[1] - uv1[1]),
        ];
        let uv_right = [
            uv1[0] + t * (uv3[0] - uv1[0]),
            uv1[1] + t * (uv3[1] - uv1[1]),
        ];

        let color_left = interpolate_color(c1, c2, t);
        let color_right = interpolate_color(c1, c3, t);

        draw_scanline_zbuffered_textured_gouraud(
            curx1 >> 16,
            curx2 >> 16,
            scanline_y,
            z_left,
            z_right,
            w_left,
            w_right,
            uv_left,
            uv_right,
            color_left,
            color_right,
            texture,
            fb,
            zbuffer,
            width,
            fog_config,
            dither_config,
            texture_mapping,
            stipple_mode,
            screen_tint,
            palette_mode,
        );

        curx1 += invslope1;
        curx2 += invslope2;
    }
}

fn fill_top_flat_triangle_zbuffered_textured_gouraud<D: DrawTarget<Color = Rgb565>>(
    p1: Point,
    p2: Point,
    p3: Point,
    z1: u32,
    z2: u32,
    z3: u32,
    w1: f32,
    w2: f32,
    w3: f32,
    uv1: [f32; 2],
    uv2: [f32; 2],
    uv3: [f32; 2],
    c1: Rgb565,
    c2: Rgb565,
    c3: Rgb565,
    texture: &crate::pipeline::rasterize::texture::Texture,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    width: usize,
    fog_config: Option<&FogConfig>,
    dither_config: Option<&DitherConfig>,
    texture_mapping: TextureMapping,
    stipple_mode: StippleMode,
    screen_tint: Option<ScreenTint>,
    palette_mode: PaletteMode,
) where
    <D as DrawTarget>::Error: Debug,
{
    let height = p3.y - p1.y;
    if height == 0 {
        return;
    }

    let invslope1 = ((p3.x - p1.x) << 16) / height;
    let invslope2 = ((p3.x - p2.x) << 16) / height;

    let mut curx1 = p3.x << 16;
    let mut curx2 = p3.x << 16;

    for scanline_y in (p1.y..=p3.y).rev() {
        let dy = p3.y - scanline_y;
        let t = dy as f32 / height as f32;

        let z_left = if height > 0 {
            (z3 as i64 + ((z1 as i64 - z3 as i64) * dy as i64 / height as i64)) as u32
        } else {
            z3
        };
        let z_right = if height > 0 {
            (z3 as i64 + ((z2 as i64 - z3 as i64) * dy as i64 / height as i64)) as u32
        } else {
            z3
        };

        let w_left = w3 + t * (w1 - w3);
        let w_right = w3 + t * (w2 - w3);

        let uv_left = [
            uv3[0] + t * (uv1[0] - uv3[0]),
            uv3[1] + t * (uv1[1] - uv3[1]),
        ];
        let uv_right = [
            uv3[0] + t * (uv2[0] - uv3[0]),
            uv3[1] + t * (uv2[1] - uv3[1]),
        ];

        let color_left = interpolate_color(c3, c1, t);
        let color_right = interpolate_color(c3, c2, t);

        draw_scanline_zbuffered_textured_gouraud(
            curx1 >> 16,
            curx2 >> 16,
            scanline_y,
            z_left,
            z_right,
            w_left,
            w_right,
            uv_left,
            uv_right,
            color_left,
            color_right,
            texture,
            fb,
            zbuffer,
            width,
            fog_config,
            dither_config,
            texture_mapping,
            stipple_mode,
            screen_tint,
            palette_mode,
        );

        curx1 -= invslope1;
        curx2 -= invslope2;
    }
}

fn draw_scanline_zbuffered_textured_gouraud<D: DrawTarget<Color = Rgb565>>(
    x1: i32,
    x2: i32,
    y: i32,
    z1: u32,
    z2: u32,
    w1: f32,
    w2: f32,
    uv1: [f32; 2],
    uv2: [f32; 2],
    color1: Rgb565,
    color2: Rgb565,
    texture: &crate::pipeline::rasterize::texture::Texture,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    width: usize,
    fog_config: Option<&FogConfig>,
    dither_config: Option<&DitherConfig>,
    texture_mapping: TextureMapping,
    stipple_mode: StippleMode,
    screen_tint: Option<ScreenTint>,
    palette_mode: PaletteMode,
) where
    <D as DrawTarget>::Error: Debug,
{
    if y < 0 {
        return;
    }
    let height = zbuffer.len() / width;
    if y as usize >= height {
        return;
    }

    let (
        left_x,
        right_x,
        z_left,
        z_right,
        w_left,
        w_right,
        uv_left,
        uv_right,
        color_left,
        color_right,
    ) = if x1 <= x2 {
        (x1, x2, z1, z2, w1, w2, uv1, uv2, color1, color2)
    } else {
        (x2, x1, z2, z1, w2, w1, uv2, uv1, color2, color1)
    };

    let start_x = left_x.max(0);
    let end_x = right_x.min(width as i32 - 1);
    if start_x > end_x {
        return;
    }

    let span = right_x - left_x;
    let inv_span = if span > 0 { 1.0 / span as f32 } else { 0.0 };
    let z_step = if span > 0 {
        (((z_right as i64 - z_left as i64) << 16) / span as i64) as i32
    } else {
        0
    };

    let left_clip = start_x - left_x;
    let mut z_curr = ((z_left as i64) << 16) + (left_clip as i64 * z_step as i64);
    let mut zbuf_idx = y as usize * width + start_x as usize;

    const SUB_SPAN_SIZE: i32 = 16;
    let mut span_x = start_x;

    while span_x <= end_x {
        let next_span_x = (span_x + SUB_SPAN_SIZE).min(end_x + 1);
        let span_len = next_span_x - span_x;

        let t_start = (span_x - left_x) as f32 * inv_span;
        let t_end = (next_span_x - 1 - left_x) as f32 * inv_span;

        let [u_start, v_start] =
            interpolate_uv(t_start, w_left, w_right, uv_left, uv_right, texture_mapping);
        let [u_end, v_end] =
            interpolate_uv(t_end, w_left, w_right, uv_left, uv_right, texture_mapping);

        let inv_sub = if span_len > 1 {
            1.0 / (span_len - 1) as f32
        } else {
            0.0
        };

        let du = (u_end - u_start) * inv_sub;
        let dv = (v_end - v_start) * inv_sub;

        let mut curr_u = u_start;
        let mut curr_v = v_start;

        for x in span_x..next_span_x {
            if should_skip_stipple(x, y, stipple_mode) {
                z_curr += z_step as i64;
                zbuf_idx += 1;
                curr_u += du;
                curr_v += dv;
                continue;
            }

            let z = (z_curr >> 16) as u32;
            z_curr += z_step as i64;
            let z_depth = crate::to_zdepth(z);

            if z_depth < zbuffer[zbuf_idx].saturating_add(crate::DEPTH_EPSILON) {
                zbuffer[zbuf_idx] = z_depth;

                let tx = (x - left_x) as f32 * inv_span;
                let c_interp = interpolate_color(color_left, color_right, tx);
                let tex_color = texture.sample(curr_u, curr_v);

                let r = ((tex_color.r() as u16 * c_interp.r() as u16) / 31) as u8;
                let g = ((tex_color.g() as u16 * c_interp.g() as u16) / 63) as u8;
                let b_val = ((tex_color.b() as u16 * c_interp.b() as u16) / 31) as u8;
                let mut final_color = Rgb565::new(r, g, b_val);

                if let Some(fog) = fog_config {
                    final_color = fog.apply(final_color, z);
                }
                if let Some(dither) = dither_config {
                    final_color = dither.apply(final_color, x, y);
                }
                if let Some(tint) = screen_tint {
                    final_color = tint.apply(final_color);
                }
                final_color = palette_mode.apply(final_color);

                fb.draw_iter([embedded_graphics_core::Pixel(Point::new(x, y), final_color)])
                    .unwrap();
            }
            zbuf_idx += 1;
            curr_u += du;
            curr_v += dv;
        }

        span_x = next_span_x;
    }
}
