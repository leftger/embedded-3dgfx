//! Lightmapped draws: a surface texture modulated by a baked lightmap.

//! Textured, lightmapped, and BSP coverage triangle rasterization.

use core::fmt::Debug;
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
use embedded_graphics_core::prelude::Point;

use super::{interpolate_uv, should_skip_stipple};
use crate::pipeline::rasterize::draw::state::RasterState;

pub fn draw_zbuffered_lightmapped<D: DrawTarget<Color = Rgb565>, const N: usize>(
    points: [nalgebra::Point2<i32>; 3],
    depths: [f32; 3],
    ws: [f32; 3],
    surface_uvs: [[f32; 2]; 3],
    lm_uvs: [[f32; 2]; 3],
    texture_id: u32,
    lightmap_id: u32,
    brightness: u8,
    dynamic_tint: Rgb565,
    texture_manager: &crate::pipeline::rasterize::texture::TextureManager<N>,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    state: &RasterState<'_>,
) where
    <D as DrawTarget>::Error: Debug,
{
    draw_zbuffered_lightmapped_mapped(
        points,
        depths,
        ws,
        surface_uvs,
        lm_uvs,
        texture_id,
        lightmap_id,
        brightness,
        dynamic_tint,
        texture_manager,
        fb,
        zbuffer,
        state,
    );
}

pub fn draw_zbuffered_lightmapped_mapped<D: DrawTarget<Color = Rgb565>, const N: usize>(
    mut points: [nalgebra::Point2<i32>; 3],
    mut depths: [f32; 3],
    mut ws: [f32; 3],
    mut surface_uvs: [[f32; 2]; 3],
    mut lm_uvs: [[f32; 2]; 3],
    texture_id: u32,
    lightmap_id: u32,
    brightness: u8,
    dynamic_tint: Rgb565,
    texture_manager: &crate::pipeline::rasterize::texture::TextureManager<N>,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    state: &RasterState<'_>,
) where
    <D as DrawTarget>::Error: Debug,
{
    let surf = match texture_manager.get(texture_id) {
        Some(t) => t,
        None => return,
    };
    let lm = if lightmap_id == u32::MAX {
        None
    } else {
        texture_manager.get(lightmap_id)
    };

    macro_rules! swap_all {
        ($i:expr, $j:expr) => {
            points.swap($i, $j);
            depths.swap($i, $j);
            ws.swap($i, $j);
            surface_uvs.swap($i, $j);
            lm_uvs.swap($i, $j);
        };
    }
    if points[0].y > points[1].y {
        swap_all!(0, 1);
    }
    if points[0].y > points[2].y {
        swap_all!(0, 2);
    }
    if points[1].y > points[2].y {
        swap_all!(1, 2);
    }

    let [p1, p2, p3] = points;
    let [z1, z2, z3] = depths;
    let [w1, w2, w3] = ws;
    let [uv1, uv2, uv3] = surface_uvs;
    let [luv1, luv2, luv3] = lm_uvs;

    let scr_w = state.width as i32;
    let scr_h = (zbuffer.len() / state.width) as i32;
    if p1.x < 0 && p2.x < 0 && p3.x < 0 {
        return;
    }
    if p1.x >= scr_w && p2.x >= scr_w && p3.x >= scr_w {
        return;
    }
    if p1.y < 0 && p2.y < 0 && p3.y < 0 {
        return;
    }
    if p1.y >= scr_h && p2.y >= scr_h && p3.y >= scr_h {
        return;
    }

    let z1_int = (z1 * 65536.0) as u32;
    let z2_int = (z2 * 65536.0) as u32;
    let z3_int = (z3 * 65536.0) as u32;

    if p2.y == p3.y {
        fill_lm_bottom_flat(
            p1,
            p2,
            p3,
            z1_int,
            z2_int,
            z3_int,
            w1,
            w2,
            w3,
            uv1,
            uv2,
            uv3,
            luv1,
            luv2,
            luv3,
            dynamic_tint,
            surf,
            lm,
            fb,
            zbuffer,
            state,
            brightness,
        );
    } else if p1.y == p2.y {
        fill_lm_top_flat(
            p1,
            p2,
            p3,
            z1_int,
            z2_int,
            z3_int,
            w1,
            w2,
            w3,
            uv1,
            uv2,
            uv3,
            luv1,
            luv2,
            luv3,
            dynamic_tint,
            surf,
            lm,
            fb,
            zbuffer,
            state,
            brightness,
        );
    } else {
        let dy31 = (p3.y - p1.y) as f32;
        let dy21 = (p2.y - p1.y) as f32;
        let t = dy21 / dy31;
        let p4x = p1.x + ((p3.x - p1.x) as f32 * t) as i32;
        let p4 = Point::new(p4x, p2.y);
        let z4_int = (z1_int as f32 + (z3_int as f32 - z1_int as f32) * t) as u32;
        let w4 = w1 + (w3 - w1) * t;
        let uv4 = [
            uv1[0] + (uv3[0] - uv1[0]) * t,
            uv1[1] + (uv3[1] - uv1[1]) * t,
        ];
        let luv4 = [
            luv1[0] + (luv3[0] - luv1[0]) * t,
            luv1[1] + (luv3[1] - luv1[1]) * t,
        ];
        let p4_2 = nalgebra::Point2::new(p4.x, p4.y);
        fill_lm_bottom_flat(
            p1,
            p2,
            p4_2,
            z1_int,
            z2_int,
            z4_int,
            w1,
            w2,
            w4,
            uv1,
            uv2,
            uv4,
            luv1,
            luv2,
            luv4,
            dynamic_tint,
            surf,
            lm,
            fb,
            zbuffer,
            state,
            brightness,
        );
        fill_lm_top_flat(
            p2,
            p4_2,
            p3,
            z2_int,
            z4_int,
            z3_int,
            w2,
            w4,
            w3,
            uv2,
            uv4,
            uv3,
            luv2,
            luv4,
            luv3,
            dynamic_tint,
            surf,
            lm,
            fb,
            zbuffer,
            state,
            brightness,
        );
    }
}

#[inline]
#[allow(clippy::too_many_arguments)]
fn fill_lm_bottom_flat<D: DrawTarget<Color = Rgb565>>(
    p1: nalgebra::Point2<i32>,
    p2: nalgebra::Point2<i32>,
    p3: nalgebra::Point2<i32>,
    z1: u32,
    z2: u32,
    z3: u32,
    w1: f32,
    w2: f32,
    w3: f32,
    uv1: [f32; 2],
    uv2: [f32; 2],
    uv3: [f32; 2],
    luv1: [f32; 2],
    luv2: [f32; 2],
    luv3: [f32; 2],
    dynamic_tint: Rgb565,
    surf: &crate::pipeline::rasterize::texture::Texture,
    lm: Option<&crate::pipeline::rasterize::texture::Texture>,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    state: &RasterState<'_>,
    brightness: u8,
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
        let z_l = (z1 as i64 + (z2 as i64 - z1 as i64) * dy as i64 / height as i64) as u32;
        let z_r = (z1 as i64 + (z3 as i64 - z1 as i64) * dy as i64 / height as i64) as u32;
        let wl = w1 + t * (w2 - w1);
        let wr = w1 + t * (w3 - w1);
        let uvl = [
            uv1[0] + t * (uv2[0] - uv1[0]),
            uv1[1] + t * (uv2[1] - uv1[1]),
        ];
        let uvr = [
            uv1[0] + t * (uv3[0] - uv1[0]),
            uv1[1] + t * (uv3[1] - uv1[1]),
        ];
        let luvl = [
            luv1[0] + t * (luv2[0] - luv1[0]),
            luv1[1] + t * (luv2[1] - luv1[1]),
        ];
        let luvr = [
            luv1[0] + t * (luv3[0] - luv1[0]),
            luv1[1] + t * (luv3[1] - luv1[1]),
        ];
        draw_scanline_lm(
            curx1 >> 16,
            curx2 >> 16,
            scanline_y,
            z_l,
            z_r,
            wl,
            wr,
            uvl,
            uvr,
            luvl,
            luvr,
            dynamic_tint,
            surf,
            lm,
            fb,
            zbuffer,
            state,
            brightness,
        );
        curx1 += invslope1;
        curx2 += invslope2;
    }
}

#[inline]
#[allow(clippy::too_many_arguments)]
fn fill_lm_top_flat<D: DrawTarget<Color = Rgb565>>(
    p1: nalgebra::Point2<i32>,
    p2: nalgebra::Point2<i32>,
    p3: nalgebra::Point2<i32>,
    z1: u32,
    z2: u32,
    z3: u32,
    w1: f32,
    w2: f32,
    w3: f32,
    uv1: [f32; 2],
    uv2: [f32; 2],
    uv3: [f32; 2],
    luv1: [f32; 2],
    luv2: [f32; 2],
    luv3: [f32; 2],
    dynamic_tint: Rgb565,
    surf: &crate::pipeline::rasterize::texture::Texture,
    lm: Option<&crate::pipeline::rasterize::texture::Texture>,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    state: &RasterState<'_>,
    brightness: u8,
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
        let dy = scanline_y - p1.y;
        let t = dy as f32 / height as f32;
        let z_l = (z1 as i64 + (z3 as i64 - z1 as i64) * dy as i64 / height as i64) as u32;
        let z_r = (z2 as i64 + (z3 as i64 - z2 as i64) * dy as i64 / height as i64) as u32;
        let wl = w1 + t * (w3 - w1);
        let wr = w2 + t * (w3 - w2);
        let uvl = [
            uv1[0] + t * (uv3[0] - uv1[0]),
            uv1[1] + t * (uv3[1] - uv1[1]),
        ];
        let uvr = [
            uv2[0] + t * (uv3[0] - uv2[0]),
            uv2[1] + t * (uv3[1] - uv2[1]),
        ];
        let luvl = [
            luv1[0] + t * (luv3[0] - luv1[0]),
            luv1[1] + t * (luv3[1] - luv1[1]),
        ];
        let luvr = [
            luv2[0] + t * (luv3[0] - luv2[0]),
            luv2[1] + t * (luv3[0] - luv2[1]),
        ];
        draw_scanline_lm(
            curx1 >> 16,
            curx2 >> 16,
            scanline_y,
            z_l,
            z_r,
            wl,
            wr,
            uvl,
            uvr,
            luvl,
            luvr,
            dynamic_tint,
            surf,
            lm,
            fb,
            zbuffer,
            state,
            brightness,
        );
        curx1 -= invslope1;
        curx2 -= invslope2;
    }
}

#[inline]
#[allow(clippy::too_many_arguments)]
fn draw_scanline_lm<D: DrawTarget<Color = Rgb565>>(
    x1: i32,
    x2: i32,
    y: i32,
    z1: u32,
    z2: u32,
    w1: f32,
    w2: f32,
    uv1: [f32; 2],
    uv2: [f32; 2],
    luv1: [f32; 2],
    luv2: [f32; 2],
    dynamic_tint: Rgb565,
    surf: &crate::pipeline::rasterize::texture::Texture,
    lm: Option<&crate::pipeline::rasterize::texture::Texture>,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    state: &RasterState<'_>,
    brightness: u8,
) where
    <D as DrawTarget>::Error: Debug,
{
    if y < 0 {
        return;
    }
    let height = zbuffer.len() / state.width;
    if y as usize >= height {
        return;
    }

    let (left_x, right_x, z_left, z_right, w_left, w_right, uv_left, uv_right, luv_left, luv_right) =
        if x1 <= x2 {
            (x1, x2, z1, z2, w1, w2, uv1, uv2, luv1, luv2)
        } else {
            (x2, x1, z2, z1, w2, w1, uv2, uv1, luv2, luv1)
        };

    let start_x = left_x.max(0);
    let end_x = right_x.min(state.width as i32 - 1);
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
    let mut zbuf_idx = y as usize * state.width + start_x as usize;

    for x in start_x..=end_x {
        if should_skip_stipple(x, y, state.stipple_mode) {
            z_curr += z_step as i64;
            zbuf_idx += 1;
            continue;
        }

        let z = (z_curr >> 16) as u32;
        z_curr += z_step as i64;
        let z_depth = crate::to_zdepth(z);

        if z_depth >= zbuffer[zbuf_idx].saturating_add(crate::DEPTH_EPSILON) {
            zbuf_idx += 1;
            continue;
        }
        zbuffer[zbuf_idx] = z_depth;

        let t = (x - left_x) as f32 * inv_span;
        let [su, sv] = interpolate_uv(t, w_left, w_right, uv_left, uv_right, state.texture_mapping);
        let surf_c = surf.sample(su, sv);

        let lit_c = if let Some(lm_tex) = lm {
            let [lu, lv] = interpolate_uv(
                t,
                w_left,
                w_right,
                luv_left,
                luv_right,
                state.texture_mapping,
            );
            let lm_c = lm_tex.sample(lu, lv);
            let r = ((surf_c.r() as u32 * lm_c.r() as u32) / 31).min(31) as u8;
            let g = ((surf_c.g() as u32 * lm_c.g() as u32) / 63).min(63) as u8;
            let b = ((surf_c.b() as u32 * lm_c.b() as u32) / 31).min(31) as u8;
            Rgb565::new(r, g, b)
        } else {
            surf_c
        };

        let lit_c = if brightness < 255 {
            let scale = brightness as u32;
            let r = ((lit_c.r() as u32 * scale) / 255) as u8;
            let g = ((lit_c.g() as u32 * scale) / 255) as u8;
            let b = ((lit_c.b() as u32 * scale) / 255) as u8;
            Rgb565::new(r, g, b)
        } else {
            lit_c
        };

        let tinted_c = Rgb565::new(
            (lit_c.r() as u16 + dynamic_tint.r() as u16).min(31) as u8,
            (lit_c.g() as u16 + dynamic_tint.g() as u16).min(63) as u8,
            (lit_c.b() as u16 + dynamic_tint.b() as u16).min(31) as u8,
        );

        let mut final_c = if let Some(fog) = state.fog {
            fog.apply(tinted_c, z)
        } else {
            tinted_c
        };

        if let Some(tint) = state.screen_tint {
            final_c = tint.apply(final_c);
        }
        final_c = state.palette_mode.apply(final_c);

        fb.draw_iter([embedded_graphics_core::Pixel(Point::new(x, y), final_c)])
            .unwrap();
    }
}
