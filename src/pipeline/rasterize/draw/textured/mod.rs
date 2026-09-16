//! Textured triangle rasterization, split by concern.
//!
//! * this module — the surface-texture path ([`draw_zbuffered_with_textures`],
//!   [`draw_zbuffered_with_textures_state`]) plus the scanline helpers the other
//!   concerns build on
//! * [`lightmap`] — a surface texture modulated by a baked lightmap
//! * [`gouraud`] — per-vertex colour interpolation combined with texturing
//! * [`coverage`] — the coverage-driven draw used by the BSP renderers

use core::fmt::Debug;
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::prelude::Point;

use crate::pipeline::assemble::primitive::DrawPrimitive;
use crate::pipeline::effects::{DitherConfig, FogConfig};
use crate::pipeline::rasterize::draw::state::{
    RasterState, apply_order, ascending_y_order, tri_fully_off_screen,
};
use crate::pipeline::shade::retro::palette::PaletteMode;
use crate::pipeline::shade::retro::stipple::StippleMode;
use crate::pipeline::shade::retro::texture_lod::TextureMapping;
use crate::pipeline::shade::retro::tint::ScreenTint;

#[cfg(feature = "raycast")]
pub mod coverage;
pub mod gouraud;
use gouraud::fill_triangle_zbuffered_textured_gouraud;
pub mod lightmap;

#[inline]
pub(crate) fn interpolate_uv(
    t: f32,
    w_left: f32,
    w_right: f32,
    uv_left: [f32; 2],
    uv_right: [f32; 2],
    mapping: TextureMapping,
) -> [f32; 2] {
    match mapping {
        TextureMapping::Affine => [
            uv_left[0] + t * (uv_right[0] - uv_left[0]),
            uv_left[1] + t * (uv_right[1] - uv_left[1]),
        ],
        TextureMapping::PerspectiveCorrect => {
            let inv_w_l = if w_left != 0.0 { 1.0 / w_left } else { 1.0 };
            let inv_w_r = if w_right != 0.0 { 1.0 / w_right } else { 1.0 };
            let inv_w = inv_w_l + t * (inv_w_r - inv_w_l);
            let w = if inv_w != 0.0 { 1.0 / inv_w } else { 1.0 };

            let u_over_w_l = uv_left[0] * inv_w_l;
            let u_over_w_r = uv_right[0] * inv_w_r;
            let u_over_w = u_over_w_l + t * (u_over_w_r - u_over_w_l);

            let v_over_w_l = uv_left[1] * inv_w_l;
            let v_over_w_r = uv_right[1] * inv_w_r;
            let v_over_w = v_over_w_l + t * (v_over_w_r - v_over_w_l);

            [u_over_w * w, v_over_w * w]
        }
    }
}

#[inline]
pub(crate) fn should_skip_stipple(x: i32, y: i32, stipple_mode: StippleMode) -> bool {
    match stipple_mode {
        StippleMode::Off => false,
        StippleMode::Checkerboard => ((x ^ y) & 1) != 0,
    }
}

#[inline]
pub fn draw_zbuffered_with_textures<D: DrawTarget<Color = Rgb565>, const N: usize>(
    primitive: DrawPrimitive,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    width: usize,
    texture_manager: &crate::pipeline::rasterize::texture::TextureManager<N>,
    fog_config: Option<&FogConfig>,
    dither_config: Option<&DitherConfig>,
) where
    <D as DrawTarget>::Error: Debug,
{
    let state = RasterState::from_zbuffer(width, zbuffer.len())
        .with_fog(fog_config)
        .with_dither(dither_config);
    draw_zbuffered_with_textures_state(primitive, fb, zbuffer, texture_manager, &state);
}

/// Textured dispatch driven by a single [`RasterState`].
///
/// This is the core that [`draw_zbuffered_with_textures`] and
/// [`draw_zbuffered_with_textures_mapped`] adapt into; the vertex sort and
/// screen-reject logic is shared with the flat path via [`super::state`].
#[inline]
pub fn draw_zbuffered_with_textures_state<D: DrawTarget<Color = Rgb565>, const N: usize>(
    primitive: DrawPrimitive,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    texture_manager: &crate::pipeline::rasterize::texture::TextureManager<N>,
    state: &RasterState<'_>,
) where
    <D as DrawTarget>::Error: Debug,
{
    match primitive {
        DrawPrimitive::TexturedTriangleWithDepth {
            mut points,
            mut depths,
            mut ws,
            mut uvs,
            texture_id,
        } => {
            let Some(texture) = texture_manager.get(texture_id) else {
                return;
            };
            let order = ascending_y_order(&points);
            apply_order(&mut points, order);
            apply_order(&mut depths, order);
            apply_order(&mut ws, order);
            apply_order(&mut uvs, order);
            if tri_fully_off_screen(&points, state.width, state.height) {
                return;
            }

            let [p1, p2, p3] = points;
            let [z1, z2, z3] = depths;
            let [w1, w2, w3] = ws;
            let [uv1, uv2, uv3] = uvs;

            fill_triangle_zbuffered_textured(
                p1,
                p2,
                p3,
                z1,
                z2,
                z3,
                w1,
                w2,
                w3,
                uv1,
                uv2,
                uv3,
                texture,
                fb,
                zbuffer,
                state.width,
                state.fog,
                state.dither,
                state.texture_mapping,
                state.stipple_mode,
                state.screen_tint,
                state.palette_mode,
            );
        }
        DrawPrimitive::TexturedGouraudTriangleWithDepth {
            mut points,
            mut depths,
            mut ws,
            mut uvs,
            mut colors,
            texture_id,
        } => {
            let Some(texture) = texture_manager.get(texture_id) else {
                return;
            };
            let order = ascending_y_order(&points);
            apply_order(&mut points, order);
            apply_order(&mut depths, order);
            apply_order(&mut ws, order);
            apply_order(&mut uvs, order);
            apply_order(&mut colors, order);
            if tri_fully_off_screen(&points, state.width, state.height) {
                return;
            }

            let [p1, p2, p3] = points;
            let [z1, z2, z3] = depths;
            let [w1, w2, w3] = ws;
            let [uv1, uv2, uv3] = uvs;
            let [c1, c2, c3] = colors;

            fill_triangle_zbuffered_textured_gouraud(
                p1,
                p2,
                p3,
                z1,
                z2,
                z3,
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
                state.width,
                state.fog,
                state.dither,
                state.texture_mapping,
                state.stipple_mode,
                state.screen_tint,
                state.palette_mode,
            );
        }
        _ => super::zbuffered::draw_zbuffered_with_state(primitive, fb, zbuffer, state),
    }
}

/// Compatibility adapter over [`draw_zbuffered_with_textures_state`].
#[inline]
#[allow(clippy::too_many_arguments)]
pub fn draw_zbuffered_with_textures_mapped<D: DrawTarget<Color = Rgb565>, const N: usize>(
    primitive: DrawPrimitive,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    width: usize,
    texture_manager: &crate::pipeline::rasterize::texture::TextureManager<N>,
    fog_config: Option<&FogConfig>,
    dither_config: Option<&DitherConfig>,
    texture_mapping: TextureMapping,
    stipple_mode: StippleMode,
    screen_tint: Option<ScreenTint>,
    palette_mode: PaletteMode,
) where
    <D as DrawTarget>::Error: Debug,
{
    let state = RasterState::from_zbuffer(width, zbuffer.len())
        .with_fog(fog_config)
        .with_dither(dither_config)
        .with_texture_mapping(texture_mapping)
        .with_stipple_mode(stipple_mode)
        .with_screen_tint(screen_tint)
        .with_palette_mode(palette_mode);
    draw_zbuffered_with_textures_state(primitive, fb, zbuffer, texture_manager, &state);
}

#[inline(always)]
fn fill_triangle_zbuffered_textured<D: DrawTarget<Color = Rgb565>>(
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
        fill_bottom_flat_triangle_zbuffered_textured(
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
        fill_top_flat_triangle_zbuffered_textured(
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
        let p4 = Point::new(
            (p1_eg.x as f32 + t * (p3_eg.x - p1_eg.x) as f32) as i32,
            p2_eg.y,
        );
        let z4_int = (z1_int as i64 + (t * (z3_int as i64 - z1_int as i64) as f32) as i64) as u32;
        let w4 = w1 + t * (w3 - w1);
        let uv4 = [
            uv1[0] + t * (uv3[0] - uv1[0]),
            uv1[1] + t * (uv3[1] - uv1[1]),
        ];

        fill_bottom_flat_triangle_zbuffered_textured(
            p1_eg,
            p2_eg,
            p4,
            z1_int,
            z2_int,
            z4_int,
            w1,
            w2,
            w4,
            uv1,
            uv2,
            uv4,
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
        fill_top_flat_triangle_zbuffered_textured(
            p2_eg,
            p4,
            p3_eg,
            z2_int,
            z4_int,
            z3_int,
            w2,
            w4,
            w3,
            uv2,
            uv4,
            uv3,
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

#[inline(always)]
fn fill_bottom_flat_triangle_zbuffered_textured<D: DrawTarget<Color = Rgb565>>(
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

        draw_scanline_zbuffered_textured(
            curx1 >> 16,
            curx2 >> 16,
            scanline_y,
            z_left,
            z_right,
            w_left,
            w_right,
            uv_left,
            uv_right,
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

#[inline(always)]
fn fill_top_flat_triangle_zbuffered_textured<D: DrawTarget<Color = Rgb565>>(
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
        let dy = scanline_y - p1.y;
        let t = dy as f32 / height as f32;

        let z_left = if height > 0 {
            (z1 as i64 + ((z3 as i64 - z1 as i64) * dy as i64 / height as i64)) as u32
        } else {
            z1
        };
        let z_right = if height > 0 {
            (z2 as i64 + ((z3 as i64 - z2 as i64) * dy as i64 / height as i64)) as u32
        } else {
            z2
        };

        let w_left = w1 + t * (w3 - w1);
        let w_right = w2 + t * (w3 - w2);

        let uv_left = [
            uv1[0] + t * (uv3[0] - uv1[0]),
            uv1[1] + t * (uv3[1] - uv1[1]),
        ];
        let uv_right = [
            uv2[0] + t * (uv3[0] - uv2[0]),
            uv2[1] + t * (uv3[1] - uv2[1]),
        ];

        draw_scanline_zbuffered_textured(
            curx1 >> 16,
            curx2 >> 16,
            scanline_y,
            z_left,
            z_right,
            w_left,
            w_right,
            uv_left,
            uv_right,
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

#[inline(always)]
pub(crate) fn draw_scanline_zbuffered_textured<D: DrawTarget<Color = Rgb565>>(
    x1: i32,
    x2: i32,
    y: i32,
    z1: u32,
    z2: u32,
    w1: f32,
    w2: f32,
    uv1: [f32; 2],
    uv2: [f32; 2],
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

    let (left_x, right_x, z_left, z_right, w_left, w_right, uv_left, uv_right) = if x1 <= x2 {
        (x1, x2, z1, z2, w1, w2, uv1, uv2)
    } else {
        (x2, x1, z2, z1, w2, w1, uv2, uv1)
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

                let mut final_color = texture.sample(curr_u, curr_v);

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::rasterize::texture::{Texture, TextureManager};
    use embedded_graphics_core::Pixel;
    use embedded_graphics_core::geometry::{OriginDimensions, Size};
    use embedded_graphics_core::pixelcolor::RgbColor;

    static TEX_DATA_RED: [Rgb565; 16] = [Rgb565::RED; 16];
    static TEX_DATA_GREEN: [Rgb565; 16] = [Rgb565::GREEN; 16];
    static TEX_DATA_WHITE: [Rgb565; 16] = [Rgb565::WHITE; 16];
    static TEX_DATA_LM: [Rgb565; 16] = [Rgb565::new(15, 30, 15); 16];

    struct TestFb<const W: usize, const H: usize> {
        pixels: [Rgb565; 400],
    }

    impl<const W: usize, const H: usize> Default for TestFb<W, H> {
        fn default() -> Self {
            Self {
                pixels: [Rgb565::BLACK; 400],
            }
        }
    }

    impl<const W: usize, const H: usize> OriginDimensions for TestFb<W, H> {
        fn size(&self) -> Size {
            Size::new(W as u32, H as u32)
        }
    }

    impl<const W: usize, const H: usize> DrawTarget for TestFb<W, H> {
        type Color = Rgb565;
        type Error = core::convert::Infallible;

        fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Pixel<Self::Color>>,
        {
            for Pixel(point, color) in pixels {
                if point.x >= 0 && point.x < W as i32 && point.y >= 0 && point.y < H as i32 {
                    let idx = (point.y as usize) * W + (point.x as usize);
                    if idx < self.pixels.len() {
                        self.pixels[idx] = color;
                    }
                }
            }
            Ok(())
        }
    }

    #[test]
    fn test_fill_triangle_textured_gouraud_perspective_and_affine() {
        let texture = Texture::new(&TEX_DATA_RED, 4, 4);

        let mut fb = TestFb::<20, 20>::default();
        let mut zbuf = [crate::Z_MAX_VALUE; 400];

        // Flat-bottom triangle
        fill_triangle_zbuffered_textured_gouraud(
            nalgebra::Point2::new(10, 2),
            nalgebra::Point2::new(2, 18),
            nalgebra::Point2::new(18, 18),
            1.0,
            2.0,
            2.0,
            1.0,
            0.5,
            0.5,
            [0.5, 0.0],
            [0.0, 1.0],
            [1.0, 1.0],
            Rgb565::WHITE,
            Rgb565::WHITE,
            Rgb565::WHITE,
            &texture,
            &mut fb,
            &mut zbuf,
            20,
            None,
            None,
            TextureMapping::PerspectiveCorrect,
            StippleMode::Off,
            None,
            PaletteMode::Off,
        );

        let center_idx = 10 * 20 + 10;
        assert_eq!(fb.pixels[center_idx], Rgb565::RED);

        // Affine mapping mode
        let mut fb_affine = TestFb::<20, 20>::default();
        let mut zbuf_affine = [crate::Z_MAX_VALUE; 400];

        fill_triangle_zbuffered_textured_gouraud(
            nalgebra::Point2::new(10, 2),
            nalgebra::Point2::new(2, 18),
            nalgebra::Point2::new(18, 18),
            1.0,
            2.0,
            2.0,
            1.0,
            0.5,
            0.5,
            [0.5, 0.0],
            [0.0, 1.0],
            [1.0, 1.0],
            Rgb565::WHITE,
            Rgb565::WHITE,
            Rgb565::WHITE,
            &texture,
            &mut fb_affine,
            &mut zbuf_affine,
            20,
            None,
            None,
            TextureMapping::Affine,
            StippleMode::Off,
            None,
            PaletteMode::Off,
        );
        assert_eq!(fb_affine.pixels[center_idx], Rgb565::RED);
    }

    #[test]
    fn test_draw_zbuffered_with_textures() {
        let texture = Texture::new(&TEX_DATA_GREEN, 4, 4);
        let mut tm = TextureManager::<4>::new();
        let tex_id = tm.add_texture(texture).unwrap();

        let mut fb = TestFb::<20, 20>::default();
        let mut zbuf = [crate::Z_MAX_VALUE; 400];

        let prim = crate::pipeline::assemble::primitive::DrawPrimitive::TexturedTriangleWithDepth {
            points: [
                nalgebra::Point2::new(2, 2),
                nalgebra::Point2::new(18, 2),
                nalgebra::Point2::new(10, 18),
            ],
            depths: [1.0, 1.0, 1.0],
            ws: [1.0, 1.0, 1.0],
            uvs: [[0.0, 0.0], [1.0, 0.0], [0.5, 1.0]],
            texture_id: tex_id,
        };

        draw_zbuffered_with_textures(prim, &mut fb, &mut zbuf, 20, &tm, None, None);
        let center_idx = 10 * 20 + 10;
        assert_eq!(fb.pixels[center_idx], Rgb565::GREEN);
    }

    #[test]
    fn test_draw_zbuffered_lightmapped() {
        let texture = Texture::new(&TEX_DATA_WHITE, 4, 4);
        let lightmap = Texture::new(&TEX_DATA_LM, 4, 4);

        let mut tm = TextureManager::<4>::new();
        let tex_id = tm.add_texture(texture).unwrap();
        let lm_id = tm.add_texture(lightmap).unwrap();

        let mut fb = TestFb::<20, 20>::default();
        let mut zbuf = [crate::Z_MAX_VALUE; 400];

        let points = [
            nalgebra::Point2::new(2, 2),
            nalgebra::Point2::new(18, 2),
            nalgebra::Point2::new(10, 18),
        ];
        let depths = [1.0, 1.0, 1.0];
        let ws = [1.0, 1.0, 1.0];
        let uvs = [[0.0, 0.0], [1.0, 0.0], [0.5, 1.0]];

        let state = RasterState::new(20, 20);
        draw_zbuffered_lightmapped(
            points,
            depths,
            ws,
            uvs,
            uvs,
            tex_id,
            lm_id,
            255,
            Rgb565::WHITE,
            &tm,
            &mut fb,
            &mut zbuf,
            &state,
        );

        let center_idx = 10 * 20 + 10;
        assert!(fb.pixels[center_idx].r() > 0);
    }
}

// Curated surface: `draw::textured::X` keeps resolving for the items that moved
// into a concern submodule, so callers do not have to know how the module is
// carved up internally.
#[cfg(feature = "raycast")]
pub use coverage::draw_bsp_coverage;
pub use lightmap::{draw_zbuffered_lightmapped, draw_zbuffered_lightmapped_mapped};
