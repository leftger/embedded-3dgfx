//! Solid / flat-colour Z-buffered triangle rasterization.

use core::fmt::Debug;
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::prelude::Point;

use crate::pipeline::effects::{DitherConfig, FogConfig};

#[inline]
pub(crate) fn fill_triangle_zbuffered<D: DrawTarget<Color = Rgb565>>(
    p1: nalgebra::Point2<i32>,
    p2: nalgebra::Point2<i32>,
    p3: nalgebra::Point2<i32>,
    z1: f32,
    z2: f32,
    z3: f32,
    color: Rgb565,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    width: usize,
    fog_config: Option<&FogConfig>,
    dither_config: Option<&DitherConfig>,
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
        fill_bottom_flat_triangle_zbuffered(
            p1_eg,
            p2_eg,
            p3_eg,
            z1_int,
            z2_int,
            z3_int,
            color,
            fb,
            zbuffer,
            width,
            fog_config,
            dither_config,
        );
    } else if p1_eg.y == p2_eg.y {
        fill_top_flat_triangle_zbuffered(
            p1_eg,
            p2_eg,
            p3_eg,
            z1_int,
            z2_int,
            z3_int,
            color,
            fb,
            zbuffer,
            width,
            fog_config,
            dither_config,
        );
    } else {
        let t = (p2_eg.y - p1_eg.y) as f32 / (p3_eg.y - p1_eg.y) as f32;
        let p4 = Point::new(
            (p1_eg.x as f32 + t * (p3_eg.x - p1_eg.x) as f32) as i32,
            p2_eg.y,
        );
        let z4_int = (z1_int as i64 + (t * (z3_int as i64 - z1_int as i64) as f32) as i64) as u32;

        fill_bottom_flat_triangle_zbuffered(
            p1_eg,
            p2_eg,
            p4,
            z1_int,
            z2_int,
            z4_int,
            color,
            fb,
            zbuffer,
            width,
            fog_config,
            dither_config,
        );
        fill_top_flat_triangle_zbuffered(
            p2_eg,
            p4,
            p3_eg,
            z2_int,
            z4_int,
            z3_int,
            color,
            fb,
            zbuffer,
            width,
            fog_config,
            dither_config,
        );
    }
}

#[inline]
pub(crate) fn fill_bottom_flat_triangle_zbuffered<D: DrawTarget<Color = Rgb565>>(
    p1: Point,
    p2: Point,
    p3: Point,
    z1: u32,
    z2: u32,
    z3: u32,
    color: Rgb565,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    width: usize,
    fog_config: Option<&FogConfig>,
    dither_config: Option<&DitherConfig>,
) where
    <D as DrawTarget>::Error: Debug,
{
    let height = p2.y - p1.y;
    if height == 0 {
        return;
    }

    let invslope1 = ((p2.x - p1.x) << 16) / height;
    let invslope2 = ((p3.x - p1.x) << 16) / height;

    let mut curx1 = (p1.x << 16) + (1 << 15);
    let mut curx2 = (p1.x << 16) + (1 << 15);

    let scr_h = (zbuffer.len() / width) as i32;
    let y_skip = (0_i32 - p1.y).max(0);
    curx1 = curx1.wrapping_add(invslope1.wrapping_mul(y_skip));
    curx2 = curx2.wrapping_add(invslope2.wrapping_mul(y_skip));
    let y_start = p1.y.max(0);
    let y_end = p2.y.min(scr_h - 1);

    for scanline_y in y_start..=y_end {
        let dy = scanline_y - p1.y;
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

        draw_scanline_zbuffered(
            curx1 >> 16,
            curx2 >> 16,
            scanline_y,
            z_left,
            z_right,
            color,
            fb,
            zbuffer,
            width,
            fog_config,
            dither_config,
        );

        curx1 = curx1.wrapping_add(invslope1);
        curx2 = curx2.wrapping_add(invslope2);
    }
}

#[inline]
pub(crate) fn fill_top_flat_triangle_zbuffered<D: DrawTarget<Color = Rgb565>>(
    p1: Point,
    p2: Point,
    p3: Point,
    z1: u32,
    z2: u32,
    z3: u32,
    color: Rgb565,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    width: usize,
    fog_config: Option<&FogConfig>,
    dither_config: Option<&DitherConfig>,
) where
    <D as DrawTarget>::Error: Debug,
{
    let height = p3.y - p1.y;
    if height == 0 {
        return;
    }

    let invslope1 = ((p3.x - p1.x) << 16) / height;
    let invslope2 = ((p3.x - p2.x) << 16) / height;

    let mut curx1 = (p3.x << 16) + (1 << 15);
    let mut curx2 = (p3.x << 16) + (1 << 15);

    let scr_h = (zbuffer.len() / width) as i32;
    let y_skip_bot = (p3.y - (scr_h - 1)).max(0);
    curx1 = curx1.wrapping_sub(invslope1.wrapping_mul(y_skip_bot));
    curx2 = curx2.wrapping_sub(invslope2.wrapping_mul(y_skip_bot));
    let y_start = p1.y.max(0);
    let y_end = p3.y.min(scr_h - 1);

    for scanline_y in (y_start..=y_end).rev() {
        let dy = scanline_y - p1.y;
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

        draw_scanline_zbuffered(
            curx1 >> 16,
            curx2 >> 16,
            scanline_y,
            z_left,
            z_right,
            color,
            fb,
            zbuffer,
            width,
            fog_config,
            dither_config,
        );

        curx1 = curx1.wrapping_sub(invslope1);
        curx2 = curx2.wrapping_sub(invslope2);
    }
}

#[inline(always)]
pub(crate) fn draw_scanline_zbuffered<D: DrawTarget<Color = Rgb565>>(
    x1: i32,
    x2: i32,
    y: i32,
    z1: u32,
    z2: u32,
    color: Rgb565,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    width: usize,
    fog_config: Option<&FogConfig>,
    dither_config: Option<&DitherConfig>,
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

    let (left_x, right_x, z_left, z_right) = if x1 <= x2 {
        (x1, x2, z1, z2)
    } else {
        (x2, x1, z2, z1)
    };

    let start_x = left_x.max(0);
    let end_x = right_x.min(width as i32 - 1);
    if start_x > end_x {
        return;
    }

    let span = right_x - left_x;
    let z_step = if span > 0 {
        (((z_right as i64 - z_left as i64) << 16) / span as i64) as i32
    } else {
        0
    };

    let mut zbuf_idx = y as usize * width + start_x as usize;

    if z_step == 0 {
        let z = z_left;
        let z_depth = crate::to_zdepth(z);
        let mut base_color = color;
        if let Some(fog) = fog_config {
            base_color = fog.apply(base_color, z);
        }
        for x in start_x..=end_x {
            if z_depth < zbuffer[zbuf_idx].saturating_add(crate::DEPTH_EPSILON) {
                zbuffer[zbuf_idx] = z_depth;
                let final_color = if let Some(dither) = dither_config {
                    dither.apply(base_color, x, y)
                } else {
                    base_color
                };
                fb.draw_iter([embedded_graphics_core::Pixel(Point::new(x, y), final_color)])
                    .unwrap();
            }
            zbuf_idx += 1;
        }
        return;
    }

    let left_clip = start_x - left_x;
    let mut z_curr = ((z_left as i64) << 16) + (left_clip as i64 * z_step as i64);

    for x in start_x..=end_x {
        let z = (z_curr >> 16) as u32;
        z_curr += z_step as i64;
        let z_depth = crate::to_zdepth(z);

        if z_depth < zbuffer[zbuf_idx].saturating_add(crate::DEPTH_EPSILON) {
            zbuffer[zbuf_idx] = z_depth;

            let mut final_color = color;

            if let Some(fog) = fog_config {
                final_color = fog.apply(final_color, z);
            }

            if let Some(dither) = dither_config {
                final_color = dither.apply(final_color, x, y);
            }

            fb.draw_iter([embedded_graphics_core::Pixel(Point::new(x, y), final_color)])
                .unwrap();
        }
        zbuf_idx += 1;
    }
}
