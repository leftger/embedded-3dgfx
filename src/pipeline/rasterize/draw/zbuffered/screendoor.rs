//! Screen-door (stipple / checkerboard) transparency fills.

//! Standard Z-buffered triangle rasterization, Gouraud shading, and translucent triangles.

use core::fmt::Debug;
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::prelude::Point;

pub(super) fn fill_triangle_zbuffered_screendoor<D: DrawTarget<Color = Rgb565>>(
    p1: nalgebra::Point2<i32>,
    p2: nalgebra::Point2<i32>,
    p3: nalgebra::Point2<i32>,
    z1: f32,
    z2: f32,
    z3: f32,
    color: Rgb565,
    alpha: u8,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    width: usize,
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
        fill_bottom_flat_screendoor(
            p1_eg, p2_eg, p3_eg, z1_int, z2_int, z3_int, color, alpha, fb, zbuffer, width,
        );
    } else if p1_eg.y == p2_eg.y {
        fill_top_flat_screendoor(
            p1_eg, p2_eg, p3_eg, z1_int, z2_int, z3_int, color, alpha, fb, zbuffer, width,
        );
    } else {
        let t = (p2_eg.y - p1_eg.y) as f32 / (p3_eg.y - p1_eg.y) as f32;
        let p4 = Point::new(
            (p1_eg.x as f32 + t * (p3_eg.x - p1_eg.x) as f32) as i32,
            p2_eg.y,
        );
        let z4_int = (z1_int as i64 + (t * (z3_int as i64 - z1_int as i64) as f32) as i64) as u32;
        fill_bottom_flat_screendoor(
            p1_eg, p2_eg, p4, z1_int, z2_int, z4_int, color, alpha, fb, zbuffer, width,
        );
        fill_top_flat_screendoor(
            p2_eg, p4, p3_eg, z2_int, z4_int, z3_int, color, alpha, fb, zbuffer, width,
        );
    }
}

#[inline(always)]
pub(super) fn fill_bottom_flat_screendoor<D: DrawTarget<Color = Rgb565>>(
    p1: Point,
    p2: Point,
    p3: Point,
    z1: u32,
    z2: u32,
    z3: u32,
    color: Rgb565,
    alpha: u8,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    width: usize,
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

    let sd = crate::pipeline::effects::ScreenDoorConfig::new(alpha);

    for scanline_y in p1.y..=p2.y {
        let dy = scanline_y - p1.y;
        let z_left = (z1 as i64 + ((z2 as i64 - z1 as i64) * dy as i64 / height as i64)) as u32;
        let z_right = (z1 as i64 + ((z3 as i64 - z1 as i64) * dy as i64 / height as i64)) as u32;

        let (left_x, right_x, z_l, z_r) = if curx1 <= curx2 {
            (curx1 >> 16, curx2 >> 16, z_left, z_right)
        } else {
            (curx2 >> 16, curx1 >> 16, z_right, z_left)
        };

        let span = right_x - left_x;
        for x in left_x..=right_x {
            if x < 0 || scanline_y < 0 || x >= width as i32 {
                continue;
            }
            if !sd.test(x, scanline_y) {
                continue;
            }
            let idx = (scanline_y as usize) * width + (x as usize);
            if idx >= zbuffer.len() {
                continue;
            }

            let z = if span > 0 {
                let t = (x - left_x) as f32 / span as f32;
                (z_l as f32 + t * (z_r as f32 - z_l as f32)) as u32
            } else {
                z_l
            };
            let z_depth = crate::to_zdepth(z);

            if z_depth < zbuffer[idx] {
                zbuffer[idx] = z_depth;
                let _ = fb.draw_iter([embedded_graphics_core::Pixel(
                    Point::new(x, scanline_y),
                    color,
                )]);
            }
        }

        curx1 += invslope1;
        curx2 += invslope2;
    }
}

#[inline(always)]
pub(super) fn fill_top_flat_screendoor<D: DrawTarget<Color = Rgb565>>(
    p1: Point,
    p2: Point,
    p3: Point,
    z1: u32,
    z2: u32,
    z3: u32,
    color: Rgb565,
    alpha: u8,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    width: usize,
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

    let sd = crate::pipeline::effects::ScreenDoorConfig::new(alpha);

    for scanline_y in (p1.y..=p3.y).rev() {
        let dy = scanline_y - p1.y;
        let z_left = (z1 as i64 + ((z3 as i64 - z1 as i64) * dy as i64 / height as i64)) as u32;
        let z_right = (z2 as i64 + ((z3 as i64 - z2 as i64) * dy as i64 / height as i64)) as u32;

        let (left_x, right_x, z_l, z_r) = if curx1 <= curx2 {
            (curx1 >> 16, curx2 >> 16, z_left, z_right)
        } else {
            (curx2 >> 16, curx1 >> 16, z_right, z_left)
        };

        let span = right_x - left_x;
        for x in left_x..=right_x {
            if x < 0 || scanline_y < 0 || x >= width as i32 {
                continue;
            }
            if !sd.test(x, scanline_y) {
                continue;
            }
            let idx = (scanline_y as usize) * width + (x as usize);
            if idx >= zbuffer.len() {
                continue;
            }

            let z = if span > 0 {
                let t = (x - left_x) as f32 / span as f32;
                (z_l as f32 + t * (z_r as f32 - z_l as f32)) as u32
            } else {
                z_l
            };
            let z_depth = crate::to_zdepth(z);

            if z_depth < zbuffer[idx] {
                zbuffer[idx] = z_depth;
                let _ = fb.draw_iter([embedded_graphics_core::Pixel(
                    Point::new(x, scanline_y),
                    color,
                )]);
            }
        }

        curx1 -= invslope1;
        curx2 -= invslope2;
    }
}
