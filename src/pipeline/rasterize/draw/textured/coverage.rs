//! Coverage-driven textured draw, used by the BSP renderers.

//! Textured, lightmapped, and BSP coverage triangle rasterization.

use core::fmt::Debug;
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::prelude::Point;

use super::{interpolate_uv, should_skip_stipple};
use crate::pipeline::shade::retro::palette::PaletteMode;
use crate::pipeline::shade::retro::stipple::StippleMode;
use crate::pipeline::shade::retro::texture_lod::TextureMapping;
use crate::pipeline::shade::retro::tint::ScreenTint;

pub fn draw_bsp_coverage<D: DrawTarget<Color = Rgb565>, const N: usize>(
    mut points: [nalgebra::Point2<i32>; 3],
    mut ws: [f32; 3],
    mut uvs: [[f32; 2]; 3],
    texture_id: u32,
    texture_manager: &crate::pipeline::rasterize::texture::TextureManager<N>,
    fb: &mut D,
    coverage: &mut crate::pipeline::rasterize::coverage::CoverageBuffer<'_>,
    texture_mapping: TextureMapping,
    stipple_mode: StippleMode,
    screen_tint: Option<ScreenTint>,
    palette_mode: PaletteMode,
) where
    <D as DrawTarget>::Error: Debug,
{
    let tex = match texture_manager.get(texture_id) {
        Some(t) => t,
        None => return,
    };

    if points[0].y > points[1].y {
        points.swap(0, 1);
        ws.swap(0, 1);
        uvs.swap(0, 1);
    }
    if points[0].y > points[2].y {
        points.swap(0, 2);
        ws.swap(0, 2);
        uvs.swap(0, 2);
    }
    if points[1].y > points[2].y {
        points.swap(1, 2);
        ws.swap(1, 2);
        uvs.swap(1, 2);
    }

    let [p1, p2, p3] = points;
    let [w1, w2, w3] = ws;
    let [uv1, uv2, uv3] = uvs;

    let w = coverage.width as i32;
    let h = coverage.height as i32;
    if p1.x < 0 && p2.x < 0 && p3.x < 0 {
        return;
    }
    if p1.x >= w && p2.x >= w && p3.x >= w {
        return;
    }
    if p1.y < 0 && p2.y < 0 && p3.y < 0 {
        return;
    }
    if p1.y >= h && p2.y >= h && p3.y >= h {
        return;
    }

    let rasterize_span =
        |x1: i32,
         x2: i32,
         y: i32,
         wl: f32,
         wr: f32,
         uvl: [f32; 2],
         uvr: [f32; 2],
         fb: &mut D,
         coverage: &mut crate::pipeline::rasterize::coverage::CoverageBuffer<'_>| {
            let start = x1.min(x2);
            let end = x1.max(x2);
            let span = end - start;
            for x in start..=end {
                if x < 0 || y < 0 || x >= w || y >= h {
                    continue;
                }
                if coverage.is_covered(x as usize, y as usize) {
                    continue;
                }
                if should_skip_stipple(x, y, stipple_mode) {
                    continue;
                }
                let t = if span > 0 {
                    (x - start) as f32 / span as f32
                } else {
                    0.0
                };
                let [su, sv] = interpolate_uv(t, wl, wr, uvl, uvr, texture_mapping);
                let mut color = tex.sample(su, sv);
                if let Some(tint) = screen_tint {
                    color = tint.apply(color);
                }
                color = palette_mode.apply(color);
                coverage.mark_covered(x as usize, y as usize);
                fb.draw_iter([embedded_graphics_core::Pixel(Point::new(x, y), color)])
                    .unwrap();
            }
        };

    let draw_flat_bottom =
        |p1: nalgebra::Point2<i32>,
         p2: nalgebra::Point2<i32>,
         p3: nalgebra::Point2<i32>,
         w1: f32,
         w2: f32,
         w3: f32,
         uv1: [f32; 2],
         uv2: [f32; 2],
         uv3: [f32; 2],
         fb: &mut D,
         coverage: &mut crate::pipeline::rasterize::coverage::CoverageBuffer<'_>| {
            let height = p2.y - p1.y;
            if height == 0 {
                return;
            }
            let invslope1 = ((p2.x - p1.x) << 16) / height;
            let invslope2 = ((p3.x - p1.x) << 16) / height;
            let mut cx1 = p1.x << 16;
            let mut cx2 = p1.x << 16;
            for sy in p1.y..=p2.y {
                let dy = sy - p1.y;
                let t = dy as f32 / height as f32;
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
                rasterize_span(cx1 >> 16, cx2 >> 16, sy, wl, wr, uvl, uvr, fb, coverage);
                cx1 += invslope1;
                cx2 += invslope2;
            }
        };

    let draw_flat_top =
        |p1: nalgebra::Point2<i32>,
         p2: nalgebra::Point2<i32>,
         p3: nalgebra::Point2<i32>,
         w1: f32,
         w2: f32,
         w3: f32,
         uv1: [f32; 2],
         uv2: [f32; 2],
         uv3: [f32; 2],
         fb: &mut D,
         coverage: &mut crate::pipeline::rasterize::coverage::CoverageBuffer<'_>| {
            let height = p3.y - p1.y;
            if height == 0 {
                return;
            }
            let invslope1 = ((p3.x - p1.x) << 16) / height;
            let invslope2 = ((p3.x - p2.x) << 16) / height;
            let mut cx1 = p3.x << 16;
            let mut cx2 = p3.x << 16;
            for sy in (p1.y..=p3.y).rev() {
                let dy = sy - p1.y;
                let t = dy as f32 / height as f32;
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
                rasterize_span(cx1 >> 16, cx2 >> 16, sy, wl, wr, uvl, uvr, fb, coverage);
                cx1 -= invslope1;
                cx2 -= invslope2;
            }
        };

    if p2.y == p3.y {
        draw_flat_bottom(p1, p2, p3, w1, w2, w3, uv1, uv2, uv3, fb, coverage);
    } else if p1.y == p2.y {
        draw_flat_top(p1, p2, p3, w1, w2, w3, uv1, uv2, uv3, fb, coverage);
    } else {
        let dy31 = (p3.y - p1.y) as f32;
        let dy21 = (p2.y - p1.y) as f32;
        let t = dy21 / dy31;
        let p4x = p1.x + ((p3.x - p1.x) as f32 * t) as i32;
        let p4 = nalgebra::Point2::new(p4x, p2.y);
        let w4 = w1 + (w3 - w1) * t;
        let uv4 = [
            uv1[0] + (uv3[0] - uv1[0]) * t,
            uv1[1] + (uv3[1] - uv1[1]) * t,
        ];
        draw_flat_bottom(p1, p2, p4, w1, w2, w4, uv1, uv2, uv4, fb, coverage);
        draw_flat_top(p2, p4, p3, w2, w4, w3, uv2, uv4, uv3, fb, coverage);
    }
}
