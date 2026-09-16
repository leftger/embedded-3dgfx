//! Standard Z-buffered triangle rasterization, Gouraud shading, and translucent triangles.

use core::fmt::Debug;
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::pixelcolor::Rgb565;

use super::state::{RasterState, apply_order, ascending_y_order, tri_fully_off_screen};
use crate::pipeline::assemble::primitive::DrawPrimitive;
use crate::pipeline::effects::{DepthBias, DepthInterpolationMode, DitherConfig, FogConfig};

pub(crate) mod flat;
#[cfg(feature = "lighting")]
pub(crate) mod gouraud;
pub(crate) mod screendoor;
pub(crate) mod translucent;

use flat::fill_triangle_zbuffered;
#[cfg(feature = "lighting")]
use gouraud::fill_triangle_zbuffered_gouraud;
use screendoor::fill_triangle_zbuffered_screendoor;
use translucent::fill_triangle_zbuffered_translucent;

/// Render a primitive with Z-buffering and default raster state.
#[inline]
pub fn draw_zbuffered<D: DrawTarget<Color = Rgb565>>(
    primitive: DrawPrimitive,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    width: usize,
) where
    <D as DrawTarget>::Error: Debug,
{
    let state = RasterState::from_zbuffer(width, zbuffer.len());
    draw_zbuffered_with_state(primitive, fb, zbuffer, &state);
}

/// Apply the per-triangle depth interpolation mode and optional constant bias.
#[inline]
fn resolve_depths(
    raw: [f32; 3],
    points: &[nalgebra::Point2<i32>; 3],
    state: &RasterState<'_>,
) -> (f32, f32, f32) {
    let (z1, z2, z3) = state.depth_mode.process_depths(raw[0], raw[1], raw[2]);
    match state.depth_bias {
        Some(b) => b.apply(points[0], points[1], points[2], z1, z2, z3),
        None => (z1, z2, z3),
    }
}

/// Render a primitive with Z-buffering, using fog / dither / depth settings
/// from `state`.
///
/// This is the single entry point the old `draw_zbuffered`,
/// `_with_effects`, `_with_options`, and `_with_bias` wrappers all forwarded to;
/// they are now expressed as [`RasterState`] builders instead of a chain of
/// near-identical functions.
pub fn draw_zbuffered_with_state<D: DrawTarget<Color = Rgb565>>(
    primitive: DrawPrimitive,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    state: &RasterState<'_>,
) where
    <D as DrawTarget>::Error: Debug,
{
    match primitive {
        DrawPrimitive::ColoredTriangleWithDepth {
            mut points,
            depths: mut raw_depths,
            color,
        } => {
            let order = ascending_y_order(&points);
            apply_order(&mut points, order);
            apply_order(&mut raw_depths, order);
            if tri_fully_off_screen(&points, state.width, state.height) {
                return;
            }
            let (z1, z2, z3) = resolve_depths(raw_depths, &points, state);
            let [p1, p2, p3] = points;
            fill_triangle_zbuffered(
                p1,
                p2,
                p3,
                z1,
                z2,
                z3,
                color,
                fb,
                zbuffer,
                state.width,
                state.fog,
                state.dither,
            );
        }

        DrawPrimitive::TranslucentTriangleWithDepth {
            mut points,
            depths: mut raw_depths,
            color,
            alpha,
        } => {
            let order = ascending_y_order(&points);
            apply_order(&mut points, order);
            apply_order(&mut raw_depths, order);
            if tri_fully_off_screen(&points, state.width, state.height) {
                return;
            }
            let (z1, z2, z3) = resolve_depths(raw_depths, &points, state);
            let [p1, p2, p3] = points;
            fill_triangle_zbuffered_translucent(
                p1,
                p2,
                p3,
                z1,
                z2,
                z3,
                color,
                alpha,
                fb,
                zbuffer,
                state.width,
            );
        }

        DrawPrimitive::ScreenDoorTriangleWithDepth {
            mut points,
            depths: mut raw_depths,
            color,
            alpha,
        } => {
            let order = ascending_y_order(&points);
            apply_order(&mut points, order);
            apply_order(&mut raw_depths, order);
            if tri_fully_off_screen(&points, state.width, state.height) {
                return;
            }
            let (z1, z2, z3) = resolve_depths(raw_depths, &points, state);
            let [p1, p2, p3] = points;
            fill_triangle_zbuffered_screendoor(
                p1,
                p2,
                p3,
                z1,
                z2,
                z3,
                color,
                alpha,
                fb,
                zbuffer,
                state.width,
            );
        }

        #[cfg(feature = "lighting")]
        DrawPrimitive::GouraudTriangleWithDepth {
            mut points,
            depths: mut raw_depths,
            mut colors,
        } => {
            let order = ascending_y_order(&points);
            apply_order(&mut points, order);
            apply_order(&mut raw_depths, order);
            apply_order(&mut colors, order);
            if tri_fully_off_screen(&points, state.width, state.height) {
                return;
            }
            let (z1, z2, z3) = resolve_depths(raw_depths, &points, state);
            let [p1, p2, p3] = points;
            let [c1, c2, c3] = colors;
            fill_triangle_zbuffered_gouraud(
                p1,
                p2,
                p3,
                z1,
                z2,
                z3,
                c1,
                c2,
                c3,
                fb,
                zbuffer,
                state.width,
                state.fog,
                state.dither,
            );
        }

        _ => super::fill::draw(primitive, fb),
    }
}

/// Compatibility adapter over [`draw_zbuffered_with_state`]: Z-buffer render
/// with optional fog / dither post-processing.
#[inline]
pub fn draw_zbuffered_with_effects<D: DrawTarget<Color = Rgb565>>(
    primitive: DrawPrimitive,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    width: usize,
    fog_config: Option<&FogConfig>,
    dither_config: Option<&DitherConfig>,
) where
    <D as DrawTarget>::Error: Debug,
{
    let state = RasterState::from_zbuffer(width, zbuffer.len())
        .with_fog(fog_config)
        .with_dither(dither_config);
    draw_zbuffered_with_state(primitive, fb, zbuffer, &state);
}

/// Compatibility adapter over [`draw_zbuffered_with_state`]: adds an explicit
/// depth interpolation mode.
#[inline]
pub fn draw_zbuffered_with_options<D: DrawTarget<Color = Rgb565>>(
    primitive: DrawPrimitive,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    width: usize,
    fog_config: Option<&FogConfig>,
    dither_config: Option<&DitherConfig>,
    depth_mode: DepthInterpolationMode,
) where
    <D as DrawTarget>::Error: Debug,
{
    let state = RasterState::from_zbuffer(width, zbuffer.len())
        .with_fog(fog_config)
        .with_dither(dither_config)
        .with_depth_mode(depth_mode);
    draw_zbuffered_with_state(primitive, fb, zbuffer, &state);
}

/// Compatibility adapter over [`draw_zbuffered_with_state`]: adds an optional
/// constant depth bias (decals / overlays).
#[inline]
pub fn draw_zbuffered_with_bias<D: DrawTarget<Color = Rgb565>>(
    primitive: DrawPrimitive,
    fb: &mut D,
    zbuffer: &mut [crate::ZDepth],
    width: usize,
    fog_config: Option<&FogConfig>,
    dither_config: Option<&DitherConfig>,
    depth_mode: DepthInterpolationMode,
    bias: Option<DepthBias>,
) where
    <D as DrawTarget>::Error: Debug,
{
    let state = RasterState::from_zbuffer(width, zbuffer.len())
        .with_fog(fog_config)
        .with_dither(dither_config)
        .with_depth_mode(depth_mode)
        .with_depth_bias(bias);
    draw_zbuffered_with_state(primitive, fb, zbuffer, &state);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::assemble::primitive::DrawPrimitive;
    use crate::pipeline::effects::{DepthInterpolationMode, FogConfig};
    use embedded_graphics_core::Pixel;
    use embedded_graphics_core::geometry::{OriginDimensions, Size};
    use embedded_graphics_core::pixelcolor::RgbColor;

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
    fn test_draw_zbuffered_primitives() {
        let mut fb = TestFb::<20, 20>::default();
        let mut zbuf = [crate::Z_MAX_VALUE; 400];

        // Draw close red triangle at z = 1.0
        let close_tri = DrawPrimitive::ColoredTriangleWithDepth {
            points: [
                nalgebra::Point2::new(10, 2),
                nalgebra::Point2::new(2, 18),
                nalgebra::Point2::new(18, 18),
            ],
            depths: [1.0, 1.0, 1.0],
            color: Rgb565::RED,
        };
        draw_zbuffered(close_tri, &mut fb, &mut zbuf, 20);

        let center_idx = 10 * 20 + 10;
        assert_eq!(fb.pixels[center_idx], Rgb565::RED);

        // Draw far green triangle at z = 5.0 (should be occluded by zbuffer)
        let far_tri = DrawPrimitive::ColoredTriangleWithDepth {
            points: [
                nalgebra::Point2::new(10, 2),
                nalgebra::Point2::new(2, 18),
                nalgebra::Point2::new(18, 18),
            ],
            depths: [5.0, 5.0, 5.0],
            color: Rgb565::GREEN,
        };
        draw_zbuffered(far_tri, &mut fb, &mut zbuf, 20);

        // Center pixel should STILL be red
        assert_eq!(fb.pixels[center_idx], Rgb565::RED);
    }

    #[test]
    fn test_draw_zbuffered_with_options() {
        let mut fb = TestFb::<20, 20>::default();
        let mut zbuf = [crate::Z_MAX_VALUE; 400];

        let tri = DrawPrimitive::ColoredTriangleWithDepth {
            points: [
                nalgebra::Point2::new(10, 2),
                nalgebra::Point2::new(2, 18),
                nalgebra::Point2::new(18, 18),
            ],
            depths: [2.0, 2.0, 2.0],
            color: Rgb565::WHITE,
        };

        let fog = FogConfig::new(Rgb565::BLUE, 1.0, 5.0);

        draw_zbuffered_with_options(
            tri,
            &mut fb,
            &mut zbuf,
            20,
            Some(&fog),
            None,
            DepthInterpolationMode::Exact,
        );

        let center_idx = 10 * 20 + 10;
        assert!(fb.pixels[center_idx].b() > 0);
    }

    #[test]
    fn test_draw_zbuffered_translucent() {
        let mut fb = TestFb::<20, 20>::default();
        let mut zbuf = [crate::Z_MAX_VALUE; 400];

        let trans_tri = DrawPrimitive::TranslucentTriangleWithDepth {
            points: [
                nalgebra::Point2::new(10, 2),
                nalgebra::Point2::new(2, 18),
                nalgebra::Point2::new(18, 18),
            ],
            depths: [1.0, 1.0, 1.0],
            color: Rgb565::RED,
            alpha: 128,
        };
        draw_zbuffered(trans_tri, &mut fb, &mut zbuf, 20);

        let center_idx = 10 * 20 + 10;
        assert!(fb.pixels[center_idx].r() > 0);
    }

    #[test]
    fn test_draw_zbuffered_screendoor() {
        let mut fb = TestFb::<20, 20>::default();
        let mut zbuf = [crate::Z_MAX_VALUE; 400];

        let screendoor_tri = DrawPrimitive::ScreenDoorTriangleWithDepth {
            points: [
                nalgebra::Point2::new(10, 2),
                nalgebra::Point2::new(2, 18),
                nalgebra::Point2::new(18, 18),
            ],
            depths: [1.0, 1.0, 1.0],
            color: Rgb565::GREEN,
            alpha: 128,
        };
        draw_zbuffered(screendoor_tri, &mut fb, &mut zbuf, 20);

        // At least some pixels inside the triangle should be rendered GREEN
        let mut green_count = 0;
        for p in fb.pixels {
            if p == Rgb565::GREEN {
                green_count += 1;
            }
        }
        assert!(green_count > 0);
    }

    #[test]
    fn test_draw_zbuffered_bias() {
        let mut fb = TestFb::<20, 20>::default();
        let mut zbuf = [crate::Z_MAX_VALUE; 400];

        // Draw a base red triangle at depth 2.0
        let base_tri = DrawPrimitive::ColoredTriangleWithDepth {
            points: [
                nalgebra::Point2::new(10, 2),
                nalgebra::Point2::new(2, 18),
                nalgebra::Point2::new(18, 18),
            ],
            depths: [2.0, 2.0, 2.0],
            color: Rgb565::RED,
        };
        draw_zbuffered(base_tri, &mut fb, &mut zbuf, 20);

        let center_idx = 10 * 20 + 10;
        assert_eq!(fb.pixels[center_idx], Rgb565::RED);

        // Draw a coplanar decal blue triangle with depth bias
        let decal_tri = DrawPrimitive::ColoredTriangleWithDepth {
            points: [
                nalgebra::Point2::new(10, 2),
                nalgebra::Point2::new(2, 18),
                nalgebra::Point2::new(18, 18),
            ],
            depths: [2.0, 2.0, 2.0],
            color: Rgb565::BLUE,
        };
        draw_zbuffered_with_bias(
            decal_tri,
            &mut fb,
            &mut zbuf,
            20,
            None,
            None,
            DepthInterpolationMode::Exact,
            Some(DepthBias::decal()),
        );

        // Decal should cleanly overwrite the base triangle due to depth bias
        assert_eq!(fb.pixels[center_idx], Rgb565::BLUE);
    }
}
