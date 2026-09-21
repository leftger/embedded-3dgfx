//! Shared per-pass rasterization state.
//!
//! Every rasterizer used to take its configuration as a tail of 6–9 positional
//! arguments (`width`, `fog`, `dither`, `texture_mapping`, `stipple`, `tint`,
//! `palette`, …). [`RasterState`] bundles that tail into one borrowed context so
//! a fill path takes a single `&RasterState` instead of a wide, easy-to-misorder
//! parameter list.
//!
//! The same bundle is what the execute driver threads through, so "how this pass
//! should look" has exactly one representation across the whole rasterize stage
//! instead of one per entry point.
//!
//! ```
//! use embedded_3dgfx::pipeline::effects::{DitherConfig, FogConfig};
//! use embedded_3dgfx::pipeline::rasterize::draw::RasterState;
//! use embedded_graphics_core::pixelcolor::Rgb565;
//!
//! let fog = FogConfig::new(Rgb565::new(0, 0, 0), 4.0, 24.0);
//! let dither = DitherConfig::new(32);
//! let state = RasterState::new(320, 240)
//!     .with_fog(Some(&fog))
//!     .with_dither(Some(&dither));
//! assert_eq!(state.width, 320);
//! ```

use crate::pipeline::effects::{DepthBias, DepthInterpolationMode, DitherConfig, FogConfig};
use crate::pipeline::shade::retro::palette::PaletteMode;
use crate::pipeline::shade::retro::sky::SkyConfig;
use crate::pipeline::shade::retro::stipple::StippleMode;
use crate::pipeline::shade::retro::texture_lod::TextureMapping;
use crate::pipeline::shade::retro::tint::ScreenTint;
use nalgebra::Point2;

/// Immutable configuration threaded through the triangle rasterizers.
///
/// `Copy` and cheap to pass by reference; all fields are scalars or shared
/// borrows, so building one per frame costs nothing measurable.
#[derive(Debug, Clone, Copy)]
pub struct RasterState<'a> {
    /// Framebuffer width in pixels.
    pub width: usize,
    /// Framebuffer height in pixels.
    pub height: usize,
    /// Depth-based fog post-process.
    pub fog: Option<&'a FogConfig>,
    /// Ordered dithering post-process.
    pub dither: Option<&'a DitherConfig>,
    /// Full-screen tint blended onto final raster colours.
    pub screen_tint: Option<ScreenTint>,
    /// Output palette quantization.
    pub palette_mode: PaletteMode,
    /// Screen-door stipple mode.
    pub stipple_mode: StippleMode,
    /// Texture UV interpolation mode.
    pub texture_mapping: TextureMapping,
    /// Optional constant depth bias applied per triangle.
    pub depth_bias: Option<DepthBias>,
    /// Depth interpolation strategy.
    pub depth_mode: DepthInterpolationMode,
    /// Procedural sky drawn before world geometry, if enabled.
    pub sky: Option<SkyConfig>,
    /// Camera forward direction, used to orient the sky gradient.
    pub camera_dir: [f32; 3],
    /// Optional hardware sink for flat-shaded triangles. `None` -- the default --
    /// rasterizes on the CPU.
    pub triangles: Option<&'a dyn crate::pipeline::rasterize::draw::sink::TriangleSink>,
}

impl Default for RasterState<'_> {
    fn default() -> Self {
        Self {
            width: 0,
            height: 0,
            fog: None,
            dither: None,
            screen_tint: None,
            palette_mode: PaletteMode::Off,
            stipple_mode: StippleMode::Off,
            texture_mapping: TextureMapping::PerspectiveCorrect,
            depth_bias: None,
            depth_mode: DepthInterpolationMode::Exact,
            sky: None,
            camera_dir: [0.0, 0.0, -1.0],
            triangles: None,
        }
    }
}

impl<'a> RasterState<'a> {
    /// Default state for a `width` × `height` framebuffer.
    pub const fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            fog: None,
            dither: None,
            screen_tint: None,
            palette_mode: PaletteMode::Off,
            stipple_mode: StippleMode::Off,
            texture_mapping: TextureMapping::PerspectiveCorrect,
            depth_bias: None,
            depth_mode: DepthInterpolationMode::Exact,
            sky: None,
            camera_dir: [0.0, 0.0, -1.0],
            triangles: None,
        }
    }

    /// State sized from a z-buffer slice, deriving the height as `len / width`.
    pub fn from_zbuffer(width: usize, zbuffer_len: usize) -> Self {
        let height = if width == 0 { 0 } else { zbuffer_len / width };
        Self::new(width, height)
    }

    #[must_use]
    pub const fn with_fog(mut self, fog: Option<&'a FogConfig>) -> Self {
        self.fog = fog;
        self
    }

    #[must_use]
    pub const fn with_dither(mut self, dither: Option<&'a DitherConfig>) -> Self {
        self.dither = dither;
        self
    }

    #[must_use]
    pub const fn with_screen_tint(mut self, tint: Option<ScreenTint>) -> Self {
        self.screen_tint = tint;
        self
    }

    #[must_use]
    pub const fn with_palette_mode(mut self, mode: PaletteMode) -> Self {
        self.palette_mode = mode;
        self
    }

    #[must_use]
    pub const fn with_stipple_mode(mut self, mode: StippleMode) -> Self {
        self.stipple_mode = mode;
        self
    }

    #[must_use]
    pub const fn with_texture_mapping(mut self, mapping: TextureMapping) -> Self {
        self.texture_mapping = mapping;
        self
    }

    #[must_use]
    pub const fn with_depth_bias(mut self, bias: Option<DepthBias>) -> Self {
        self.depth_bias = bias;
        self
    }

    #[must_use]
    pub const fn with_depth_mode(mut self, mode: DepthInterpolationMode) -> Self {
        self.depth_mode = mode;
        self
    }

    #[must_use]
    pub const fn with_sky(mut self, sky: Option<SkyConfig>) -> Self {
        self.sky = sky;
        self
    }

    #[must_use]
    pub const fn with_camera_dir(mut self, camera_dir: [f32; 3]) -> Self {
        self.camera_dir = camera_dir;
        self
    }

    /// Route flat-shaded triangles to a hardware sink instead of the CPU
    /// rasterizer. `None` -- the default -- keeps everything on the CPU.
    ///
    /// [`TriangleSink`]: crate::pipeline::rasterize::draw::sink::TriangleSink
    #[must_use]
    pub const fn with_triangle_sink(
        mut self,
        sink: Option<&'a dyn crate::pipeline::rasterize::draw::sink::TriangleSink>,
    ) -> Self {
        self.triangles = sink;
        self
    }
}

/// Permutation `[i0, i1, i2]` that orders three points by ascending `y`.
///
/// Callers apply the result to *every* parallel per-vertex array via
/// [`apply_order`], which keeps the sort logic in one place regardless of how
/// many attributes a triangle carries.
#[inline]
pub(crate) fn ascending_y_order(points: &[Point2<i32>; 3]) -> [usize; 3] {
    let mut order = [0usize, 1, 2];
    let y = |i: usize| points[i].y;
    if y(order[0]) > y(order[1]) {
        order.swap(0, 1);
    }
    if y(order[0]) > y(order[2]) {
        order.swap(0, 2);
    }
    if y(order[1]) > y(order[2]) {
        order.swap(1, 2);
    }
    order
}

/// Reorder a three-element per-vertex array by a permutation from
/// [`ascending_y_order`].
#[inline]
pub(crate) fn apply_order<T: Copy>(arr: &mut [T; 3], order: [usize; 3]) {
    let src = *arr;
    arr[0] = src[order[0]];
    arr[1] = src[order[1]];
    arr[2] = src[order[2]];
}

/// Conservative whole-triangle reject: true only when all three vertices are
/// outside the same screen edge.
#[inline]
pub(crate) fn tri_fully_off_screen(points: &[Point2<i32>; 3], width: usize, height: usize) -> bool {
    let [p1, p2, p3] = *points;
    let w = width as i32;
    let h = height as i32;
    (p1.x < 0 && p2.x < 0 && p3.x < 0)
        || (p1.x >= w && p2.x >= w && p3.x >= w)
        || (p1.y < 0 && p2.y < 0 && p3.y < 0)
        || (p1.y >= h && p2.y >= h && p3.y >= h)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pts(y0: i32, y1: i32, y2: i32) -> [Point2<i32>; 3] {
        [Point2::new(0, y0), Point2::new(1, y1), Point2::new(2, y2)]
    }

    #[test]
    fn default_state_disables_effects() {
        let s = RasterState::new(8, 4);
        assert_eq!((s.width, s.height), (8, 4));
        assert!(s.fog.is_none());
        assert!(s.dither.is_none());
        assert!(s.screen_tint.is_none());
        assert_eq!(s.stipple_mode, StippleMode::Off);
        assert_eq!(s.palette_mode, PaletteMode::Off);
        assert!(s.sky.is_none());
        assert_eq!(s.camera_dir, [0.0, 0.0, -1.0]);
    }

    #[test]
    fn from_zbuffer_derives_height() {
        let s = RasterState::from_zbuffer(16, 64);
        assert_eq!((s.width, s.height), (16, 4));
        // Guard against divide-by-zero on a degenerate buffer.
        assert_eq!(RasterState::from_zbuffer(0, 64).height, 0);
    }

    #[test]
    fn ascending_y_order_sorts_and_permutes_payload() {
        let points = pts(30, 10, 20);
        let order = ascending_y_order(&points);
        // y=10,20,30 -> original indices 1,2,0
        assert_eq!(order, [1, 2, 0]);

        let mut colors = [10u8, 20, 30];
        apply_order(&mut colors, order);
        assert_eq!(colors, [20, 30, 10]);
    }

    #[test]
    fn ascending_y_order_matches_legacy_swap_network() {
        // y = [5, 5, 1]: the (0,2) comparator fires, giving indices [2, 1, 0].
        // This mirrors the historical inline swap network exactly, so tie
        // ordering is unchanged by the refactor.
        let points = pts(5, 5, 1);
        let order = ascending_y_order(&points);
        assert_eq!(order, [2, 1, 0]);
        let mut ys: [i32; 3] = [5, 5, 1];
        apply_order(&mut ys, order);
        assert_eq!(ys, [1, 5, 5]);
    }

    #[test]
    fn cull_rejects_only_fully_outside_triangles() {
        assert!(tri_fully_off_screen(&pts(-5, -4, -3), 16, 16));
        assert!(!tri_fully_off_screen(&pts(-5, -4, 3), 16, 16));
        assert!(tri_fully_off_screen(&pts(20, 21, 22), 16, 16));
    }
}
