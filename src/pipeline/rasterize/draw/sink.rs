//! Hardware rasterization sink hook.
//!
//! Lets a hardware block take over the engine's flat-shaded primitives -- triangles
//! and lines -- instead of the CPU rasterizer.
//!
//! Triangles are offered *after* culling and *after* depth has been resolved through
//! the state's depth mode and bias, so the sink sees exactly the triangle the CPU
//! rasterizer would have drawn. Lines are offered with their final screen-space
//! endpoints.
//!
//! A sink that accepts a primitive takes responsibility for drawing it; the CPU
//! path is then skipped for that primitive only. The contract is the same
//! `true` = handled / `false` = fall back as
//! [`HardwareAccelerator`](crate::pipeline::output::display_backend::HardwareAccelerator),
//! so one backend can implement both.
//!
//! Methods take `&self` so the sink can live in [`RasterState`], which is `Copy`
//! and passed by shared reference all the way down the rasterizer. Implementations
//! that need to accumulate state use interior mutability -- for a GPU that means
//! buffering primitives and submitting them in one batch, which is the whole point:
//! per-primitive submission would cost more in command overhead than the
//! rasterization it saves.
//!
//! [`RasterState`]: crate::pipeline::rasterize::draw::state::RasterState

use core::fmt::Debug;

use embedded_graphics_core::pixelcolor::Rgb565;
use nalgebra::Point2;

/// A hardware sink for flat-shaded rasterization primitives.
pub trait RasterSink: Debug {
    /// Offer one triangle to hardware.
    ///
    /// * `points` -- screen-space vertices, already transformed, sorted and culled
    /// * `depths` -- per-vertex depth in `points` order, already resolved through
    ///   the state's depth interpolation mode and depth bias
    /// * `color` -- flat colour for the triangle
    ///
    /// Return `true` if the triangle was consumed, `false` to let the CPU
    /// rasterizer draw it -- the right answer for anything the sink cannot
    /// represent, for example once its own buffer is full.
    fn triangle(&self, points: &[Point2<i32>; 3], depths: &[f32; 3], color: Rgb565) -> bool;

    /// Offer one line to hardware.
    ///
    /// `a` and `b` are final screen-space endpoints in the destination's coordinate
    /// space. Return `true` if the line was consumed, `false` to fall back to the
    /// CPU Bresenham path.
    fn line(&self, a: Point2<i32>, b: Point2<i32>, color: Rgb565) -> bool;
}
