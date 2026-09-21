//! Hardware triangle sink hook.
//!
//! Lets a hardware block take over flat-coloured, depth-tested triangle
//! rasterization. The engine offers every fully transformed triangle to the sink
//! *after* culling and *after* depth has been resolved through the state's depth
//! mode and bias, so the sink sees exactly the triangle the CPU rasterizer would
//! have drawn. A sink that accepts a triangle takes responsibility for drawing it;
//! the CPU rasterizer is then skipped for that triangle only.
//!
//! The contract is deliberately the same `true` = handled / `false` = fall back as
//! [`HardwareAccelerator`](crate::pipeline::output::display_backend::HardwareAccelerator),
//! so one backend can implement both.
//!
//! The method takes `&self` so the sink can live in [`RasterState`], which is
//! `Copy` and passed by shared reference all the way down the rasterizer.
//! Implementations that need to accumulate state use interior mutability -- for a
//! GPU that means buffering triangles and submitting them in one batch, which is
//! the whole point: per-triangle submission would cost more in command overhead
//! than the rasterization it saves.
//!
//! [`RasterState`]: crate::pipeline::rasterize::draw::state::RasterState

use core::fmt::Debug;

use embedded_graphics_core::pixelcolor::Rgb565;
use nalgebra::Point2;

/// A hardware sink for flat-shaded, screen-space triangles.
pub trait TriangleSink: Debug {
    /// Offer one triangle to hardware.
    ///
    /// * `points` -- screen-space vertices, already transformed, sorted and culled
    /// * `depths` -- per-vertex depth in `points` order, already resolved through
    ///   the state's depth interpolation mode and depth bias
    /// * `color` -- flat colour for the triangle
    ///
    /// Return `true` if the triangle was consumed. Return `false` to let the CPU
    /// rasterizer draw it -- the right answer for anything the sink cannot
    /// represent, for example once its own buffer is full.
    fn triangle(&self, points: &[Point2<i32>; 3], depths: &[f32; 3], color: Rgb565) -> bool;
}
