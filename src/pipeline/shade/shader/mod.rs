//! Composable per-fragment shading for the custom-material path.
//!
//! # Two rendering layers
//!
//! The engine has two complementary raster layers, and it is worth being
//! precise about which one a given call goes through:
//!
//! * **[`crate::pipeline::rasterize::draw`]** — the built-in fast path used by
//!   [`K3dengine`](crate::engine::K3dengine)`::record`/`execute`. It consumes
//!   [`DrawPrimitive`](crate::pipeline::assemble::primitive::DrawPrimitive) command-buffer entries
//!   and has a hand-specialised rasterizer for every built-in [`RenderMode`].
//! * **This module** — a generic [`FragmentShader`] seam over
//!   `raster::triangle` for *your own* materials. Decorators
//!   ([`FogShader`], [`DitherShader`], [`ScreenTintShader`], …) wrap an inner
//!   shader and add a post-step; the compiler monomorphises the whole chain
//!   away, so a decorated built-in shader costs the same as the hand-written
//!   path.
//!
//! Both layers share one set of config types: [`FogConfig`] and [`DitherConfig`]
//! live in [`crate::pipeline::effects`].
//!
//! [`RenderMode`]: crate::pipeline::vertex::mesh::RenderMode

use crate::ZDepth;
use crate::pipeline::effects::{DitherConfig, FogConfig};
use embedded_graphics_core::pixelcolor::Rgb565;

// Compile-time proof that the decorator shaders and the built-in `draw` path
// share one `FogConfig` / `DitherConfig` definition. If either type is ever
// forked again these two lines stop compiling.
const _: fn(crate::pipeline::effects::FogConfig) -> FogConfig = core::convert::identity;
const _: fn(crate::pipeline::effects::DitherConfig) -> DitherConfig = core::convert::identity;

pub mod blend;
pub mod depth_darken;
pub mod dither;
pub mod fog;
pub mod retro;
pub mod screen_door;
pub mod water_reflect;

pub use blend::{
    fast_blend_rgb565, fast_blend_rgba8888, fast_blend_rgba8888_to_rgb565, reverse_color_rgb565,
    reverse_color_rgba8888,
};
pub use depth_darken::{DepthDarkenConfig, DepthDarkenShader};
pub use dither::DitherShader;
pub use fog::FogShader;
pub use retro::{PaletteShader, ScreenTintShader};
pub use screen_door::ScreenDoorShader;
pub use water_reflect::{WaterReflectConfig, WaterReflectShader};

/// Zero-cost composable fragment shader interface.
pub trait FragmentShader {
    type Interpolants: Copy;

    /// Evaluate fragment color at `(x, y)` with depth `z` and per-pixel interpolants.
    fn shade(&self, x: i32, y: i32, z: ZDepth, interps: Self::Interpolants) -> Rgb565;
}

/// Constant flat color fragment shader.
#[derive(Debug, Clone, Copy)]
pub struct FlatColorShader {
    pub color: Rgb565,
}

impl FragmentShader for FlatColorShader {
    type Interpolants = ();

    #[inline(always)]
    fn shade(&self, _x: i32, _y: i32, _z: ZDepth, _interps: ()) -> Rgb565 {
        self.color
    }
}

/// Gouraud vertex-interpolated color fragment shader.
#[derive(Debug, Clone, Copy)]
pub struct GouraudShader;

impl FragmentShader for GouraudShader {
    type Interpolants = Rgb565;

    #[inline(always)]
    fn shade(&self, _x: i32, _y: i32, _z: ZDepth, color: Rgb565) -> Rgb565 {
        color
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};

    #[test]
    fn flat_color_shader_returns_constant_color() {
        let shader = FlatColorShader { color: Rgb565::RED };
        let out = shader.shade(5, 10, 0, ());
        assert_eq!(out, Rgb565::RED);
    }

    #[test]
    fn gouraud_shader_returns_interpolant() {
        let shader = GouraudShader;
        let out = shader.shade(0, 0, 0, Rgb565::BLUE);
        assert_eq!(out, Rgb565::BLUE);
    }
}
