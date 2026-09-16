//! Ordered (Bayer 4x4) dithering.
//!
//! [`DitherConfig`] is defined once in [`crate::pipeline::effects`] and re-exported
//! here so the [`DitherShader`] decorator and the built-in `draw` path can
//! never drift apart.

use embedded_graphics_core::pixelcolor::Rgb565;

use crate::pipeline::effects::DitherConfig;

/// Fragment shader decorator that applies Bayer 4x4 dithering.
#[derive(Debug, Clone, Copy)]
pub struct DitherShader<'a, S> {
    pub inner: S,
    pub dither: &'a DitherConfig,
}

impl<'a, S: super::FragmentShader> super::FragmentShader for DitherShader<'a, S> {
    type Interpolants = S::Interpolants;

    #[inline(always)]
    fn shade(&self, x: i32, y: i32, z: crate::ZDepth, interps: Self::Interpolants) -> Rgb565 {
        let base = self.inner.shade(x, y, z, interps);
        self.dither.apply(base, x, y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::shade::shader::{FlatColorShader, FragmentShader};
    use embedded_graphics_core::pixelcolor::RgbColor;

    #[test]
    fn test_dither_intensity_zero_is_identity() {
        let dither = DitherConfig::new(0);
        let color = Rgb565::RED;
        assert_eq!(dither.apply(color, 0, 0), color);
        assert_eq!(dither.apply(color, 12, 15), color);
    }

    #[test]
    fn test_dither_pattern_varies_with_pixel_coordinates() {
        let dither = DitherConfig::new(128);
        let color = Rgb565::new(16, 32, 16);
        let c0 = dither.apply(color, 0, 0);
        let c1 = dither.apply(color, 1, 1);
        // Different matrix positions should produce different dither results
        assert_ne!(c0, c1);
    }

    #[test]
    fn test_dither_shader_decorator() {
        let dither = DitherConfig::new(64);
        let base_shader = FlatColorShader {
            color: Rgb565::GREEN,
        };
        let dither_shader = DitherShader {
            inner: base_shader,
            dither: &dither,
        };
        let shaded = dither_shader.shade(2, 3, crate::to_zdepth(100), ());
        assert!(shaded.g() > 0);
    }
}
