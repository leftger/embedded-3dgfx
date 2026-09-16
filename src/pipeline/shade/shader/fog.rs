//! Depth-based fog.
//!
//! [`FogConfig`] is defined once in [`crate::pipeline::effects`] and re-exported
//! here so the [`FogShader`] decorator and the built-in `draw` path can never
//! drift apart.

use embedded_graphics_core::pixelcolor::Rgb565;

use crate::pipeline::effects::FogConfig;

/// Fragment shader decorator that applies depth-based fog.
#[derive(Debug, Clone, Copy)]
pub struct FogShader<'a, S> {
    pub inner: S,
    pub fog: &'a FogConfig,
}

impl<'a, S: super::FragmentShader> super::FragmentShader for FogShader<'a, S> {
    type Interpolants = S::Interpolants;

    #[inline(always)]
    fn shade(&self, x: i32, y: i32, z: crate::ZDepth, interps: Self::Interpolants) -> Rgb565 {
        let base = self.inner.shade(x, y, z, interps);
        self.fog.apply(base, u32::from(z))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::shade::shader::{FlatColorShader, FragmentShader};
    use embedded_graphics_core::pixelcolor::RgbColor;

    #[test]
    fn test_fog_interpolation() {
        let fog_color = Rgb565::WHITE;
        let base_color = Rgb565::BLACK;
        let fog = FogConfig::new(fog_color, 1.0, 10.0);

        // Near or closer: no fog
        assert_eq!(fog.apply(base_color, (1.0 * 65536.0) as u32), base_color);
        assert_eq!(fog.apply(base_color, 0), base_color);

        // Far or farther: full fog
        assert_eq!(fog.apply(base_color, (10.0 * 65536.0) as u32), fog_color);
        assert_eq!(fog.apply(base_color, (20.0 * 65536.0) as u32), fog_color);

        // Halfway between near and far: 50% fog
        let mid_depth = (5.5 * 65536.0) as u32;
        let mid_color = fog.apply(base_color, mid_depth);
        assert!(mid_color.r() > 10 && mid_color.r() < 25);
    }

    #[test]
    fn test_fog_shader_decorator() {
        let fog = FogConfig::new(Rgb565::WHITE, 1.0, 5.0);
        let base_shader = FlatColorShader { color: Rgb565::RED };
        let fog_shader = FogShader {
            inner: base_shader,
            fog: &fog,
        };

        // Shading at near depth should return flat base color
        let near_depth = crate::to_zdepth(65536);
        assert_eq!(fog_shader.shade(0, 0, near_depth, ()), Rgb565::RED);
    }
}
