//! Texture UV mapping style and distance-based texture LOD.

use embedded_graphics_core::pixelcolor::Rgb565;

/// Texture coordinate interpolation style for textured triangles.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum TextureMapping {
    /// Perspective-correct interpolation using clip-space W.
    PerspectiveCorrect,
    /// Affine interpolation in screen space (retro "texture swim").
    Affine,
}

/// Configuration for distance-based texture LOD and flat-color shedding.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TextureLodConfig {
    /// Distance below which textures render with full detail.
    pub near_distance: f32,
    /// Distance at or beyond which textures are dropped and rendered as solid color.
    pub far_distance: f32,
    /// Fallback flat material color when texture is dropped.
    pub fallback_color: Rgb565,
}

impl TextureLodConfig {
    /// Create a new texture LOD threshold configuration.
    #[must_use]
    pub const fn new(near_distance: f32, far_distance: f32, fallback_color: Rgb565) -> Self {
        Self {
            near_distance,
            far_distance,
            fallback_color,
        }
    }

    /// Evaluates whether a triangle at depth `z` should drop texture sampling.
    #[must_use]
    #[inline]
    pub fn should_drop_texture(&self, z: f32) -> bool {
        z >= self.far_distance
    }

    /// Evaluates if `z` is in the transition crossfade band [near_distance, far_distance).
    #[must_use]
    #[inline]
    pub fn is_in_transition(&self, z: f32) -> bool {
        z >= self.near_distance && z < self.far_distance
    }

    /// Compute blend factor [0.0, 1.0] toward flat color.
    #[must_use]
    #[inline]
    pub fn flat_blend_factor(&self, z: f32) -> f32 {
        if z <= self.near_distance {
            0.0
        } else if z >= self.far_distance {
            1.0
        } else {
            (z - self.near_distance) / (self.far_distance - self.near_distance)
        }
    }
}
