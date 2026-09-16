//! Coarse retro-style preset bundle tying the individual effects together.

use embedded_graphics_core::pixelcolor::Rgb565;

use crate::pipeline::effects::{DitherConfig, FogConfig};

use super::light_levels::LightLevels;
use super::palette::PaletteMode;
use super::sky::SkyConfig;
use super::stipple::StippleMode;
use super::texture_lod::TextureMapping;
use super::tint::ScreenTint;

/// Coarse visual-style controls for retro rendering presets.
#[derive(Debug, Clone, Copy)]
pub struct RetroStyle {
    /// Optional depth fog.
    pub fog: Option<FogConfig>,
    /// Optional ordered dithering.
    pub dither: Option<DitherConfig>,
    /// NDC snap precision in fractional bits. `0` disables snapping.
    pub vertex_snap_bits: u8,
    /// UV interpolation mode for textured surfaces.
    pub texture_mapping: TextureMapping,
    /// Sector light behavior.
    pub light_levels: LightLevels,
    /// Optional checkerboard stippling for fake transparency.
    pub stipple_mode: StippleMode,
    /// Optional full-screen tint.
    pub screen_tint: Option<ScreenTint>,
    /// Optional palette quantization.
    pub palette_mode: PaletteMode,
    /// Optional procedural sky.
    pub sky: Option<SkyConfig>,
}

impl Default for RetroStyle {
    fn default() -> Self {
        Self::modern()
    }
}

impl RetroStyle {
    /// Neutral defaults: no extra post-process and perspective-correct texturing.
    #[must_use]
    pub const fn modern() -> Self {
        Self {
            fog: None,
            dither: None,
            vertex_snap_bits: 0,
            texture_mapping: TextureMapping::PerspectiveCorrect,
            light_levels: LightLevels::Linear,
            stipple_mode: StippleMode::Off,
            screen_tint: None,
            palette_mode: PaletteMode::Off,
            sky: None,
        }
    }

    /// Doom-leaning preset for coarse affine textures without fog.
    #[must_use]
    pub const fn doom_walkable() -> Self {
        Self {
            fog: None,
            dither: Some(DitherConfig { intensity: 20 }),
            vertex_snap_bits: 0,
            texture_mapping: TextureMapping::Affine,
            light_levels: LightLevels::Doom32,
            stipple_mode: StippleMode::Off,
            screen_tint: None,
            palette_mode: PaletteMode::Rgb332,
            sky: Some(SkyConfig::retro_blue()),
        }
    }

    /// PSX-leaning preset: snapped vertices + affine textures + visible dither.
    #[must_use]
    pub fn psx() -> Self {
        Self {
            fog: Some(FogConfig::new(Rgb565::new(2, 2, 4), 6.0, 20.0)),
            dither: Some(DitherConfig::new(72)),
            vertex_snap_bits: 6,
            texture_mapping: TextureMapping::Affine,
            light_levels: LightLevels::Linear,
            stipple_mode: StippleMode::Off,
            screen_tint: None,
            palette_mode: PaletteMode::Rgb332,
            sky: Some(SkyConfig::retro_blue()),
        }
    }
}
