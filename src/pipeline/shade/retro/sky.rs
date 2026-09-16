//! Procedural sky background gradient.

use embedded_graphics_core::pixelcolor::Rgb565;

/// Procedural sky background rendered before world geometry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SkyConfig {
    pub top_color: Rgb565,
    pub bottom_color: Rgb565,
    pub stripe_color: Rgb565,
    pub stripe_strength: u8,
    pub stripe_width: u8,
}

impl SkyConfig {
    /// Classic blue gradient with a bright horizon band.
    #[must_use]
    pub const fn retro_blue() -> Self {
        Self {
            top_color: Rgb565::new(6, 16, 31),
            bottom_color: Rgb565::new(1, 4, 12),
            stripe_color: Rgb565::new(18, 30, 31),
            stripe_strength: 18,
            stripe_width: 16,
        }
    }
}
