//! Full-screen tint blended onto final raster colours.

use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::pixelcolor::RgbColor;

/// Full-screen tint blended onto final raster colors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScreenTint {
    pub color: Rgb565,
    /// Blend strength in [0, 255].
    pub strength: u8,
}

impl ScreenTint {
    #[must_use]
    #[inline]
    pub fn apply(&self, base: Rgb565) -> Rgb565 {
        let a = self.strength as u16;
        let inv = 255u16.saturating_sub(a);
        let r = ((base.r() as u16 * inv + self.color.r() as u16 * a) / 255) as u8;
        let g = ((base.g() as u16 * inv + self.color.g() as u16 * a) / 255) as u8;
        let b = ((base.b() as u16 * inv + self.color.b() as u16 * a) / 255) as u8;
        Rgb565::new(r, g, b)
    }
}
