//! Procedural sky background drawn before world geometry.

use core::fmt::Debug;

use embedded_graphics_core::{
    Pixel,
    draw_target::DrawTarget,
    pixelcolor::Rgb565,
    prelude::{OriginDimensions, Point},
};

use super::post::{apply_post, blend_rgb565};
use crate::{
    error::RenderError, pipeline::shade::retro::palette::PaletteMode,
    pipeline::shade::retro::sky::SkyConfig, pipeline::shade::retro::tint::ScreenTint,
};

#[inline]
pub(super) fn stripe_on_at(x: i32, scroll: i32, stripe_w: i32) -> bool {
    (((x + scroll).div_euclid(stripe_w)) & 1) == 0
}

pub(super) fn draw_sky_background<D>(
    fb: &mut D,
    width: usize,
    height: usize,
    sky: SkyConfig,
    camera_dir: [f32; 3],
    screen_tint: Option<ScreenTint>,
    palette_mode: PaletteMode,
) -> Result<(), RenderError>
where
    D: DrawTarget<Color = Rgb565> + OriginDimensions,
    D::Error: Debug,
{
    let w = width as i32;
    let h = height as i32;
    if w <= 0 || h <= 0 {
        return Ok(());
    }

    let horizon = (h as f32 * (0.5 + camera_dir[1].clamp(-1.0, 1.0) * 0.25)) as i32;
    let stripe_w = sky.stripe_width.max(1) as i32;
    let scroll = (camera_dir[0] * 128.0) as i32;
    let stripe_fade_span = (h / 6).max(1);

    for y in 0..h {
        let dy = (y - horizon + h / 2).clamp(0, h);
        let t_q8 = ((dy as i64 * 255) / h.max(1) as i64) as u16;
        let base = blend_rgb565(sky.top_color, sky.bottom_color, t_q8);
        let below_horizon = (y - horizon).max(0);
        let stripe_strength = if below_horizon == 0 {
            sky.stripe_strength as u16
        } else if below_horizon >= stripe_fade_span {
            0
        } else {
            let rem = stripe_fade_span - below_horizon;
            ((sky.stripe_strength as i32 * rem) / stripe_fade_span) as u16
        };
        for x in 0..w {
            let stripe_on = stripe_on_at(x, scroll, stripe_w);
            let mut color = if stripe_on && stripe_strength > 0 {
                blend_rgb565(base, sky.stripe_color, stripe_strength)
            } else {
                base
            };
            color = apply_post(color, screen_tint, palette_mode);
            fb.draw_iter([Pixel(Point::new(x, y), color)])
                .map_err(|_| RenderError::InvalidInput("draw target rejected sky write"))?;
        }
    }
    Ok(())
}
