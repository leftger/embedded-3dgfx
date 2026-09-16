//! Per-pixel post-processing applied on the way to the framebuffer.

use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};

use crate::{
    pipeline::assemble::primitive::DrawPrimitive, pipeline::shade::retro::palette::PaletteMode,
    pipeline::shade::retro::tint::ScreenTint,
};

#[inline]
pub(super) fn apply_post(
    color: Rgb565,
    tint: Option<ScreenTint>,
    palette_mode: PaletteMode,
) -> Rgb565 {
    let tinted = if let Some(t) = tint {
        t.apply(color)
    } else {
        color
    };
    palette_mode.apply(tinted)
}

pub(super) fn tint_primitive(
    primitive: &DrawPrimitive,
    tint: Option<ScreenTint>,
    palette_mode: PaletteMode,
) -> DrawPrimitive {
    match primitive.clone() {
        DrawPrimitive::ColoredPoint(p, color) => {
            DrawPrimitive::ColoredPoint(p, apply_post(color, tint, palette_mode))
        }
        DrawPrimitive::Line(points, color) => {
            DrawPrimitive::Line(points, apply_post(color, tint, palette_mode))
        }
        DrawPrimitive::ColoredTriangle(points, color) => {
            DrawPrimitive::ColoredTriangle(points, apply_post(color, tint, palette_mode))
        }
        DrawPrimitive::ColoredTriangleWithDepth {
            points,
            depths,
            color,
        } => DrawPrimitive::ColoredTriangleWithDepth {
            points,
            depths,
            color: apply_post(color, tint, palette_mode),
        },
        DrawPrimitive::TranslucentTriangleWithDepth {
            points,
            depths,
            color,
            alpha,
        } => DrawPrimitive::TranslucentTriangleWithDepth {
            points,
            depths,
            color: apply_post(color, tint, palette_mode),
            alpha,
        },
        DrawPrimitive::ScreenDoorTriangleWithDepth {
            points,
            depths,
            color,
            alpha,
        } => DrawPrimitive::ScreenDoorTriangleWithDepth {
            points,
            depths,
            color: apply_post(color, tint, palette_mode),
            alpha,
        },
        #[cfg(feature = "lighting")]
        DrawPrimitive::GouraudTriangle { points, colors } => DrawPrimitive::GouraudTriangle {
            points,
            colors: [
                apply_post(colors[0], tint, palette_mode),
                apply_post(colors[1], tint, palette_mode),
                apply_post(colors[2], tint, palette_mode),
            ],
        },
        #[cfg(feature = "lighting")]
        DrawPrimitive::GouraudTriangleWithDepth {
            points,
            depths,
            colors,
        } => DrawPrimitive::GouraudTriangleWithDepth {
            points,
            depths,
            colors: [
                apply_post(colors[0], tint, palette_mode),
                apply_post(colors[1], tint, palette_mode),
                apply_post(colors[2], tint, palette_mode),
            ],
        },
        #[cfg(feature = "textured")]
        DrawPrimitive::LightmappedTriangle {
            points,
            depths,
            ws,
            surface_uvs,
            lm_uvs,
            texture_id,
            lightmap_id,
            brightness,
            dynamic_tint,
        } => DrawPrimitive::LightmappedTriangle {
            points,
            depths,
            ws,
            surface_uvs,
            lm_uvs,
            texture_id,
            lightmap_id,
            brightness,
            dynamic_tint: apply_post(dynamic_tint, tint, palette_mode),
        },
        #[cfg(feature = "textured")]
        DrawPrimitive::TexturedGouraudTriangleWithDepth {
            points,
            depths,
            ws,
            uvs,
            colors,
            texture_id,
        } => DrawPrimitive::TexturedGouraudTriangleWithDepth {
            points,
            depths,
            ws,
            uvs,
            colors: [
                apply_post(colors[0], tint, palette_mode),
                apply_post(colors[1], tint, palette_mode),
                apply_post(colors[2], tint, palette_mode),
            ],
            texture_id,
        },
        #[cfg(feature = "textured")]
        other => other,
    }
}

#[inline]
pub(super) fn blend_rgb565(a: Rgb565, b: Rgb565, t_q8: u16) -> Rgb565 {
    let inv = 255u16.saturating_sub(t_q8);
    let r = ((a.r() as u16 * inv + b.r() as u16 * t_q8) / 255) as u8;
    let g = ((a.g() as u16 * inv + b.g() as u16 * t_q8) / 255) as u8;
    let bch = ((a.b() as u16 * inv + b.b() as u16 * t_q8) / 255) as u8;
    Rgb565::new(r, g, bch)
}
