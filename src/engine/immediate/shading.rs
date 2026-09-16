//! Per-vertex / per-sector shading and tinting helpers used by the traversal.

#[allow(unused_imports)]
use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
#[allow(unused_imports)]
use nalgebra::{Matrix4, Point3, Vector3, Vector4};

#[allow(unused_imports)]
use micromath::F32Ext;

#[cfg(feature = "lighting")]
use crate::engine::K3dengine;
#[cfg(feature = "lod-crossfade")]
use crate::pipeline::assemble::primitive::DrawPrimitive;
#[allow(unused_imports)]
use crate::pipeline::vertex::transform::{transform_point, transform_point_with_w};

#[cfg(feature = "lod-crossfade")]
#[inline]
pub(crate) fn apply_draw_alpha(prim: DrawPrimitive, alpha: u8) -> DrawPrimitive {
    match prim {
        DrawPrimitive::ColoredTriangleWithDepth {
            points,
            depths,
            color,
        } => DrawPrimitive::TranslucentTriangleWithDepth {
            points,
            depths,
            color,
            alpha,
        },
        DrawPrimitive::TranslucentTriangleWithDepth {
            points,
            depths,
            color,
            alpha: prev,
        } => DrawPrimitive::TranslucentTriangleWithDepth {
            points,
            depths,
            color,
            alpha: ((prev as u16 * alpha as u16) / 255) as u8,
        },
        other => other,
    }
}

#[cfg(feature = "lighting")]
#[inline]
pub(crate) fn light_tint_at(engine: &K3dengine, world_pos: Point3<f32>) -> Rgb565 {
    let mut acc_r = 0u16;
    let mut acc_g = 0u16;
    let mut acc_b = 0u16;
    for light in &engine.point_lights {
        let tint = light.contribution_at(world_pos);
        acc_r += tint.r() as u16;
        acc_g += tint.g() as u16;
        acc_b += tint.b() as u16;
    }
    Rgb565::new(
        (acc_r.min(31)) as u8,
        (acc_g.min(63)) as u8,
        (acc_b.min(31)) as u8,
    )
}

#[cfg(feature = "lighting")]
#[inline]
pub(crate) fn add_tint(base: Rgb565, tint: Rgb565) -> Rgb565 {
    Rgb565::new(
        (base.r() as u16 + tint.r() as u16).min(31) as u8,
        (base.g() as u16 + tint.g() as u16).min(63) as u8,
        (base.b() as u16 + tint.b() as u16).min(31) as u8,
    )
}

#[cfg(feature = "lighting")]
const DOOM_LIGHT_TABLE: [u8; 32] = [
    8, 12, 16, 20, 24, 28, 34, 40, 48, 56, 64, 72, 82, 92, 102, 112, 124, 136, 148, 160, 172, 184,
    196, 206, 216, 224, 232, 238, 244, 248, 252, 255,
];

#[cfg(feature = "lighting")]
#[inline]
pub(crate) fn sector_shaded_color(
    engine: &K3dengine,
    base: Rgb565,
    brightness: u8,
    face_center: Point3<f32>,
) -> Rgb565 {
    let level_u8 = match engine.light_levels {
        crate::pipeline::shade::retro::light_levels::LightLevels::Linear => brightness,
        crate::pipeline::shade::retro::light_levels::LightLevels::Doom32 => {
            let base_level = (brightness as usize * 31) / 255;
            let distance = (face_center - engine.camera.position).norm();
            let distance_drop = (distance * 2.0) as usize;
            let idx = base_level.saturating_sub(distance_drop).min(31);
            DOOM_LIGHT_TABLE[idx]
        }
    };

    let factor = level_u8 as f32 / 255.0;
    Rgb565::new(
        (base.r() as f32 * factor) as u8,
        (base.g() as f32 * factor) as u8,
        (base.b() as f32 * factor) as u8,
    )
}
