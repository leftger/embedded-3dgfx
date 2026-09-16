//! Gizmo wireframe recording.

use embedded_graphics_core::pixelcolor::Rgb565;

use crate::engine::K3dengine;
use crate::error::RenderError;
use crate::pipeline::command_buffer::{CommandBuffer, RenderCommand};

#[cfg(feature = "gizmos")]
pub(crate) fn record_aabb_gizmo<const MAX: usize>(
    engine: &K3dengine,
    aabb: &crate::pipeline::vertex::bounds::Aabb,
    model_matrix: &nalgebra::Matrix4<f32>,
    color: Rgb565,
    commands: &mut CommandBuffer<MAX>,
) -> Result<(), RenderError> {
    let mut err = None;
    crate::gizmos::emit_aabb_wireframe_projected(
        aabb,
        model_matrix,
        |p| {
            crate::pipeline::vertex::transform::transform_point(
                &engine.camera,
                engine.width,
                engine.height,
                &p,
                engine.camera.vp_matrix,
            )
        },
        color,
        |prim| {
            if err.is_none()
                && let Err(e) = commands.push(RenderCommand::Draw(prim))
            {
                err = Some(e);
            }
        },
    );
    match err {
        Some(e) => Err(e),
        None => Ok(()),
    }
}

#[cfg(feature = "gizmos")]
pub(crate) fn record_frustum_gizmo<const MAX: usize>(
    engine: &K3dengine,
    color: Rgb565,
    commands: &mut CommandBuffer<MAX>,
) -> Result<(), RenderError> {
    let mut err = None;
    crate::gizmos::emit_frustum_wireframe(
        &engine.camera,
        |p| {
            crate::pipeline::vertex::transform::transform_point(
                &engine.camera,
                engine.width,
                engine.height,
                &p,
                engine.camera.vp_matrix,
            )
        },
        color,
        |prim| {
            if err.is_none()
                && let Err(e) = commands.push(RenderCommand::Draw(prim))
            {
                err = Some(e);
            }
        },
    );
    match err {
        Some(e) => Err(e),
        None => Ok(()),
    }
}
