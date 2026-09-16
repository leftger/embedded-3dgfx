//! The execute half: drive a recorded command buffer into a target.

use core::fmt::Debug;
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::geometry::OriginDimensions;
use embedded_graphics_core::pixelcolor::Rgb565;

use crate::engine::K3dengine;
use crate::error::RenderError;
use crate::pipeline::command_buffer::{CommandBuffer, RenderCommand};

pub(crate) fn execute<D, const MAX: usize>(
    engine: &K3dengine,
    fb: &mut D,
    frame: &mut crate::pipeline::renderer::FrameCtx<'_>,
    commands: &CommandBuffer<MAX>,
    telemetry: Option<&mut crate::telemetry::ExecuteTelemetry>,
) -> Result<Option<crate::pipeline::renderer::DirtyRegion>, RenderError>
where
    D: DrawTarget<Color = Rgb565> + OriginDimensions,
    D::Error: Debug,
{
    if let Some(t) = telemetry {
        t.commands_total = commands.len();
        t.draw_commands = commands
            .iter()
            .filter(|cmd| matches!(cmd, RenderCommand::Draw(_)))
            .count();
        t.clear_color_commands = commands
            .iter()
            .filter(|cmd| matches!(cmd, RenderCommand::ClearColor(_)))
            .count();
        t.clear_depth_commands = commands
            .iter()
            .filter(|cmd| matches!(cmd, RenderCommand::ClearDepth(_)))
            .count();
    }
    let state = engine.raster_state(frame.width, frame.height);
    crate::pipeline::renderer::execute::execute_commands(fb, frame, commands, &state)
}

#[cfg(feature = "textured")]
pub(crate) fn execute_with_textures<D, const MAX: usize, const N: usize>(
    engine: &K3dengine,
    fb: &mut D,
    frame: &mut crate::pipeline::renderer::FrameCtx<'_>,
    commands: &CommandBuffer<MAX>,
    texture_manager: &crate::pipeline::rasterize::texture::TextureManager<N>,
    telemetry: Option<&mut crate::telemetry::ExecuteTelemetry>,
) -> Result<Option<crate::pipeline::renderer::DirtyRegion>, RenderError>
where
    D: DrawTarget<Color = Rgb565> + OriginDimensions,
    D::Error: Debug,
{
    if let Some(t) = telemetry {
        t.commands_total = commands.len();
        t.draw_commands = commands
            .iter()
            .filter(|cmd| matches!(cmd, RenderCommand::Draw(_)))
            .count();
        t.clear_color_commands = commands
            .iter()
            .filter(|cmd| matches!(cmd, RenderCommand::ClearColor(_)))
            .count();
        t.clear_depth_commands = commands
            .iter()
            .filter(|cmd| matches!(cmd, RenderCommand::ClearDepth(_)))
            .count();
    }
    let state = engine.raster_state(frame.width, frame.height);
    crate::pipeline::renderer::execute::execute_commands_textured(
        fb,
        frame,
        commands,
        texture_manager,
        &state,
    )
}

pub(crate) fn execute_tiled<D, const MAX: usize, const BIN_CAP: usize>(
    engine: &K3dengine,
    fb: &mut D,
    frame: &mut crate::pipeline::renderer::FrameCtx<'_>,
    commands: &CommandBuffer<MAX>,
    tile: crate::pipeline::rasterize::tilebin::TileConfig,
) -> Result<crate::pipeline::rasterize::tilebin::TileBinStats, RenderError>
where
    D: DrawTarget<Color = Rgb565> + OriginDimensions,
    D::Error: Debug,
{
    let state = engine.raster_state(frame.width, frame.height);
    crate::pipeline::renderer::execute::execute_commands_tiled::<D, MAX, BIN_CAP>(
        fb, frame, commands, tile, &state,
    )
}
