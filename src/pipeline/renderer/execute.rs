//! The execute half of the record/execute split.
use core::fmt::Debug;

use embedded_graphics_core::{
    Pixel,
    draw_target::DrawTarget,
    pixelcolor::Rgb565,
    prelude::{OriginDimensions, Point},
};

use super::bounds::{clamp_bounds_to_frame, primitive_bounds};
use super::post::{apply_post, tint_primitive};
use super::sky::draw_sky_background;
use super::{DirtyRegion, FrameCtx};
#[cfg(feature = "textured")]
use crate::pipeline::assemble::primitive::DrawPrimitive;
use crate::{
    error::RenderError,
    pipeline::command_buffer::{CommandBuffer, RenderCommand},
    pipeline::rasterize::draw::state::RasterState,
    pipeline::rasterize::draw::zbuffered::draw_zbuffered_with_state,
};

pub fn execute_commands<D, const MAX: usize>(
    fb: &mut D,
    frame: &mut FrameCtx<'_>,
    cmd: &CommandBuffer<MAX>,
    state: &RasterState<'_>,
) -> Result<Option<DirtyRegion>, RenderError>
where
    D: DrawTarget<Color = Rgb565> + OriginDimensions,
    D::Error: Debug,
{
    frame.validate()?;
    let mut dirty_bounds: Option<(i32, i32, i32, i32)> = None;
    if let Some(sky_cfg) = state.sky {
        draw_sky_background(
            fb,
            frame.width,
            frame.height,
            sky_cfg,
            state.camera_dir,
            state.screen_tint,
            state.palette_mode,
        )?;
        dirty_bounds = Some((
            0,
            0,
            frame.width.saturating_sub(1) as i32,
            frame.height.saturating_sub(1) as i32,
        ));
    }

    for c in cmd.iter() {
        match c {
            RenderCommand::ClearColor(color) => {
                let w = frame.width as i32;
                let h = frame.height as i32;
                let clear_color = apply_post(*color, state.screen_tint, state.palette_mode);
                for y in 0..h {
                    for x in 0..w {
                        fb.draw_iter([Pixel(Point::new(x, y), clear_color)])
                            .map_err(|_| {
                                RenderError::InvalidInput("draw target rejected clear write")
                            })?;
                    }
                }
            }
            RenderCommand::ClearDepth(value) => {
                crate::clear_zbuffer(frame.zbuffer, *value);
            }
            RenderCommand::Draw(primitive) => {
                let prim = tint_primitive(primitive, state.screen_tint, state.palette_mode);
                draw_zbuffered_with_state(prim, fb, frame.zbuffer, state);
                let (min_x, min_y, max_x, max_y) = primitive_bounds(primitive);
                if let Some((min_x, min_y, max_x, max_y)) =
                    clamp_bounds_to_frame(min_x, min_y, max_x, max_y, frame.width, frame.height)
                {
                    dirty_bounds = Some(match dirty_bounds {
                        Some((cx0, cy0, cx1, cy1)) => (
                            cx0.min(min_x),
                            cy0.min(min_y),
                            cx1.max(max_x),
                            cy1.max(max_y),
                        ),
                        None => (min_x, min_y, max_x, max_y),
                    });
                }
            }
        }
    }

    let region = dirty_bounds.and_then(|(x0, y0, x1, y1)| DirtyRegion::from_bounds(x0, y0, x1, y1));
    Ok(region)
}

/// Like [`execute_commands`], but resolves
/// [`DrawPrimitive::TexturedTriangleWithDepth`]/[`DrawPrimitive::LightmappedTriangle`]
/// via `texture_manager` instead of silently dropping them (mirrors
/// `bsp::execute_bsp_textured`'s dispatch pattern). Non-textured primitives
/// still go through `tint_primitive` + `draw_zbuffered_with_state`, so mixing
/// textured and flat-colored meshes in one scene keeps consistent
/// tint/palette behavior.
#[cfg(feature = "textured")]
pub fn execute_commands_textured<D, const MAX: usize, const N: usize>(
    fb: &mut D,
    frame: &mut FrameCtx<'_>,
    cmd: &CommandBuffer<MAX>,
    texture_manager: &crate::pipeline::rasterize::texture::TextureManager<N>,
    state: &RasterState<'_>,
) -> Result<Option<DirtyRegion>, RenderError>
where
    D: DrawTarget<Color = Rgb565> + OriginDimensions,
    D::Error: Debug,
{
    use crate::pipeline::rasterize::draw::textured::draw_zbuffered_with_textures_state;
    use crate::pipeline::rasterize::draw::textured::lightmap::draw_zbuffered_lightmapped_mapped;
    frame.validate()?;
    let mut dirty_bounds: Option<(i32, i32, i32, i32)> = None;
    if let Some(sky_cfg) = state.sky {
        draw_sky_background(
            fb,
            frame.width,
            frame.height,
            sky_cfg,
            state.camera_dir,
            state.screen_tint,
            state.palette_mode,
        )?;
        dirty_bounds = Some((
            0,
            0,
            frame.width.saturating_sub(1) as i32,
            frame.height.saturating_sub(1) as i32,
        ));
    }

    for c in cmd.iter() {
        match c {
            RenderCommand::ClearColor(color) => {
                let w = frame.width as i32;
                let h = frame.height as i32;
                let clear_color = apply_post(*color, state.screen_tint, state.palette_mode);
                for y in 0..h {
                    for x in 0..w {
                        fb.draw_iter([Pixel(Point::new(x, y), clear_color)])
                            .map_err(|_| {
                                RenderError::InvalidInput("draw target rejected clear write")
                            })?;
                    }
                }
            }
            RenderCommand::ClearDepth(value) => {
                crate::clear_zbuffer(frame.zbuffer, *value);
            }
            RenderCommand::Draw(primitive) => {
                match primitive {
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
                    } => {
                        draw_zbuffered_lightmapped_mapped(
                            *points,
                            *depths,
                            *ws,
                            *surface_uvs,
                            *lm_uvs,
                            *texture_id,
                            *lightmap_id,
                            *brightness,
                            *dynamic_tint,
                            texture_manager,
                            fb,
                            frame.zbuffer,
                            state,
                        );
                    }
                    #[cfg(feature = "textured")]
                    DrawPrimitive::TexturedTriangle { .. }
                    | DrawPrimitive::TexturedTriangleWithDepth { .. }
                    | DrawPrimitive::TexturedGouraudTriangleWithDepth { .. } => {
                        draw_zbuffered_with_textures_state(
                            primitive.clone(),
                            fb,
                            frame.zbuffer,
                            texture_manager,
                            state,
                        );
                    }
                    _ => {
                        let prim = tint_primitive(primitive, state.screen_tint, state.palette_mode);
                        draw_zbuffered_with_state(prim, fb, frame.zbuffer, state);
                    }
                }
                let (min_x, min_y, max_x, max_y) = primitive_bounds(primitive);
                if let Some((min_x, min_y, max_x, max_y)) =
                    clamp_bounds_to_frame(min_x, min_y, max_x, max_y, frame.width, frame.height)
                {
                    dirty_bounds = Some(match dirty_bounds {
                        Some((cx0, cy0, cx1, cy1)) => (
                            cx0.min(min_x),
                            cy0.min(min_y),
                            cx1.max(max_x),
                            cy1.max(max_y),
                        ),
                        None => (min_x, min_y, max_x, max_y),
                    });
                }
            }
        }
    }

    let region = dirty_bounds.and_then(|(x0, y0, x1, y1)| DirtyRegion::from_bounds(x0, y0, x1, y1));
    Ok(region)
}

pub fn execute_commands_tiled<D, const MAX: usize, const BIN_CAP: usize>(
    fb: &mut D,
    frame: &mut FrameCtx<'_>,
    cmd: &CommandBuffer<MAX>,
    tile: crate::pipeline::rasterize::tilebin::TileConfig,
    state: &RasterState<'_>,
) -> Result<crate::pipeline::rasterize::tilebin::TileBinStats, RenderError>
where
    D: DrawTarget<Color = Rgb565> + OriginDimensions,
    D::Error: Debug,
{
    frame.validate()?;
    if let Some(sky_cfg) = state.sky {
        draw_sky_background(
            fb,
            frame.width,
            frame.height,
            sky_cfg,
            state.camera_dir,
            state.screen_tint,
            state.palette_mode,
        )?;
    }
    let (bins, stats) = crate::pipeline::rasterize::tilebin::build_bins::<MAX, BIN_CAP>(
        cmd,
        frame.width,
        frame.height,
        tile,
    )?;
    let mut executed_draw = [false; MAX];

    for command in cmd.iter() {
        match command {
            RenderCommand::ClearColor(color) => {
                let w = frame.width as i32;
                let h = frame.height as i32;
                let clear_color = apply_post(*color, state.screen_tint, state.palette_mode);
                for y in 0..h {
                    for x in 0..w {
                        fb.draw_iter([Pixel(Point::new(x, y), clear_color)])
                            .map_err(|_| {
                                RenderError::InvalidInput("draw target rejected clear write")
                            })?;
                    }
                }
            }
            RenderCommand::ClearDepth(value) => crate::clear_zbuffer(frame.zbuffer, *value),
            RenderCommand::Draw(_) => {}
        }
    }

    for bin in bins.iter() {
        for idx in bin.iter().copied() {
            if idx >= MAX || executed_draw[idx] {
                continue;
            }
            let Some(RenderCommand::Draw(primitive)) = cmd.get(idx) else {
                continue;
            };
            let prim = tint_primitive(primitive, state.screen_tint, state.palette_mode);
            draw_zbuffered_with_state(prim, fb, frame.zbuffer, state);
            executed_draw[idx] = true;
        }
    }

    Ok(stats)
}

/// Execute commands using 2xSSAA (Super-Sampling Anti-Aliasing) scanline rasterization.
#[cfg(feature = "aa")]
pub fn execute_commands_2xssaa<D, const MAX: usize>(
    fb: &mut D,
    frame: &mut FrameCtx<'_>,
    cmd_buf: &CommandBuffer<MAX>,
) -> Result<(), RenderError>
where
    D: DrawTarget<Color = Rgb565> + crate::pipeline::rasterize::raster::aa::ReadPixel,
    <D as DrawTarget>::Error: Debug,
{
    frame.validate()?;
    for c in cmd_buf.iter() {
        if let RenderCommand::Draw(primitive) = c {
            crate::pipeline::rasterize::draw::aa::draw_zbuffered_2xssaa(
                primitive.clone(),
                fb,
                frame.zbuffer,
                frame.width,
            );
        }
    }
    Ok(())
}
