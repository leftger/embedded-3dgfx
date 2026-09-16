//! Ray/primitive picking executed over a command buffer.

use core::fmt::Debug;

use embedded_graphics_core::{Pixel, draw_target::DrawTarget, pixelcolor::Rgb565, prelude::Point};

use super::bounds::{clamp_bounds_to_frame, primitive_bounds};
use super::{DirtyRegion, FrameCtx, PickQuery, PickResult};
use crate::{
    error::RenderError,
    pipeline::command_buffer::{CommandBuffer, RenderCommand},
};

/// Execute commands and evaluate integrated screen-space pick queries during the rasterization pass.
pub fn execute_commands_with_picking<D, const MAX: usize>(
    fb: &mut D,
    frame: &mut FrameCtx<'_>,
    cmd: &CommandBuffer<MAX>,
    queries: &[PickQuery],
    results: &mut [Option<PickResult>],
) -> Result<Option<DirtyRegion>, RenderError>
where
    D: DrawTarget<Color = Rgb565>,
    <D as DrawTarget>::Error: Debug,
{
    frame.validate()?;
    let mut dirty_bounds: Option<(i32, i32, i32, i32)> = None;

    for (cmd_idx, c) in cmd.iter().enumerate() {
        match c {
            RenderCommand::ClearColor(color) => {
                let w = frame.width as i32;
                let h = frame.height as i32;
                for y in 0..h {
                    for x in 0..w {
                        fb.draw_iter([Pixel(Point::new(x, y), *color)])
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
                let mut prev_depths = [0 as crate::ZDepth; 16];
                let check_count = queries.len().min(16).min(results.len());
                for (q_i, query) in queries.iter().take(check_count).enumerate() {
                    if query.x >= 0
                        && query.x < frame.width as i32
                        && query.y >= 0
                        && query.y < frame.height as i32
                    {
                        let idx = query.y as usize * frame.width + query.x as usize;
                        prev_depths[q_i] = frame.zbuffer[idx];
                    }
                }

                crate::pipeline::rasterize::draw::zbuffered::draw_zbuffered(
                    primitive.clone(),
                    fb,
                    frame.zbuffer,
                    frame.width,
                );

                for (q_i, query) in queries.iter().take(check_count).enumerate() {
                    if query.x >= 0
                        && query.x < frame.width as i32
                        && query.y >= 0
                        && query.y < frame.height as i32
                    {
                        let idx = query.y as usize * frame.width + query.x as usize;
                        let new_depth = frame.zbuffer[idx];
                        if new_depth < prev_depths[q_i] {
                            results[q_i] = Some(PickResult {
                                x: query.x,
                                y: query.y,
                                depth: new_depth,
                                command_index: cmd_idx,
                            });
                        }
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
