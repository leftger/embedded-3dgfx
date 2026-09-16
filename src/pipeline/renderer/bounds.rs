//! Dirty-region bounds: primitive extents clamped to the frame.

//! Command execution: rasterize a recorded command buffer into a framebuffer.

use crate::pipeline::assemble::primitive::DrawPrimitive;

#[inline(always)]
pub(super) fn primitive_bounds(primitive: &DrawPrimitive) -> (i32, i32, i32, i32) {
    primitive.bounds()
}

pub(super) fn clamp_bounds_to_frame(
    min_x: i32,
    min_y: i32,
    max_x: i32,
    max_y: i32,
    width: usize,
    height: usize,
) -> Option<(i32, i32, i32, i32)> {
    let w = width as i32;
    let h = height as i32;
    let clamped_min_x = min_x.clamp(0, w.saturating_sub(1));
    let clamped_min_y = min_y.clamp(0, h.saturating_sub(1));
    let clamped_max_x = max_x.clamp(0, w.saturating_sub(1));
    let clamped_max_y = max_y.clamp(0, h.saturating_sub(1));
    if clamped_max_x < clamped_min_x || clamped_max_y < clamped_min_y {
        return None;
    }
    Some((clamped_min_x, clamped_min_y, clamped_max_x, clamped_max_y))
}
