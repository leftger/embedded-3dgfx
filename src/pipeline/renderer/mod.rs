//! Command execution: rasterize a recorded command buffer into a framebuffer.

use core::fmt::Debug;

use crate::error::{BudgetKind, RenderError};

pub(crate) mod bounds;
pub(crate) mod execute;
pub(crate) mod picking;
pub(crate) mod post;
pub(crate) mod sky;

// The execute driver is the module's public surface; callers keep using
// `renderer::execute_commands*` without knowing how it is carved up.
#[cfg(feature = "aa")]
pub use execute::execute_commands_2xssaa;
#[cfg(feature = "textured")]
pub use execute::execute_commands_textured;
pub use execute::{execute_commands, execute_commands_tiled};
pub use picking::execute_commands_with_picking;

pub struct FrameCtx<'a> {
    pub zbuffer: &'a mut [crate::ZDepth],
    pub width: usize,
    pub height: usize,
}

impl<'a> FrameCtx<'a> {
    pub fn validate(&self) -> Result<(), RenderError> {
        let expected = self.width * self.height;
        if self.zbuffer.len() != expected {
            return Err(RenderError::OutOfBudget(BudgetKind::ZBufferLength {
                expected,
                got: self.zbuffer.len(),
            }));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DirtyRegion {
    pub x: usize,
    pub y: usize,
    pub width: usize,
    pub height: usize,
}

impl DirtyRegion {
    fn from_bounds(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> Option<Self> {
        if max_x < min_x || max_y < min_y {
            return None;
        }
        Some(Self {
            x: min_x as usize,
            y: min_y as usize,
            width: (max_x - min_x + 1) as usize,
            height: (max_y - min_y + 1) as usize,
        })
    }
}

/// A screen-space pick query asking for the topmost hit at pixel `(x, y)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PickQuery {
    pub x: i32,
    pub y: i32,
}

impl PickQuery {
    pub const fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }
}

/// Result of an integrated screen-space pick query during command execution.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PickResult {
    /// Screen X coordinate tested.
    pub x: i32,
    /// Screen Y coordinate tested.
    pub y: i32,
    /// Closest depth recorded at this pixel.
    pub depth: crate::ZDepth,
    /// Index of the command in the CommandBuffer that hit this pixel.
    pub command_index: usize,
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::sky::stripe_on_at;

    use crate::pipeline::rasterize::draw::state::RasterState;

    #[test]
    fn stripe_phase_is_periodic_with_negative_scroll() {
        let stripe_w = 10;
        let scroll = -3;
        for x in -40..40 {
            assert_eq!(
                stripe_on_at(x, scroll, stripe_w),
                stripe_on_at(x + stripe_w * 2, scroll, stripe_w)
            );
        }
    }

    #[test]
    fn stripe_runs_do_not_exceed_width() {
        let stripe_w = 8;
        let scroll = -5;
        let mut max_run = 0usize;
        let mut run = 0usize;
        let mut prev = stripe_on_at(-64, scroll, stripe_w);
        for x in -63..=64 {
            let cur = stripe_on_at(x, scroll, stripe_w);
            if cur == prev {
                run += 1;
            } else {
                max_run = max_run.max(run);
                run = 1;
                prev = cur;
            }
        }
        max_run = max_run.max(run);
        assert!(max_run <= stripe_w as usize);
    }

    #[test]
    fn test_execute_commands_with_picking() {
        use crate::pipeline::assemble::primitive::DrawPrimitive;
        use crate::pipeline::command_buffer::{CommandBuffer, RenderCommand};
        use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
        use embedded_graphics_framebuf::{
            FrameBuf,
            backends::{EndianCorrectedBuffer, EndianCorrection},
        };
        use nalgebra::Point2;

        let backing = std::vec![Rgb565::BLACK; 64 * 64].leak();
        let mut fb = FrameBuf::new(
            EndianCorrectedBuffer::new(backing, EndianCorrection::ToLittleEndian),
            64,
            64,
        );
        let mut zbuf = [crate::Z_MAX_VALUE; 64 * 64];
        let mut frame = super::FrameCtx {
            zbuffer: &mut zbuf,
            width: 64,
            height: 64,
        };

        let mut cmd = CommandBuffer::<8>::new();
        cmd.push(RenderCommand::Draw(
            DrawPrimitive::ColoredTriangleWithDepth {
                points: [
                    Point2::new(10, 10),
                    Point2::new(40, 10),
                    Point2::new(25, 40),
                ],
                depths: [10.0, 10.0, 10.0],
                color: Rgb565::RED,
            },
        ))
        .unwrap();

        let queries = [super::PickQuery::new(25, 20), super::PickQuery::new(0, 0)];
        let mut results = [None, None];

        let region =
            super::execute_commands_with_picking(&mut fb, &mut frame, &cmd, &queries, &mut results)
                .unwrap();

        assert!(region.is_some());
        assert!(results[0].is_some());
        let hit = results[0].unwrap();
        assert_eq!(hit.x, 25);
        assert_eq!(hit.y, 20);
        assert_eq!(hit.command_index, 0);
        assert!(results[1].is_none()); // (0, 0) was outside triangle
    }

    #[test]
    fn test_execute_commands_variants() {
        use crate::pipeline::assemble::primitive::DrawPrimitive;
        use crate::pipeline::command_buffer::{CommandBuffer, RenderCommand};
        use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
        use embedded_graphics_framebuf::{
            FrameBuf,
            backends::{EndianCorrectedBuffer, EndianCorrection},
        };
        use nalgebra::Point2;

        let backing = std::vec![Rgb565::BLACK; 16 * 16].leak();
        let mut fb = FrameBuf::new(
            EndianCorrectedBuffer::new(backing, EndianCorrection::ToLittleEndian),
            16,
            16,
        );
        let mut zbuf = [crate::Z_MAX_VALUE; 16 * 16];
        let mut frame = super::FrameCtx {
            zbuffer: &mut zbuf,
            width: 16,
            height: 16,
        };

        let mut cmd = CommandBuffer::<4>::new();
        cmd.push(RenderCommand::ClearColor(Rgb565::BLUE)).unwrap();
        cmd.push(RenderCommand::ClearDepth(crate::Z_MAX_VALUE))
            .unwrap();
        cmd.push(RenderCommand::Draw(
            DrawPrimitive::ColoredTriangleWithDepth {
                points: [Point2::new(2, 2), Point2::new(12, 2), Point2::new(7, 12)],
                depths: [10.0; 3],
                color: Rgb565::RED,
            },
        ))
        .unwrap();

        let state = RasterState::new(frame.width, frame.height);
        assert!(super::execute_commands(&mut fb, &mut frame, &cmd, &state).is_ok());

        let dirty = super::execute_commands(&mut fb, &mut frame, &cmd, &state).unwrap();
        assert!(dirty.is_some());
        let dirty = dirty.unwrap();
        assert!(dirty.width >= 1);
        assert!(dirty.height >= 1);

        let region = super::execute_commands(&mut fb, &mut frame, &cmd, &state).unwrap();
        assert!(region.is_some());

        // Sky path reports the full frame dirty.
        let sky_state = state.with_sky(Some(
            crate::pipeline::shade::retro::sky::SkyConfig::retro_blue(),
        ));
        let full_region = super::execute_commands(&mut fb, &mut frame, &cmd, &sky_state).unwrap();
        let full = full_region.unwrap();
        assert_eq!(full.x, 0);
        assert_eq!(full.y, 0);
        assert_eq!(full.width, 16);
        assert_eq!(full.height, 16);
    }

    #[test]
    fn test_execute_commands_tiled_and_frame_validation() {
        use crate::pipeline::assemble::primitive::DrawPrimitive;
        use crate::pipeline::command_buffer::{CommandBuffer, RenderCommand};
        use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
        use embedded_graphics_framebuf::{
            FrameBuf,
            backends::{EndianCorrectedBuffer, EndianCorrection},
        };
        use nalgebra::Point2;

        let backing = std::vec![Rgb565::BLACK; 16 * 16].leak();
        let mut fb = FrameBuf::new(
            EndianCorrectedBuffer::new(backing, EndianCorrection::ToLittleEndian),
            16,
            16,
        );

        // Validation fails on mismatched zbuffer length.
        let mut short_zbuf = [crate::Z_MAX_VALUE; 4];
        let mut bad_frame = super::FrameCtx {
            zbuffer: &mut short_zbuf,
            width: 16,
            height: 16,
        };
        let empty = CommandBuffer::<1>::new();
        let bad_state = RasterState::new(bad_frame.width, bad_frame.height);
        assert!(super::execute_commands(&mut fb, &mut bad_frame, &empty, &bad_state).is_err());

        let mut zbuf = [crate::Z_MAX_VALUE; 16 * 16];
        let mut frame = super::FrameCtx {
            zbuffer: &mut zbuf,
            width: 16,
            height: 16,
        };

        let mut cmd = CommandBuffer::<2>::new();
        cmd.push(RenderCommand::Draw(
            DrawPrimitive::ColoredTriangleWithDepth {
                points: [Point2::new(3, 3), Point2::new(13, 3), Point2::new(8, 13)],
                depths: [5.0; 3],
                color: Rgb565::GREEN,
            },
        ))
        .unwrap();

        let state = RasterState::new(frame.width, frame.height);
        let stats = super::execute_commands_tiled::<_, 2, 16>(
            &mut fb,
            &mut frame,
            &cmd,
            crate::pipeline::rasterize::tilebin::TileConfig {
                tile_width: 8,
                tile_height: 8,
            },
            &state,
        )
        .unwrap();
        assert!(stats.draw_commands >= 1);
        assert!(stats.bins_used >= 1);
    }

    #[test]
    fn test_tint_primitive_and_primitives_coverage() {
        use crate::pipeline::assemble::primitive::DrawPrimitive;
        use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
        use nalgebra::Point2;

        let tint = Some(crate::pipeline::shade::retro::tint::ScreenTint {
            color: Rgb565::RED,
            strength: 128,
        });
        let pal = crate::pipeline::shade::retro::palette::PaletteMode::Off;

        let p1 = DrawPrimitive::ColoredPoint(Point2::new(1, 1), Rgb565::WHITE);
        let p1_t = super::post::tint_primitive(&p1, tint, pal);
        assert!(matches!(p1_t, DrawPrimitive::ColoredPoint(..)));

        let l = DrawPrimitive::Line([Point2::new(1, 1), Point2::new(2, 2)], Rgb565::WHITE);
        let l_t = super::post::tint_primitive(&l, tint, pal);
        assert!(matches!(l_t, DrawPrimitive::Line(..)));

        let t = DrawPrimitive::ColoredTriangle(
            [Point2::new(1, 1), Point2::new(2, 2), Point2::new(3, 3)],
            Rgb565::WHITE,
        );
        let t_t = super::post::tint_primitive(&t, tint, pal);
        assert!(matches!(t_t, DrawPrimitive::ColoredTriangle(..)));

        let td = DrawPrimitive::TranslucentTriangleWithDepth {
            points: [Point2::new(1, 1), Point2::new(2, 2), Point2::new(3, 3)],
            depths: [1.0; 3],
            color: Rgb565::WHITE,
            alpha: 128,
        };
        let td_t = super::post::tint_primitive(&td, tint, pal);
        assert!(matches!(
            td_t,
            DrawPrimitive::TranslucentTriangleWithDepth { .. }
        ));

        let sd = DrawPrimitive::ScreenDoorTriangleWithDepth {
            points: [Point2::new(1, 1), Point2::new(2, 2), Point2::new(3, 3)],
            depths: [1.0; 3],
            color: Rgb565::WHITE,
            alpha: 128,
        };
        let sd_t = super::post::tint_primitive(&sd, tint, pal);
        assert!(matches!(
            sd_t,
            DrawPrimitive::ScreenDoorTriangleWithDepth { .. }
        ));

        #[cfg(feature = "lighting")]
        {
            let g = DrawPrimitive::GouraudTriangle {
                points: [Point2::new(1, 1), Point2::new(2, 2), Point2::new(3, 3)],
                colors: [Rgb565::WHITE; 3],
            };
            let g_t = super::post::tint_primitive(&g, tint, pal);
            assert!(matches!(g_t, DrawPrimitive::GouraudTriangle { .. }));

            let gd = DrawPrimitive::GouraudTriangleWithDepth {
                points: [Point2::new(1, 1), Point2::new(2, 2), Point2::new(3, 3)],
                depths: [1.0; 3],
                colors: [Rgb565::WHITE; 3],
            };
            let gd_t = super::post::tint_primitive(&gd, tint, pal);
            assert!(matches!(
                gd_t,
                DrawPrimitive::GouraudTriangleWithDepth { .. }
            ));
        }

        #[cfg(feature = "textured")]
        {
            let lm = DrawPrimitive::LightmappedTriangle {
                points: [Point2::new(1, 1), Point2::new(2, 2), Point2::new(3, 3)],
                depths: [1.0; 3],
                ws: [1.0; 3],
                surface_uvs: [[0.0; 2]; 3],
                lm_uvs: [[0.0; 2]; 3],
                texture_id: 0,
                lightmap_id: 0,
                brightness: 255,
                dynamic_tint: Rgb565::WHITE,
            };
            let lm_t = super::post::tint_primitive(&lm, tint, pal);
            assert!(matches!(lm_t, DrawPrimitive::LightmappedTriangle { .. }));

            let tg = DrawPrimitive::TexturedGouraudTriangleWithDepth {
                points: [Point2::new(1, 1), Point2::new(2, 2), Point2::new(3, 3)],
                depths: [1.0; 3],
                ws: [1.0; 3],
                uvs: [[0.0; 2]; 3],
                colors: [Rgb565::WHITE; 3],
                texture_id: 0,
            };
            let tg_t = super::post::tint_primitive(&tg, tint, pal);
            assert!(matches!(
                tg_t,
                DrawPrimitive::TexturedGouraudTriangleWithDepth { .. }
            ));
        }
    }

    #[test]
    fn test_colored_triangle_with_depth_tint() {
        use crate::pipeline::assemble::primitive::DrawPrimitive;
        use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
        use nalgebra::Point2;

        let tint = Some(crate::pipeline::shade::retro::tint::ScreenTint {
            color: Rgb565::GREEN,
            strength: 64,
        });
        let pal = crate::pipeline::shade::retro::palette::PaletteMode::Off;

        // ColoredTriangleWithDepth variant in tint_primitive
        let ctd = DrawPrimitive::ColoredTriangleWithDepth {
            points: [Point2::new(1, 1), Point2::new(5, 1), Point2::new(3, 5)],
            depths: [1.0, 2.0, 3.0],
            color: Rgb565::WHITE,
        };
        let ctd_t = super::post::tint_primitive(&ctd, tint, pal);
        assert!(matches!(
            ctd_t,
            DrawPrimitive::ColoredTriangleWithDepth { .. }
        ));
        // No tint
        let ctd_none = super::post::tint_primitive(&ctd, None, pal);
        assert!(matches!(
            ctd_none,
            DrawPrimitive::ColoredTriangleWithDepth { .. }
        ));
    }

    #[test]
    fn test_dirty_region_from_bounds_edge_cases() {
        // max < min → None
        assert!(super::DirtyRegion::from_bounds(10, 10, 5, 5).is_none());
        // equal → 1x1
        let r = super::DirtyRegion::from_bounds(4, 4, 4, 4).unwrap();
        assert_eq!(r.width, 1);
        assert_eq!(r.height, 1);
    }

    #[test]
    fn test_execute_commands_tiled_effects_with_sky() {
        use crate::pipeline::assemble::primitive::DrawPrimitive;
        use crate::pipeline::command_buffer::{CommandBuffer, RenderCommand};
        use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
        use embedded_graphics_framebuf::{
            FrameBuf,
            backends::{EndianCorrectedBuffer, EndianCorrection},
        };
        use nalgebra::Point2;

        let backing = std::vec![Rgb565::BLACK; 16 * 16].leak();
        let mut fb = FrameBuf::new(
            EndianCorrectedBuffer::new(backing, EndianCorrection::ToLittleEndian),
            16,
            16,
        );
        let mut zbuf = [crate::Z_MAX_VALUE; 16 * 16];
        let mut frame = super::FrameCtx {
            zbuffer: &mut zbuf,
            width: 16,
            height: 16,
        };

        let mut cmd = CommandBuffer::<4>::new();
        cmd.push(RenderCommand::ClearDepth(crate::Z_MAX_VALUE))
            .unwrap();
        cmd.push(RenderCommand::Draw(
            DrawPrimitive::ColoredTriangleWithDepth {
                points: [Point2::new(2, 2), Point2::new(14, 2), Point2::new(8, 14)],
                depths: [5.0; 3],
                color: Rgb565::RED,
            },
        ))
        .unwrap();

        // Tiled path with the sky pass enabled.
        let state = RasterState::new(frame.width, frame.height)
            .with_sky(Some(
                crate::pipeline::shade::retro::sky::SkyConfig::retro_blue(),
            ))
            .with_camera_dir([0.1, 0.0, -1.0]);
        let stats = super::execute_commands_tiled::<_, 4, 8>(
            &mut fb,
            &mut frame,
            &cmd,
            crate::pipeline::rasterize::tilebin::TileConfig {
                tile_width: 8,
                tile_height: 8,
            },
            &state,
        )
        .unwrap();
        assert!(stats.draw_commands >= 1);
    }

    #[test]
    fn test_frame_ctx_validate() {
        // Wrong zbuffer size → Err
        let mut short_zbuf = [crate::Z_MAX_VALUE; 4];
        let ctx = super::FrameCtx {
            zbuffer: &mut short_zbuf,
            width: 8,
            height: 8,
        };
        assert!(ctx.validate().is_err());

        // Correct size → Ok
        let mut zbuf = [crate::Z_MAX_VALUE; 64];
        let ctx2 = super::FrameCtx {
            zbuffer: &mut zbuf,
            width: 8,
            height: 8,
        };
        assert!(ctx2.validate().is_ok());
    }

    #[test]
    fn test_pick_query_new_and_eq() {
        let q = super::PickQuery::new(3, 7);
        assert_eq!(q.x, 3);
        assert_eq!(q.y, 7);
        assert_eq!(q, super::PickQuery { x: 3, y: 7 });
    }

    #[test]
    fn test_execute_commands_tiled_effects_with_clearcolor() {
        use crate::pipeline::assemble::primitive::DrawPrimitive;
        use crate::pipeline::command_buffer::{CommandBuffer, RenderCommand};
        use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
        use embedded_graphics_framebuf::{
            FrameBuf,
            backends::{EndianCorrectedBuffer, EndianCorrection},
        };
        use nalgebra::Point2;

        let backing = std::vec![Rgb565::BLACK; 16 * 16].leak();
        let mut fb = FrameBuf::new(
            EndianCorrectedBuffer::new(backing, EndianCorrection::ToLittleEndian),
            16,
            16,
        );
        let mut zbuf = [crate::Z_MAX_VALUE; 16 * 16];
        let mut frame = super::FrameCtx {
            zbuffer: &mut zbuf,
            width: 16,
            height: 16,
        };

        let mut cmd = CommandBuffer::<4>::new();
        // ClearColor goes through the non-tiled pass
        cmd.push(RenderCommand::ClearColor(Rgb565::BLUE)).unwrap();
        cmd.push(RenderCommand::ClearDepth(crate::Z_MAX_VALUE))
            .unwrap();
        cmd.push(RenderCommand::Draw(
            DrawPrimitive::ColoredTriangleWithDepth {
                points: [Point2::new(2, 2), Point2::new(14, 2), Point2::new(8, 14)],
                depths: [5.0; 3],
                color: Rgb565::RED,
            },
        ))
        .unwrap();

        let state = RasterState::new(frame.width, frame.height);
        let stats = super::execute_commands_tiled::<_, 4, 8>(
            &mut fb,
            &mut frame,
            &cmd,
            crate::pipeline::rasterize::tilebin::TileConfig {
                tile_width: 8,
                tile_height: 8,
            },
            &state,
        )
        .unwrap();
        assert!(stats.draw_commands >= 1);
    }

    #[test]
    fn test_execute_commands_with_picking_clearcolor_and_oob_query() {
        use crate::pipeline::assemble::primitive::DrawPrimitive;
        use crate::pipeline::command_buffer::{CommandBuffer, RenderCommand};
        use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
        use embedded_graphics_framebuf::{
            FrameBuf,
            backends::{EndianCorrectedBuffer, EndianCorrection},
        };
        use nalgebra::Point2;

        let backing = std::vec![Rgb565::BLACK; 16 * 16].leak();
        let mut fb = FrameBuf::new(
            EndianCorrectedBuffer::new(backing, EndianCorrection::ToLittleEndian),
            16,
            16,
        );
        let mut zbuf = [crate::Z_MAX_VALUE; 16 * 16];
        let mut frame = super::FrameCtx {
            zbuffer: &mut zbuf,
            width: 16,
            height: 16,
        };

        let mut cmd = CommandBuffer::<4>::new();
        cmd.push(RenderCommand::ClearColor(Rgb565::GREEN)).unwrap();
        cmd.push(RenderCommand::ClearDepth(crate::Z_MAX_VALUE))
            .unwrap();
        cmd.push(RenderCommand::Draw(
            DrawPrimitive::ColoredTriangleWithDepth {
                points: [Point2::new(2, 2), Point2::new(14, 2), Point2::new(8, 14)],
                depths: [5.0; 3],
                color: Rgb565::RED,
            },
        ))
        .unwrap();

        // Out-of-bounds queries (negative x/y should be skipped)
        let queries = [
            super::PickQuery::new(-1, -1), // negative — out of bounds
            super::PickQuery::new(8, 8),   // inside triangle
        ];
        let mut results = [None, None];
        let region =
            super::execute_commands_with_picking(&mut fb, &mut frame, &cmd, &queries, &mut results)
                .unwrap();
        assert!(region.is_some());
        assert!(results[0].is_none()); // oob query never gets a hit
        assert!(results[1].is_some()); // (8,8) is inside the triangle
    }

    #[test]
    fn test_execute_commands_with_effects_tint_and_dither() {
        use crate::pipeline::assemble::primitive::DrawPrimitive;
        use crate::pipeline::command_buffer::{CommandBuffer, RenderCommand};
        use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
        use embedded_graphics_framebuf::{
            FrameBuf,
            backends::{EndianCorrectedBuffer, EndianCorrection},
        };
        use nalgebra::Point2;

        let backing = std::vec![Rgb565::BLACK; 16 * 16].leak();
        let mut fb = FrameBuf::new(
            EndianCorrectedBuffer::new(backing, EndianCorrection::ToLittleEndian),
            16,
            16,
        );
        let mut zbuf = [crate::Z_MAX_VALUE; 16 * 16];
        let mut frame = super::FrameCtx {
            zbuffer: &mut zbuf,
            width: 16,
            height: 16,
        };

        let mut cmd = CommandBuffer::<4>::new();
        cmd.push(RenderCommand::Draw(
            DrawPrimitive::ColoredTriangleWithDepth {
                points: [Point2::new(1, 1), Point2::new(14, 1), Point2::new(7, 14)],
                depths: [3.0; 3],
                color: Rgb565::WHITE,
            },
        ))
        .unwrap();

        let tint = Some(crate::pipeline::shade::retro::tint::ScreenTint {
            color: Rgb565::RED,
            strength: 80,
        });
        let dither_cfg = crate::pipeline::effects::DitherConfig { intensity: 128 };
        let state = RasterState::new(frame.width, frame.height)
            .with_dither(Some(&dither_cfg))
            .with_screen_tint(tint);
        let region = super::execute_commands(&mut fb, &mut frame, &cmd, &state).unwrap();
        assert!(region.is_some());
    }
}
