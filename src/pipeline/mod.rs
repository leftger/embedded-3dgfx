//! The graphics pipeline.
//!
//! Rendering is modelled as five ordered stages, each owning one module. A
//! stage consumes the previous stage's output and nothing else:
//!
//! | # | Stage | Module | Input -> output |
//! |---|-------|--------|-----------------|
//! | 1 | [`Stage::Vertex`]    | [`vertex`]    | model-space mesh -> clip-space vertices |
//! | 2 | [`Stage::Assemble`]  | [`assemble`]  | clip-space vertices -> screen-space [`DrawPrimitive`](assemble::primitive::DrawPrimitive) |
//! | 3 | [`Stage::Rasterize`] | [`rasterize`] | screen primitives -> covered pixels |
//! | 4 | [`Stage::Shade`]     | [`shade`]     | fragments -> `Rgb565` colours |
//! | 5 | [`Stage::Output`]    | [`output`]    | framebuffer -> presented frame |
//!
//! [`command_buffer`] is the transport between the record half (stages 1-2) and
//! the execute half (stages 3-5); [`renderer`] drives that execute half.
//! Configuration shared by several stages lives in [`effects`].
//!
//! # Dependency direction
//!
//! A stage may depend on lower-numbered stages, [`effects`], and the crate-root
//! `core` types — never on a higher-numbered stage. The one intentional
//! exception is that [`rasterize`] invokes the fragment programs defined in
//! [`shade`]: the shader interface is a *parameter* of the rasterizer, so
//! `shade` sits below `rasterize` in the dependency order even though it runs
//! afterwards in the data flow.

pub mod assemble;
pub mod command_buffer;
pub mod effects;
pub mod output;
pub mod rasterize;
pub mod renderer;
pub mod shade;
pub mod vertex;

use embedded_graphics_core::pixelcolor::Rgb565;

/// One ordered stage of the graphics pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Stage {
    /// Model-space geometry and camera transforms.
    Vertex,
    /// Screen-space primitive assembly.
    Assemble,
    /// Triangle / line / point rasterization.
    Rasterize,
    /// Fragment shading.
    Shade,
    /// Framebuffer writes and presentation.
    Output,
}

impl Stage {
    /// Every stage, in execution order.
    pub const ORDER: [Stage; 5] = [
        Stage::Vertex,
        Stage::Assemble,
        Stage::Rasterize,
        Stage::Shade,
        Stage::Output,
    ];

    /// Zero-based position of this stage in [`Stage::ORDER`].
    #[must_use]
    pub const fn index(self) -> usize {
        self as usize
    }

    /// The stage that consumes this stage's output.
    #[must_use]
    pub const fn next(self) -> Option<Stage> {
        match self {
            Stage::Vertex => Some(Stage::Assemble),
            Stage::Assemble => Some(Stage::Rasterize),
            Stage::Rasterize => Some(Stage::Shade),
            Stage::Shade => Some(Stage::Output),
            Stage::Output => None,
        }
    }
}

/// Implemented by the principal type of each [`Stage`].
pub trait StageKind {
    /// Which pipeline stage this type belongs to.
    const STAGE: Stage;
}

/// A homogeneous clip-space vertex, passed from [`vertex`] to [`assemble`].
pub type ClipVertex = nalgebra::Vector4<f32>;

/// The screen-space primitive handed from [`assemble`] to [`rasterize`].
pub type ScreenPrimitive = assemble::primitive::DrawPrimitive;

/// A shaded fragment colour.
pub type FragmentColor = Rgb565;

impl StageKind for vertex::camera::Camera {
    const STAGE: Stage = Stage::Vertex;
}
impl StageKind for assemble::primitive::DrawPrimitive {
    const STAGE: Stage = Stage::Assemble;
}
impl<'a> StageKind for rasterize::draw::state::RasterState<'a> {
    const STAGE: Stage = Stage::Rasterize;
}
impl StageKind for shade::shader::FlatColorShader {
    const STAGE: Stage = Stage::Shade;
}
impl StageKind for crate::error::DisplayError {
    const STAGE: Stage = Stage::Output;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stage_order_is_sequential() {
        assert_eq!(Stage::ORDER.len(), 5);
        for pair in Stage::ORDER.windows(2) {
            assert_eq!(pair[0].next(), Some(pair[1]));
            assert!(pair[0] < pair[1]);
        }
        assert_eq!(Stage::Output.next(), None);
        assert_eq!(Stage::Vertex.index(), 0);
        assert_eq!(Stage::Output.index(), 4);
    }

    #[test]
    fn principal_types_declare_their_stage() {
        assert_eq!(vertex::camera::Camera::STAGE, Stage::Vertex);
        assert_eq!(
            <assemble::primitive::DrawPrimitive as StageKind>::STAGE,
            Stage::Assemble
        );
        assert_eq!(
            <shade::shader::FlatColorShader as StageKind>::STAGE,
            Stage::Shade
        );
        assert_eq!(
            <crate::error::DisplayError as StageKind>::STAGE,
            Stage::Output
        );
    }
}
