//! Pipeline stage 2 — primitive assembly.
//!
//! Clip-space triangles are viewport-transformed and near-plane clipped into
//! the screen-space [`DrawPrimitive`](primitive::DrawPrimitive) values that the
//! rasterize stage consumes.

pub mod primitive;
