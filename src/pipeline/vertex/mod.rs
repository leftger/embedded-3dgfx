//! Pipeline stage 1 — vertex processing.
//!
//! Model-space geometry ([`mesh`], [`shapes`]), the [`camera`]'s view /
//! projection matrices, frustum and backface culling ([`bounds`],
//! [`view_frustum`]), LOD selection ([`lod`]), and the clip-space
//! [`transform`] that hands vertices to the assemble stage.

#[cfg(feature = "aabb-cull")]
pub mod bounds;
pub mod camera;
pub mod camera_controller;
pub mod lod;
pub mod mesh;
#[cfg(feature = "render-layers")]
pub mod render_layers;
pub mod shapes;
pub mod transform;
pub mod view_frustum;
