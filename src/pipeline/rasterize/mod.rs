//! Pipeline stage 3 — rasterization.
//!
//! Screen-space primitives become covered pixels. [`raster`] holds the
//! scanline / triangle / line primitives, [`draw`] the z-buffered passes,
//! [`coverage`] the 1-bit front-to-back coverage target, and [`texture`] the
//! sampling source. [`tilebin`] bins recorded commands for tiled execution.

pub mod coverage;
pub mod draw;
pub mod raster;
#[cfg(feature = "textured")]
pub mod texture;
pub mod tilebin;
