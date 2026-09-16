//! Pipeline stage 4 — fragment shading.
//!
//! [`shader`] holds the zero-cost `FragmentShader` interface and its
//! decorators, [`dither`] the ordered-dithering matrix toolkit, [`retro`] the
//! low-fidelity colour controls, and [`lights`] the runtime light set.

pub mod dither;
#[cfg(feature = "lighting")]
pub mod lights;
pub mod retro;
pub mod shader;
