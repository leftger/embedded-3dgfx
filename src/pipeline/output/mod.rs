//! Pipeline stage 5 — output.
//!
//! Framebuffer backends ([`display_backend`]), triple-buffered presentation
//! ([`swapchain`]), async DMA completion slots ([`completion`]), and the 2D
//! [`hud`] overlay composited over the rendered frame.
//!
//! # Writing a display backend
//!
//! The traits below name types from `embedded-graphics-framebuf` in their
//! signatures, so they are re-exported here: a driver needs exactly one `use`:
//!
//! ```
//! use embedded_3dgfx::pipeline::output::{
//!     DisplayBackend, DisplayRegion, DmaTransfer, FrameBuf, StandardSwapChain, TransferError,
//! };
//! ```

pub mod completion;
pub mod display_backend;
#[cfg(feature = "hud")]
pub mod hud;
pub mod swapchain;

// Stage facade: the vocabulary for driving a panel, gathered so that a board
// driver imports one path instead of reaching into four submodules. Individual
// submodules stay minimal — this is the deliberate, curated surface for the
// stage as a whole.
pub use completion::{CompletionSlot, WaitTransfer};
pub use display_backend::{
    AsyncDmaTransfer, DisplayBackend, DisplayRegion, DmaTransfer, HardwareAccelerator,
    SimulatorBackend, TransferError,
};
pub use swapchain::{StandardSwapChain, SwapChain};

// Re-exported because `DisplayBackend` and `DmaTransfer` name them in their
// signatures: an implementor must be able to spell these types without adding
// `embedded-graphics-framebuf` to its own dependencies.
pub use embedded_graphics_framebuf::{FrameBuf, backends::DMACapableFrameBufferBackend};

#[cfg(feature = "triple-buffering")]
pub use swapchain::{StandardTripleSwapChain, TripleSwapChain};
