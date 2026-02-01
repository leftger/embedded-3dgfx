//! Swap chain implementation for double-buffered rendering
//!
//! A swap chain manages two framebuffers (front and back) and coordinates
//! DMA transfers to eliminate visual tearing and improve performance.
//!
//! # Architecture
//! - **Back buffer**: CPU renders to this buffer
//! - **Front buffer**: Display hardware reads from this buffer (via DMA)
//! - **Swap**: When rendering completes, buffers are swapped atomically
//!
//! # Benefits
//! - No visual tearing (never display a partially-rendered frame)
//! - Better performance (CPU and display work in parallel)
//! - Predictable frame timing

use crate::display_backend::{DisplayBackend, DisplayError};
use crate::framebuffer::DmaReadyFramebuffer;

/// Double-buffered swap chain for tear-free rendering
///
/// Manages two framebuffers and coordinates swapping between them.
/// Uses a display backend for platform-specific DMA transfers.
///
/// # Type Parameters
/// - `W`: Framebuffer width in pixels
/// - `H`: Framebuffer height in pixels
/// - `B`: Display backend implementing `DisplayBackend<W, H>`
pub struct SwapChain<const W: usize, const H: usize, B: DisplayBackend<W, H>> {
    /// Front buffer (currently being displayed)
    front: DmaReadyFramebuffer<W, H>,
    /// Back buffer (currently being rendered to)
    back: DmaReadyFramebuffer<W, H>,
    /// Display backend for DMA transfers
    backend: B,
    /// Frame counter for statistics
    frame_count: u64,
}

impl<const W: usize, const H: usize, B: DisplayBackend<W, H>> SwapChain<W, H, B> {
    /// Create a new swap chain with two framebuffers
    ///
    /// # Arguments
    /// * `front_ptr` - Pointer to front framebuffer memory
    /// * `back_ptr` - Pointer to back framebuffer memory
    /// * `big_endian` - Whether to use big-endian byte order for colors
    /// * `backend` - Display backend for DMA operations
    ///
    /// # Safety
    /// The framebuffer pointers must point to valid memory regions of size W×H×2 bytes
    pub fn new(
        front_ptr: *mut core::ffi::c_void,
        back_ptr: *mut core::ffi::c_void,
        big_endian: bool,
        backend: B,
    ) -> Self {
        Self {
            front: DmaReadyFramebuffer::new(front_ptr, big_endian),
            back: DmaReadyFramebuffer::new(back_ptr, big_endian),
            backend,
            frame_count: 0,
        }
    }

    /// Get mutable reference to the back buffer for rendering
    ///
    /// Render all graphics operations to this buffer. When ready to display,
    /// call `present()` to swap buffers.
    pub fn get_back_buffer(&mut self) -> &mut DmaReadyFramebuffer<W, H> {
        &mut self.back
    }

    /// Get reference to the front buffer (currently being displayed)
    ///
    /// This is rarely needed, as you should render to the back buffer.
    pub fn get_front_buffer(&self) -> &DmaReadyFramebuffer<W, H> {
        &self.front
    }

    /// Present the back buffer to the display (blocking)
    ///
    /// This function:
    /// 1. Waits for any in-progress DMA transfer to complete
    /// 2. Swaps the front and back buffers
    /// 3. Starts a new DMA transfer of the (new) front buffer
    ///
    /// After this call returns, you can immediately start rendering to the
    /// (new) back buffer while the display hardware reads the front buffer.
    ///
    /// # Returns
    /// `Ok(())` if successful, `Err(DisplayError)` if DMA fails
    pub fn present(&mut self) -> Result<(), DisplayError> {
        // Wait for any in-progress DMA to finish
        self.backend.wait_for_dma();

        // Swap front and back buffers
        core::mem::swap(&mut self.front, &mut self.back);

        // Start DMA transfer of new front buffer
        self.backend.start_dma_transfer(&self.front)?;

        self.frame_count += 1;
        Ok(())
    }

    /// Try to present the back buffer (non-blocking)
    ///
    /// Like `present()`, but returns immediately with an error if DMA is busy.
    /// Useful for variable-rate rendering where you want to skip frames
    /// if rendering is too slow.
    ///
    /// # Returns
    /// - `Ok(())` if swap succeeded
    /// - `Err(DisplayError::Busy)` if DMA is still transferring
    /// - `Err(DisplayError::HardwareError)` on other errors
    pub fn try_present(&mut self) -> Result<(), DisplayError> {
        // Check if DMA is ready
        if !self.backend.is_dma_ready() {
            return Err(DisplayError::Busy);
        }

        // Swap front and back buffers
        core::mem::swap(&mut self.front, &mut self.back);

        // Start DMA transfer of new front buffer
        self.backend.start_dma_transfer(&self.front)?;

        self.frame_count += 1;
        Ok(())
    }

    /// Wait for the current DMA transfer to complete
    ///
    /// Useful if you need to ensure a frame has been fully displayed
    /// before proceeding (e.g., before taking a screenshot or exiting).
    pub fn wait_for_vsync(&mut self) {
        self.backend.wait_for_dma();
    }

    /// Check if DMA is ready for a new transfer
    ///
    /// Returns `true` if `try_present()` would succeed, `false` otherwise.
    pub fn is_ready(&self) -> bool {
        self.backend.is_dma_ready()
    }

    /// Get the number of frames presented
    pub fn frame_count(&self) -> u64 {
        self.frame_count
    }

    /// Reset the frame counter
    pub fn reset_frame_count(&mut self) {
        self.frame_count = 0;
    }

    /// Get framebuffer dimensions
    pub fn dimensions(&self) -> (usize, usize) {
        (W, H)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::display_backend::SimulatorBackend;

    #[test]
    fn test_swapchain_creation() {
        let mut fb0_data = [0u16; 320 * 240];
        let mut fb1_data = [0u16; 320 * 240];

        let swap_chain = SwapChain::<320, 240, _>::new(
            fb0_data.as_mut_ptr() as *mut core::ffi::c_void,
            fb1_data.as_mut_ptr() as *mut core::ffi::c_void,
            false,
            SimulatorBackend::new(),
        );

        assert_eq!(swap_chain.dimensions(), (320, 240));
        assert_eq!(swap_chain.frame_count(), 0);
        assert!(swap_chain.is_ready());
    }

    #[test]
    fn test_swapchain_present() {
        let mut fb0_data = [0u16; 320 * 240];
        let mut fb1_data = [0u16; 320 * 240];

        let mut swap_chain = SwapChain::<320, 240, _>::new(
            fb0_data.as_mut_ptr() as *mut core::ffi::c_void,
            fb1_data.as_mut_ptr() as *mut core::ffi::c_void,
            false,
            SimulatorBackend::new(),
        );

        // Render to back buffer (just fill with a value)
        swap_chain.get_back_buffer().as_mut_slice().fill(0x1234);

        // Present should succeed
        assert!(swap_chain.present().is_ok());
        assert_eq!(swap_chain.frame_count(), 1);

        // Back buffer should now be different from what we filled
        // (because buffers were swapped)
        let back = swap_chain.get_back_buffer();
        // The back buffer is now the old front buffer, which should be 0
        assert_eq!(back.as_slice()[0], 0);

        // Front buffer should have our rendered data
        assert_eq!(swap_chain.get_front_buffer().as_slice()[0], 0x1234);
    }

    #[test]
    fn test_swapchain_multiple_presents() {
        let mut fb0_data = [0u16; 320 * 240];
        let mut fb1_data = [0u16; 320 * 240];

        let mut swap_chain = SwapChain::<320, 240, _>::new(
            fb0_data.as_mut_ptr() as *mut core::ffi::c_void,
            fb1_data.as_mut_ptr() as *mut core::ffi::c_void,
            false,
            SimulatorBackend::new(),
        );

        // Present multiple frames
        for i in 0..5 {
            swap_chain.get_back_buffer().as_mut_slice().fill(i as u16);
            assert!(swap_chain.present().is_ok());
        }

        assert_eq!(swap_chain.frame_count(), 5);
    }

    #[test]
    fn test_swapchain_try_present() {
        let mut fb0_data = [0u16; 320 * 240];
        let mut fb1_data = [0u16; 320 * 240];

        let mut swap_chain = SwapChain::<320, 240, _>::new(
            fb0_data.as_mut_ptr() as *mut core::ffi::c_void,
            fb1_data.as_mut_ptr() as *mut core::ffi::c_void,
            false,
            SimulatorBackend::new(),
        );

        // try_present should succeed (SimulatorBackend is always ready)
        assert!(swap_chain.try_present().is_ok());
        assert_eq!(swap_chain.frame_count(), 1);
    }

    #[test]
    fn test_swapchain_frame_counter() {
        let mut fb0_data = [0u16; 320 * 240];
        let mut fb1_data = [0u16; 320 * 240];

        let mut swap_chain = SwapChain::<320, 240, _>::new(
            fb0_data.as_mut_ptr() as *mut core::ffi::c_void,
            fb1_data.as_mut_ptr() as *mut core::ffi::c_void,
            false,
            SimulatorBackend::new(),
        );

        assert_eq!(swap_chain.frame_count(), 0);

        swap_chain.present().unwrap();
        assert_eq!(swap_chain.frame_count(), 1);

        swap_chain.present().unwrap();
        assert_eq!(swap_chain.frame_count(), 2);

        swap_chain.reset_frame_count();
        assert_eq!(swap_chain.frame_count(), 0);
    }
}
