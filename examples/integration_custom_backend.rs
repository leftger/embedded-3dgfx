//! # Custom display backend integration — plug in your panel
//!
//! The engine never talks to hardware directly. Everything display-specific
//! goes through two traits in the output stage:
//!
//! * [`DisplayBackend`] — "start pushing this framebuffer to the panel".
//! * [`DmaTransfer`] — the in-flight token that owns the framebuffer until the
//!   hardware is done with it and can cancel the transfer if dropped.
//!
//! That ownership split is the safety story: once a buffer is handed to a
//! backend, the type system makes it impossible to write to it again until the
//! transfer hands it back. This example implements both traits for a fake
//! panel, then drives a real [`StandardSwapChain`] on top of them — including
//! the back-pressure path you hit when the panel is slower than the renderer.
//!
//! Replace the `// TODO(hardware)` comments with your DMA controller and this
//! becomes a real board integration.
//!
//! ```text
//! cargo run --example integration_custom_backend --features std
//! ```

use core::cell::{Cell, RefCell};

// Two imports cover the entire engine surface used here:
//   * the prelude — frame driver, buffers, geometry, depth, errors
//   * the output stage — the panel-driver vocabulary
// Note that `FrameBuf` and `DMACapableFrameBufferBackend` come from the engine
// too: they appear in the backend trait signatures, so you do not need a direct
// dependency on `embedded-graphics-framebuf`.
use embedded_3dgfx::pipeline::output::{
    DMACapableFrameBufferBackend, DisplayBackend, DisplayRegion, DmaTransfer, FrameBuf,
    StandardSwapChain, TransferError,
};
use embedded_3dgfx::prelude::*;

use embedded_graphics::{
    pixelcolor::{Rgb565, RgbColor, WebColors},
    prelude::*,
};
use embedded_graphics_core::pixelcolor::IntoStorage;
use embedded_graphics_simulator::{
    OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent, Window, sdl2::Keycode,
};
use nalgebra::Point3;
use std::rc::Rc;
use std::thread;
use std::time::Duration;

const WIDTH: usize = 320;
const HEIGHT: usize = 240;
const COMMAND_CAPACITY: usize = 4096;

/// How many polls of [`DmaTransfer::is_done`] a fake transfer takes.
///
/// Stands in for "how long your panel takes to shift out a frame" — set to 0
/// for a panel that is always ready.
const TRANSFER_LATENCY: u32 = 3;

/// Telemetry the backend publishes to the application.
///
/// Real backends usually want this: transfer count, underruns, error counts.
#[derive(Default)]
struct PanelStats {
    transfers: u32,
    last_checksum: u32,
    last_lit_pixels: usize,
}

/// Framebuffer bytes as they are handed to the panel.
struct PanelTransfer<FB>
where
    FB: DMACapableFrameBufferBackend<Color = Rgb565>,
{
    framebuffer: Option<FrameBuf<Rgb565, FB>>,
    /// Polls remaining before the fake transfer completes.
    polls_left: Cell<u32>,
}

impl<FB> DmaTransfer for PanelTransfer<FB>
where
    FB: DMACapableFrameBufferBackend<Color = Rgb565>,
{
    type Buffer = FrameBuf<Rgb565, FB>;

    fn is_done(&self) -> bool {
        // TODO(hardware): read your DMA "transfer complete" flag here.
        let left = self.polls_left.get().saturating_sub(1);
        self.polls_left.set(left);
        left == 0
    }

    fn wait(self) -> Self::Buffer {
        // TODO(hardware): `wfi()` until the DMA-complete interrupt instead of
        // spinning. Returning `self` by value is what makes it impossible for
        // the caller to touch the buffer while the panel is still reading it.
        while !self.is_done() {
            core::hint::spin_loop();
        }
        self.framebuffer
            .expect("transfer token consumed after completion")
    }

    // A real implementation should cancel the DMA in `Drop` so that dropping a
    // token without waiting never leaves the controller pointing at a buffer
    // that is about to be reused:
    //
    // fn drop(&mut self) {
    //     if !self.is_done() { /* halt the channel */ }
    // }
}

/// A display backend: turns "here is a framebuffer" into "the panel is showing it".
struct PanelBackend {
    stats: Rc<RefCell<PanelStats>>,
}

impl PanelBackend {
    fn new(stats: Rc<RefCell<PanelStats>>) -> Self {
        Self { stats }
    }
}

impl<const W: usize, const H: usize, FB> DisplayBackend<W, H, FB> for PanelBackend
where
    FB: DMACapableFrameBufferBackend<Color = Rgb565>,
{
    type Transfer = PanelTransfer<FB>;

    fn start_dma_transfer(
        &mut self,
        framebuffer: FrameBuf<Rgb565, FB>,
    ) -> Result<Self::Transfer, TransferError<FB>> {
        // TODO(hardware): program the DMA controller with the framebuffer's
        // address and length, then let it run.
        //
        // Reading the pixels here is what a real driver does when it has to
        // push them by hand (SPI/parallel panels). For a memory-mapped panel
        // you would skip this entirely and just hand over the address.
        let mut stats = self.stats.borrow_mut();
        let mut checksum = 0u32;
        let mut lit = 0usize;
        for Pixel(_, color) in &framebuffer {
            // Raw 16-bit panel value, exactly what you would push over the bus.
            let raw: u16 = color.into_storage();
            checksum = checksum.wrapping_mul(31).wrapping_add(raw as u32);
            if color != Rgb565::BLACK {
                lit += 1;
            }
        }

        stats.transfers += 1;
        stats.last_checksum = checksum;
        stats.last_lit_pixels = lit;

        Ok(PanelTransfer {
            framebuffer: Some(framebuffer),
            polls_left: Cell::new(TRANSFER_LATENCY),
        })
    }

    fn start_dma_transfer_region(
        &mut self,
        framebuffer: FrameBuf<Rgb565, FB>,
        _region: DisplayRegion,
    ) -> Result<Self::Transfer, TransferError<FB>> {
        // This panel has no windowed-DMA support, so ignore the region and fall
        // back to a full-frame transfer. (The default trait method does the
        // same thing — it is spelled out here to show the hook exists.)
        DisplayBackend::<W, H, FB>::start_dma_transfer(self, framebuffer)
    }
}

fn cube() -> (Vec<[f32; 3]>, Vec<[usize; 3]>) {
    let vertices = vec![
        [-1.0, -1.0, 1.0],
        [1.0, -1.0, 1.0],
        [1.0, 1.0, 1.0],
        [-1.0, 1.0, 1.0],
        [-1.0, -1.0, -1.0],
        [1.0, -1.0, -1.0],
        [1.0, 1.0, -1.0],
        [-1.0, 1.0, -1.0],
    ];
    let faces = vec![
        [0, 1, 2],
        [0, 2, 3],
        [5, 4, 7],
        [5, 7, 6],
        [3, 2, 6],
        [3, 6, 7],
        [4, 5, 1],
        [4, 1, 0],
        [1, 5, 6],
        [1, 6, 2],
        [4, 0, 3],
        [4, 3, 7],
    ];
    (vertices, faces)
}

fn main() {
    // ── Two framebuffers, owned by the swap chain ───────────────────────────
    // On an MCU these are `static mut` arrays (or a `StaticCell`); leaking a
    // Vec just gives us the `&'static mut` the API wants on desktop.
    let front: &'static mut [Rgb565] = vec![Rgb565::BLACK; WIDTH * HEIGHT].leak();
    let back: &'static mut [Rgb565] = vec![Rgb565::BLACK; WIDTH * HEIGHT].leak();

    let stats = Rc::new(RefCell::new(PanelStats::default()));
    let backend = PanelBackend::new(Rc::clone(&stats));

    let mut swap_chain = StandardSwapChain::<WIDTH, HEIGHT, _>::from_static_slices(
        front, back, false, // byte order your panel expects
        backend,
    );

    // ── Engine + scene ──────────────────────────────────────────────────────
    let mut engine = K3dengine::new(WIDTH as u16, HEIGHT as u16);
    apply_default_caps(&mut engine);
    engine.camera.set_position(Point3::new(3.0, 2.5, 6.0));
    engine.camera.set_target(Point3::new(0.0, 0.0, 0.0));

    let (vertices, faces) = cube();
    let geometry = Geometry::new(&vertices, &faces);
    let mut mesh = K3dMesh::new(geometry);
    mesh.set_color(Rgb565::CSS_TURQUOISE);
    mesh.set_render_mode(RenderMode::Solid);

    // ── Application resources ───────────────────────────────────────────────
    let mut zbuffer = vec![Z_MAX_VALUE; WIDTH * HEIGHT];
    let mut commands = CommandBuffer::<COMMAND_CAPACITY>::new();

    let mut display = SimulatorDisplay::<Rgb565>::new(Size::new(WIDTH as u32, HEIGHT as u32));
    let output_settings = OutputSettingsBuilder::new().scale(1).build();
    let mut window = Window::new("Integration: custom display backend", &output_settings);

    println!("ESC to quit");

    let frame_limit: Option<u64> = std::env::var("E3DGFX_EXAMPLE_FRAMES")
        .ok()
        .and_then(|v| v.parse().ok());

    let mut angle = 0.0_f32;
    let mut frames = 0_u64;
    let mut busy_waits = 0_u64;

    window.update(&display);

    'running: loop {
        for event in window.events() {
            match event {
                SimulatorEvent::KeyDown {
                    keycode: Keycode::Escape,
                    ..
                }
                | SimulatorEvent::Quit => break 'running,
                _ => {}
            }
        }
        if frame_limit.is_some_and(|limit| frames >= limit) {
            break 'running;
        }

        angle += 0.01;
        engine
            .camera
            .set_position(Point3::new(6.0 * angle.sin(), 2.5, 6.0 * angle.cos()));

        // ── Render into the swap chain's back buffer ────────────────────────
        // The back buffer is *always* available: only the front buffer is ever
        // handed to the DMA engine, so rendering never blocks on the panel.
        {
            let back_buffer = swap_chain.get_back_buffer();
            back_buffer.clear(Rgb565::BLACK).unwrap();
            zbuffer.fill(Z_MAX_VALUE);

            engine
                .record(core::iter::once(&mesh), &mut commands, None)
                .unwrap();

            let mut frame = FrameCtx {
                zbuffer: &mut zbuffer,
                width: WIDTH,
                height: HEIGHT,
            };
            engine
                .execute::<_, COMMAND_CAPACITY>(back_buffer, &mut frame, &commands, None)
                .unwrap();

            // On real hardware you would NOT do this — the DMA transfer below
            // is what puts the frame on screen. The simulator has no DMA
            // controller, so copy the buffer over to look at it.
            let pixels: Vec<Pixel<Rgb565>> = back_buffer.into_iter().collect();
            display.draw_iter(pixels).unwrap();
        }

        // ── Present, handling panel back-pressure ───────────────────────────
        match swap_chain.try_present() {
            Ok(()) => {}
            Err(DisplayError::Busy) => {
                // The panel is still shifting out the previous frame. Wait for
                // it (or skip this present to drop a frame — your call), then
                // retry.
                swap_chain.wait_for_vsync();
                swap_chain
                    .try_present()
                    .expect("panel must be ready straight after vsync");
                busy_waits += 1;
            }
            Err(other) => {
                // `DisplayError` is `#[non_exhaustive]`: expect more variants.
                eprintln!("panel error: {other:?}");
                break 'running;
            }
        }

        window.update(&display);
        frames += 1;
        thread::sleep(Duration::from_millis(1));
    }

    let stats = stats.borrow();
    println!(
        "presented {} frames ({} waited on the panel, {} busy retries)\n\
         backend: {} transfers, {} lit pixels, checksum {:#010x}",
        swap_chain.frame_count(),
        frames,
        busy_waits,
        stats.transfers,
        stats.last_lit_pixels,
        stats.last_checksum,
    );
}
