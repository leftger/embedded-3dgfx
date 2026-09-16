//! # Minimal integration — bring your own framebuffer
//!
//! The smallest complete integration of the engine: **no window, no SDL, no
//! simulator**. The engine needs exactly one thing from your platform — a
//! `DrawTarget<Color = Rgb565>` that reports its own size — and it does the
//! rest.
//!
//! This is deliberately the shape you want on a microcontroller, where the
//! "display" is a panel you drive yourself (SPI / DSI / parallel) and `pixels`
//! lives in a `static`. The `Framebuffer` type below is `no_std`-compatible as
//! written; only `main`'s `println!`/`assert!` need `std`.
//!
//! ```text
//! cargo run --example integration_minimal --features std
//! ```

// Everything the engine needs is in one import.
use embedded_3dgfx::prelude::*;

// The rest is the `embedded-graphics` traits your framebuffer must implement.
use embedded_graphics_core::{
    Pixel,
    draw_target::DrawTarget,
    geometry::Size,
    pixelcolor::{Rgb565, RgbColor, WebColors},
    prelude::OriginDimensions,
};
use nalgebra::Point3;

/// The entire surface the engine requires: a slice of pixels plus its size.
///
/// Swap `&'a mut [Rgb565]` for whatever your platform hands you — a `static mut`
/// array, a `StaticCell`, a memory-mapped panel buffer — as long as you can
/// produce `&mut [Rgb565]`.
struct Framebuffer<'a> {
    pixels: &'a mut [Rgb565],
    width: usize,
    height: usize,
}

impl OriginDimensions for Framebuffer<'_> {
    fn size(&self) -> Size {
        Size::new(self.width as u32, self.height as u32)
    }
}

impl DrawTarget for Framebuffer<'_> {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            // Clip: the rasterizer can hand you pixels just off-screen.
            if point.x < 0 || point.y < 0 {
                continue;
            }
            let (x, y) = (point.x as usize, point.y as usize);
            if x < self.width && y < self.height {
                self.pixels[y * self.width + x] = color;
            }
        }
        Ok(())
    }

    /// The engine clears the frame itself, so override the default per-pixel
    /// path with a `memset`-style fill.
    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.pixels.fill(color);
        Ok(())
    }
}

/// A unit cube as plain slices — exactly the data layout you would emit from
/// your asset pipeline (`Geometry` borrows, so keep the arrays alive).
fn unit_cube() -> (Vec<[f32; 3]>, Vec<[usize; 3]>) {
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
    const WIDTH: usize = 320;
    const HEIGHT: usize = 240;

    // ── 1. Platform memory ──────────────────────────────────────────────────
    // Framebuffer + Z-buffer + command buffer. Sizes are yours to choose:
    // `WIDTH * HEIGHT * 2` bytes for the framebuffer, the same for a `u16`
    // depth buffer, and `MAX` primitives' worth for the command buffer.
    let mut pixels = vec![Rgb565::BLACK; WIDTH * HEIGHT];
    let mut zbuffer = vec![Z_MAX_VALUE; WIDTH * HEIGHT];
    let mut commands = CommandBuffer::<2048>::new();

    // ── 2. Engine ───────────────────────────────────────────────────────────
    let mut engine = K3dengine::new(WIDTH as u16, HEIGHT as u16);
    // Adopt the budget profile matching your MCU class (defaults to M33
    // balanced). This caps primitives/meshes/textures per frame.
    apply_default_caps(&mut engine);

    engine.camera.set_position(Point3::new(0.0, 1.5, 4.0));
    engine.camera.set_target(Point3::new(0.0, 0.0, 0.0));

    // ── 3. Scene ────────────────────────────────────────────────────────────
    let (vertices, faces) = unit_cube();
    // Only vertices and faces are required; the optional slots default to empty.
    let geometry = Geometry::new(&vertices, &faces);

    let mut mesh = K3dMesh::new(geometry);
    mesh.set_color(Rgb565::CSS_ORANGE);
    mesh.set_render_mode(RenderMode::Solid);

    // ── 4. One frame: clear → record → execute ──────────────────────────────
    {
        let mut fb = Framebuffer {
            pixels: &mut pixels,
            width: WIDTH,
            height: HEIGHT,
        };

        // `record` walks the scene on the CPU and appends commands; it never
        // touches the framebuffer, so it can run before a DMA transfer ends.
        engine
            .record(core::iter::once(&mesh), &mut commands, None)
            .unwrap();

        // `execute` replays the command buffer into your `DrawTarget`.
        let mut frame = FrameCtx {
            zbuffer: &mut zbuffer,
            width: WIDTH,
            height: HEIGHT,
        };
        engine
            .execute::<_, 2048>(&mut fb, &mut frame, &commands, None)
            .unwrap();
    } // the borrow of `pixels` ends here

    // ── 5. Present ──────────────────────────────────────────────────────────
    // Whatever your platform does next: memcpy into panel RAM, stream over
    // SPI, hand the slice to a DMA channel, or push into a swap chain.
    let lit = pixels.iter().filter(|p| **p != Rgb565::BLACK).count();
    println!("rendered {WIDTH}x{HEIGHT}, {lit} lit pixels");
    assert!(lit > 0, "the cube should have covered some pixels");
}
