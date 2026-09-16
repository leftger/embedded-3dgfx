# Application Integration

How to put `embedded-3dgfx` inside an application you already have.

The engine has **no `run()`**. It is a library of stages you call: you own the
loop, the buffers, and the clock. This guide walks the six seams where your code
and the engine meet, in the order you will hit them.

Every snippet below has a complete, runnable counterpart in
[`examples/integration_*.rs`](../examples) — see the [example index](#runnable-examples)
at the end.

---

## The contract at a glance

You provide:

| You own | Type | Notes |
|---------|------|-------|
| Framebuffer | any `DrawTarget<Color = Rgb565> + OriginDimensions` | your panel, your `&mut [Rgb565]`, or the simulator |
| Depth buffer | `&mut [ZDepth]` | exactly `width * height` entries |
| Command buffer | `CommandBuffer<MAX>` | `heapless`; capacity is primitives, not bytes |

You call, per frame:

```text
clear → record → execute → present
```

---

## Imports: one line for the engine

Everything the frame loop touches comes from the prelude:

```rust
use embedded_3dgfx::prelude::*;
```

That single import covers the frame driver (`K3dengine`), the buffers
(`CommandBuffer`, `FrameCtx`, `ZDepth`, `Z_MAX_VALUE`), geometry (`Geometry`,
`K3dMesh`, `RenderMode`, `Camera`), primitives (`DrawPrimitive`), per-pass
configuration (`RasterState`, `draw_zbuffered_with_state`, `FogConfig`,
`DitherConfig`, `apply_default_caps`), the error taxonomy (`RenderError`,
`BudgetKind`, `DisplayError`) and the stage markers (`Stage`, `StageKind`).
Feature-gated types (`PointLight`, `Texture` / `TextureManager`) join it when the
feature is enabled.

Writing a **display backend** needs exactly one more import. The output stage
gathers its own vocabulary, so a driver never reaches into four submodules:

```rust
use embedded_3dgfx::pipeline::output::{
    DMACapableFrameBufferBackend, DisplayBackend, DisplayRegion, DmaTransfer, FrameBuf,
    StandardSwapChain, TransferError,
};
```

`FrameBuf` and `DMACapableFrameBufferBackend` are re-exported there on purpose:
they appear in the backend trait signatures, so implementing a backend does not
require `embedded-graphics-framebuf` in your own `Cargo.toml`.

Everything else — `telemetry`, `config` policies, `scene_format`, `physics`,
or an individual stage module like `pipeline::shade::shader` — stays explicitly
named, so it is always obvious which subsystem you reached into.

---

## 1. Framebuffer, depth buffer, command buffer

The engine needs a pixel sink. The only trait involved is
`embedded_graphics_core::draw_target::DrawTarget`; the whole adapter is ~30
lines:

```rust
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
            if point.x < 0 || point.y < 0 {
                continue; // the rasterizer can emit off-screen pixels
            }
            let (x, y) = (point.x as usize, point.y as usize);
            if x < self.width && y < self.height {
                self.pixels[y * self.width + x] = color;
            }
        }
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.pixels.fill(color); // override the default per-pixel path
        Ok(())
    }
}
```

Full version: [`integration_minimal.rs`](../examples/integration_minimal.rs).

**Depth buffer.** `ZDepth` is `u32`, or `u16` with the `depth-u16` feature
(halves the memory). Fill it with `Z_MAX_VALUE` each frame:

```rust
let mut zbuffer = vec![Z_MAX_VALUE; WIDTH * HEIGHT];
```

`FrameCtx` validates the length when you execute, so a mismatch is a clean
`RenderError::OutOfBudget(BudgetKind::ZBufferLength { .. })` rather than a
silent wrap.

**Command buffer.** `CommandBuffer<MAX>` holds up to `MAX` primitives. Size it
above the largest frame you expect, e.g. `MAX ≈ max_draw_primitives` from your
[profile](#4-budgets-telemetry-and-degradation):

```rust
let mut commands = CommandBuffer::<8192>::new();
```

Overflow is reported, not fatal: `record` returns
`RenderError::OutOfBudget(BudgetKind::DrawPrimitives { attempted, max })`, and
`record_with_fallback` / `record_with_degradation` can shed work instead.

**Memory math per frame** — a `320x240` configuration with `depth-u16`:

| Buffer | Bytes |
|--------|-------|
| Framebuffer (`Rgb565`) | 153,600 |
| Z-buffer (`u16`) | 153,600 |
| Command buffer | `MAX * size_of::<RenderCommand>()` |
| Double buffer | +153,600 |

See [`docs/feature-size.md`](feature-size.md) for measured `.text` budgets.

---

## 2. The frame loop

Two calls, deliberately split:

```rust
// Record: CPU work — transforms, culling, primitive assembly.
engine.record(core::iter::once(&mesh), &mut commands, None)?;

// Execute: rasterize the recorded commands into your target.
let mut frame = FrameCtx { zbuffer: &mut zbuffer, width: WIDTH, height: HEIGHT };
engine.execute::<_, 8192>(&mut fb, &mut frame, &commands, None)?;
```

Why the split matters for integration:

- `record` takes `&self` and never touches the framebuffer, so it is safe to run
  **while DMA is still pushing the previous frame** — the CPU is not idle.
- `execute` writes only to the target you pass. It does not allocate and does not
  own your buffers.
- You must clear both buffers yourself each frame; the engine does not clear
  unless you record a clear command.

`record_with_degradation` takes `&mut self` because applying a degradation step
changes engine state (quality tier).

Full version: [`integration_app_loop.rs`](../examples/integration_app_loop.rs).

---

## 3. Presenting to a panel

### The simple case

You already own the pixel slice, so presentation is whatever your platform does:
`memcpy` into panel RAM, stream over SPI, or hand the slice to a DMA channel.

### Swap chain

For double buffering with an ownership-safe handoff, use a swap chain:

```rust
use embedded_3dgfx::pipeline::output::swapchain::StandardSwapChain;

let mut swap = StandardSwapChain::<320, 240, _>::from_static_slices(
    front_data,   // &'static mut [Rgb565]
    back_data,    // &'static mut [Rgb565]
    false,        // true if your panel wants big-endian pixels
    MyPanelBackend::new(),
);

loop {
    // The back buffer is always available; DMA only ever touches the front.
    let back = swap.get_back_buffer();
    back.clear(Rgb565::BLACK)?;
    // …record + execute into `back`…

    swap.present()?;          // blocking: waits for the previous transfer
    // or: swap.try_present()?  -> Err(DisplayError::Busy) if still transferring
}
```

`DisplayError::Busy` is **normal back-pressure**, not a failure: wait with
`swap.wait_for_vsync()` (or drop the frame) and retry.

### Your own display backend

Presentation goes through two traits in the output stage:

- `DisplayBackend<W, H, FB>` — starts a transfer of a framebuffer.
- `DmaTransfer` — the in-flight token. It **owns** the framebuffer until the
  hardware finishes, so the borrow checker prevents writing to a buffer the
  panel is still reading. Implement `Drop` to cancel the DMA channel, so a
  dropped token can never leave the controller pointing at freed memory.

```rust
impl<FB: DMACapableFrameBufferBackend<Color = Rgb565>> DmaTransfer for MyTransfer<FB> {
    type Buffer = FrameBuf<Rgb565, FB>;

    fn is_done(&self) -> bool { /* read the DMA status flag */ true }
    fn wait(self) -> Self::Buffer { /* WFI until the ISR fires, then return it */ }
}
```

Full version (including back-pressure handling):
[`integration_custom_backend.rs`](../examples/integration_custom_backend.rs).

### Bare metal: signalling completion from an ISR

`CompletionSlot` is runtime-agnostic. Signal it from the DMA interrupt, then use
`WaitTransfer` as the token — it polls the same slot synchronously, and doubles
as a future under an async executor.

```rust
use embedded_3dgfx::pipeline::output::completion::{CompletionSlot, WaitTransfer};

static DMA_DONE: CompletionSlot = CompletionSlot::new();

#[interrupt]
fn DMA_IRQ() {
    DMA_DONE.signal();          // safe from interrupt context
}

// in your backend:
fn start_dma_transfer(&mut self, fb: FrameBuf<Rgb565, FB>) -> Result<Self::Transfer, _> {
    // …program the DMA channel with `fb`'s address and length…
    Ok(WaitTransfer::new(fb, &DMA_DONE))   // resets the slot for you
}
```

### Async

If your executor can await, both the transfer and the present are awaitable:

```rust
swap.present_async().await?;          // requires AsyncDmaTransfer on the token
swap.wait_for_vsync_async().await;
```

### Triple buffering

The `triple-buffering` feature adds `TripleSwapChain`, which rotates render /
ready / display buffers so the CPU never waits for scan-out. Use it when your
frame times are bursty.

---

## 4. Budgets, telemetry and degradation

`apply_default_caps` adopts a per-MCU budget profile (defaults to
`PROFILE_M33_BALANCED`, which caps a frame at **320×240**):

```rust
use embedded_3dgfx::config::{apply_default_caps, PROFILE_M55_PERF};

apply_default_caps(&mut engine);        // default profile
engine.set_caps(PROFILE_M55_PERF);      // or pick one explicitly
engine.clear_caps();                    // or lift the caps entirely
```

Runtime override for desktop builds: `EMBEDDED_3DGFX_CAPS=off` (or
`=m0|m3|m4|m33|m55`). Compile-time: the `desktop-unbounded` feature.

Telemetry is opt-in and free when unused — pass `None`:

```rust
let mut record_tel = RecordTelemetry::default();
let mut execute_tel = ExecuteTelemetry::default();
engine.record(&meshes, &mut commands, Some(&mut record_tel))?;
engine.execute::<_, 8192>(&mut fb, &mut frame, &commands, Some(&mut execute_tel))?;
```

`RecordTelemetry` gives `meshes_visible`, `unique_textures`, `draw_commands`;
`ExecuteTelemetry` gives `commands_total` and the clear counts. Feed them to a
HUD or a host-side logger.

**Graceful degradation** — instead of failing a heavy frame, shed work:

```rust
static DEGRADATION: [DegradationStep; 2] = [
    DegradationStep::MeshDecimationStride(2),
    DegradationStep::DowngradeQuality,
];

let outcome = engine.record_with_degradation(
    &meshes,
    &mut commands,
    DegradationPolicy { steps: &DEGRADATION },
    Some(&mut record_tel),
)?;
if outcome.used_degradation {
    // surface it: the frame is cheaper but visibly different
}
```

Both steps are no-ops on a frame that fits, so the policy can stay enabled
permanently.

---

## 5. Assets & textures

Textures live in a fixed-capacity `TextureManager<N>` holding `'static` pixel
data, and are looked up by the id on `Geometry::texture_id`:

```rust
use embedded_3dgfx::pipeline::rasterize::texture::{Texture, TextureManager};

let mut textures = TextureManager::<8>::new();
let id = textures.add_texture(Texture::new(include_bytes!(...), 32, 32)).unwrap();

let mut frame = FrameCtx { zbuffer: &mut zbuffer, width: W, height: H };
engine.execute_with_textures::<_, 8192, 8>(
    &mut fb, &mut frame, &commands, &textures, None,
)?;
```

`Geometry` borrows its slices, so keep vertex/face/texture arrays alive for at
least as long as the `K3dMesh` — `&'static` data straight from flash is ideal
(see [`docs/asset-pipeline.md`](asset-pipeline.md) for the offline generator).

---

## 6. Overlays and custom shading

**HUD / 2D overlay.** Anything that implements `DrawTarget` can draw on top of
the 3D frame — including `embedded-graphics` primitives and text:

```rust
engine.execute::<_, 8192>(&mut display, &mut frame, &commands, None)?;

let style = MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_WHITE);
Text::new(&format!("{fps:.1} fps"), Point::new(4, 10), style).draw(&mut display)?;
```

Draw it *after* `execute`, and remember the z-buffer is independent of your 2D
overlay.

**Custom passes that match the engine's effects.** `engine.raster_state(w, h)`
returns the engine's own per-pass configuration — fog, dither, tint, palette,
stipple, depth bias, depth interpolation. Feed it to a raster entry point and
your hand-rolled geometry is pixel-identical to the engine's:

```rust
use embedded_3dgfx::pipeline::rasterize::draw::draw_zbuffered_with_state;

let state = engine.raster_state(WIDTH, HEIGHT);
draw_zbuffered_with_state(primitive, &mut display, &mut zbuffer, &state);
```

**Custom materials.** `FragmentShader` is the seam for procedural shading. The
built-in decorators wrap any shader, so fog/dither apply to your material too:

```rust
let material = MyWaveShader { base_color, time };
let fogged   = FogShader   { inner: material, fog: &fog_config };
let shaded   = DitherShader{ inner: fogged,   dither: &dither_config };

let color = shaded.shade(x, y, z, interpolants);
```

Note this is an evaluation API: you compose the stack and sample it per
fragment. The engine's built-in rasterization is driven by `DrawPrimitive`, so a
custom material is used either to compute colours you feed into primitives, or
in your own raster pass on top of `RasterState`.

---

## Choosing features per target

| Target | Suggestion |
|--------|------------|
| Desktop / simulator | `std`, `desktop-unbounded` |
| Cortex-M0/M0+ | `row_width_96`, `depth-u16`, no `lighting`/`textured` |
| Cortex-M4F | `row_width_160`, `depth-u16`, `lighting` |
| Cortex-M33/M55 | `row_width_240`, `depth-u16`, `lighting`, `textured` |
| Doom-style raycaster | add `raycast`, `hud` |
| Physics / animation | add `physics`, `scene` |

`row_width_*` sizes the internal scanline buffer; it must cover the frame width.
If several are enabled the widest wins deterministically.

---

## Gotchas

1. **Clear both buffers before `execute`.** The engine does not clear for you.
2. **`FrameCtx` depth length must equal `width * height`** — otherwise
   `RenderError::OutOfBudget(BudgetKind::ZBufferLength)`.
3. **Default caps are 320×240.** A larger frame fails with
   `OutOfBudget(FramebufferDimensions { .. })` until you raise the caps
   (`desktop-unbounded`, `EMBEDDED_3DGFX_CAPS=off`, or `set_caps`).
4. **Command buffer capacity is primitives, not pixels.** Undersizing it
   surfaces as `BudgetKind::DrawPrimitives`.
5. **`DisplayError::Busy` is routine.** Handle it as back-pressure.
6. **Public enums are `#[non_exhaustive]`.** Match with a `_` arm — including
   `DisplayError`, `RenderMode` and the error taxonomy.
7. **`record_with_degradation` needs `&mut engine`**, unlike plain `record`.
8. **Geometry borrows its data.** `K3dMesh` cannot outlive the arrays behind its
   `Geometry`. Build one with `Geometry::new(&vertices, &faces)` and add the
   optional slots with the `with_*` builders — or use
   `Geometry { vertices, faces, ..Default::default() }` if you already hold the
   data as named struct fields.
9. **With the simulator, call `window.update()` once before `window.events()`** —
   the event iterator panics otherwise.

---

## Runnable examples

| Example | Seam it demonstrates | Run |
|---------|---------------------|-----|
| [`integration_minimal`](../examples/integration_minimal.rs) | your own `DrawTarget`, one frame, no SDL | `cargo run --example integration_minimal --features std` |
| [`integration_app_loop`](../examples/integration_app_loop.rs) | fixed timestep, input, telemetry, degradation, `RasterState` overlay | `cargo run --example integration_app_loop --features std` |
| [`integration_custom_backend`](../examples/integration_custom_backend.rs) | `DisplayBackend` / `DmaTransfer`, swap chain, back-pressure | `cargo run --example integration_custom_backend --features std` |

All three accept `E3DGFX_EXAMPLE_FRAMES=<n>` to exit after `n` frames, which
makes them usable as headless smoke tests:

```bash
# No display server needed.
SDL_VIDEODRIVER=dummy E3DGFX_EXAMPLE_FRAMES=60 \
  cargo run --example integration_app_loop --features std
```

### Related reading

| Document | Topic |
|----------|-------|
| [`MIGRATION.md`](../MIGRATION.md) | Upgrading from the pre-`pipeline` module layout |
| [`docs/backend-integration.md`](backend-integration.md) | Board bring-up, memory sizing, hardware profiling |
| [`docs/caps-and-telemetry.md`](caps-and-telemetry.md) | Caps, telemetry, CI budgets |
| [`docs/feature-size.md`](feature-size.md) | Slim vs full flash (`.text`) budgets |
| [`docs/asset-pipeline.md`](asset-pipeline.md) | Offline assets, scene streaming |
