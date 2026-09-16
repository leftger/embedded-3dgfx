# Migration guide

## Pipeline layout (0.6.x → 0.7)

The rendering modules were reorganised into an explicit graphics pipeline. This
release is **source-breaking**: most public paths moved one level deeper, under
`embedded_3dgfx::pipeline::*`. No feature flag changed and no rendering
behaviour changed.

If you only need the common types, the quickest fix is the prelude:

```rust
use embedded_3dgfx::prelude::*;
```

Everything under "Module moves" below is a pure path change; the types,
generics, and signatures are otherwise identical unless called out in
[API changes](#api-changes).

---

## Why

The crate now reads as a five-stage pipeline, and each stage owns one module:

| # | Stage | Module | Input → output |
|---|-------|--------|----------------|
| 1 | `Stage::Vertex`    | `pipeline::vertex`    | model-space mesh → clip-space vertices |
| 2 | `Stage::Assemble`  | `pipeline::assemble`  | clip-space vertices → screen-space `DrawPrimitive` |
| 3 | `Stage::Rasterize` | `pipeline::rasterize` | screen primitives → covered pixels |
| 4 | `Stage::Shade`     | `pipeline::shade`     | fragments → `Rgb565` colours |
| 5 | `Stage::Output`    | `pipeline::output`    | framebuffer → presented frame |

`pipeline::command_buffer` carries the recorded commands between the record
half (stages 1–2) and the execute half (stages 3–5); `pipeline::renderer`
drives the execute half; `engine` is the frame driver on top. Shared per-pass
configuration lives in `pipeline::effects`.

A stage may depend on lower-numbered stages, `effects`, and the crate-root core
types — never on a higher-numbered stage. The one deliberate exception is that
`rasterize` invokes the fragment programs defined in `shade`, so `shade` sits
*below* `rasterize` in dependency order even though it runs afterwards.

---

## Module moves

Replace the left column with the right column. All are `embedded_3dgfx::`
paths; `crate::` internal paths follow the same mapping.

| Old | New |
|-----|-----|
| `embedded_3dgfx::camera` | `embedded_3dgfx::pipeline::vertex::camera` |
| `embedded_3dgfx::camera_controller` | `embedded_3dgfx::pipeline::vertex::camera_controller` |
| `embedded_3dgfx::mesh` | `embedded_3dgfx::pipeline::vertex::mesh` |
| `embedded_3dgfx::shapes` | `embedded_3dgfx::pipeline::vertex::shapes` |
| `embedded_3dgfx::bounds` *(feature `aabb-cull`)* | `embedded_3dgfx::pipeline::vertex::bounds` |
| `embedded_3dgfx::view_frustum` | `embedded_3dgfx::pipeline::vertex::view_frustum` |
| `embedded_3dgfx::lod` | `embedded_3dgfx::pipeline::vertex::lod` |
| `embedded_3dgfx::render_layers` *(feature `render-layers`)* | `embedded_3dgfx::pipeline::vertex::render_layers` |
| `embedded_3dgfx::primitive` | `embedded_3dgfx::pipeline::assemble::primitive` |
| `embedded_3dgfx::raster` | `embedded_3dgfx::pipeline::rasterize::raster` |
| `embedded_3dgfx::draw` | `embedded_3dgfx::pipeline::rasterize::draw` |
| `embedded_3dgfx::draw::effects` | `embedded_3dgfx::pipeline::effects` |
| `embedded_3dgfx::texture` *(feature `textured`)* | `embedded_3dgfx::pipeline::rasterize::texture` |
| `embedded_3dgfx::tilebin` | `embedded_3dgfx::pipeline::rasterize::tilebin` |
| `embedded_3dgfx::bsp::coverage` | `embedded_3dgfx::pipeline::rasterize::coverage` |
| `embedded_3dgfx::shader` | `embedded_3dgfx::pipeline::shade::shader` |
| `embedded_3dgfx::retro` | `embedded_3dgfx::pipeline::shade::retro` |
| `embedded_3dgfx::lights` *(feature `lighting`)* | `embedded_3dgfx::pipeline::shade::lights` |
| `embedded_3dgfx::dither` | `embedded_3dgfx::pipeline::shade::dither` |
| `embedded_3dgfx::display_backend` | `embedded_3dgfx::pipeline::output::display_backend` |
| `embedded_3dgfx::swapchain` | `embedded_3dgfx::pipeline::output::swapchain` |
| `embedded_3dgfx::completion` | `embedded_3dgfx::pipeline::output::completion` |
| `embedded_3dgfx::hud` *(feature `hud`)* | `embedded_3dgfx::pipeline::output::hud` |
| `embedded_3dgfx::command_buffer` | `embedded_3dgfx::pipeline::command_buffer` |
| `embedded_3dgfx::renderer` | `embedded_3dgfx::pipeline::renderer` |

The `retro` submodules are still re-exported from their parent, so both
`pipeline::shade::retro::PaletteMode` and
`pipeline::shade::retro::palette::PaletteMode` work.

### Modules that did **not** move

`engine`, `color`, `config`, `error`, `simd_dsp`, and every subsystem:
`physics`, `softbody`, `raycast`, `bsp`, `sector_lights`, `ray_primitive`,
`navmesh`, `occlusion`, `painters`, `matcap`, `lens_flare`, `decal`,
`state_machine`, `pool`, `curve`, `tween`, `transform_anim`,
`color_gradient`, `animation`, `absm`, `skeleton`, `character`, `particles`,
`billboard`, `gizmos`, `simplex_stroke_font`, `input`, `bridge`, `embassy`,
`hardware_profile`, `perfcounter`, `scene_format`, `scene_stream`,
`telemetry`, `timer`.

Crate-root items are unchanged: `ZDepth`, `Z_MAX_VALUE`, `DEPTH_EPSILON`,
`to_zdepth`, `clear_zbuffer`, `MeshRayCastHit`, `mesh_ray_cast`,
`mesh_ray_cast_bounded`, `mesh_ray_cast_mesh`.

---

## Types that moved between modules

| Old path | New path |
|----------|----------|
| `draw::FogConfig`, `draw::DitherConfig` | `pipeline::effects::{FogConfig, DitherConfig}` |
| `shader::FogConfig`, `shader::DitherConfig` | `pipeline::effects::{FogConfig, DitherConfig}` |
| `draw::DepthInterpolationMode`, `draw::DepthBias`, `draw::InterlaceField` | `pipeline::effects::{DepthInterpolationMode, DepthBias, InterlaceField}` |
| `draw::ScreenDoorConfig`, `draw::CheckerboardField` | `pipeline::effects::{ScreenDoorConfig, CheckerboardField}` |
| `display_backend::DisplayError` | `error::DisplayError` |
| `bsp::coverage::CoverageBuffer` | `pipeline::rasterize::coverage::CoverageBuffer` |
| `draw::ReadPixel`, `draw::PixelRead`, `raster::ReadPixel` | `pipeline::rasterize::raster::aa::{ReadPixel, PixelRead}` |
| `telemetry::BspTelemetry` | **removed re-export** — use `bsp::BspTelemetry` |

Notes:

* `FogConfig` and `DitherConfig` now have exactly one definition (they used to
  be duplicated in `draw::effects` and `shader::{fog,dither}`). `shader::fog`
  and `shader::dither` no longer re-export them.
* `DisplayError` moved to the core `error` module because
  `RenderError::Backend(DisplayError)` needs it; `display_backend` imports it
  from there.
* The duplicated `ReadPixel` trait was unified onto the raster layer's copy.

---

## API changes

### Raster state (new)

`pipeline::rasterize::draw::RasterState` bundles the per-pass raster config that
used to be a tail of positional arguments:

```rust
use embedded_3dgfx::pipeline::effects::{DitherConfig, FogConfig};
use embedded_3dgfx::pipeline::rasterize::draw::{
    RasterState, draw_zbuffered_with_state, draw_zbuffered_with_textures_state,
};

let fog = FogConfig::new(Rgb565::new(0, 0, 0), 4.0, 24.0);
let state = RasterState::new(320, 240).with_fog(Some(&fog));

draw_zbuffered_with_state(primitive, &mut fb, &mut zbuffer, &state);
```

`K3dengine` lowers its own configuration into one with:

```rust
let state = engine.raster_state(320, 240);
```

The previous entry points — `draw_zbuffered`, `draw_zbuffered_with_effects`,
`draw_zbuffered_with_options`, `draw_zbuffered_with_bias`,
`draw_zbuffered_with_textures`, `draw_zbuffered_with_textures_mapped` — are
**still present** as thin adapters over the state API, so they keep compiling.
Prefer the `*_with_state` variants for new code.

### Execute entry points (breaking)

`RasterState` is now the single per-pass configuration bundle for the whole
rasterize stage, and the execute driver consumes it instead of a positional
parameter tail. The eight `execute_commands*` entry points collapse to five, one
per dispatch strategy, and the three that carry configuration all take
`&RasterState<'_>`.

| Before | After |
|--------|-------|
| `execute_commands(fb, frame, cmd, fog)` | `execute_commands(fb, frame, cmd, &state)` |
| `execute_commands_with_dirty_region(fb, frame, cmd, fog)` | `execute_commands(fb, frame, cmd, &state)` |
| `execute_commands_with_dirty_region_effects(fb, frame, cmd, fog, dither, screen_tint, stipple_mode, palette_mode, sky, camera_dir)` | `execute_commands(fb, frame, cmd, &state)` |
| `execute_commands_with_dirty_region_effects_textured(…, texture_manager, fog, …, camera_dir)` | `execute_commands_textured(fb, frame, cmd, texture_manager, &state)` |
| `execute_commands_tiled(fb, frame, cmd, tile, fog)` | `execute_commands_tiled(fb, frame, cmd, tile, &state)` |
| `execute_commands_tiled_effects(…, fog, …, camera_dir)` | `execute_commands_tiled(fb, frame, cmd, tile, &state)` |
| `execute_commands_2xssaa(fb, frame, cmd)` | unchanged (consumes no effects) |
| `execute_commands_with_picking(…)` | unchanged |

`RasterState` gained the two frame-level knobs the driver used to take
separately, so the bundle is complete:

```rust
let state = engine.raster_state(320, 240);   // now includes sky + camera_dir
execute_commands(&mut fb, &mut frame, &cmd, &state);
```

Adding an effect is now **one field** on `RasterState` rather than a new
signature threaded through three functions and their callers.

Two long parameter tails were removed along the way:
`draw_zbuffered_lightmapped_mapped` (18 args → 12) and the three private
`fill_lm_*` / `draw_scanline_lm` helpers.

**Behaviour fix:** the textured and lightmapped paths used to hardcode
`TextureMapping::PerspectiveCorrect`, silently ignoring
`engine.set_texture_mapping(...)`. Both now read `state.texture_mapping`, so the
setting takes effect. `Affine` mapping on textured meshes looks different than
before — that was the bug.

### `config::apply_default_caps`

Signature changed from `&mut K3dengine` to a sink trait so `config` no longer
depends on `engine`:

```rust
// before
pub fn apply_default_caps(engine: &mut K3dengine);

// after
pub trait CapSink { /* set_caps / clear_caps / apply_render_defaults */ }
pub fn apply_default_caps(sink: &mut impl CapSink);
```

`K3dengine` implements `CapSink`, so existing
`apply_default_caps(&mut engine)` calls are unchanged.

### `#[non_exhaustive]`

Added to public enums, so downstream `match` arms need a wildcard:
`RenderMode`, `DrawPrimitive`, `BudgetKind`, `RenderError`, `BackendFaultKind`,
`StallKind`, `RuntimeFaultKind`, `RecoveryAction`, `DisplayError`,
`PaletteMode`, `StippleMode`, `TextureMapping`, `LightLevels`.

### `#[must_use]`

Added to pure helpers (`PaletteMode::apply`, `ScreenTint::apply`,
`TextureLodConfig::*`, `PaletteSlice/Cycler::{advance, map_index}`,
`FogConfig::apply`, `DitherConfig::apply`, `DepthInterpolationMode::process_depths`,
`InterlaceField::{includes_scanline, toggle}`, `mesh_ray_cast*`, …).

### Re-exports

Glob re-exports were removed (`draw::*`, `raster::*`, `shader::*`,
`skeleton::*`, and the cross-layer ones). Each module now exposes a curated,
explicit surface; reach anything else through its own submodule:

```rust
// before
embedded_3dgfx::raster::Bresenham
// after (Bresenham is still re-exported by the raster module)
embedded_3dgfx::pipeline::rasterize::raster::Bresenham
// the same module's scanline internals are only reachable via their submodule
embedded_3dgfx::pipeline::rasterize::raster::scanline::EdgeStepper
```

### `row_width_*` feature mixes

Previously an ambiguous mix such as `row_width_160` + `row_width_96` failed to
compile. Now the widest enabled width wins deterministically.

### Prelude

`embedded_3dgfx::prelude::*` is now the one import a frame loop needs. It
exports:

- the driver and errors: `K3dengine`, `RenderError`, `BudgetKind`, `DisplayError`
- buffers and depth: `CommandBuffer`, `FrameCtx`, `ZDepth`, `Z_MAX_VALUE`
- geometry and primitives: `Geometry`, `K3dMesh`, `RenderMode`, `Camera`,
  `DrawPrimitive`
- per-pass configuration: `RasterState`, `draw_zbuffered_with_state`,
  `FogConfig`, `DitherConfig`, `apply_default_caps`
- stage markers: `Stage`, `StageKind`
- feature-gated: `PointLight` (`lighting`), `Texture` / `TextureManager`
  (`textured`)

If you were importing these from their old flat paths, replace the block with
`use embedded_3dgfx::prelude::*;` and delete the rest. Types outside this list
are still reached by their real paths, so the import stays honest about which
subsystem you are using.

### Display backends

The output stage now gathers the panel-driver vocabulary, so a backend
implementation needs one import instead of four:

```rust
use embedded_3dgfx::pipeline::output::{
    DMACapableFrameBufferBackend, DisplayBackend, DisplayRegion, DmaTransfer, FrameBuf,
    StandardSwapChain, TransferError,
};
```

`FrameBuf` and `DMACapableFrameBufferBackend` come from `embedded-graphics-framebuf`
and are re-exported here because the backend traits name them in their
signatures — you no longer need that crate in your own `Cargo.toml`.
`CompletionSlot`, `WaitTransfer`, `AsyncDmaTransfer`, `TripleSwapChain` (feature
`triple-buffering`) and `SimulatorBackend` are re-exported there too.

### `Geometry` construction (new)

`Geometry` previously had to be built with all nine fields spelled out. It now
has a constructor and builders, so the common case is one line:

```rust
// before
let geometry = Geometry {
    vertices: &vertices, faces: &faces, colors: &[], lines: &[],
    normals: &[], vertex_normals: &[], uvs: &[], texture_id: None,
};

// after
let geometry = Geometry::new(&vertices, &faces);
let geometry = Geometry::new(&vertices, &faces)
    .with_normals(&normals)
    .with_uvs(&uvs)
    .with_texture(id);
```

`..Default::default()` also works, since the optional fields already default to
empty slices. The field-explicit form still compiles unchanged.


---

## Codemod

There is no built-in `cargo fix` for path renames. A best-effort rewrite:

```python
# migrate.py — run from your project root
import pathlib, re, sys

MOVES = [
    ("draw::effects",    "pipeline::effects"),
    ("bsp::coverage",    "pipeline::rasterize::coverage"),
    ("camera_controller","pipeline::vertex::camera_controller"),
    ("camera",           "pipeline::vertex::camera"),
    ("view_frustum",     "pipeline::vertex::view_frustum"),
    ("render_layers",    "pipeline::vertex::render_layers"),
    ("shapes",           "pipeline::vertex::shapes"),
    ("bounds",           "pipeline::vertex::bounds"),
    ("lod",              "pipeline::vertex::lod"),
    ("mesh",             "pipeline::vertex::mesh"),
    ("primitive",        "pipeline::assemble::primitive"),
    ("raster",           "pipeline::rasterize::raster"),
    ("texture",          "pipeline::rasterize::texture"),
    ("tilebin",          "pipeline::rasterize::tilebin"),
    ("draw",             "pipeline::rasterize::draw"),
    ("shader",           "pipeline::shade::shader"),
    ("retro",            "pipeline::shade::retro"),
    ("lights",           "pipeline::shade::lights"),
    ("dither",           "pipeline::shade::dither"),
    ("display_backend",  "pipeline::output::display_backend"),
    ("swapchain",        "pipeline::output::swapchain"),
    ("completion",       "pipeline::output::completion"),
    ("hud",              "pipeline::output::hud"),
    ("command_buffer",   "pipeline::command_buffer"),
    ("renderer",         "pipeline::renderer"),
]

files = [p for p in pathlib.Path(".").rglob("*.rs")]
for f in files:
    t = f.read_text()
    nt = t
    for old, new in MOVES:                      # order matters: longest first
        nt = re.sub(r"embedded_3dgfx::" + re.escape(old) + r"\b",
                    "embedded_3dgfx::" + new, nt)
    if nt != t:
        f.write_text(nt)
        print("rewrote", f)
```

Then fix the stragglers by hand:

1. `draw::FogConfig` / `draw::DitherConfig` / `draw::*Config` — see
   [Types that moved](#types-that-moved-between-modules).
2. Brace-group imports, e.g.
   `use embedded_3dgfx::{engine::K3dengine, mesh::{Geometry}};` — the codemod
   only rewrites fully-qualified paths.
3. `DisplayError` and `ReadPixel`/`PixelRead` imports.
4. Add wildcard `_` arms where `match` is now non-exhaustive.

The compiler is the authority: `cargo check` will list every remaining path.

---

## Verification checklist

After migrating, these should all pass:

```bash
cargo check --all-features --all-targets
cargo test --lib
cargo test --features lighting,textured
cargo clippy --lib --all-features
# embedded targets
cargo check --no-default-features --features row_width_240
cargo build --target thumbv7em-none-eabihf
```

To find leftovers:

```bash
rg 'embedded_3dgfx::(mesh|draw|raster|shader|retro|primitive|camera|texture|renderer|command_buffer|lights|hud|swapchain|display_backend|completion|tilebin|bounds|shapes|lod|view_frustum|render_layers|camera_controller|dither)\b'
```
