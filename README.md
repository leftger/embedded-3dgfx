# embedded-3dgfx

<p align="center">
  <img src="assets/aztec_rustacean.png" alt="embedded-3dgfx" width="100%">
</p>

[![crates.io](https://img.shields.io/crates/v/embedded-3dgfx.svg)](https://crates.io/crates/embedded-3dgfx)
[![docs.rs](https://img.shields.io/docsrs/embedded-3dgfx)](https://docs.rs/embedded-3dgfx)
[![CI](https://github.com/leftger/embedded-3dgfx/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/leftger/embedded-3dgfx/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/leftger/embedded-3dgfx/branch/master/graph/badge.svg)](https://codecov.io/gh/leftger/embedded-3dgfx)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](LICENSE-MIT)

A `no_std` 3D graphics and physics engine for embedded systems: software rasterization, rigid/soft-body physics, skeletal animation, and effects tuned for MCUs.

> Fork of [embedded-gfx](https://github.com/Kezii/embedded-gfx) by [Kezii](https://github.com/Kezii), extended with textures, fog/dithering, DMA swapchains, AA, physics, BSP, and more.

## Highlights

- **A graphics pipeline, spelled out** — five stages (`pipeline::vertex` → `assemble` → `rasterize` → `shade` → `output`), each with a `StageKind` marker and a documented dependency direction, plus a [`prelude`](https://docs.rs/embedded-3dgfx/latest/embedded_3dgfx/prelude/index.html) for the common types
- **Two raster layers, one config** — the built-in `draw` path (hand-specialised per `RenderMode`, driven by `record`/`execute`) shares its `FogConfig`/`DitherConfig` with a zero-cost `FragmentShader` seam for custom materials and decorators (`FogShader`, `DitherShader`, `ScreenTintShader`, `PaletteShader`, `WaterReflectShader`)
- **Unified raster state** — `pipeline::rasterize::draw::RasterState` bundles per-pass config (fog, dither, tint, palette, stipple, depth bias), so rasterizers take one borrowed context instead of a wide positional argument tail
- **One home per type** — no glob re-exports; every type is reached through its owning module, so the public surface is explicit rather than accidental
- **Record / execute** — traverse once, rasterize from a fixed-capacity command buffer (`PrimitiveHeader` + packed typed descriptors for low RAM footprint)
- **Rendering** — MVP + frustum/backface cull, Z-buffer, flat/Gouraud/Blinn-Phong, perspective textures, Bayer dither, Reinhard tonemapping, sub-pixel Q16.16 rasterization, lights, particles, LOD, HUD
- **Physics & Navigation** *(features `physics`, `scene`)* — rigid bodies, joints, soft body, ray primitives, NavMesh A* pathfinding
- **Animation** — skeletal LBS, vertex morphs, ABSM state machines, transform tracks, spline curves / tweens
- **Embedded-friendly** — `heapless` caps, Cortex-M SWAR/DSP SIMD optimizations, async-agnostic swapchain present, silicon hardware offloading hooks (`HardwareAccelerator`)


## Screenshots

<table>
  <tr>
    <td align="center"><img src="assets/gif_water_ssr.gif" alt="SSR Water Reflection" width="320"><br><em>SSR water reflection + palette cycling</em></td>
    <td align="center"><img src="assets/gif_suzanne.gif" alt="Blinn-Phong Suzanne" width="320"><br><em>Blinn-Phong</em></td>
  </tr>
  <tr>
    <td align="center"><img src="assets/gif_physics.gif" alt="Physics balls" width="320"><br><em>Rigid body physics</em></td>
    <td align="center"><img src="assets/gif_particles.gif" alt="Particles + fog" width="320"><br><em>Particles + fog</em></td>
  </tr>
  <tr>
    <td align="center"><img src="assets/gif_cloth.gif" alt="Cloth" width="320"><br><em>Soft-body cloth</em></td>
    <td align="center"><img src="assets/gif_point_lights.gif" alt="Point lights" width="320"><br><em>Point lights</em></td>
  </tr>
</table>


```bash
cargo run --release --example screenshots --features "std,lighting,textured,raycast,scene,physics"
```

## Installation

```toml
[dependencies]
# Embedded (no_std) — slim default is row_width_240
embedded-3dgfx = { version = "0.7", default-features = false, features = ["row_width_320", "depth-u16"] }

# Orientation-style lit meshes
embedded-3dgfx = { version = "0.7", default-features = false, features = ["row_width_320", "depth-u16", "lighting"] }

# Desktop / simulator
embedded-3dgfx = { version = "0.7", features = ["std", "physics"] }
```

### MCU feature recipes

| Recipe | Features |
| :--- | :--- |
| Minimal wireframe | `default-features = false`, `row_width_320`, `depth-u16` |
| Lit mesh (Gouraud / Blinn / Toon) | add `lighting` |
| Retro / Doom-style | `lighting`, `textured`, `raycast`, `hud` |
| Physics demo | add `physics` |

## Quick start

```rust
use embedded_3dgfx::prelude::*;

let mut engine = K3dengine::new(320, 240);
engine.camera.set_position(Vector3::new(0.0, 0.0, 5.0).into());

let geometry = Geometry { vertices: &CUBE_VERTS, faces: &CUBE_FACES, /* ... */ };
let mut mesh = K3dMesh::new(geometry);
mesh.set_render_mode(RenderMode::Lines);

let mut commands = CommandBuffer::<512>::new();
engine.record(core::iter::once(&mesh), &mut commands, None).unwrap();
engine.execute(&mut display, &mut frame_ctx, &commands, None).unwrap();
```

### Custom raster draws (`RasterState`)

Everything the engine applies to a frame — fog, dither, screen tint, palette
quantization, stipple, depth bias — is lowered once into a
[`RasterState`](https://docs.rs/embedded-3dgfx/latest/embedded_3dgfx/pipeline/rasterize/draw/struct.RasterState.html):

```rust
use embedded_3dgfx::pipeline::rasterize::draw::draw_zbuffered_with_state;

let state = engine.raster_state(320, 240);
draw_zbuffered_with_state(primitive, &mut fb, &mut zbuffer, &state);
```

## Architecture: a five-stage pipeline

The crate is organised as a graphics pipeline. Each stage owns one module and
consumes the previous stage's output:

| # | Stage | Module | Input → output |
|---|-------|--------|----------------|
| 1 | `Stage::Vertex`    | `pipeline::vertex`    | model-space mesh → clip-space vertices |
| 2 | `Stage::Assemble`  | `pipeline::assemble`  | clip-space vertices → screen-space `DrawPrimitive` |
| 3 | `Stage::Rasterize` | `pipeline::rasterize` | screen primitives → covered pixels |
| 4 | `Stage::Shade`     | `pipeline::shade`     | fragments → `Rgb565` colours |
| 5 | `Stage::Output`    | `pipeline::output`    | framebuffer → presented frame |

`pipeline::command_buffer` is the transport between the record half (stages
1–2) and the execute half (stages 3–5); `pipeline::renderer` drives the execute
half, and `engine` is the frame driver on top. Shared per-pass configuration
lives in `pipeline::effects`.

The dependency direction is enforced by review, not the compiler: a stage may
depend on lower-numbered stages, `effects`, and the crate-root core types —
never on a higher-numbered stage. The single exception is that `rasterize`
invokes the fragment programs defined in `shade`, so `shade` sits *below*
`rasterize` in dependency order even though it runs afterwards.

Every stage has a `StageKind` marker so the pipeline is queryable at compile
time (`Camera: Stage::Vertex`, `DrawPrimitive: Stage::Assemble`,
`RasterState: Stage::Rasterize`, `FlatColorShader: Stage::Shade`,
`DisplayError: Stage::Output`).

### Module map

| Tree | Contents |
|------|----------|
| `pipeline::vertex` | `mesh`, `shapes`, `bounds`, `camera`, `camera_controller`, `view_frustum`, `lod`, `transform` |
| `pipeline::assemble` | `primitive` (`DrawPrimitive`) |
| `pipeline::rasterize` | `raster`, `draw`, `coverage`, `texture`, `tilebin` |
| `pipeline::shade` | `shader`, `retro`, `dither`, `lights` |
| `pipeline::output` | `display_backend`, `swapchain`, `completion`, `hud` |
| `pipeline::{command_buffer, renderer, effects}` | record/execute transport, execute driver, shared config |
| `engine` | `K3dengine` frame driver (`record` / `execute`) |
| core | `color`, `config`, `error`, `simd_dsp`, plus the `prelude` |
| subsystems | `physics`, `raycast`, `bsp`, `navmesh`, `skeleton`, `animation`, `absm`, `tween`, `scene_format`, `scene_stream`, … |

### Geometry & Surface Normals for Lighting

When using lit render modes (`RenderMode::SolidLightDir`, `BlinnPhong`, `Toon`, `GouraudLightDir`), the engine requires **surface face normals** in `Geometry.normals` (or `vertex_normals`) to evaluate light angles (`N · L`):

* **Static Flash ROM Storage (Recommended for MCUs):** Precompute face normals offline or at compile-time and store them alongside vertices as `&'static [[f32; 3]]` (0 RAM overhead).
* **On-Demand Helper:** If authoring procedural geometry in code, use `Geometry::compute_face_normals_into(&verts, &faces, &mut out_normals)` or `Geometry::compute_face_normals(&verts, &faces)`:

```rust
let mut normals = [[0.0f32; 3]; CUBE_FACES.len()];
Geometry::compute_face_normals_into(&CUBE_VERTS, &CUBE_FACES, &mut normals);

let geometry = Geometry {
    vertices: &CUBE_VERTS,
    faces: &CUBE_FACES,
    normals: &normals,
    ..Default::default()
};
```

More patterns (particles, lights, fog, physics, skeleton, soft body, async present) live under `examples/` and on [docs.rs](https://docs.rs/embedded-3dgfx).


## Feature flags

| Flag | Default | Description |
|------|---------|-------------|
| `row_width_*` | `240` | Row-buffer width (`96` / `160` / `240` / `320`). Meant to be mutually exclusive; if a build enables several, the widest wins deterministically |
| `std` | off | Desktop helpers / `perfcounter` |
| `lighting` | off | `SolidLightDir` / Gouraud / Blinn / Toon / `SectorBright` + `lights` |
| `textured` | off | Texture modes + `texture` module (implies `lighting`) |
| `raycast` | off | Doom-style raycaster, BSP helpers, `sector_lights` |
| `scene` | off | Skeleton, character, particles, billboard, animation / scene stream |
| `hud` | off | HUD helpers |
| `painters` | off | Painter's algorithm helpers (`painters` module) |
| `physics` | off | Rigid body, soft body, physics raycast |
| `aa-heuristic` / `aa-coverage` | off | Triangle edge AA (coverage needs a W×H buffer) |
| `dsp` / `fixed-transform` / `fixed-raster` | off | Shared Q16.16 / quat path via [`embedded-dsp`](https://crates.io/crates/embedded-dsp) |
| `triple-buffering` / `embassy` / `dma2d` | off | Swapchain / Embassy / DMA2D hooks |
| `perfcounter` / `dwt-profiler` / `rtt-trace` / `itm-trace` | off | Timing / trace sinks |

Flash impact of the slim recipes is tracked in [`docs/feature-size.md`](docs/feature-size.md) (`size_harness` + CI budget).

### Optional scene extras *(off by default — keeps MCU binaries lean)*

| Feature | What you get |
|---------|----------------|
| `aabb-cull` | Cached AABB, two-stage frustum cull, raycast broadphase |
| `render-layers` | Camera ↔ mesh layer bitmasks |
| `record-sort` | Priority / distance sort in `record` |
| `lod-crossfade` | LOD fade margins |
| `anim-blend` | Clip blending, bone slerp, skinned AABBs (also enables `scene`) |
| `gizmos` | AABB / frustum debug wireframes |
| `visibility-extras` | `aabb-cull` + `render-layers` + `record-sort` + `lod-crossfade` |
| `scene-extras` | All of the above |

```toml
embedded-3dgfx = { version = "0.7", features = ["std", "scene-extras"] }
```

```bash
cargo test --test scene_extras --features "std,scene-extras"
```

## Examples

```bash
cargo run --example rotating_cube --features std
cargo run --example lighting_demo --features "std,lighting"
cargo run --example texture_mapping_demo --features "std,textured"
cargo run --example skeletal_animation_demo --features "std,scene"
cargo run --example star_striker_demo --features "std,lighting,scene"
# physics demos also need: --features "std,physics"
```

Rendering: `basic_rendering`, `rotating_cube`, `scene_viewer`, `lighting_demo`, `gouraud_demo`, `blinn_phong_demo`, `fog_dithering_demo`, `texture_mapping_demo`, `mesh_texture_demo`, `retro_presets_demo`, `bsp_builder_demo`, `dma_rendering_demo`, `billboard_demo`, `lod_demo`, `vertex_animation_demo`, `painters_algorithm_demo`, `boot_menu`, `stl_viewer`, `water_reflection_ssr_demo`, `hybrid_hud_sprite_demo`, `star_striker_demo`, …

Physics: `physics_rolling_ball`, `physics_bouncing_balls`, `physics_pendulum`, `physics_newtons_cradle`, `physics_stack_tower`, `cloth_simulation`, `jelly_cube_demo`, `raycast_demo`, `walkable_demo`, `capsule_physics_demo`, …

## Docs & tools

| Resource | Topic |
|-----|-------|
| [`MIGRATION.md`](MIGRATION.md) | Upgrading from the pre-`pipeline` module layout |
| [`docs/app-integration.md`](docs/app-integration.md) | Adding the engine to your application (with runnable templates) |
| [`docs/caps-and-telemetry.md`](docs/caps-and-telemetry.md) | Caps, telemetry, CI budgets |
| [`docs/feature-size.md`](docs/feature-size.md) | Slim vs full flash (`.text`) budgets |
| [`docs/backend-integration.md`](docs/backend-integration.md) | Board bring-up, memory sizing |
| [`docs/asset-pipeline.md`](docs/asset-pipeline.md) | Offline assets / scene streaming |
| [`tools/blender_addon`](tools/blender_addon/README.md) | Blender mesh / animation export add-on |

**Typical target:** Cortex-M4F/M33 with FPU; ~128 KB RAM minimum, ~512 KB+ recommended for double-buffer + Z + physics at 240×135.

## Testing

```bash
cargo test --lib
cargo test --lib --features dma2d,depth-u16
cargo test --test scene_extras --features "std,scene-extras"
```

Git hooks (fmt on commit / push): `./scripts/install-git-hooks.sh`

## Contributing

PRs welcome — especially board backends, broad-phase spatial structures, and extra joint / collider types.

### Import policy

Aggregating surfaces — the `prelude` and the per-stage facades such as
`pipeline::output` — are **public API for downstream crates**. Code inside `src/`
must not import from them: internal modules name the module that actually
defines an item, so trimming an aggregator can never silently reshape the
internals. Glob imports are confined to `#[cfg(test)]` modules.

```bash
python3 .github/scripts/check_internal_imports.py   # run before pushing
```

The check runs as the `import-policy` CI job. `tests/`, `examples/` and
`benches/` are consumers of the public API and may use the facades freely.

### Feature gate policy

A module reached only through `#[cfg(feature = "F")] mod x;` cannot be compiled
without `F`, so a `#[cfg(feature = "F")]` inside it does nothing. Those gates
read as if they mattered and hide which ones are load-bearing, so they are
rejected:

```bash
python3 .github/scripts/check_feature_gates.py   # run before pushing
```

The check runs as the `feature-gate-policy` CI job. It only reports gates that
are *provably* redundant — `any(...)` and `not(...)` establish nothing, and a
gate that still constrains a second feature is kept.

## License

Dual-licensed under **MIT OR Apache-2.0**. See [`LICENSE-MIT`](./LICENSE-MIT), [`LICENSE-APACHE`](./LICENSE-APACHE), and [`NOTICE`](./NOTICE).
