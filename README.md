# Embedded-3dgfx
<a href="https://crates.io/crates/embedded-3dgfx"><img alt="crates.io" src="https://img.shields.io/crates/v/embedded-3dgfx"></a>
<a href="https://github.com/leftger/embedded-3dgfx/actions"><img alt="actions" src="https://github.com/leftger/embedded-3dgfx/actions/workflows/rust.yml/badge.svg"></a>

A no_std 3D graphics engine for embedded systems, built around embedded-graphics. Features a complete rendering pipeline with Z-buffering, texture mapping, lighting, and advanced visual effects optimized for resource-constrained devices.

> **Note**: This is a fork of [embedded-gfx](https://github.com/Kezii/embedded-gfx) by [Kezii](https://github.com/Kezii). This fork adds texture mapping, fog/dithering effects, DMA rendering, and Z-buffer improvements.

## Features

### Core Rendering
- **Full MVP Pipeline** - Model-View-Projection transformations with perspective projection
- **Z-buffering** - Depth testing for correct occlusion with configurable epsilon
- **Backface Culling** - Skips ~50% of faces pointing away from camera
- **Frustum Culling** - Automatic clipping of off-screen objects (50-200% speedup)
- **Fixed-point Rasterization** - Integer-only triangle filling optimized for embedded systems
- **Integer Z-buffer** - 16.16 fixed-point depth values for memory efficiency

### Rendering Modes
- **Point Cloud Rendering** - 3D point visualization
- **Wireframe Rendering** - Line-based mesh visualization
- **Flat Shading** - Per-face solid colors
- **Gouraud Shading** - Smooth vertex color interpolation across triangles
- **Painter's Algorithm** - Back-to-front sorting without Z-buffer (saves 1.92MB RAM)

### Lighting & Materials
- **Directional Lighting** - Diffuse lighting with ambient term
- **Blinn-Phong Shading** - Specular highlights with half-vector approximation
- **Pre-computed Lighting** - Per-mesh constants for performance

### Texture Mapping
- **Affine Texture Mapping** - UV-mapped textures with nearest-neighbor sampling
- **TextureManager** - Stores multiple textures using heapless::Vec
- **Power-of-2 Textures** - Fast wrapping using bit masks (no divisions)
- **Static Texture Data** - RGB565 format for embedded efficiency

### Visual Effects
- **Fog Effects** - Depth-based color blending with configurable near/far planes
- **Ordered Dithering** - 4x4 Bayer matrix for retro visual effects
- **Billboard System** - Camera-facing quads for particles and sprites
- **Vertex Animation** - Keyframe interpolation for animated models

### Performance Optimizations
- **DMA Rendering** - Double-buffered rendering with platform-agnostic backend trait (~30% FPS boost)
- **LOD System** - Distance-based mesh detail switching
- **Inline Optimization** - Hot functions marked for aggressive inlining
- **Swap Chain** - Asynchronous framebuffer transfers while rendering

### Hardware Abstraction
- **DisplayBackend Trait** - Platform-agnostic DMA interface
- **SimulatorBackend** - No-op implementation for desktop testing
- **Mesh Loading** - Import geometry from STL files

## Recent Additions (2026)

### Fog & Dithering Effects
Scanline-based effects with zero overhead when disabled:
```rust
let fog = FogConfig::new(Rgb565::CSS_GRAY, 5.0, 15.0);
let dither = DitherConfig { intensity: 8 };
draw_zbuffered_with_effects(prims, fb, zbuf, Some(&fog), Some(&dither));
```

### Affine Texture Mapping
Complete texture system for embedded devices:
```rust
static TEXTURE_DATA: [Rgb565; 64] = [...];
let texture = Texture::new(&TEXTURE_DATA, 8, 8);
let mut tex_mgr = TextureManager::<16>::new();
let tex_id = tex_mgr.add_texture(texture).unwrap();

Geometry {
    vertices: &VERTS,
    uvs: &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
    texture_id: Some(tex_id),
    // ...
}
```

### DMA Rendering (Double Buffering)
Hardware abstraction for asynchronous framebuffer transfers (~30% FPS improvement):
```rust
let mut swap_chain = SwapChain::new(
    fb0.as_mut_ptr(),
    fb1.as_mut_ptr(),
    false,
    SimulatorBackend::new(),
);

loop {
    let back_buffer = swap_chain.get_back_buffer();
    render_to(back_buffer);
    swap_chain.present().unwrap();
}
```

### Z-Fighting Improvements
Enhanced depth buffer precision control:
```rust
// Tune camera planes for your scene depth range
camera.set_near_far(1.0, 15.0);  // Ratio: 15:1 (excellent)
println!("Ratio: {:.1}:1", camera.get_near_far_ratio());
```
See `ZBUFFER_TUNING.md` for comprehensive tuning guide.

## Examples

Interactive visual examples are available in the `examples/` directory. These demos use the `embedded-graphics-simulator` to run on desktop:

```bash
# Run examples (requires SDL2)
cargo run --example basic_rendering --features std
cargo run --example rotating_cube --features std
cargo run --example scene_viewer --features std
cargo run --example lighting_demo --features std
cargo run --example gouraud_demo --features std
cargo run --example fog_dithering_demo --features std
cargo run --example texture_mapping_demo --features std
cargo run --example dma_rendering_demo --features std
```

### Available Examples

**Core Rendering:**
- **basic_rendering** - Cycle through render modes (Points, Lines, Solid)
- **rotating_cube** - Animated 3D rotation with FPS counter
- **scene_viewer** - Interactive scene with camera controls

**Lighting & Shading:**
- **lighting_demo** - Directional lighting demonstration
- **gouraud_demo** - Smooth vertex color interpolation
- **painters_algorithm_demo** - Back-to-front rendering without Z-buffer

**Advanced Features:**
- **fog_dithering_demo** - Interactive fog and dithering effects
- **texture_mapping_demo** - UV-mapped textures with multiple patterns
- **dma_rendering_demo** - Double-buffered rendering performance comparison
- **billboard_demo** - Camera-facing sprites and particles
- **lod_demo** - Distance-based level of detail switching
- **vertex_animation_demo** - Keyframe-based model animation

See [examples/README.md](examples/README.md) for detailed documentation.

## Recommendations for Embedded Systems

### ARM Cortex-M33 with FPU (e.g., STM32WBA65RI)

**Immediate Use (Production-Ready):**
- ✓ DMA rendering - ~30% FPS improvement, essential for production
- ✓ Fog & dithering - Professional look with < 10% overhead
- ✓ Affine texture mapping - Classic 3D look, manageable performance cost
- ✓ Z-buffer tuning - Adjust camera near/far planes for your scene depth

**Performance Budget Example:**
For 60 FPS (16.67ms per frame) at 240×135:
- Vertex transform: 2ms (1000 vertices)
- Rasterization: 10ms (500 triangles)
- Clear/setup: 1ms
- Display transfer: 3ms (with DMA, parallel)
- **Total: ~13ms → 75 FPS achievable**

With optimizations:
- LOD: 3-10x fewer triangles at distance
- Frustum culling: 50-90% fewer objects
- DMA: 30% better throughput
- **Result: Complex scenes at 60 FPS**

### Memory Considerations

- **Single-buffered**: 1× framebuffer (e.g., 320×240×2 = 150KB)
- **Double-buffered**: 2× framebuffer (300KB) - recommended for DMA
- **Z-buffer**: 1× depth buffer (e.g., 320×240×4 = 300KB for u32)
- **Painter's Algorithm**: No Z-buffer needed (saves 300KB for 320×240 scenes)

## Future Enhancements

**Potential additions for the library:**
- Perspective-correct texture mapping (upgrade from affine)
- Lightmapping (baked lighting for static scenes)
- Portal rendering (indoor scene optimization)
- Hardware-specific display backends (ESP32, STM32, RP2040)
- BSP trees (complex static scenes)
- Tile-based deferred rendering

## Testing

The library includes a comprehensive test suite covering all major components:

```bash
# Run all tests
cargo test

# Run with verbose output
cargo test -- --nocapture
```

Test coverage includes:
- Camera transformations and projections
- Mesh geometry validation
- Coordinate transformation and clipping
- Performance counter timing
- Drawing primitives (points, lines, triangles)
- Texture sampling and management
- Display backend and swap chain
- Full rendering pipeline integration tests

## Production Example

You can find a working embedded example in the *Rust on M5Stack Cardputer* project:

https://github.com/Kezii/Rust-M5Stack-Cardputer

https://github.com/Kezii/Rust-M5Stack-Cardputer/assets/3357750/658bd303-03c5-4dc2-9c49-75a98b667719

## Contributing

Contributions are welcome! Areas of interest:
- Hardware-specific display backends (ESP32, STM32, RP2040)
- Perspective-correct texture mapping
- Lightmapping support
- Portal rendering for indoor scenes
- Additional visual effects

## References

- [Tricks of the 3D Game Programming Gurus](https://www.amazon.com/Tricks-Game-Programming-Gurus-Advanced/dp/0672318350) - Classic techniques
- [Michael Abrash's Graphics Programming Black Book](https://github.com/jagregory/abrash-black-book) - Optimization techniques
- [PSX Graphics Programming](https://psx-spx.consoledev.net/graphicsprocessingunitgpu/) - Affine texture mapping
- [Fabien Sanglard's Game Engine Black Books](https://fabiensanglard.net/gebb/) - Doom, Quake architecture

## License

See LICENSE file for details.
