# Embedded-gfx
<a href="https://crates.io/crates/embedded-gfx"><img alt="crates.io" src="https://img.shields.io/crates/v/embedded-gfx"></a>
<a href="https://github.com/Kezii/embedded-gfx/actions"><img alt="actions" src="https://github.com/Kezii/embedded-gfx/actions/workflows/rust.yml/badge.svg"></a>

This is an opengl-like library to draw 3D graphics in an embedded system, built around embedded-graphics.

## Features

- [x] full mvp pipeline with perspective projection
- [x] point cloud rendering
- [x] wireframe rendering
- [x] solid color triangle rendering
- [x] simple per-triangle lighting
- [x] mesh transformation
- [x] mesh loading from stl files

## Todo
- [ ] z-buffer
- [ ] per-fragment interpolation
- [ ] proper pipeline for vertex / fragment shading
- [ ] texture mapping ?

## Examples

Interactive visual examples are available in the `examples/` directory. These demos use the `embedded-graphics-simulator` to run on desktop:

```bash
# Run examples (requires SDL2)
cargo run --example basic_rendering --features std
cargo run --example rotating_cube --features std
cargo run --example scene_viewer --features std
cargo run --example lighting_demo --features std
```

### Available Examples

- **basic_rendering** - Cycle through render modes (Points, Lines, Solid)
- **rotating_cube** - Animated 3D rotation with FPS counter
- **scene_viewer** - Interactive scene with camera controls
- **lighting_demo** - Directional lighting demonstration

See [examples/README.md](examples/README.md) for detailed documentation.

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
- Full rendering pipeline integration tests

## Production Example

You can find a working embedded example in the *Rust on M5Stack Cardputer* project:

https://github.com/Kezii/Rust-M5Stack-Cardputer

https://github.com/Kezii/Rust-M5Stack-Cardputer/assets/3357750/658bd303-03c5-4dc2-9c49-75a98b667719
