# embedded-3dgfx Examples

This directory contains interactive visual examples demonstrating the capabilities of the embedded-3dgfx 3D graphics engine.

## Prerequisites

These examples use the `embedded-graphics-simulator` to provide a desktop window for visualization. SDL2 is required:

### macOS
```bash
brew install sdl2
```

### Ubuntu/Debian
```bash
sudo apt-get install libsdl2-dev
```

### Windows
Download SDL2 development libraries from [libsdl.org](https://www.libsdl.org/download-2.0.php)

## Running the Examples

All examples require the `std` feature to be enabled:

```bash
# Run a specific example
cargo run --example basic_rendering --features std

# Or using the shorthand
cargo run --example rotating_cube --features std
```

### Window Sizes

Examples use the following display resolutions:
- `basic_rendering`, `rotating_cube`: 640×480 pixels
- `scene_viewer`, `lighting_demo`: 800×600 pixels

To adjust the window size, modify the `SimulatorDisplay::new()` size and/or the `scale()` parameter in the example's source code.

## Available Examples

### 1. basic_rendering
**File:** `basic_rendering.rs`

Demonstrates the three fundamental render modes:
- **Points mode**: Renders mesh vertices as individual points
- **Lines mode**: Renders mesh edges as wireframe
- **Solid mode**: Renders filled triangles

**Controls:**
- `SPACE` - Cycle through render modes
- `ESC` - Exit

```bash
cargo run --example basic_rendering --features std
```

### 2. rotating_cube
**File:** `rotating_cube.rs`

Shows continuous animation with a rotating cube. Demonstrates:
- Time-based transformations
- Real-time FPS counter using PerformanceCounter
- Smooth 3D rotation on multiple axes

**Controls:**
- `ESC` - Exit

```bash
cargo run --example rotating_cube --features std
```

### 3. scene_viewer
**File:** `scene_viewer.rs`

Interactive 3D scene with multiple objects and camera controls. Features:
- Multiple meshes with different render modes
- Ground plane with point grid
- Interactive camera rotation and zoom
- Animated object transformations

**Controls:**
- `Arrow Keys` - Rotate camera around the scene
- `Up/Down` - Adjust camera height
- `+/-` - Zoom in/out
- `ESC` - Exit

```bash
cargo run --example scene_viewer --features std
```

### 4. lighting_demo
**File:** `lighting_demo.rs`

Demonstrates directional lighting on 3D meshes. Features:
- Icosphere (sphere-like shape) and cube with per-face normals
- Dynamic directional light with visible changes as light moves
- Objects rotate to show lighting from different angles
- Visual light direction indicator in top-right corner
- Toggle between auto-rotation and manual control

**Controls:**
- `SPACE` - Toggle auto-rotation of light
- `Arrow Keys` - Manually adjust light direction
- `ESC` - Exit

```bash
cargo run --example lighting_demo --features std
```

**Note:** This demo now uses per-face normals (one normal per triangle) which correctly shows how light direction affects surface brightness. Watch how the brightness of different faces changes as the light rotates around the objects!

## Implementation Notes

### Performance Counter

All examples use the `PerformanceCounter` utility to measure and display FPS. In example/test mode, it uses `std::time`, while in embedded mode (without the `std` feature), it uses `embassy-time`.

### Render Modes

The engine supports several render modes:
- `RenderMode::Points` - Draw vertices as points
- `RenderMode::Lines` - Draw edges as lines (wireframe)
- `RenderMode::Solid` - Draw filled triangles
- `RenderMode::SolidLightDir(direction)` - Draw filled triangles with directional lighting

### Mesh Creation

Examples demonstrate creating meshes from scratch using the `Geometry` struct:

```rust
let geometry = Geometry {
    vertices: &vertices,      // Array of [f32; 3] positions
    faces: &faces,            // Array of [usize; 3] triangle indices
    colors: &[],              // Optional per-vertex colors
    lines: &[],               // Optional explicit edge pairs
    normals: &normals,        // Optional per-vertex normals for lighting
};

let mut mesh = K3dMesh::new(geometry);
```

### Camera Control

The engine provides a flexible camera system:

```rust
let mut engine = K3dengine::new(width, height);
engine.camera.set_position(Point3::new(0.0, 2.0, 5.0));
engine.camera.set_target(Point3::new(0.0, 0.0, 0.0));
engine.camera.set_fovy(PI / 4.0); // 45-degree field of view
```

## Building for Embedded Targets

When building for embedded targets, omit the `std` feature and ensure you have an appropriate `embassy-time` driver configured:

```bash
cargo build --target thumbv7em-none-eabihf
```

The examples will not compile for embedded targets as they depend on SDL2, but the core library works in no_std environments.
