# 3D Rendering Techniques for Embedded Devices

This document outlines various 3D rendering techniques suitable for embedded devices like the STM32WBA65RI (ARM Cortex-M33 with FPU).

## Already Implemented ✓

- **Z-buffering** - Depth testing for correct occlusion
- **Backface culling** - Skips ~50% of faces pointing away from camera
- **Frustum culling** - Skips off-screen objects (50-200% speedup)
- **Flat shading** - Per-face colors
- **Directional lighting** - Simple diffuse lighting with ambient term
- **Integer Z-buffer** - Fixed-point depth values (16.16 format)
- **Inline optimization** - Hot functions marked for aggressive inlining
- **Fixed-point rasterization** - Integer-only triangle filling
- **Pre-computed lighting constants** - Per-mesh instead of per-face
- **Blinn-Phong shading** - Specular highlights with half-vector approximation
- **Billboard system** - Camera-facing quads for particles and sprites
- **LOD system** - Distance-based mesh detail switching
- **Gouraud shading** - Smooth vertex color interpolation across triangles

## High-Impact Techniques to Consider

### 1. Gouraud Shading (Vertex Color Interpolation)

**Description**: Interpolate vertex colors across triangle faces for smooth shading.

**Implementation**:
```rust
// Store per-vertex colors
let vertex_colors = [...];

// During scanline rasterization, interpolate RGB values
let color = interpolate_color(v1_color, v2_color, v3_color, barycentric);
```

**Performance**:
- Cost: Low (just linear interpolation in scanline loop)
- Benefit: Smooth color gradients, much better visual quality
- Memory: Per-vertex color storage

**Recommended**: ⭐⭐⭐⭐⭐ Easy to add, huge visual impact

---

### 2. Affine Texture Mapping

**Description**: Fast texture mapping without perspective correction (classic PS1/N64 look).

**Implementation**:
```rust
// Per-triangle UV coordinates
struct TexturedTriangle {
    vertices: [Point3<f32>; 3],
    uvs: [[f32; 2]; 3],  // Texture coordinates
    texture: &Texture,
}

// Linear UV interpolation (cheaper than perspective-correct)
let uv = interpolate_uv(uv1, uv2, uv3, barycentric);
let color = texture.sample(uv);
```

**Performance**:
- Cost: Medium (adds UV interpolation + texture lookups)
- Benefit: Huge visual improvement
- Memory: Texture storage (consider 64x64 or 128x128 textures)

**Trade-offs**: Works well at close range, has visible warping at steep angles

**Recommended**: ⭐⭐⭐⭐ Classic 3D look, manageable cost

---

### 3. Binary Space Partitioning (BSP) Trees

**Description**: Pre-computed spatial data structure for efficient rendering.

**Features**:
- Eliminates need for Z-buffer sorting in some cases
- Enables perfect back-to-front rendering
- Used in Doom, Quake engines

**Implementation**:
```rust
struct BSPNode {
    plane: Plane,
    front: Box<BSPNode>,
    back: Box<BSPNode>,
    polygons: Vec<Triangle>,
}

// Traverse tree from camera position
fn render_bsp(node: &BSPNode, camera_pos: Point3<f32>) {
    if camera in front of node.plane {
        render_bsp(node.back);
        draw(node.polygons);
        render_bsp(node.front);
    } else {
        render_bsp(node.front);
        draw(node.polygons);
        render_bsp(node.back);
    }
}
```

**Performance**:
- Cost: Preprocessing time + memory for BSP tree
- Benefit: Can be faster than Z-buffering for complex static scenes
- Best for: Indoor scenes with many walls

**Recommended**: ⭐⭐⭐ Advanced technique, good for specific use cases

---

### 4. Painter's Algorithm (Back-to-Front)

**Description**: Sort triangles by depth and draw furthest first.

**Implementation**:
```rust
// Sort faces by average Z depth
faces.sort_by(|a, b| {
    let z_a = (a.v1.z + a.v2.z + a.v3.z) / 3.0;
    let z_b = (b.v1.z + b.v2.z + b.v3.z) / 3.0;
    z_b.partial_cmp(&z_a).unwrap()
});

// Draw back-to-front (no Z-buffer needed!)
for face in faces {
    draw_triangle(face);
}
```

**Performance**:
- Cost: Sorting overhead per frame (O(n log n))
- Benefit: Eliminates Z-buffer memory (saves 1.92MB at 800x600!)
- Drawback: Doesn't handle overlapping triangles perfectly

**Recommended**: ⭐⭐⭐ Good for low-RAM systems

---

### 5. Billboarding (Sprite-based 3D)

**Description**: 2D sprites that always face the camera.

**Use Cases**:
- Particles (explosions, smoke, sparks)
- Distant objects (trees, rocks)
- Vegetation
- UI elements in 3D space

**Implementation**:
```rust
// Calculate billboard orientation
let to_camera = (camera_pos - billboard_pos).normalize();
let right = camera_up.cross(&to_camera);
let up = to_camera.cross(&right);

// Create quad facing camera
let quad = [
    billboard_pos - right * size + up * size,
    billboard_pos + right * size + up * size,
    billboard_pos + right * size - up * size,
    billboard_pos - right * size - up * size,
];
```

**Performance**:
- Cost: Very low (just 2 triangles per billboard)
- Benefit: Huge performance win vs 3D models
- Memory: Small textures (16x16 to 64x64)

**Recommended**: ⭐⭐⭐⭐⭐ Essential for particles and effects

---

### 6. Level of Detail (LOD)

**Description**: Multiple mesh resolutions per object, switch based on distance.

**Implementation**:
```rust
struct LODMesh {
    high: Geometry,    // 2000 triangles
    medium: Geometry,  // 500 triangles
    low: Geometry,     // 100 triangles
}

fn select_lod(distance: f32) -> &Geometry {
    match distance {
        d if d > 100.0 => &self.low,
        d if d > 50.0  => &self.medium,
        _              => &self.high,
    }
}
```

**Performance**:
- Cost: Extra mesh storage (3x memory per model)
- Benefit: 3-10x performance improvement
- Best for: Large scenes with many objects

**Recommended**: ⭐⭐⭐⭐⭐ Major performance boost

---

### 7. Lightmapping (Baked Lighting)

**Description**: Pre-computed lighting stored in textures.

**Features**:
- Static lighting calculated offline
- Just apply as texture at runtime
- Supports complex lighting (shadows, ambient occlusion, GI)

**Implementation**:
```rust
// Offline: Bake lighting to texture
// Runtime: Just sample lightmap texture
let light_value = lightmap.sample(uv);
let final_color = base_color * light_value;
```

**Performance**:
- Cost: Texture memory (one lightmap per mesh/scene)
- Benefit: Beautiful lighting with almost no runtime cost
- Limitation: Only for static geometry

**Recommended**: ⭐⭐⭐⭐ Excellent for static scenes

---

### 8. Portal Rendering

**Description**: Only render what's visible through doorways/portals.

**Use Cases**:
- Indoor scenes with rooms
- Games like Duke Nukem 3D, Portal

**Implementation**:
```rust
fn render_room(room: &Room, portal_bounds: Rect) {
    // Draw room geometry clipped to portal
    draw_room_geometry(room, portal_bounds);

    // Recursively render visible portals
    for portal in room.portals {
        if portal.visible_in(portal_bounds) {
            let new_bounds = portal.project_bounds();
            render_room(portal.connected_room, new_bounds);
        }
    }
}
```

**Performance**:
- Cost: Scene setup complexity
- Benefit: Massive culling for indoor scenes (render only 1-2 rooms)
- Best for: Room-based level design

**Recommended**: ⭐⭐⭐⭐ Excellent for indoor games

---

### 9. Vertex Animation (Morph Targets)

**Description**: Store keyframe vertices, interpolate between them.

**Implementation**:
```rust
struct MorphTarget {
    keyframes: Vec<Vec<[f32; 3]>>,  // Multiple vertex positions
    times: Vec<f32>,
}

// Interpolate between keyframes
let vertices = lerp(keyframe1, keyframe2, t);
```

**Performance**:
- Cost: Memory for keyframes (N frames × vertex count)
- Benefit: Cheaper than skeletal animation (no bone math)
- Best for: Facial animation, simple character animation

**Recommended**: ⭐⭐⭐ Good for simple animations

---

### 10. Mode 7-Style Effects (Affine Transformations)

**Description**: Ground plane rendering with affine transforms (Super Mario Kart, F-Zero style).

**Implementation**:
```rust
// Render ground plane scanline by scanline
for y in 0..screen_height {
    let world_y = calculate_world_y(y, camera_height);
    let scale = camera_height / world_y;

    for x in 0..screen_width {
        let world_x = (x - screen_center) * scale;
        let tex_coord = transform(world_x, world_y);
        draw_pixel(x, y, sample_texture(tex_coord));
    }
}
```

**Performance**:
- Cost: Very low (simple per-pixel math)
- Benefit: Great for racing games, flight sims
- Enables: Rotation, scaling, perspective ground

**Recommended**: ⭐⭐⭐⭐ Perfect for specific game genres

---

## Embedded-Specific Optimizations

### 11. Fixed-Point Everything

**Description**: Convert all vertex math to fixed-point arithmetic.

```rust
// 16.16 fixed-point
type Fixed = i32;

fn to_fixed(f: f32) -> Fixed {
    (f * 65536.0) as i32
}

fn from_fixed(fixed: Fixed) -> f32 {
    fixed as f32 / 65536.0
}

// Fixed-point multiplication
fn fixed_mul(a: Fixed, b: Fixed) -> Fixed {
    ((a as i64 * b as i64) >> 16) as i32
}
```

**Performance**:
- Benefit: 5-10x faster on MCUs without FPU
- Trade-off: Reduced precision (acceptable for graphics)

**Note**: Your STM32WBA65RI has FPU, so this is less critical but still useful for inner loops.

---

### 12. DMA-Based Rendering (Double Buffering)

**Description**: Use DMA to transfer framebuffer while CPU renders next frame.

```rust
// Start DMA transfer of current frame
display.start_dma_transfer(&framebuffer_a);

// Render next frame to alternate buffer
render_to(&mut framebuffer_b);

// Wait for DMA, then swap
display.wait_dma();
swap(&mut framebuffer_a, &mut framebuffer_b);
```

**Performance**:
- Benefit: ~30% FPS improvement (parallel rendering + transfer)
- Cost: 2x framebuffer memory
- Essential: For any production embedded graphics system

**Recommended**: ⭐⭐⭐⭐⭐ Must-have for real applications

---

### 13. Tile-Based Deferred Rendering

**Description**: Divide screen into tiles, render each independently.

**Benefits**:
- Better cache locality
- Can parallelize across cores
- Used in mobile GPUs (Mali, Adreno)

**Implementation**:
```rust
const TILE_SIZE: usize = 16;

for tile_y in 0..(height / TILE_SIZE) {
    for tile_x in 0..(width / TILE_SIZE) {
        // Render only triangles overlapping this tile
        render_tile(tile_x, tile_y, &triangles);
    }
}
```

**Performance**:
- Benefit: Better memory access patterns
- Enables: Reduced Z-buffer size (per-tile instead of full-screen)

**Recommended**: ⭐⭐⭐ Advanced technique

---

### 14. Scanline-Based Effects

**Description**: Process effects during scanline rendering instead of post-processing.

**Techniques**:
```rust
// Fog (depth-based)
let fog_factor = (z - fog_near) / (fog_far - fog_near);
let final_color = lerp(color, fog_color, fog_factor);

// Dithering (more perceived colors)
let dither_pattern = DITHER_MATRIX[x % 4][y % 4];
let dithered_color = color + dither_pattern;

// Alpha blending
let final_color = src_color * alpha + dst_color * (1.0 - alpha);
```

**Performance**:
- Cost: Minimal (happens during rasterization anyway)
- Benefit: Rich visual effects with no extra passes

**Recommended**: ⭐⭐⭐⭐ Easy wins

---

### 15. Ray Casting (Not Ray Tracing!)

**Description**: Cast one ray per screen column (Wolfenstein 3D style).

**Use Case**: Grid-based worlds (mazes, dungeons)

```rust
// For each screen column
for x in 0..screen_width {
    let ray_angle = player_angle + fov * (x / screen_width - 0.5);

    // Cast ray until hitting wall
    let (distance, wall_type) = cast_ray(ray_angle);

    // Draw vertical line for this column
    let wall_height = screen_height / distance;
    draw_vertical_line(x, wall_height, wall_type);
}
```

**Performance**:
- Cost: Very low (simpler than 3D polygon rendering)
- Benefit: Perfect for maze/dungeon games
- Limitation: Grid-based worlds only

**Recommended**: ⭐⭐⭐⭐ Perfect for specific game types

---

## Top Recommendations for STM32WBA65RI

Given your hardware (ARM Cortex-M33 with FPU), prioritize these:

### Immediate Impact (Easy to Add)
1. **Gouraud shading** ⭐⭐⭐⭐⭐
   - Easy to implement
   - Huge visual improvement
   - Low performance cost

2. **Billboarding** ⭐⭐⭐⭐⭐
   - Essential for particles and effects
   - Very low cost
   - Large visual impact

3. **Scanline-based effects** ⭐⭐⭐⭐
   - Fog, dithering
   - Minimal cost
   - Professional look

### Medium-Term (Moderate Effort)
4. **Affine texture mapping** ⭐⭐⭐⭐
   - Classic 3D look
   - Manageable performance cost
   - Large visual improvement

5. **LOD system** ⭐⭐⭐⭐⭐
   - Major performance boost
   - Essential for complex scenes
   - Straightforward implementation

6. **DMA rendering** ⭐⭐⭐⭐⭐
   - Free 30% performance gain
   - Essential for production
   - Requires hardware setup

### Advanced (For Optimization)
7. **Lightmapping** ⭐⭐⭐⭐
   - Beautiful static lighting
   - Requires offline baking tool
   - Worth it for quality

8. **Portal rendering** ⭐⭐⭐⭐
   - If building indoor environments
   - Large performance win
   - Requires scene design

---

## Implementation Priority

**Phase 1: Visual Quality**
- Gouraud shading
- Billboarding
- Fog effects

**Phase 2: Performance**
- LOD system
- DMA rendering
- Better frustum culling

**Phase 3: Advanced Graphics**
- Affine texture mapping
- Lightmapping
- Shadows (if needed)

**Phase 4: Specialized**
- Portal rendering (if indoor game)
- Ray casting (if maze game)
- Mode 7 (if racing game)

---

## Performance Budget Example

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

---

## References and Further Reading

- [Tricks of the 3D Game Programming Gurus](https://www.amazon.com/Tricks-Game-Programming-Gurus-Advanced/dp/0672318350) - Classic techniques
- [Michael Abrash's Graphics Programming Black Book](https://github.com/jagregory/abrash-black-book) - Optimization techniques
- [PSX Graphics Programming](https://psx-spx.consoledev.net/graphicsprocessingunitgpu/) - Affine texture mapping
- [Fabien Sanglard's Game Engine Black Books](https://fabiensanglard.net/gebb/) - Doom, Quake architecture

---

## Contributing

Feel free to implement any of these techniques and submit PRs! Priority areas:
1. Gouraud shading
2. Affine texture mapping
3. LOD system
4. DMA rendering support

---

*Last updated: 2026-01-31*
