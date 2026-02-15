//! Skeletal Subspace Deformation (SSD / Linear Blend Skinning) Demo
//!
//! Demonstrates vertex skinning: a mesh deformed by an underlying skeleton.
//! Each vertex is influenced by one or more bones with associated weights.
//! The skeleton is animated procedurally and the mesh vertices are computed
//! each frame using SSD (also known as linear blend skinning).
//!
//! This demo builds a simple "arm" mesh with two bones (upper arm + forearm)
//! and shows the mesh deforming smoothly as the bones rotate.
//!
//! Controls:
//! - SPACE: Toggle animation playback
//! - LEFT/RIGHT: Manually adjust forearm bend angle
//! - UP/DOWN: Manually adjust upper arm rotation
//! - R: Reset to bind pose
//! - ESC: Exit

use embedded_3dgfx::K3dengine;
use embedded_3dgfx::draw::draw_zbuffered;
use embedded_3dgfx::mesh::{Geometry, K3dMesh, RenderMode};
use embedded_3dgfx::perfcounter::PerformanceCounter;
use embedded_graphics::mono_font::{MonoTextStyle, ascii::FONT_6X10};
use embedded_graphics::text::Text;
use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
use embedded_graphics_core::prelude::*;
use embedded_graphics_simulator::{
    OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent, Window, sdl2::Keycode,
};
use nalgebra::Point3;
use std::f32::consts::PI;
use std::thread;
use std::time::Duration;

/// A bone in the skeleton
struct Bone {
    /// Position of the bone joint (pivot point) in bind pose
    origin: [f32; 3],
    /// Length of the bone along its local Y axis
    length: f32,
    /// Parent bone index (None for root)
    parent: Option<usize>,
    /// Current rotation angle (radians, around Z axis for simplicity)
    angle: f32,
}

/// Skin weight: which bone influences a vertex and by how much
#[derive(Clone)]
struct SkinWeight {
    bone_indices: [usize; 2],
    weights: [f32; 2],
}

/// Compute a 2D rotation matrix (around Z axis) as a 3x3 row-major array
fn rotation_z(angle: f32) -> [[f32; 3]; 3] {
    let c = angle.cos();
    let s = angle.sin();
    [
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ]
}

/// Multiply 3x3 matrix by a 3D point
fn mat3_mul_point(m: &[[f32; 3]; 3], p: &[f32; 3]) -> [f32; 3] {
    [
        m[0][0] * p[0] + m[0][1] * p[1] + m[0][2] * p[2],
        m[1][0] * p[0] + m[1][1] * p[1] + m[1][2] * p[2],
        m[2][0] * p[0] + m[2][1] * p[1] + m[2][2] * p[2],
    ]
}

/// Multiply two 3x3 matrices
fn mat3_mul(a: &[[f32; 3]; 3], b: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut r = [[0.0f32; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    r
}

/// Compute world-space transform for each bone: (rotation_matrix, world_origin)
fn compute_bone_transforms(bones: &[Bone]) -> Vec<([[f32; 3]; 3], [f32; 3])> {
    let mut transforms = Vec::with_capacity(bones.len());

    for (_i, bone) in bones.iter().enumerate() {
        let local_rot = rotation_z(bone.angle);

        if let Some(parent_idx) = bone.parent {
            let (parent_rot, parent_origin): &([[f32; 3]; 3], [f32; 3]) = &transforms[parent_idx];
            // World rotation = parent_rot * local_rot
            let world_rot = mat3_mul(parent_rot, &local_rot);
            // World origin = parent's tip position (parent_origin + parent_rot * [0, parent_length, 0])
            let parent_bone = &bones[parent_idx];
            let parent_tip = [0.0, parent_bone.length, 0.0];
            let rotated_tip = mat3_mul_point(parent_rot, &parent_tip);
            let world_origin = [
                parent_origin[0] + rotated_tip[0],
                parent_origin[1] + rotated_tip[1],
                parent_origin[2] + rotated_tip[2],
            ];
            transforms.push((world_rot, world_origin));
        } else {
            transforms.push((local_rot, bone.origin));
        }
    }
    transforms
}

/// Apply SSD: transform bind-pose vertices to posed vertices
fn skin_vertices(
    bind_vertices: &[[f32; 3]],
    weights: &[SkinWeight],
    _bones: &[Bone],
    bind_transforms: &[([[f32; 3]; 3], [f32; 3])],
    posed_transforms: &[([[f32; 3]; 3], [f32; 3])],
    output: &mut [[f32; 3]],
) {
    for (vi, vert) in bind_vertices.iter().enumerate() {
        let sw = &weights[vi];
        let mut result = [0.0f32; 3];

        for k in 0..2 {
            let w = sw.weights[k];
            if w < 1e-6 {
                continue;
            }
            let bi = sw.bone_indices[k];

            // Transform vertex to bone-local space (bind pose inverse)
            let (bind_rot, bind_origin) = &bind_transforms[bi];
            let local = [
                vert[0] - bind_origin[0],
                vert[1] - bind_origin[1],
                vert[2] - bind_origin[2],
            ];
            // Inverse rotation (transpose for orthogonal matrix)
            let bind_rot_inv = [
                [bind_rot[0][0], bind_rot[1][0], bind_rot[2][0]],
                [bind_rot[0][1], bind_rot[1][1], bind_rot[2][1]],
                [bind_rot[0][2], bind_rot[1][2], bind_rot[2][2]],
            ];
            let bone_local = mat3_mul_point(&bind_rot_inv, &local);

            // Transform from bone-local to world (posed)
            let (posed_rot, posed_origin) = &posed_transforms[bi];
            let posed = mat3_mul_point(posed_rot, &bone_local);

            result[0] += w * (posed[0] + posed_origin[0]);
            result[1] += w * (posed[1] + posed_origin[1]);
            result[2] += w * (posed[2] + posed_origin[2]);
        }

        output[vi] = result;
    }
}

/// Create a cylindrical "arm" mesh along the Y axis with given segments
fn create_arm_mesh(
    base_y: f32,
    total_length: f32,
    radius: f32,
    segments_along: usize,
    segments_around: usize,
) -> (Vec<[f32; 3]>, Vec<[usize; 3]>, Vec<SkinWeight>) {
    let mut vertices = Vec::new();
    let mut weights = Vec::new();

    let bone_boundary = total_length * 0.5; // midpoint where bone 0 ends and bone 1 starts

    for iy in 0..=segments_along {
        let t = iy as f32 / segments_along as f32;
        let y = base_y + t * total_length;

        // Compute bone weight based on position along the arm
        let along = t * total_length;
        let blend_zone = total_length * 0.2; // 20% of total length is the blend zone
        let w1 = if along < bone_boundary - blend_zone {
            0.0 // fully bone 0
        } else if along > bone_boundary + blend_zone {
            1.0 // fully bone 1
        } else {
            // smooth blend
            (along - (bone_boundary - blend_zone)) / (2.0 * blend_zone)
        };

        for ia in 0..segments_around {
            let theta = (ia as f32 / segments_around as f32) * 2.0 * PI;
            let x = radius * theta.cos();
            let z = radius * theta.sin();

            vertices.push([x, y, z]);
            weights.push(SkinWeight {
                bone_indices: [0, 1],
                weights: [1.0 - w1, w1],
            });
        }
    }

    // Generate faces (triangle strip between rings)
    let mut faces = Vec::new();
    for iy in 0..segments_along {
        for ia in 0..segments_around {
            let curr = iy * segments_around + ia;
            let next_ring = curr + segments_around;
            let next_around = iy * segments_around + (ia + 1) % segments_around;
            let diag = next_around + segments_around;

            faces.push([curr, next_ring, next_around]);
            faces.push([next_around, next_ring, diag]);
        }
    }

    (vertices, faces, weights)
}

fn main() {
    let mut display = SimulatorDisplay::<Rgb565>::new(Size::new(800, 600));
    let output_settings = OutputSettingsBuilder::new().scale(1).build();
    let mut window = Window::new("SSD Skinning Demo - SPACE=anim arrows=manual R=reset ESC=exit", &output_settings);

    let mut engine = K3dengine::new(800, 600);
    engine.camera.set_position(Point3::new(6.0, 4.0, 10.0));
    engine.camera.set_target(Point3::new(0.0, 3.0, 0.0));

    let mut perf = PerformanceCounter::new();
    perf.only_fps(true);
    let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_WHITE);
    let mut zbuffer = vec![u32::MAX; 800 * 600];

    // Skeleton: 2 bones (upper arm + forearm) forming a simple arm
    let mut bones = vec![
        Bone { origin: [0.0, 0.0, 0.0], length: 3.0, parent: None, angle: 0.0 },      // upper arm
        Bone { origin: [0.0, 3.0, 0.0], length: 3.0, parent: Some(0), angle: 0.0 },    // forearm
    ];

    // Compute bind-pose transforms (bones at angle=0)
    let bind_transforms = compute_bone_transforms(&bones);

    // Create arm mesh
    let arm_length = 6.0;
    let (bind_vertices, arm_faces, skin_weights) = create_arm_mesh(
        0.0, arm_length, 0.4, 16, 8,
    );

    let mut animated_verts = vec![[0.0f32; 3]; bind_vertices.len()];

    // Also create a second arm to show comparison (bind pose, no deformation)
    let bind_verts_offset: Vec<[f32; 3]> = bind_vertices.iter()
        .map(|v| [v[0] + 3.0, v[1], v[2]])
        .collect();

    let mut anim_playing = true;
    let mut anim_time = 0.0f32;
    let mut manual_upper = 0.0f32;
    let mut manual_forearm = 0.0f32;

    println!("SSD / Linear Blend Skinning Demo");
    println!("Left: skinned mesh (deforming)");
    println!("Right: bind pose (reference)");
    println!("SPACE = toggle animation, arrows = manual control");
    println!("R = reset, ESC = exit");

    display.clear(Rgb565::BLACK).unwrap();
    window.update(&display);

    'running: loop {
        perf.start_of_frame();

        for event in window.events() {
            match event {
                SimulatorEvent::KeyDown { keycode, .. } => match keycode {
                    Keycode::Escape => break 'running,
                    Keycode::Space => {
                        anim_playing = !anim_playing;
                        println!("Animation: {}", if anim_playing { "PLAYING" } else { "PAUSED" });
                    }
                    Keycode::Left => {
                        anim_playing = false;
                        manual_forearm -= 0.1;
                    }
                    Keycode::Right => {
                        anim_playing = false;
                        manual_forearm += 0.1;
                    }
                    Keycode::Up => {
                        anim_playing = false;
                        manual_upper -= 0.1;
                    }
                    Keycode::Down => {
                        anim_playing = false;
                        manual_upper += 0.1;
                    }
                    Keycode::R => {
                        anim_time = 0.0;
                        manual_upper = 0.0;
                        manual_forearm = 0.0;
                        bones[0].angle = 0.0;
                        bones[1].angle = 0.0;
                        println!("Reset to bind pose");
                    }
                    _ => {}
                },
                SimulatorEvent::Quit => break 'running,
                _ => {}
            }
        }

        // Update bone angles
        if anim_playing {
            let dt = 1.0 / 60.0;
            anim_time += dt;

            // Procedural animation: wave the arm
            bones[0].angle = (anim_time * 1.5).sin() * 0.4;
            bones[1].angle = ((anim_time * 2.0) + 0.5).sin() * 0.8 - 0.3;
        } else {
            bones[0].angle = manual_upper;
            bones[1].angle = manual_forearm;
        }

        // Compute posed bone transforms
        let posed_transforms = compute_bone_transforms(&bones);

        // Apply SSD skinning
        skin_vertices(
            &bind_vertices,
            &skin_weights,
            &bones,
            &bind_transforms,
            &posed_transforms,
            &mut animated_verts,
        );

        // Create skinned mesh
        let skinned_geom = Geometry {
            vertices: &animated_verts,
            faces: &arm_faces,
            colors: &[], lines: &[],
            normals: &[], uvs: &[], texture_id: None,
        };
        let mut skinned_mesh = K3dMesh::new(skinned_geom);
        skinned_mesh.set_position(-1.5, 0.0, 0.0);
        skinned_mesh.set_color(Rgb565::CSS_CORAL);
        skinned_mesh.set_render_mode(RenderMode::Solid);

        // Create wireframe of the same skinned mesh
        let skinned_wire_geom = Geometry {
            vertices: &animated_verts,
            faces: &arm_faces,
            colors: &[], lines: &[],
            normals: &[], uvs: &[], texture_id: None,
        };
        let mut skinned_wire = K3dMesh::new(skinned_wire_geom);
        skinned_wire.set_position(-1.5, 0.0, 0.0);
        skinned_wire.set_color(Rgb565::CSS_WHITE);
        skinned_wire.set_render_mode(RenderMode::Lines);

        // Reference bind pose mesh (static)
        let bind_geom = Geometry {
            vertices: &bind_verts_offset,
            faces: &arm_faces,
            colors: &[], lines: &[],
            normals: &[], uvs: &[], texture_id: None,
        };
        let mut bind_mesh = K3dMesh::new(bind_geom);
        bind_mesh.set_position(0.0, 0.0, 0.0);
        bind_mesh.set_color(Rgb565::CSS_DIM_GRAY);
        bind_mesh.set_render_mode(RenderMode::Lines);

        // Clear
        display.clear(Rgb565::BLACK).unwrap();
        zbuffer.fill(u32::MAX);

        // Render
        engine.render([&skinned_mesh, &skinned_wire, &bind_mesh].iter().copied(), |prim| {
            draw_zbuffered(prim, &mut display, &mut zbuffer, 800);
        });

        // HUD
        perf.print();
        let info = format!(
            "{}\nSSD Skinning: 2 bones, {} verts\nUpper: {:.1} rad | Forearm: {:.1} rad | {}",
            perf.get_text(),
            bind_vertices.len(),
            bones[0].angle,
            bones[1].angle,
            if anim_playing { "ANIM" } else { "MANUAL" },
        );
        Text::new(&info, Point::new(10, 20), text_style)
            .draw(&mut display).unwrap();
        Text::new("SPC:anim L/R:forearm U/D:upper R:reset ESC:exit", Point::new(10, 580), text_style)
            .draw(&mut display).unwrap();

        window.update(&display);
        thread::sleep(Duration::from_millis(16));
    }
}
