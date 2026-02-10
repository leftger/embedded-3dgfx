//! Physics demo: falling cubes with gravity
//!
//! Demonstrates the physics engine foundation:
//! - Gravity and freefall
//! - Multiple rigid bodies
//! - Force application (impulse on keypress)
//! - Syncing physics state to mesh transforms
//!
//! Controls:
//! - SPACE: Apply upward impulse to all cubes
//! - R: Reset positions
//! - ESC: Exit

use embedded_3dgfx::K3dengine;
use embedded_3dgfx::draw::draw;
use embedded_3dgfx::mesh::{Geometry, K3dMesh, RenderMode};
use embedded_3dgfx::perfcounter::PerformanceCounter;
use embedded_3dgfx::physics::{BodyId, PhysicsWorld, RigidBody, sync_body_to_mesh};
use embedded_graphics::mono_font::{MonoTextStyle, ascii::FONT_6X10};
use embedded_graphics::text::Text;
use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
use embedded_graphics_core::prelude::*;
use embedded_graphics_simulator::{
    OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent, Window, sdl2::Keycode,
};
use nalgebra::{Point3, Vector3};
use std::thread;
use std::time::Duration;

fn make_cube() -> (Vec<[f32; 3]>, Vec<[usize; 3]>, Vec<[f32; 3]>) {
    let vertices = vec![
        [-0.5, -0.5, 0.5],
        [0.5, -0.5, 0.5],
        [0.5, 0.5, 0.5],
        [-0.5, 0.5, 0.5],
        [-0.5, -0.5, -0.5],
        [0.5, -0.5, -0.5],
        [0.5, 0.5, -0.5],
        [-0.5, 0.5, -0.5],
    ];

    let faces = vec![
        [0, 1, 2], [0, 2, 3], // front
        [5, 4, 7], [5, 7, 6], // back
        [3, 2, 6], [3, 6, 7], // top
        [4, 5, 1], [4, 1, 0], // bottom
        [1, 5, 6], [1, 6, 2], // right
        [4, 0, 3], [4, 3, 7], // left
    ];

    let normals = vec![
        [0.0, 0.0, 1.0], [0.0, 0.0, 1.0],
        [0.0, 0.0, -1.0], [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0], [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0], [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0], [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0], [-1.0, 0.0, 0.0],
    ];

    (vertices, faces, normals)
}

const NUM_CUBES: usize = 5;

fn main() {
    let mut display = SimulatorDisplay::<Rgb565>::new(Size::new(640, 480));
    let output_settings = OutputSettingsBuilder::new().scale(1).build();
    let mut window = Window::new(
        "Physics Demo - SPACE=impulse R=reset ESC=exit",
        &output_settings,
    );

    // Create 3D engine
    let mut engine = K3dengine::new(640, 480);
    engine.camera.set_position(Point3::new(0.0, 5.0, 15.0));
    engine.camera.set_target(Point3::new(0.0, 2.0, 0.0));

    // Create cube geometry (shared by all cubes)
    let (vertices, faces, normals) = make_cube();

    // Create physics world
    let mut physics = PhysicsWorld::<8>::new();
    physics.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    // Spawn cubes at different positions with different colors
    let colors = [
        Rgb565::CSS_CYAN,
        Rgb565::CSS_ORANGE,
        Rgb565::CSS_LIME,
        Rgb565::CSS_MAGENTA,
        Rgb565::CSS_YELLOW,
    ];

    let initial_positions: Vec<Vector3<f32>> = (0..NUM_CUBES)
        .map(|i| {
            Vector3::new(
                (i as f32 - (NUM_CUBES as f32 - 1.0) / 2.0) * 2.5, // spread along X
                5.0 + i as f32 * 2.0,                                // staggered heights
                0.0,
            )
        })
        .collect();

    let mut body_ids: Vec<BodyId> = Vec::new();
    let mut meshes: Vec<K3dMesh> = Vec::new();

    for i in 0..NUM_CUBES {
        // Physics body
        let body = RigidBody::new(1.0)
            .with_position(initial_positions[i])
            .with_damping(0.02)
            .with_restitution(0.6);
        let id = physics.add_body(body).unwrap();
        body_ids.push(id);

        // Mesh
        let geometry = Geometry {
            vertices: &vertices,
            faces: &faces,
            colors: &[],
            lines: &[],
            normals: &normals,
            uvs: &[],
            texture_id: None,
        };
        let mut mesh = K3dMesh::new(geometry);
        mesh.set_render_mode(RenderMode::SolidLightDir(Vector3::new(0.5, 1.0, 0.3)));
        mesh.set_color(colors[i % colors.len()]);
        mesh.set_position(
            initial_positions[i].x,
            initial_positions[i].y,
            initial_positions[i].z,
        );
        meshes.push(mesh);
    }

    // Add a static floor (just a visual, no physics collision yet)
    let floor_verts: Vec<[f32; 3]> = vec![
        [-8.0, 0.0, 5.0],
        [8.0, 0.0, 5.0],
        [8.0, 0.0, -5.0],
        [-8.0, 0.0, -5.0],
    ];
    let floor_faces: Vec<[usize; 3]> = vec![[0, 1, 2], [0, 2, 3]];
    let floor_normals: Vec<[f32; 3]> = vec![[0.0, 1.0, 0.0], [0.0, 1.0, 0.0]];

    let floor_geometry = Geometry {
        vertices: &floor_verts,
        faces: &floor_faces,
        colors: &[],
        lines: &[],
        normals: &floor_normals,
        uvs: &[],
        texture_id: None,
    };
    let mut floor_mesh = K3dMesh::new(floor_geometry);
    floor_mesh.set_render_mode(RenderMode::SolidLightDir(Vector3::new(0.5, 1.0, 0.3)));
    floor_mesh.set_color(Rgb565::new(8, 16, 8));

    let mut perf = PerformanceCounter::new();
    perf.only_fps(true);
    let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_WHITE);

    let dt = 1.0 / 60.0;

    println!("Physics Demo");
    println!("SPACE = apply upward impulse");
    println!("R = reset positions");
    println!("ESC = exit");

    display.clear(Rgb565::BLACK).unwrap();
    window.update(&display);

    'running: loop {
        perf.start_of_frame();

        // Handle events
        for event in window.events() {
            match event {
                SimulatorEvent::KeyDown { keycode, .. } => match keycode {
                    Keycode::Escape => break 'running,
                    Keycode::Space => {
                        // Apply upward impulse to all cubes
                        for &id in &body_ids {
                            physics
                                .body_mut(id)
                                .unwrap()
                                .apply_impulse(Vector3::new(0.0, 8.0, 0.0));
                        }
                    }
                    Keycode::R => {
                        // Reset positions
                        for (i, &id) in body_ids.iter().enumerate() {
                            let body = physics.body_mut(id).unwrap();
                            body.position = initial_positions[i];
                            body.velocity = Vector3::zeros();
                        }
                    }
                    _ => {}
                },
                SimulatorEvent::Quit => break 'running,
                _ => {}
            }
        }

        // Step physics (2 substeps for stability)
        physics.step_fixed(dt, 2);

        // Simple floor collision: clamp Y >= 0.5 (half cube height) and bounce
        for &id in &body_ids {
            let body = physics.body_mut(id).unwrap();
            if body.position.y < 0.5 {
                body.position.y = 0.5;
                if body.velocity.y < 0.0 {
                    body.velocity.y = -body.velocity.y * body.restitution;
                    // Stop tiny bounces
                    if body.velocity.y.abs() < 0.5 {
                        body.velocity.y = 0.0;
                    }
                }
            }
        }

        // Sync physics -> meshes
        for (i, &id) in body_ids.iter().enumerate() {
            let body = physics.body(id).unwrap();
            sync_body_to_mesh(body, &mut meshes[i]);
        }

        // Render
        display.clear(Rgb565::BLACK).unwrap();

        let all_meshes: Vec<&K3dMesh> = meshes.iter().chain(std::iter::once(&floor_mesh)).collect();
        engine.render(all_meshes.into_iter(), |prim| {
            draw(prim, &mut display);
        });

        // HUD
        perf.print();
        Text::new(perf.get_text(), Point::new(10, 15), text_style)
            .draw(&mut display)
            .unwrap();
        Text::new("SPACE=impulse R=reset", Point::new(10, 470), text_style)
            .draw(&mut display)
            .unwrap();

        window.update(&display);
        thread::sleep(Duration::from_millis(16));
    }
}
