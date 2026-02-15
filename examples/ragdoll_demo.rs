//! Ragdoll demo
//!
//! A simple ragdoll made of connected body parts using distance constraints.
//! Demonstrates multi-body constraint systems with gravity and collisions.
//!
//! Controls:
//! - SPACE: Toss the ragdoll upward with random spin
//! - R: Reset ragdoll to starting pose
//! - ESC: Exit

use embedded_3dgfx::K3dengine;
use embedded_3dgfx::draw::draw;
use embedded_3dgfx::mesh::{Geometry, K3dMesh, RenderMode};
use embedded_3dgfx::perfcounter::PerformanceCounter;
use embedded_3dgfx::physics::{BodyId, Collider, PhysicsWorld, RigidBody, sync_body_to_mesh};
use embedded_graphics::mono_font::{MonoTextStyle, ascii::FONT_6X10};
use embedded_graphics::text::Text;
use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
use embedded_graphics_core::prelude::*;
use embedded_graphics_simulator::{
    OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent, Window, sdl2::Keycode,
};
use nalgebra::{Point3, UnitQuaternion, Vector3};
use std::thread;
use std::time::Duration;

fn make_cube() -> (Vec<[f32; 3]>, Vec<[usize; 3]>, Vec<[f32; 3]>) {
    let vertices = vec![
        [-0.5, -0.5, 0.5], [0.5, -0.5, 0.5], [0.5, 0.5, 0.5], [-0.5, 0.5, 0.5],
        [-0.5, -0.5, -0.5], [0.5, -0.5, -0.5], [0.5, 0.5, -0.5], [-0.5, 0.5, -0.5],
    ];
    let faces = vec![
        [0, 1, 2], [0, 2, 3], [5, 4, 7], [5, 7, 6],
        [3, 2, 6], [3, 6, 7], [4, 5, 1], [4, 1, 0],
        [1, 5, 6], [1, 6, 2], [4, 0, 3], [4, 3, 7],
    ];
    let normals = vec![
        [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0.0, 0.0, -1.0], [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, -1.0, 0.0], [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [-1.0, 0.0, 0.0],
    ];
    (vertices, faces, normals)
}

/// Ragdoll body part definitions: (name, position, mass, scale, color)
struct PartDef {
    pos: Vector3<f32>,
    mass: f32,
    scale: f32,
    color: Rgb565,
    radius: f32,
}

fn main() {
    let mut display = SimulatorDisplay::<Rgb565>::new(Size::new(640, 480));
    let output_settings = OutputSettingsBuilder::new().scale(1).build();
    let mut window = Window::new("Ragdoll Demo - SPACE=toss R=reset ESC=exit", &output_settings);

    let mut engine = K3dengine::new(640, 480);
    engine.camera.set_position(Point3::new(0.0, 5.0, 18.0));
    engine.camera.set_target(Point3::new(0.0, 4.0, 0.0));

    let (vertices, faces, normals) = make_cube();

    // Physics: up to 16 bodies, 16 constraints
    let mut physics = PhysicsWorld::<16, 16>::new();
    physics.set_gravity(Vector3::new(0.0, -9.81, 0.0));
    physics.solver_iterations = 10;

    // Define ragdoll parts: head, torso, left/right upper arm, left/right lower arm,
    // left/right upper leg, left/right lower leg = 9 parts
    let parts = [
        PartDef { pos: Vector3::new(0.0, 9.0, 0.0), mass: 0.8, scale: 0.7, color: Rgb565::CSS_PEACH_PUFF, radius: 0.35 },  // head
        PartDef { pos: Vector3::new(0.0, 7.5, 0.0), mass: 2.0, scale: 0.9, color: Rgb565::CSS_DODGER_BLUE, radius: 0.45 },  // torso
        PartDef { pos: Vector3::new(-1.5, 7.5, 0.0), mass: 0.5, scale: 0.5, color: Rgb565::CSS_DODGER_BLUE, radius: 0.25 }, // left upper arm
        PartDef { pos: Vector3::new(-2.5, 7.5, 0.0), mass: 0.4, scale: 0.45, color: Rgb565::CSS_PEACH_PUFF, radius: 0.22 }, // left lower arm
        PartDef { pos: Vector3::new(1.5, 7.5, 0.0), mass: 0.5, scale: 0.5, color: Rgb565::CSS_DODGER_BLUE, radius: 0.25 },  // right upper arm
        PartDef { pos: Vector3::new(2.5, 7.5, 0.0), mass: 0.4, scale: 0.45, color: Rgb565::CSS_PEACH_PUFF, radius: 0.22 },  // right lower arm
        PartDef { pos: Vector3::new(-0.5, 5.5, 0.0), mass: 0.8, scale: 0.55, color: Rgb565::CSS_DARK_SLATE_GRAY, radius: 0.28 }, // left upper leg
        PartDef { pos: Vector3::new(-0.5, 4.0, 0.0), mass: 0.6, scale: 0.5, color: Rgb565::CSS_PEACH_PUFF, radius: 0.25 },  // left lower leg
        PartDef { pos: Vector3::new(0.5, 5.5, 0.0), mass: 0.8, scale: 0.55, color: Rgb565::CSS_DARK_SLATE_GRAY, radius: 0.28 },  // right upper leg
        PartDef { pos: Vector3::new(0.5, 4.0, 0.0), mass: 0.6, scale: 0.5, color: Rgb565::CSS_PEACH_PUFF, radius: 0.25 },   // right lower leg
    ];

    let mut body_ids: Vec<BodyId> = Vec::new();
    let mut meshes: Vec<K3dMesh> = Vec::new();
    let mut initial_positions: Vec<Vector3<f32>> = Vec::new();

    for part in &parts {
        let body = RigidBody::new(part.mass)
            .with_position(part.pos)
            .with_damping(0.01)
            .with_collider(Collider::Sphere { radius: part.radius })
            .with_inertia_sphere(part.radius)
            .with_angular_damping(0.05)
            .with_restitution(0.2)
            .with_friction(0.6);
        let id = physics.add_body(body).unwrap();
        body_ids.push(id);
        initial_positions.push(part.pos);

        let geometry = Geometry {
            vertices: &vertices, faces: &faces, colors: &[], lines: &[],
            normals: &normals, uvs: &[], texture_id: None,
        };
        let mut mesh = K3dMesh::new(geometry);
        mesh.set_render_mode(RenderMode::SolidLightDir(Vector3::new(0.5, 1.0, 0.3)));
        mesh.set_color(part.color);
        mesh.set_scale(part.scale);
        mesh.set_position(part.pos.x, part.pos.y, part.pos.z);
        meshes.push(mesh);
    }

    // Connect body parts with distance constraints
    // Indices: 0=head, 1=torso, 2=L-upper-arm, 3=L-lower-arm,
    //          4=R-upper-arm, 5=R-lower-arm, 6=L-upper-leg, 7=L-lower-leg,
    //          8=R-upper-leg, 9=R-lower-leg
    let joints: &[(usize, usize)] = &[
        (0, 1), // head - torso
        (1, 2), // torso - left upper arm
        (2, 3), // left upper arm - left lower arm
        (1, 4), // torso - right upper arm
        (4, 5), // right upper arm - right lower arm
        (1, 6), // torso - left upper leg
        (6, 7), // left upper leg - left lower leg
        (1, 8), // torso - right upper leg
        (8, 9), // right upper leg - right lower leg
    ];

    for &(a, b) in joints {
        physics.add_distance_constraint(
            body_ids[a], Vector3::zeros(),
            body_ids[b], Vector3::zeros(),
            0.0, // rigid
        ).unwrap();
    }

    // Floor
    physics.add_body(
        RigidBody::new_static()
            .with_position(Vector3::new(0.0, -0.1, 0.0))
            .with_collider(Collider::Aabb { half_extents: Vector3::new(10.0, 0.1, 10.0) })
            .with_restitution(0.3)
            .with_friction(0.7),
    ).unwrap();

    let floor_verts: Vec<[f32; 3]> = vec![
        [-8.0, 0.0, 5.0], [8.0, 0.0, 5.0], [8.0, 0.0, -5.0], [-8.0, 0.0, -5.0],
    ];
    let floor_faces: Vec<[usize; 3]> = vec![[0, 1, 2], [0, 2, 3]];
    let floor_normals: Vec<[f32; 3]> = vec![[0.0, 1.0, 0.0], [0.0, 1.0, 0.0]];
    let floor_geom = Geometry {
        vertices: &floor_verts, faces: &floor_faces, colors: &[], lines: &[],
        normals: &floor_normals, uvs: &[], texture_id: None,
    };
    let mut floor_mesh = K3dMesh::new(floor_geom);
    floor_mesh.set_render_mode(RenderMode::SolidLightDir(Vector3::new(0.5, 1.0, 0.3)));
    floor_mesh.set_color(Rgb565::new(8, 16, 8));

    let mut perf = PerformanceCounter::new();
    perf.only_fps(true);
    let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_WHITE);
    let mut toss_counter: u32 = 0;

    println!("Ragdoll Demo (10 parts, 9 joints)");
    println!("SPACE = toss ragdoll, R = reset, ESC = exit");

    display.clear(Rgb565::BLACK).unwrap();
    window.update(&display);

    'running: loop {
        perf.start_of_frame();

        for event in window.events() {
            match event {
                SimulatorEvent::KeyDown { keycode, .. } => match keycode {
                    Keycode::Escape => break 'running,
                    Keycode::Space => {
                        toss_counter = toss_counter.wrapping_add(1);
                        let angle = toss_counter as f32 * 2.1;
                        for &id in &body_ids {
                            let body = physics.body_mut(id).unwrap();
                            if body.active {
                                body.apply_impulse(Vector3::new(
                                    angle.sin() * 4.0,
                                    12.0,
                                    angle.cos() * 2.0,
                                ));
                                body.apply_angular_impulse(Vector3::new(
                                    angle.cos() * 3.0,
                                    angle.sin() * 2.0,
                                    (angle * 0.5).sin() * 3.0,
                                ));
                            }
                        }
                    }
                    Keycode::R => {
                        for (i, &id) in body_ids.iter().enumerate() {
                            physics.set_active(id, true);
                            let body = physics.body_mut(id).unwrap();
                            body.position = initial_positions[i];
                            body.velocity = Vector3::zeros();
                            body.orientation = UnitQuaternion::identity();
                            body.angular_velocity = Vector3::zeros();
                        }
                    }
                    _ => {}
                },
                SimulatorEvent::Quit => break 'running,
                _ => {}
            }
        }

        physics.step_fixed::<16>(1.0 / 60.0, 4);

        for (i, &id) in body_ids.iter().enumerate() {
            let body = physics.body(id).unwrap();
            sync_body_to_mesh(body, &mut meshes[i]);
        }

        display.clear(Rgb565::BLACK).unwrap();

        let all_meshes: Vec<&K3dMesh> = meshes.iter().chain(std::iter::once(&floor_mesh)).collect();
        engine.render(all_meshes.into_iter(), |prim| { draw(prim, &mut display); });

        perf.print();
        Text::new(perf.get_text(), Point::new(10, 15), text_style)
            .draw(&mut display).unwrap();
        Text::new("SPACE=toss R=reset ESC=exit", Point::new(10, 470), text_style)
            .draw(&mut display).unwrap();

        window.update(&display);
        thread::sleep(Duration::from_millis(16));
    }
}
