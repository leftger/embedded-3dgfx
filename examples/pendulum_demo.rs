//! Pendulum chain demo
//!
//! A chain of spheres hanging from a fixed point, swinging under gravity.
//! Demonstrates distance constraints forming a multi-link pendulum.
//!
//! Controls:
//! - SPACE: Apply sideways impulse to the bottom link
//! - R: Reset chain to rest position
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

const NUM_LINKS: usize = 8;
const LINK_SPACING: f32 = 1.5;

fn main() {
    let mut display = SimulatorDisplay::<Rgb565>::new(Size::new(640, 480));
    let output_settings = OutputSettingsBuilder::new().scale(1).build();
    let mut window = Window::new("Pendulum Chain - SPACE=kick R=reset ESC=exit", &output_settings);

    let mut engine = K3dengine::new(640, 480);
    engine.camera.set_position(Point3::new(0.0, 4.0, 18.0));
    engine.camera.set_target(Point3::new(0.0, 2.0, 0.0));

    let (vertices, faces, normals) = make_cube();

    // Physics world: links + anchor + floor
    let mut physics = PhysicsWorld::<16, 16>::new();
    physics.set_gravity(Vector3::new(0.0, -9.81, 0.0));
    physics.solver_iterations = 8;

    // Static anchor at the top
    let anchor = physics.add_body(
        RigidBody::new_static()
            .with_position(Vector3::new(0.0, 10.0, 0.0)),
    ).unwrap();

    let colors = [
        Rgb565::CSS_CYAN, Rgb565::CSS_ORANGE, Rgb565::CSS_LIME,
        Rgb565::CSS_MAGENTA, Rgb565::CSS_YELLOW, Rgb565::CSS_SALMON,
        Rgb565::CSS_LIGHT_BLUE, Rgb565::CSS_CORAL,
    ];

    let mut body_ids: Vec<BodyId> = Vec::new();
    let mut meshes: Vec<K3dMesh> = Vec::new();
    let mut initial_positions: Vec<Vector3<f32>> = Vec::new();

    let mut prev_id = anchor;
    for i in 0..NUM_LINKS {
        let y = 10.0 - (i as f32 + 1.0) * LINK_SPACING;
        let pos = Vector3::new(0.0, y, 0.0);
        let body = RigidBody::new(1.0)
            .with_position(pos)
            .with_damping(0.005)
            .with_collider(Collider::Sphere { radius: 0.3 })
            .with_inertia_sphere(0.3)
            .with_angular_damping(0.01)
            .with_restitution(0.3)
            .with_friction(0.5);
        let id = physics.add_body(body).unwrap();

        physics.add_distance_constraint(
            prev_id, Vector3::zeros(),
            id, Vector3::zeros(),
            0.0,
        ).unwrap();

        body_ids.push(id);
        initial_positions.push(pos);
        prev_id = id;

        let geometry = Geometry {
            vertices: &vertices, faces: &faces, colors: &[], lines: &[],
            normals: &normals, uvs: &[], texture_id: None,
        };
        let mut mesh = K3dMesh::new(geometry);
        mesh.set_render_mode(RenderMode::SolidLightDir(Vector3::new(0.5, 1.0, 0.3)));
        mesh.set_color(colors[i % colors.len()]);
        mesh.set_scale(0.6);
        mesh.set_position(pos.x, pos.y, pos.z);
        meshes.push(mesh);
    }

    // Floor
    physics.add_body(
        RigidBody::new_static()
            .with_position(Vector3::new(0.0, -0.1, 0.0))
            .with_collider(Collider::Aabb { half_extents: Vector3::new(10.0, 0.1, 10.0) })
            .with_restitution(0.5)
            .with_friction(0.6),
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
    let mut kick_dir = 1.0_f32;

    println!("Pendulum Chain Demo ({} links)", NUM_LINKS);
    println!("SPACE = kick bottom link, R = reset, ESC = exit");

    display.clear(Rgb565::BLACK).unwrap();
    window.update(&display);

    'running: loop {
        perf.start_of_frame();

        for event in window.events() {
            match event {
                SimulatorEvent::KeyDown { keycode, .. } => match keycode {
                    Keycode::Escape => break 'running,
                    Keycode::Space => {
                        // Kick the bottom link sideways
                        if let Some(&last) = body_ids.last() {
                            let body = physics.body_mut(last).unwrap();
                            if body.active {
                                body.apply_impulse(Vector3::new(kick_dir * 10.0, 3.0, 0.0));
                                kick_dir = -kick_dir;
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
        Text::new("SPACE=kick R=reset ESC=exit", Point::new(10, 470), text_style)
            .draw(&mut display).unwrap();

        window.update(&display);
        thread::sleep(Duration::from_millis(16));
    }
}
