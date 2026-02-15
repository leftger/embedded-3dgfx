//! Newton's Cradle demo
//!
//! Classic desk toy: a row of hanging spheres where momentum transfers
//! through the chain on impact.
//!
//! Controls:
//! - SPACE: Pull and release the leftmost ball
//! - 2: Pull and release the two leftmost balls
//! - R: Reset to rest
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

const NUM_BALLS: usize = 5;
const BALL_SPACING: f32 = 1.2;
const STRING_LENGTH: f32 = 6.0;
const ANCHOR_Y: f32 = 12.0;

fn main() {
    let mut display = SimulatorDisplay::<Rgb565>::new(Size::new(640, 480));
    let output_settings = OutputSettingsBuilder::new().scale(1).build();
    let mut window = Window::new("Newton's Cradle - SPACE=1ball 2=2balls R=reset ESC=exit", &output_settings);

    let mut engine = K3dengine::new(640, 480);
    engine.camera.set_position(Point3::new(0.0, 6.0, 16.0));
    engine.camera.set_target(Point3::new(0.0, 5.0, 0.0));

    let (vertices, faces, normals) = make_cube();

    // Physics: 5 balls + 5 anchors + floor = 11 bodies, 5 constraints
    let mut physics = PhysicsWorld::<16, 8>::new();
    physics.set_gravity(Vector3::new(0.0, -9.81, 0.0));
    physics.solver_iterations = 12;

    let mut body_ids: Vec<BodyId> = Vec::new();
    let mut anchor_ids: Vec<BodyId> = Vec::new();
    let mut meshes: Vec<K3dMesh> = Vec::new();
    let mut rest_positions: Vec<Vector3<f32>> = Vec::new();

    let start_x = -((NUM_BALLS - 1) as f32) * BALL_SPACING / 2.0;

    let colors = [
        Rgb565::CSS_SILVER, Rgb565::CSS_LIGHT_GRAY, Rgb565::CSS_WHITE,
        Rgb565::CSS_LIGHT_GRAY, Rgb565::CSS_SILVER,
    ];

    for i in 0..NUM_BALLS {
        let x = start_x + i as f32 * BALL_SPACING;
        let ball_y = ANCHOR_Y - STRING_LENGTH;
        let pos = Vector3::new(x, ball_y, 0.0);

        // Static anchor above
        let anchor = physics.add_body(
            RigidBody::new_static()
                .with_position(Vector3::new(x, ANCHOR_Y, 0.0)),
        ).unwrap();
        anchor_ids.push(anchor);

        // Dynamic ball
        let body = RigidBody::new(1.0)
            .with_position(pos)
            .with_damping(0.001)
            .with_collider(Collider::Sphere { radius: 0.55 })
            .with_inertia_sphere(0.55)
            .with_angular_damping(0.01)
            .with_restitution(0.95)
            .with_friction(0.05);
        let id = physics.add_body(body).unwrap();

        // Distance constraint = pendulum string
        physics.add_distance_constraint(
            anchor, Vector3::zeros(),
            id, Vector3::zeros(),
            0.0,
        ).unwrap();

        body_ids.push(id);
        rest_positions.push(pos);

        let geometry = Geometry {
            vertices: &vertices, faces: &faces, colors: &[], lines: &[],
            normals: &normals, uvs: &[], texture_id: None,
        };
        let mut mesh = K3dMesh::new(geometry);
        mesh.set_render_mode(RenderMode::SolidLightDir(Vector3::new(0.5, 1.0, 0.3)));
        mesh.set_color(colors[i % colors.len()]);
        mesh.set_scale(1.0);
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

    println!("Newton's Cradle Demo ({} balls)", NUM_BALLS);
    println!("SPACE = pull 1 ball, 2 = pull 2 balls, R = reset, ESC = exit");

    display.clear(Rgb565::BLACK).unwrap();
    window.update(&display);

    'running: loop {
        perf.start_of_frame();

        for event in window.events() {
            match event {
                SimulatorEvent::KeyDown { keycode, .. } => match keycode {
                    Keycode::Escape => break 'running,
                    Keycode::Space => {
                        // Pull leftmost ball and release
                        let body = physics.body_mut(body_ids[0]).unwrap();
                        body.velocity = Vector3::new(-8.0, 2.0, 0.0);
                    }
                    Keycode::Num2 => {
                        // Pull two leftmost balls
                        for idx in 0..2.min(NUM_BALLS) {
                            let body = physics.body_mut(body_ids[idx]).unwrap();
                            body.velocity = Vector3::new(-8.0, 2.0, 0.0);
                        }
                    }
                    Keycode::R => {
                        for (i, &id) in body_ids.iter().enumerate() {
                            physics.set_active(id, true);
                            let body = physics.body_mut(id).unwrap();
                            body.position = rest_positions[i];
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

        physics.step_fixed::<16>(1.0 / 60.0, 6);

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
        Text::new("SPACE=1ball 2=2balls R=reset ESC=exit", Point::new(10, 470), text_style)
            .draw(&mut display).unwrap();

        window.update(&display);
        thread::sleep(Duration::from_millis(16));
    }
}
