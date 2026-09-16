//! # Application loop integration — own the loop, not the engine
//!
//! The engine has no `run()`. You own the loop, and you decide where the
//! boundaries are. This example shows the structure most applications want:
//!
//! 1. **Input**      — drain events, never block the renderer on them.
//! 2. **Simulation** — fixed timestep (`1/60 s`), decoupled from frame rate.
//! 3. **Record**     — walk the scene into a `CommandBuffer`, with a
//!    degradation policy so a heavy frame sheds work instead of failing.
//! 4. **Execute**    — replay the commands into your `DrawTarget`.
//! 5. **Overlay**    — HUD text *and* a hand-rolled pass drawn with the very
//!    same `RasterState` the engine used, so effects line up exactly.
//!
//! ```text
//! cargo run --example integration_app_loop --features std
//! ```
//!
//! Controls: `SPACE` pause · `H` toggle HUD · `R` toggle camera orbit · `ESC` quit.

// The whole frame loop — driver, buffers, raster state, depth, primitives —
// comes from the prelude. Only the policy knobs and telemetry are named
// explicitly, because they are optional extras rather than everyday types.
use embedded_3dgfx::config::{DegradationPolicy, DegradationStep};
use embedded_3dgfx::prelude::*;
use embedded_3dgfx::telemetry::{ExecuteTelemetry, RecordTelemetry};

use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::{Rgb565, RgbColor, WebColors},
    prelude::*,
    text::Text,
};
use embedded_graphics_simulator::{
    OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent, Window, sdl2::Keycode,
};
use nalgebra::{Point2, Point3, UnitQuaternion, Vector3};
use std::thread;
use std::time::{Duration, Instant};

// The default `PROFILE_M33_BALANCED` caps a frame at 320x240, so this example
// sits exactly on the limit. For a larger frame add the `desktop-unbounded`
// feature at compile time, or set `EMBEDDED_3DGFX_CAPS=off` at run time — see
// `docs/caps-and-telemetry.md`.
const WIDTH: usize = 320;
const HEIGHT: usize = 240;
const FIXED_DT: f32 = 1.0 / 60.0;
const COMMAND_CAPACITY: usize = 8192;

/// Applied when a frame would exceed the profile's primitive budget.
///
/// First halve the mesh detail, then drop a quality tier. Both are no-ops on a
/// frame that fits, so this can stay on permanently — that is the point.
static DEGRADATION: [DegradationStep; 2] = [
    DegradationStep::MeshDecimationStride(2),
    DegradationStep::DowngradeQuality,
];

fn cube() -> (Vec<[f32; 3]>, Vec<[usize; 3]>) {
    let vertices = vec![
        [-1.0, -1.0, 1.0],
        [1.0, -1.0, 1.0],
        [1.0, 1.0, 1.0],
        [-1.0, 1.0, 1.0],
        [-1.0, -1.0, -1.0],
        [1.0, -1.0, -1.0],
        [1.0, 1.0, -1.0],
        [-1.0, 1.0, -1.0],
    ];
    let faces = vec![
        [0, 1, 2],
        [0, 2, 3],
        [5, 4, 7],
        [5, 7, 6],
        [3, 2, 6],
        [3, 6, 7],
        [4, 5, 1],
        [4, 1, 0],
        [1, 5, 6],
        [1, 6, 2],
        [4, 0, 3],
        [4, 3, 7],
    ];
    (vertices, faces)
}

/// A flat quad on the XZ plane, used as a ground plane.
fn ground() -> (Vec<[f32; 3]>, Vec<[usize; 3]>) {
    let vertices = vec![
        [-6.0, 0.0, -6.0],
        [6.0, 0.0, -6.0],
        [6.0, 0.0, 6.0],
        [-6.0, 0.0, 6.0],
    ];
    let faces = vec![[0, 1, 2], [0, 2, 3]];
    (vertices, faces)
}

fn main() {
    // ── Resources you own ───────────────────────────────────────────────────
    let mut display = SimulatorDisplay::<Rgb565>::new(Size::new(WIDTH as u32, HEIGHT as u32));
    let mut zbuffer = vec![Z_MAX_VALUE; WIDTH * HEIGHT];
    let mut commands = CommandBuffer::<COMMAND_CAPACITY>::new();

    // ── Engine ──────────────────────────────────────────────────────────────
    let mut engine = K3dengine::new(WIDTH as u16, HEIGHT as u16);
    apply_default_caps(&mut engine);

    // ── Scene ───────────────────────────────────────────────────────────────
    let (ground_v, ground_f) = ground();
    let (cube_v, cube_f) = cube();

    let ground_geometry = Geometry::new(&ground_v, &ground_f);
    let cube_geometry = Geometry::new(&cube_v, &cube_f);

    let mut ground_mesh = K3dMesh::new(ground_geometry);
    ground_mesh.set_color(Rgb565::CSS_DARK_SLATE_GRAY);
    ground_mesh.set_render_mode(RenderMode::Solid);

    let mut cube_mesh = K3dMesh::new(cube_geometry);
    cube_mesh.set_color(Rgb565::CSS_ORANGE);
    cube_mesh.set_render_mode(RenderMode::Solid);
    cube_mesh.set_position(0.0, 1.0, 0.0);

    // ── Diagnostics you can surface however you like ────────────────────────
    let mut record_telemetry = RecordTelemetry::default();
    let mut execute_telemetry = ExecuteTelemetry::default();

    // ── Window ──────────────────────────────────────────────────────────────
    let output_settings = OutputSettingsBuilder::new().scale(1).build();
    let mut window = Window::new("Integration: application loop", &output_settings);

    println!("SPACE pause · H HUD · R orbit · ESC quit");

    // ── Loop state ──────────────────────────────────────────────────────────
    let mut last_frame = Instant::now();
    let mut accumulator = 0.0_f32;
    let mut sim_time = 0.0_f32;
    let mut paused = false;
    let mut show_hud = true;
    let mut orbit = true;
    let mut frames = 0_u64;
    let mut degraded_frames = 0_u64;
    let mut fps = 0.0_f32;
    let mut fps_mark = Instant::now();
    let mut frames_since_mark = 0_u32;

    // Optional headless run: `E3DGFX_EXAMPLE_FRAMES=120 cargo run ...` exits
    // after that many frames, which is handy for CI smoke tests.
    let frame_limit: Option<u64> = std::env::var("E3DGFX_EXAMPLE_FRAMES")
        .ok()
        .and_then(|v| v.parse().ok());

    // `Window::events()` panics until `update()` has been called once, so do a
    // first present before entering the loop.
    window.update(&display);

    'running: loop {
        // ── 1. Input ────────────────────────────────────────────────────────
        for event in window.events() {
            match event {
                SimulatorEvent::KeyDown { keycode, .. } => match keycode {
                    Keycode::Escape => break 'running,
                    Keycode::Space => paused = !paused,
                    Keycode::H => show_hud = !show_hud,
                    Keycode::R => orbit = !orbit,
                    _ => {}
                },
                SimulatorEvent::Quit => break 'running,
                _ => {}
            }
        }

        if frame_limit.is_some_and(|limit| frames >= limit) {
            break 'running;
        }

        // ── 2. Simulation, at a fixed timestep ──────────────────────────────
        let now = Instant::now();
        let dt = (now - last_frame).as_secs_f32().min(0.25); // clamp after a stall
        last_frame = now;

        if !paused {
            accumulator += dt;
            while accumulator >= FIXED_DT {
                sim_time += FIXED_DT;
                accumulator -= FIXED_DT;
            }
        }

        // Animation reads `sim_time`, never `dt`, so it is frame-rate independent.
        cube_mesh.set_rotation(UnitQuaternion::from_axis_angle(
            &Vector3::y_axis(),
            sim_time * 0.8,
        ));

        let angle = if orbit { sim_time * 0.5 } else { 0.75 };
        let radius = 8.0_f32;
        engine
            .camera
            .set_position(Point3::new(radius * angle.sin(), 3.5, radius * angle.cos()));
        engine.camera.set_target(Point3::new(0.0, 0.5, 0.0));

        // ── 3. Record, with a degradation policy ────────────────────────────
        display.clear(Rgb565::BLACK).unwrap();
        zbuffer.fill(Z_MAX_VALUE);

        let meshes = [&ground_mesh, &cube_mesh];
        let policy = DegradationPolicy {
            steps: &DEGRADATION,
        };
        let outcome = engine
            .record_with_degradation(&meshes, &mut commands, policy, Some(&mut record_telemetry))
            .expect("recording failed");

        if outcome.used_degradation {
            degraded_frames += 1;
        }

        // ── 4. Execute into the framebuffer ─────────────────────────────────
        {
            let mut frame = FrameCtx {
                zbuffer: &mut zbuffer,
                width: WIDTH,
                height: HEIGHT,
            };
            engine
                .execute::<_, COMMAND_CAPACITY>(
                    &mut display,
                    &mut frame,
                    &commands,
                    Some(&mut execute_telemetry),
                )
                .expect("execute failed");
        }

        // ── 5a. A hand-rolled pass that respects the engine's effects ───────
        // `raster_state` is the engine's own per-pass configuration: fog,
        // dither, tint, palette, stipple, depth bias. Anything you draw with it
        // is pixel-identical to what the engine would have drawn.
        {
            let state = engine.raster_state(WIDTH, HEIGHT);
            let (cx, cy) = (WIDTH as i32 / 2, HEIGHT as i32 / 2 + 40);
            for dx in -8..=8 {
                draw_zbuffered_with_state(
                    DrawPrimitive::ColoredPoint(Point2::new(cx + dx, cy), Rgb565::CSS_SPRING_GREEN),
                    &mut display,
                    &mut zbuffer,
                    &state,
                );
            }
        }

        // ── 5b. HUD text on top ─────────────────────────────────────────────
        frames += 1;
        frames_since_mark += 1;
        if fps_mark.elapsed() >= Duration::from_millis(250) {
            fps = frames_since_mark as f32 / fps_mark.elapsed().as_secs_f32();
            frames_since_mark = 0;
            fps_mark = Instant::now();
        }

        if show_hud {
            let hud = format!(
                "{fps:5.1} fps  frame {frames}\n\
                 visible {}/{} meshes  {} cmds  {} degraded",
                record_telemetry.meshes_visible,
                record_telemetry.meshes_total,
                execute_telemetry.commands_total,
                degraded_frames,
            );
            let style = MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_WHITE);
            let _ = Text::new(&hud, Point::new(4, 10), style).draw(&mut display);
        }

        window.update(&display);

        // A real application paces itself against vsync (or a swap chain
        // `try_present`); this just keeps a desktop core from spinning hot.
        thread::sleep(Duration::from_millis(1));
    }

    println!("presented {frames} frames ({degraded_frames} degraded)");
}
