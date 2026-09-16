//! Scene traversal that records meshes into a command buffer.

use embedded_graphics_core::pixelcolor::Rgb565;
use nalgebra::Point3;

use super::K3dengine;
use crate::error::RenderError;
use crate::pipeline::assemble::primitive::DrawPrimitive;
use crate::pipeline::command_buffer::{CommandBuffer, RenderCommand};
use crate::pipeline::vertex::mesh::K3dMesh;
use crate::pipeline::vertex::transform::transform_point_with_w;
use mesh::record_impl;

pub(super) mod degrade;
pub(super) mod execute;
#[cfg(feature = "gizmos")]
pub(super) mod gizmos;
pub(super) mod mesh;
pub(crate) fn record<'a, MS, const MAX: usize>(
    engine: &K3dengine,
    meshes: MS,
    commands: &mut CommandBuffer<MAX>,
    telemetry: Option<&mut crate::telemetry::RecordTelemetry>,
) -> Result<(), RenderError>
where
    MS: IntoIterator<Item = &'a K3dMesh<'a>>,
{
    record_impl(engine, meshes, commands, telemetry)
}

pub(crate) fn record_drop_shadow<const MAX: usize>(
    engine: &K3dengine,
    mesh: &K3dMesh,
    floor_y: f32,
    shadow_radius: f32,
    max_fade_distance: f32,
    shadow_opacity: u8,
    color: Rgb565,
    commands: &mut CommandBuffer<MAX>,
) -> Result<(), RenderError> {
    let pos = mesh.get_position();
    let height = pos.y - floor_y;
    if height < 0.0 || height >= max_fade_distance {
        return Ok(());
    }

    let fade = 1.0 - (height / max_fade_distance).clamp(0.0, 1.0);
    let radius = shadow_radius * fade;
    let opacity = (shadow_opacity as f32 * fade) as u8;
    if opacity == 0 {
        return Ok(());
    }

    let y_pos = floor_y + 0.01;
    let center_world = [pos.x, y_pos, pos.z];
    let center_proj = transform_point_with_w(
        &engine.camera,
        engine.width,
        engine.height,
        &center_world,
        engine.camera.vp_matrix,
    );
    let Some((c_pt, _c_w)) = center_proj else {
        return Ok(());
    };

    let mut outer_proj: [Option<(Point3<i32>, f32)>; 8] = [None; 8];
    for i in 0..8 {
        let angle = (i as f32) * (core::f32::consts::PI / 4.0);
        let px = pos.x + radius * micromath::F32Ext::cos(angle);
        let pz = pos.z + radius * micromath::F32Ext::sin(angle);
        outer_proj[i] = transform_point_with_w(
            &engine.camera,
            engine.width,
            engine.height,
            &[px, y_pos, pz],
            engine.camera.vp_matrix,
        );
    }

    for i in 0..8 {
        let next_idx = (i + 1) % 8;
        if let (Some((p1, _w1)), Some((p2, _w2))) = (outer_proj[i], outer_proj[next_idx]) {
            commands.push(RenderCommand::Draw(
                DrawPrimitive::TranslucentTriangleWithDepth {
                    points: [c_pt.xy(), p1.xy(), p2.xy()],
                    depths: [c_pt.z as f32, p1.z as f32, p2.z as f32],
                    color,
                    alpha: opacity,
                },
            ))?;
        }
    }

    Ok(())
}

#[cfg(all(test, feature = "gizmos"))]
mod tests {
    #[cfg(feature = "gizmos")]
    use super::gizmos::{record_aabb_gizmo, record_frustum_gizmo};
    use super::*;
    use crate::pipeline::vertex::bounds::Aabb;
    use embedded_graphics_core::pixelcolor::RgbColor;
    use nalgebra::{Matrix4, Point3, Vector3};

    #[test]
    fn test_record_aabb_gizmo() {
        let mut engine = K3dengine::new(240, 240);
        engine.camera.set_position(Point3::new(0.0, 0.0, -10.0));
        engine.camera.set_target(Point3::new(0.0, 0.0, 0.0));
        let aabb = Aabb::from_min_max(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
        let mut commands = CommandBuffer::<32>::new();

        let res = record_aabb_gizmo(
            &engine,
            &aabb,
            &Matrix4::identity(),
            Rgb565::RED,
            &mut commands,
        );
        assert!(res.is_ok());
        // 12 edges for an AABB wireframe
        assert_eq!(commands.len(), 12);
    }

    #[test]
    fn test_record_frustum_gizmo() {
        let engine = K3dengine::new(240, 240);
        let mut commands = CommandBuffer::<32>::new();

        let res = record_frustum_gizmo(&engine, Rgb565::GREEN, &mut commands);
        assert!(res.is_ok());
        assert!(!commands.is_empty());
    }

    #[test]
    fn test_record_aabb_gizmo_buffer_overflow() {
        let mut engine = K3dengine::new(240, 240);
        engine.camera.set_position(Point3::new(0.0, 0.0, -10.0));
        engine.camera.set_target(Point3::new(0.0, 0.0, 0.0));
        let aabb = Aabb::from_min_max(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(1.0, 1.0, 1.0));
        // Buffer capacity of only 4 commands, but 12 are needed
        let mut small_commands = CommandBuffer::<4>::new();

        let res = record_aabb_gizmo(
            &engine,
            &aabb,
            &Matrix4::identity(),
            Rgb565::RED,
            &mut small_commands,
        );
        assert!(res.is_err());
    }
}
