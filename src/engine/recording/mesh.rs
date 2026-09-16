//! Per-mesh projection and primitive emission.

use crate::engine::K3dengine;
use crate::engine::immediate::render;
use crate::error::{BudgetKind, RenderError};
use crate::pipeline::assemble::primitive::DrawPrimitive;
use crate::pipeline::command_buffer::{CommandBuffer, RenderCommand};
use crate::pipeline::vertex::mesh::K3dMesh;
use crate::pipeline::vertex::transform::should_cull_mesh;

pub(crate) fn record_impl<'a, MS, const MAX: usize>(
    engine: &K3dengine,
    meshes: MS,
    commands: &mut CommandBuffer<MAX>,
    telemetry: Option<&mut crate::telemetry::RecordTelemetry>,
) -> Result<(), RenderError>
where
    MS: IntoIterator<Item = &'a K3dMesh<'a>>,
{
    commands.clear();
    commands.push(RenderCommand::ClearDepth(crate::Z_MAX_VALUE))?;
    if let Some(caps) = engine.caps {
        caps.validate_framebuffer(engine.width as usize, engine.height as usize)?;
    }

    let mut first_error = None;
    let mut visible_meshes = 0usize;
    let mut used_texture_ids: heapless::Vec<u32, 64> = heapless::Vec::new();
    let mut meshes_total = 0usize;

    #[cfg(feature = "record-sort")]
    {
        let mut sorted: heapless::Vec<(u8, i32, &K3dMesh<'a>), 256> = heapless::Vec::new();
        for mesh in meshes {
            meshes_total += 1;
            if mesh.geometry.vertices.is_empty() {
                continue;
            }
            if should_cull_mesh(&engine.camera, mesh) {
                continue;
            }
            let distance = (mesh.get_position() - engine.camera.position).norm();
            let dist_key = (distance * 1000.0) as i32;
            if sorted.push((mesh.priority, dist_key, mesh)).is_err() {
                break;
            }
        }
        sorted.sort_unstable_by(|a, b| b.0.cmp(&a.0).then_with(|| a.1.cmp(&b.1)));

        for &(_, _, mesh) in sorted.iter() {
            record_one_mesh(
                engine,
                mesh,
                commands,
                &mut first_error,
                &mut visible_meshes,
                &mut used_texture_ids,
            )?;
            if let Some(err) = first_error.take() {
                return Err(err);
            }
        }
    }

    #[cfg(not(feature = "record-sort"))]
    {
        for mesh in meshes {
            meshes_total += 1;
            if mesh.geometry.vertices.is_empty() {
                continue;
            }
            if should_cull_mesh(&engine.camera, mesh) {
                continue;
            }
            record_one_mesh(
                engine,
                mesh,
                commands,
                &mut first_error,
                &mut visible_meshes,
                &mut used_texture_ids,
            )?;
            if let Some(err) = first_error.take() {
                return Err(err);
            }
        }
    }

    if let Some(t) = telemetry {
        t.meshes_total = meshes_total;
        t.meshes_visible = visible_meshes;
        t.unique_textures = used_texture_ids.len();
        t.draw_commands = commands
            .iter()
            .filter(|cmd| matches!(cmd, RenderCommand::Draw(_)))
            .count();
        t.fallback_used = false;
        t.degradation_steps_applied = 0;
        t.dropped_meshes = 0;
    }

    Ok(())
}

pub(crate) fn record_one_mesh<'a, const MAX: usize>(
    engine: &K3dengine,
    mesh: &'a K3dMesh<'a>,
    commands: &mut CommandBuffer<MAX>,
    first_error: &mut Option<RenderError>,
    visible_meshes: &mut usize,
    used_texture_ids: &mut heapless::Vec<u32, 64>,
) -> Result<(), RenderError> {
    let distance = (mesh.get_position() - engine.camera.position).norm();
    let geometry = mesh.select_lod(distance);

    if let Some(caps) = engine.caps {
        *visible_meshes += 1;
        if *visible_meshes > caps.max_meshes_per_frame {
            return Err(RenderError::OutOfBudget(BudgetKind::MeshesPerFrame {
                attempted: *visible_meshes,
                max: caps.max_meshes_per_frame,
            }));
        }

        if geometry.vertices.len() > caps.max_vertices_per_mesh {
            return Err(RenderError::OutOfBudget(BudgetKind::VerticesPerMesh {
                attempted: geometry.vertices.len(),
                max: caps.max_vertices_per_mesh,
            }));
        }

        if geometry.faces.len() > caps.max_triangles_per_mesh {
            return Err(RenderError::OutOfBudget(BudgetKind::TrianglesPerMesh {
                attempted: geometry.faces.len(),
                max: caps.max_triangles_per_mesh,
            }));
        }

        if let Some(texture_id) = geometry.texture_id
            && !used_texture_ids.contains(&texture_id)
        {
            let attempted = used_texture_ids.len() + 1;
            if attempted > caps.max_textures {
                return Err(RenderError::OutOfBudget(BudgetKind::Textures {
                    attempted,
                    max: caps.max_textures,
                }));
            }

            if used_texture_ids.push(texture_id).is_err() {
                return Err(RenderError::OutOfBudget(BudgetKind::Textures {
                    attempted,
                    max: caps.max_textures,
                }));
            }
        }
    }

    let mut push_draw = |primitive: DrawPrimitive, err: &mut Option<RenderError>| {
        if err.is_none()
            && let Err(e) = commands.push(RenderCommand::Draw(primitive))
        {
            *err = Some(e);
        }
    };

    #[cfg(feature = "lod-crossfade")]
    {
        match mesh.select_lod_pick(distance) {
            crate::pipeline::vertex::mesh::LodPick::Single(_) => {
                mesh.lod_force.set(None);
                mesh.draw_alpha.set(None);
                render(engine, core::iter::once(mesh), |primitive| {
                    push_draw(primitive, first_error);
                });
            }
            crate::pipeline::vertex::mesh::LodPick::Crossfade { near, far, t } => {
                let near_lvl = mesh.lod_level_of(near);
                let far_lvl = mesh.lod_level_of(far);
                if t < 0.5 {
                    mesh.lod_force.set(Some(near_lvl));
                    mesh.draw_alpha.set(None);
                    render(engine, core::iter::once(mesh), |primitive| {
                        push_draw(primitive, first_error);
                    });
                    let a = (t * 255.0) as u8;
                    if a > 16 {
                        mesh.lod_force.set(Some(far_lvl));
                        mesh.draw_alpha.set(Some(a));
                        render(engine, core::iter::once(mesh), |primitive| {
                            push_draw(primitive, first_error);
                        });
                    }
                } else {
                    mesh.lod_force.set(Some(far_lvl));
                    mesh.draw_alpha.set(None);
                    render(engine, core::iter::once(mesh), |primitive| {
                        push_draw(primitive, first_error);
                    });
                    let a = ((1.0 - t) * 255.0) as u8;
                    if a > 16 {
                        mesh.lod_force.set(Some(near_lvl));
                        mesh.draw_alpha.set(Some(a));
                        render(engine, core::iter::once(mesh), |primitive| {
                            push_draw(primitive, first_error);
                        });
                    }
                }
                mesh.lod_force.set(None);
                mesh.draw_alpha.set(None);
            }
        }
    }
    #[cfg(not(feature = "lod-crossfade"))]
    {
        render(engine, core::iter::once(mesh), |primitive| {
            push_draw(primitive, first_error);
        });
    }
    Ok(())
}
