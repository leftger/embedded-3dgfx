//! Budget fallback and degradation-policy recording paths.

use super::mesh::record_impl;
use crate::config::{DegradationPolicy, DegradationStep, QualityTier};
use crate::engine::K3dengine;
use crate::error::{BudgetKind, RecoveryAction, RenderError, RuntimeFaultKind};
use crate::pipeline::command_buffer::CommandBuffer;
use crate::pipeline::vertex::mesh::K3dMesh;

pub(crate) fn record_with_fallback<'a, MS, FS, const MAX: usize>(
    engine: &K3dengine,
    primary: MS,
    fallback: FS,
    commands: &mut CommandBuffer<MAX>,
    telemetry: Option<&mut crate::telemetry::RecordTelemetry>,
) -> Result<crate::engine::BudgetFallbackOutcome, RenderError>
where
    MS: IntoIterator<Item = &'a K3dMesh<'a>>,
    FS: IntoIterator<Item = &'a K3dMesh<'a>>,
{
    let mut local_telemetry = crate::telemetry::RecordTelemetry::default();
    match record_impl(engine, primary, commands, Some(&mut local_telemetry)) {
        Ok(()) => {
            if let Some(t) = telemetry {
                *t = local_telemetry;
                t.fallback_used = false;
            }
            Ok(crate::engine::BudgetFallbackOutcome {
                used_fallback: false,
                primary_budget_error: None,
            })
        }
        Err(RenderError::OutOfBudget(kind)) => {
            let mut fallback_telemetry = crate::telemetry::RecordTelemetry::default();
            record_impl(engine, fallback, commands, Some(&mut fallback_telemetry))?;
            if let Some(t) = telemetry {
                *t = fallback_telemetry;
                t.fallback_used = true;
            }
            Ok(crate::engine::BudgetFallbackOutcome {
                used_fallback: true,
                primary_budget_error: Some(kind),
            })
        }
        Err(e) => Err(e),
    }
}

pub(super) fn downgraded_quality_tier(tier: QualityTier) -> QualityTier {
    match tier {
        QualityTier::Quality => QualityTier::Balanced,
        QualityTier::Balanced => QualityTier::Fastest,
        QualityTier::Fastest => QualityTier::Fastest,
    }
}

pub(crate) fn record_with_degradation<'a, const MAX: usize>(
    engine: &mut K3dengine,
    meshes: &[&'a K3dMesh<'a>],
    commands: &mut CommandBuffer<MAX>,
    policy: DegradationPolicy<'_>,
    telemetry: Option<&mut crate::telemetry::RecordTelemetry>,
) -> Result<crate::engine::DegradationOutcome, RenderError> {
    let original_quality = engine.quality_tier;
    let mut active_quality = engine.quality_tier;

    let mut outcome = crate::engine::DegradationOutcome {
        used_degradation: false,
        steps_applied: 0,
        dropped_meshes: 0,
        final_quality_tier: active_quality,
        primary_budget_error: None,
    };

    let mut local_telemetry = crate::telemetry::RecordTelemetry::default();
    match record_impl(
        engine,
        meshes.iter().copied(),
        commands,
        Some(&mut local_telemetry),
    ) {
        Ok(()) => {
            if let Some(t) = telemetry {
                *t = local_telemetry;
            }
            return Ok(outcome);
        }
        Err(RenderError::OutOfBudget(kind)) => {
            outcome.primary_budget_error = Some(kind);
        }
        Err(e) => return Err(e),
    }

    for step in policy.steps {
        outcome.used_degradation = true;
        outcome.steps_applied += 1;

        let mut selected: heapless::Vec<&K3dMesh<'_>, 512> = heapless::Vec::new();
        match *step {
            DegradationStep::RaisePriorityFloor(min_priority) => {
                for mesh in meshes {
                    if mesh.priority >= min_priority {
                        let _ = selected.push(*mesh);
                    } else {
                        outcome.dropped_meshes += 1;
                    }
                }
            }
            DegradationStep::MeshDecimationStride(stride) => {
                if stride == 0 {
                    engine.quality_tier = original_quality;
                    return Err(RenderError::InvalidInput(
                        "mesh decimation stride must be >= 1",
                    ));
                }
                for (idx, mesh) in meshes.iter().enumerate() {
                    if idx % stride == 0 {
                        let _ = selected.push(*mesh);
                    } else {
                        outcome.dropped_meshes += 1;
                    }
                }
            }
            DegradationStep::DowngradeQuality => {
                active_quality = downgraded_quality_tier(active_quality);
                engine.quality_tier = active_quality;
                for mesh in meshes {
                    let _ = selected.push(*mesh);
                }
            }
        }

        if selected.is_empty() {
            continue;
        }

        let mut step_telemetry = crate::telemetry::RecordTelemetry::default();
        let attempt = record_impl(
            engine,
            selected.iter().copied(),
            commands,
            Some(&mut step_telemetry),
        );

        if let Ok(()) = attempt {
            outcome.final_quality_tier = engine.quality_tier;
            if let Some(t) = telemetry {
                *t = step_telemetry;
                t.fallback_used = true;
                t.degradation_steps_applied = outcome.steps_applied;
                t.dropped_meshes = outcome.dropped_meshes;
            }
            engine.quality_tier = original_quality;
            return Ok(outcome);
        }
    }

    engine.quality_tier = original_quality;
    Err(RenderError::Recoverable {
        fault: RuntimeFaultKind::Budget(outcome.primary_budget_error.unwrap_or(
            BudgetKind::DrawPrimitives {
                attempted: commands.len(),
                max: MAX,
            },
        )),
        action: RecoveryAction::SkipFrame,
    })
}
