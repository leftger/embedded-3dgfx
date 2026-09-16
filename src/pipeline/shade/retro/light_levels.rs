//! Sector brightness models.

/// Sector brightness model.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum LightLevels {
    /// Direct linear brightness scale.
    Linear,
    /// Doom-like 32-step distance banding.
    Doom32,
}
