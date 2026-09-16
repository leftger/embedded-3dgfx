//! Screen-space stipple (screen-door) transparency mode.

/// Screen-space stipple transparency mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum StippleMode {
    /// No stippling; every covered pixel is written.
    Off,
    /// 1-in-4 checkerboard discard for fake translucency.
    Checkerboard,
}
