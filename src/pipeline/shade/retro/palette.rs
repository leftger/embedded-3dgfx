//! Indexed palettes: RGB332 quantization and animated palette cycling.

use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::pixelcolor::RgbColor;

/// Palette quantization mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum PaletteMode {
    /// Leave colours untouched.
    Off,
    /// 3-3-2 bit palette approximation (256 colors).
    Rgb332,
}

impl PaletteMode {
    #[must_use]
    #[inline]
    pub fn apply(self, color: Rgb565) -> Rgb565 {
        match self {
            PaletteMode::Off => color,
            PaletteMode::Rgb332 => {
                let r3 = ((color.r() as u16 * 7 + 15) / 31) as u8;
                let g3 = ((color.g() as u16 * 7 + 31) / 63) as u8;
                let b2 = ((color.b() as u16 * 3 + 15) / 31) as u8;
                let r = (r3 as u16 * 31 / 7) as u8;
                let g = (g3 as u16 * 63 / 7) as u8;
                let b = (b2 as u16 * 31 / 3) as u8;
                Rgb565::new(r, g, b)
            }
        }
    }
}

/// Dynamic color palette with runtime cycling and animation support.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AnimatedPalette<const N: usize> {
    /// Base palette colors.
    pub colors: [Rgb565; N],
    /// Cycle start index (inclusive).
    pub cycle_start: usize,
    /// Cycle end index (exclusive).
    pub cycle_end: usize,
    /// Current cycle offset.
    pub offset: usize,
}

impl<const N: usize> AnimatedPalette<N> {
    /// Create a new animated palette from static colors.
    #[must_use]
    pub const fn new(colors: [Rgb565; N]) -> Self {
        Self {
            colors,
            cycle_start: 0,
            cycle_end: N,
            offset: 0,
        }
    }

    /// Constrain palette cycling to a sub-range [start, end).
    #[must_use]
    pub const fn with_cycle_range(mut self, start: usize, end: usize) -> Self {
        self.cycle_start = start;
        self.cycle_end = if end <= N { end } else { N };
        self
    }

    /// Step the palette cycle by `steps` positions.
    pub fn step(&mut self, steps: usize) {
        let range_len = self.cycle_end.saturating_sub(self.cycle_start);
        if range_len > 1 {
            self.offset = (self.offset + steps) % range_len;
        }
    }

    /// Look up a color by palette index, applying the active animation offset if within cycle range.
    #[must_use]
    #[inline]
    pub fn get_color(&self, index: usize) -> Rgb565 {
        if index >= N {
            return Rgb565::BLACK;
        }
        if index >= self.cycle_start && index < self.cycle_end {
            let range_len = self.cycle_end - self.cycle_start;
            let cycled_idx =
                self.cycle_start + (index - self.cycle_start + self.offset) % range_len;
            self.colors[cycled_idx]
        } else {
            self.colors[index]
        }
    }
}

/// Cycling direction for retro animated palette slices.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CycleDirection {
    /// Forward looping cycle (0 -> 1 -> 2 -> ... -> N-1 -> 0).
    #[default]
    Forward,
    /// Reverse looping cycle (N-1 -> N-2 -> ... -> 0 -> N-1).
    Reverse,
    /// Ping-pong alternating cycle (0 -> 1 -> 2 -> 1 -> 0).
    PingPong,
}

/// A cycling slice configuration within an indexed color palette.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PaletteSlice {
    /// Start index in palette (inclusive).
    pub start: usize,
    /// End index in palette (exclusive).
    pub end: usize,
    /// Cycling rate in cycles/steps per second (Hz).
    pub rate_hz: f32,
    /// Cycling direction.
    pub direction: CycleDirection,
    /// Accumulated time in seconds.
    pub timer: f32,
    /// Current step offset within the slice.
    pub current_step: usize,
    /// Ping-pong traversal state (true = forward, false = backward).
    pub ping_pong_forward: bool,
}

impl PaletteSlice {
    /// Create a new cycling palette slice.
    #[must_use]
    pub const fn new(start: usize, end: usize, rate_hz: f32, direction: CycleDirection) -> Self {
        Self {
            start,
            end,
            rate_hz,
            direction,
            timer: 0.0,
            current_step: 0,
            ping_pong_forward: true,
        }
    }

    /// Advance time by `dt` seconds. Returns `true` if the cycle stepped.
    #[must_use]
    pub fn advance(&mut self, dt: f32) -> bool {
        let span = self.end.saturating_sub(self.start);
        if span <= 1 || self.rate_hz <= 0.0 {
            return false;
        }

        self.timer += dt;
        let period = 1.0 / self.rate_hz;
        let mut changed = false;

        while self.timer >= period {
            self.timer -= period;
            changed = true;

            match self.direction {
                CycleDirection::Forward => {
                    self.current_step = (self.current_step + 1) % span;
                }
                CycleDirection::Reverse => {
                    self.current_step = if self.current_step == 0 {
                        span - 1
                    } else {
                        self.current_step - 1
                    };
                }
                CycleDirection::PingPong => {
                    if self.ping_pong_forward {
                        if self.current_step + 1 >= span {
                            self.ping_pong_forward = false;
                            self.current_step = self.current_step.saturating_sub(1);
                        } else {
                            self.current_step += 1;
                        }
                    } else if self.current_step == 0 {
                        self.ping_pong_forward = true;
                        self.current_step = 1.min(span - 1);
                    } else {
                        self.current_step -= 1;
                    }
                }
            }
        }

        changed
    }

    /// Maps an indexed palette color index through this slice's current animation step.
    #[must_use]
    #[inline]
    pub fn map_index(&self, index: usize) -> usize {
        let span = self.end.saturating_sub(self.start);
        if span <= 1 || index < self.start || index >= self.end {
            return index;
        }
        self.start + (index - self.start + self.current_step) % span
    }
}

/// Multi-slice palette cycler for zero-allocation, zero-geometry retro animations.
///
/// Cycles indexed color ranges in real time (e.g. water cascades, fire glows, force field pulses,
/// and neon flashing) by shifting indices or updating palette tables without modifying mesh buffers.
#[derive(Debug, Clone, Copy)]
pub struct PaletteCycler<const MAX_SLICES: usize = 4> {
    /// Active animation slices.
    pub slices: [Option<PaletteSlice>; MAX_SLICES],
    /// Number of configured slices.
    pub slice_count: usize,
}

impl<const MAX_SLICES: usize> Default for PaletteCycler<MAX_SLICES> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const MAX_SLICES: usize> PaletteCycler<MAX_SLICES> {
    /// Create a new empty palette cycler.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            slices: [None; MAX_SLICES],
            slice_count: 0,
        }
    }

    /// Add an animated slice. Returns `true` if added successfully, `false` if full.
    pub fn add_slice(&mut self, slice: PaletteSlice) -> bool {
        if self.slice_count < MAX_SLICES {
            self.slices[self.slice_count] = Some(slice);
            self.slice_count += 1;
            true
        } else {
            false
        }
    }

    /// Add a slice by specifying parameters directly.
    pub fn add_range(
        &mut self,
        start: usize,
        end: usize,
        rate_hz: f32,
        direction: CycleDirection,
    ) -> bool {
        self.add_slice(PaletteSlice::new(start, end, rate_hz, direction))
    }

    /// Advance all slices by delta time `dt` in seconds.
    ///
    /// Returns `true` if any slice advanced a step (indicating palette redraw/sync is needed).
    #[must_use]
    pub fn advance(&mut self, dt: f32) -> bool {
        let mut any_changed = false;
        for slice in self.slices.iter_mut().take(self.slice_count).flatten() {
            if slice.advance(dt) {
                any_changed = true;
            }
        }
        any_changed
    }

    /// Map an index through all active slices.
    #[must_use]
    #[inline]
    pub fn map_index(&self, index: usize) -> usize {
        let mut mapped = index;
        for slice in self.slices.iter().take(self.slice_count).flatten() {
            if index >= slice.start && index < slice.end {
                mapped = slice.map_index(index);
                break;
            }
        }
        mapped
    }

    /// Read colors from `base` palette and write cycled colors into `dest` palette.
    pub fn cycle_palette<const N: usize>(&self, base: &[Rgb565; N], dest: &mut [Rgb565; N]) {
        for i in 0..N {
            dest[i] = base[self.map_index(i)];
        }
    }

    /// Cycle slice in-place using a small temporary buffer.
    pub fn cycle_slice(&self, base: &[Rgb565], dest: &mut [Rgb565]) {
        let count = base.len().min(dest.len());
        for i in 0..count {
            dest[i] = base[self.map_index(i)];
        }
    }
}
