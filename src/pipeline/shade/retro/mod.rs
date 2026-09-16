//! Retro / low-fidelity rendering controls.
//!
//! This used to be one flat `retro.rs` holding every low-fi concern. It is now
//! split by concern so each one is discoverable:
//!
//! * [`palette`] — RGB332 quantization and animated palette cycling
//! * [`sky`] — procedural sky gradient
//! * [`tint`] — full-screen tint blend
//! * [`stipple`] — screen-door transparency mode
//! * [`texture_lod`] — UV mapping style and distance LOD
//! * [`light_levels`] — sector brightness model
//! * [`style`] — [`RetroStyle`] preset bundle tying them together
//!
//! Every public item is re-exported here, so the historical
//! `embedded_3dgfx::pipeline::shade::retro::PaletteMode`-style paths keep resolving.

pub mod light_levels;
pub mod palette;
pub mod sky;
pub mod stipple;
pub mod style;
pub mod texture_lod;
pub mod tint;

pub use light_levels::LightLevels;
pub use palette::{AnimatedPalette, CycleDirection, PaletteCycler, PaletteMode, PaletteSlice};
pub use sky::SkyConfig;
pub use stipple::StippleMode;
pub use style::RetroStyle;
pub use texture_lod::{TextureLodConfig, TextureMapping};
pub use tint::ScreenTint;

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};

    #[test]
    fn palette_off_is_identity() {
        let c = Rgb565::new(17, 45, 23);
        assert_eq!(PaletteMode::Off.apply(c), c);
    }

    #[test]
    fn palette_rgb332_quantizes_channels() {
        let c = Rgb565::new(17, 45, 23);
        let q = PaletteMode::Rgb332.apply(c);
        // Blue channel is reduced to 2 bits; red/green map to 3 bits.
        assert_eq!(q, Rgb565::new(17, 45, 20));
    }

    #[test]
    fn screen_tint_strength_extremes() {
        let base = Rgb565::new(7, 20, 5);
        let tint = Rgb565::new(31, 0, 31);

        let off = ScreenTint {
            color: tint,
            strength: 0,
        };
        assert_eq!(off.apply(base), base);

        let full = ScreenTint {
            color: tint,
            strength: 255,
        };
        assert_eq!(full.apply(base), tint);
    }

    #[test]
    fn doom_walkable_preset_matches_expected_profile() {
        let s = RetroStyle::doom_walkable();
        assert!(s.fog.is_none());
        assert_eq!(s.vertex_snap_bits, 0);
        assert_eq!(s.texture_mapping, TextureMapping::Affine);
        assert_eq!(s.light_levels, LightLevels::Doom32);
        assert_eq!(s.stipple_mode, StippleMode::Off);
        assert_eq!(s.palette_mode, PaletteMode::Rgb332);
        assert!(s.sky.is_some());
        assert!(s.dither.is_some());
    }

    #[test]
    fn psx_preset_enables_snap_and_fog() {
        let s = RetroStyle::psx();
        assert_eq!(s.vertex_snap_bits, 6);
        assert_eq!(s.texture_mapping, TextureMapping::Affine);
        assert_eq!(s.light_levels, LightLevels::Linear);
        assert_eq!(s.palette_mode, PaletteMode::Rgb332);
        assert!(s.fog.is_some());
        assert!(s.dither.is_some());
        assert!(s.sky.is_some());
    }

    #[test]
    fn modern_is_the_default_and_effect_free() {
        let s = RetroStyle::default();
        assert!(s.fog.is_none());
        assert!(s.dither.is_none());
        assert!(s.screen_tint.is_none());
        assert_eq!(s.palette_mode, PaletteMode::Off);
        assert_eq!(s.stipple_mode, StippleMode::Off);
        assert_eq!(s.vertex_snap_bits, 0);
    }

    #[test]
    fn test_texture_lod_config() {
        let lod = TextureLodConfig::new(50.0, 150.0, Rgb565::new(16, 32, 16));
        assert!(!lod.should_drop_texture(40.0));
        assert!(!lod.should_drop_texture(100.0));
        assert!(lod.should_drop_texture(150.0));
        assert!(lod.should_drop_texture(200.0));

        assert!(!lod.is_in_transition(40.0));
        assert!(lod.is_in_transition(100.0));
        assert!(!lod.is_in_transition(150.0));

        assert_eq!(lod.flat_blend_factor(50.0), 0.0);
        assert_eq!(lod.flat_blend_factor(100.0), 0.5);
        assert_eq!(lod.flat_blend_factor(150.0), 1.0);
    }

    #[test]
    fn test_animated_palette() {
        let colors = [Rgb565::RED, Rgb565::GREEN, Rgb565::BLUE, Rgb565::WHITE];
        let mut pal = AnimatedPalette::new(colors).with_cycle_range(1, 3);

        assert_eq!(pal.get_color(0), Rgb565::RED);
        assert_eq!(pal.get_color(1), Rgb565::GREEN);
        assert_eq!(pal.get_color(2), Rgb565::BLUE);
        assert_eq!(pal.get_color(3), Rgb565::WHITE);

        pal.step(1);
        assert_eq!(pal.get_color(0), Rgb565::RED); // Uncycled
        assert_eq!(pal.get_color(1), Rgb565::BLUE); // Cycled
        assert_eq!(pal.get_color(2), Rgb565::GREEN); // Cycled
        assert_eq!(pal.get_color(3), Rgb565::WHITE); // Uncycled
    }

    #[test]
    fn test_palette_cycler_forward_and_reverse() {
        let mut cycler = PaletteCycler::<2>::new();
        // Forward water cycle: indices [2..5] at 10 Hz
        cycler.add_range(2, 5, 10.0, CycleDirection::Forward);

        assert_eq!(cycler.map_index(0), 0);
        assert_eq!(cycler.map_index(2), 2);
        assert_eq!(cycler.map_index(3), 3);
        assert_eq!(cycler.map_index(4), 4);
        assert_eq!(cycler.map_index(5), 5);

        // Advance 0.1s -> 1 step
        assert!(cycler.advance(0.1));
        assert_eq!(cycler.map_index(2), 3);
        assert_eq!(cycler.map_index(3), 4);
        assert_eq!(cycler.map_index(4), 2);

        // Reverse fire cycle
        let mut rev_slice = PaletteSlice::new(0, 3, 5.0, CycleDirection::Reverse);
        assert_eq!(rev_slice.current_step, 0);
        let _ = rev_slice.advance(0.2); // 1 step backward
        assert_eq!(rev_slice.current_step, 2);
    }

    #[test]
    fn test_palette_cycler_ping_pong() {
        let mut slice = PaletteSlice::new(0, 4, 10.0, CycleDirection::PingPong);
        assert_eq!(slice.current_step, 0);
        let _ = slice.advance(0.1);
        assert_eq!(slice.current_step, 1);
        let _ = slice.advance(0.1);
        assert_eq!(slice.current_step, 2);
        let _ = slice.advance(0.1);
        assert_eq!(slice.current_step, 3);
        let _ = slice.advance(0.1);
        assert_eq!(slice.current_step, 2); // Reversing
        let _ = slice.advance(0.1);
        assert_eq!(slice.current_step, 1);
        let _ = slice.advance(0.1);
        assert_eq!(slice.current_step, 0); // At start, switches to forward
        let _ = slice.advance(0.1);
        assert_eq!(slice.current_step, 1);
    }

    #[test]
    fn test_palette_cycler_cycle_palette() {
        let mut cycler = PaletteCycler::<1>::new();
        cycler.add_range(1, 3, 20.0, CycleDirection::Forward);

        let base = [Rgb565::RED, Rgb565::GREEN, Rgb565::BLUE, Rgb565::WHITE];
        let mut dest = [Rgb565::BLACK; 4];

        cycler.cycle_palette(&base, &mut dest);
        assert_eq!(dest[0], Rgb565::RED);
        assert_eq!(dest[1], Rgb565::GREEN);
        assert_eq!(dest[2], Rgb565::BLUE);
        assert_eq!(dest[3], Rgb565::WHITE);

        let _ = cycler.advance(0.05); // 1 step
        cycler.cycle_palette(&base, &mut dest);
        assert_eq!(dest[1], Rgb565::BLUE);
        assert_eq!(dest[2], Rgb565::GREEN);
    }
}
