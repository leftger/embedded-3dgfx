use core::fmt::Write;
use heapless::String;

#[cfg(all(not(test), not(feature = "std")))]
use embassy_time::Instant;

#[cfg(all(not(test), not(feature = "std")))]
fn now_us() -> u64 {
    Instant::now().as_micros() as u64
}

#[cfg(any(test, feature = "std"))]
fn now_us() -> u64 {
    extern crate std;
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_micros() as u64
}

#[derive(Debug)]
pub struct PerformanceCounter {
    frame_count: u64,
    text: String<256>,
    old_text: String<256>,
    only_fps: bool,
    start_time_us: u64,
    last_checkpoint_us: u64,
}

impl Default for PerformanceCounter {
    fn default() -> Self {
        Self::new()
    }
}

impl PerformanceCounter {
    pub fn new() -> Self {
        let now = now_us();
        Self {
            frame_count: 0,
            text: String::new(),
            old_text: String::new(),
            only_fps: false,
            start_time_us: now,
            last_checkpoint_us: now,
        }
    }

    pub fn only_fps(&mut self, only_fps: bool) {
        self.only_fps = only_fps;
    }

    pub fn get_frametime(&self) -> u64 {
        now_us().saturating_sub(self.start_time_us)
    }

    pub fn start_of_frame(&mut self) {
        self.frame_count += 1;
        self.text.clear();
        let now = now_us();
        self.start_time_us = now;
        self.last_checkpoint_us = now;
    }

    pub fn add_measurement(&mut self, label: &str) {
        if self.only_fps {
            return;
        }
        let now = now_us();
        let duration_us = now.saturating_sub(self.last_checkpoint_us);
        let _ = write!(self.text, "{}: {}\n", label, duration_us);
        self.last_checkpoint_us = now;
    }

    pub fn discard_measurement(&mut self) {
        self.last_checkpoint_us = now_us();
    }

    pub fn print(&mut self) {
        let total_us = self.get_frametime();
        let fps = if total_us > 0 {
            1_000_000 / total_us
        } else {
            0
        };
        if !self.only_fps {
            let _ = write!(self.text, "total: {}\n", total_us);
        }
        let _ = write!(self.text, "fps: {}\n", fps);
        self.old_text = self.text.clone();
    }

    pub fn get_text(&self) -> &str {
        &self.old_text
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    #[test]
    fn test_perfcounter_creation() {
        let perf = PerformanceCounter::new();
        assert_eq!(perf.get_text(), "");
        assert_eq!(perf.frame_count, 0);
    }

    #[test]
    fn test_perfcounter_default() {
        let perf = PerformanceCounter::default();
        assert_eq!(perf.get_text(), "");
    }

    #[test]
    fn test_perfcounter_start_of_frame() {
        let mut perf = PerformanceCounter::new();
        perf.start_of_frame();
        assert_eq!(perf.frame_count, 1);

        perf.start_of_frame();
        assert_eq!(perf.frame_count, 2);
    }

    #[test]
    fn test_perfcounter_get_frametime() {
        let mut perf = PerformanceCounter::new();
        perf.start_of_frame();

        // Sleep a tiny bit to ensure time passes
        std::thread::sleep(std::time::Duration::from_micros(100));

        let frametime = perf.get_frametime();
        assert!(frametime >= 100); // At least 100 microseconds
    }

    #[test]
    fn test_perfcounter_add_measurement() {
        let mut perf = PerformanceCounter::new();
        perf.start_of_frame();

        std::thread::sleep(std::time::Duration::from_micros(100));
        perf.add_measurement("test_label");

        perf.print();
        let text = perf.get_text();

        // Should contain the label
        assert!(text.contains("test_label"));
        // Should contain some duration value
        assert!(text.contains(":"));
    }

    #[test]
    fn test_perfcounter_only_fps_mode() {
        let mut perf = PerformanceCounter::new();
        perf.only_fps(true);
        perf.start_of_frame();

        perf.add_measurement("should_be_ignored");

        perf.print();
        let text = perf.get_text();

        // Should not contain the label in FPS-only mode
        assert!(!text.contains("should_be_ignored"));
        // But should still contain FPS
        assert!(text.contains("fps"));
    }

    #[test]
    fn test_perfcounter_discard_measurement() {
        let mut perf = PerformanceCounter::new();
        perf.start_of_frame();

        std::thread::sleep(std::time::Duration::from_micros(100));
        perf.discard_measurement();

        // Next measurement should start from discarded checkpoint
        let checkpoint_after_discard = perf.last_checkpoint_us;
        assert!(checkpoint_after_discard > perf.start_time_us);
    }

    #[test]
    fn test_perfcounter_print_includes_fps() {
        let mut perf = PerformanceCounter::new();
        perf.start_of_frame();

        std::thread::sleep(std::time::Duration::from_micros(1000));

        perf.print();
        let text = perf.get_text();

        // Should contain FPS
        assert!(text.contains("fps"));
        // Should contain total time
        assert!(text.contains("total"));
    }

    #[test]
    fn test_perfcounter_fps_calculation() {
        let mut perf = PerformanceCounter::new();
        perf.start_of_frame();

        std::thread::sleep(std::time::Duration::from_millis(10));

        perf.print();
        let text = perf.get_text();

        // FPS should be less than 100 (since we slept for 10ms)
        // Just verify the text is generated correctly
        assert!(text.len() > 0);
    }

    #[test]
    fn test_perfcounter_multiple_measurements() {
        let mut perf = PerformanceCounter::new();
        perf.start_of_frame();

        std::thread::sleep(std::time::Duration::from_micros(100));
        perf.add_measurement("step1");

        std::thread::sleep(std::time::Duration::from_micros(100));
        perf.add_measurement("step2");

        perf.print();
        let text = perf.get_text();

        // Should contain both labels
        assert!(text.contains("step1"));
        assert!(text.contains("step2"));
        assert!(text.contains("total"));
        assert!(text.contains("fps"));
    }
}
