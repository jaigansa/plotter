//! HP-GL (Hewlett-Packard Graphics Language) Generator for Redsail Vinyl Cutters
//! Standard resolution: 1016 DPI (40 units / mm, 0.025 mm per step)

pub const STEPS_PER_MM: f64 = 40.0;

#[derive(Debug, Clone)]
pub struct CutterSettings {
    pub speed: u32,       // Speed in cm/s (typically 1 to 60)
    pub force: u32,       // Cutting force in grams (typically 10 to 400)
    #[allow(dead_code)]
    pub blade_offset_mm: f64, // Drag blade tip offset (typically 0.25 mm)
}

impl Default for CutterSettings {
    fn default() -> Self {
        Self {
            speed: 20,
            force: 80,
            blade_offset_mm: 0.25,
        }
    }
}

pub struct HpglBuilder {
    buffer: String,
    current_pen_down: bool,
    last_x: Option<i32>,
    last_y: Option<i32>,
}

impl HpglBuilder {
    pub fn new(settings: &CutterSettings) -> Self {
        let mut builder = Self {
            buffer: String::new(),
            current_pen_down: false,
            last_x: None,
            last_y: None,
        };

        // Standard Redsail HP-GL initialization sequence
        builder.buffer.push_str("IN;");                             // Initialize
        builder.buffer.push_str("SP1;");                            // Select Pen 1
        builder.buffer.push_str("PA;");                             // Plot Absolute (crucial for correct shapes)
        builder.buffer.push_str(&format!("VS{};", settings.speed)); // Velocity
        builder.buffer.push_str(&format!("FS{};", settings.force)); // Force
        builder
    }

    /// Pen Up move (rapid travel without cutting).
    /// If pen is currently down, it issues an immediate in-place lift `PU;` first,
    /// guaranteeing the blade lifts cleanly before rapid carriage motion begins.
    pub fn pen_up(&mut self, x: i32, y: i32) {
        if self.current_pen_down {
            self.buffer.push_str("PU;");
            self.current_pen_down = false;
        }
        if self.last_x != Some(x) || self.last_y != Some(y) {
            self.buffer.push_str(&format!("PU{},{};", x, y));
            self.last_x = Some(x);
            self.last_y = Some(y);
        }
    }

    /// Explicitly raise blade in place without any coordinate motion
    pub fn pen_up_in_place(&mut self) {
        self.buffer.push_str("PU;");
        self.current_pen_down = false;
    }

    /// Pen Down cut (cutting line to destination).
    /// Automatically suppresses duplicate consecutive points to prevent stepper hesitation.
    pub fn pen_down(&mut self, x: i32, y: i32) {
        if self.current_pen_down && self.last_x == Some(x) && self.last_y == Some(y) {
            return;
        }
        self.buffer.push_str(&format!("PD{},{};", x, y));
        self.current_pen_down = true;
        self.last_x = Some(x);
        self.last_y = Some(y);
    }

    /// Complete cut job, safely lift knife, return to origin (0,0), and park
    pub fn finish(mut self) -> String {
        self.pen_up_in_place();         // Lift tool safely in place
        self.buffer.push_str("PU0,0;"); // Return to Origin (0,0) with blade UP
        self.buffer.push_str("SP0;");   // Deselect tool
        self.buffer
    }

    #[allow(dead_code)]
    pub fn as_str(&self) -> &str {
        &self.buffer
    }
}

/// Convert millimeters to cutter step coordinates (strictly non-negative)
pub fn mm_to_plotter(mm: f64) -> i32 {
    (mm.max(0.0) * STEPS_PER_MM).round() as i32
}

/// Convert cutter step coordinates to millimeters
#[allow(dead_code)]
pub fn plotter_to_mm(units: i32) -> f64 {
    units as f64 / STEPS_PER_MM
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_blade_lift_in_place_before_travel() {
        let settings = CutterSettings::default();
        let mut builder = HpglBuilder::new(&settings);

        builder.pen_up(100, 100);
        builder.pen_down(200, 200);
        // After cutting to (200, 200), pen_up to (300, 300) must lift in place first
        builder.pen_up(300, 300);

        let hpgl = builder.finish();
        // Must contain "PU;" to lift solenoid, followed by "PU300,300;"
        assert!(hpgl.contains("PD200,200;PU;PU300,300;"));
        // Must finish cleanly by lifting and returning to origin (0,0) before deselecting tool
        assert!(hpgl.ends_with("PU;PU0,0;SP0;"));
        assert!(!hpgl.contains("!PG;"));
    }

    #[test]
    fn test_explicit_pen_up_in_place() {
        let settings = CutterSettings::default();
        let mut builder = HpglBuilder::new(&settings);

        builder.pen_up(10, 10);
        builder.pen_down(50, 50);
        builder.pen_up_in_place();

        let hpgl = builder.finish();
        assert!(hpgl.contains("PD50,50;PU;"));
        assert!(hpgl.ends_with("PU0,0;SP0;"));
        assert!(!hpgl.contains("!PG;"));
    }

    #[test]
    fn test_duplicate_point_elimination() {
        let settings = CutterSettings::default();
        let mut builder = HpglBuilder::new(&settings);

        builder.pen_up(10, 10);
        builder.pen_down(20, 20);
        builder.pen_down(20, 20); // duplicate point
        builder.pen_down(30, 30);

        let hpgl = builder.finish();
        assert_eq!(hpgl.matches("PD20,20;").count(), 1);
        assert!(hpgl.contains("PD20,20;PD30,30;"));
    }
}
