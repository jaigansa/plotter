use kurbo::Point;

// ---------------------------------------------------------------------------
// Impossible-cut prevention
// ---------------------------------------------------------------------------

/// Physical limits of the plotter / cutting job.
/// `max_cut_width_mm` is enforced; `max_cut_length_mm` is `None` when feed length is unlimited.
/// `min_feature_mm` is the smallest cut feature the blade can reliably produce
/// (typically >= 2.0 x blade_offset).
#[derive(Debug, Clone, Copy)]
pub struct CutLimits {
    pub max_cut_width_mm: Option<f64>,
    pub max_cut_length_mm: Option<f64>,
    pub min_feature_mm: f64,
}

impl Default for CutLimits {
    fn default() -> Self {
        Self {
            max_cut_width_mm: Some(600.0), // common Redsail max carriage width
            max_cut_length_mm: None,       // feed length unlimited
            min_feature_mm: 0.5,           // 2.0 x 0.25 mm default blade offset
        }
    }
}

/// Validate that a cut job is physically possible before generating HP-GL.
/// Returns `Err` with a human-readable message if the job is impossible.
pub fn validate_cut_limits(
    active_paths: &[Vec<Point>],
    weeding_box: Option<&Vec<Point>>,
    limits: &CutLimits,
) -> Result<(), String> {
    if active_paths.is_empty() {
        return Ok(());
    }

    // ---- Overall bounds ----
    let mut min_x = f64::MAX;
    let mut min_y = f64::MAX;
    let mut max_x = f64::MIN;
    let mut max_y = f64::MIN;

    for path in active_paths {
        for pt in path {
            if pt.x < min_x { min_x = pt.x; }
            if pt.x > max_x { max_x = pt.x; }
            if pt.y < min_y { min_y = pt.y; }
            if pt.y > max_y { max_y = pt.y; }
        }
    }
    if let Some(wbox) = weeding_box {
        for pt in wbox {
            if pt.x < min_x { min_x = pt.x; }
            if pt.x > max_x { max_x = pt.x; }
            if pt.y < min_y { min_y = pt.y; }
            if pt.y > max_y { max_y = pt.y; }
        }
    }

    if min_x > max_x || min_y > max_y {
        return Ok(()); // nothing to validate
    }

    let width_mm = max_x - min_x;
    let length_mm = max_y - min_y;

    if let Some(max_w) = limits.max_cut_width_mm {
        if width_mm > max_w {
            return Err(format!(
                "Cut width {:.1} mm exceeds plotter max width {:.1} mm",
                width_mm, max_w
            ));
        }
    }

    if let Some(max_l) = limits.max_cut_length_mm {
        if length_mm > max_l {
            return Err(format!(
                "Cut length {:.1} mm exceeds plotter max length {:.1} mm",
                length_mm, max_l
            ));
        }
    }

    // ---- Min feature size ----
    if limits.min_feature_mm > 0.0 {
        for path in active_paths {
            if path.len() < 2 {
                continue;
            }
            for window in path.windows(2) {
                let dx = window[1].x - window[0].x;
                let dy = window[1].y - window[0].y;
                let seg = (dx * dx + dy * dy).sqrt();
                if seg > 0.0 && seg < limits.min_feature_mm {
                    return Err(format!(
                        "Cut segment {:.2} mm below min feature {:.2} mm (2 x blade offset)",
                        seg, limits.min_feature_mm
                    ));
                }
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normal_cut_passes() {
        let paths = vec![
            vec![
                Point::new(10.0, 10.0),
                Point::new(110.0, 10.0),
                Point::new(110.0, 110.0),
                Point::new(10.0, 110.0),
                Point::new(10.0, 10.0),
            ],
        ];
        let limits = CutLimits::default();
        assert!(validate_cut_limits(&paths, None, &limits).is_ok());
    }

    #[test]
    fn test_width_exceeds_max() {
        let paths = vec![vec![
            Point::new(0.0, 0.0),
            Point::new(700.0, 0.0),
            Point::new(700.0, 10.0),
            Point::new(0.0, 10.0),
            Point::new(0.0, 0.0),
        ]];
        let limits = CutLimits {
            max_cut_width_mm: Some(600.0),
            ..Default::default()
        };
        let err = validate_cut_limits(&paths, None, &limits).unwrap_err();
        assert!(err.contains("exceeds plotter max width"));
    }

    #[test]
    fn test_unlimited_width_passes() {
        let paths = vec![vec![
            Point::new(0.0, 0.0),
            Point::new(5000.0, 0.0),
            Point::new(5000.0, 10.0),
            Point::new(0.0, 10.0),
            Point::new(0.0, 0.0),
        ]];
        let limits = CutLimits {
            max_cut_width_mm: None,
            ..Default::default()
        };
        assert!(validate_cut_limits(&paths, None, &limits).is_ok());
    }

    #[test]
    fn test_segment_below_min_feature() {
        let paths = vec![vec![
            Point::new(0.0, 0.0),
            Point::new(0.1, 0.0), // 0.1 mm segment < 0.5 mm min
            Point::new(10.0, 0.0),
        ]];
        let limits = CutLimits::default();
        let err = validate_cut_limits(&paths, None, &limits).unwrap_err();
        assert!(err.contains("below min feature"));
    }

    #[test]
    fn test_weeding_box_counts_toward_width() {
        let paths = vec![vec![Point::new(500.0, 0.0), Point::new(510.0, 0.0)]];
        let wbox = vec![
            Point::new(490.0, -5.0),
            Point::new(520.0, -5.0),
            Point::new(520.0, 15.0),
            Point::new(490.0, 15.0),
            Point::new(490.0, -5.0),
        ];
        let limits = CutLimits {
            max_cut_width_mm: Some(20.0),
            ..Default::default()
        };
        let err = validate_cut_limits(&paths, Some(&wbox), &limits).unwrap_err();
        assert!(err.contains("exceeds plotter max width"));
    }

    #[test]
    fn test_empty_paths_ok() {
        let limits = CutLimits::default();
        assert!(validate_cut_limits(&[], None, &limits).is_ok());
    }
}
