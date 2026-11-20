use kurbo::Point;
use std::f64::consts::PI;

/// Drag-knife compensation:
/// Adjusts toolpaths to account for the blade offset (caster) of swivel drag knives.
#[derive(Debug, Clone)]
pub struct BladeCompensator {
    pub offset: f64, // Blade offset in mm (e.g. 0.25 mm)
    pub angle_threshold: f64, // Min angle difference (radians) to trigger swivel arc
}

impl BladeCompensator {
    pub fn new(offset: f64) -> Self {
        Self {
            offset,
            angle_threshold: 0.52, // ~30 degrees (ensures circles & arcs remain perfectly smooth)
        }
    }

    /// Process a continuous polyline (series of connected points)
    pub fn process_path(&self, points: &[Point]) -> Vec<Point> {
        if points.len() < 2 || self.offset <= 0.005 {
            return points.to_vec();
        }

        let mut compensated: Vec<Point> = Vec::new();
        let n = points.len();

        for i in 0..n - 1 {
            let p0 = points[i];
            let p1 = points[i + 1];

            let dx = p1.x - p0.x;
            let dy = p1.y - p0.y;
            let len = (dx * dx + dy * dy).sqrt();

            if len < 1e-6 {
                // Near-duplicate point: still record it so the path isn't broken,
                // but skip corner compensation for this degenerate segment.
                if compensated.is_empty() {
                    compensated.push(p0);
                }
                compensated.push(p1);
                continue;
            }

            let angle1 = dy.atan2(dx);

            if compensated.is_empty() {
                compensated.push(p0);
            }

            // Check if there is a next segment to pivot towards
            if i + 2 < n {
                let p2 = points[i + 2];
                let ndx = p2.x - p1.x;
                let ndy = p2.y - p1.y;
                let nlen = (ndx * ndx + ndy * ndy).sqrt();

                if nlen >= 1e-6 {
                    let angle2 = ndy.atan2(ndx);
                    let mut diff = angle2 - angle1;
                    while diff > PI {
                        diff -= 2.0 * PI;
                    }
                    while diff < -PI {
                        diff += 2.0 * PI;
                    }

                    // Only perform corner overcut/swivel for sharp corners (>30 deg)
                    if diff.abs() > self.angle_threshold {
                        // 1. Cut slightly past vertex (overcut) to ensure clean corner cut
                        let overcut = Point::new(
                            p1.x + self.offset * angle1.cos(),
                            p1.y + self.offset * angle1.sin(),
                        );
                        compensated.push(overcut);

                        // 2. Swivel blade in place to orient tip for the new path direction
                        let swivel_pt = Point::new(
                            p1.x + self.offset * angle2.cos(),
                            p1.y + self.offset * angle2.sin(),
                        );
                        compensated.push(swivel_pt);
                    } else {
                        compensated.push(p1);
                    }
                } else {
                    compensated.push(p1);
                }
            } else {
                // Final segment of path
                let final_pt = Point::new(
                    p1.x + self.offset * angle1.cos(),
                    p1.y + self.offset * angle1.sin(),
                );
                compensated.push(final_pt);
            }
        }

        // Closed loop overcut: ensure knife closes and cuts the initial segment start
        let is_closed = n >= 3 && (points[0].x - points[n - 1].x).hypot(points[0].y - points[n - 1].y) < 0.2;
        if is_closed && n >= 2 {
            let start_dx = points[1].x - points[0].x;
            let start_dy = points[1].y - points[0].y;
            let start_len = (start_dx * start_dx + start_dy * start_dy).sqrt();
            if start_len > 1e-6 {
                let start_angle = start_dy.atan2(start_dx);
                let close_overcut = Point::new(
                    points[0].x + self.offset * start_angle.cos(),
                    points[0].y + self.offset * start_angle.sin(),
                );
                compensated.push(close_overcut);
            }
        }

        compensated
    }
}
