use kurbo::{BezPath, Circle, Ellipse, Line, PathEl, Point, Rect, Shape, Vec2};
use roxmltree::{Document, Node};

/// A single cut path extracted from an SVG shape or path element
#[derive(Debug, Clone, PartialEq)]
pub struct SvgPathItem {
    pub id: String,
    pub name: String,
    pub points: Vec<Point>,
    pub stroke_color: Option<String>,
    pub is_closed: bool,
    pub selected: bool,
}

/// A hierarchical group or layer containing sub-groups and paths
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SvgGroupItem {
    pub id: String,
    pub name: String,
    pub is_layer: bool,
    pub groups: Vec<SvgGroupItem>,
    pub paths: Vec<SvgPathItem>,
    pub selected: bool,
    pub expanded: bool,
}

impl SvgGroupItem {
    pub fn new(id: String, name: String, is_layer: bool) -> Self {
        Self {
            id,
            name,
            is_layer,
            groups: Vec::new(),
            paths: Vec::new(),
            selected: true,
            expanded: true,
        }
    }

    /// Recursively collect all points for paths that are selected
    pub fn collect_selected_paths(&self, out: &mut Vec<Vec<Point>>) {
        for path in &self.paths {
            if path.selected && path.points.len() >= 2 {
                out.push(path.points.clone());
            }
        }
        for grp in &self.groups {
            grp.collect_selected_paths(out);
        }
    }

    /// Recursively collect all paths for rendering, tagged with (points, is_active)
    pub fn collect_all_paths(&self, out: &mut Vec<(Vec<Point>, bool)>) {
        for path in &self.paths {
            if path.points.len() >= 2 {
                out.push((path.points.clone(), path.selected));
            }
        }
        for grp in &self.groups {
            grp.collect_all_paths(out);
        }
    }

    /// Recursively select or deselect all children
    pub fn set_selected_recursive(&mut self, sel: bool) {
        self.selected = sel;
        for path in &mut self.paths {
            path.selected = sel;
        }
        for grp in &mut self.groups {
            grp.set_selected_recursive(sel);
        }
    }

    /// Recursively invert selection of all children
    pub fn invert_selected_recursive(&mut self) {
        self.selected = !self.selected;
        for path in &mut self.paths {
            path.selected = !path.selected;
        }
        for grp in &mut self.groups {
            grp.invert_selected_recursive();
        }
    }

    pub fn total_paths(&self) -> usize {
        let mut count = self.paths.len();
        for grp in &self.groups {
            count += grp.total_paths();
        }
        count
    }

    pub fn selected_paths(&self) -> usize {
        let mut count = self.paths.iter().filter(|p| p.selected).count();
        for grp in &self.groups {
            count += grp.selected_paths();
        }
        count
    }
}

/// Vector cut document loaded from an SVG file with complete layer/group hierarchy
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SvgDocument {
    pub filename: String,
    pub filepath: Option<String>,
    pub width_mm: Option<f64>,
    pub height_mm: Option<f64>,
    pub root_group: SvgGroupItem,
    /// Flattened view of all selected paths for easy iteration
    pub paths: Vec<Vec<Point>>,
    pub selected: bool,
    pub expanded: bool,
}

impl SvgDocument {
    #[allow(dead_code)]
    pub fn collect_selected_paths(&self, out: &mut Vec<Vec<Point>>) {
        if self.selected {
            self.root_group.collect_selected_paths(out);
        }
    }

    pub fn collect_all_paths(&self, out: &mut Vec<(Vec<Point>, bool)>) {
        if self.selected {
            self.root_group.collect_all_paths(out);
        } else {
            let mut temp = Vec::new();
            self.root_group.collect_all_paths(&mut temp);
            for (pts, _) in temp {
                out.push((pts, false));
            }
        }
    }

    pub fn set_selected_recursive(&mut self, sel: bool) {
        self.selected = sel;
        self.root_group.set_selected_recursive(sel);
    }

    pub fn invert_selected_recursive(&mut self) {
        self.root_group.invert_selected_recursive();
        self.selected = self.root_group.selected_paths() > 0;
    }

    pub fn total_paths(&self) -> usize {
        self.root_group.total_paths()
    }

    pub fn selected_paths(&self) -> usize {
        if self.selected {
            self.root_group.selected_paths()
        } else {
            0
        }
    }
}

pub fn parse_svg(svg_content: &str, tolerance_mm: f64) -> Result<SvgDocument, String> {
    let doc = Document::parse(svg_content).map_err(|e| format!("XML parse error: {}", e))?;
    let mut result = SvgDocument {
        filename: "Design.svg".to_string(),
        filepath: None,
        width_mm: None,
        height_mm: None,
        root_group: SvgGroupItem::new("root".to_string(), "Root".to_string(), false),
        paths: Vec::new(),
        selected: true,
        expanded: true,
    };

    let root = doc.root_element();
    if let Some(w) = root.attribute("width") {
        result.width_mm = parse_length(w);
    }
    if let Some(h) = root.attribute("height") {
        result.height_mm = parse_length(h);
    }

    result.root_group = parse_group_node(root, tolerance_mm);
    result.root_group.name = "Document Root".to_string();

    let mut all_selected = Vec::new();
    result.root_group.collect_selected_paths(&mut all_selected);
    result.paths = all_selected;

    Ok(result)
}

fn get_node_attribute<'a>(node: &'a Node<'a, 'a>, attr_name: &str) -> Option<&'a str> {
    if let Some(val) = node.attribute(attr_name) {
        return Some(val);
    }
    let local = if let Some((_, l)) = attr_name.split_once(':') {
        l
    } else {
        attr_name
    };
    node.attributes()
        .find(|a| a.name() == attr_name || a.name() == local)
        .map(|a| a.value())
}

fn parse_group_node(node: Node, tolerance: f64) -> SvgGroupItem {
    let id = get_node_attribute(&node, "id").unwrap_or("").to_string();
    let is_layer = get_node_attribute(&node, "inkscape:groupmode") == Some("layer")
        || id.to_lowercase().starts_with("layer");

    let raw_name = get_node_attribute(&node, "inkscape:label")
        .or_else(|| get_node_attribute(&node, "id"))
        .unwrap_or(if is_layer { "Layer" } else { "Group" });

    let name = raw_name.to_string();
    let mut group = SvgGroupItem::new(id, name, is_layer);

    for child in node.children() {
        if !child.is_element() {
            continue;
        }

        match child.tag_name().name() {
            "g" => {
                let sub_grp = parse_group_node(child, tolerance);
                if sub_grp.total_paths() > 0 || !sub_grp.groups.is_empty() {
                    group.groups.push(sub_grp);
                }
            }
            "path" => {
                if let Some(d) = child.attribute("d")
                    && let Ok(bez) = BezPath::from_svg(d)
                {
                    let subpaths = extract_bezpath_subpaths(&bez, tolerance);
                    let child_id = child.attribute("id").unwrap_or("").to_string();
                    let stroke = child.attribute("stroke").map(|s| s.to_string());
                    for (idx, pts) in subpaths.into_iter().enumerate() {
                        let name = if child_id.is_empty() {
                            format!("Path {}", group.paths.len() + 1)
                        } else if idx == 0 {
                            child_id.clone()
                        } else {
                            format!("{}-{}", child_id, idx + 1)
                        };
                        let is_closed = pts.len() >= 3 && pts.first() == pts.last();
                        group.paths.push(SvgPathItem {
                            id: if idx == 0 { child_id.clone() } else { format!("{}-{}", child_id, idx) },
                            name,
                            points: pts,
                            stroke_color: stroke.clone(),
                            is_closed,
                            selected: true,
                        });
                    }
                }
            }
            "rect" => {
                let x: f64 = child.attribute("x").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                let y: f64 = child.attribute("y").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                let w: f64 = child.attribute("width").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                let h: f64 = child.attribute("height").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                if w > 0.0 && h > 0.0 {
                    let rect = Rect::new(x, y, x + w, y + h);
                    let subpaths = extract_bezpath_subpaths(&rect.to_path(tolerance), tolerance);
                    let child_id = child.attribute("id").unwrap_or("").to_string();
                    let stroke = child.attribute("stroke").map(|s| s.to_string());
                    for pts in subpaths {
                        group.paths.push(SvgPathItem {
                            id: child_id.clone(),
                            name: if child_id.is_empty() {
                                format!("Rectangle {:.1}×{:.1}mm", w, h)
                            } else {
                                child_id.clone()
                            },
                            points: pts,
                            stroke_color: stroke.clone(),
                            is_closed: true,
                            selected: true,
                        });
                    }
                }
            }
            "circle" => {
                let cx: f64 = child.attribute("cx").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                let cy: f64 = child.attribute("cy").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                let r: f64 = child.attribute("r").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                if r > 0.0 {
                    let circle = Circle::new(Point::new(cx, cy), r);
                    let subpaths = extract_bezpath_subpaths(&circle.to_path(tolerance), tolerance);
                    let child_id = child.attribute("id").unwrap_or("").to_string();
                    let stroke = child.attribute("stroke").map(|s| s.to_string());
                    for pts in subpaths {
                        group.paths.push(SvgPathItem {
                            id: child_id.clone(),
                            name: if child_id.is_empty() {
                                format!("Circle Ø{:.1}mm", r * 2.0)
                            } else {
                                child_id.clone()
                            },
                            points: pts,
                            stroke_color: stroke.clone(),
                            is_closed: true,
                            selected: true,
                        });
                    }
                }
            }
            "ellipse" => {
                let cx: f64 = child.attribute("cx").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                let cy: f64 = child.attribute("cy").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                let rx: f64 = child.attribute("rx").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                let ry: f64 = child.attribute("ry").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                if rx > 0.0 && ry > 0.0 {
                    let ellipse = Ellipse::new(Point::new(cx, cy), Vec2::new(rx, ry), 0.0);
                    let subpaths = extract_bezpath_subpaths(&ellipse.to_path(tolerance), tolerance);
                    let child_id = child.attribute("id").unwrap_or("").to_string();
                    let stroke = child.attribute("stroke").map(|s| s.to_string());
                    for pts in subpaths {
                        group.paths.push(SvgPathItem {
                            id: child_id.clone(),
                            name: if child_id.is_empty() {
                                format!("Ellipse {:.1}×{:.1}mm", rx * 2.0, ry * 2.0)
                            } else {
                                child_id.clone()
                            },
                            points: pts,
                            stroke_color: stroke.clone(),
                            is_closed: true,
                            selected: true,
                        });
                    }
                }
            }
            "line" => {
                let x1: f64 = child.attribute("x1").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                let y1: f64 = child.attribute("y1").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                let x2: f64 = child.attribute("x2").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                let y2: f64 = child.attribute("y2").and_then(|v| v.parse().ok()).unwrap_or(0.0);
                let line = Line::new(Point::new(x1, y1), Point::new(x2, y2));
                let subpaths = extract_bezpath_subpaths(&line.to_path(tolerance), tolerance);
                let child_id = child.attribute("id").unwrap_or("").to_string();
                let stroke = child.attribute("stroke").map(|s| s.to_string());
                for pts in subpaths {
                    group.paths.push(SvgPathItem {
                        id: child_id.clone(),
                        name: if child_id.is_empty() {
                            "Line".to_string()
                        } else {
                            child_id.clone()
                        },
                        points: pts,
                        stroke_color: stroke.clone(),
                        is_closed: false,
                        selected: true,
                    });
                }
            }
            "polyline" | "polygon" => {
                let is_polygon = child.tag_name().name() == "polygon";
                if let Some(pts_str) = child.attribute("points") {
                    let coords: Vec<f64> = pts_str
                        .split(|c: char| c.is_whitespace() || c == ',')
                        .filter(|s| !s.is_empty())
                        .filter_map(|s| s.parse::<f64>().ok())
                        .collect();
                    let mut pts = Vec::new();
                    for &[x, y] in coords.as_chunks::<2>().0 {
                        pts.push(Point::new(x, y));
                    }
                    if is_polygon
                        && pts.len() >= 3
                        && let Some(&first) = pts.first()
                        && pts.last() != Some(&first)
                    {
                        pts.push(first);
                    }
                    if pts.len() >= 2 {
                        let child_id = child.attribute("id").unwrap_or("").to_string();
                        let stroke = child.attribute("stroke").map(|s| s.to_string());
                        group.paths.push(SvgPathItem {
                            id: child_id.clone(),
                            name: if child_id.is_empty() {
                                if is_polygon {
                                    "Polygon".to_string()
                                } else {
                                    "Polyline".to_string()
                                }
                            } else {
                                child_id
                            },
                            points: pts,
                            stroke_color: stroke,
                            is_closed: is_polygon,
                            selected: true,
                        });
                    }
                }
            }
            _ => {}
        }
    }

    group
}

fn extract_bezpath_subpaths(bez: &BezPath, tolerance: f64) -> Vec<Vec<Point>> {
    let mut out: Vec<Vec<Point>> = Vec::new();
    let mut current_subpath: Vec<Point> = Vec::new();
    let mut start_point: Option<Point> = None;

    kurbo::flatten(bez, tolerance, |el| match el {
        PathEl::MoveTo(p) => {
            if current_subpath.len() >= 2 {
                out.push(std::mem::take(&mut current_subpath));
            } else {
                current_subpath.clear();
            }
            current_subpath.push(p);
            start_point = Some(p);
        }
        PathEl::LineTo(p) => {
            current_subpath.push(p);
        }
        PathEl::ClosePath => {
            if let Some(sp) = start_point
                && current_subpath.last() != Some(&sp)
            {
                current_subpath.push(sp);
            }
            if current_subpath.len() >= 2 {
                out.push(std::mem::take(&mut current_subpath));
            }
        }
        _ => {}
    });

    if current_subpath.len() >= 2 {
        out.push(current_subpath);
    }

    out
}

fn parse_length(s: &str) -> Option<f64> {
    let s = s.trim();
    if let Some(val) = s.strip_suffix("mm") {
        val.parse().ok()
    } else if let Some(val) = s.strip_suffix("cm") {
        val.parse::<f64>().ok().map(|v| v * 10.0)
    } else if let Some(val) = s.strip_suffix("in") {
        val.parse::<f64>().ok().map(|v| v * 25.4)
    } else if let Some(val) = s.strip_suffix("px") {
        val.parse::<f64>().ok().map(|v| v * 0.264583) // 96 DPI
    } else {
        s.parse::<f64>().ok().map(|v| v * 0.264583)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hierarchical_layer_and_group_parsing() {
        let svg = r#"<svg xmlns="http://www.w3.org/2000/svg" xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape" width="100mm" height="100mm">
            <g inkscape:groupmode="layer" id="layer_red" inkscape:label="Red Layer">
                <rect id="rect1" x="10" y="10" width="20" height="20" stroke="red"/>
                <g id="subgroup_stars">
                    <circle id="star_center" cx="20" cy="20" r="5" stroke="red"/>
                </g>
            </g>
            <g inkscape:groupmode="layer" id="layer_blue" inkscape:label="Blue Layer">
                <line id="line1" x1="0" y1="0" x2="50" y2="50" stroke="blue"/>
            </g>
        </svg>"#;

        let mut doc = parse_svg(svg, 0.05).expect("SVG should parse successfully");
        assert_eq!(doc.total_paths(), 3);
        assert_eq!(doc.selected_paths(), 3);

        // Check layer detection
        let layers: Vec<&SvgGroupItem> = doc.root_group.groups.iter().filter(|g| g.is_layer).collect();
        assert_eq!(layers.len(), 2);
        assert_eq!(layers[0].name, "Red Layer");
        assert_eq!(layers[1].name, "Blue Layer");

        // Test deselecting Red Layer
        doc.root_group.groups[0].set_selected_recursive(false);
        assert_eq!(doc.selected_paths(), 1); // Only Blue Layer line is active

        let mut active = Vec::new();
        doc.collect_selected_paths(&mut active);
        assert_eq!(active.len(), 1);

        // Test invert selection
        doc.invert_selected_recursive();
        assert_eq!(doc.selected_paths(), 2); // Now only Red Layer (rect + circle) is active

        let mut active_after_invert = Vec::new();
        doc.collect_selected_paths(&mut active_after_invert);
        assert_eq!(active_after_invert.len(), 2);
    }
}
