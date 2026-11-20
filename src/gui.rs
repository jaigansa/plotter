use eframe::egui::{self, Color32, Pos2, Stroke, Vec2};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::Arc;
use std::time::Duration;

use crate::blade_offset::BladeCompensator;
use crate::cut_validation;
use crate::device::{list_ports, DetectedPort, FlowControlMode, RedsailDevice};
use crate::hpgl::{mm_to_plotter, CutterSettings, HpglBuilder};
use crate::svg_parser::{parse_svg, SvgDocument, SvgGroupItem, SvgPathItem};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanvasOriginCorner {
    BottomLeft,  // CAD Standard: 0,0 at bottom-left (+Width Right, +Length Up) (Default)
    BottomRight, // Machine Standard: 0,0 at bottom-right (+Width Left, +Length Up)
    TopLeft,     // Screen Standard: 0,0 at top-left (+Width Right, +Length Down)
}

impl CanvasOriginCorner {
    pub fn name(&self) -> &'static str {
        match self {
            CanvasOriginCorner::BottomLeft => "Origin: Bottom-Left (CAD Standard)",
            CanvasOriginCorner::BottomRight => "Origin: Bottom-Right (Machine Standard)",
            CanvasOriginCorner::TopLeft => "Origin: Top-Left (Screen Standard)",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MaterialPreset {
    PenPlotter,    // Pen on Paper / Calibration (Zero Offset)
    StandardVinyl, // Oracal 651 / 631
    HtvTShirt,     // Heat Transfer Vinyl (Siser EasyWeed)
    GlitterVinyl,  // Thick / Glitter Vinyl
    Cardstock,     // Cardstock / Paper (180-220g)
    Custom,
}

impl MaterialPreset {
    pub fn all() -> &'static [MaterialPreset] {
        &[
            MaterialPreset::PenPlotter,
            MaterialPreset::StandardVinyl,
            MaterialPreset::HtvTShirt,
            MaterialPreset::GlitterVinyl,
            MaterialPreset::Cardstock,
            MaterialPreset::Custom,
        ]
    }

    pub fn name(&self) -> &'static str {
        match self {
            MaterialPreset::PenPlotter => "Pen Plotting / Calibration (Zero Offset)",
            MaterialPreset::StandardVinyl => "Adhesive Vinyl (Oracal 651/631)",
            MaterialPreset::HtvTShirt => "Heat Transfer / HTV (T-Shirt)",
            MaterialPreset::GlitterVinyl => "Thick / Glitter Vinyl",
            MaterialPreset::Cardstock => "Cardstock / Paper (180-220g)",
            MaterialPreset::Custom => "Custom / Manual Settings",
        }
    }

    pub fn settings(&self) -> (u32, u32, f64) {
        match self {
            MaterialPreset::PenPlotter => (25, 40, 0.0),
            MaterialPreset::StandardVinyl => (20, 80, 0.25),
            MaterialPreset::HtvTShirt => (25, 60, 0.25),
            MaterialPreset::GlitterVinyl => (12, 140, 0.30),
            MaterialPreset::Cardstock => (15, 180, 0.25),
            MaterialPreset::Custom => (20, 80, 0.25),
        }
    }
}

const EMBEDDED_TEST_CUT_SVG: &str = r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 30 30" width="30mm" height="30mm">
  <!-- Outer calibration square (20mm x 20mm) -->
  <rect x="5" y="5" width="20" height="20" fill="none" stroke="black" stroke-width="0.5"/>
  <!-- Inner calibration circle (14mm diameter) -->
  <circle cx="15" cy="15" r="7" fill="none" stroke="black" stroke-width="0.5"/>
</svg>"#;

const EMBEDDED_AXIS_TEST_SVG: &str = r#"<svg xmlns="http://www.w3.org/2000/svg" width="75mm" height="50mm" viewBox="0 0 75 50">
  <!-- Origin crosshair at (5, 5) -->
  <line x1="2" y1="5" x2="8" y2="5" stroke="black" stroke-width="0.5"/>
  <line x1="5" y1="2" x2="5" y2="8" stroke="black" stroke-width="0.5"/>

  <!-- X Axis (Longer: 50mm, from 5 to 55) -->
  <line x1="5" y1="5" x2="55" y2="5" stroke="black" stroke-width="0.5"/>
  <!-- X Arrowhead -->
  <polyline points="50,8 55,5 50,2" fill="none" stroke="black" stroke-width="0.5"/>
  <!-- Letter X -->
  <line x1="57" y1="2" x2="63" y2="8" stroke="black" stroke-width="0.5"/>
  <line x1="57" y1="8" x2="63" y2="2" stroke="black" stroke-width="0.5"/>

  <!-- Y Axis (Shorter: 30mm, from 5 to 35) -->
  <line x1="5" y1="5" x2="5" y2="35" stroke="black" stroke-width="0.5"/>
  <!-- Y Arrowhead -->
  <polyline points="2,30 5,35 8,30" fill="none" stroke="black" stroke-width="0.5"/>
  <!-- Letter Y -->
  <polyline points="2,42 5,38 8,42" fill="none" stroke="black" stroke-width="0.5"/>
  <line x1="5" y1="38" x2="5" y2="34" stroke="black" stroke-width="0.5"/>

  <!-- Asymmetric orientation test glyph: Letter F -->
  <line x1="20" y1="12" x2="20" y2="26" stroke="black" stroke-width="0.5"/>
  <line x1="20" y1="26" x2="29" y2="26" stroke="black" stroke-width="0.5"/>
  <line x1="20" y1="19" x2="26" y2="19" stroke="black" stroke-width="0.5"/>

  <!-- 10mm calibration reference square -->
  <rect x="35" y="12" width="10" height="10" fill="none" stroke="black" stroke-width="0.5"/>
</svg>"#;

#[derive(Default, Clone)]
pub struct TransformedOutput {
    pub active_paths: Vec<Vec<kurbo::Point>>,
    pub inactive_paths: Vec<Vec<kurbo::Point>>,
    pub weeding_box: Option<Vec<kurbo::Point>>,
    #[allow(dead_code)]
    pub bounds: (f64, f64, f64, f64),
    #[allow(dead_code)]
    pub active_bounds: (f64, f64, f64, f64),
}

pub enum SerialCommand {
    ExecuteHpgl {
        hpgl: String,
        #[allow(dead_code)]
        total_pts: usize,
    },
    RawCommand {
        cmd: String,
        description: String,
    },
    EmergencyHalt,
    ConfigurePort {
        port: String,
        baud: u32,
        flow: FlowControlMode,
    },
}

pub enum SerialResponse {
    Log(String),
    CutProgress { sent: usize, total: usize },
    CutCompleted(bool),
    CommandCompleted,
    PortError(String),
}

fn ensure_device<'a>(
    device: &'a mut Option<RedsailDevice>,
    port: &str,
    baud: u32,
    flow: FlowControlMode,
    resp: &Sender<SerialResponse>,
) -> Option<&'a mut RedsailDevice> {
    if port.is_empty() {
        let _ = resp.send(SerialResponse::PortError("No serial port selected".to_string()));
        return None;
    }
    if device.is_none() {
        match RedsailDevice::open(port, baud, flow) {
            Ok(dev) => {
                *device = Some(dev);
            }
            Err(e) => {
                let _ = resp.send(SerialResponse::PortError(e));
                return None;
            }
        }
    }
    device.as_mut()
}

fn spawn_serial_worker(
    initial_port: String,
    initial_baud: u32,
    initial_flow: FlowControlMode,
    cmd_rx: Receiver<SerialCommand>,
    resp_tx: Sender<SerialResponse>,
    abort_flag: Arc<AtomicBool>,
) {
    std::thread::spawn(move || {
        let mut active_device: Option<RedsailDevice> = None;
        let mut current_port = initial_port;
        let mut current_baud = initial_baud;
        let mut current_flow = initial_flow;

        while let Ok(cmd) = cmd_rx.recv() {
            match cmd {
                SerialCommand::ConfigurePort { port, baud, flow } => {
                    if port != current_port || baud != current_baud || flow != current_flow {
                        active_device = None;
                        current_port = port;
                        current_baud = baud;
                        current_flow = flow;
                    }
                }
                SerialCommand::RawCommand { cmd, description } => {
                    abort_flag.store(false, Ordering::SeqCst);
                    if let Some(dev) = ensure_device(
                        &mut active_device,
                        &current_port,
                        current_baud,
                        current_flow,
                        &resp_tx,
                    ) {
                        match dev.send_hpgl(&cmd) {
                            Ok(_) => {
                                let _ = resp_tx.send(SerialResponse::Log(format!("[Machine] {}", description)));
                                let _ = resp_tx.send(SerialResponse::CommandCompleted);
                            }
                            Err(e) => {
                                active_device = None;
                                let _ = resp_tx.send(SerialResponse::PortError(format!("Command failed: {}", e)));
                            }
                        }
                    }
                }
                SerialCommand::ExecuteHpgl { hpgl, total_pts: _ } => {
                    abort_flag.store(false, Ordering::SeqCst);
                    let progress = Arc::new(AtomicUsize::new(0));
                    let total_bytes = hpgl.len();

                    if let Some(dev) = ensure_device(
                        &mut active_device,
                        &current_port,
                        current_baud,
                        current_flow,
                        &resp_tx,
                    ) {
                        let prog_clone = progress.clone();
                        let resp_clone = resp_tx.clone();
                        let flag_clone = abort_flag.clone();

                        let monitor = std::thread::spawn(move || {
                            let mut last_sent = 0;
                            while !flag_clone.load(Ordering::SeqCst) {
                                let current = prog_clone.load(Ordering::SeqCst);
                                if current != last_sent {
                                    last_sent = current;
                                    let _ = resp_clone.send(SerialResponse::CutProgress {
                                        sent: current,
                                        total: total_bytes,
                                    });
                                }
                                if current >= total_bytes {
                                    break;
                                }
                                std::thread::sleep(Duration::from_millis(50));
                            }
                        });

                        let res = dev.send_hpgl_cancellable(
                            &hpgl,
                            Some(&abort_flag),
                            Some(&progress),
                        );
                        let _ = monitor.join();

                        match res {
                            Ok(true) => {
                                let _ = resp_tx.send(SerialResponse::CutProgress {
                                    sent: total_bytes,
                                    total: total_bytes,
                                });
                                let _ = resp_tx.send(SerialResponse::CutCompleted(true));
                            }
                            Ok(false) => {
                                let _ = resp_tx.send(SerialResponse::CutCompleted(false));
                            }
                            Err(e) => {
                                active_device = None;
                                let _ = resp_tx.send(SerialResponse::PortError(format!("Cut error: {}", e)));
                                let _ = resp_tx.send(SerialResponse::CutCompleted(false));
                            }
                        }
                    } else {
                        let _ = resp_tx.send(SerialResponse::CutCompleted(false));
                    }
                }
                SerialCommand::EmergencyHalt => {
                    abort_flag.store(true, Ordering::SeqCst);
                    if let Some(dev) = active_device.as_mut() {
                        let _ = dev.emergency_halt();
                    } else if !current_port.is_empty()
                        && let Ok(mut dev) = RedsailDevice::open(&current_port, current_baud, current_flow)
                    {
                        let _ = dev.emergency_halt();
                    }
                    let _ = resp_tx.send(SerialResponse::Log("[EMERGENCY STOP] Cutter stopped immediately. Blade lifted.".to_string()));
                }
            }
        }
    });
}

fn render_path_item(ui: &mut egui::Ui, path: &mut SvgPathItem, depth: usize, dirty: &mut bool) {
    ui.horizontal(|ui| {
        ui.add_space((depth as f32) * 14.0);
        if dot_checkbox(ui, &mut path.selected, "").clicked() {
            *dirty = true;
        }
        let badge = if path.is_closed { "[Closed]" } else { "[Open]" };
        let path_label = if !path.name.is_empty() {
            format!("{} {}", path.name, badge)
        } else if !path.id.is_empty() {
            format!("#{} {}", path.id, badge)
        } else {
            format!("Path ({} pts) {}", path.points.len(), badge)
        };
        let text_color = if path.selected {
            Color32::LIGHT_GRAY
        } else {
            Color32::from_rgb(100, 100, 110)
        };
        ui.label(egui::RichText::new(path_label).size(11.0).color(text_color));
    });
}

fn render_group_item(ui: &mut egui::Ui, group: &mut SvgGroupItem, depth: usize, dirty: &mut bool) {
    let indent = (depth as f32) * 14.0;
    let total = group.total_paths();
    if total == 0 {
        return;
    }
    let sel_count = group.selected_paths();
    let is_all = sel_count == total;
    let is_none = sel_count == 0;

    ui.horizontal(|ui| {
        ui.add_space(indent);
        let arrow = if group.expanded { icons::CARET_DOWN } else { icons::CARET_RIGHT };
        if ui.small_button(arrow).clicked() {
            group.expanded = !group.expanded;
        }

        let mut checked = is_all;
        if dot_checkbox(ui, &mut checked, "").clicked() {
            *dirty = true;
            if is_all {
                group.set_selected_recursive(false);
            } else {
                group.set_selected_recursive(true);
            }
        }

        let tag = if group.is_layer {
            egui::RichText::new("[Layer]").color(Color32::from_rgb(100, 180, 255)).strong().size(11.0)
        } else {
            egui::RichText::new("[Group]").color(Color32::from_rgb(255, 180, 100)).size(11.0)
        };
        ui.label(tag);

        let display_name = if !group.name.is_empty() {
            &group.name
        } else if !group.id.is_empty() {
            &group.id
        } else {
            "Group"
        };

        let count_color = if is_all {
            Color32::from_rgb(0, 210, 255)
        } else if is_none {
            Color32::DARK_GRAY
        } else {
            Color32::from_rgb(255, 200, 50)
        };

        ui.label(
            egui::RichText::new(format!("{} ({}/{})", display_name, sel_count, total))
                .size(11.5)
                .color(count_color),
        );
    });

    if group.expanded {
        for sub_grp in &mut group.groups {
            render_group_item(ui, sub_grp, depth + 1, dirty);
        }
        for path in &mut group.paths {
            render_path_item(ui, path, depth + 1, dirty);
        }
    }
}

pub struct RedsailGuiApp {
    svg_path_input: String,
    loaded_docs: Vec<SvgDocument>,
    file_picker_tx: Sender<Vec<PathBuf>>,
    file_picker_rx: Receiver<Vec<PathBuf>>,
    selected_preset: MaterialPreset,
    speed: u32,
    force: u32,
    blade_offset: f64,
    flatten_tolerance: f64,
    max_cut_width: f64,
    min_feature: f64,
    scale: f64,
    custom_width_mm: f64,
    custom_height_mm: f64,
    orig_width_mm: f64,
    orig_height_mm: f64,
    lock_aspect_ratio: bool,
    normalize_origin: bool,

    // Canvas (display) axis controls — affects what you see on screen
    canvas_flip_x: bool,
    canvas_flip_y: bool,
    canvas_swap_xy: bool,

    // Machine (cutting) axis controls — affects what the plotter cuts
    machine_flip_x: bool,
    machine_flip_y: bool,
    machine_swap_xy: bool,

    add_weeding_box: bool,
    weeding_margin: f64,
    ports: Vec<DetectedPort>,
    selected_port: String,
    baud: u32,
    flow_mode: FlowControlMode,
    status_log: Vec<String>,
    cutter_x_steps: i32,
    cutter_y_steps: i32,
    show_machine_panel: bool,
    show_console_log: bool,
    developer_mode: bool,
    jog_step_mm: f64,
    est_pos_x: f64,
    est_pos_y: f64,
    is_cutting: bool,
    abort_flag: Arc<AtomicBool>,
    cut_progress_bytes: usize,
    cut_total_bytes: usize,
    cmd_tx: Sender<SerialCommand>,
    resp_rx: Receiver<SerialResponse>,
    canvas_zoom: f32,
    canvas_pan: Vec2,
    pub canvas_origin_corner: CanvasOriginCorner,
    pub show_rulers: bool,
    pub pos_x_mm: f64,
    pub pos_y_mm: f64,
    pub pos_active_only: bool,
    pub target_move_x: f64,
    pub target_move_y: f64,

    // Transform cache: separate canvas (display) and machine (cutting) caches
    // since they use different axis controls.
    canvas_cache: Option<TransformedOutput>,
    machine_cache: Option<TransformedOutput>,
    transform_dirty: bool,
}

fn find_preferred_port(ports: &[DetectedPort]) -> String {
    ports
        .iter()
        .find(|p| p.is_likely_cutter)
        .or_else(|| ports.first())
        .map(|p| p.name.clone())
        .unwrap_or_default()
}

/// Checkbox that paints a filled dot when checked instead of a checkmark.
fn dot_checkbox(ui: &mut egui::Ui, checked: &mut bool, label: &str) -> egui::Response {
    let icon_size = ui.spacing().interact_size.y;
    let font_id = ui.style().text_styles[&egui::TextStyle::Body].clone();
    let label_size = font_id.size * label.len() as f32 * 0.6;
    let total_width = icon_size + ui.spacing().item_spacing.x + label_size;

    let (rect, mut response) = ui.allocate_exact_size(
        egui::vec2(total_width, icon_size),
        egui::Sense::click(),
    );

    if response.clicked() {
        *checked = !*checked;
        response.mark_changed();
    }

    let box_rect = egui::Rect::from_min_size(rect.min, egui::vec2(icon_size, icon_size));
    let visuals = ui.style().interact_selectable(&response, *checked);
    let text_color = visuals.fg_stroke.color;
    ui.painter()
        .rect_stroke(box_rect, egui::CornerRadius::same(2), visuals.fg_stroke, egui::StrokeKind::Inside);

    if *checked {
        let center = box_rect.center();
        ui.painter()
            .circle_filled(center, icon_size * 0.22, text_color);
    }

    if !label.is_empty() {
        let galley = ui.fonts(|f| f.layout(label.to_string(), font_id, text_color, f32::INFINITY));
        let text_pos = egui::pos2(
            box_rect.right() + ui.spacing().item_spacing.x,
            rect.center().y - galley.size().y / 2.0,
        );
        ui.painter().galley(text_pos, galley, text_color);
    }

    response
}

impl Default for RedsailGuiApp {
    fn default() -> Self {
        let ports = list_ports();
        let selected_port = find_preferred_port(&ports);

        let baud = 9600;
        let flow_mode = FlowControlMode::None;
        let abort_flag = Arc::new(AtomicBool::new(false));

        let (cmd_tx, cmd_rx) = channel::<SerialCommand>();
        let (resp_tx, resp_rx) = channel::<SerialResponse>();
        let (file_picker_tx, file_picker_rx) = channel::<Vec<PathBuf>>();

        spawn_serial_worker(
            selected_port.clone(),
            baud,
            flow_mode,
            cmd_rx,
            resp_tx,
            abort_flag.clone(),
        );

        let mut app = Self {
            svg_path_input: String::new(),
            loaded_docs: Vec::new(),
            file_picker_tx,
            file_picker_rx,
            selected_preset: MaterialPreset::PenPlotter,
            speed: 20,
            force: 40,
            blade_offset: 0.0,
            flatten_tolerance: 0.1,
            max_cut_width: 600.0,
            min_feature: 0.5,
            scale: 1.0,
            custom_width_mm: 30.0,
            custom_height_mm: 30.0,
            orig_width_mm: 30.0,
            orig_height_mm: 30.0,
            lock_aspect_ratio: true,
            normalize_origin: true,

            // Canvas defaults: match SVG as-is
            canvas_flip_x: false,
            canvas_flip_y: false,
            canvas_swap_xy: false,

            // Machine defaults: plotter X/Y swapped (feed=X, carriage=Y), flip Y for standard Redsail
            machine_flip_x: false,
            machine_flip_y: true,
            machine_swap_xy: true,

            add_weeding_box: false,
            weeding_margin: 5.0,
            ports,
            selected_port,
            baud,
            flow_mode,
            status_log: vec!["Plotter GUI initialized.".to_string()],
            cutter_x_steps: 0,
            cutter_y_steps: 0,
            show_machine_panel: true,
            show_console_log: false,
            developer_mode: false,
            jog_step_mm: 10.0,
            est_pos_x: 0.0,
            est_pos_y: 0.0,
            is_cutting: false,
            abort_flag,
            cut_progress_bytes: 0,
            cut_total_bytes: 0,
            cmd_tx,
            resp_rx,
            canvas_zoom: 1.0,
            canvas_pan: Vec2::ZERO,
            canvas_origin_corner: CanvasOriginCorner::BottomRight,
            show_rulers: true,
            pos_x_mm: 0.0,
            pos_y_mm: 0.0,
            pos_active_only: true,
            target_move_x: 0.0,
            target_move_y: 0.0,
            canvas_cache: None,
            machine_cache: None,
            transform_dirty: true,
        };

        app.load_svg();
        app
    }
}

impl RedsailGuiApp {
    fn total_cut_length_mm(output: &TransformedOutput) -> f64 {
        let mut total_len = 0.0;
        for path in &output.active_paths {
            for window in path.windows(2) {
                let dx = window[1].x - window[0].x;
                let dy = window[1].y - window[0].y;
                total_len += (dx * dx + dy * dy).sqrt();
            }
        }
        if let Some(wbox) = &output.weeding_box {
            for window in wbox.windows(2) {
                let dx = window[1].x - window[0].x;
                let dy = window[1].y - window[0].y;
                total_len += (dx * dx + dy * dy).sqrt();
            }
        }
        total_len
    }

    /// Compute the full transform with the given axis controls, and store it in the cache.
    /// flip_x/flip_y/swap_xy are applied AFTER normalize/scale/offset, in final output space.
    fn compute_transform(&mut self, flip_x: bool, flip_y: bool, swap_xy: bool) -> TransformedOutput {
        let mut all_paths: Vec<(Vec<kurbo::Point>, bool)> = Vec::new();
        for doc in &self.loaded_docs {
            doc.collect_all_paths(&mut all_paths);
        }

        if all_paths.is_empty() {
            return TransformedOutput::default();
        }

        // ---- Phase 1: raw document bounds ----
        let mut doc_min_x = f64::MAX;
        let mut doc_min_y = f64::MAX;
        let mut doc_max_x = f64::MIN;
        let mut doc_max_y = f64::MIN;

        let mut act_raw_min_x = f64::MAX;
        let mut act_raw_min_y = f64::MAX;
        let mut has_active = false;

        for (path, is_active) in &all_paths {
            for pt in path {
                if pt.x < doc_min_x { doc_min_x = pt.x; }
                if pt.x > doc_max_x { doc_max_x = pt.x; }
                if pt.y < doc_min_y { doc_min_y = pt.y; }
                if pt.y > doc_max_y { doc_max_y = pt.y; }

                if *is_active {
                    has_active = true;
                    if pt.x < act_raw_min_x { act_raw_min_x = pt.x; }
                    if pt.y < act_raw_min_y { act_raw_min_y = pt.y; }
                }
            }
        }

        if doc_min_x > doc_max_x || doc_min_y > doc_max_y {
            doc_min_x = 0.0; doc_max_x = 0.0;
            doc_min_y = 0.0; doc_max_y = 0.0;
        }
        if !has_active {
            act_raw_min_x = 0.0;
            act_raw_min_y = 0.0;
        }

        let (eff_scale_x, eff_scale_y) = if self.orig_width_mm > 0.0 && self.orig_height_mm > 0.0 {
            (
                self.custom_width_mm / self.orig_width_mm,
                self.custom_height_mm / self.orig_height_mm,
            )
        } else {
            (self.scale, self.scale)
        };

        let margin_mm = (self.blade_offset * 2.0).max(2.0);
        let extra_offset = if self.add_weeding_box { self.weeding_margin } else { 0.0 };
        let base_offset = margin_mm + extra_offset;

        // ---- Phase 2: compute base points (no flip/swap yet) ----
        let mut base_active_paths: Vec<Vec<kurbo::Point>> = Vec::new();
        let mut base_inactive_paths: Vec<Vec<kurbo::Point>> = Vec::new();

        let mut base_act_min_x = f64::MAX;
        let mut base_act_min_y = f64::MAX;
        let mut base_act_max_x = f64::MIN;
        let mut base_act_max_y = f64::MIN;

        for (path, is_active) in all_paths {
            let pts: Vec<kurbo::Point> = path
                .iter()
                .map(|pt| {
                    let (norm_x, norm_y, offset_x, offset_y) = if self.pos_active_only && has_active {
                        if is_active {
                            let nx = if self.normalize_origin {
                                pt.x - act_raw_min_x
                            } else {
                                pt.x
                            };
                            let ny = if self.normalize_origin {
                                pt.y - act_raw_min_y
                            } else {
                                pt.y
                            };
                            (nx, ny, self.pos_x_mm, self.pos_y_mm)
                        } else {
                            let nx = if self.normalize_origin {
                                pt.x - doc_min_x
                            } else {
                                pt.x
                            };
                            let ny = if self.normalize_origin {
                                pt.y - doc_min_y
                            } else {
                                pt.y
                            };
                            (nx, ny, 0.0, 0.0)
                        }
                    } else {
                        let nx = if self.normalize_origin {
                            pt.x - doc_min_x
                        } else {
                            pt.x
                        };
                        let ny = if self.normalize_origin {
                            pt.y - doc_min_y
                        } else {
                            pt.y
                        };
                        (nx, ny, self.pos_x_mm, self.pos_y_mm)
                    };

                    let x = norm_x * eff_scale_x + base_offset + offset_x;
                    let y = norm_y * eff_scale_y + base_offset + offset_y;
                    kurbo::Point::new(x, y)
                })
                .collect();

            if is_active {
                for pt in &pts {
                    if pt.x < base_act_min_x { base_act_min_x = pt.x; }
                    if pt.x > base_act_max_x { base_act_max_x = pt.x; }
                    if pt.y < base_act_min_y { base_act_min_y = pt.y; }
                    if pt.y > base_act_max_y { base_act_max_y = pt.y; }
                }
                base_active_paths.push(pts);
            } else {
                base_inactive_paths.push(pts);
            }
        }

        // ---- Phase 3: apply flip/swap to get final points ----
        let apply_axis = |pt: &kurbo::Point, max_x: f64, max_y: f64| -> kurbo::Point {
            let mut x = pt.x;
            let mut y = pt.y;
            if flip_x { x = max_x - x; }
            if flip_y { y = max_y - y; }
            if swap_xy { std::mem::swap(&mut x, &mut y); }
            kurbo::Point::new(x, y)
        };

        let mut active_paths: Vec<Vec<kurbo::Point>> = Vec::new();
        let mut inactive_paths: Vec<Vec<kurbo::Point>> = Vec::new();

        let mut act_min_x = f64::MAX;
        let mut act_min_y = f64::MAX;
        let mut act_max_x = f64::MIN;
        let mut act_max_y = f64::MIN;

        for path in &base_active_paths {
            let pts: Vec<kurbo::Point> = path.iter().map(|pt| apply_axis(pt, base_act_max_x, base_act_max_y)).collect();
            for pt in &pts {
                if pt.x < act_min_x { act_min_x = pt.x; }
                if pt.x > act_max_x { act_max_x = pt.x; }
                if pt.y < act_min_y { act_min_y = pt.y; }
                if pt.y > act_max_y { act_max_y = pt.y; }
            }
            active_paths.push(pts);
        }

        for path in &base_inactive_paths {
            let pts: Vec<kurbo::Point> = path.iter().map(|pt| apply_axis(pt, doc_max_x, doc_max_y)).collect();
            inactive_paths.push(pts);
        }

        let active_bounds = if act_min_x <= act_max_x && act_min_y <= act_max_y {
            (act_min_x, act_min_y, act_max_x, act_max_y)
        } else {
            (0.0, 0.0, 0.0, 0.0)
        };

        let weeding_box = if self.add_weeding_box && !active_paths.is_empty() && act_min_x <= act_max_x {
            let wm = self.weeding_margin;
            let bx0 = (act_min_x - wm).max(0.0);
            let by0 = (act_min_y - wm).max(0.0);
            let bx1 = act_max_x + wm;
            let by1 = act_max_y + wm;
            Some(vec![
                kurbo::Point::new(bx0, by0),
                kurbo::Point::new(bx1, by0),
                kurbo::Point::new(bx1, by1),
                kurbo::Point::new(bx0, by1),
                kurbo::Point::new(bx0, by0),
            ])
        } else {
            None
        };

        TransformedOutput {
            active_paths,
            inactive_paths,
            weeding_box,
            bounds: (doc_min_x, doc_min_y, doc_max_x, doc_max_y),
            active_bounds,
        }
    }

    /// Compute both canvas and machine transforms if dirty and store them in the cache.
    fn ensure_transform_computed(&mut self) {
        if self.transform_dirty {
            let canvas = self.compute_transform(
                self.canvas_flip_x,
                self.canvas_flip_y,
                self.canvas_swap_xy,
            );
            let machine = self.compute_transform(
                self.machine_flip_x,
                self.machine_flip_y,
                self.machine_swap_xy,
            );
            self.canvas_cache = Some(canvas);
            self.machine_cache = Some(machine);
            self.transform_dirty = false;
        }
    }

    fn log(&mut self, msg: String) {
        self.status_log.push(msg);
        if self.status_log.len() > 30 {
            self.status_log.remove(0);
        }
    }

    fn notify_port_config_changed(&self) {
        let _ = self.cmd_tx.send(SerialCommand::ConfigurePort {
            port: self.selected_port.clone(),
            baud: self.baud,
            flow: self.flow_mode,
        });
    }

    fn refresh_ports(&mut self) {
        self.ports = list_ports();
        if !self.ports.iter().any(|p| p.name == self.selected_port) {
            self.selected_port = find_preferred_port(&self.ports);
        }
        self.notify_port_config_changed();
        self.log(format!("Refreshed ports: found {} device(s).", self.ports.len()));
    }

    fn open_multi_file_dialog(&self) {
        let tx = self.file_picker_tx.clone();
        std::thread::spawn(move || {
            let output = std::process::Command::new("zenity")
                .env("GDK_BACKEND", "x11") // Bypass Wayland portal parent window association deadlock on Hyprland
                .arg("--file-selection")
                .arg("--multiple")
                .arg("--separator=|")
                .arg("--file-filter=SVG Files (*.svg) | *.svg *.SVG")
                .arg("--title=Select SVG Cut Files")
                .output();
            if let Ok(out) = output && out.status.success() {
                let stdout = String::from_utf8_lossy(&out.stdout);
                let paths: Vec<PathBuf> = stdout
                    .trim()
                    .split('|')
                    .filter(|s| !s.trim().is_empty())
                    .map(|s| PathBuf::from(s.trim()))
                    .collect();
                if !paths.is_empty() {
                    let _ = tx.send(paths);
                }
            }
        });
    }

    fn update_dimensions_from_docs(&mut self) {
        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;

        let mut has_pts = false;
        for doc in &self.loaded_docs {
            let mut paths = Vec::new();
            doc.collect_all_paths(&mut paths);
            for (path, _) in paths {
                for pt in path {
                    has_pts = true;
                    if pt.x < min_x { min_x = pt.x; }
                    if pt.x > max_x { max_x = pt.x; }
                    if pt.y < min_y { min_y = pt.y; }
                    if pt.y > max_y { max_y = pt.y; }
                }
            }
        }

        if has_pts && min_x <= max_x && min_y <= max_y {
            let w = (max_x - min_x).max(1.0);
            let h = (max_y - min_y).max(1.0);
            self.orig_width_mm = (w * 10.0).round() / 10.0;
            self.orig_height_mm = (h * 10.0).round() / 10.0;
            self.custom_width_mm = (w * self.scale * 10.0).round() / 10.0;
            self.custom_height_mm = (h * self.scale * 10.0).round() / 10.0;
        } else {
            self.orig_width_mm = 30.0;
            self.orig_height_mm = 30.0;
            self.custom_width_mm = 30.0;
            self.custom_height_mm = 30.0;
        }
        self.transform_dirty = true;
    }

    fn add_svg_content(&mut self, content: &str, label: &str, path: Option<&Path>) {
        // Quick reject: content must look like XML/SVG before we attempt a parse
        let trimmed = content.trim_start();
        if !trimmed.starts_with("<?xml") && !trimmed.starts_with("<svg") && !trimmed.starts_with("<SVG") {
            self.log(format!(
                "Failed to parse file '{}': content does not appear to be an SVG/XML file",
                label
            ));
            return;
        }

        match parse_svg(content, self.flatten_tolerance) {
            Ok(mut doc) => {
                doc.filename = path
                    .and_then(|p| p.file_name())
                    .map(|f| f.to_string_lossy().to_string())
                    .unwrap_or_else(|| label.to_string());
                doc.filepath = path.map(|p| p.to_string_lossy().to_string());
                let p_count = doc.total_paths();
                self.log(format!("Added file '{}': {} vector paths extracted.", doc.filename, p_count));
                self.loaded_docs.push(doc);
                self.update_dimensions_from_docs();
            }
            Err(err) => {
                self.log(format!("Failed to parse SVG '{}': {}", label, err));
            }
        }
    }

    fn add_svg_from_path(&mut self, path: &Path) {
        // Reject directories — read_to_string on a directory errors, but check up front for a clearer message
        if let Ok(metadata) = fs::metadata(path) {
            if metadata.is_dir() {
                self.log(format!("Failed to read file '{}': path is a directory, not a file", path.display()));
                return;
            }
        }

        // Sanity-check file size to avoid OOM on accidentally-opened huge files
        if let Ok(metadata) = fs::metadata(path) {
            const MAX_FILE_BYTES: u64 = 50_000_000; // 50 MB cap
            if metadata.len() > MAX_FILE_BYTES {
                self.log(format!(
                    "Failed to read file '{}': file is {:.1} MB, exceeds {:.1} MB limit",
                    path.display(),
                    metadata.len() as f64 / 1_000_000.0,
                    MAX_FILE_BYTES as f64 / 1_000_000.0
                ));
                return;
            }
        }

        match fs::read_to_string(path) {
            Ok(content) => {
                if content.trim().is_empty() {
                    self.log(format!("Failed to read file '{}': file is empty", path.display()));
                    return;
                }
                let label = path
                    .file_name()
                    .map(|f| f.to_string_lossy().to_string())
                    .unwrap_or_else(|| path.to_string_lossy().to_string());
                self.add_svg_content(&content, &label, Some(path));
            }
            Err(e) => {
                self.log(format!("Failed to read file '{}': {}", path.display(), e));
            }
        }
    }

    fn clear_all_documents(&mut self) {
        self.loaded_docs.clear();
        self.orig_width_mm = 30.0;
        self.orig_height_mm = 30.0;
        self.custom_width_mm = 30.0;
        self.custom_height_mm = 30.0;
        self.transform_dirty = true;
        self.log("Cleared all loaded design files.".to_string());
    }

    fn select_all(&mut self, sel: bool) {
        self.transform_dirty = true;
        for doc in &mut self.loaded_docs {
            doc.set_selected_recursive(sel);
        }
    }

    fn invert_selection(&mut self) {
        self.transform_dirty = true;
        for doc in &mut self.loaded_docs {
            doc.invert_selected_recursive();
        }
    }

    fn total_paths_count(&self) -> usize {
        self.loaded_docs.iter().map(|d| d.total_paths()).sum()
    }

    fn selected_paths_count(&self) -> usize {
        self.loaded_docs.iter().map(|d| d.selected_paths()).sum()
    }

    fn load_svg(&mut self) {
        let input = self.svg_path_input.trim();
        if input.is_empty() {
            return;
        }
        let path = PathBuf::from(input);

        // Reject directories up front
        if let Ok(metadata) = fs::metadata(&path) {
            if metadata.is_dir() {
                self.log(format!("File does not exist: '{}' is a directory", input));
                return;
            }
        }

        if path.exists() {
            self.add_svg_from_path(&path);
        } else if input == "test_cut.svg" {
            self.add_svg_content(EMBEDDED_TEST_CUT_SVG, "test_cut.svg", None);
        } else if input == "axis_test.svg" {
            self.add_svg_content(EMBEDDED_AXIS_TEST_SVG, "axis_test.svg", None);
        } else {
            self.log(format!("File does not exist: {}", input));
        }
    }


    fn generate_current_hpgl(&mut self) -> Result<(String, usize), String> {
        self.ensure_transform_computed();
        let output = self.machine_cache.as_ref().unwrap().clone();
        if output.active_paths.is_empty() {
            return Err("No vector paths selected to cut. Please check at least one layer or path in the Design Hierarchy tree.".to_string());
        }

        // Impossible-cut prevention: validate plotter max width and min feature size.
        let limits = cut_validation::CutLimits {
            max_cut_width_mm: Some(self.max_cut_width),
            max_cut_length_mm: None, // feed length unlimited
            min_feature_mm: self.min_feature,
        };
        if let Err(e) = cut_validation::validate_cut_limits(
            &output.active_paths,
            output.weeding_box.as_ref(),
            &limits,
        ) {
            return Err(e);
        }

        let settings = CutterSettings {
            speed: self.speed,
            force: self.force,
            blade_offset_mm: self.blade_offset,
        };

        let compensator = BladeCompensator::new(self.blade_offset);
        let mut builder = HpglBuilder::new(&settings);
        let mut total_points = 0;

        // 1. Cut all active/selected design paths first
        // Machine axis controls (flip_x/flip_y/swap_xy) already applied in compute_transform.
        // Plotter X/Y swapped: HP-GL PU{x},{y} where x=feed, y=carriage.
        for path in &output.active_paths {
            if path.is_empty() {
                continue;
            }
            let compensated = compensator.process_path(path);
            if compensated.is_empty() {
                continue;
            }

            total_points += compensated.len();
            let start = compensated[0];
            builder.pen_up(mm_to_plotter(start.y), mm_to_plotter(start.x));
            for pt in &compensated[1..] {
                builder.pen_down(mm_to_plotter(pt.y), mm_to_plotter(pt.x));
            }
            builder.pen_up_in_place();
        }

        // 2. Cut outer weeding border last
        if let Some(wbox) = &output.weeding_box {
            let compensated_box = compensator.process_path(wbox);
            if !compensated_box.is_empty() {
                total_points += compensated_box.len();
                let start = compensated_box[0];
                builder.pen_up(mm_to_plotter(start.y), mm_to_plotter(start.x));
                for pt in &compensated_box[1..] {
                    builder.pen_down(mm_to_plotter(pt.y), mm_to_plotter(pt.x));
                }
                builder.pen_up_in_place();
            }
        }

        Ok((builder.finish(), total_points))
    }

    fn execute_test_pattern(&mut self) {
        if self.is_cutting {
            self.log("[Warning] A cut job is already in progress.".to_string());
            return;
        }

        self.add_svg_content(EMBEDDED_TEST_CUT_SVG, "test_cut.svg", None);
        self.log("Cutting Test Pattern (with blade compensation & safe background streaming)...".to_string());
        self.execute_cut(false);
    }

    fn execute_test_box(&mut self) {
        if self.is_cutting {
            self.log("[Warning] A cut job or test is already in progress.".to_string());
            return;
        }

        if self.selected_port.is_empty() {
            self.log("[Error] No serial port selected!".to_string());
            return;
        }

        self.ensure_transform_computed();
        let output = self.machine_cache.as_ref().unwrap().clone();
        if output.active_paths.is_empty() {
            self.log("[Warning] No active vector paths to trace. Please check at least one layer in Artwork & Layers.".to_string());
            return;
        }

        // Impossible-cut prevention: validate plotter max width and min feature size.
        let limits = cut_validation::CutLimits {
            max_cut_width_mm: Some(self.max_cut_width),
            max_cut_length_mm: None, // feed length unlimited
            min_feature_mm: self.min_feature,
        };
        if let Err(e) = cut_validation::validate_cut_limits(
            &output.active_paths,
            output.weeding_box.as_ref(),
            &limits,
        ) {
            self.log(format!("[Error] {}", e));
            return;
        }

        // Determine outer bounding rectangle in cut coordinates
        let (min_x, min_y, max_x, max_y) = if let Some(wbox) = &output.weeding_box {
            let mut bx0 = f64::MAX;
            let mut by0 = f64::MAX;
            let mut bx1 = f64::MIN;
            let mut by1 = f64::MIN;
            for pt in wbox {
                if pt.x < bx0 { bx0 = pt.x; }
                if pt.x > bx1 { bx1 = pt.x; }
                if pt.y < by0 { by0 = pt.y; }
                if pt.y > by1 { by1 = pt.y; }
            }
            (bx0, by0, bx1, by1)
        } else {
            let mut mx0 = f64::MAX;
            let mut my0 = f64::MAX;
            let mut mx1 = f64::MIN;
            let mut my1 = f64::MIN;
            for path in &output.active_paths {
                for pt in path {
                    if pt.x < mx0 { mx0 = pt.x; }
                    if pt.x > mx1 { mx1 = pt.x; }
                    if pt.y < my0 { my0 = pt.y; }
                    if pt.y > my1 { my1 = pt.y; }
                }
            }
            (mx0, my0, mx1, my1)
        };

        if min_x > max_x || min_y > max_y {
            self.log("[Warning] Invalid geometry bounds for Test Box.".to_string());
            return;
        }

        let width_mm = max_x - min_x;
        let height_mm = max_y - min_y;

        self.log(format!(
            "Tracing Test Box ({:.1} mm x {:.1} mm) from ({:.1}, {:.1}) to ({:.1}, {:.1}) with blade lifted (PU)...",
            width_mm, height_mm, min_x, min_y, max_x, max_y
        ));

        let settings = CutterSettings {
            speed: self.speed,
            force: self.force,
            blade_offset_mm: 0.0,
        };

        let mut builder = HpglBuilder::new(&settings);
        builder.pen_up_in_place();

        // Trace the four corners with Pen Up (PU)
        let p0_x = mm_to_plotter(min_x);
        let p0_y = mm_to_plotter(min_y);
        let p1_x = mm_to_plotter(max_x);
        let p1_y = mm_to_plotter(min_y);
        let p2_x = mm_to_plotter(max_x);
        let p2_y = mm_to_plotter(max_y);
        let p3_x = mm_to_plotter(min_x);
        let p3_y = mm_to_plotter(max_y);

        builder.pen_up(p0_y, p0_x);
        builder.pen_up(p1_y, p1_x);
        builder.pen_up(p2_y, p2_x);
        builder.pen_up(p3_y, p3_x);
        builder.pen_up(p0_y, p0_x);

        let hpgl = builder.finish();

        self.cut_progress_bytes = 0;
        self.cut_total_bytes = hpgl.len();
        self.is_cutting = true;

        let _ = self.cmd_tx.send(SerialCommand::ExecuteHpgl {
            hpgl,
            total_pts: 5,
        });
    }

    fn execute_axis_test_pattern(&mut self) {
        if self.is_cutting {
            self.log("[Warning] A cut job is already in progress.".to_string());
            return;
        }

        self.add_svg_content(EMBEDDED_AXIS_TEST_SVG, "axis_test.svg", None);
        self.log("Cutting Axis Test Pattern (aligned with visual canvas preview)...".to_string());
        self.execute_cut(false);
    }

    fn cancel_cut(&mut self) {
        self.abort_flag.store(true, Ordering::SeqCst);
        let _ = self.cmd_tx.send(SerialCommand::EmergencyHalt);
        self.log("Stopping cut...".to_string());
    }

    fn execute_cut(&mut self, dry_run: bool) {
        if self.is_cutting {
            self.log("[Warning] A cut job is already in progress.".to_string());
            return;
        }

        match self.generate_current_hpgl() {
            Ok((hpgl, pts)) => {
                self.log(format!(
                    "Generated HP-GL (PA mode): {} bytes, {} cutter points.",
                    hpgl.len(),
                    pts
                ));
                if dry_run {
                    self.log("[DRY RUN] Simulation successful. Ready to cut.".to_string());
                    return;
                }

                if self.selected_port.is_empty() {
                    self.log("[Error] No serial port selected!".to_string());
                    return;
                }

                self.log(format!(
                    "Starting cut job on {} at {} baud (Flow: {})...",
                    self.selected_port, self.baud, self.flow_mode
                ));

                self.cut_progress_bytes = 0;
                self.cut_total_bytes = hpgl.len();
                self.is_cutting = true;

                let _ = self.cmd_tx.send(SerialCommand::ExecuteHpgl {
                    hpgl,
                    total_pts: pts,
                });
            }
            Err(e) => self.log(format!("[Error] {}", e)),
        }
    }

    fn send_raw_command(&mut self, cmd: &str, desc: &str) {
        if self.selected_port.is_empty() {
            self.log("[Error] No serial port selected!".to_string());
            return;
        }
        let _ = self.cmd_tx.send(SerialCommand::RawCommand {
            cmd: cmd.to_string(),
            description: desc.to_string(),
        });
    }

    fn jog_carriage(&mut self, d_mm: f64) {
        if self.is_cutting {
            self.log("[Blocked] Cannot jog while a cut job is in progress.".to_string());
            return;
        }

        // On this Redsail plotter the carriage (left/right) is wired to the plotter's Y axis,
        // so carriage jog updates cutter_y_steps and the second HP-GL coordinate.
        let d_steps = (d_mm * 40.0).round() as i32;
        self.cutter_y_steps += d_steps;
        let cmd = format!("PA;PU{},{};", self.cutter_x_steps, self.cutter_y_steps);
        let desc = format!("Carriage move {:+.1} mm (Y={:.1} mm)", d_mm, self.cutter_y_steps as f64 / 40.0);
        self.send_raw_command(&cmd, &desc);
        self.est_pos_y = (self.cutter_y_steps as f64 / 40.0 * 10.0).round() / 10.0;
    }

    fn jog_feed(&mut self, d_mm: f64) {
        if self.is_cutting {
            self.log("[Blocked] Cannot jog while a cut job is in progress.".to_string());
            return;
        }

        // On this Redsail plotter the vinyl feed (forward/back) is wired to the plotter's X axis,
        // so feed jog updates cutter_x_steps and the first HP-GL coordinate.
        let d_steps = (d_mm * 40.0).round() as i32;
        self.cutter_x_steps += d_steps;
        let cmd = format!("PA;PU{},{};", self.cutter_x_steps, self.cutter_y_steps);
        let desc = format!("Feed move {:+.1} mm (X={:.1} mm)", d_mm, self.cutter_x_steps as f64 / 40.0);
        self.send_raw_command(&cmd, &desc);
        self.est_pos_x = (self.cutter_x_steps as f64 / 40.0 * 10.0).round() / 10.0;
    }

    fn go_home(&mut self) {
        self.cutter_x_steps = 0;
        self.cutter_y_steps = 0;
        self.est_pos_x = 0.0;
        self.est_pos_y = 0.0;
        self.send_raw_command("PA;PU0,0;", "Return to Origin (0,0)");
    }

    fn move_to_position(&mut self, x_mm: f64, y_mm: f64) {
        if self.is_cutting {
            self.log("[Blocked] Cannot move while a cut job is in progress.".to_string());
            return;
        }

        // On this swapped-axis Redsail plotter:
        //   X axis = vinyl feed, Y axis = carriage
        //   HP-GL PU{x},{y}: first coord = feed(X), second coord = carriage(Y)
        let target_x_steps = (y_mm * 40.0).round() as i32; // feed position -> first coord
        let target_y_steps = (x_mm * 40.0).round() as i32; // carriage position -> second coord
        self.cutter_x_steps = target_x_steps;
        self.cutter_y_steps = target_y_steps;
        let cmd = format!("PA;PU{},{};", self.cutter_x_steps, self.cutter_y_steps);
        let desc = format!("Rapid move to Feed(X)={:.1} mm, Carriage(Y)={:.1} mm", y_mm, x_mm);
        self.send_raw_command(&cmd, &desc);
        self.est_pos_x = (y_mm * 10.0).round() / 10.0;
        self.est_pos_y = (x_mm * 10.0).round() / 10.0;
    }

    fn set_origin_here(&mut self) {
        self.cutter_x_steps = 0;
        self.cutter_y_steps = 0;
        self.est_pos_x = 0.0;
        self.est_pos_y = 0.0;
        // Reinitialize HP-GL frame at the current physical position without commanding any motor movement
        let cmd = format!("IN;SP1;PA;VS{};FS{};", self.speed, self.force);
        self.send_raw_command(&cmd, "Origin set at current tool position (0.0, 0.0)");
    }

    fn feed_out(&mut self, dist_mm: f64) {
        self.jog_feed(dist_mm);
    }

    fn pen_drop(&mut self) {
        self.send_raw_command("PD;", "Pen/Blade Down");
    }

    fn pen_lift(&mut self) {
        self.send_raw_command("PU;", "Pen/Blade Up");
    }

    fn pen_tap(&mut self) {
        self.send_raw_command("PD;PU;", "Pen/Blade Tap Test");
    }

    fn machine_reset(&mut self) {
        self.cutter_x_steps = 0;
        self.cutter_y_steps = 0;
        self.est_pos_x = 0.0;
        self.est_pos_y = 0.0;
        let cmd = format!("IN;SP1;PA;VS{};FS{};PU0,0;", self.speed, self.force);
        self.send_raw_command(&cmd, "Machine Reset / Reinitialize");
    }
}

impl eframe::App for RedsailGuiApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Continuously service Wayland / X11 compositor ping events to prevent ANR popups
        ctx.request_repaint_after(Duration::from_millis(100));

        // Handle drag-and-drop of multiple SVG files onto the window
        ctx.input(|i| {
            for dropped in &i.raw.dropped_files {
                if let Some(path) = &dropped.path {
                    self.add_svg_from_path(path);
                }
            }
        });

        // Poll file picker responses from background zenity thread
        while let Ok(paths) = self.file_picker_rx.try_recv() {
            for p in paths {
                self.add_svg_from_path(&p);
            }
        }

        // Poll background serial worker responses
        while let Ok(resp) = self.resp_rx.try_recv() {
            match resp {
                SerialResponse::Log(text) => self.log(text),
                SerialResponse::CutProgress { sent, total } => {
                    self.cut_progress_bytes = sent;
                    self.cut_total_bytes = total;
                    ctx.request_repaint();
                }
                SerialResponse::CutCompleted(success) => {
                    self.is_cutting = false;
                    if success {
                        self.log("[Success] Cut job completed successfully!".to_string());
                        self.cutter_x_steps = 0;
                        self.cutter_y_steps = 0;
                        self.est_pos_x = 0.0;
                        self.est_pos_y = 0.0;
                    } else {
                        self.log("Cut job stopped. Blade lifted.".to_string());
                    }
                    ctx.request_repaint();
                }
                SerialResponse::CommandCompleted => {
                    ctx.request_repaint();
                }
                SerialResponse::PortError(err) => {
                    self.is_cutting = false;
                    self.log(format!("[Error] {}", err));
                    ctx.request_repaint();
                }
            }
        }

        // Ensure transform cache is computed before UI reads dimensions
        self.ensure_transform_computed();

        // Top Panel: Header & Main Toolbar
        egui::TopBottomPanel::top("header_panel")
            .frame(egui::Frame::NONE.fill(Color32::from_rgb(18, 20, 26)).inner_margin(egui::Margin::symmetric(14, 8)))
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    // Logo and Title
                    ui.label(egui::RichText::new("PLOTTER").strong().size(16.0).color(Color32::from_rgb(0, 210, 255)));
                    ui.label(egui::RichText::new("Controller").size(15.0).color(Color32::from_rgb(180, 190, 210)));
                    ui.label(egui::RichText::new(concat!("v", env!("CARGO_PKG_VERSION"))).monospace().size(11.0).color(Color32::DARK_GRAY));

                    // Right-aligned header controls
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        // Emergency Stop Button
                        let halt_btn = egui::Button::new(
                            egui::RichText::new(format!("{} STOP", icons::STOP))
                                .color(Color32::WHITE)
                                .strong()
                                .size(13.0),
                        )
                        .fill(Color32::from_rgb(220, 38, 38));
                        if ui.add_sized([95.0, 26.0], halt_btn).on_hover_text("Emergency halt: immediately stops machine and lifts blade").clicked() {
                            self.cancel_cut();
                        }

                        ui.add_space(10.0);
                        ui.toggle_value(&mut self.developer_mode, format!("{} Dev", icons::GEAR));
                        ui.toggle_value(&mut self.show_console_log, format!("{} Console", icons::TERMINAL_WINDOW));
                        ui.toggle_value(&mut self.show_machine_panel, format!("{} Controls", icons::SLIDERS));
                    });
                });
            });

        // Bottom Status Bar
        egui::TopBottomPanel::bottom("bottom_status_bar")
            .frame(egui::Frame::NONE.fill(Color32::from_rgb(18, 20, 26)).inner_margin(egui::Margin::symmetric(14, 6)))
            .show(ctx, |ui| {
                if self.show_console_log {
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("Terminal / Serial Log").strong().size(11.5));
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.small_button("Clear").clicked() {
                                self.status_log.clear();
                            }
                            if ui.small_button(icons::X).on_hover_text("Close Console").clicked() {
                                self.show_console_log = false;
                            }
                        });
                    });
                    egui::ScrollArea::vertical()
                        .max_height(100.0)
                        .stick_to_bottom(true)
                        .show(ui, |ui| {
                            for line in &self.status_log {
                                ui.label(egui::RichText::new(line).monospace().size(11.0).color(Color32::from_rgb(180, 195, 220)));
                            }
                        });
                    ui.separator();
                }

                ui.horizontal(|ui| {
                    let last_log = self.status_log.last().map(|s| s.as_str()).unwrap_or("Ready.");
                    ui.label(egui::RichText::new("Status:").size(11.5).strong().color(Color32::GRAY));
                    ui.label(egui::RichText::new(last_log).size(11.5).color(Color32::from_rgb(220, 225, 235)));

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(egui::RichText::new(format!("{} baud | {}", self.baud, self.flow_mode)).size(11.0).color(Color32::DARK_GRAY));
                        ui.separator();
                        let cut_dist = {
                            self.ensure_transform_computed();
                            RedsailGuiApp::total_cut_length_mm(self.machine_cache.as_ref().unwrap())
                        };
                        ui.label(egui::RichText::new(format!("Total Cut Distance: {:.1} mm", cut_dist)).size(11.0).color(Color32::from_rgb(0, 180, 240)));
                    });
                });
            });

        // Left Panel: Design & Cut Studio
        egui::SidePanel::left("control_panel")
            .min_width(360.0)
            .max_width(420.0)
            .show(ctx, |ui| {
                ui.add_space(6.0);

                // --- 1. Design Files & Hierarchy ---
                ui.group(|ui| {
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new(format!("{} Artwork & Layers", icons::FOLDER_OPEN)).strong().size(12.5));
                        let tot_files = self.loaded_docs.len();
                        let sel_p = self.selected_paths_count();
                        let tot_p = self.total_paths_count();
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.label(egui::RichText::new(format!("{} files | {}/{} paths", tot_files, sel_p, tot_p)).size(11.0).color(Color32::from_rgb(0, 210, 255)));
                        });
                    });

                    ui.add_space(4.0);

                    ui.horizontal(|ui| {
                        if ui
                            .button(format!("{} Browse / Add Files...", icons::FOLDER_OPEN))
                            .on_hover_text("Open native file dialog to select one or multiple SVG files")
                            .clicked()
                        {
                            self.open_multi_file_dialog();
                        }
                        if !self.loaded_docs.is_empty()
                            && ui
                                .button("Clear All")
                                .on_hover_text("Remove all loaded SVG files")
                                .clicked()
                        {
                            self.clear_all_documents();
                        }
                    });

                    ui.horizontal(|ui| {
                        ui.text_edit_singleline(&mut self.svg_path_input);
                        if ui.button("+ Add").clicked() {
                            self.load_svg();
                        }
                    });

                    ui.horizontal(|ui| {
                        if self.developer_mode {
                            ui.label(egui::RichText::new("Presets:").size(11.0).color(Color32::GRAY));
                            if ui.small_button("+ Test Cut").on_hover_text("Add embedded calibration pattern").clicked() {
                                self.add_svg_content(EMBEDDED_TEST_CUT_SVG, "test_cut.svg", None);
                            }
                            if ui.small_button("+ Axis Test").on_hover_text("Add embedded axis alignment pattern").clicked() {
                                self.add_svg_content(EMBEDDED_AXIS_TEST_SVG, "axis_test.svg", None);
                            }
                        } else {
                            ui.label(egui::RichText::new("Select:").size(11.0).color(Color32::GRAY));
                        }
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.small_button("Invert").on_hover_text("Invert selection").clicked() {
                                self.invert_selection();
                            }
                            if ui.small_button("None").on_hover_text("Deselect all").clicked() {
                                self.select_all(false);
                            }
                            if ui.small_button("All").on_hover_text("Select all").clicked() {
                                self.select_all(true);
                            }
                        });
                    });

                    ui.add_space(4.0);

                    // Scrollable File / Layer / Group / Path Tree
                    egui::ScrollArea::vertical()
                        .max_height(190.0)
                        .show(ui, |ui| {
                            if self.loaded_docs.is_empty() {
                                ui.colored_label(
                                    Color32::DARK_GRAY,
                                    "No SVG files loaded. Click 'Browse / Add Files...' or drop SVGs here.",
                                );
                            } else {
                                let mut remove_idx = None;
                                let mut tree_dirty = false;
                                for (idx, doc) in self.loaded_docs.iter_mut().enumerate() {
                                    let d_total = doc.total_paths();
                                    let d_sel = doc.selected_paths();
                                    let is_all = d_sel == d_total && d_total > 0;
                                    let is_none = d_sel == 0;

                                    ui.horizontal(|ui| {
                                        let arrow = if doc.expanded { icons::CARET_DOWN } else { icons::CARET_RIGHT };
                                        if ui.small_button(arrow).clicked() {
                                            doc.expanded = !doc.expanded;
                                        }

                                        let mut checked = is_all;
                                        if dot_checkbox(ui, &mut checked, "").clicked() {
                                            if is_all {
                                                doc.set_selected_recursive(false);
                                            } else {
                                                doc.set_selected_recursive(true);
                                            }
                                            self.transform_dirty = true;
                                        }

                                        let doc_color = if is_all {
                                            Color32::from_rgb(0, 220, 255)
                                        } else if is_none {
                                            Color32::DARK_GRAY
                                        } else {
                                            Color32::from_rgb(255, 200, 50)
                                        };

                                        ui.label(
                                            egui::RichText::new(format!(
                                                "{} {} ({}/{})",
                                                icons::FILE_SVG, doc.filename, d_sel, d_total
                                            ))
                                            .strong()
                                            .size(11.5)
                                            .color(doc_color),
                                        );

                                        ui.with_layout(
                                            egui::Layout::right_to_left(egui::Align::Center),
                                            |ui| {
                                                if ui
                                                    .small_button(icons::TRASH)
                                                    .on_hover_text("Remove this file")
                                                    .clicked()
                                                {
                                                    remove_idx = Some(idx);
                                                }
                                            },
                                        );
                                    });

                                    if doc.expanded {
                                        for sub_grp in &mut doc.root_group.groups {
                                            render_group_item(ui, sub_grp, 1, &mut tree_dirty);
                                        }
                                        for path in &mut doc.root_group.paths {
                                            render_path_item(ui, path, 1, &mut tree_dirty);
                                        }
                                    }
                                }

                                if tree_dirty {
                                    self.transform_dirty = true;
                                }

                                if let Some(i) = remove_idx {
                                    self.loaded_docs.remove(i);
                                    self.update_dimensions_from_docs();
                                    self.log("Removed file from project.".to_string());
                                }
                            }
                        });
                });

                ui.add_space(6.0);

                // --- 2. Sizing & Position ---
                ui.group(|ui| {
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new(format!("{} Size & Position", icons::RULER)).strong().size(12.0));
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.small_button("200%").clicked() && self.orig_width_mm > 0.0 {
                                self.scale = 2.0;
                                self.custom_width_mm = (self.orig_width_mm * 2.0 * 10.0).round() / 10.0;
                                self.custom_height_mm = (self.orig_height_mm * 2.0 * 10.0).round() / 10.0;
                                self.transform_dirty = true;
                            }
                            if ui.small_button("100%").clicked() && self.orig_width_mm > 0.0 {
                                self.scale = 1.0;
                                self.custom_width_mm = self.orig_width_mm;
                                self.custom_height_mm = self.orig_height_mm;
                                self.transform_dirty = true;
                            }
                            if ui.small_button("50%").clicked() && self.orig_width_mm > 0.0 {
                                self.scale = 0.5;
                                self.custom_width_mm = (self.orig_width_mm * 0.5 * 10.0).round() / 10.0;
                                self.custom_height_mm = (self.orig_height_mm * 0.5 * 10.0).round() / 10.0;
                                self.transform_dirty = true;
                            }
                        });
                    });

                    // Width / Height row with aspect ratio badge
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("Width:").strong()).on_hover_text("Artwork width in millimetres");
                        let w_resp = ui.add(
                            egui::DragValue::new(&mut self.custom_width_mm)
                                .speed(0.5)
                                .range(1.0..=2000.0)
                                .max_decimals(1)
                                .suffix(" mm"),
                        );
                        if w_resp.changed() {
                            self.transform_dirty = true;
                            if self.lock_aspect_ratio && self.orig_width_mm > 0.0 {
                                let ratio = self.custom_width_mm / self.orig_width_mm;
                                self.custom_height_mm = (self.orig_height_mm * ratio * 10.0).round() / 10.0;
                                self.scale = ratio;
                            }
                        }

                        ui.add_space(12.0);

                        ui.label(egui::RichText::new("Height:").strong()).on_hover_text("Artwork height in millimetres");
                        let h_resp = ui.add(
                            egui::DragValue::new(&mut self.custom_height_mm)
                                .speed(0.5)
                                .range(1.0..=2000.0)
                                .max_decimals(1)
                                .suffix(" mm"),
                        );
                        if h_resp.changed() {
                            self.transform_dirty = true;
                            if self.lock_aspect_ratio && self.orig_height_mm > 0.0 {
                                let ratio = self.custom_height_mm / self.orig_height_mm;
                                self.custom_width_mm = (self.orig_width_mm * ratio * 10.0).round() / 10.0;
                                self.scale = ratio;
                            }
                        }

                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            let lock_icon = if self.lock_aspect_ratio {
                                format!("{} Locked", icons::LOCK)
                            } else {
                                format!("{} Free", icons::LOCK_OPEN)
                            };
                            let lock_changed = dot_checkbox(ui, &mut self.lock_aspect_ratio, &lock_icon).changed();
                            if lock_changed {
                                self.transform_dirty = true;
                            }
                        });
                    });

                    // Position row
                    ui.add_space(4.0);
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("Pos X:").strong()).on_hover_text("Horizontal offset from origin");
                        let px_resp = ui.add(
                            egui::DragValue::new(&mut self.pos_x_mm)
                                .speed(0.5)
                                .range(0.0..=2000.0)
                                .max_decimals(1)
                                .suffix(" mm"),
                        );

                        ui.add_space(12.0);

                        ui.label(egui::RichText::new("Pos Y:").strong()).on_hover_text("Vertical offset from origin");
                        let py_resp = ui.add(
                            egui::DragValue::new(&mut self.pos_y_mm)
                                .speed(0.5)
                                .range(0.0..=5000.0)
                                .max_decimals(1)
                                .suffix(" mm"),
                        );

                        if px_resp.changed() || py_resp.changed() {
                            self.transform_dirty = true;
                        }

                        if (self.pos_x_mm != 0.0 || self.pos_y_mm != 0.0)
                            && ui.small_button("Reset 0,0").on_hover_text("Reset position offsets to 0,0").clicked()
                        {
                            self.pos_x_mm = 0.0;
                            self.pos_y_mm = 0.0;
                            self.transform_dirty = true;
                        }
                    });

                    ui.horizontal(|ui| {
                        dot_checkbox(ui, &mut self.pos_active_only, "Active Cut Only")
                            .on_hover_text("When checked, position offsets and zero alignment apply strictly to the selected/active cut paths");
                        if self.pos_active_only {
                            ui.label(
                                egui::RichText::new("(active layer only)")
                                    .size(10.0)
                                    .color(Color32::from_rgb(0, 210, 255)),
                            );
                        } else {
                            ui.label(
                                egui::RichText::new("(all artwork)")
                                    .size(10.0)
                                    .color(Color32::from_rgb(140, 150, 170)),
                            );
                        }
                    });

                    ui.horizontal(|ui| {
                        let lock_icon = if self.lock_aspect_ratio {
                            format!("{} Aspect Ratio: Locked", icons::LOCK)
                        } else {
                            format!("{} Aspect Ratio: Free", icons::LOCK_OPEN)
                        };
                        dot_checkbox(ui, &mut self.lock_aspect_ratio, &lock_icon);
                        ui.separator();

                        // Axis controls — Canvas (display) vs Machine (cutting)
                        ui.collapsing("Axis Controls", |ui| {
                            ui.horizontal_wrapped(|ui| {
                                ui.label("Canvas:");
                                let cf_x = dot_checkbox(ui, &mut self.canvas_flip_x, "Flip X").changed();
                                let cf_y = dot_checkbox(ui, &mut self.canvas_flip_y, "Flip Y").changed();
                                let cf_s = dot_checkbox(ui, &mut self.canvas_swap_xy, "Swap X/Y").changed();
                                if cf_x || cf_y || cf_s { self.transform_dirty = true; }
                            });
                            ui.horizontal_wrapped(|ui| {
                                ui.label("Machine:");
                                let mf_x = dot_checkbox(ui, &mut self.machine_flip_x, "Flip X").changed();
                                let mf_y = dot_checkbox(ui, &mut self.machine_flip_y, "Flip Y").changed();
                                let mf_s = dot_checkbox(ui, &mut self.machine_swap_xy, "Swap X/Y").changed();
                                if mf_x || mf_y || mf_s { self.transform_dirty = true; }
                            });
                        });
                    });

                    // Weeding Border inside Dimensions
                    ui.separator();
                    ui.horizontal(|ui| {
                        let weeding_changed = dot_checkbox(ui, &mut self.add_weeding_box, "Weeding Border").changed();
                        if self.add_weeding_box {
                            let margin_changed = ui.add(
                                egui::Slider::new(&mut self.weeding_margin, 1.0..=25.0)
                                    .step_by(0.5)
                                    .suffix(" mm"),
                            ).changed();
                            if weeding_changed || margin_changed {
                                self.transform_dirty = true;
                            }
                        } else if weeding_changed {
                            self.transform_dirty = true;
                        }
                    });

                    if self.developer_mode {
                        ui.separator();
                        ui.label(egui::RichText::new("Axes Alignment (canvas):").size(11.0).color(Color32::GRAY));
                        ui.horizontal(|ui| {
                            let nz_changed = dot_checkbox(ui, &mut self.normalize_origin, "Auto-Zero").changed();
                            let cf_x = dot_checkbox(ui, &mut self.canvas_flip_x, "Flip X").changed();
                            let cf_y = dot_checkbox(ui, &mut self.canvas_flip_y, "Flip Y").changed();
                            if nz_changed || cf_x || cf_y {
                                self.transform_dirty = true;
                            }
                        });
                    }
                });

                ui.add_space(6.0);

                // --- 3. Cutter Settings & Material ---
                ui.group(|ui| {
                    ui.label(egui::RichText::new(format!("{} Material & Cut Settings", icons::SLIDERS)).strong().size(12.0));

                    egui::ComboBox::from_id_salt("material_preset_combo")
                        .selected_text(self.selected_preset.name())
                        .show_ui(ui, |ui| {
                            for preset in MaterialPreset::all() {
                                if ui.selectable_value(&mut self.selected_preset, *preset, preset.name()).clicked() {
                                    let (sp, fo, off) = preset.settings();
                                    if *preset != MaterialPreset::Custom {
                                        self.speed = sp;
                                        self.force = fo;
                                        self.blade_offset = off;
                                        self.transform_dirty = true;
                                        self.log(format!("Applied preset: {} (Speed: {} cm/s, Force: {} g, Offset: {:.2} mm)", preset.name(), sp, fo, off));
                                    }
                                }
                            }
                        });

                    ui.add_space(4.0);

                    if ui.add(egui::Slider::new(&mut self.speed, 1..=60).text("Speed (cm/s)")).changed() {
                        self.selected_preset = MaterialPreset::Custom;
                    }
                    if ui.add(egui::Slider::new(&mut self.force, 10..=400).text("Force (g)")).changed() {
                        self.selected_preset = MaterialPreset::Custom;
                    }
                    if self.developer_mode
                        && ui.add(
                            egui::Slider::new(&mut self.blade_offset, 0.0..=1.0)
                                .step_by(0.05)
                                .text("Blade Offset (mm)"),
                        ).changed()
                    {
                        self.selected_preset = MaterialPreset::Custom;
                        self.transform_dirty = true;
                    }

                    if self.developer_mode
                        && ui.add(
                            egui::Slider::new(&mut self.flatten_tolerance, 0.02..=1.0)
                                .step_by(0.01)
                                .text("Flatten Tolerance (mm)"),
                        ).changed()
                    {
                        self.selected_preset = MaterialPreset::Custom;
                        self.transform_dirty = true;
                        self.log(format!("Flatten tolerance changed to {:.2} mm — re-load SVGs to apply.", self.flatten_tolerance));
                    }

                    if self.developer_mode
                        && ui.add(
                            egui::Slider::new(&mut self.max_cut_width, 50.0..=2000.0)
                                .step_by(10.0)
                                .text("Max Cut Width (mm)"),
                        ).changed()
                    {
                        self.selected_preset = MaterialPreset::Custom;
                        self.transform_dirty = true;
                    }

                    if self.developer_mode
                        && ui.add(
                            egui::Slider::new(&mut self.min_feature, 0.0..=5.0)
                                .step_by(0.05)
                                .text("Min Feature Size (mm)"),
                        ).changed()
                    {
                        self.selected_preset = MaterialPreset::Custom;
                        self.transform_dirty = true;
                    }
                });

                ui.add_space(8.0);

                // --- 4. Cut Execution Action Bar ---
                ui.group(|ui| {
                    ui.label(egui::RichText::new(format!("{} Job Execution", icons::PLAY)).strong().size(12.0));

                    let cut_btn = egui::Button::new(
                        egui::RichText::new(if self.is_cutting {
                            "CUTTING IN PROGRESS...".to_string()
                        } else {
                            format!("{} CUT NOW", icons::PLAY)
                        })
                        .color(Color32::WHITE)
                        .strong()
                        .size(14.0),
                    )
                    .fill(if self.is_cutting {
                        Color32::from_rgb(180, 120, 0)
                    } else {
                        Color32::from_rgb(22, 163, 74)
                    });

                    if ui.add_sized([ui.available_width(), 38.0], cut_btn).clicked() {
                        self.execute_cut(false);
                    }

                    ui.add_space(4.0);

                    ui.horizontal(|ui| {
                        let count = if self.developer_mode { 3.0 } else { 2.0 };
                        let spacing = (count - 1.0) * 6.0;
                        let btn_w = ((ui.available_width() - spacing) / count).max(40.0);

                        if ui.add_sized([btn_w, 28.0], egui::Button::new(format!("{} Dry Run", icons::EYE)))
                            .on_hover_text("Simulate cut trajectory without sending to hardware")
                            .clicked()
                        {
                            self.execute_cut(true);
                        }

                        if ui.add_sized([btn_w, 28.0], egui::Button::new(format!("{} Test Box", icons::SQUARE)))
                            .on_hover_text("Trace outer rectangle (min_x, min_y to max_x, max_y) with blade lifted (PU) to visually verify roll width, length, and pinch rollers without cutting")
                            .clicked()
                        {
                            self.execute_test_box();
                        }

                        if self.developer_mode
                            && ui.add_sized([btn_w, 28.0], egui::Button::new(format!("{} Test Cut", icons::SCISSORS)))
                                .on_hover_text("Cut small calibration pattern (square & circle)")
                                .clicked()
                        {
                            self.execute_test_pattern();
                        }
                    });

                    if self.is_cutting {
                        ui.add_space(6.0);
                        let sent = self.cut_progress_bytes;
                        let total = self.cut_total_bytes.max(1);
                        let progress = (sent as f32 / total as f32).clamp(0.0, 1.0);
                        ui.add(egui::ProgressBar::new(progress).text(format!("Sending: {:.0}% ({}/{} B)", progress * 100.0, sent, total)));
                        ui.add_space(4.0);
                        let cancel_btn = egui::Button::new(format!("{} Cancel Cut", icons::X)).fill(Color32::from_rgb(220, 38, 38));
                        if ui.add_sized([ui.available_width(), 28.0], cancel_btn).clicked() {
                            self.cancel_cut();
                        }
                    }
                });
            });

        // Right Panel: Machine Workstation
        if self.show_machine_panel {
            egui::SidePanel::right("machine_control_panel")
                .min_width(280.0)
                .max_width(320.0)
                .show(ctx, |ui| {
                    ui.add_space(6.0);
                    ui.label(egui::RichText::new("Machine Workstation").heading().size(15.0));
                    ui.add_space(4.0);

                    // --- 1. Serial Connection ---
                    ui.group(|ui| {
                        ui.horizontal(|ui| {
                            ui.label(egui::RichText::new("Serial Connection").strong().size(12.0));
                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                if ui.small_button(icons::ARROW_CLOCKWISE).on_hover_text("Refresh Serial Devices").clicked() {
                                    self.refresh_ports();
                                }
                            });
                        });

                        ui.add_space(2.0);

                        // Connection Status Pill
                        let has_cutter = self.ports.iter().any(|p| p.name == self.selected_port && p.is_likely_cutter);
                        let has_port = !self.selected_port.is_empty();

                        let (pill_bg, dot_color, status_text) = if has_cutter {
                            (Color32::from_rgb(16, 42, 30), Color32::from_rgb(34, 197, 94), "Cutter Ready")
                        } else if has_port {
                            (Color32::from_rgb(44, 36, 20), Color32::from_rgb(234, 179, 8), "Connected")
                        } else {
                            (Color32::from_rgb(34, 36, 44), Color32::from_rgb(148, 163, 184), "Offline / No Device")
                        };

                        egui::Frame::NONE
                            .fill(pill_bg)
                            .corner_radius(egui::CornerRadius::same(6))
                            .inner_margin(egui::Margin::symmetric(8, 4))
                            .show(ui, |ui| {
                                ui.horizontal(|ui| {
                                    ui.colored_label(dot_color, icons::PLUG);
                                    ui.label(egui::RichText::new(status_text).size(11.0).strong().color(Color32::WHITE));
                                });
                            });

                        ui.add_space(3.0);

                        let old_port = self.selected_port.clone();
                        let selected_label = if self.selected_port.is_empty() {
                            "Select Device...".to_string()
                        } else {
                            self.selected_port.clone()
                        };
                        egui::ComboBox::from_id_salt("right_panel_port_combo")
                            .selected_text(selected_label)
                            .width(ui.available_width() - 8.0)
                            .show_ui(ui, |ui| {
                                if self.ports.is_empty() {
                                    ui.label(egui::RichText::new("No serial devices detected").italics().color(Color32::GRAY));
                                } else {
                                    for p in &self.ports {
                                        let label = if p.is_likely_cutter {
                                            format!("{} (Cutter)", p.name)
                                        } else {
                                            p.name.clone()
                                        };
                                        let resp = ui.selectable_value(&mut self.selected_port, p.name.clone(), label);
                                        if !p.description.is_empty() {
                                            resp.on_hover_text(&p.description);
                                        }
                                    }
                                }
                            });
                        if self.selected_port != old_port {
                            self.notify_port_config_changed();
                        }

                        if self.developer_mode {
                            ui.add_space(4.0);
                            ui.horizontal(|ui| {
                                ui.label("Baud:");
                                if ui.radio_value(&mut self.baud, 9600, "9600").changed()
                                    || ui.radio_value(&mut self.baud, 38400, "38400").changed()
                                {
                                    self.notify_port_config_changed();
                                }
                            });

                            ui.horizontal(|ui| {
                                ui.label("Flow:");
                                let old_flow = self.flow_mode;
                                egui::ComboBox::from_id_salt("right_flow_ctrl_combo")
                                    .selected_text(self.flow_mode.to_string())
                                    .show_ui(ui, |ui| {
                                        ui.selectable_value(&mut self.flow_mode, FlowControlMode::None, "Paced");
                                        ui.selectable_value(&mut self.flow_mode, FlowControlMode::Hardware, "RTS/CTS");
                                        ui.selectable_value(&mut self.flow_mode, FlowControlMode::Software, "XON/XOFF");
                                    });
                                if self.flow_mode != old_flow {
                                    self.notify_port_config_changed();
                                }
                            });
                        }
                    });

                    ui.add_space(4.0);

                    // --- 2. Digital Readout (DRO) & Zeroing ---
                    ui.group(|ui| {
                        ui.horizontal(|ui| {
                            ui.label(egui::RichText::new("Head Coordinates (DRO)").strong().size(12.0));
                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                if ui.small_button("Zero").on_hover_text("Set current carriage position as 0,0").clicked() {
                                    self.set_origin_here();
                                }
                            });
                        });

                        // Monospace LCD Display Card
                        egui::Frame::NONE
                            .fill(Color32::from_rgb(14, 16, 22))
                            .corner_radius(egui::CornerRadius::same(6))
                            .stroke(Stroke::new(1.0_f32, Color32::from_rgb(38, 42, 58)))
                            .inner_margin(egui::Margin::symmetric(10, 8))
                            .show(ui, |ui| {
                                ui.horizontal(|ui| {
                                    ui.label(egui::RichText::new("X:").monospace().color(Color32::from_rgb(255, 100, 100)));
                                    ui.label(egui::RichText::new(format!("{:>+6.1} mm", self.est_pos_x)).monospace().strong().size(14.0).color(Color32::WHITE));
                                    ui.separator();
                                    ui.label(egui::RichText::new("Y:").monospace().color(Color32::from_rgb(100, 255, 120)));
                                    ui.label(egui::RichText::new(format!("{:>+6.1} mm", self.est_pos_y)).monospace().strong().size(14.0).color(Color32::WHITE));
                                });
                            });

                        ui.add_space(3.0);
                        ui.horizontal(|ui| {
                            ui.label("Go X:");
                            ui.add(egui::DragValue::new(&mut self.target_move_x).speed(1.0).max_decimals(1).suffix(" mm"));
                            ui.label("Y:");
                            ui.add(egui::DragValue::new(&mut self.target_move_y).speed(1.0).max_decimals(1).suffix(" mm"));
                            if ui.button("Go").on_hover_text("Rapidly travel tool head to absolute coordinates (blade lifted)").clicked() {
                                self.move_to_position(self.target_move_x, self.target_move_y);
                            }
                        });

                        ui.add_space(4.0);

                        ui.vertical_centered_justified(|ui| {
                            if ui.button(format!("{} Set Origin (Here)", icons::CROSSHAIR)).on_hover_text("Zero coordinates at current blade position without moving motors").clicked() {
                                self.set_origin_here();
                            }
                            if ui.button(format!("{} Return to Origin (0,0)", icons::HOUSE)).on_hover_text("Physically rapid travel tool carriage and feed back to 0,0").clicked() {
                                self.go_home();
                            }
                        });
                    });

                    ui.add_space(4.0);

                    // --- 3. Precision Jog Controller ---
                    ui.group(|ui| {
                        ui.label(egui::RichText::new("Precision Jog").strong().size(12.0));

                        ui.horizontal_wrapped(|ui| {
                            ui.label(egui::RichText::new("Step:").size(11.0).color(Color32::GRAY));
                            for &s in &[1.0, 5.0, 10.0, 50.0, 100.0] {
                                let label = format!("{}mm", s as i32);
                                let selected = (self.jog_step_mm - s).abs() < 0.1;
                                if ui.selectable_label(selected, label).clicked() {
                                    self.jog_step_mm = s;
                                }
                            }
                        });

                        ui.add_space(6.0);

                        ui.vertical_centered(|ui| {
                            if ui.add_sized([130.0, 28.0], egui::Button::new(format!("{} Feed Out (+)", icons::ARROW_UP))).on_hover_text("Feed vinyl forward").clicked() {
                                self.jog_feed(self.jog_step_mm);
                            }

                            ui.add_space(2.0);

                            ui.horizontal(|ui| {
                                let total_w = 78.0 + 72.0 + 78.0 + 8.0;
                                ui.add_space(((ui.available_width() - total_w) / 2.0).max(0.0));
                                if ui.add_sized([78.0, 28.0], egui::Button::new(format!("{} Left", icons::ARROW_LEFT))).on_hover_text("Move carriage left").clicked() {
                                    self.jog_carriage(self.jog_step_mm);
                                }
                                if ui.add_sized([72.0, 28.0], egui::Button::new(format!("{} Home", icons::HOUSE))).on_hover_text("Return to Origin 0,0").clicked() {
                                    self.go_home();
                                }
                                if ui.add_sized([78.0, 28.0], egui::Button::new(format!("Right {}", icons::ARROW_RIGHT))).on_hover_text("Move carriage right").clicked() {
                                    self.jog_carriage(-self.jog_step_mm);
                                }
                            });

                            ui.add_space(2.0);

                            if ui.add_sized([130.0, 28.0], egui::Button::new(format!("{} Retract (-)", icons::ARROW_DOWN))).on_hover_text("Retract vinyl backward").clicked() {
                                self.jog_feed(-self.jog_step_mm);
                            }
                        });
                    });

                    ui.add_space(4.0);

                    // --- 4. Vinyl Media Feed & Diagnostics ---
                    ui.group(|ui| {
                        ui.label(egui::RichText::new("Media Feed & Tool").strong().size(12.0));

                        ui.horizontal(|ui| {
                            let w = ((ui.available_width() - 8.0) / 2.0).max(40.0);
                            if ui.add_sized([w, 26.0], egui::Button::new("+100 mm Feed")).on_hover_text("Advance vinyl 100mm forward").clicked() {
                                self.feed_out(100.0);
                            }
                            if ui.add_sized([w, 26.0], egui::Button::new("-100 mm Retract")).on_hover_text("Pull vinyl 100mm back").clicked() {
                                self.feed_out(-100.0);
                            }
                        });

                        if self.developer_mode {
                            ui.add_space(4.0);
                            ui.horizontal(|ui| {
                                let w = ((ui.available_width() - 16.0) / 3.0).max(35.0);
                                if ui.add_sized([w, 26.0], egui::Button::new("Tap")).on_hover_text("Quickly touch blade down and lift").clicked() {
                                    self.pen_tap();
                                }
                                if ui.add_sized([w, 26.0], egui::Button::new("Drop")).on_hover_text("Lower blade/pen").clicked() {
                                    self.pen_drop();
                                }
                                if ui.add_sized([w, 26.0], egui::Button::new("Lift")).on_hover_text("Raise blade/pen").clicked() {
                                    self.pen_lift();
                                }
                            });

                            ui.add_space(4.0);
                            if ui.add_sized([ui.available_width(), 26.0], egui::Button::new(format!("{} Axis Test Pattern", icons::COMPASS))).on_hover_text("Cut 20x20mm test pattern with X/Y directional indicators").clicked() {
                                self.execute_axis_test_pattern();
                            }
                            if ui.add_sized([ui.available_width(), 26.0], egui::Button::new("Reset Machine (IN;SP1;)")).clicked() {
                                self.machine_reset();
                            }
                        }
                    });
                });
        }

        // Central Panel: Interactive CAD Canvas
        egui::CentralPanel::default().show(ctx, |ui| {
            // Top Toolbar inside Canvas
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new("Interactive CAD Canvas").strong().size(13.0));
                ui.separator();
                ui.label({
                    let mut w = self.custom_width_mm;
                    let mut h = self.custom_height_mm;
                    if let Some(out) = &self.canvas_cache {
                        let mut min_x = f64::MAX;
                        let mut min_y = f64::MAX;
                        let mut max_x = f64::MIN;
                        let mut max_y = f64::MIN;
                        for path in &out.active_paths {
                            for pt in path {
                                if pt.x < min_x { min_x = pt.x; }
                                if pt.x > max_x { max_x = pt.x; }
                                if pt.y < min_y { min_y = pt.y; }
                                if pt.y > max_y { max_y = pt.y; }
                            }
                        }
                        if min_x <= max_x && min_y <= max_y {
                            w = (max_x - min_x).max(0.0);
                            h = (max_y - min_y).max(0.0);
                        }
                    }
                    egui::RichText::new(format!("Cut Dimensions: {:.1} mm × {:.1} mm", w, h))
                        .strong()
                        .color(Color32::from_rgb(0, 200, 255))
                });

                let sel_p = self.selected_paths_count();
                let tot_p = self.total_paths_count();
                let sel_color = if sel_p > 0 {
                    Color32::from_rgb(0, 210, 255)
                } else {
                    Color32::from_rgb(255, 100, 100)
                };
                ui.label(
                    egui::RichText::new(format!("|  Paths: {}/{} Selected", sel_p, tot_p))
                        .color(sel_color),
                );

                if self.add_weeding_box {
                    ui.label(
                        egui::RichText::new(format!(
                            "|  Weeding: {:.1} mm × {:.1} mm",
                            self.custom_width_mm + (self.weeding_margin * 2.0),
                            self.custom_height_mm + (self.weeding_margin * 2.0)
                        ))
                        .color(Color32::from_rgb(255, 215, 0)),
                    );
                }

                ui.separator();
                egui::ComboBox::from_id_salt("canvas_origin_selector")
                    .selected_text(self.canvas_origin_corner.name())
                    .show_ui(ui, |ui| {
                        ui.selectable_value(
                            &mut self.canvas_origin_corner,
                            CanvasOriginCorner::BottomRight,
                            "Bottom-Right (Machine Standard)",
                        );
                        ui.selectable_value(
                            &mut self.canvas_origin_corner,
                            CanvasOriginCorner::BottomLeft,
                            "Bottom-Left (CAD Standard)",
                        );
                        ui.selectable_value(
                            &mut self.canvas_origin_corner,
                            CanvasOriginCorner::TopLeft,
                            "Top-Left (Screen Standard)",
                        );
                    });

                ui.separator();
                let ruler_btn = if self.show_rulers {
                    egui::Button::new(format!("{} Ruler", icons::RULER))
                        .fill(Color32::from_rgb(0, 100, 140))
                } else {
                    egui::Button::new(format!("{} Ruler", icons::RULER))
                };
                if ui
                    .add(ruler_btn)
                    .on_hover_text("Toggle metric CAD edge rulers on/off")
                    .clicked()
                {
                    self.show_rulers = !self.show_rulers;
                }

                // Canvas Zoom Controls on the Right
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.small_button(format!("{} Fit View", icons::ARROWS_OUT)).on_hover_text("Reset Pan and Zoom to fit artwork").clicked() {
                        self.canvas_zoom = 1.0;
                        self.canvas_pan = Vec2::ZERO;
                    }
                    if ui.small_button("1:1").on_hover_text("Reset to 100% Zoom").clicked() {
                        self.canvas_zoom = 1.0;
                    }
                    if ui.small_button(icons::MAGNIFYING_GLASS_MINUS).on_hover_text("Zoom Out").clicked() {
                        self.canvas_zoom = (self.canvas_zoom * 0.85).max(0.2);
                    }
                    ui.label(egui::RichText::new(format!("{:.0}%", self.canvas_zoom * 100.0)).size(11.0).monospace().color(Color32::GRAY));
                    if ui.small_button(icons::MAGNIFYING_GLASS_PLUS).on_hover_text("Zoom In").clicked() {
                        self.canvas_zoom = (self.canvas_zoom * 1.15).min(8.0);
                    }
                });
            });
            ui.separator();

            let (response, painter) =
                ui.allocate_painter(ui.available_size(), egui::Sense::drag());
            let canvas_rect = response.rect;

            // Pan with mouse drag
            if response.dragged_by(egui::PointerButton::Secondary)
                || response.dragged_by(egui::PointerButton::Middle)
                || (response.dragged_by(egui::PointerButton::Primary) && ui.input(|i| i.modifiers.alt))
            {
                self.canvas_pan += response.drag_delta();
            }

            // Zoom with scroll wheel
            let scroll = ctx.input(|i| i.raw_scroll_delta.y);
            if canvas_rect.contains(ctx.input(|i| i.pointer.hover_pos().unwrap_or_default())) && scroll.abs() > 0.1 {
                let zoom_factor = if scroll > 0.0 { 1.08 } else { 0.92 };
                self.canvas_zoom = (self.canvas_zoom * zoom_factor).clamp(0.2, 8.0);
            }

            // Draw deep CAD canvas background
            painter.rect_filled(canvas_rect, 0.0, Color32::from_rgb(17, 18, 24));

            let base_origin = match self.canvas_origin_corner {
                CanvasOriginCorner::BottomLeft => canvas_rect.left_bottom() + Vec2::new(50.0, -50.0),
                CanvasOriginCorner::BottomRight => canvas_rect.right_bottom() - Vec2::new(50.0, 50.0),
                CanvasOriginCorner::TopLeft => canvas_rect.left_top() + Vec2::new(50.0, 50.0),
            };
            let origin = base_origin + self.canvas_pan;

            // Transform paths (cached — only recomputes when transform-relevant state changes)
            self.ensure_transform_computed();
            let output = self.canvas_cache.as_ref().unwrap().clone();
            let has_paths = !output.active_paths.is_empty() || !output.inactive_paths.is_empty();

            // Compute actual output bounds (accounts for mirror, scale, flip, etc.)
            let (out_min_x, out_min_y, out_max_x, out_max_y) = {
                let mut min_x = f64::MAX;
                let mut min_y = f64::MAX;
                let mut max_x = f64::MIN;
                let mut max_y = f64::MIN;
                for path in &output.active_paths {
                    for pt in path {
                        if pt.x < min_x { min_x = pt.x; }
                        if pt.x > max_x { max_x = pt.x; }
                        if pt.y < min_y { min_y = pt.y; }
                        if pt.y > max_y { max_y = pt.y; }
                    }
                }
                if let Some(wbox) = &output.weeding_box {
                    for pt in wbox {
                        if pt.x < min_x { min_x = pt.x; }
                        if pt.x > max_x { max_x = pt.x; }
                        if pt.y < min_y { min_y = pt.y; }
                        if pt.y > max_y { max_y = pt.y; }
                    }
                }
                if min_x <= max_x && min_y <= max_y {
                    (min_x, min_y, max_x, max_y)
                } else {
                    (0.0, 0.0, 0.0, 0.0)
                }
            };

            let max_w = ((out_max_x - out_min_x).max(0.0) + (self.weeding_margin * 2.0)).max(10.0);
            let max_h = ((out_max_y - out_min_y).max(0.0) + (self.weeding_margin * 2.0)).max(10.0);
            let max_dim = (max_w.max(max_h)) as f32;
            let avail_w = (canvas_rect.width() - 120.0).max(50.0);
            let avail_h = (canvas_rect.height() - 120.0).max(50.0);
            let auto_fit_zoom = (avail_w / max_dim).min(avail_h / max_dim).clamp(0.2, 10.0);
            let effective_zoom = auto_fit_zoom * self.canvas_zoom;

            let origin_corner = self.canvas_origin_corner;

            // Canvas shows design as-is; canvas and HP-GL use identical transform output.
            let (min_sx, max_sx) = {
                let mut min_val = f32::MAX;
                let mut max_val = f32::MIN;
                let mut check_pt = |pt: &kurbo::Point| {
                    let sx = pt.x as f32;
                    if sx < min_val { min_val = sx; }
                    if sx > max_val { max_val = sx; }
                };
                if let Some(wbox) = &output.weeding_box {
                    for pt in wbox { check_pt(pt); }
                }
                for p in &output.active_paths {
                    for pt in p { check_pt(pt); }
                }
                for p in &output.inactive_paths {
                    for pt in p { check_pt(pt); }
                }
                if min_val > max_val {
                    (0.0_f32, (self.custom_width_mm + self.pos_x_mm + (self.weeding_margin * 2.0)).max(10.0) as f32)
                } else {
                    (min_val, max_val)
                }
            };

            let to_screen = |pt: kurbo::Point| -> Pos2 {
                // Canvas and HP-GL use identical transform output — no swap_xy applied.
                let sx = pt.x as f32;
                let sy = pt.y as f32;
                match origin_corner {
                    CanvasOriginCorner::BottomRight => {
                        Pos2::new(
                            origin.x - ((max_sx + min_sx - sx) * effective_zoom),
                            origin.y - (sy * effective_zoom),
                        )
                    }
                    CanvasOriginCorner::BottomLeft => {
                        Pos2::new(origin.x + (sx * effective_zoom), origin.y - (sy * effective_zoom))
                    }
                    CanvasOriginCorner::TopLeft => {
                        Pos2::new(origin.x + (sx * effective_zoom), origin.y + (sy * effective_zoom))
                    }
                }
            };

            // Draw Metric CAD Grid (10mm minor, 50mm major)
            if effective_zoom > 0.3 {
                let minor_step = if effective_zoom > 1.8 { 10.0 } else { 50.0 };
                let grid_w = ((self.custom_width_mm + self.pos_x_mm).max(100.0) * 1.3) as f32;
                let grid_h = ((self.custom_height_mm + self.pos_y_mm).max(100.0) * 1.3) as f32;

                let minor_stroke = Stroke::new(0.5_f32, Color32::from_rgb(26, 28, 38));
                let major_stroke = Stroke::new(0.8_f32, Color32::from_rgb(38, 42, 58));

                let mut gx = 0.0;
                while gx <= grid_w {
                    let is_major = (gx as i32 % 50) == 0;
                    let (p0, p1) = match origin_corner {
                        CanvasOriginCorner::BottomLeft => (
                            Pos2::new(origin.x + (gx * effective_zoom), origin.y),
                            Pos2::new(origin.x + (gx * effective_zoom), origin.y - (grid_h * effective_zoom)),
                        ),
                        CanvasOriginCorner::BottomRight => (
                            Pos2::new(origin.x - (gx * effective_zoom), origin.y),
                            Pos2::new(origin.x - (gx * effective_zoom), origin.y - (grid_h * effective_zoom)),
                        ),
                        CanvasOriginCorner::TopLeft => (
                            Pos2::new(origin.x + (gx * effective_zoom), origin.y),
                            Pos2::new(origin.x + (gx * effective_zoom), origin.y + (grid_h * effective_zoom)),
                        ),
                    };
                    if canvas_rect.x_range().contains(p0.x) || canvas_rect.x_range().contains(p1.x) {
                        painter.line_segment([p0, p1], if is_major { major_stroke } else { minor_stroke });
                    }
                    gx += minor_step;
                }

                let mut gy = 0.0;
                while gy <= grid_h {
                    let is_major = (gy as i32 % 50) == 0;
                    let (p0, p1) = match origin_corner {
                        CanvasOriginCorner::BottomLeft => (
                            Pos2::new(origin.x, origin.y - (gy * effective_zoom)),
                            Pos2::new(origin.x + (grid_w * effective_zoom), origin.y - (gy * effective_zoom)),
                        ),
                        CanvasOriginCorner::BottomRight => (
                            Pos2::new(origin.x, origin.y - (gy * effective_zoom)),
                            Pos2::new(origin.x - (grid_w * effective_zoom), origin.y - (gy * effective_zoom)),
                        ),
                        CanvasOriginCorner::TopLeft => (
                            Pos2::new(origin.x, origin.y + (gy * effective_zoom)),
                            Pos2::new(origin.x + (grid_w * effective_zoom), origin.y + (gy * effective_zoom)),
                        ),
                    };
                    if canvas_rect.y_range().contains(p0.y) || canvas_rect.y_range().contains(p1.y) {
                        painter.line_segment([p0, p1], if is_major { major_stroke } else { minor_stroke });
                    }
                    gy += minor_step;
                }
            }

            // Origin marker (0, 0)
            painter.circle_filled(origin, 5.0, Color32::from_rgb(239, 68, 68));
            painter.circle_stroke(origin, 8.0, Stroke::new(1.0_f32, Color32::from_rgb(239, 68, 68)));

            // Draw axis guides
            let axis_len = 65.0 * effective_zoom.clamp(0.6, 1.2);
            match self.canvas_origin_corner {
                CanvasOriginCorner::BottomLeft => {
                    painter.text(
                        origin + Vec2::new(10.0, 10.0),
                        egui::Align2::LEFT_TOP,
                        "Origin (0,0)",
                        egui::FontId::monospace(11.0),
                        Color32::from_rgb(240, 240, 240),
                    );

                    // +Width (Carriage) extends Rightward
                    painter.line_segment(
                        [origin, origin + Vec2::new(axis_len, 0.0)],
                        Stroke::new(1.5_f32, Color32::from_rgb(239, 68, 68)),
                    );
                    painter.text(
                        origin + Vec2::new(axis_len + 4.0, 0.0),
                        egui::Align2::LEFT_CENTER,
                        "+Width (Carriage)",
                        egui::FontId::monospace(10.0),
                        Color32::from_rgb(255, 120, 120),
                    );

                    // +Length (Feed) extends Upward
                    painter.line_segment(
                        [origin, origin + Vec2::new(0.0, -axis_len)],
                        Stroke::new(1.5_f32, Color32::from_rgb(34, 197, 94)),
                    );
                    painter.text(
                        origin + Vec2::new(0.0, -axis_len - 4.0),
                        egui::Align2::CENTER_BOTTOM,
                        "+Length (Feed)",
                        egui::FontId::monospace(10.0),
                        Color32::from_rgb(100, 255, 140),
                    );
                }
                CanvasOriginCorner::BottomRight => {
                    painter.text(
                        origin + Vec2::new(-10.0, 10.0),
                        egui::Align2::RIGHT_TOP,
                        "Origin (0,0)",
                        egui::FontId::monospace(11.0),
                        Color32::from_rgb(240, 240, 240),
                    );

                    // +Width (Carriage) extends Leftward
                    painter.line_segment(
                        [origin, origin + Vec2::new(-axis_len, 0.0)],
                        Stroke::new(1.5_f32, Color32::from_rgb(239, 68, 68)),
                    );
                    painter.text(
                        origin + Vec2::new(-axis_len - 4.0, 0.0),
                        egui::Align2::RIGHT_CENTER,
                        "+Width (Carriage)",
                        egui::FontId::monospace(10.0),
                        Color32::from_rgb(255, 120, 120),
                    );

                    // +Length (Feed) extends Upward
                    painter.line_segment(
                        [origin, origin + Vec2::new(0.0, -axis_len)],
                        Stroke::new(1.5_f32, Color32::from_rgb(34, 197, 94)),
                    );
                    painter.text(
                        origin + Vec2::new(0.0, -axis_len - 4.0),
                        egui::Align2::CENTER_BOTTOM,
                        "+Length (Feed)",
                        egui::FontId::monospace(10.0),
                        Color32::from_rgb(100, 255, 140),
                    );
                }
                CanvasOriginCorner::TopLeft => {
                    painter.text(
                        origin + Vec2::new(10.0, -10.0),
                        egui::Align2::LEFT_BOTTOM,
                        "Origin (0,0)",
                        egui::FontId::monospace(11.0),
                        Color32::from_rgb(240, 240, 240),
                    );

                    // +Width (Carriage) extends Rightward
                    painter.line_segment(
                        [origin, origin + Vec2::new(axis_len, 0.0)],
                        Stroke::new(1.5_f32, Color32::from_rgb(239, 68, 68)),
                    );
                    painter.text(
                        origin + Vec2::new(axis_len + 4.0, 0.0),
                        egui::Align2::LEFT_CENTER,
                        "+Width (Carriage)",
                        egui::FontId::monospace(10.0),
                        Color32::from_rgb(255, 120, 120),
                    );

                    // +Length (Feed) extends Downward
                    painter.line_segment(
                        [origin, origin + Vec2::new(0.0, axis_len)],
                        Stroke::new(1.5_f32, Color32::from_rgb(34, 197, 94)),
                    );
                    painter.text(
                        origin + Vec2::new(0.0, axis_len + 4.0),
                        egui::Align2::CENTER_TOP,
                        "+Length (Feed)",
                        egui::FontId::monospace(10.0),
                        Color32::from_rgb(100, 255, 140),
                    );
                }
            }

            if has_paths {
                // 1. Draw inactive / unselected paths in dimmed dark slate
                let inactive_stroke = Stroke::new(1.0_f32, Color32::from_rgb(60, 65, 80));
                for path in &output.inactive_paths {
                    if path.len() < 2 {
                        continue;
                    }
                    for i in 0..path.len() - 1 {
                        let p0 = to_screen(path[i]);
                        let p1 = to_screen(path[i + 1]);

                        if canvas_rect.contains(p0) || canvas_rect.contains(p1) {
                            painter.line_segment([p0, p1], inactive_stroke);
                        }
                    }
                }

                // 2. Draw active / selected paths in vibrant Neon Cyan
                let active_stroke = Stroke::new(1.8_f32, Color32::from_rgb(0, 210, 255));
                for path in &output.active_paths {
                    if path.len() < 2 {
                        continue;
                    }
                    for i in 0..path.len() - 1 {
                        let p0 = to_screen(path[i]);
                        let p1 = to_screen(path[i + 1]);

                        if canvas_rect.contains(p0) || canvas_rect.contains(p1) {
                            painter.line_segment([p0, p1], active_stroke);
                        }
                    }
                }

                // 3. Draw Weeding Box in Gold / Amber
                if let Some(wbox) = &output.weeding_box {
                    let box_stroke = Stroke::new(2.0_f32, Color32::from_rgb(245, 158, 11));
                    for i in 0..wbox.len() - 1 {
                        let p0 = to_screen(wbox[i]);
                        let p1 = to_screen(wbox[i + 1]);
                        painter.line_segment([p0, p1], box_stroke);
                    }
                    let p_wbox_label = to_screen(wbox[0]);
                    let (wbox_offset, wbox_align) = match self.canvas_origin_corner {
                        CanvasOriginCorner::BottomLeft | CanvasOriginCorner::BottomRight => {
                            (Vec2::new(4.0, -4.0), egui::Align2::LEFT_BOTTOM)
                        }
                        CanvasOriginCorner::TopLeft => (Vec2::new(4.0, 4.0), egui::Align2::LEFT_TOP),
                    };
                    painter.text(
                        p_wbox_label + wbox_offset,
                        wbox_align,
                        "Weeding Border",
                        egui::FontId::monospace(10.0),
                        Color32::from_rgb(245, 158, 11),
                    );
                }

                // 4. Floating Legend Card
                let legend_rect = match self.canvas_origin_corner {
                    CanvasOriginCorner::BottomLeft => egui::Rect::from_min_size(
                        canvas_rect.right_bottom() - Vec2::new(190.0, 98.0),
                        Vec2::new(180.0, 62.0),
                    ),
                    CanvasOriginCorner::BottomRight => egui::Rect::from_min_size(
                        canvas_rect.left_bottom() + Vec2::new(16.0, -98.0),
                        Vec2::new(180.0, 62.0),
                    ),
                    CanvasOriginCorner::TopLeft => egui::Rect::from_min_size(
                        canvas_rect.right_bottom() - Vec2::new(190.0, 70.0),
                        Vec2::new(180.0, 62.0),
                    ),
                };
                painter.rect(
                    legend_rect,
                    egui::CornerRadius::same(6),
                    Color32::from_rgba_unmultiplied(16, 18, 24, 210),
                    Stroke::new(1.0_f32, Color32::from_rgb(38, 42, 56)),
                    egui::StrokeKind::Outside,
                );

                let p_leg = legend_rect.left_top() + Vec2::new(10.0, 12.0);
                painter.line_segment([p_leg, p_leg + Vec2::new(18.0, 0.0)], active_stroke);
                painter.text(p_leg + Vec2::new(24.0, 0.0), egui::Align2::LEFT_CENTER, "Active (To Cut)", egui::FontId::proportional(11.0), Color32::from_rgb(0, 210, 255));

                let p_leg2 = p_leg + Vec2::new(0.0, 18.0);
                painter.line_segment([p_leg2, p_leg2 + Vec2::new(18.0, 0.0)], inactive_stroke);
                painter.text(p_leg2 + Vec2::new(24.0, 0.0), egui::Align2::LEFT_CENTER, "Inactive (Excluded)", egui::FontId::proportional(11.0), Color32::from_rgb(140, 150, 170));

                let p_leg3 = p_leg2 + Vec2::new(0.0, 18.0);
                painter.circle_filled(p_leg3 + Vec2::new(9.0, 0.0), 3.5, Color32::from_rgb(239, 68, 68));
                painter.text(p_leg3 + Vec2::new(24.0, 0.0), egui::Align2::LEFT_CENTER, "Plotter Origin (0,0)", egui::FontId::proportional(11.0), Color32::WHITE);
            } else {
                painter.text(
                    canvas_rect.center(),
                    egui::Align2::CENTER_CENTER,
                    "No SVG files loaded. Click 'Browse / Add Files...' or drop SVGs here.",
                    egui::FontId::proportional(16.0),
                    Color32::from_rgb(80, 85, 105),
                );
            }

            // Metric CAD Edge Rulers (Origin-Synchronized)
            if self.show_rulers {
                let ruler_th_h = 20.0_f32;
                let ruler_th_w = 26.0_f32;

                // Match ruler edges directly with the active origin corner
                let (h_ruler_rect, v_ruler_rect, corner_rect) = match origin_corner {
                    CanvasOriginCorner::BottomLeft => {
                        let h = egui::Rect::from_min_max(
                            Pos2::new(canvas_rect.min.x + ruler_th_w, canvas_rect.max.y - ruler_th_h),
                            canvas_rect.max,
                        );
                        let v = egui::Rect::from_min_max(
                            Pos2::new(canvas_rect.min.x, canvas_rect.min.y),
                            Pos2::new(canvas_rect.min.x + ruler_th_w, canvas_rect.max.y - ruler_th_h),
                        );
                        let c = egui::Rect::from_min_max(
                            Pos2::new(canvas_rect.min.x, canvas_rect.max.y - ruler_th_h),
                            Pos2::new(canvas_rect.min.x + ruler_th_w, canvas_rect.max.y),
                        );
                        (h, v, c)
                    }
                    CanvasOriginCorner::BottomRight => {
                        let h = egui::Rect::from_min_max(
                            Pos2::new(canvas_rect.min.x, canvas_rect.max.y - ruler_th_h),
                            Pos2::new(canvas_rect.max.x - ruler_th_w, canvas_rect.max.y),
                        );
                        let v = egui::Rect::from_min_max(
                            Pos2::new(canvas_rect.max.x - ruler_th_w, canvas_rect.min.y),
                            Pos2::new(canvas_rect.max.x, canvas_rect.max.y - ruler_th_h),
                        );
                        let c = egui::Rect::from_min_max(
                            Pos2::new(canvas_rect.max.x - ruler_th_w, canvas_rect.max.y - ruler_th_h),
                            canvas_rect.max,
                        );
                        (h, v, c)
                    }
                    CanvasOriginCorner::TopLeft => {
                        let h = egui::Rect::from_min_max(
                            Pos2::new(canvas_rect.min.x + ruler_th_w, canvas_rect.min.y),
                            Pos2::new(canvas_rect.max.x, canvas_rect.min.y + ruler_th_h),
                        );
                        let v = egui::Rect::from_min_max(
                            Pos2::new(canvas_rect.min.x, canvas_rect.min.y + ruler_th_h),
                            Pos2::new(canvas_rect.min.x + ruler_th_w, canvas_rect.max.y),
                        );
                        let c = egui::Rect::from_min_max(
                            canvas_rect.min,
                            Pos2::new(canvas_rect.min.x + ruler_th_w, canvas_rect.min.y + ruler_th_h),
                        );
                        (h, v, c)
                    }
                };

                // Ruler background strips
                let ruler_bg = Color32::from_rgb(22, 24, 32);
                let ruler_border = Stroke::new(1.0_f32, Color32::from_rgb(45, 50, 68));
                painter.rect_filled(h_ruler_rect, 0.0, ruler_bg);
                match origin_corner {
                    CanvasOriginCorner::BottomLeft | CanvasOriginCorner::BottomRight => {
                        painter.line_segment([h_ruler_rect.left_top(), h_ruler_rect.right_top()], ruler_border);
                    }
                    CanvasOriginCorner::TopLeft => {
                        painter.line_segment([h_ruler_rect.left_bottom(), h_ruler_rect.right_bottom()], ruler_border);
                    }
                }

                painter.rect_filled(v_ruler_rect, 0.0, ruler_bg);
                match origin_corner {
                    CanvasOriginCorner::BottomLeft | CanvasOriginCorner::TopLeft => {
                        painter.line_segment([v_ruler_rect.right_top(), v_ruler_rect.right_bottom()], ruler_border);
                    }
                    CanvasOriginCorner::BottomRight => {
                        painter.line_segment([v_ruler_rect.left_top(), v_ruler_rect.left_bottom()], ruler_border);
                    }
                }

                // Corner junction badge
                painter.rect_filled(corner_rect, 0.0, Color32::from_rgb(28, 31, 42));
                match origin_corner {
                    CanvasOriginCorner::BottomLeft => {
                        painter.line_segment([corner_rect.left_top(), corner_rect.right_top()], ruler_border);
                        painter.line_segment([corner_rect.right_top(), corner_rect.right_bottom()], ruler_border);
                    }
                    CanvasOriginCorner::BottomRight => {
                        painter.line_segment([corner_rect.left_top(), corner_rect.right_top()], ruler_border);
                        painter.line_segment([corner_rect.left_top(), corner_rect.left_bottom()], ruler_border);
                    }
                    CanvasOriginCorner::TopLeft => {
                        painter.line_segment([corner_rect.left_bottom(), corner_rect.right_bottom()], ruler_border);
                        painter.line_segment([corner_rect.right_top(), corner_rect.right_bottom()], ruler_border);
                    }
                }
                painter.text(
                    corner_rect.center(),
                    egui::Align2::CENTER_CENTER,
                    "mm",
                    egui::FontId::monospace(10.0),
                    Color32::from_rgb(0, 200, 255),
                );

                // Zero-alignment dashed guides linking ruler 0 ticks directly to Plotter Origin (0,0) marker
                let zero_guide_stroke = Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(239, 68, 68, 140));
                if canvas_rect.x_range().contains(origin.x) {
                    let (y0, y1) = match origin_corner {
                        CanvasOriginCorner::BottomLeft | CanvasOriginCorner::BottomRight => (origin.y, h_ruler_rect.min.y),
                        CanvasOriginCorner::TopLeft => (h_ruler_rect.max.y, origin.y),
                    };
                    if (y1 - y0).abs() > 3.0 {
                        painter.line_segment([Pos2::new(origin.x, y0), Pos2::new(origin.x, y1)], zero_guide_stroke);
                    }
                }
                if canvas_rect.y_range().contains(origin.y) {
                    let (x0, x1) = match origin_corner {
                        CanvasOriginCorner::BottomLeft => (v_ruler_rect.max.x, origin.x),
                        CanvasOriginCorner::BottomRight => (origin.x, v_ruler_rect.min.x),
                        CanvasOriginCorner::TopLeft => (v_ruler_rect.max.x, origin.x),
                    };
                    if (x1 - x0).abs() > 3.0 {
                        painter.line_segment([Pos2::new(x0, origin.y), Pos2::new(x1, origin.y)], zero_guide_stroke);
                    }
                }

                // Adaptive step sizing based on zoom
                let (step_major, step_mid, step_minor) = if effective_zoom > 4.0 {
                    (10.0_f64, 5.0_f64, 1.0_f64)
                } else if effective_zoom > 1.8 {
                    (20.0_f64, 10.0_f64, 2.0_f64)
                } else if effective_zoom > 0.8 {
                    (50.0_f64, 25.0_f64, 5.0_f64)
                } else if effective_zoom > 0.35 {
                    (100.0_f64, 50.0_f64, 10.0_f64)
                } else {
                    (200.0_f64, 100.0_f64, 20.0_f64)
                };

                // 1. Horizontal Ruler Graduations (Width / Carriage)
                if effective_zoom > 0.001 && h_ruler_rect.width() > 10.0 {
                    let h_raw_a = match origin_corner {
                        CanvasOriginCorner::BottomLeft | CanvasOriginCorner::TopLeft => ((h_ruler_rect.min.x - origin.x) / effective_zoom) as f64,
                        CanvasOriginCorner::BottomRight => ((origin.x - h_ruler_rect.max.x) / effective_zoom) as f64,
                    };
                    let h_raw_b = match origin_corner {
                        CanvasOriginCorner::BottomLeft | CanvasOriginCorner::TopLeft => ((h_ruler_rect.max.x - origin.x) / effective_zoom) as f64,
                        CanvasOriginCorner::BottomRight => ((origin.x - h_ruler_rect.min.x) / effective_zoom) as f64,
                    };
                    if h_raw_a.is_finite() && h_raw_b.is_finite() {
                        let (h_min_mm, h_max_mm) = if h_raw_a < h_raw_b { (h_raw_a, h_raw_b) } else { (h_raw_b, h_raw_a) };
                        let span = (h_max_mm - h_min_mm).max(1.0);
                        let eff_step = if span / step_minor > 150.0 { step_major } else { step_minor };
                        let h_start_idx = (h_min_mm / eff_step).floor() as i32;
                        let h_end_idx = (h_max_mm / eff_step).ceil() as i32;
                        let clamped_end = h_end_idx.min(h_start_idx.saturating_add(200));

                        for idx in h_start_idx..=clamped_end {
                            let val_mm = (idx as f64) * eff_step;
                            let screen_x = match origin_corner {
                                CanvasOriginCorner::BottomLeft | CanvasOriginCorner::TopLeft => origin.x + (val_mm as f32 * effective_zoom),
                                CanvasOriginCorner::BottomRight => origin.x - (val_mm as f32 * effective_zoom),
                            };

                            if screen_x >= h_ruler_rect.min.x && screen_x <= h_ruler_rect.max.x {
                                let is_major = (val_mm.round() as i64).rem_euclid(step_major as i64) == 0;
                                let is_mid = !is_major && (val_mm.round() as i64).rem_euclid(step_mid as i64) == 0;

                                let (tick_len, stroke) = if is_major {
                                    (10.0_f32, Stroke::new(1.0_f32, Color32::from_rgb(170, 185, 210)))
                                } else if is_mid {
                                    (6.0_f32, Stroke::new(0.8_f32, Color32::from_rgb(110, 120, 145)))
                                } else {
                                    (3.5_f32, Stroke::new(0.6_f32, Color32::from_rgb(70, 78, 98)))
                                };

                                let (p_base, p_tip) = match origin_corner {
                                    CanvasOriginCorner::BottomLeft | CanvasOriginCorner::BottomRight => (
                                        Pos2::new(screen_x, h_ruler_rect.min.y),
                                        Pos2::new(screen_x, h_ruler_rect.min.y + tick_len),
                                    ),
                                    CanvasOriginCorner::TopLeft => (
                                        Pos2::new(screen_x, h_ruler_rect.max.y),
                                        Pos2::new(screen_x, h_ruler_rect.max.y - tick_len),
                                    ),
                                };
                                painter.line_segment([p_base, p_tip], stroke);

                                if is_major {
                                    let label = format!("{:.0}", val_mm);
                                    let (label_pos, align) = match origin_corner {
                                        CanvasOriginCorner::BottomLeft => (
                                            Pos2::new(screen_x + 2.0, h_ruler_rect.max.y - 2.0),
                                            egui::Align2::LEFT_BOTTOM,
                                        ),
                                        CanvasOriginCorner::BottomRight => (
                                            Pos2::new(screen_x - 2.0, h_ruler_rect.max.y - 2.0),
                                            egui::Align2::RIGHT_BOTTOM,
                                        ),
                                        CanvasOriginCorner::TopLeft => (
                                            Pos2::new(screen_x + 2.0, h_ruler_rect.min.y + 2.0),
                                            egui::Align2::LEFT_TOP,
                                        ),
                                    };
                                    painter.text(
                                        label_pos,
                                        align,
                                        label,
                                        egui::FontId::monospace(9.0),
                                        if val_mm.abs() < 0.001 {
                                             Color32::from_rgb(239, 68, 68)
                                        } else {
                                            Color32::from_rgb(170, 185, 210)
                                        },
                                    );
                                }
                            }
                        }
                    }
                }

                // 2. Vertical Ruler Graduations (Length / Feed)
                if effective_zoom > 0.001 && v_ruler_rect.height() > 10.0 {
                    let v_raw_a = match origin_corner {
                        CanvasOriginCorner::BottomLeft | CanvasOriginCorner::BottomRight => ((origin.y - v_ruler_rect.max.y) / effective_zoom) as f64,
                        CanvasOriginCorner::TopLeft => ((v_ruler_rect.min.y - origin.y) / effective_zoom) as f64,
                    };
                    let v_raw_b = match origin_corner {
                        CanvasOriginCorner::BottomLeft | CanvasOriginCorner::BottomRight => ((origin.y - v_ruler_rect.min.y) / effective_zoom) as f64,
                        CanvasOriginCorner::TopLeft => ((v_ruler_rect.max.y - origin.y) / effective_zoom) as f64,
                    };
                    if v_raw_a.is_finite() && v_raw_b.is_finite() {
                        let (v_min_mm, v_max_mm) = if v_raw_a < v_raw_b { (v_raw_a, v_raw_b) } else { (v_raw_b, v_raw_a) };
                        let span = (v_max_mm - v_min_mm).max(1.0);
                        let eff_step = if span / step_minor > 150.0 { step_major } else { step_minor };
                        let v_start_idx = (v_min_mm / eff_step).floor() as i32;
                        let v_end_idx = (v_max_mm / eff_step).ceil() as i32;
                        let clamped_end = v_end_idx.min(v_start_idx.saturating_add(200));

                        for idx in v_start_idx..=clamped_end {
                            let val_mm = (idx as f64) * eff_step;
                            let screen_y = match origin_corner {
                                CanvasOriginCorner::BottomLeft | CanvasOriginCorner::BottomRight => origin.y - (val_mm as f32 * effective_zoom),
                                CanvasOriginCorner::TopLeft => origin.y + (val_mm as f32 * effective_zoom),
                            };

                            if screen_y >= v_ruler_rect.min.y && screen_y <= v_ruler_rect.max.y {
                                let is_major = (val_mm.round() as i64).rem_euclid(step_major as i64) == 0;
                                let is_mid = !is_major && (val_mm.round() as i64).rem_euclid(step_mid as i64) == 0;

                                let (tick_len, stroke) = if is_major {
                                    (10.0_f32, Stroke::new(1.0_f32, Color32::from_rgb(170, 185, 210)))
                                } else if is_mid {
                                    (6.0_f32, Stroke::new(0.8_f32, Color32::from_rgb(110, 120, 145)))
                                } else {
                                    (3.5_f32, Stroke::new(0.6_f32, Color32::from_rgb(70, 78, 98)))
                                };

                                let (p_base, p_tip) = match origin_corner {
                                    CanvasOriginCorner::BottomLeft | CanvasOriginCorner::TopLeft => (
                                        Pos2::new(v_ruler_rect.max.x, screen_y),
                                        Pos2::new(v_ruler_rect.max.x - tick_len, screen_y),
                                    ),
                                    CanvasOriginCorner::BottomRight => (
                                        Pos2::new(v_ruler_rect.min.x, screen_y),
                                        Pos2::new(v_ruler_rect.min.x + tick_len, screen_y),
                                    ),
                                };
                                painter.line_segment([p_base, p_tip], stroke);

                                if is_major {
                                    let label = format!("{:.0}", val_mm);
                                    let (label_pos, align) = match origin_corner {
                                        CanvasOriginCorner::BottomLeft | CanvasOriginCorner::TopLeft => (
                                            Pos2::new(v_ruler_rect.min.x + 2.0, screen_y - 2.0),
                                            egui::Align2::LEFT_BOTTOM,
                                        ),
                                        CanvasOriginCorner::BottomRight => (
                                            Pos2::new(v_ruler_rect.max.x - 2.0, screen_y - 2.0),
                                            egui::Align2::RIGHT_BOTTOM,
                                        ),
                                    };
                                    painter.text(
                                        label_pos,
                                        align,
                                        label,
                                        egui::FontId::monospace(8.0),
                                        if val_mm.abs() < 0.001 {
                                            Color32::from_rgb(239, 68, 68)
                                        } else {
                                            Color32::from_rgb(170, 185, 210)
                                        },
                                    );
                                }
                            }
                        }
                    }
                }

                // 3. Mouse Cursor Tracking on Rulers
                if let Some(hover_pos) = ctx.input(|i| i.pointer.hover_pos())
                    && canvas_rect.contains(hover_pos)
                {
                    let cursor_x_mm = match origin_corner {
                            CanvasOriginCorner::BottomLeft | CanvasOriginCorner::TopLeft => (hover_pos.x - origin.x) / effective_zoom,
                            CanvasOriginCorner::BottomRight => (origin.x - hover_pos.x) / effective_zoom,
                        };
                        let cursor_y_mm = match origin_corner {
                            CanvasOriginCorner::BottomLeft | CanvasOriginCorner::BottomRight => (origin.y - hover_pos.y) / effective_zoom,
                            CanvasOriginCorner::TopLeft => (hover_pos.y - origin.y) / effective_zoom,
                        };

                        // Hairline on horizontal ruler
                        if hover_pos.x >= h_ruler_rect.min.x && hover_pos.x <= h_ruler_rect.max.x {
                            painter.line_segment(
                                [Pos2::new(hover_pos.x, h_ruler_rect.min.y), Pos2::new(hover_pos.x, h_ruler_rect.max.y)],
                                Stroke::new(1.5_f32, Color32::from_rgb(0, 210, 255)),
                            );
                        }

                        // Hairline on vertical ruler
                        if hover_pos.y >= v_ruler_rect.min.y && hover_pos.y <= v_ruler_rect.max.y {
                            painter.line_segment(
                                [Pos2::new(v_ruler_rect.min.x, hover_pos.y), Pos2::new(v_ruler_rect.max.x, hover_pos.y)],
                                Stroke::new(1.5_f32, Color32::from_rgb(34, 197, 94)),
                            );
                        }

                        // Floating cursor coordinate pill (placed in opposite corner from rulers)
                        let pill_rect = match origin_corner {
                            CanvasOriginCorner::BottomLeft => egui::Rect::from_min_size(
                                Pos2::new(canvas_rect.max.x - 157.0, canvas_rect.min.y + 12.0),
                                Vec2::new(145.0, 20.0),
                            ),
                            CanvasOriginCorner::BottomRight => egui::Rect::from_min_size(
                                Pos2::new(canvas_rect.min.x + 12.0, canvas_rect.min.y + 12.0),
                                Vec2::new(145.0, 20.0),
                            ),
                            CanvasOriginCorner::TopLeft => egui::Rect::from_min_size(
                                Pos2::new(canvas_rect.max.x - 157.0, canvas_rect.max.y - 32.0),
                                Vec2::new(145.0, 20.0),
                            ),
                        };
                        painter.rect(
                            pill_rect,
                            egui::CornerRadius::same(4),
                            Color32::from_rgba_unmultiplied(16, 20, 30, 210),
                            Stroke::new(1.0_f32, Color32::from_rgb(45, 52, 70)),
                            egui::StrokeKind::Outside,
                        );
                        painter.text(
                            pill_rect.center(),
                            egui::Align2::CENTER_CENTER,
                            format!("X: {:.1} mm  Y: {:.1} mm", cursor_x_mm, cursor_y_mm),
                            egui::FontId::monospace(10.0),
                            Color32::from_rgb(200, 225, 255),
                        );
                    }
            }
        });
    }
}

#[allow(dead_code)]
pub mod icons {
    pub const ARROW_CLOCKWISE: &str = "\u{e036}";
    pub const ARROW_DOWN: &str = "\u{e03e}";
    pub const ARROW_LEFT: &str = "\u{e058}";
    pub const ARROW_RIGHT: &str = "\u{e06c}";
    pub const ARROW_UP: &str = "\u{e08e}";
    pub const ARROWS_OUT: &str = "\u{e0a2}";
    pub const CARET_DOWN: &str = "\u{e136}";
    pub const CARET_RIGHT: &str = "\u{e13a}";
    pub const COMPASS: &str = "\u{e1c8}";
    pub const CROSSHAIR: &str = "\u{e1d6}";
    pub const EYE: &str = "\u{e220}";
    pub const FILE_SVG: &str = "\u{ed08}";
    pub const FOLDER_OPEN: &str = "\u{e256}";
    pub const GEAR: &str = "\u{e270}";
    pub const HOUSE: &str = "\u{e2c2}";
    pub const LOCK: &str = "\u{e2fa}";
    pub const LOCK_OPEN: &str = "\u{e306}";
    pub const MAGNIFYING_GLASS_MINUS: &str = "\u{e30e}";
    pub const MAGNIFYING_GLASS_PLUS: &str = "\u{e310}";
    pub const PLAY: &str = "\u{e3d0}";
    pub const PLUG: &str = "\u{e946}";
    pub const RULER: &str = "\u{e6b8}";
    pub const SCISSORS: &str = "\u{eae0}";
    pub const SLIDERS: &str = "\u{e432}";
    pub const SQUARE: &str = "\u{e45e}";
    pub const STOP: &str = "\u{e46c}";
    pub const TARGET: &str = "\u{e47c}";
    pub const TERMINAL_WINDOW: &str = "\u{eae8}";
    pub const TRASH: &str = "\u{e4a6}";
    pub const X: &str = "\u{e4f6}";
}

fn setup_custom_theme(ctx: &egui::Context) {
    let mut fonts = egui::FontDefinitions::default();
    fonts.font_data.insert(
        "Phosphor".to_owned(),
        std::sync::Arc::new(egui::FontData::from_static(include_bytes!(
            "../assets/fonts/Phosphor.ttf"
        ))),
    );
    fonts
        .families
        .entry(egui::FontFamily::Proportional)
        .or_default()
        .push("Phosphor".to_owned());
    fonts
        .families
        .entry(egui::FontFamily::Monospace)
        .or_default()
        .push("Phosphor".to_owned());
    ctx.set_fonts(fonts);

    let mut visuals = egui::Visuals::dark();
    visuals.panel_fill = Color32::from_rgb(18, 20, 26);
    visuals.window_fill = Color32::from_rgb(22, 24, 32);
    visuals.extreme_bg_color = Color32::from_rgb(13, 14, 18);

    let radius = egui::CornerRadius::same(6);

    // Noninteractive
    visuals.widgets.noninteractive.bg_fill = Color32::from_rgb(26, 29, 38);
    visuals.widgets.noninteractive.fg_stroke = Stroke::new(1.0_f32, Color32::from_rgb(180, 188, 205));
    visuals.widgets.noninteractive.bg_stroke = Stroke::new(1.0_f32, Color32::from_rgb(38, 42, 56));
    visuals.widgets.noninteractive.corner_radius = radius;

    // Inactive
    visuals.widgets.inactive.bg_fill = Color32::from_rgb(30, 34, 46);
    visuals.widgets.inactive.fg_stroke = Stroke::new(1.0_f32, Color32::from_rgb(220, 226, 238));
    visuals.widgets.inactive.bg_stroke = Stroke::new(1.0_f32, Color32::from_rgb(46, 52, 70));
    visuals.widgets.inactive.corner_radius = radius;

    // Hovered
    visuals.widgets.hovered.bg_fill = Color32::from_rgb(42, 48, 66);
    visuals.widgets.hovered.fg_stroke = Stroke::new(1.2_f32, Color32::WHITE);
    visuals.widgets.hovered.bg_stroke = Stroke::new(1.2_f32, Color32::from_rgb(0, 180, 240));
    visuals.widgets.hovered.corner_radius = radius;

    // Active
    visuals.widgets.active.bg_fill = Color32::from_rgb(0, 140, 200);
    visuals.widgets.active.fg_stroke = Stroke::new(1.5_f32, Color32::WHITE);
    visuals.widgets.active.bg_stroke = Stroke::new(1.5_f32, Color32::from_rgb(0, 210, 255));
    visuals.widgets.active.corner_radius = radius;

    // Open
    visuals.widgets.open.bg_fill = Color32::from_rgb(34, 38, 52);
    visuals.widgets.open.corner_radius = radius;

    ctx.set_visuals(visuals);

    let mut style = (*ctx.style()).clone();
    style.spacing.item_spacing = Vec2::new(8.0, 6.0);
    style.spacing.button_padding = Vec2::new(10.0, 5.0);
    ctx.set_style(style);
}

pub fn run_gui() -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 800.0])
            .with_min_inner_size([960.0, 600.0])
            .with_title("Plotter - Vinyl Cutter Controller")
            .with_app_id("redsail-cutter"),
        ..Default::default()
    };

    eframe::run_native(
        "redsail-cutter",
        options,
        Box::new(|cc| {
            setup_custom_theme(&cc.egui_ctx);
            Ok(Box::new(RedsailGuiApp::default()))
        }),
    )
}
