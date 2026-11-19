use eframe::egui::{self, Color32, Pos2, Stroke, Vec2};
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::Arc;
use std::time::Duration;

use crate::blade_offset::BladeCompensator;
use crate::device::{list_ports, DetectedPort, FlowControlMode, RedsailDevice};
use crate::hpgl::{mm_to_plotter, CutterSettings, HpglBuilder};
use crate::svg_parser::{parse_svg, SvgDocument, SvgGroupItem, SvgPathItem};

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

const EMBEDDED_TEST_CUT_SVG: &str = include_str!("../test_cut.svg");
const EMBEDDED_AXIS_TEST_SVG: &str = include_str!("../axis_test.svg");

#[derive(Default, Clone)]
pub struct TransformedOutput {
    pub active_paths: Vec<Vec<kurbo::Point>>,
    pub inactive_paths: Vec<Vec<kurbo::Point>>,
    pub weeding_box: Option<Vec<kurbo::Point>>,
    #[allow(dead_code)]
    pub bounds: (f64, f64, f64, f64),
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

fn render_path_item(ui: &mut egui::Ui, path: &mut SvgPathItem, depth: usize) {
    ui.horizontal(|ui| {
        ui.add_space((depth as f32) * 14.0);
        ui.checkbox(&mut path.selected, "");
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

fn render_group_item(ui: &mut egui::Ui, group: &mut SvgGroupItem, depth: usize) {
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
        let arrow = if group.expanded { "▼" } else { "▶" };
        if ui.small_button(arrow).clicked() {
            group.expanded = !group.expanded;
        }

        let mut checked = is_all;
        if ui.checkbox(&mut checked, "").clicked() {
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
            render_group_item(ui, sub_grp, depth + 1);
        }
        for path in &mut group.paths {
            render_path_item(ui, path, depth + 1);
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
    scale: f64,
    custom_width_mm: f64,
    custom_height_mm: f64,
    orig_width_mm: f64,
    orig_height_mm: f64,
    lock_aspect_ratio: bool,
    normalize_origin: bool,
    flip_y: bool,
    swap_xy: bool,
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
}

impl Default for RedsailGuiApp {
    fn default() -> Self {
        let ports = list_ports();
        let selected_port = ports
            .iter()
            .find(|p| p.is_likely_cutter)
            .map(|p| p.name.clone())
            .unwrap_or_else(|| {
                ports.first().map(|p| p.name.clone()).unwrap_or_default()
            });

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
            svg_path_input: "test_cut.svg".to_string(),
            loaded_docs: Vec::new(),
            file_picker_tx,
            file_picker_rx,
            selected_preset: MaterialPreset::PenPlotter,
            speed: 20,
            force: 40,
            blade_offset: 0.0,
            scale: 1.0,
            custom_width_mm: 30.0,
            custom_height_mm: 30.0,
            orig_width_mm: 30.0,
            orig_height_mm: 30.0,
            lock_aspect_ratio: true,
            normalize_origin: true,
            flip_y: true,
            swap_xy: true,
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
        };

        app.load_svg();
        app
    }
}

impl RedsailGuiApp {
    fn total_cut_length_mm(&self) -> f64 {
        let output = self.transform_paths();
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
            self.selected_port = self
                .ports
                .iter()
                .find(|p| p.is_likely_cutter)
                .map(|p| p.name.clone())
                .unwrap_or_default();
        }
        self.notify_port_config_changed();
        self.log(format!("Refreshed ports: found {} device(s).", self.ports.len()));
    }

    fn open_multi_file_dialog(&self) {
        let tx = self.file_picker_tx.clone();
        std::thread::spawn(move || {
            let output = std::process::Command::new("zenity")
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
    }

    fn add_svg_content(&mut self, content: &str, label: &str, path: Option<&Path>) {
        match parse_svg(content, 0.05) {
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
        match fs::read_to_string(path) {
            Ok(content) => {
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
        self.log("Cleared all loaded design files.".to_string());
    }

    fn select_all(&mut self, sel: bool) {
        for doc in &mut self.loaded_docs {
            doc.set_selected_recursive(sel);
        }
    }

    fn invert_selection(&mut self) {
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
        let path = PathBuf::from(&self.svg_path_input);
        if path.exists() {
            self.add_svg_from_path(&path);
        } else if self.svg_path_input == "test_cut.svg" || self.svg_path_input.is_empty() {
            self.add_svg_content(EMBEDDED_TEST_CUT_SVG, "test_cut.svg", None);
        } else if self.svg_path_input == "axis_test.svg" {
            self.add_svg_content(EMBEDDED_AXIS_TEST_SVG, "axis_test.svg", None);
        } else {
            self.log(format!("File does not exist: {}", self.svg_path_input));
        }
    }

    fn transform_paths(&self) -> TransformedOutput {
        let mut all_paths: Vec<(Vec<kurbo::Point>, bool)> = Vec::new();
        for doc in &self.loaded_docs {
            doc.collect_all_paths(&mut all_paths);
        }

        if all_paths.is_empty() {
            return TransformedOutput::default();
        }

        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;

        for (path, _) in &all_paths {
            for pt in path {
                if pt.x < min_x { min_x = pt.x; }
                if pt.x > max_x { max_x = pt.x; }
                if pt.y < min_y { min_y = pt.y; }
                if pt.y > max_y { max_y = pt.y; }
            }
        }

        if min_x > max_x || min_y > max_y {
            min_x = 0.0; max_x = 0.0;
            min_y = 0.0; max_y = 0.0;
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

        let mut active_paths: Vec<Vec<kurbo::Point>> = Vec::new();
        let mut inactive_paths: Vec<Vec<kurbo::Point>> = Vec::new();

        let mut act_min_x = f64::MAX;
        let mut act_min_y = f64::MAX;
        let mut act_max_x = f64::MIN;
        let mut act_max_y = f64::MIN;

        for (path, is_active) in all_paths {
            let pts: Vec<kurbo::Point> = path
                .iter()
                .map(|pt| {
                    let norm_x = if self.normalize_origin {
                        pt.x - min_x
                    } else {
                        pt.x
                    };

                    let norm_y = if self.flip_y {
                        max_y - pt.y
                    } else if self.normalize_origin {
                        pt.y - min_y
                    } else {
                        pt.y
                    };

                    let mut x = norm_x * eff_scale_x + base_offset;
                    let mut y = norm_y * eff_scale_y + base_offset;

                    if self.swap_xy {
                        std::mem::swap(&mut x, &mut y);
                    }

                    kurbo::Point::new(x, y)
                })
                .collect();

            if is_active {
                for pt in &pts {
                    if pt.x < act_min_x { act_min_x = pt.x; }
                    if pt.x > act_max_x { act_max_x = pt.x; }
                    if pt.y < act_min_y { act_min_y = pt.y; }
                    if pt.y > act_max_y { act_max_y = pt.y; }
                }
                active_paths.push(pts);
            } else {
                inactive_paths.push(pts);
            }
        }

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
            bounds: (min_x, min_y, max_x, max_y),
        }
    }

    fn generate_current_hpgl(&self) -> Result<(String, usize), String> {
        let output = self.transform_paths();
        if output.active_paths.is_empty() {
            return Err("No vector paths selected to cut. Please check at least one layer or path in the Design Hierarchy tree.".to_string());
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
            builder.pen_up(mm_to_plotter(start.x), mm_to_plotter(start.y));
            for pt in &compensated[1..] {
                builder.pen_down(mm_to_plotter(pt.x), mm_to_plotter(pt.y));
            }
            builder.pen_up_in_place();
        }

        // 2. Cut outer weeding border last
        if let Some(wbox) = &output.weeding_box {
            let compensated_box = compensator.process_path(wbox);
            if !compensated_box.is_empty() {
                total_points += compensated_box.len();
                let start = compensated_box[0];
                builder.pen_up(mm_to_plotter(start.x), mm_to_plotter(start.y));
                for pt in &compensated_box[1..] {
                    builder.pen_down(mm_to_plotter(pt.x), mm_to_plotter(pt.y));
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

        let d_steps = (d_mm * 40.0).round() as i32;
        self.cutter_y_steps += d_steps;
        let cmd = format!("PA;PU{},{};", self.cutter_x_steps, self.cutter_y_steps);
        let desc = format!("Carriage move {:+.1} mm (Y={:.1} mm)", d_mm, self.cutter_y_steps as f64 / 40.0);
        self.send_raw_command(&cmd, &desc);
        self.est_pos_x = (self.cutter_y_steps as f64 / 40.0 * 10.0).round() / 10.0;
    }

    fn jog_feed(&mut self, d_mm: f64) {
        if self.is_cutting {
            self.log("[Blocked] Cannot jog while a cut job is in progress.".to_string());
            return;
        }

        let d_steps = (d_mm * 40.0).round() as i32;
        self.cutter_x_steps += d_steps;
        let cmd = format!("PA;PU{},{};", self.cutter_x_steps, self.cutter_y_steps);
        let desc = format!("Feed move {:+.1} mm (X={:.1} mm)", d_mm, self.cutter_x_steps as f64 / 40.0);
        self.send_raw_command(&cmd, &desc);
        self.est_pos_y = (self.cutter_x_steps as f64 / 40.0 * 10.0).round() / 10.0;
    }

    fn go_home(&mut self) {
        self.cutter_x_steps = 0;
        self.cutter_y_steps = 0;
        self.est_pos_x = 0.0;
        self.est_pos_y = 0.0;
        self.send_raw_command("PA;PU0,0;", "Return to Origin (0,0)");
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

        // Top Panel: Header & Main Toolbar
        egui::TopBottomPanel::top("header_panel")
            .frame(egui::Frame::NONE.fill(Color32::from_rgb(18, 20, 26)).inner_margin(egui::Margin::symmetric(14, 8)))
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    // Logo and Title
                    ui.label(egui::RichText::new("PLOTTER").strong().size(16.0).color(Color32::from_rgb(0, 210, 255)));
                    ui.label(egui::RichText::new("Controller").size(15.0).color(Color32::from_rgb(180, 190, 210)));
                    ui.label(egui::RichText::new("v0.1.0").monospace().size(11.0).color(Color32::DARK_GRAY));

                    ui.add_space(16.0);
                    ui.separator();
                    ui.add_space(16.0);

                    // Connection Status Pill
                    let has_cutter = self.ports.iter().any(|p| p.name == self.selected_port && p.is_likely_cutter);
                    let has_port = !self.selected_port.is_empty();

                    let (pill_bg, dot_color, status_text) = if has_cutter {
                        (Color32::from_rgb(16, 42, 30), Color32::from_rgb(34, 197, 94), format!("Cutter Ready: {}", self.selected_port))
                    } else if has_port {
                        (Color32::from_rgb(44, 36, 20), Color32::from_rgb(234, 179, 8), format!("Port: {}", self.selected_port))
                    } else {
                        (Color32::from_rgb(34, 36, 44), Color32::from_rgb(148, 163, 184), "Offline / No Port".to_string())
                    };

                    egui::Frame::NONE
                        .fill(pill_bg)
                        .corner_radius(egui::CornerRadius::same(12))
                        .inner_margin(egui::Margin::symmetric(10, 4))
                        .show(ui, |ui| {
                            ui.horizontal(|ui| {
                                ui.colored_label(dot_color, "●");
                                ui.label(egui::RichText::new(status_text).size(11.5).strong().color(Color32::WHITE));
                            });
                        });

                    // Port Dropdown right in header
                    ui.add_space(6.0);
                    let old_port = self.selected_port.clone();
                    egui::ComboBox::from_id_salt("header_port_select")
                        .selected_text(if self.selected_port.is_empty() { "Select Port".to_string() } else { self.selected_port.clone() })
                        .show_ui(ui, |ui| {
                            for p in &self.ports {
                                let label = if p.is_likely_cutter {
                                    format!("{} (Cutter)", p.name)
                                } else {
                                    p.name.clone()
                                };
                                ui.selectable_value(&mut self.selected_port, p.name.clone(), label);
                            }
                        });
                    if self.selected_port != old_port {
                        self.notify_port_config_changed();
                    }

                    if ui.small_button("↻").on_hover_text("Refresh Serial Ports").clicked() {
                        self.refresh_ports();
                    }

                    // Right-aligned header controls
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        // Emergency Stop Button
                        let halt_btn = egui::Button::new(
                            egui::RichText::new("🛑 STOP")
                                .color(Color32::WHITE)
                                .strong()
                                .size(12.0),
                        )
                        .fill(Color32::from_rgb(220, 38, 38));
                        if ui.add_sized([90.0, 26.0], halt_btn).on_hover_text("Emergency halt: immediately stops machine and lifts blade").clicked() {
                            self.cancel_cut();
                        }

                        ui.add_space(10.0);
                        ui.toggle_value(&mut self.developer_mode, "Developer");
                        ui.toggle_value(&mut self.show_console_log, "Console");
                        ui.toggle_value(&mut self.show_machine_panel, "Machine Controls");
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
                            if ui.small_button("✕").on_hover_text("Close Console").clicked() {
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
                        ui.label(egui::RichText::new(format!("{} baud • {}", self.baud, self.flow_mode)).size(11.0).color(Color32::DARK_GRAY));
                        ui.separator();
                        let cut_dist = self.total_cut_length_mm();
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
                        ui.label(egui::RichText::new("📁 Design Artwork").strong().size(12.5));
                        let tot_files = self.loaded_docs.len();
                        let sel_p = self.selected_paths_count();
                        let tot_p = self.total_paths_count();
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            ui.label(egui::RichText::new(format!("{} files • {}/{} paths", tot_files, sel_p, tot_p)).size(11.0).color(Color32::from_rgb(0, 210, 255)));
                        });
                    });

                    ui.add_space(4.0);

                    ui.horizontal(|ui| {
                        if ui
                            .button("📁 Browse / Add Files...")
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
                        ui.label(egui::RichText::new("Presets:").size(11.0).color(Color32::GRAY));
                        if ui.small_button("+ Test Cut").on_hover_text("Add embedded calibration pattern").clicked() {
                            self.add_svg_content(EMBEDDED_TEST_CUT_SVG, "test_cut.svg", None);
                        }
                        if ui.small_button("+ Axis Test").on_hover_text("Add embedded axis alignment pattern").clicked() {
                            self.add_svg_content(EMBEDDED_AXIS_TEST_SVG, "axis_test.svg", None);
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
                                for (idx, doc) in self.loaded_docs.iter_mut().enumerate() {
                                    let d_total = doc.total_paths();
                                    let d_sel = doc.selected_paths();
                                    let is_all = d_sel == d_total && d_total > 0;
                                    let is_none = d_sel == 0;

                                    ui.horizontal(|ui| {
                                        let arrow = if doc.expanded { "▼" } else { "▶" };
                                        if ui.small_button(arrow).clicked() {
                                            doc.expanded = !doc.expanded;
                                        }

                                        let mut checked = is_all;
                                        if ui.checkbox(&mut checked, "").clicked() {
                                            if is_all {
                                                doc.set_selected_recursive(false);
                                            } else {
                                                doc.set_selected_recursive(true);
                                            }
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
                                                "📄 {} ({}/{})",
                                                doc.filename, d_sel, d_total
                                            ))
                                            .strong()
                                            .size(11.5)
                                            .color(doc_color),
                                        );

                                        ui.with_layout(
                                            egui::Layout::right_to_left(egui::Align::Center),
                                            |ui| {
                                                if ui
                                                    .small_button("✕")
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
                                            render_group_item(ui, sub_grp, 1);
                                        }
                                        for path in &mut doc.root_group.paths {
                                            render_path_item(ui, path, 1);
                                        }
                                    }
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

                // --- 2. Sizing & Dimensions ---
                ui.group(|ui| {
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("📐 Size & Dimensions").strong().size(12.0));
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.small_button("200%").clicked() && self.orig_width_mm > 0.0 {
                                self.scale = 2.0;
                                self.custom_width_mm = (self.orig_width_mm * 2.0 * 10.0).round() / 10.0;
                                self.custom_height_mm = (self.orig_height_mm * 2.0 * 10.0).round() / 10.0;
                            }
                            if ui.small_button("100%").clicked() && self.orig_width_mm > 0.0 {
                                self.scale = 1.0;
                                self.custom_width_mm = self.orig_width_mm;
                                self.custom_height_mm = self.orig_height_mm;
                            }
                            if ui.small_button("50%").clicked() && self.orig_width_mm > 0.0 {
                                self.scale = 0.5;
                                self.custom_width_mm = (self.orig_width_mm * 0.5 * 10.0).round() / 10.0;
                                self.custom_height_mm = (self.orig_height_mm * 0.5 * 10.0).round() / 10.0;
                            }
                        });
                    });

                    ui.horizontal(|ui| {
                        ui.label("Width:");
                        let w_resp = ui.add(
                            egui::DragValue::new(&mut self.custom_width_mm)
                                .speed(0.5)
                                .range(1.0..=2000.0)
                                .max_decimals(1)
                                .suffix(" mm"),
                        );
                        if w_resp.changed() && self.lock_aspect_ratio && self.orig_width_mm > 0.0 {
                            let ratio = self.custom_width_mm / self.orig_width_mm;
                            self.custom_height_mm = (self.orig_height_mm * ratio * 10.0).round() / 10.0;
                            self.scale = ratio;
                        }

                        ui.add_space(8.0);

                        ui.label("Height:");
                        let h_resp = ui.add(
                            egui::DragValue::new(&mut self.custom_height_mm)
                                .speed(0.5)
                                .range(1.0..=2000.0)
                                .max_decimals(1)
                                .suffix(" mm"),
                        );
                        if h_resp.changed() && self.lock_aspect_ratio && self.orig_height_mm > 0.0 {
                            let ratio = self.custom_height_mm / self.orig_height_mm;
                            self.custom_width_mm = (self.orig_width_mm * ratio * 10.0).round() / 10.0;
                            self.scale = ratio;
                        }
                    });

                    ui.horizontal(|ui| {
                        let lock_icon = if self.lock_aspect_ratio { "🔗 Aspect Ratio: Locked" } else { "🔓 Aspect Ratio: Free" };
                        ui.checkbox(&mut self.lock_aspect_ratio, lock_icon);
                    });

                    // Weeding Border inside Dimensions
                    ui.separator();
                    ui.horizontal(|ui| {
                        ui.checkbox(&mut self.add_weeding_box, "Weeding Border");
                        if self.add_weeding_box {
                            ui.add(
                                egui::Slider::new(&mut self.weeding_margin, 1.0..=25.0)
                                    .step_by(0.5)
                                    .suffix(" mm"),
                            );
                        }
                    });

                    if self.developer_mode {
                        ui.separator();
                        ui.label(egui::RichText::new("Axes Alignment:").size(11.0).color(Color32::GRAY));
                        ui.horizontal(|ui| {
                            ui.checkbox(&mut self.normalize_origin, "Auto-Zero");
                            ui.checkbox(&mut self.flip_y, "Flip Y");
                            ui.checkbox(&mut self.swap_xy, "Swap X/Y");
                        });
                    }
                });

                ui.add_space(6.0);

                // --- 3. Cutter Settings & Material ---
                ui.group(|ui| {
                    ui.label(egui::RichText::new("⚙ Material & Cut Settings").strong().size(12.0));

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
                    }
                });

                ui.add_space(8.0);

                // --- 4. Cut Execution Action Bar ---
                ui.group(|ui| {
                    ui.label(egui::RichText::new("🚀 Job Execution").strong().size(12.0));

                    let cut_btn = egui::Button::new(
                        egui::RichText::new(if self.is_cutting { "CUTTING IN PROGRESS..." } else { "▶ CUT NOW" })
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
                        let btn_w = (ui.available_width() - 8.0) / 2.0;
                        if ui.add_sized([btn_w, 28.0], egui::Button::new("👁 Dry Run")).on_hover_text("Simulate cut trajectory without sending to hardware").clicked() {
                            self.execute_cut(true);
                        }
                        if ui.add_sized([btn_w, 28.0], egui::Button::new("✂ Test Cut")).on_hover_text("Cut small 12mm test square & triangle").clicked() {
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
                        let cancel_btn = egui::Button::new("🛑 Cancel Cut").fill(Color32::from_rgb(220, 38, 38));
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

                    // --- 1. Serial Port & Communication ---
                    ui.group(|ui| {
                        ui.horizontal(|ui| {
                            ui.label(egui::RichText::new("Serial Connection").strong().size(12.0));
                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                if ui.small_button("↻").on_hover_text("Refresh Ports").clicked() {
                                    self.refresh_ports();
                                }
                            });
                        });

                        let old_port = self.selected_port.clone();
                        egui::ComboBox::from_id_salt("right_panel_port_combo")
                            .selected_text(if self.selected_port.is_empty() { "None".to_string() } else { self.selected_port.clone() })
                            .show_ui(ui, |ui| {
                                for p in &self.ports {
                                    let label = if p.is_likely_cutter {
                                        format!("{} (Cutter)", p.name)
                                    } else {
                                        p.name.clone()
                                    };
                                    ui.selectable_value(&mut self.selected_port, p.name.clone(), label);
                                }
                            });
                        if self.selected_port != old_port {
                            self.notify_port_config_changed();
                        }

                        if self.developer_mode {
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

                        ui.add_space(4.0);

                        ui.vertical_centered_justified(|ui| {
                            if ui.button("📍 Set Origin (Here)").on_hover_text("Zero coordinates at current blade position without moving motors").clicked() {
                                self.set_origin_here();
                            }
                            if ui.button("🏠 Return to Origin (0,0)").on_hover_text("Physically rapid travel tool carriage and feed back to 0,0").clicked() {
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
                            if ui.add_sized([120.0, 28.0], egui::Button::new("▲ Feed Out (+)")).on_hover_text("Feed vinyl forward").clicked() {
                                self.jog_feed(self.jog_step_mm);
                            }

                            ui.add_space(2.0);

                            ui.horizontal(|ui| {
                                let total_w = 75.0 + 65.0 + 75.0 + 8.0;
                                ui.add_space(((ui.available_width() - total_w) / 2.0).max(0.0));
                                if ui.add_sized([75.0, 28.0], egui::Button::new("◀ Left")).on_hover_text("Move carriage left").clicked() {
                                    self.jog_carriage(self.jog_step_mm);
                                }
                                if ui.add_sized([65.0, 28.0], egui::Button::new("⌂ Home")).on_hover_text("Return to Origin 0,0").clicked() {
                                    self.go_home();
                                }
                                if ui.add_sized([75.0, 28.0], egui::Button::new("Right ▶")).on_hover_text("Move carriage right").clicked() {
                                    self.jog_carriage(-self.jog_step_mm);
                                }
                            });

                            ui.add_space(2.0);

                            if ui.add_sized([120.0, 28.0], egui::Button::new("▼ Retract (-)")).on_hover_text("Retract vinyl backward").clicked() {
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
                            if ui.add_sized([ui.available_width(), 26.0], egui::Button::new("📐 Axis Test Pattern")).on_hover_text("Cut 20x20mm test pattern with X/Y directional indicators").clicked() {
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
                ui.label(
                    egui::RichText::new(format!(
                        "Cut Dimensions: {:.1} mm × {:.1} mm",
                        self.custom_width_mm, self.custom_height_mm
                    ))
                    .strong()
                    .color(Color32::from_rgb(0, 200, 255)),
                );

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

                // Canvas Zoom Controls on the Right
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.small_button("Fit View").on_hover_text("Reset Pan and Zoom to fit artwork").clicked() {
                        self.canvas_zoom = 1.0;
                        self.canvas_pan = Vec2::ZERO;
                    }
                    if ui.small_button("1:1").on_hover_text("Reset to 100% Zoom").clicked() {
                        self.canvas_zoom = 1.0;
                    }
                    if ui.small_button(" - ").on_hover_text("Zoom Out").clicked() {
                        self.canvas_zoom = (self.canvas_zoom * 0.85).max(0.2);
                    }
                    ui.label(egui::RichText::new(format!("{:.0}%", self.canvas_zoom * 100.0)).size(11.0).monospace().color(Color32::GRAY));
                    if ui.small_button(" + ").on_hover_text("Zoom In").clicked() {
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

            let base_origin = canvas_rect.left_top() + Vec2::new(50.0, 50.0);
            let origin = base_origin + self.canvas_pan;

            // Transform paths
            let output = self.transform_paths();
            let has_paths = !output.active_paths.is_empty() || !output.inactive_paths.is_empty();

            let max_dim = (self.custom_width_mm.max(self.custom_height_mm)
                + (self.weeding_margin * 2.0))
                .max(10.0) as f32;
            let avail_w = (canvas_rect.width() - 120.0).max(50.0);
            let avail_h = (canvas_rect.height() - 120.0).max(50.0);
            let auto_fit_zoom = (avail_w / max_dim).min(avail_h / max_dim).clamp(0.2, 10.0);
            let effective_zoom = auto_fit_zoom * self.canvas_zoom;

            let to_screen = |pt: kurbo::Point| -> Pos2 {
                let (sx, sy) = if self.swap_xy {
                    (pt.y as f32, pt.x as f32)
                } else {
                    (pt.x as f32, pt.y as f32)
                };
                Pos2::new(origin.x + (sx * effective_zoom), origin.y + (sy * effective_zoom))
            };

            // Draw Metric CAD Grid (10mm minor, 50mm major)
            if effective_zoom > 0.3 {
                let minor_step = if effective_zoom > 1.8 { 10.0 } else { 50.0 };
                let grid_w = (self.custom_width_mm.max(100.0) * 1.3) as f32;
                let grid_h = (self.custom_height_mm.max(100.0) * 1.3) as f32;

                let minor_stroke = Stroke::new(0.5_f32, Color32::from_rgb(26, 28, 38));
                let major_stroke = Stroke::new(0.8_f32, Color32::from_rgb(38, 42, 58));

                let mut gx = 0.0;
                while gx <= grid_w {
                    let is_major = (gx as i32 % 50) == 0;
                    let p0 = to_screen(kurbo::Point::new(gx as f64, 0.0));
                    let p1 = to_screen(kurbo::Point::new(gx as f64, grid_h as f64));
                    if canvas_rect.x_range().contains(p0.x) || canvas_rect.x_range().contains(p1.x) {
                        painter.line_segment([p0, p1], if is_major { major_stroke } else { minor_stroke });
                    }
                    gx += minor_step;
                }

                let mut gy = 0.0;
                while gy <= grid_h {
                    let is_major = (gy as i32 % 50) == 0;
                    let p0 = to_screen(kurbo::Point::new(0.0, gy as f64));
                    let p1 = to_screen(kurbo::Point::new(grid_w as f64, gy as f64));
                    if canvas_rect.y_range().contains(p0.y) || canvas_rect.y_range().contains(p1.y) {
                        painter.line_segment([p0, p1], if is_major { major_stroke } else { minor_stroke });
                    }
                    gy += minor_step;
                }
            }

            // Origin marker (0, 0)
            painter.circle_filled(origin, 5.0, Color32::from_rgb(239, 68, 68));
            painter.circle_stroke(origin, 8.0, Stroke::new(1.0_f32, Color32::from_rgb(239, 68, 68)));
            painter.text(
                origin + Vec2::new(10.0, -10.0),
                egui::Align2::LEFT_BOTTOM,
                "Origin (0,0)",
                egui::FontId::monospace(11.0),
                Color32::from_rgb(240, 240, 240),
            );

            // Draw axis guides
            let axis_len = 65.0 * effective_zoom.clamp(0.6, 1.2);
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
                    painter.text(
                        p_wbox_label + Vec2::new(4.0, 4.0),
                        egui::Align2::LEFT_TOP,
                        "Weeding Border",
                        egui::FontId::monospace(10.0),
                        Color32::from_rgb(245, 158, 11),
                    );
                }

                // 4. Floating Legend Card in Bottom-Right
                let legend_rect = egui::Rect::from_min_size(
                    canvas_rect.right_bottom() - Vec2::new(190.0, 70.0),
                    Vec2::new(180.0, 62.0),
                );
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
        });
    }
}

fn setup_custom_theme(ctx: &egui::Context) {
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
