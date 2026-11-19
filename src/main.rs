mod blade_offset;
mod device;
mod gui;
mod hpgl;
mod svg_parser;

use clap::{Parser, Subcommand};
use std::fs;
use std::path::PathBuf;

use blade_offset::BladeCompensator;
use device::{list_ports, FlowControlMode, RedsailDevice};
use hpgl::{mm_to_plotter, CutterSettings, HpglBuilder};
use svg_parser::parse_svg;

#[derive(Parser, Debug)]
#[command(name = "redsail-cutter")]
#[command(about = "Pure Rust Vinyl Cutter Controller & HP-GL Generator for Plotters", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Option<Commands>,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Launch the interactive GUI
    Gui,

    /// Enumerate and inspect available serial and USB ports
    ListPorts,

    /// Parse an SVG file, apply blade compensation, and cut on the plotter
    Cut {
        /// Path to input SVG file
        #[arg(value_name = "FILE")]
        input: PathBuf,

        /// Serial port path (e.g. /dev/ttyUSB0)
        #[arg(short, long)]
        port: Option<String>,

        /// Baud rate (typically 9600 or 38400 for Redsail)
        #[arg(short, long, default_value_t = 9600)]
        baud: u32,

        /// Cutting speed in cm/s (1 - 60)
        #[arg(short, long, default_value_t = 20)]
        speed: u32,

        /// Cutting force / pressure in grams (10 - 400)
        #[arg(short, long, default_value_t = 80)]
        force: u32,

        /// Drag-knife blade offset in mm (e.g. 0.25)
        #[arg(long, default_value_t = 0.25)]
        blade_offset: f64,

        /// Curve flattening tolerance in mm (lower = smoother curves, more points)
        #[arg(long, default_value_t = 0.05)]
        tolerance: f64,

        /// Scale factor applied to dimensions (1.0 = 100%)
        #[arg(long, default_value_t = 1.0)]
        scale: f64,

        /// Auto-zero origin to minimum X and Y
        #[arg(long, default_value_t = true)]
        normalize_origin: bool,

        /// Flip Y axis (SVG down to cutter standard)
        #[arg(long, default_value_t = true)]
        flip_y: bool,

        /// Swap X and Y axes to match Redsail carriage and feed orientation
        #[arg(long, default_value_t = true)]
        swap_xy: bool,

        /// Cut an outer weeding box rectangle around the artwork
        #[arg(long, default_value_t = true)]
        weeding_box: bool,

        /// Margin around artwork for the weeding box in mm
        #[arg(long, default_value_t = 5.0)]
        weeding_margin: f64,

        /// Simulate the job without sending commands to hardware
        #[arg(long)]
        dry_run: bool,
    },

    /// Convert an SVG vector file directly into an HP-GL file (.plt / .hpgl)
    Export {
        /// Path to input SVG file
        #[arg(value_name = "SVG_FILE")]
        input: PathBuf,

        /// Path to output HP-GL file
        #[arg(short, long, value_name = "HPGL_FILE")]
        output: PathBuf,

        /// Cutting speed in cm/s
        #[arg(short, long, default_value_t = 20)]
        speed: u32,

        /// Cutting force in grams
        #[arg(short, long, default_value_t = 80)]
        force: u32,

        /// Drag-knife blade offset in mm
        #[arg(long, default_value_t = 0.25)]
        blade_offset: f64,

        /// Curve flattening tolerance in mm
        #[arg(long, default_value_t = 0.05)]
        tolerance: f64,

        /// Scale factor
        #[arg(long, default_value_t = 1.0)]
        scale: f64,

        /// Auto-zero origin
        #[arg(long, default_value_t = true)]
        normalize_origin: bool,

        /// Flip Y axis
        #[arg(long, default_value_t = true)]
        flip_y: bool,

        /// Swap X and Y axes to match Redsail carriage and feed orientation
        #[arg(long, default_value_t = true)]
        swap_xy: bool,

        /// Cut an outer weeding box rectangle
        #[arg(long, default_value_t = true)]
        weeding_box: bool,

        /// Margin around artwork for the weeding box in mm
        #[arg(long, default_value_t = 5.0)]
        weeding_margin: f64,
    },

    /// Send a pre-generated HP-GL file directly to the plotter
    Send {
        /// Path to HP-GL file (.hpgl or .plt)
        #[arg(value_name = "HPGL_FILE")]
        input: PathBuf,

        /// Serial port path (e.g. /dev/ttyUSB0)
        #[arg(short, long)]
        port: String,

        /// Baud rate
        #[arg(short, long, default_value_t = 9600)]
        baud: u32,
    },
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    match cli.command {
        None | Some(Commands::Gui) => {
            println!("Launching Plotter GUI...");
            if let Err(e) = gui::run_gui() {
                eprintln!("GUI Error: {}", e);
            }
        }

        Some(Commands::ListPorts) => {
            println!("Scanning for serial / USB devices...\n");
            let ports = list_ports();
            if ports.is_empty() {
                println!("No serial ports detected.");
            } else {
                for p in ports {
                    let marker = if p.is_likely_cutter {
                        "  [Likely Cutter]"
                    } else {
                        ""
                    };
                    println!("- {}{}", p.name, marker);
                    if !p.description.is_empty() {
                        println!("    {}", p.description);
                    }
                }
            }
        }

        Some(Commands::Export {
            input,
            output,
            speed,
            force,
            blade_offset,
            tolerance,
            scale,
            normalize_origin,
            flip_y,
            swap_xy,
            weeding_box,
            weeding_margin,
        }) => {
            println!("Reading SVG: {}", input.display());
            let svg_content = fs::read_to_string(&input)?;
            let hpgl_data = generate_hpgl(
                &svg_content,
                speed,
                force,
                blade_offset,
                tolerance,
                scale,
                normalize_origin,
                flip_y,
                swap_xy,
                weeding_box,
                weeding_margin,
            )?;

            fs::write(&output, &hpgl_data)?;
            println!("Exported HP-GL to: {} ({} bytes)", output.display(), hpgl_data.len());
        }

        Some(Commands::Cut {
            input,
            port,
            baud,
            speed,
            force,
            blade_offset,
            tolerance,
            scale,
            normalize_origin,
            flip_y,
            swap_xy,
            weeding_box,
            weeding_margin,
            dry_run,
        }) => {
            println!("Reading SVG: {}", input.display());
            let svg_content = fs::read_to_string(&input)?;
            let hpgl_data = generate_hpgl(
                &svg_content,
                speed,
                force,
                blade_offset,
                tolerance,
                scale,
                normalize_origin,
                flip_y,
                swap_xy,
                weeding_box,
                weeding_margin,
            )?;

            println!("Generated {} bytes of HP-GL commands.", hpgl_data.len());

            if dry_run {
                println!("[DRY RUN] Simulating cut without sending to hardware.");
                println!("--- HP-GL Preview (first 250 characters) ---");
                let preview = if hpgl_data.len() > 250 {
                    &hpgl_data[..250]
                } else {
                    &hpgl_data
                };
                println!("{}\n...", preview);
                println!("--- Dry run complete ---");
                return Ok(());
            }

            let target_port = match port {
                Some(p) => p,
                None => {
                    let ports = list_ports();
                    let likely = ports.into_iter().find(|p| p.is_likely_cutter);
                    match likely {
                        Some(p) => {
                            println!("Auto-selected cutter port: {}", p.name);
                            p.name
                        }
                        None => {
                            return Err("No serial port specified. Use --port /dev/ttyUSB0 or run 'redsail list-ports'.".into());
                        }
                    }
                }
            };

            println!("Connecting to {} at {} baud (Paced Flow Control)...", target_port, baud);
            let mut device = RedsailDevice::open(&target_port, baud, FlowControlMode::None)?;

            println!("Streaming cut job to cutter (with FIFO pacing)...");
            device.send_hpgl(&hpgl_data)?;
            println!("Cut completed successfully!");
        }

        Some(Commands::Send { input, port, baud }) => {
            println!("Reading HP-GL file: {}", input.display());
            let hpgl_data = fs::read_to_string(&input)?;

            println!("Connecting to {} at {} baud (Paced Flow Control)...", port, baud);
            let mut device = RedsailDevice::open(&port, baud, FlowControlMode::None)?;

            println!("Sending {} bytes to cutter...", hpgl_data.len());
            device.send_hpgl(&hpgl_data)?;
            println!("Transmission complete!");
        }
    }

    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn generate_hpgl(
    svg_content: &str,
    speed: u32,
    force: u32,
    blade_offset: f64,
    tolerance: f64,
    scale: f64,
    normalize_origin: bool,
    flip_y: bool,
    swap_xy: bool,
    weeding_box: bool,
    weeding_margin: f64,
) -> Result<String, Box<dyn std::error::Error>> {
    let doc = parse_svg(svg_content, tolerance)?;
    println!("Parsed {} continuous vector paths.", doc.paths.len());

    let mut min_x = f64::MAX;
    let mut min_y = f64::MAX;
    let mut max_x = f64::MIN;
    let mut max_y = f64::MIN;

    for path in &doc.paths {
        for pt in path {
            if pt.x < min_x { min_x = pt.x; }
            if pt.x > max_x { max_x = pt.x; }
            if pt.y < min_y { min_y = pt.y; }
            if pt.y > max_y { max_y = pt.y; }
        }
    }

    if min_x > max_x {
        min_x = 0.0;
        min_y = 0.0;
        max_y = 0.0;
    }

    let margin_mm = (blade_offset * 2.0).max(2.0);
    let extra_offset = if weeding_box { weeding_margin } else { 0.0 };
    let base_offset = margin_mm + extra_offset;

    let mut transformed: Vec<Vec<kurbo::Point>> = Vec::new();
    let mut t_min_x = f64::MAX;
    let mut t_min_y = f64::MAX;
    let mut t_max_x = f64::MIN;
    let mut t_max_y = f64::MIN;

    for path in &doc.paths {
        if path.is_empty() {
            continue;
        }

        let pts: Vec<kurbo::Point> = path
            .iter()
            .map(|pt| {
                let mut x = if normalize_origin {
                    (pt.x - min_x) + base_offset
                } else {
                    pt.x + extra_offset
                };

                let mut y = if flip_y {
                    (max_y - pt.y) + base_offset
                } else if normalize_origin {
                    (pt.y - min_y) + base_offset
                } else {
                    pt.y + extra_offset
                };

                if swap_xy {
                    std::mem::swap(&mut x, &mut y);
                }

                let scaled = kurbo::Point::new(x * scale, y * scale);
                if scaled.x < t_min_x { t_min_x = scaled.x; }
                if scaled.x > t_max_x { t_max_x = scaled.x; }
                if scaled.y < t_min_y { t_min_y = scaled.y; }
                if scaled.y > t_max_y { t_max_y = scaled.y; }
                scaled
            })
            .collect();

        transformed.push(pts);
    }

    let settings = CutterSettings {
        speed,
        force,
        blade_offset_mm: blade_offset,
    };

    let compensator = BladeCompensator::new(blade_offset);
    let mut builder = HpglBuilder::new(&settings);
    let mut total_points = 0;

    // 1. Cut all design paths first
    for path in &transformed {
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

    // 2. Cut weeding box last
    if weeding_box && !transformed.is_empty() {
        let wm = weeding_margin * scale;
        let bx0 = (t_min_x - wm).max(0.0);
        let by0 = (t_min_y - wm).max(0.0);
        let bx1 = t_max_x + wm;
        let by1 = t_max_y + wm;
        let wbox = vec![
            kurbo::Point::new(bx0, by0),
            kurbo::Point::new(bx1, by0),
            kurbo::Point::new(bx1, by1),
            kurbo::Point::new(bx0, by1),
            kurbo::Point::new(bx0, by0),
        ];

        let compensated_box = compensator.process_path(&wbox);
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

    println!("Prepared {} compensated cutter points (including weeding box).", total_points);
    Ok(builder.finish())
}
