use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::Duration;
use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, SerialPortType, StopBits};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FlowControlMode {
    #[default]
    None,     // Paced software timing (universal standard, works with all 3-wire and USB cables)
    Hardware, // RTS/CTS (requires hardware handshake pins connected)
    Software, // XON/XOFF
}

impl std::fmt::Display for FlowControlMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FlowControlMode::None => write!(f, "Paced Only (Universal)"),
            FlowControlMode::Hardware => write!(f, "RTS/CTS (Hardware)"),
            FlowControlMode::Software => write!(f, "XON/XOFF (Software)"),
        }
    }
}

pub struct RedsailDevice {
    port: Box<dyn SerialPort>,
    #[allow(dead_code)]
    baud_rate: u32,
}

impl RedsailDevice {
    /// Open a connection to the cutter with proper flow control and timings
    pub fn open(port_name: &str, baud_rate: u32, flow_mode: FlowControlMode) -> Result<Self, String> {
        let flow_ctrl = match flow_mode {
            FlowControlMode::Hardware => FlowControl::Hardware,
            FlowControlMode::Software => FlowControl::Software,
            FlowControlMode::None => FlowControl::None,
        };

        let port = serialport::new(port_name, baud_rate)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(flow_ctrl)
            .timeout(Duration::from_millis(1000))
            .open()
            .map_err(|e| {
                let err_str = e.to_string();
                if err_str.contains("Permission denied") {
                    format!(
                        "Permission denied on '{}'. Fix with:\n  sudo chmod 666 {}\nor install udev rules:\n  sudo install -m 644 assets/99-redsail.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger",
                        port_name, port_name
                    )
                } else {
                    format!("Failed to open serial port '{}': {}", port_name, err_str)
                }
            })?;

        Ok(Self { port, baud_rate })
    }

    /// Immediately lift blade, stop cutter motion, and clear hardware FIFO
    pub fn emergency_halt(&mut self) -> io::Result<()> {
        let _ = self.port.write_all(b"PU;SP0;\x1B;");
        let _ = self.port.flush();
        let _ = self.port.clear(ClearBuffer::All);
        Ok(())
    }

    /// Stream HP-GL commands safely with motion-aware pacing, atomic command delivery, and abort support
    pub fn send_hpgl_cancellable(
        &mut self,
        commands: &str,
        abort_flag: Option<&AtomicBool>,
        progress: Option<&AtomicUsize>,
    ) -> io::Result<bool> {
        let mut last_x: f64 = 0.0;
        let mut last_y: f64 = 0.0;
        let mut current_speed_mms: f64 = 200.0; // Default 20 cm/s = 200 mm/s
        let travel_speed_mms: f64 = 120.0;      // Conservative rapid travel speed ~120 mm/s

        let total_bytes = commands.len();
        let mut sent_bytes = 0;

        // Split HP-GL into discrete, complete commands by semicolon ';'
        // This prevents chopping numeric coordinates across arbitrary chunk boundaries
        for raw_cmd in commands.split(';') {
            let cmd = raw_cmd.trim();
            if cmd.is_empty() {
                continue;
            }

            if let Some(flag) = abort_flag
                && flag.load(Ordering::SeqCst)
            {
                self.emergency_halt()?;
                return Ok(false);
            }

            let cmd_with_semi = format!("{};", cmd);
            self.port.write_all(cmd_with_semi.as_bytes())?;

            sent_bytes += cmd_with_semi.len();
            if let Some(prog) = progress {
                prog.store(sent_bytes.min(total_bytes), Ordering::SeqCst);
            }

            // Calculate physical motion execution delay for each command
            // to completely prevent overflowing the cutter's internal 64-byte RAM buffer
            let delay = if let Some(speed_str) = cmd.strip_prefix("VS") {
                if let Ok(v) = speed_str.parse::<f64>() {
                    current_speed_mms = (v * 10.0).clamp(20.0, 800.0);
                }
                Duration::from_millis(40)
            } else if cmd == "IN" {
                Duration::from_millis(300) // Hardware reinitialization settling
            } else if cmd.starts_with("SP") {
                Duration::from_millis(150) // Tool select dwell
            } else if cmd == "PU" || cmd == "PD" {
                Duration::from_millis(80)  // Mechanical solenoid lift/drop dwell
            } else if let Some(coords) = cmd.strip_prefix("PU") {
                // Rapid travel move: PUx,y
                if let Some((xs, ys)) = coords.split_once(',') {
                    if let (Ok(x), Ok(y)) = (xs.parse::<f64>(), ys.parse::<f64>()) {
                        let target_x = x / 40.0;
                        let target_y = y / 40.0;
                        let dist = (target_x - last_x).hypot(target_y - last_y);
                        last_x = target_x;
                        last_y = target_y;
                        // Realistic stepper acceleration curve for travel moves:
                        let move_ms = ((dist / travel_speed_mms) * 1000.0).round() as u64;
                        Duration::from_millis(move_ms.max(40) + 30)
                    } else {
                        Duration::from_millis(50)
                    }
                } else {
                    Duration::from_millis(50)
                }
            } else if let Some(coords) = cmd.strip_prefix("PD") {
                // Cutting move: PDx,y
                if let Some((xs, ys)) = coords.split_once(',') {
                    if let (Ok(x), Ok(y)) = (xs.parse::<f64>(), ys.parse::<f64>()) {
                        let target_x = x / 40.0;
                        let target_y = y / 40.0;
                        let dist = (target_x - last_x).hypot(target_y - last_y);
                        last_x = target_x;
                        last_y = target_y;
                        // Safe cutting motion pacing accounting for stepper inertia:
                        let eff_speed = current_speed_mms.min(75.0);
                        let move_ms = ((dist / eff_speed) * 1000.0).round() as u64;
                        Duration::from_millis(move_ms.max(35) + 30)
                    } else {
                        Duration::from_millis(45)
                    }
                } else {
                    Duration::from_millis(45)
                }
            } else {
                Duration::from_millis(30)
            };

            std::thread::sleep(delay);
        }

        // Final settling delay to let cutter hardware finish the last movement
        std::thread::sleep(Duration::from_millis(150));
        Ok(true)
    }

    /// Stream HP-GL commands safely with buffer pacing
    pub fn send_hpgl(&mut self, commands: &str) -> io::Result<()> {
        let _ = self.send_hpgl_cancellable(commands, None, None)?;
        Ok(())
    }
}

/// Information about a detected serial or USB port
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DetectedPort {
    pub name: String,
    pub description: String,
    pub is_likely_cutter: bool,
}

/// Enumerate all available serial ports on the system (deduplicated and prioritized)
pub fn list_ports() -> Vec<DetectedPort> {
    let mut results = Vec::new();
    if let Ok(ports) = serialport::available_ports() {
        for p in ports {
            let (desc, likely) = match p.port_type {
                SerialPortType::UsbPort(usb) => {
                    let vid = usb.vid;
                    let pid = usb.pid;
                    let mfg = usb.manufacturer.unwrap_or_default();
                    let prod = usb.product.unwrap_or_default();
                    let desc = format!("USB [VID:{:04x} PID:{:04x}] {} {}", vid, pid, mfg, prod).trim().to_string();
                    let is_cutter = vid == 0x1a86 || vid == 0x0403 || vid == 0x10c4 || vid == 0x4348;
                    (desc, is_cutter)
                }
                SerialPortType::PciPort => ("PCI Serial Port".to_string(), false),
                SerialPortType::BluetoothPort => ("Bluetooth Port".to_string(), false),
                SerialPortType::Unknown => ("Standard Serial Port".to_string(), false),
            };

            // Deduplicate by port_name to avoid repeated device entries
            if !results.iter().any(|existing: &DetectedPort| existing.name == p.port_name) {
                results.push(DetectedPort {
                    name: p.port_name,
                    description: desc,
                    is_likely_cutter: likely,
                });
            }
        }
    }

    // Sort so likely cutters appear first, followed alphabetically by device name
    results.sort_by(|a, b| {
        b.is_likely_cutter
            .cmp(&a.is_likely_cutter)
            .then_with(|| a.name.cmp(&b.name))
    });

    results
}
