# Plotter - Vinyl Cutter Controller (Pure Rust)

A lightweight, standalone pure Rust desktop CAD/CAM application & CLI tool to parse SVGs, compute swivel blade offset compensation, generate standard HP-GL cut commands, and stream directly to vinyl cutting plotters (including Redsail and compatible HP-GL plotters) on Linux.

## Features

- **Pure Rust**: No Python runtime, no Wine, no C dependencies.
- **Hardware Handshaking**: RTS/CTS hardware flow control + baud-timed FIFO pacing prevents buffer overruns and pauses.
- **Visual Canvas Preview**: Real-time rendering of artwork and outer weeding frame.
- **Auto-Weeding Box**: Automatically cuts a rectangular outer weeding frame around your artwork with configurable margin. Cuts last to keep vinyl anchored.
- **Drag Blade Compensation**: Applies swivel blade offset and corner overcut/arcs so sharp corners don't tear or round off.
- **Axes Normalization**: Auto-zeros origin with positive safety margins and flips SVG Y-axis to match plotter coordinates.

## Quick Start & Installation

### Build & Run
```bash
./build.sh          # Builds optimized release binary
./build.sh --debug  # Builds debug binary
```

### Install to System / Desktop Launcher
```bash
# Install for current user (~/.local/bin + desktop app launcher icon):
./install.sh --user

# Or install system-wide (/usr/local/bin):
sudo ./install.sh --system

# To uninstall:
./install.sh --uninstall
```

### Packaging & GitHub Releases
```bash
# Create distribution tarball (.tar.gz) + SHA256SUMS in dist/:
./package.sh

# Create tag and publish release to GitHub using gh CLI:
./release.sh
./release.sh --dry-run   # Preview release notes without publishing
```

## Launching the GUI

```bash
redsail-cutter gui
```
*(or `./target/release/redsail-cutter gui`)*

## CLI Commands

### 1. Check Connected Cutters / Ports
```bash
./target/release/redsail-cutter list-ports
```

### 2. Preview / Dry Run a Cut
```bash
./target/release/redsail-cutter cut design.svg --dry-run
```

### 3. Cut to Plotter (with Weeding Box)
```bash
# Auto-detects /dev/ttyUSB0, cuts artwork and 5mm outer weeding box:
./target/release/redsail-cutter cut design.svg --speed 20 --force 80 --weeding-box --weeding-margin 5.0

# Specify port explicitly:
./target/release/redsail-cutter cut design.svg --port /dev/ttyUSB0 --speed 20 --force 80
```

### 4. Export SVG to HP-GL (.plt / .hpgl)
```bash
./target/release/redsail-cutter export design.svg -o output.hpgl --weeding-box --weeding-margin 5.0
```

## Options

- `--speed`, `-s`: Cutting speed in cm/s (default: 20)
- `--force`, `-f`: Downward pressure/force in grams (default: 80)
- `--blade-offset`: Knife tip offset in mm (default: 0.25 mm)
- `--weeding-box`: Add an outer rectangular weeding frame (default: true)
- `--weeding-margin`: Distance in mm between artwork and weeding box (default: 5.0 mm)
- `--flip-y`: Flip Y axis to match plotter orientation (default: true)
- `--swap-xy`: Rotate cut by 90 degrees (default: false)
- `--tolerance`: Curve flattening resolution in mm (default: 0.05 mm)
- `--scale`: Scale factor (default: 1.0)
