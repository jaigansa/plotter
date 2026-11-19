#!/usr/bin/env bash
set -euo pipefail

# -----------------------------------------------------------------------------
# Redsail Cutter - GitHub Release Script
# -----------------------------------------------------------------------------

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${PROJECT_ROOT}"

DRY_RUN=0
PRERELEASE=0
DRAFT=0

print_usage() {
    cat <<EOF
Usage: ./release.sh [OPTIONS]

Options:
    --dry-run       Build package and generate release notes without publishing
    --prerelease    Mark release as pre-release
    --draft         Create release as draft
    -h, --help      Show this help message
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        --prerelease)
            PRERELEASE=1
            shift
            ;;
        --draft)
            DRAFT=1
            shift
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            print_usage
            exit 1
            ;;
    esac
done

VERSION=$(grep -m1 '^version *=' Cargo.toml | sed -E 's/version *= *"([^"]+)".*/\1/')
TAG="v${VERSION}"
ARCH=$(uname -m)
DIST_DIR="${PROJECT_ROOT}/dist"
PKG_NAME="redsail-cutter-${TAG}-linux-${ARCH}"
ARCHIVE_PATH="${DIST_DIR}/${PKG_NAME}.tar.gz"
CHECKSUM_PATH="${DIST_DIR}/SHA256SUMS"
NOTES_FILE="${DIST_DIR}/RELEASE_NOTES_${TAG}.md"

echo "==> Preparing GitHub Release for ${TAG}..."

"${PROJECT_ROOT}/package.sh"

if [[ ! -f "${ARCHIVE_PATH}" || ! -f "${CHECKSUM_PATH}" ]]; then
    echo "Error: Release artifacts not found in ${DIST_DIR}" >&2
    exit 1
fi

cat << EOF > "${NOTES_FILE}"
# Plotter ${TAG}

Native Linux controller and HP-GL streamer for vinyl cutting plotters (Redsail RS360C, RS720C, RS800C, RS1120C, RS1360C, and compatible HP-GL models).

### Features
- **100% Pure Rust**: Standalone native Linux binary with no Wine, Python, or external runtime shims.
- **Hardware Integration**: Built-in direct streaming to CH340 / FTDI USB-to-serial adapters (\`/dev/ttyUSB*\`).
- **Interactive GUI**:
  - Live 2D vector preview with automatic scaling, custom width/height, and aspect ratio locking.
  - Material presets (Standard Vinyl, Heat Transfer Film, Holographic, Heavy/Thick, Custom).
  - Prominent real-time **Emergency Stop** and Start/Reset controls.
  - Dedicated **Machine Controls** panel with jog D-pad, media feed, step sizes, and local origin setup.
  - **Developer Options** toggle to reduce clutter for daily production while keeping advanced settings accessible.
- **HP-GL Vector Engine**:
  - High-precision SVG path parsing (curves, arcs, polygons, rectangles, circles).
  - Accurate 1016 DPI (40.0 steps/mm) hardware coordinate mapping.
  - Hardware-accurate axis alignment (Carriage vs Vinyl Feed).
  - Weeding border generation with configurable margins.
  - Software blade offset compensation for sharp vinyl corners.
- **CLI Mode**: Headless command-line interface for scripting, batch jobs, and headless automation.

### Artifacts & Checksums
- \`$(basename "${ARCHIVE_PATH}")\`
\`\`\`
$(cat "${CHECKSUM_PATH}")
\`\`\`

### Installation
Extract the archive and run:
\`\`\`bash
tar -xzf $(basename "${ARCHIVE_PATH}")
cd ${PKG_NAME}
./install.sh
\`\`\`
EOF

echo "==> Release notes generated at ${NOTES_FILE}:"
cat "${NOTES_FILE}"
echo ""

if [[ ${DRY_RUN} -eq 1 ]]; then
    echo "==> [DRY RUN] Release artifacts prepared in ${DIST_DIR}. Skipping publish."
    exit 0
fi

if git rev-parse "${TAG}" >/dev/null 2>&1; then
    echo "==> Git tag ${TAG} already exists."
else
    echo "==> Creating git tag ${TAG}..."
    git tag -a "${TAG}" -m "Release ${TAG}" || {
        echo "Note: Could not create git tag (ensure git is initialized with a commit)."
    }
fi

if command -v gh >/dev/null 2>&1; then
    echo "==> Found GitHub CLI (gh)."
    
    if gh auth status >/dev/null 2>&1; then
        echo "==> Publishing release to GitHub..."
        
        GH_ARGS=()
        if [[ ${PRERELEASE} -eq 1 ]]; then
            GH_ARGS+=("--prerelease")
        fi
        if [[ ${DRAFT} -eq 1 ]]; then
            GH_ARGS+=("--draft")
        fi

        gh release create "${TAG}" \
            "${ARCHIVE_PATH}" \
            "${CHECKSUM_PATH}" \
            --title "Plotter ${TAG}" \
            --notes-file "${NOTES_FILE}" \
            "${GH_ARGS[@]}"

        echo "==> GitHub release published successfully!"
    else
        echo "==> GitHub CLI is installed but not authenticated."
        echo "    Run: gh auth login"
        echo "    Then re-run this script to publish automatically."
    fi
else
    echo "==> GitHub CLI (gh) not found."
    echo "    To publish, upload the following files manually to your GitHub Releases page:"
    echo "      - ${ARCHIVE_PATH}"
    echo "      - ${CHECKSUM_PATH}"
    echo "      - ${NOTES_FILE}"
fi
