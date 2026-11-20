#!/usr/bin/env bash
set -euo pipefail

# -----------------------------------------------------------------------------
# Redsail Cutter - Packaging / Release Archive Script
# -----------------------------------------------------------------------------

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${PROJECT_ROOT}"

VERSION=$(grep -m1 '^version *=' Cargo.toml | sed -E 's/version *= *"([^"]+)".*/\1/')
ARCH=$(uname -m)
PKG_NAME="redsail-cutter-v${VERSION}-linux-${ARCH}"
DIST_DIR="${PROJECT_ROOT}/dist"
STAGE_DIR="${DIST_DIR}/${PKG_NAME}"
ARCHIVE_PATH="${DIST_DIR}/${PKG_NAME}.tar.gz"

echo "==> Packaging Redsail Cutter"
echo "    Version:      v${VERSION}"
echo "    Architecture: ${ARCH}"
echo "    Output File:  ${ARCHIVE_PATH}"

echo "==> Building optimized release binary..."
"${PROJECT_ROOT}/build.sh" --release

rm -rf "${STAGE_DIR}" "${ARCHIVE_PATH}"
mkdir -p "${STAGE_DIR}/assets"

install -m 755 "${PROJECT_ROOT}/target/release/redsail-cutter" "${STAGE_DIR}/redsail-cutter"
if command -v strip >/dev/null 2>&1; then
    strip "${STAGE_DIR}/redsail-cutter" || true
fi

install -m 644 "${PROJECT_ROOT}/assets/redsail-cutter.desktop" "${STAGE_DIR}/assets/"
install -m 644 "${PROJECT_ROOT}/assets/redsail-cutter.svg" "${STAGE_DIR}/assets/"
install -m 644 "${PROJECT_ROOT}/assets/99-redsail.rules" "${STAGE_DIR}/assets/"

install -m 644 "${PROJECT_ROOT}/README.md" "${STAGE_DIR}/"

cat << 'EOF' > "${STAGE_DIR}/install.sh"
#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $EUID -eq 0 ]]; then
    BIN_DIR="/usr/local/bin"
    DESKTOP_DIR="/usr/share/applications"
    ICON_DIR="/usr/share/icons/hicolor/scalable/apps"
    UDEV_DIR="/etc/udev/rules.d"
    echo "==> Installing Redsail Cutter system-wide..."
else
    BIN_DIR="${HOME}/.local/bin"
    DESKTOP_DIR="${HOME}/.local/share/applications"
    ICON_DIR="${HOME}/.local/share/icons/hicolor/scalable/apps"
    UDEV_DIR=""
    echo "==> Installing Redsail Cutter for current user (${HOME}/.local)..."
fi

mkdir -p "${BIN_DIR}" "${DESKTOP_DIR}" "${ICON_DIR}"

install -m 755 "${DIR}/redsail-cutter" "${BIN_DIR}/redsail-cutter"
install -m 644 "${DIR}/assets/redsail-cutter.desktop" "${DESKTOP_DIR}/redsail-cutter.desktop"
install -m 644 "${DIR}/assets/redsail-cutter.svg" "${ICON_DIR}/redsail-cutter.svg"

if [[ -n "${UDEV_DIR}" && -d "${UDEV_DIR}" ]]; then
    install -m 644 "${DIR}/assets/99-redsail.rules" "${UDEV_DIR}/99-redsail.rules"
    if command -v udevadm >/dev/null 2>&1; then
        udevadm control --reload-rules || true
        udevadm trigger || true
    fi
fi

if command -v update-desktop-database >/dev/null 2>&1; then
    update-desktop-database "${DESKTOP_DIR}" || true
fi

echo "==> Installation complete! Run 'redsail-cutter gui' to start."
EOF
chmod +x "${STAGE_DIR}/install.sh"

echo "==> Compressing archive..."
tar -czf "${ARCHIVE_PATH}" -C "${DIST_DIR}" "${PKG_NAME}"

echo "==> Generating SHA256 checksums..."
cd "${DIST_DIR}"
sha256sum "$(basename "${ARCHIVE_PATH}")" > "${DIST_DIR}/SHA256SUMS"

rm -rf "${STAGE_DIR}"

ARCHIVE_SIZE=$(du -h "${ARCHIVE_PATH}" | cut -f1)
echo "--------------------------------------------------------"
echo " Package Created Successfully!"
echo " Archive:  ${ARCHIVE_PATH}"
echo " Size:     ${ARCHIVE_SIZE}"
echo " Checksum: $(cat "${DIST_DIR}/SHA256SUMS")"
echo "--------------------------------------------------------"
