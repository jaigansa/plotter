#!/usr/bin/env bash
set -euo pipefail

# -----------------------------------------------------------------------------
# Redsail Cutter - Installation Script
# -----------------------------------------------------------------------------

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${PROJECT_ROOT}"

UNINSTALL=0
USER_MODE=0

if [[ $EUID -eq 0 ]]; then
    IS_ROOT=1
else
    IS_ROOT=0
fi

print_usage() {
    cat <<EOF
Usage: ./install.sh [OPTIONS]

Options:
    --user         Install for current user only (~/.local/bin, no sudo required)
    --system       Install system-wide to /usr/local/bin (requires root or sudo)
    --uninstall    Remove installed redsail-cutter binary, desktop entry & icon
    -h, --help     Show this help message

Default:
    Installs system-wide if run as root, otherwise installs for current user (~/.local/bin).
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --user)
            USER_MODE=1
            shift
            ;;
        --system)
            USER_MODE=0
            shift
            ;;
        --uninstall)
            UNINSTALL=1
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

if [[ ${USER_MODE} -eq 1 || (${IS_ROOT} -eq 0 && ${USER_MODE} -eq 0) ]]; then
    INSTALL_TYPE="User (${HOME}/.local)"
    BIN_DIR="${HOME}/.local/bin"
    DESKTOP_DIR="${HOME}/.local/share/applications"
    ICON_DIR="${HOME}/.local/share/icons/hicolor/scalable/apps"
    UDEV_DIR=""
else
    INSTALL_TYPE="System (/usr/local & /usr/share)"
    BIN_DIR="/usr/local/bin"
    DESKTOP_DIR="/usr/share/applications"
    ICON_DIR="/usr/share/icons/hicolor/scalable/apps"
    UDEV_DIR="/etc/udev/rules.d"
fi

# -----------------------------------------------------------------------------
# Uninstall Mode
# -----------------------------------------------------------------------------
if [[ ${UNINSTALL} -eq 1 ]]; then
    echo "==> Uninstalling Redsail Cutter (${INSTALL_TYPE})..."
    
    rm -f "${BIN_DIR}/redsail-cutter"
    rm -f "${DESKTOP_DIR}/redsail-cutter.desktop"
    rm -f "${ICON_DIR}/redsail-cutter.svg"
    
    if [[ -n "${UDEV_DIR}" && -f "${UDEV_DIR}/99-redsail.rules" ]]; then
        echo "==> Removing udev rules..."
        rm -f "${UDEV_DIR}/99-redsail.rules"
        udevadm control --reload-rules || true
        udevadm trigger || true
    fi

    command -v update-desktop-database >/dev/null 2>&1 && update-desktop-database "${DESKTOP_DIR}" || true
    command -v gtk-update-icon-cache >/dev/null 2>&1 && gtk-update-icon-cache -f -t "$(dirname "$(dirname "${ICON_DIR}")")" || true

    echo "==> Redsail Cutter successfully uninstalled!"
    exit 0
fi

# -----------------------------------------------------------------------------
# Install Mode
# -----------------------------------------------------------------------------
echo "==> Installing Redsail Cutter..."
echo "    Mode:        ${INSTALL_TYPE}"
echo "    Binary Dir:  ${BIN_DIR}"
echo "    Desktop Dir: ${DESKTOP_DIR}"
echo "    Icon Dir:    ${ICON_DIR}"

RELEASE_BIN="${PROJECT_ROOT}/target/release/redsail-cutter"
if [[ ! -f "${RELEASE_BIN}" ]]; then
    echo "==> Release binary not found. Building now..."
    "${PROJECT_ROOT}/build.sh" --release
fi

mkdir -p "${BIN_DIR}"
mkdir -p "${DESKTOP_DIR}"
mkdir -p "${ICON_DIR}"

# 1. Install binary
echo "==> Installing executable to ${BIN_DIR}/redsail-cutter..."
install -m 755 "${RELEASE_BIN}" "${BIN_DIR}/redsail-cutter"

# 2. Install desktop file
echo "==> Installing desktop entry to ${DESKTOP_DIR}/redsail-cutter.desktop..."
install -m 644 "${PROJECT_ROOT}/assets/redsail-cutter.desktop" "${DESKTOP_DIR}/redsail-cutter.desktop"

# 3. Install icon
echo "==> Installing icon to ${ICON_DIR}/redsail-cutter.svg..."
install -m 644 "${PROJECT_ROOT}/assets/redsail-cutter.svg" "${ICON_DIR}/redsail-cutter.svg"

# 4. Install udev rule if system mode and root
if [[ -n "${UDEV_DIR}" && ${IS_ROOT} -eq 1 ]]; then
    echo "==> Installing udev rules to ${UDEV_DIR}/99-redsail.rules..."
    install -m 644 "${PROJECT_ROOT}/assets/99-redsail.rules" "${UDEV_DIR}/99-redsail.rules"
    if command -v udevadm >/dev/null 2>&1; then
        echo "==> Reloading udev rules..."
        udevadm control --reload-rules || true
        udevadm trigger || true
    fi
elif [[ -f "${PROJECT_ROOT}/assets/99-redsail.rules" && ${IS_ROOT} -eq 0 ]]; then
    echo ""
    echo "Note: To allow non-root USB serial access for all users, you can install udev rules with:"
    echo "      sudo install -m 644 assets/99-redsail.rules /etc/udev/rules.d/99-redsail.rules"
    echo "      sudo udevadm control --reload-rules && sudo udevadm trigger"
    echo "      sudo usermod -aG dialout,uucp \$USER"
    echo ""
fi

# 5. Refresh system caches
if command -v update-desktop-database >/dev/null 2>&1; then
    update-desktop-database "${DESKTOP_DIR}" || true
fi

if [[ "${INSTALL_TYPE}" == *"User"* ]]; then
    if [[ ":$PATH:" != *":${HOME}/.local/bin:"* ]]; then
        echo "Tip: Ensure ${HOME}/.local/bin is in your PATH environment variable:"
        echo "     export PATH=\"\$HOME/.local/bin:\$PATH\""
    fi
fi

echo "--------------------------------------------------------"
echo " Installation Complete!"
echo " You can now launch Redsail Cutter:"
echo "   - From terminal: redsail-cutter gui"
echo "   - From app launcher: Search for 'Redsail Cutter'"
echo "--------------------------------------------------------"
