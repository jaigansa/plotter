#!/usr/bin/env bash
set -euo pipefail

# -----------------------------------------------------------------------------
# Redsail Cutter - Build Script
# -----------------------------------------------------------------------------

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${PROJECT_ROOT}"

export CARGO_HOME="${CARGO_HOME:-/tmp/cargo}"

run_cargo() {
    cargo "$@" 2>&1 | awk '
        /warning: failed to save last-use data/ { skip=6; next }
        skip > 0 { skip--; next }
        { print }
    '
    return "${PIPESTATUS[0]}"
}

MODE="release"
DO_CLEAN=0
DO_CHECK_ONLY=0
DO_TEST=0

print_usage() {
    cat <<EOF
Usage: ./build.sh [OPTIONS]

Options:
    --release      Build optimized release binary (default)
    --debug        Build unoptimized debug binary
    --check        Run cargo check and linting only (no binary output)
    --test         Run unit tests
    --clean        Clean build directory before compiling
    -h, --help     Show this help message
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --release)
            MODE="release"
            shift
            ;;
        --debug)
            MODE="debug"
            shift
            ;;
        --check)
            DO_CHECK_ONLY=1
            shift
            ;;
        --test)
            DO_TEST=1
            shift
            ;;
        --clean)
            DO_CLEAN=1
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

echo "==> Redsail Cutter Build"
echo "    Project Root: ${PROJECT_ROOT}"
echo "    Cargo Home:   ${CARGO_HOME}"

if [[ ${DO_CLEAN} -eq 1 ]]; then
    echo "==> Cleaning build artifacts..."
    run_cargo clean
fi

if [[ ${DO_CHECK_ONLY} -eq 1 ]]; then
    echo "==> Running cargo check..."
    run_cargo check
    echo "==> Check passed successfully!"
    exit 0
fi

if [[ ${DO_TEST} -eq 1 ]]; then
    echo "==> Running cargo test..."
    run_cargo test
    echo "==> Tests passed successfully!"
    exit 0
fi

START_TIME=$(date +%s)

if [[ "${MODE}" == "release" ]]; then
    echo "==> Compiling in RELEASE mode (--release)..."
    run_cargo build --release
    TARGET_BIN="${PROJECT_ROOT}/target/release/redsail-cutter"
    
    if command -v strip >/dev/null 2>&1; then
        echo "==> Stripping release binary..."
        strip "${TARGET_BIN}" 2>/dev/null || true
    fi
else
    echo "==> Compiling in DEBUG mode..."
    run_cargo build
    TARGET_BIN="${PROJECT_ROOT}/target/debug/redsail-cutter"
fi

END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))

if [[ -f "${TARGET_BIN}" ]]; then
    BIN_SIZE=$(du -h "${TARGET_BIN}" | cut -f1)
    echo "--------------------------------------------------------"
    echo " Build Complete in ${DURATION}s!"
    echo " Mode:        ${MODE}"
    echo " Binary:      ${TARGET_BIN}"
    echo " Binary Size: ${BIN_SIZE}"
    echo "--------------------------------------------------------"
else
    echo "Error: Target binary not found at ${TARGET_BIN}" >&2
    exit 1
fi
