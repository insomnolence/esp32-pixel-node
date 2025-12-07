#!/bin/bash
# Build script for ESP32 LED Mesh Firmware
# Runs inside Docker container

set -e

# Source ESP-IDF environment
. /opt/esp-idf/export.sh

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Output directory for binaries
OUTPUT_DIR="/project/release"

# Supported targets
TARGETS=("esp32" "esp32c3" "esp32c2" "esp32s3")

print_banner() {
    echo ""
    echo "========================================"
    echo "  ESP32 LED Mesh Firmware Builder"
    echo "  ESP-IDF: $(idf.py --version 2>/dev/null || echo 'v6.0')"
    echo "========================================"
    echo ""
}

build_target() {
    local target=$1
    echo -e "${YELLOW}Building for target: ${target}${NC}"

    # Clean previous build
    rm -rf /project/build

    # Set target
    idf.py set-target ${target}

    # Build
    idf.py build

    # Create output directory
    mkdir -p ${OUTPUT_DIR}/${target}

    # Copy binaries
    cp /project/build/esp32_led_mesh.bin ${OUTPUT_DIR}/${target}/
    cp /project/build/bootloader/bootloader.bin ${OUTPUT_DIR}/${target}/
    cp /project/build/partition_table/partition-table.bin ${OUTPUT_DIR}/${target}/

    # Copy flash args for reference
    cp /project/build/flash_args ${OUTPUT_DIR}/${target}/ 2>/dev/null || true
    cp /project/build/flasher_args.json ${OUTPUT_DIR}/${target}/ 2>/dev/null || true

    # Generate merged binary for easy flashing
    # Must run from build directory since flash_args uses relative paths
    cd /project/build
    esptool --chip ${target} merge-bin \
        -o ${OUTPUT_DIR}/${target}/esp32_led_mesh_merged.bin \
        --flash-mode dio \
        --flash-size 4MB \
        $(cat flash_args)
    cd /project

    echo -e "${GREEN}Successfully built for ${target}${NC}"
    echo "Output: ${OUTPUT_DIR}/${target}/"
    echo ""
}

show_usage() {
    echo "Usage: docker run --rm -v \$(pwd):/project esp32-led-mesh-builder [TARGET]"
    echo ""
    echo "Targets:"
    echo "  esp32     - Build for ESP32 (dual-core)"
    echo "  esp32c3   - Build for ESP32-C3 (single-core RISC-V)"
    echo "  esp32c2   - Build for ESP32-C2 (single-core RISC-V)"
    echo "  esp32s3   - Build for ESP32-S3 (dual-core)"
    echo "  all       - Build for all supported targets"
    echo "  shell     - Open interactive shell"
    echo ""
    echo "Output binaries will be in ./release/<target>/"
}

print_banner

case "$1" in
    esp32|esp32c3|esp32c2|esp32s3)
        build_target "$1"
        ;;
    all)
        echo "Building for all targets..."
        for target in "${TARGETS[@]}"; do
            build_target "$target"
        done
        echo -e "${GREEN}All builds complete!${NC}"
        echo "Binaries available in: ${OUTPUT_DIR}/"
        ;;
    shell)
        echo "Starting interactive shell..."
        echo "ESP-IDF environment is ready. Use 'idf.py' commands."
        exec /bin/bash
        ;;
    help|--help|-h)
        show_usage
        ;;
    *)
        echo -e "${RED}Unknown target: $1${NC}"
        show_usage
        exit 1
        ;;
esac
