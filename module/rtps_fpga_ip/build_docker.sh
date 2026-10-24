#!/usr/bin/env bash
set -e

export PATH="/usr/local/bin:${PATH}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IP_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"
RTPS_HDL_DIR="${IP_ROOT}/hdl/rtps_fastpath_ip"

IMAGE_NAME="pynq-ecat-builder"

TARGET="${1:-all}"
if [ "$TARGET" = "make" ]; then
    shift
    TARGET="${1:-all}"
fi

echo "===> Building Docker Cross-Compiler & Verilog Sim Image..."
docker build -t "${IMAGE_NAME}" "${SCRIPT_DIR}"

if [ "$TARGET" = "clean" ]; then
    echo "===> Running make clean inside Docker..."
    docker run --rm \
        -v "${BUILD_DIR}:/build:ro" \
        -v "${SCRIPT_DIR}:/workspace" \
        -w "/workspace" \
        "${IMAGE_NAME}" \
        make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- KDIR=/build clean

    rm -f "${RTPS_HDL_DIR}/sim/sim_rtps.vvp"

    echo "===> Clean complete!"
    exit 0
fi

echo "===> [1/2] Running Icarus Verilog Hardware Simulation (4 KB RTPS v2.x Fast-Path & Option B Plugin)..."
docker run --rm \
    -v "${RTPS_HDL_DIR}:/rtps" \
    -w "/rtps/sim" \
    "${IMAGE_NAME}" \
    bash -c "iverilog -o sim_rtps.vvp ../rtps_peer_emulator.v ../rtps_fastpath_ip.v tb_rtps_fastpath.v && vvp sim_rtps.vvp && rm -f sim_rtps.vvp"

echo "===> [2/2] Running ARM Kernel Module & Option B Transport Plugin Cross-Compilation..."
docker run --rm \
    -v "${BUILD_DIR}:/build:ro" \
    -v "${SCRIPT_DIR}:/workspace" \
    -w "/workspace" \
    "${IMAGE_NAME}" \
    make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- KDIR=/build "${TARGET}"

echo "===> Build & Verification complete! Output files in module/rtps_fpga_ip/:"
ls -lh "${SCRIPT_DIR}/axi_rtps.ko" "${SCRIPT_DIR}/test_dma" 2>/dev/null || true

