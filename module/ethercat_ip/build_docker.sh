#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IP_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"
ECAT_HDL_DIR="${IP_ROOT}/hdl/ecat_master_ip"

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

    rm -f "${ECAT_HDL_DIR}/sim/sim_ecat.vvp"

    echo "===> Clean complete!"
    exit 0
fi

echo "===> [1/2] Running Icarus Verilog Hardware Simulation (40-Slave, 5 KB Ring)..."
docker run --rm \
    -v "${ECAT_HDL_DIR}:/ecat" \
    -w "/ecat/sim" \
    "${IMAGE_NAME}" \
    bash -c "iverilog -g2012 -o sim_ecat.vvp ../ecat_40slave_emulator.v ../ecat_cyclic_master_ip.v tb_ecat_master.v && vvp sim_ecat.vvp && rm -f sim_ecat.vvp"

echo "===> [2/2] Running ARM Kernel Module & Benchmark Cross-Compilation..."
docker run --rm \
    -v "${BUILD_DIR}:/build:ro" \
    -v "${SCRIPT_DIR}:/workspace" \
    -w "/workspace" \
    "${IMAGE_NAME}" \
    make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- KDIR=/build "${TARGET}"

echo "===> Build & Verification complete! Output files in module/ethercat_ip/:"
ls -lh "${SCRIPT_DIR}/axi_dma.ko" "${SCRIPT_DIR}/test_dma" 2>/dev/null || true
