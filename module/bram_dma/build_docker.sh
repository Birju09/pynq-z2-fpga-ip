#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"

IMAGE_NAME="pynq-dma-builder"

TARGET="${1:-all}"
if [ "$TARGET" = "make" ]; then
    shift
    TARGET="${1:-all}"
fi

echo "===> Building Docker Cross-Compiler Image..."
echo "===> Image Name: ${IMAGE_NAME}"
echo "===> Build Context: ${SCRIPT_DIR}"
docker build -t "${IMAGE_NAME}" "${SCRIPT_DIR}"

if [ "$TARGET" = "clean" ]; then
    echo "===> Running make clean inside Docker..."
    docker run --rm \
        -v "${BUILD_DIR}:/build:ro" \
        -v "${SCRIPT_DIR}:/workspace" \
        -w "/workspace" \
        "${IMAGE_NAME}" \
        make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- KDIR=/build clean

    echo "===> Clean complete!"
else
    echo "===> Running Cross-Compilation inside Docker..."
    docker run --rm \
        -v "${BUILD_DIR}:/build:ro" \
        -v "${SCRIPT_DIR}:/workspace" \
        -w "/workspace" \
        "${IMAGE_NAME}" \
        make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- KDIR=/build "${TARGET}"

    echo "===> Build complete! Output files in module/:"
    ls -lh "${SCRIPT_DIR}/axi_dma.ko" "${SCRIPT_DIR}/test_dma" 2>/dev/null || true
fi
