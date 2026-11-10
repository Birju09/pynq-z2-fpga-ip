#!/usr/bin/env bash
set -e

export PATH="/usr/local/bin:${PATH}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ARM_DOCKER_CTX_DIR="${REPO_ROOT}/module/rtps_fpga_ip"
ARM_IMAGE_NAME="pynq-ecat-builder"
X86_IMAGE_NAME="pynq-fastdds-x86-builder"

ARM_BUILD_DIR="build-armhf"
X86_BUILD_DIR="build-x86_64"

TARGET="${1:-armhf}"

print_usage() {
    echo "Usage: $0 [armhf|x86|all|clean]"
    echo "  armhf   Cross-compile FPGA + Remote apps for PYNQ-Z2 ARM (default)"
    echo "  x86     Build Remote DDS app for x86_64 Ubuntu 22.04"
    echo "  all     Build both ARM (armhf) and x86_64 targets"
    echo "  clean   Remove build-armhf and build-x86_64 directories"
}

build_armhf() {
    echo "===> [ARM] Building Docker Cross-Compilation Image (${ARM_IMAGE_NAME})..."
    docker build -t "${ARM_IMAGE_NAME}" "${ARM_DOCKER_CTX_DIR}"

    echo "===> [ARM] Cross-Compiling Fast DDS + FPGA Fastpath Apps (arm-linux-gnueabihf)..."
    docker run --rm \
        -v "${REPO_ROOT}:/project" \
        -w "/project/fastdds_fpga_fastpath" \
        "${ARM_IMAGE_NAME}" \
        bash -c "
            mkdir -p ${ARM_BUILD_DIR} && \
            cd ${ARM_BUILD_DIR} && \
            cmake .. \
                -DCMAKE_BUILD_TYPE=Release \
                -DCMAKE_TOOLCHAIN_FILE=/project/fastdds_fpga_fastpath/armhf-toolchain.cmake && \
            make -j\$(nproc) && \
            echo '' && \
            echo '===> [ARM] Binary Architecture Verification:' && \
            file fpga_fastpath_node remote_dds_node
        "

    echo ""
    echo "===> [ARM] Build Complete! Binaries in ${SCRIPT_DIR}/${ARM_BUILD_DIR}/:"
    ls -lh "${SCRIPT_DIR}/${ARM_BUILD_DIR}/fpga_fastpath_node" "${SCRIPT_DIR}/${ARM_BUILD_DIR}/remote_dds_node"
}

build_x86() {
    echo "===> [x86_64] Building Docker Image for Ubuntu 22.04 x86_64 (${X86_IMAGE_NAME})..."
    docker build --platform linux/amd64 -f "${SCRIPT_DIR}/Dockerfile.x86" -t "${X86_IMAGE_NAME}" "${SCRIPT_DIR}"

    echo "===> [x86_64] Building Remote DDS App for Ubuntu 22.04 x86_64..."
    docker run --rm --platform linux/amd64 \
        -v "${REPO_ROOT}:/project" \
        -w "/project/fastdds_fpga_fastpath" \
        "${X86_IMAGE_NAME}" \
        bash -c "
            mkdir -p ${X86_BUILD_DIR} && \
            cd ${X86_BUILD_DIR} && \
            cmake .. \
                -DCMAKE_BUILD_TYPE=Release \
                -DBUILD_FPGA_APP=OFF \
                -DBUILD_REMOTE_APP=ON && \
            make -j\$(nproc) && \
            echo '' && \
            echo '===> [x86_64] Binary Architecture Verification:' && \
            file remote_dds_node
        "

    echo ""
    echo "===> [x86_64] Build Complete! Binary in ${SCRIPT_DIR}/${X86_BUILD_DIR}/:"
    ls -lh "${SCRIPT_DIR}/${X86_BUILD_DIR}/remote_dds_node"
}

case "${TARGET}" in
    armhf|arm)
        build_armhf
        ;;
    x86|x86_64|amd64|remote)
        build_x86
        ;;
    all)
        build_armhf
        echo ""
        build_x86
        ;;
    clean)
        echo "===> Cleaning ${SCRIPT_DIR}/${ARM_BUILD_DIR} and ${SCRIPT_DIR}/${X86_BUILD_DIR}..."
        rm -rf "${SCRIPT_DIR}/${ARM_BUILD_DIR}" "${SCRIPT_DIR}/${X86_BUILD_DIR}"
        echo "===> Clean complete!"
        ;;
    -h|--help|help)
        print_usage
        exit 0
        ;;
    *)
        echo "[-] Unknown target: ${TARGET}"
        print_usage
        exit 1
        ;;
esac
