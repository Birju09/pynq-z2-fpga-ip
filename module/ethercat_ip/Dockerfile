FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    gcc-12-arm-linux-gnueabihf \
    g++-12-arm-linux-gnueabihf \
    iverilog \
    bison \
    flex \
    libssl-dev \
    bc \
    kmod \
    rsync \
    libc6-armhf-cross \
    && rm -rf /var/lib/apt/lists/*

# Setup QEMU armhf dynamic linker fallback
ENV QEMU_LD_PREFIX=/usr/arm-linux-gnueabihf
RUN mkdir -p /lib && ln -sf /usr/arm-linux-gnueabihf/lib/ld-linux-armhf.so.3 /lib/ld-linux-armhf.so.3

# Set GCC 12 as default arm-linux-gnueabihf-gcc
RUN update-alternatives --install /usr/bin/arm-linux-gnueabihf-gcc arm-linux-gnueabihf-gcc /usr/bin/arm-linux-gnueabihf-gcc-12 100

WORKDIR /workspace

