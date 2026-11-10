FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Enable armhf multiarch for Fast DDS / FastCDR cross-compilation libraries
RUN dpkg --add-architecture armhf && \
    if [ "$(dpkg --print-architecture)" = "amd64" ]; then \
        sed -i 's/^deb /deb [arch=amd64] /g' /etc/apt/sources.list && \
        echo "deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports/ jammy main restricted universe multiverse" >> /etc/apt/sources.list && \
        echo "deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports/ jammy-updates main restricted universe multiverse" >> /etc/apt/sources.list && \
        echo "deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports/ jammy-security main restricted universe multiverse" >> /etc/apt/sources.list; \
    else \
        sed -i 's/^deb /deb [arch=arm64,armhf] /g' /etc/apt/sources.list; \
    fi

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    file \
    pkg-config \
    gcc-12-arm-linux-gnueabihf \
    g++-12-arm-linux-gnueabihf \
    iverilog \
    bison \
    flex \
    libssl-dev \
    libssl-dev:armhf \
    libtinyxml2-dev:armhf \
    libfastrtps-dev:armhf \
    libfastcdr-dev:armhf \
    bc \
    kmod \
    rsync \
    libc6-armhf-cross \
    && rm -rf /var/lib/apt/lists/*

# Setup QEMU armhf dynamic linker fallback
ENV QEMU_LD_PREFIX=/usr/arm-linux-gnueabihf
RUN mkdir -p /lib && ln -sf /usr/arm-linux-gnueabihf/lib/ld-linux-armhf.so.3 /lib/ld-linux-armhf.so.3

# Set GCC/G++ 12 as default arm-linux-gnueabihf compilers
RUN update-alternatives --install /usr/bin/arm-linux-gnueabihf-gcc arm-linux-gnueabihf-gcc /usr/bin/arm-linux-gnueabihf-gcc-12 100 && \
    update-alternatives --install /usr/bin/arm-linux-gnueabihf-g++ arm-linux-gnueabihf-g++ /usr/bin/arm-linux-gnueabihf-g++-12 100

WORKDIR /workspace
