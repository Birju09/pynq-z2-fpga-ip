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
    git \
    pkg-config \
    gcc-12-arm-linux-gnueabihf \
    g++-12-arm-linux-gnueabihf \
    iverilog \
    bison \
    flex \
    libssl-dev \
    libssl-dev:armhf \
    libtinyxml2-dev:armhf \
    libfoonathan-memory-dev:armhf \
    libasio-dev \
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

# Cross-compile Fast-CDR 2.x and Fast-DDS 3.x for armhf into /opt/fastdds-armhf
ARG FASTCDR_VERSION=v2.2.6
ARG FASTDDS_VERSION=v3.1.2
RUN cat <<'EOF' > /tmp/armhf-fastdds.cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER /usr/bin/arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabihf-g++)
set(CMAKE_LIBRARY_ARCHITECTURE arm-linux-gnueabihf)
set(CMAKE_FIND_ROOT_PATH /opt/fastdds-armhf /usr/arm-linux-gnueabihf /usr/lib/arm-linux-gnueabihf /usr/include/arm-linux-gnueabihf /usr)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
list(APPEND CMAKE_PREFIX_PATH /opt/fastdds-armhf /usr/lib/arm-linux-gnueabihf/cmake /usr/lib/arm-linux-gnueabihf /usr/share)
EOF
RUN git clone --depth 1 --branch "${FASTCDR_VERSION}" https://github.com/eProsima/Fast-CDR.git /tmp/Fast-CDR && \
    cmake -S /tmp/Fast-CDR -B /tmp/Fast-CDR/build \
        -DCMAKE_TOOLCHAIN_FILE=/tmp/armhf-fastdds.cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/fastdds-armhf \
        -DBUILD_SHARED_LIBS=ON && \
    cmake --build /tmp/Fast-CDR/build -j"$(nproc)" && \
    cmake --install /tmp/Fast-CDR/build && \
    git clone --depth 1 --branch "${FASTDDS_VERSION}" https://github.com/eProsima/Fast-DDS.git /tmp/Fast-DDS && \
    cmake -S /tmp/Fast-DDS -B /tmp/Fast-DDS/build \
        -DCMAKE_TOOLCHAIN_FILE=/tmp/armhf-fastdds.cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/fastdds-armhf \
        -DCMAKE_CXX_FLAGS="-Wno-error=maybe-uninitialized -Wno-psabi" \
        -DBUILD_SHARED_LIBS=ON \
        -DCOMPILE_EXAMPLES=OFF \
        -DCOMPILE_TOOLS=OFF \
        -DBUILD_TESTING=OFF \
        -DSQLITE3_SUPPORT=OFF && \
    cmake --build /tmp/Fast-DDS/build -j"$(nproc)" && \
    cmake --install /tmp/Fast-DDS/build && \
    rm -rf /tmp/Fast-CDR /tmp/Fast-DDS /tmp/armhf-fastdds.cmake

WORKDIR /workspace
