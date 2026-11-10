cmake_minimum_required(VERSION 3.16)
project(fastdds_fpga_fastpath VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build options
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    option(BUILD_FPGA_APP "Build the PYNQ-Z2 FPGA Fastpath Node" ON)
else()
    option(BUILD_FPGA_APP "Build the PYNQ-Z2 FPGA Fastpath Node" OFF)
endif()
option(BUILD_REMOTE_APP "Build the Remote Workstation Fast DDS Node" ON)

# Locate FastCDR and Fast DDS (supports both Fast DDS 2.x and 3.x)
find_package(fastcdr REQUIRED)
find_package(fastdds QUIET)
if(TARGET fastdds)
    set(FASTDDS_TARGET fastdds)
    message(STATUS "Found Fast DDS 3.x (target: fastdds)")
else()
    find_package(fastrtps REQUIRED)
    set(FASTDDS_TARGET fastrtps)
    message(STATUS "Found Fast DDS 2.x (target: fastrtps)")
endif()

find_package(Threads REQUIRED)

# FastSample IDL Type Library
add_library(fast_sample_types STATIC
    idl/FastSamplePubSubTypes.cxx
)
target_include_directories(fast_sample_types PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/idl
)
target_link_libraries(fast_sample_types PUBLIC
    ${FASTDDS_TARGET}
    fastcdr
)

# 1. PYNQ-Z2 FPGA Hybrid Fastpath Application (Option 3)
if(BUILD_FPGA_APP)
    set(FPGA_DRIVER_INC "${CMAKE_CURRENT_SOURCE_DIR}/../module/rtps_fpga_ip")
    if(NOT EXISTS "${FPGA_DRIVER_INC}/axi_rtps.h")
        message(FATAL_ERROR "Cannot find axi_rtps.h in ${FPGA_DRIVER_INC}")
    endif()

    add_executable(fpga_fastpath_node
        fpga_app/main_fpga.cpp
    )
    target_include_directories(fpga_fastpath_node PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/idl
        ${FPGA_DRIVER_INC}
    )
    target_link_libraries(fpga_fastpath_node PRIVATE
        fast_sample_types
        ${FASTDDS_TARGET}
        fastcdr
        Threads::Threads
    )
    target_compile_options(fpga_fastpath_node PRIVATE -O3 -Wall -Wextra)
endif()

# 2. Remote Workstation Standard Fast DDS Node
if(BUILD_REMOTE_APP)
    add_executable(remote_dds_node
        remote_app/main_remote.cpp
    )
    target_include_directories(remote_dds_node PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/idl
    )
    target_link_libraries(remote_dds_node PRIVATE
        fast_sample_types
        ${FASTDDS_TARGET}
        fastcdr
        Threads::Threads
    )
    target_compile_options(remote_dds_node PRIVATE -O3 -Wall -Wextra)
endif()
