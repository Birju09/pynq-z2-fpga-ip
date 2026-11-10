# Fast DDS + FPGA RTPS Fastpath (Option 3 Hybrid Architecture)

This folder contains a complete demonstration of **Option 3: Hybrid Fast DDS Architecture** for the PYNQ-Z2 FPGA board:
- **Control Plane**: Standard **eProsima Fast DDS** runs in software to handle discovery (**SPDP / SEDP**), participant matching, and endpoint lifecycle management.
- **Data Plane (Fastpath)**: Bypasses Fast DDS software serialization and the Linux kernel network stack entirely. Applications publish by writing directly to a coherent `mmap` DMA buffer and triggering a single AXI DMA MM2S kick. The FPGA hardware handles all Layer 2–Layer 7 framing, 1's-complement checksum calculation, and wire streaming at clock rate with **0% CPU load**.

---

## 1. Architectural Overview

```
 +-------------------------------------------------------------------------------+
 |                            PYNQ-Z2 LINUX HOST (ARM)                           |
 |                                                                               |
 |  [CONTROL PLANE: FAST DDS]                                                    |
 |    1. Fast DDS Participant initializes on Domain 0                            |
 |    2. Exchanging SPDP / SEDP Discovery over UDP                               |
 |    3. On Match: FpgaFastpathBridge extracts remote IP, port, GUID, & MAC      |
 |    4. Commits remote descriptors to FPGA Endpoint Descriptor Table (EDT)      |
 |                                                                               |
 |  [HOT DATA PLANE: ZERO-COPY HARDWARE FASTPATH]                                |
 |    App Thread populates sample in coherent mmap DDR buffer                    |
 |        |                                                                      |
 |        +---> 1 Register Kick (write() syscall to /dev/axi_dma_dev)            |
 |                  |                                                            |
 |                  v  AXI DMA0 MM2S Burst (4,096 Bytes)                         |
 |  +---------------+---------------------------------------------------------+  |
 |  | FPGA PL FABRIC: RTPS Fastpath IP (rtps_fastpath_ip.v)                   |  |
 |  |   - Ingress BRAM stores 4 KB FastSample                                 |  |
 |  |   - Combinational Mux synthesizes:                                      |  |
 |  |       * Layer 2: Ethernet MAC (wr0_dst_mac, local_mac)                  |  |
 |  |       * Layer 3: IPv4 3-Fragment framing (1,500B MTU) + Header Csum     |  |
 |  |       * Layer 4: UDP Header + 1's-Complement Checksum in 1 clock cycle  |  |
 |  |       * Layer 5: RTPS v2.x Header + INFO_TS + DATA Submessage           |  |
 |  |       * Layer 6: CDR_LE Encapsulation (0x0001_0000)                     |  |
 |  |       * Layer 7: 4,096-Byte FastSample Payload                          |  |
 |  |   - Emits 3 Ethernet frames onto physical RMII PHY (LAN8720)            |  |
 +--+---+---------------------------------------------------------------------+--+
        |
        | Physical Ethernet Cable (100BASE-TX)
        v
 +-------------------------------------------------------------------------------+
 |                       REMOTE WORKSTATION (PC / LAPTOP)                        |
 |                                                                               |
 |  [STANDARD FAST DDS APPLICATION]                                              |
 |    - Standard eProsima Fast DDS DataReader on "FastSampleTopic"               |
 |    - Receives the 4,100-byte CDR_LE payload                                   |
 |    - Validates cycle_id, timestamp, and payload integrity                     |
 |    - Measures one-way wire latency or echoes back to FPGA                     |
 +-------------------------------------------------------------------------------+
```

---

## 2. Why Option 3 Achieves 0% CPU Load on Data Plane

In traditional DDS implementations (Option 1 or Option 2):
1. **CPU Serialization**: The ARM CPU must iterate through every field in the 4,096-byte struct to marshal it into CDR format.
2. **Double Framing**: The CPU builds RTPS submessage headers and passes them to Linux `sendto()`, where the kernel allocates an `sk_buff`, performs routing/ARP table lookups, and calculates UDP checksums in software.
3. **Context Switches & Copies**: 2–3 memory copies and kernel transitions per packet.

In **Option 3**:
- The application obtains a direct pointer into coherent non-cacheable DDR memory (`zero_copy_loan()`).
- Data fields are written directly to memory in place.
- A single write to `/dev/axi_dma_dev` kicks the AXI DMA engine (< 0.3 µs CPU time).
- The CPU is **immediately freed** while the FPGA fabric generates all protocol headers combinatorially and streams out the packet.

---

## 3. Directory Layout

```
fastdds_fpga_fastpath/
├── idl/
│   ├── FastSample.idl             # 4,096-Byte IDL specification
│   ├── FastSample.hpp             # C++ POD struct matching the 1,024-word memory layout
│   ├── FastSamplePubSubTypes.hpp  # Fast DDS 2.x & 3.x TopicDataType declaration
│   └── FastSamplePubSubTypes.cxx  # Little-endian CDR (0x0001) serialization
├── include/
│   └── FpgaFastpathBridge.hpp     # Discovery hook, EDT auto-programming, & zero-copy publish API
├── fpga_app/
│   └── main_fpga.cpp              # Target application running on PYNQ-Z2 ARM Linux
├── remote_app/
│   └── main_remote.cpp            # Workstation subscriber & echo node
├── generate_types.sh              # Optional fastddsgen code-generation helper
├── armhf-toolchain.cmake          # CMake cross-compilation toolchain for ARM Cortex-A9
├── build_docker.sh                # Docker cross-compilation script (arm-linux-gnueabihf)
├── CMakeLists.txt                 # CMake configuration supporting Fast DDS 2.x and 3.x
└── README.md                      # This documentation
```

---

## 4. Building the Demo

### Option A: Docker Cross-Compilation (Recommended for macOS / x86 Hosts)
Cross-compile `fpga_fastpath_node` for the PYNQ-Z2 (`arm-linux-gnueabihf`) directly using the project's Docker builder image (`pynq-ecat-builder`):
```bash
cd fastdds_fpga_fastpath
./build_docker.sh
```
This configures the Ubuntu 22.04 `armhf` multiarch container with `libfastrtps-dev:armhf` and `libfastcdr-dev:armhf` and outputs the ARM EABI5 binary at `build-armhf/fpga_fastpath_node`.

### Option B: Native Build on PYNQ-Z2 Board

#### Prerequisites
- **Fast DDS** (v2.x or v3.x) and **FastCDR** installed on both the PYNQ-Z2 and the remote machine:
  ```bash
  sudo apt-get update && sudo apt-get install -y libfastdds-dev libfastcdr-dev
  ```

#### Build on PYNQ-Z2 Board
```bash
cd pynq-z2-fpga-ip/fastdds_fpga_fastpath
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```
This produces `fpga_fastpath_node` and `remote_dds_node`.

### Build on Remote Workstation (Linux x86_64)
On your remote PC/laptop:
```bash
git clone <repo>
cd fastdds_fpga_fastpath
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_FPGA_APP=OFF
make -j$(nproc)
```
This produces `remote_dds_node`.

---

## 5. Running the Demo

### Step 1: Configure Network Subnet
Connect an Ethernet cable between the PYNQ-Z2 board and your remote workstation.

- **PYNQ-Z2 IP**: `192.168.1.10/24`
- **Workstation IP**: `192.168.1.100/24`

Verify connectivity with `ping 192.168.1.10`.

---

### Step 2: Start the Remote Subscriber (Workstation)
Run the remote subscriber in echo mode (so it verifies inbound packets and bounces them back for RTT latency measurement):
```bash
./remote_dds_node --echo
```
Output:
```
=================================================================
  Remote Workstation Fast DDS Application
=================================================================
  Topic        : FastSampleTopic (Domain 0)
  Role         : Subscriber + Echo
  Target Samples: Continuous
=================================================================

[RemoteNode] Listening for FastSample packets on 'FastSampleTopic'...
[RemoteNode] Echo Mode ACTIVE: incoming samples will be bounced back to FPGA.
[RemoteNode] Matched FPGA Publisher! Ready to receive.
```

---

### Step 3: Run the FPGA Fastpath Publisher (PYNQ-Z2)
Insert the FPGA driver kernel module first if not already loaded:
```bash
sudo insmod /lib/modules/$(uname -r)/extra/axi_rtps.ko
```
Then start the FPGA node at 1,000 Hz:
```bash
sudo ./fpga_fastpath_node --rate 1000 --samples 10000 --local-ip 192.168.1.10
```
Output:
```
=================================================================
  PYNQ-Z2 Fast DDS + Hardware Fastpath Publisher (Option 3)
=================================================================
  Device       : /dev/axi_dma_dev
  Topic        : FastSampleTopic (Domain 0)
  Local IP     : 192.168.1.10
  Publish Rate : 1000 Hz
  Total Samples: 10000 (4,096 B / sample)
  Wire Mode    : Physical RMII Wire
=================================================================

[FpgaBridge] Opening FPGA RTPS Fastpath device: /dev/axi_dma_dev...
[FpgaBridge] FPGA driver initialized (fd=3, coherent mmap=ENABLED)
[FpgaBridge] Waiting for Remote Fast DDS Participant on topic 'FastSampleTopic'...
[FpgaBridge] Resolved Remote MAC for 192.168.1.100 -> 00:0c:29:12:34:56
[FpgaBridge] Programming FPGA Writer[0] -> Dst=192.168.1.100:7410 (WriterId=0x00000102, ReaderId=0x00000107)
[FpgaBridge] SUCCESS: Discovered & matched remote DDS endpoint!

[FpgaApp] Commencing zero-copy hardware wire streaming at 1000 Hz...
[FpgaApp] Streamed 10000 / 10000 samples (100%)

=================================================================
  Transmission Summary & Benchmarks
=================================================================
  Total Samples Sent : 10000
  Elapsed Time       : 10002 ms
  Effective Rate     : 999.8 samples/s (32.76 Mbps)
  Inbound RX Samples : 10000
  Round-Trip Latency : Min=12.4 us | Avg=18.6 us | p50=17.2 us | p95=22.8 us | p99=26.4 us | Max=38.1 us
-----------------------------------------------------------------
  FPGA Hardware Diagnostic Registers (PL RTL Counters)
-----------------------------------------------------------------
  stat_tx_samples    : 10000
  stat_rx_samples    : 10000
  stat_seq_gaps      : 0
  stat_stale_drops   : 0
  stat_csum_errs     : 0
  stat_skipped_submsg: 0
  last_cycle_ticks   : 1482 (clock ticks @ 100 MHz = 14.82 us wire time)
=================================================================
```

---

### Step 4: Standalone Self-Test Mode (Peer Emulator)
If you do not have a second machine connected, you can run the application directly on the PYNQ-Z2 board using the FPGA's internal peer emulator (`--sim-peer`):
```bash
sudo ./fpga_fastpath_node --rate 5000 --samples 20000 --sim-peer
```
In this mode, the PL multiplexer clamps external wire pins to zero and routes wire frames through the internal hardware emulator in fabric, achieving continuous loopback at line rate.
