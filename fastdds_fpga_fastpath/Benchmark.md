# Benchmark Results: FPGA Fastpath vs. Pure Fast DDS

## Test Configuration

| Parameter | Value |
|---|---|
| Target Board | PYNQ-Z2 (Zynq-7020, dual-core ARM Cortex-A9) |
| Payload | 4,096 B (`FastSample`, 1,024 × 32-bit words) |
| Target Rate & Volume | 2,000 Hz, 1,000,000 samples |
| QoS | Best-Effort, UDPv4 transport |
| Peer | Remote x86 workstation running standard Fast DDS (`remote_dds_node`) |
| Network Interface | FPGA Fastpath: **100 Mbps** PL RMII PHY (`100BASE-TX`)<br>Pure Fast DDS: **1 Gbps** PS `eth0` (`1000BASE-T`) |

> **Measurement Note:** Round-trip time (RTT) is measured on the PYNQ-Z2 from the monotonic timestamp stamped immediately before publishing to the RX callback on the board, and therefore includes wire serialization in both directions plus the remote x86 echo path.

---

## 1. TX-Only (No Echo)

| Metric | FPGA Fastpath (100 Mbps PL) | Pure Fast DDS (1 Gbps PS) |
|---|---|---|
| Effective Rate | 2,000.0 samples/s | 1,999.9 samples/s |
| Publish Call Latency (p50 / p99) | 18.7 / 374.1 µs | 330.8 / 389.9 µs |
| Process CPU per Sample | 91.5 µs | 354.3 µs |
| Process CPU Usage (1-core norm.) | 18.3% | 70.9% |
| System CPU Busy (`/proc/stat`) | 2.5% | 46.1% |

---

## 2. Round-Trip with Remote Echo (Decoupled Async FSMs, 2,000 Hz)

### 2a. Head-to-Head Comparison (10,000 samples)

| Metric | FPGA Fastpath (100 Mbps PL) | Pure Fast DDS (1 Gbps PS) |
|---|---|---|
| Effective Rate | **2,000.0 samples/s** (65.5 Mbps) | 1,921.7 samples/s (63.0 Mbps, CPU-limited) |
| Inbound RX Samples | **10,000 (0.00% loss, SeqGaps=0)** | 9,998 (0.02% loss, SeqGaps=0) |
| Publish Call Latency (Min / p50 / Avg) | **17.5 / 19.2 / 79.6 µs** | 276.7 / 365.1 / 451.6 µs |
| Publish Call Latency (p95 / p99 / Max) | **369.6 / 380.0 / 467.9 µs** | 695.7 / 755.1 / 1,461.4 µs |
| RTT Min | 1,839.9 µs | **1,127.7 µs** |
| RTT p50 | 2,131.7 µs | **1,893.3 µs** |
| RTT Avg | 2,137.9 µs | **1,881.2 µs** |
| RTT p95 | **2,360.2 µs** | 2,465.3 µs |
| RTT p99 | **2,426.9 µs** | 2,629.3 µs |
| RTT p99.9 | **2,603.6 µs** | 3,496.9 µs |
| RTT Max | **2,699.7 µs** | 4,398.4 µs |
| Process CPU per Sample | **133.09 µs** (User 2.81 / Sys 130.29 µs) | 683.30 µs (User 264.68 / Sys 418.62 µs) |
| Process CPU Usage (1-core norm.) | **26.62%** (13.31% system-wide) | 131.31% (65.66% system-wide) |
| System CPU Busy (`/proc/stat`) | **10.45%** (SoftIRQ: **0.00%**) | 68.18% (SoftIRQ: 13.20%) |

### 2b. Sustained Run (FPGA Fastpath, 100,000 samples @ 2,000 Hz)

| Metric | FPGA Fastpath (100 Mbps PL) |
|---|---|
| Elapsed Time | 50,000.2 ms |
| Effective Rate | **2,000.0 samples/s** (65.5 Mbps) |
| Inbound RX Samples | **99,982** (0.02% loss, Corrupted=0, SeqGaps=23) |
| Publish Call Latency (Min / p50 / Avg) | **17.5 / 19.3 / 127.5 µs** |
| Publish Call Latency (p95 / p99 / p99.9 / Max) | **371.3 / 387.4 / 470.3 / 4,354.2 µs** |
| RTT (Min / p50 / Avg) | 1,483.5 / 2,131.9 / 2,144.2 µs |
| RTT (p95 / p99 / p99.9 / Max) | **2,360.5 / 2,435.4 / 2,713.6 / 5,999.4 µs** |
| Process CPU per Sample | **139.15 µs** (User 9.81 / Sys 129.34 µs) |
| Process CPU Usage (1-core norm.) | **27.83%** (13.91% system-wide) |
| System CPU Busy (`/proc/stat`) | **10.19%** (SoftIRQ: **0.02%**) |
| PL RTL Counters | `stat_tx_samples=102,023`, `stat_rx_samples=101,980`, `stat_stale_drops=0`, `stat_csum_errs=0` |

---

## Key Findings & Analysis

### 1. CPU Offload and Rate Sustainability
- **Lower CPU cost per sample:** Offloading CDR/RTPS/UDP/IP/Ethernet framing and checksums to PL fabric reduces process CPU time by **~3.9×** in TX-only mode (354.3 µs → 91.5 µs/sample) and **~5.1×** in round-trip mode (683.30 µs → 133.09 µs/sample).
- **Elimination of kernel network-stack load:** Total system CPU busy drops from **46.1% to 2.5%** (TX-only) and from **68.18% to 10.45%** (round-trip), with Linux SoftIRQ overhead dropping from **13.20% to 0.00%–0.02%**.
- **Sustained target rate under load:** In the round-trip test, the FPGA path maintains the full **2,000.0 samples/s** across both 10k and 100k sample runs with ample CPU headroom (26.6%–27.8% of one core), whereas Pure Fast DDS saturates both Cortex-A9 cores (131.3% 1-core, 68.18% system busy) and drops to **1,921.7 samples/s**.

### 2. Publish Call Latency and Tail Behavior
- **Median publish latency:** Moving framing out of software cuts median publish call latency from **330.8–365.1 µs** down to **18.7–19.3 µs** (~18×–19× reduction), freeing the application thread immediately after the DMA kick.
- **Bimodal FPGA publish tail (~370–387 µs at p95/p99):** A 4,096 B sample split into three `DATA_FRAG` Ethernet frames (~4.4–4.6 KB on the wire including headers and inter-frame gaps) requires **~355–370 µs** to serialize on a 100 Mbps RMII link. At 2,000 Hz (500 µs period), the previous packet normally drains before the next publish arrives, yielding the ~19 µs median. When timer jitter causes a write to arrive while the previous packet is still serializing on the RMII wire, the call blocks for roughly one wire-frame duration (~370 µs).

### 3. Round-Trip Latency and Decoupled FSM Impact
- **Physical link speed offset:** The FPGA path currently uses a **100 Mbps** PL RMII PHY, whereas the Pure Fast DDS baseline uses the **1 Gbps** PS Ethernet MAC. Serializing ~4.5 KB takes **~360 µs per direction (~720 µs round-trip)** on 100 Mbps versus **~36 µs per direction (~72 µs round-trip)** on 1 Gbps—an inherent ~650 µs wire-time disadvantage for the 100 Mbps PL interface.
- **Deterministic tail latency after FSM decoupling:** Because MM2S TX and S2MM RX egress now run on independent FSMs without blocking each other, FPGA tail latency stays tightly bounded even at 2,000 Hz: **p95 (2,360 µs), p99 (2,427–2,435 µs), p99.9 (2,604–2,714 µs), and Max (2,700 µs on 10k)** all outperform Pure Fast DDS (**p95: 2,465 µs, p99: 2,629 µs, p99.9: 3,497 µs, Max: 4,398 µs**) despite the 10× slower physical link. Upgrading the PL interface to a 1 Gbps RGMII PHY would remove the ~650 µs wire-serialization penalty.

---

## Pure Fast DDS Baseline Adjustments Made During Testing

- **Self-delivery `CacheChange` exhaustion:** Initial Pure Fast DDS runs hit `"Problem reserving CacheChange"` errors caused by the board's `DataWriter` delivering samples to its own `DataReader` inside the same `DomainParticipant`. This was resolved by isolating the pure-DDS reader in a separate `DomainParticipant` that ignores the writer's participant.
- **Buffer sizing:** Reader history/resource limits and UDP socket buffers (4 MB) were increased to prevent artificial packet drops in the software baseline.
