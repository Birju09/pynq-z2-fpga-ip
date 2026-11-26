# Benchmark Results: FPGA Fastpath vs. Pure Fast DDS

## Test Configuration

| Parameter | Value |
|---|---|
| Target Board | PYNQ-Z2 (Zynq-7020, dual-core ARM Cortex-A9) |
| Payload | 4,096 B (`FastSample`, 1,024 × 32-bit words) |
| Target Rate | 2,000 Hz |
| QoS | Best-Effort, UDPv4 transport |
| Peer | Remote x86 workstation running standard Fast DDS (`remote_dds_node`) |
| Network Interface | FPGA Fastpath: **100 Mbps** PL RMII PHY (`100BASE-TX`)<br>Pure Fast DDS: **1 Gbps** PS `eth0` (`1000BASE-T`) |

> **Measurement Note:** Round-trip time (RTT) is measured on the PYNQ-Z2 from the monotonic timestamp stamped immediately before publishing to the RX callback on the board, and includes wire serialization in both directions plus the remote x86 echo path.

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

## 2. Round-Trip with Remote Echo (2,000 Hz)

### Sustained Run (100,000 samples @ 2,000 Hz)

| Metric | FPGA Fastpath (100 Mbps PL) | Pure Fast DDS (1 Gbps PS) |
|---|---|---|
| Elapsed Time | 50,000.2 ms | 50,000.9 ms |
| Effective Rate | **2,000.0 samples/s** (65.5 Mbps) | 2,000.0 samples/s (65.5 Mbps) |
| Publish Call Latency (Min / p50 / Avg) | **17.1 / 18.7 / 19.6 µs** | 261.5 / 341.8 / 416.0 µs |
| Publish Call Latency (p95 / p99 / p99.9 / Max) | **23.0 / 23.3 / 29.0 / 789.8 µs** | 671.2 / 706.2 / 792.8 / 1,547.2 µs |
| Inbound RX Samples | 1 *(Corrupted=1; PL `stat_rx_samples=300,000`)* | 99,992 (0.01% loss, Corrupted=0, SeqGaps=0) |
| Round-Trip Latency (Min / p50 / Avg / p99 / Max) | — | 929.4 / 1,822.1 / 1,824.3 / 2,527.1 / 4,371.9 µs |
| Process CPU per Sample | **58.13 µs** (User 28.98 / Sys 29.15 µs) | 656.58 µs (User 261.42 / Sys 395.16 µs) |
| Process CPU Usage (1-core norm.) | **11.63%** (5.81% system-wide) | 131.31% (65.66% system-wide) |
| Context Switches | **1.02 / sample** (Vol=101,835, Invol=2) | 1.32 / sample (Vol=130,917, Invol=1,149) |
| System CPU Busy (`/proc/stat`) | **0.15%** (SoftIRQ: **0.00%**) | 69.38% (SoftIRQ: 11.31%) |
| Process RSS / Peak HWM | **8.88 → 9.62 MB** (Peak 9.62 MB) | 9.25 → 10.75 MB (Peak 10.75 MB) |
| PL RTL Counters | `stat_tx_samples=300,000`, `stat_rx_samples=300,000`, `stat_seq_gaps=0`, `stat_stale_drops=0`, `stat_csum_errs=0`, `last_cycle_ticks=32,145` | — |
