# Benchmark Results: FPGA Fastpath vs. Pure Fast DDS

## Test Configuration

| Parameter | Value |
|---|---|
| Target Board | PYNQ-Z2 (Zynq-7020, dual-core ARM Cortex-A9) |
| Payload | 4,096 B (`FastSample`, 1,024 × 32-bit words) |
| Target Rate & Volume | 2,000 Hz, 20,000 samples |
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

## 2. Round-Trip with Remote Echo

| Metric | FPGA Fastpath (100 Mbps PL) | Pure Fast DDS (1 Gbps PS) |
|---|---|---|
| Effective Rate | 2,000.0 samples/s | 1,701.0 samples/s (CPU-limited) |
| Inbound RX Samples | 21,509 (`SeqGaps=39`)* | 19,995 (0.03% loss) |
| Publish Call Latency (p50 / p99) | 18.9 / 409.1 µs | 450.8 / 830.9 µs |
| RTT Min | 1,677 µs | 1,229 µs |
| RTT p50 | 2,404 µs | 2,032 µs |
| RTT p95 | 2,418 µs | 2,562 µs |
| RTT p99 | 2,602 µs | 2,866 µs |
| RTT p99.9 | 8,612 µs | 7,418 µs |
| RTT Max | 13,487 µs | 9,746 µs |
| Process CPU per Sample | 144.0 µs | 785.4 µs |
| Process CPU Usage (1-core norm.) | 28.8% | 133.6% |
| System CPU Busy (`/proc/stat`) | 3.6% | 69.7% |

*\*See Observation 4 below regarding the FPGA echo RX count.*

---

## Key Findings & Analysis

### 1. CPU Offload and Rate Sustainability
- **Lower CPU cost per sample:** Offloading CDR/RTPS/UDP/IP/Ethernet framing and checksums to PL fabric reduces process CPU time by **~3.9×** in TX-only mode (354.3 µs → 91.5 µs/sample) and **~5.5×** in round-trip mode (785.4 µs → 144.0 µs/sample).
- **Elimination of kernel network-stack load:** Total system CPU busy drops from **46.1% to 2.5%** (TX-only) and from **69.7% to 3.6%** (round-trip), reflecting the removal of Linux socket and `NET_TX`/`NET_RX` softirq overhead.
- **Sustained target rate under load:** In the round-trip test, the FPGA path maintains the full **2,000.0 samples/s** with ample CPU headroom (28.8% of one core), whereas Pure Fast DDS saturates both Cortex-A9 cores (133.6% 1-core, 69.7% system busy) and drops to **1,701.0 samples/s**.

### 2. Publish Call Latency and Tail Behavior
- **Median publish latency:** Moving framing out of software cuts median publish call latency from **330.8–450.8 µs** down to **18.7–18.9 µs** (~18×–24× reduction), freeing the application thread immediately after the DMA kick.
- **Bimodal FPGA publish tail (~370–409 µs at p95/p99):** A 4,096 B sample split into three `DATA_FRAG` Ethernet frames (~4.4–4.6 KB on the wire including headers and inter-frame gaps) requires **~355–370 µs** to serialize on a 100 Mbps RMII link. At 2,000 Hz (500 µs period), the previous packet normally drains before the next publish arrives, yielding the ~19 µs median. When timer jitter or an overlapping S2MM burst causes a write to arrive while the previous packet is still serializing on the RMII wire, the call blocks for roughly one wire-frame duration (~370 µs).

### 3. Round-Trip Latency and the 100 Mbps vs. 1 Gbps Link Difference
- **Physical link speed offset:** The FPGA path currently uses a **100 Mbps** PL RMII PHY, whereas the Pure Fast DDS baseline uses the **1 Gbps** PS Ethernet MAC. Serializing ~4.5 KB takes **~360 µs per direction (~720 µs round-trip)** on 100 Mbps versus **~36 µs per direction (~72 µs round-trip)** on 1 Gbps—an inherent ~650 µs wire-time disadvantage for the 100 Mbps PL interface.
- **Net RTT comparison:** Because of this link-speed difference and the remote x86 echo overhead (~1.2–1.6 ms in both paths), FPGA median and minimum RTT are ~370–450 µs higher than Pure Fast DDS (`p50`: 2,404 vs. 2,032 µs; `min`: 1,677 vs. 1,229 µs). Even with the 10× slower physical link, however, the FPGA path achieves **lower p95 and p99 RTT** (2,418 / 2,602 µs vs. 2,562 / 2,866 µs) because it avoids the CPU saturation and scheduling jitter seen in Pure Fast DDS at 2,000 Hz. Upgrading the PL interface to a 1 Gbps RGMII PHY would remove the ~650 µs wire-serialization penalty.

### 4. RX Accounting Caveat (FPGA Echo Run)
- The FPGA round-trip run recorded **21,509** RX samples for 20,000 sent (`SeqGaps=39`). Because the S2MM RX worker thread starts prior to the timed TX loop and counts incoming packets without filtering by the active run's start timestamp, pre-existing or non-echo RX traffic was included in the tally. As a result, the RX loss percentage for this FPGA echo run is not meaningful and should be re-measured after gating RX counters to the active TX window.

---

## Pure Fast DDS Baseline Adjustments Made During Testing

- **Self-delivery `CacheChange` exhaustion:** Initial Pure Fast DDS runs hit `"Problem reserving CacheChange"` errors caused by the board's `DataWriter` delivering samples to its own `DataReader` inside the same `DomainParticipant`. This was resolved by isolating the pure-DDS reader in a separate `DomainParticipant` that ignores the writer's participant.
- **Buffer sizing:** Reader history/resource limits and UDP socket buffers (4 MB) were increased to prevent artificial packet drops in the software baseline.
