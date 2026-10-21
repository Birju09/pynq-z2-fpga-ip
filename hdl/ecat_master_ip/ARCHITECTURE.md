# Hybrid PS+PL EtherCAT Cyclic Master & 40-Slave Ring Emulator Architecture

## 1. Executive Summary & Design Philosophy

This IP implements a **Stack-Agnostic Hybrid EtherCAT Master** (`ecat_cyclic_master_ip.v`) paired with a **Cycle-Accurate 40-Slave, 5 KB Ring Emulator** (`ecat_40slave_emulator.v`) for the Xilinx Zynq-7000 SoC (**PYNQ-Z2**).

### Why Hybrid & Stack-Agnostic?
In an industrial robotics controller, ~85% of an EtherCAT Master stack (**IgH EtherLab** or **SOEM**) consists of asynchronous startup and configuration logic:
* Discovering slaves via Auto-Increment (`APRD`/`APWR`)
* Reading Slave Information Interface (SII) EEPROMs
* Running **CoE (CANopen over EtherCAT)** SDO mailbox exchanges (`Pre-Op`)
* Programming **SyncManagers** and **FMMUs (Fieldbus Memory Management Units)** (`Safe-Op`)

Once FMMUs are configured, the entire network's inputs and outputs are mapped into a flat, contiguous **Logical Address Space** (`0x0000_0000 .. 0x0000_13FF`). At the moment the Master transitions to **Operational (`OP`)**, the cyclic real-time robotics loop only needs to:
1. Exchange a contiguous **Process Data Image (PDI)** (e.g., **5 KB = 5,120 bytes** across **40 slaves**), fragmented into $N$ standard IEEE 802.3 Ethernet frames (`EtherType = 0x88A4`).
2. Verify that every returning frame has a valid **Ethernet FCS (`CRC-32`)** and that each datagram's **Working Counter (`WKC`)** matches the expected count of participating slaves (`EXPECTED_WKC`).

By exposing a hardware **Datagram Descriptor Table (DDT)** over AXI4-Lite and streaming the Process Data Image over **Zero-Copy AXI DMA (`MM2S` / `S2MM`)**, this FPGA IP is **100% independent of whether the PS runs IgH EtherLab, SOEM, or a standalone C++ robotics loop**.

---

## 2. Wire Timing & Fragmentation Analysis (5 KB PDO, 40 Slaves @ 100 Mbps)

Standard EtherCAT operates over **100BASE-TX (100 Mbps full-duplex)** where $1\text{ byte} = 80\text{ ns}$ on the wire:
* **Maximum Ethernet Payload (MTU):** $1,500\text{ bytes}$
* **EtherCAT Overhead inside Ethernet Payload:**
  * EtherCAT Frame Header: $2\text{ bytes}$
  * EtherCAT Datagram Header (`CMD`, `IDX`, `LOGICAL_ADDR`, `LEN/FLAGS`, `IRQ`): $10\text{ bytes}$
  * Datagram Working Counter (`WKC`): $2\text{ bytes}$
  * **Max PDO Bytes per Single-Datagram Frame:** $1,500 - 14 = \mathbf{1,486\text{ bytes}}$ ($371\text{ words}$)
* **Slave-Aligned Partitioning for 40 Slaves $\times$ 128 Bytes/Slave ($5,120\text{ Bytes}$):**
  To align cleanly on 32-bit word and 128-byte slave boundaries (11 slaves per full frame = $1,408\text{ bytes}$):

| Frame / Descriptor | Slaves Covered | Logical Address Start | Payload Size (Bytes / Words) | Expected `WKC` (`LRW` = +3/slave) | Wire Time @ 100 Mbps |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Descriptor 0** | Slaves `0 .. 10` (11 slaves) | `0x0000_0000` (0) | `1,408 B` (`352 words`) | `33` ($11 \times 3$) | $116.80\ \mu\text{s}$ |
| **Descriptor 1** | Slaves `11 .. 21` (11 slaves) | `0x0000_0580` (1408) | `1,408 B` (`352 words`) | `33` ($11 \times 3$) | $116.80\ \mu\text{s}$ |
| **Descriptor 2** | Slaves `22 .. 32` (11 slaves) | `0x0000_0B00` (2816) | `1,408 B` (`352 words`) | `33` ($11 \times 3$) | $116.80\ \mu\text{s}$ |
| **Descriptor 3** | Slaves `33 .. 39` (7 slaves) | `0x0000_1080` (4224) | `896 B` (`224 words`) | `21` ($7 \times 3$) | $75.84\ \mu\text{s}$ |
| **Total per Cycle** | **40 Slaves** | `0x0000_0000 .. 0x0000_13FF` | **5,120 B (1,280 words)** | **Total WKC = 120** | **~426.24 $\mu$s** (+ ~20–40 $\mu$s ESC forwarding) |

*(Note: The hardware Descriptor Table supports any arbitrary partitioning up to 8 descriptors and up to 8 KB total PDO—including unaligned 1,484-byte MTU splits or slave-aligned 1,408-byte splits.)*

---

## 3. AXI4-Lite Control, Status & Descriptor Register Map

Base Address: `0x4000_0000` (4 KB aperture).

### 3.1 Global Control & Status Registers (`0x00 .. 0x3C`)

| Byte Offset | Word Index | Name | R/W | Description |
| :--- | :--- | :--- | :--- | :--- |
| `0x00` | `0` | `TRIGGER_ADDR` | W | Write anything: Force FSM abort/reset back to `S_IDLE`. |
| `0x04` | `1` | `STATUS_ADDR` | R/W | **Read:**<br>• `Bit [0]`: `ip_done_irq` (Level-High Interrupt state)<br>• `Bits [4:1]`: `state` (Master FSM state)<br>• `Bit [5]`: `crc_err` (1 = DMA Ingress CRC or Ethernet FCS error)<br>• `Bit [6]`: `wkc_err` (1 = Any frame returned `actual_wkc != expected_wkc`)<br>• `Bits [10:7]`: `completed_frames` (Number of frames processed in last cycle)<br>• `Bits [15:12]`: `num_descriptors` (Active descriptor count `1..8`)<br>• `Bits [31:16]`: `total_actual_wkc` (Sum of `WKC` across all frames in cycle, e.g., `120`)<br>**Write:**<br>• In `S_IDLE`: `Bit [0]=1` triggers manual cyclic exchange<br>• In `S_DONE_IRQ`: Acknowledges & lowers Level-High `ip_done_irq`<br>• In `S_WAIT_EGRESS`: Kicks `M_AXIS` S2MM DMA egress stream |
| `0x08` | `2` | `COMPUTE_CTRL_ADDR` | R/W | **Master Control Register:**<br>• `Bits [3:0]`: `num_descriptors` (`1..8`, default `4`)<br>• `Bit [4]`: `loopback_emu_en` (`1` = route through internal 40-slave emulator, `0` = external PHY stream)<br>• `Bits [15:8]`: `cycle_seq_id` (Incremented per cycle or set by PS) |
| `0x0C` | `3` | `EXPECTED_CRC_ADDR` | R/W | **Write:** Expected 32-bit CRC of DMA MM2S `TxPDO` stream (`0` = bypass check).<br>**Read:** Calculated 32-bit ingress CRC accumulator. |
| `0x10` | `4` | `PACING_TIMER_ADDR`<br>*(alias: `TIMER_ADDR`)* | R/W | Wire pacing delay cycles per beat (`0` = line-rate simulation speedup, `>0` = throttled wire timing to match 100BASE-TX wire speed). |
| `0x14` | `5` | `CFG_PAYLOAD_ADDR` | R/W | Total Process Data Image (PDI) size in 32-bit words (default `1280` words = `5,120` bytes; max `2048` words = `8,192` bytes). |
| `0x18` | `6` | `SLAVE_MASK_LO_ADDR` | R/W | Emulator Slaves `0..31` Online Bitmask (default `0xFFFF_FFFF`). Clearing bit $k$ simulates Slave $k$ dropping offline (`WKC` fault injection). |
| `0x1C` | `7` | `SLAVE_MASK_HI_ADDR` | R/W | Emulator Slaves `32..39` Online Bitmask (`Bits [7:0]`, default `0x0000_00FF`). |
| `0x20` | `8` | `LAST_CYCLE_TICKS` | R | Hardware clock cycles elapsed during the last multi-frame EtherCAT wire exchange (`S_ECAT_CYCLE`). |

### 3.2 Datagram Descriptor Table (`0x40 .. 0xBF`, 8 Descriptors $\times$ 16 Bytes)

For descriptor $i \in \{0, 1, \dots, 7\}$, base offset is `0x40 + (i * 0x10)`:

| Offset from Desc Base | Word Offset | Register Name | Bit Fields |
| :--- | :--- | :--- | :--- |
| `+0x00` | `16 + 4*i + 0` | `DESC_LOGICAL_ADDR[i]` | `[31:0]`: 32-bit EtherCAT Logical Address (`FMMU` start byte address, e.g., `0`, `1408`, `2816`, `4224`). |
| `+0x04` | `16 + 4*i + 1` | `DESC_CTRL[i]` | • `[11:0]`: `payload_words` (Payload length in 32-bit words, e.g., `352` words = `1,408` bytes)<br>• `[23:16]`: `ecat_cmd` (`0x0C` = `LRW`, `0x0A` = `LRD`, `0x0B` = `LWR`)<br>• `[31:24]`: `datagram_idx` (Frame sequence index) |
| `+0x08` | `16 + 4*i + 2` | `DESC_PDI_WKC[i]` | • `[15:0]`: `pdi_word_offset` (Starting 32-bit word offset in `tx_pdo_bram` / `rx_pdo_bram`)<br>• `[31:16]`: `expected_wkc` (Expected Working Counter after traversing slaves, e.g., `33` or `21`) |
| `+0x0C` | `16 + 4*i + 3` | `DESC_STATUS[i]` | **Read-Only Diagnostic (Updated by RX Parser):**<br>• `[15:0]`: `actual_wkc` (Received Working Counter)<br>• `Bit [16]`: `frame_wkc_err` (`actual_wkc != expected_wkc`)<br>• `Bit [17]`: `frame_fcs_err` (Ethernet FCS CRC-32 mismatch)<br>• `Bit [18]`: `frame_done` (`1` = frame received and parsed) |

---

## 4. On-Wire Frame Format (32-Bit Streaming Between Master & 40-Slave Ring)

Each descriptor $i$ generates a complete EtherCAT frame over the internal/external AXI4-Stream Ethernet bus (`m_axis_eth` $\rightarrow$ `ecat_40slave_emulator` $\rightarrow$ `s_axis_eth`):

1. **Beat 0 (`ETH_HDR_0`):** `{DstMAC[31:0]}` = `32'hFFFF_FFFF` (Broadcast / EtherCAT MAC)
2. **Beat 1 (`ETH_HDR_1`):** `{DstMAC[47:32], SrcMAC[47:32]}` = `32'hFFFF_0200`
3. **Beat 2 (`ETH_HDR_2`):** `{SrcMAC[31:0]}` = `32'h5E10_0001`
4. **Beat 3 (`ECAT_HDR`):** `{EtherType (16'h88A4), ECAT_Header (2B: type=4'h1, len_bytes)}`
5. **Beat 4 (`DGRM_HDR_0`):** `{ecat_cmd[7:0], datagram_idx[7:0], payload_bytes[15:0]}`
6. **Beat 5 (`DGRM_HDR_1`):** `{logical_addr[31:0]}`
7. **Beats `6 .. 6 + payload_words - 1` (`PDO_DATA`):**
   * Master transmits `tx_pdo_bram[pdi_word_offset + k]`.
   * **40-Slave Ring Emulator** computes the current byte address `cur_byte_addr = logical_addr + (k * 4)`, determines `slave_id = cur_byte_addr[12:7]` (`0..39`), checks `slave_online_mask[slave_id]`, stores the Master's `TxPDO` word into Slave RAM, and replaces the wire word on the fly with:
     $$\text{RxPDO}[k] = \sim\text{TxPDO}[k] + \{16'\text{h0000},\ \text{slave\_id}[5:0],\ 10'\text{h100}\}$$
   * For every distinct online slave (`slave_id`) touched by the datagram, the emulator increments the frame's `WKC` by $+3$ (for `LRW`), $+1$ (for `LRD`), or $+2$ (for `LWR`).
8. **Beat `6 + payload_words` (`WKC_TRAILER`):** `{16'h0000, WKC[15:0]}` (Master sends `0x0000`; Emulator outputs updated `WKC`).
9. **Beat `7 + payload_words` (`FCS_TRAILER` + `tlast=1`):** 32-bit Ethernet Frame Check Sequence (`CRC-32`) computed over the entire frame (including the updated `RxPDO` and `WKC`).

