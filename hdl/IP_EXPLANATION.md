# Store-and-Forward BRAM Accelerator IP: Architecture & Specification

This document provides a comprehensive breakdown of the Custom Hardware Accelerator IP ([`bram_dma_ip.v`](file:///Volumes/SWStorage/Projects/fpga-projects/pynq-z2-ps-pl-loopback/hdl/bram_dma_ip.v)) designed for the PYNQ-Z2 (Zynq-7000 SoC). 

The IP implements a **Store-Compute-Forward** architecture featuring:
* Memory-Mapped AXI4-Lite Ingress directly to internal Block RAM.
* Real-time, zero-overhead On-the-Fly Full 32-bit CRC-32 calculation.
* Hardware Mailbox / Doorbell Trigger detection.
* Decoupled True Dual-Port Block RAM (`RAMB36E1`).
* Dedicated Status Register (`0xFF8`) & Full 32-bit CRC Readback Register (`0xFFC`).
* Level-High Interrupt notification to the ARM GIC (`IRQ_F2P[2]`).
* AXI4-Stream Egress to the Xilinx AXI DMA S2MM engine.

---

## 1. System & Hardware Topology

```
+---------------------------------------------------------------------------------------+
|                               Processing System (PS)                                  |
|                                                                                       |
|   +-------------------+        +--------------------+        +--------------------+   |
|   |   ARM Cortex-A9   |        |   GIC Controller   |        |    DDR Controller  |   |
|   |     (Linux OS)    |        |       (GIC)        |        |    (Physical RAM)  |   |
|   +---------+---------+        +---------+----------+        +---------+----------+   |
|             |                            ^                             ^              |
+-------------|----------------------------|-----------------------------|--------------+
              | M_AXI_GP0                  | IRQ_F2P[2]                  | S_AXI_HP0
              | (Memory-Mapped Bus)        | (Compute Done / Error)      | (S2MM DMA Bus)
+-------------|----------------------------|-----------------------------|--------------+
|             v                            |                             v              |
|   +---------+-----------+                |                 +-----------+----------+   |
|   | AXI Interconnect /  |                |                 |    Xilinx AXI DMA    |   |
|   | Central DMA (CDMA)  |                |                 |    (S2MM Engine)     |   |
|   +---------+-----------+                |                 +-----------^----------+   |
|             |                            |                             | S_AXIS_S2MM  |
|             | S_AXI (0x000 - 0xFFC)      |                             |              |
|             v                            |                             |              |
|   +---------+----------------------------+-----------------------------+----------+   |
|   |                           Custom Accelerator IP                               |   |
|   |                                                                               |   |
|   |   +-----------------------+                         +---------------------+   |   |
|   |   |   AXI Slave Handler   |                         |  AXI-Stream Master  |   |   |
|   |   |  - Address & Data Sync|                         |  - Synchronous Read |   |   |
|   |   |  - Doorbell Detector  |                         |  - TLAST Framing    |   |   |
|   |   |  - Full 32-bit CRC-32 |                         |  - Auto IRQ Deassert|   |   |
|   |   +-----------+-----------+                         +----------^----------+   |   |
|   |               | Port A                                         | Port B       |   |
|   |   +-----------v------------------------------------------------+----------+   |   |
|   |   |                  True Dual-Port Block RAM (4 KB)                      |   |   |
|   |   |                      (Inferred RAMB36E1)                              |   |   |
|   |   +------------------------------------------------------------+----------+   |   |
|   |                                                                | Port B       |   |
|   |                                                     +----------v----------+   |   |
|   |                                                     |   Compute Engine    |   |   |
|   |                                                     | - Multi-Cycle Math  |   |   |
|   |                                                     | - In-Place Updates  |   |   |
|   |                                                     +---------------------+   |   |
|   +-------------------------------------------------------------------------------+   |
|                                                                                       |
|                               Programmable Logic (PL)                                 |
+---------------------------------------------------------------------------------------+
```

---

## 2. Memory Map & Register Specification

The IP exposes a **4 KB (1024 $\times$ 32-bit words)** memory-mapped address space:

| Byte Offset | Word Index | Access | Name | Function |
|---|---|---|---|---|
| `0x000` – `0xFF4` | `0` – `1021` | R/W | `BRAM_DATA[0..1021]` | Input payload buffer / In-place processed output (up to 4088 bytes). |
| `0xFF8` | `1022` | **Read-Only** | `IP_STATUS_REG` | **Hardware Status Register** (see bitfield below). |
| `0xFFC` | `1023` | **Write** | `DOORBELL_TRIGGER` | **Hardware Trigger & Expected CRC:**<br>Writing this word arms `trigger_detected`.<br>The value written is treated as `expected_crc[31:0]`.<br>Writing `0x0` bypasses CRC verification. |
| `0xFFC` | `1023` | **Read** | `CRC_RESULT_REG` | **Full 32-bit Accumulated CRC:**<br>Returns the complete `crc_accum[31:0]` calculated across ingress words. |

### `IP_STATUS_REG` Read Bitfield (Offset `0xFF8`)

```
 31                                                  6   5   4        1    0
+-----------------------------------------------------+-----+----------+-----+
|                      Reserved                       | CRC |   state  | IRQ |
|                       (26'b0)                       | ERR |   [3:0]  | PIN |
+-----------------------------------------------------+-----+----------+-----+
```

* **Bit [0] (`ip_done_irq`):** Current electrical state of the physical interrupt line to GIC.
* **Bits [4:1] (`state`):** Current FSM state:
  * `0x0`: `S_IDLE`
  * `0x1`: `S_CHECK_CRC`
  * `0x2`: `S_COMP_READ`
  * `0x3`: `S_COMP_WAIT`
  * `0x4`: `S_COMP_WRITE`
  * `0x5`: `S_DONE_IRQ`
  * `0x6`: `S_WAIT_EGRESS`
  * `0x7`: `S_TX_FETCH`
  * `0x8`: `S_TX_WAIT_RAM`
  * `0x9`: `S_TX_SEND`
* **Bit [5] (`crc_err`):** `1` = Ingress CRC mismatch detected; compute was aborted.
* **Bits [31:6]:** Reserved (zeros).

---

## 3. Finite State Machine (FSM) Flowchart

```mermaid
flowchart TD
    IDLE["S_IDLE (0)<br>• m_axis_tvalid = 0<br>• ip_done_irq = 0<br>• Accumulate CRC on Port A writes [0..1021]"]
    
    CHECK_CRC{"S_CHECK_CRC (1)<br>Verify Full 32-bit Checksum:<br>crc_accum == expected_crc<br>OR expected_crc == 0x0?"}
    
    COMP_READ["S_COMP_READ (2)<br>• Set portb_addr = comp_ptr<br>• Load work_timer = 100"]
    
    COMP_WAIT{"S_COMP_WAIT (3)<br>work_timer == 0?"}
    
    COMP_WRITE["S_COMP_WRITE (4)<br>• In-place update: bram[comp_ptr] <= f(dout)<br>• portb_we = 1"]
    
    DONE_IRQ["S_DONE_IRQ (5)<br>• Assert ip_done_irq = 1 (Level-High)<br>• Hold until Driver writes ACK to 0x0FF8"]
    
    WAIT_EGRESS["S_WAIT_EGRESS (6)<br>• ip_done_irq = 0<br>• Wait for Driver read() to kick egress (write 0x0FF8)"]

    TX_FETCH["S_TX_FETCH (7)<br>• portb_addr = rd_ptr<br>• Request BRAM synchronous read"]
    
    TX_WAIT["S_TX_WAIT_RAM (8)<br>• Wait 1 cycle for BRAM latency<br>• portb_dout settles"]
    
    TX_SEND["S_TX_SEND (9)<br>• Drive m_axis_tdata = portb_dout<br>• Drive m_axis_tvalid = 1<br>• Set m_axis_tlast on final word (1021)"]

    IDLE -->|"Write to TRIGGER_ADDR (0xFFC)<br>(trigger_detected == 1)"| CHECK_CRC
    CHECK_CRC -->|"Pass / Bypass"| COMP_READ
    CHECK_CRC -->|"CRC Mismatch<br>(crc_err = 1)"| DONE_IRQ
    
    COMP_READ --> COMP_WAIT
    COMP_WAIT -->|"Decrement timer"| COMP_WAIT
    COMP_WAIT -->|"Timer expired"| COMP_WRITE
    
    COMP_WRITE -->|"comp_ptr < pkt_len - 1<br>(comp_ptr++)"| COMP_READ
    COMP_WRITE -->|"comp_ptr == pkt_len - 1<br>(ip_done_irq <= 1)"| DONE_IRQ
    
    DONE_IRQ -->|"Driver ISR Acks IRQ<br>(Write to 0x0FF8)"| WAIT_EGRESS
    DONE_IRQ -->|"CRC Error Reset<br>(Write to 0x0FF8/0xFFC)"| IDLE
    
    WAIT_EGRESS -->|"Driver read() kicks egress<br>(Write to 0x0FF8)"| TX_FETCH

    TX_FETCH --> TX_WAIT
    TX_WAIT --> TX_SEND
    
    TX_SEND -->|"Handshake done && rd_ptr < pkt_len - 1<br>(rd_ptr++)"| TX_FETCH
    TX_SEND -->|"Handshake done && rd_ptr == pkt_len - 1<br>(TLAST transfer complete)"| IDLE
```

---

## 4. End-to-End System Sequence

```mermaid
sequenceDiagram
    autonumber
    participant App as Userspace App
    participant Driver as Linux Kernel Driver
    participant CDMA as AXI Master / CDMA
    participant IP_PortA as Custom IP (Port A & Ingress)
    participant BRAM as Dual-Port BRAM
    participant IP_FSM as Custom IP (Compute FSM)
    participant GIC as ARM GIC-400
    participant S2MM as AXI DMA (S2MM Egress)

    Note over App,IP_PortA: Phase 1: Ingress & Full 32-bit CRC Calculation
    App->>Driver: write(fd, tx_buf, 4088)
    Driver->>CDMA: Initiate Memory Transfer (DDR -> BRAM 0x000..0xFF4)
    loop Words 0 to 1021
        CDMA->>IP_PortA: s_axi_awaddr, s_axi_wdata
        IP_PortA->>BRAM: Store word in Port A
        IP_PortA->>IP_PortA: Update crc_accum <= next_crc32(crc_accum, wdata)
        IP_PortA-->>CDMA: s_axi_bvalid = OKAY
    end

    Note over App,IP_FSM: Phase 2: Hardware Doorbell Trigger
    Driver->>IP_PortA: Write expected_crc[31:0] to TRIGGER_ADDR (0x0FFC)
    IP_PortA->>IP_FSM: Assert trigger_detected, latch expected_crc
    IP_FSM->>IP_FSM: State -> S_CHECK_CRC

    alt CRC Valid (crc_accum == expected_crc OR expected_crc == 0)
        IP_FSM->>IP_FSM: State -> S_COMP_READ
        Note over IP_FSM,BRAM: Phase 3: Hardware Compute Pipeline
        loop For each word in BRAM (0 .. 1021)
            IP_FSM->>BRAM: Read Port B address
            BRAM-->>IP_FSM: Return word (1 cycle latency)
            IP_FSM->>IP_FSM: Process math (100 cycles)
            IP_FSM->>BRAM: Write updated result back to Port B
        end
    else CRC Invalid
        IP_FSM->>IP_FSM: Set crc_err = 1, bypass compute
    end

    Note over IP_FSM,GIC: Phase 4: Interrupt Notification
    IP_FSM->>GIC: Assert Level-High ip_done_irq (IRQ_F2P[2])
    GIC->>Driver: Trigger Kernel ISR (SPI 31 / HW ID 63)
    Driver->>IP_PortA: Read 0x0FF8 (Inspect IP_STATUS_REG: crc_err, state)
    
    alt If CRC Error Detected
        Driver->>IP_PortA: Read 0x0FFC (Read full calculated 32-bit CRC)
        Driver->>App: Return -EIO / Checksum Fault
    else Compute Success
        Note over Driver,S2MM: Phase 5: S2MM Egress Streaming
        Driver->>S2MM: Prime S2MM Descriptor (CURDESC -> DMACR.RS=1 -> TAILDESC)
        S2MM->>IP_FSM: Assert m_axis_tready = 1
        IP_FSM->>GIC: Deassert ip_done_irq <= 0
        IP_FSM->>IP_FSM: State -> S_TX_FETCH
        loop Stream Words 0 to 1021
            IP_FSM->>BRAM: Fetch from Port B
            BRAM-->>IP_FSM: Latch dout
            IP_FSM->>S2MM: m_axis_tdata, tvalid = 1 (tlast on 1021)
            S2MM->>Driver: DMA transfers payload into DDR rx_buf
        end
        S2MM->>Driver: Fire S2MM Interrupt (Transfer Complete)
        Driver->>App: read() returns processed payload
    end
```

---

## 5. Linux Driver Integration Guide

### 1. Register Offsets
```c
#define IP_STATUS_OFFSET  0x0FF8  /* Read: {crc_err, state, ip_done_irq} */
#define IP_TRIGGER_OFFSET 0x0FFC  /* Write: Kick + CRC, Read: Full 32-bit CRC */
```

### 2. Issuing the Doorbell from Software
```c
/* 1. Copy data into BRAM window (up to 4088 bytes = 1022 words) */
memcpy_toio(dev->ip_regs, user_payload, 4088);

/* 2. Write expected 32-bit CRC (or 0 to bypass) to TRIGGER_ADDR */
iowrite32(calculated_crc32, dev->ip_regs + IP_TRIGGER_OFFSET);
```

### 3. Handling Completion & Diagnosing CRC in ISR
```c
static irqreturn_t custom_ip_isr(int irq, void *dev_id)
{
    struct axi_dma_dev *dev = (struct axi_dma_dev *)dev_id;
    u32 status = ioread32(dev->ip_regs + IP_STATUS_OFFSET);
    
    if (status & (1 << 5)) { /* Bit 5 = crc_err */
        u32 actual_crc = ioread32(dev->ip_regs + IP_TRIGGER_OFFSET);
        pr_err("custom_ip: CRC mismatch! Actual CRC=0x%08x, Status=0x%08x\n", 
               actual_crc, status);
        dev->crc_error = 1;
        /* Re-write 0 to TRIGGER_ADDR to clear error and return FSM to IDLE */
        iowrite32(0, dev->ip_regs + IP_TRIGGER_OFFSET);
    } else {
        dev->compute_done = 1;
        /* Trigger S2MM DMA transfer to read results back into DDR */
        start_s2mm_dma(dev);
    }
    
    wake_up_interruptible(&dev->comp_waitq);
    return IRQ_HANDLED;
}
```
