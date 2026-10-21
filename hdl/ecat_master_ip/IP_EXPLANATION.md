# Hybrid EtherCAT Cyclic Master & 40-Slave Ring Emulator: Detailed Hardware Flowcharts

This document provides a detailed, state-by-state and beat-by-beat architectural walkthrough of [`ecat_cyclic_master_ip.v`](/ecat_cyclic_master_ip.v) and [`ecat_40slave_emulator.v`](/ecat_40slave_emulator.v).

---

## 1. High-Level System & Datapath Architecture

```mermaid
flowchart TD
    subgraph PS["Zynq-7000 Processing System (PS - ARM Cortex-A9)"]
        STACK["EtherCAT Stack (SOEM / IgH) + Robotics Loop"]
        DDR["Coherent DDR3 Buffers (Zero-Copy mmap TxPDO / RxPDO)"]
    end

    subgraph DMA["Xilinx AXI DMA Engine"]
        MM2S["MM2S Channel (DDR TxPDO -> PL S_AXIS)"]
        S2MM["S2MM Channel (PL M_AXIS -> DDR RxPDO)"]
    end

    subgraph MASTER["ecat_cyclic_master_ip.v (PL Master Accelerator)"]
        direction TB
        CSR["AXI4-Lite Slave (0x00..0x20 CSRs)"]
        DDT["Datagram Descriptor Table (0x40..0xBF, 8 Slots)"]
        ING_CRC["DMA Ingress CRC-32 Checker"]

        subgraph BRAM["Dual-Port PDI Block RAM (2048 x 32-bit = 8 KB)"]
            PortA["Port A (Write: MM2S Ingress OR EtherCAT RX Parser)"]
            PortB["Port B (Read: EtherCAT TX Prefetch OR S2MM Egress)"]
        end

        TX_SEQ["Multi-Frame EtherCAT TX Builder (eth_tx_axis)"]
        RX_PAR["Multi-Frame EtherCAT RX Parser & WKC/FCS Verifier"]
        SKID["Pipelined S2MM Egress + Skid Buffer"]
    end

    subgraph RING["ecat_40slave_emulator.v (In-PL 40-Slave, 5 KB Ring)"]
        direction TB
        SLV_PARSE["On-the-Fly Datagram & Logical Addr Decoder"]
        SLV_RAM["Slave Process Data RAM (40 Slaves x 128B = 5,120B)"]
        SLV_WKC["Per-Slave Online Mask & WKC Accumulator (+3/Slave)"]
        SLV_FCS["On-the-Fly Ethernet FCS (CRC-32) Regenerator"]
        SLV_PARSE --> SLV_RAM --> SLV_WKC --> SLV_FCS
    end

    STACK -->|"1. Safe-Op -> OP: Program 4 Descriptors (ioctl)"| DDT
    STACK -->|"2. Control / IRQ ACK"| CSR
    DDR -->|"3. Stream 5 KB TxPDO"| MM2S
    MM2S -->|"s_axis (1,280 words)"| PortA
    MM2S -.->|"Every Beat"| ING_CRC

    DDT -->|"Load Desc[0..3]"| TX_SEQ
    PortB -->|"Prefetch TxPDO Slice"| TX_SEQ
    TX_SEQ -->|"eth_tx_tdata (4 Ethernet Frames)"| SLV_PARSE
    SLV_FCS -->|"eth_rx_tdata (Modified RxPDO + WKC + FCS)"| RX_PAR
    RX_PAR -->|"Write Extracted RxPDO"| PortA
    RX_PAR -->|"Record Actual WKC & Errors"| DDT

    PortB -->|"Stream Updated 5 KB RxPDO"| SKID
    SKID -->|"m_axis (Backpressure Safe)"| S2MM
    S2MM -->|"Zero-Copy RxPDO"| DDR
    MASTER -->|"Level-High ip_done_irq (IRQ_F2P)"| STACK
```

---

## 2. Top-Level Master FSM Lifecycle (`ecat_cyclic_master_ip.v`)

The Master FSM (`state[3:0]`) coordinates three distinct phases in every robotics control cycle:
1. **DMA Ingress (`S_IDLE` $\rightarrow$ `S_CHECK_CRC`)**: Pulls the 5 KB `TxPDO` image from PS DDR into `bram` Port A.
2. **Multi-Frame EtherCAT Ring Exchange (`S_ECAT_FRAME_INIT` $\leftrightarrow$ `S_ECAT_XFER`)**: Iterates through `cur_desc_idx = 0 .. cfg_num_descs - 1` (4 frames for 5 KB), streaming each frame out through the 40-slave ring and parsing the returning `RxPDO`, `WKC`, and `FCS` back into `bram` Port A.
3. **Interrupt & DMA Egress (`S_DONE_IRQ` $\rightarrow$ `S_WAIT_EGRESS` $\rightarrow$ `S_TX_RUN`)**: Wakes the PS via Level-High IRQ and streams the updated 5 KB `RxPDO` image back to PS DDR via `m_axis`.

```mermaid
stateDiagram-v2
    [*] --> S_IDLE : Reset (rst_n = 0)

    state "S_IDLE (4'd0)" as S_IDLE
    state "S_CHECK_CRC (4'd1)" as S_CHECK_CRC
    state "S_ECAT_FRAME_INIT (4'd2)" as S_ECAT_FRAME_INIT
    state "S_ECAT_XFER (4'd3)" as S_ECAT_XFER
    state "S_DONE_IRQ (4'd5)" as S_DONE_IRQ
    state "S_WAIT_EGRESS (4'd6)" as S_WAIT_EGRESS
    state "S_TX_RUN (4'd7)" as S_TX_RUN

    S_IDLE --> S_IDLE : AXI-Lite CSR / Descriptor Writes\n(Never triggers transition!)
    S_IDLE --> S_CHECK_CRC : s_axis_tlast handshake\nOR manual CSR kick (STATUS[0]=1)

    S_CHECK_CRC --> S_DONE_IRQ : CRC Mismatch\n(crc_err <= 1, ip_done_irq <= 1)
    S_CHECK_CRC --> S_ECAT_FRAME_INIT : CRC Pass (or Bypass)\ncur_desc_idx <= 0

    S_ECAT_FRAME_INIT --> S_ECAT_XFER : Latch Desc[cur_desc_idx]\nPrime Port B (ecat_tx_rd_addr <= cur_pdi_offset)

    S_ECAT_XFER --> S_ECAT_FRAME_INIT : eth_rx_tlast && (cur_desc_idx + 1 < cfg_num_descs)\ncur_desc_idx <= cur_desc_idx + 1
    S_ECAT_XFER --> S_DONE_IRQ : eth_rx_tlast && (Last Frame Done)\nip_done_irq <= 1 (Assert Level-High)

    S_DONE_IRQ --> S_IDLE : CPU ACK if crc_err == 1\nOR Force Abort (TRIGGER_ADDR)
    S_DONE_IRQ --> S_WAIT_EGRESS : CPU Writes STATUS_ADDR (0x04)\nip_done_irq <= 0

    S_WAIT_EGRESS --> S_TX_RUN : Driver read() Writes STATUS_ADDR (0x04)\negress_rd_ptr <= 0
    S_TX_RUN --> S_IDLE : m_axis_tvalid && m_axis_tready && m_axis_tlast
```

---

## 3. Detailed Multi-Frame EtherCAT TX & RX Engine (`S_ECAT_FRAME_INIT` & `S_ECAT_XFER`)

Inside `S_ECAT_XFER`, the Master operates **full-duplex**: the Frame Transmitter (`tx_beat_idx`) builds and injects an IEEE 802.3 + EtherCAT (`0x88A4`) frame into the slave ring while the Frame Receiver (`rx_beat_idx`) simultaneously strips returning `RxPDO` words and validates the Working Counter (`WKC`) and Ethernet FCS (`CRC-32`).

### Why `ecat_tx_rd_addr` Advances at `tx_beat_idx >= 5` (1-Cycle BRAM Prefetch)
* Synchronous Block RAM (`portb_dout <= bram[portb_addr]`) has a **1-clock-cycle read latency**.
* In `S_ECAT_FRAME_INIT`, `ecat_tx_rd_addr` is initialized to `cur_pdi_offset` (Word 0 of the frame's PDO slice).
* Therefore, during header beats `tx_beat_idx = 0..5`, `portb_dout` is already holding `bram[cur_pdi_offset + 0]`.
* On `tx_beat_idx == 5` (the beat right before `tx_beat_idx == 6` where PDO Word 0 is transmitted), `ecat_tx_rd_addr` increments to `cur_pdi_offset + 1` so that `portb_dout` will hold PDO Word 1 on `tx_beat_idx == 6`, staying 1 beat ahead across the entire payload!

```mermaid
flowchart TD
    subgraph Init["S_ECAT_FRAME_INIT (1 Clock Cycle per Descriptor)"]
        LoadDesc["Latch Descriptor[cur_desc_idx]:<br>• cur_logical_addr <= desc_logical_addr[i]<br>• cur_payload_words <= desc_ctrl[i][11:0]<br>• cur_ecat_cmd <= desc_ctrl[i][23:16]<br>• cur_pdi_offset <= desc_pdi_wkc[i][11:0]<br>• cur_expected_wkc <= desc_pdi_wkc[i][31:16]<br>• ecat_tx_rd_addr <= desc_pdi_wkc[i][10:0] (Prefetch Word 0)"]
    end

    subgraph Xfer["S_ECAT_XFER (Full-Duplex TX Builder & RX Parser)"]
        direction LR

        subgraph TXPath["A. Frame Transmitter (tx_beat_idx)"]
            TXBeat{"Evaluate tx_beat_idx"}
            TXHdr["Beats 0..5 (Headers):<br>• 0: 0xFFFF_FFFF (Dst MAC)<br>• 1: 0xFFFF_0200 (Dst/Src MAC)<br>• 2: 0x5E10_0001 (Src MAC)<br>• 3: {0x88A4, ECAT_Hdr}<br>• 4: {cmd, idx, payload_bytes}<br>• 5: cur_logical_addr<br>(At Beat 5: ecat_tx_rd_addr++)"]
            TXPayload["Beats 6 .. 6+N-1 (TxPDO Payload):<br>• eth_tx_tdata <= portb_dout<br>• ecat_tx_rd_addr <= ecat_tx_rd_addr + 1<br>• Update tx_fcs_accum"]
            TXWkc["Beat 6+N (WKC Trailer):<br>• eth_tx_tdata <= 32'h0000_0000<br>• Update tx_fcs_accum"]
            TXFcs["Beat 7+N (Ethernet FCS + TLAST):<br>• eth_tx_tdata <= tx_fcs_accum<br>• eth_tx_tlast <= 1'b1<br>• tx_frame_done <= 1'b1"]

            TXBeat -->|"0..5"| TXHdr
            TXBeat -->|"6 .. 6+N-1"| TXPayload
            TXBeat -->|"6+N"| TXWkc
            TXBeat -->|"7+N"| TXFcs
        end

        subgraph RXPath["B. Frame Receiver (rx_beat_idx)"]
            RXBeat{"Evaluate rx_beat_idx<br>(when eth_rx_tvalid)"}
            RXHdr["Beats 0..5 (Headers):<br>• Accumulate rx_fcs_accum"]
            RXPayload["Beats 6 .. 6+N-1 (RxPDO Extraction):<br>• Write Port A: bram[cur_pdi_offset + (rx_beat_idx - 6)] <= eth_rx_tdata<br>• Accumulate rx_fcs_accum"]
            RXWkc["Beat 6+N (WKC Check):<br>• rx_latched_wkc <= eth_rx_tdata[15:0]<br>• rx_frame_wkc_err <= (eth_rx_tdata[15:0] != cur_expected_wkc)<br>• Accumulate rx_fcs_accum"]
            RXFcs["Beat 7+N (eth_rx_tlast == 1):<br>• Verify eth_rx_tdata == rx_fcs_accum<br>• Save desc_status[cur_desc_idx]<br>• total_actual_wkc += rx_latched_wkc<br>• completed_frames++"]

            RXBeat -->|"0..5"| RXHdr
            RXBeat -->|"6 .. 6+N-1"| RXPayload
            RXBeat -->|"6+N"| RXWkc
            RXBeat -->|"7+N (tlast)"| RXFcs
        end
    end

    NextFrame{"cur_desc_idx + 1 < cfg_num_descs?"}
    Init --> Xfer
    RXFcs --> NextFrame
    NextFrame -- "Yes (More Frames in Cycle)" --> IncDesc["cur_desc_idx <= cur_desc_idx + 1"]
    IncDesc --> Init
    NextFrame -- "No (All 4 Frames Complete)" --> DoneIRQ["last_cycle_ticks <= cycle_tick_cnt + 1<br>ip_done_irq <= 1'b1<br>state <= S_DONE_IRQ"]
```

---

## 4. 40-Slave, 5 KB Ring Emulator Flowchart (`ecat_40slave_emulator.v`)

The emulator processes the Master's AXI-Stream frame **on the fly (1 beat per clock cycle)** without store-and-forward buffering, mirroring how physical Beckhoff ESC ASICs process frames in a daisy-chain ring:

```mermaid
flowchart TD
    InBeat(["Incoming Beat on s_axis_tdata (beat_fire)"]) --> WhichBeat{"Check beat_idx"}

    WhichBeat -- "Beats 0..3 (MAC + 0x88A4)" --> PassHdr["wire_out_word = s_axis_tdata<br>Update fcs_accum"]
    WhichBeat -- "Beat 4 (Datagram Hdr 0)" --> LatchCmd["latched_cmd <= s_axis_tdata[31:24]<br>latched_payload_words <= s_axis_tdata[13:2]<br>wkc_accum <= 0, slave_visited <= 0<br>wire_out_word = s_axis_tdata"]
    WhichBeat -- "Beat 5 (Datagram Hdr 1)" --> LatchAddr["latched_logical_addr <= s_axis_tdata<br>wire_out_word = s_axis_tdata"]

    WhichBeat -- "Beats 6 .. 6+N-1 (PDO Words)" --> CalcAddr["Compute Byte & Slave Address:<br>• pdo_word_idx = beat_idx - 6<br>• cur_byte_addr = latched_logical_addr + (pdo_word_idx * 4)<br>• cur_slave_id = cur_byte_addr[12:7] (128B per slave -> 0..39)"]

    CalcAddr --> IsOnline{"cur_slave_id < 40 &&<br>slave_online_mask[cur_slave_id] == 1?"}

    IsOnline -- "No (Slave Offline / Fault Injected)" --> BypassSlave["• Do NOT write Slave RAM<br>• wire_out_word = s_axis_tdata (Unmodified)<br>• Do NOT increment wkc_accum"]

    IsOnline -- "Yes (Slave Online)" --> ProcessSlave["1. Write Master Outgoing RxPDO to Slave RAM:<br>   slave_pdo_ram[cur_byte_addr[12:2]] <= s_axis_tdata<br>2. Inject Slave TxPDO Response onto Wire:<br>   wire_out_word = (~s_axis_tdata) + {16'h0, cur_slave_id, 10'h100}"]

    ProcessSlave --> FirstHit{"!slave_visited[cur_slave_id]?"}
    FirstHit -- "Yes (First word of this slave in frame)" --> IncWKC["wkc_accum <= wkc_accum + wkc_step (+3 for LRW)<br>slave_visited[cur_slave_id] <= 1'b1"]
    FirstHit -- "No (Already counted this slave)" --> KeepWKC["Keep wkc_accum unchanged"]

    WhichBeat -- "Beat 6+N (WKC Beat)" --> OutWKC["wire_out_word = {16'h0000, s_axis_tdata[15:0] + wkc_accum}<br>Update fcs_accum"]
    WhichBeat -- "Beat 7+N (s_axis_tlast == 1)" --> OutFCS["wire_out_word = fcs_accum (Fresh Ethernet CRC-32)<br>Reset beat_idx <= 0 for next frame"]

    PassHdr --> RegOut["Register m_axis_tdata <= wire_out_word"]
    LatchCmd --> RegOut
    LatchAddr --> RegOut
    BypassSlave --> RegOut
    IncWKC --> RegOut
    KeepWKC --> RegOut
    OutWKC --> RegOut
    OutFCS --> RegOut
```

---

## 5. End-to-End Sequence Diagram (PS Driver $\leftrightarrow$ PL Master $\leftrightarrow$ 40-Slave Ring)

```mermaid
sequenceDiagram
    autonumber
    participant App as PS Robotics App (test_dma / SOEM)
    participant Drv as Linux Driver (axi_dma.ko)
    participant Master as PL Master IP (ecat_cyclic_master_ip)
    participant Ring as PL 40-Slave Ring (ecat_40slave_emulator)

    Note over App,Master: Phase 1: Safe-Op -> OP State Handoff (Once at Startup)
    App->>Drv: ioctl(AXI_DMA_IOC_CONFIG_ECAT, 40 slaves, 5 KB)
    Drv->>Master: AXI-Lite Write CTRL (4 Descs), PAYLOAD_LEN (1280W), DESC[0..3] (0x40..0x78)
    Note right of Master: Remains in S_IDLE (Rule 1a)

    Note over App,Ring: Phase 2: Cyclic Real-Time Control Loop (Every Cycle)
    App->>Drv: write(fd, tx_buf, 5120) [Zero-Copy mmap]
    Drv->>Master: Write EXPECTED_CRC (0x0C) + Start MM2S DMA (1,280 words)
    Master->>Master: S_IDLE: Store 1,280 words in BRAM Port A & accumulate CRC
    Master->>Master: S_CHECK_CRC: Verify Ingress CRC-32

    loop For cur_desc_idx = 0 to 3 (4 Ethernet Frames)
        Master->>Master: S_ECAT_FRAME_INIT: Load Desc[i], prime Port B
        Master->>Ring: S_ECAT_XFER: Stream Frame i (Slaves slice, WKC=0, FCS)
        Ring->>Ring: Swap RxPDO/TxPDO in 40-Slave RAM, add +3 WKC/slave, regen FCS
        Ring->>Master: Return Frame i (Updated TxPDO, WKC=33 or 21, FCS)
        Master->>Master: Write RxPDO to BRAM Port A, verify FCS & WKC == Expected
    end

    Master->>Drv: Assert Level-High ip_done_irq (S_DONE_IRQ, Total WKC = 120)
    Drv->>Master: ISR Reads STATUS (0x04) & Writes 1 to STATUS (0x04) to ACK IRQ
    Master->>Master: Lower ip_done_irq <= 0, move to S_WAIT_EGRESS

    App->>Drv: read(fd, rx_buf, 5120) [Zero-Copy mmap]
    Drv->>Master: Prime S2MM DMA & Write 1 to STATUS (0x04)
    Master->>Drv: S_TX_RUN: Stream 1,280 RxPDO words via Skid Buffer (M_AXIS)
    Drv-->>App: Wake read() -> Robotics loop reads updated 40-slave RxPDO in-place!
```

