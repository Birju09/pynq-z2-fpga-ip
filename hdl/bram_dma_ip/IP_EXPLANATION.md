# Pipelined Store-and-Forward BRAM Accelerator IP: Architecture & Flow Specification

This document provides a detailed specification and architectural breakdown of the custom hardware accelerator IP ([`bram_dma_ip.v`](file:///Volumes/SWStorage/Projects/fpga-projects/pynq-z2-ps-pl-loopback/pynq-z2-fpga-ip/hdl/bram_dma_ip.v)) targeting the PYNQ-Z2 (Zynq-7000 SoC).

---

## 1. High-Level System Architecture

The accelerator is structured as a **Store-Compute-Forward** engine with full 100 MHz (400 MB/s line rate) pipelining on both compute and streaming egress:

```mermaid
flowchart TD
    subgraph PS["Processing System (PS)"]
        CPU["ARM Cortex-A9 (Linux Driver)"]
        DDR["Physical DDR3 Memory"]
    end

    subgraph DMA["Xilinx AXI DMA"]
        MM2S["MM2S Engine (Tx to FPGA)"]
        S2MM["S2MM Engine (Rx from FPGA)"]
    end

    subgraph IP["bram_dma_ip (FPGA PL)"]
        direction TB
        CSR["AXI4-Lite Slave (CSR & Controls)"]
        CRC["On-The-Fly CRC-32 Accumulator"]
        
        subgraph BRAM["Dual-Port Block RAM (Up to 64K x 32-bit = 256 KB)"]
            PortA["Port A (Write Port)"]
            PortB["Port B (Read Port)"]
        end

        ALU["Pipelined ALU Compute (~data + 0x1000)"]
        SKID["AXI4-Stream Skid Buffer Engine"]
    end

    DDR -->|Physical DMA Ingress| MM2S
    MM2S -->|S_AXIS Stream| PortA
    MM2S -.->|Every Beat| CRC
    CPU -->|AXI-Lite Writes: STATUS / PAYLOAD / TIMER| CSR

    PortB -->|Stage 1: Read| ALU
    ALU -->|Stage 3: Write Back| PortA

    PortB -->|1-Cycle Read Latency| SKID
    SKID -->|M_AXIS Stream with Backpressure Protection| S2MM
    S2MM -->|Physical DMA Egress| DDR
    IP -->|Level-High Interrupt IRQ_F2P| CPU
```

---

## 2. Ingress (RX: DMA MM2S $\rightarrow$ BRAM) Workflow

The ingress path receives 32-bit streaming data from the DMA MM2S channel directly into BRAM Port A.

```mermaid
flowchart TD
    Start(["S_IDLE: Awaiting Frame"]) --> CheckHandshake{"s_axis_tvalid &&<br>s_axis_tready (rx_handshake)?"}
    
    CheckHandshake -- No --> Start
    CheckHandshake -- Yes --> WriteBRAM["Write to Port A: bram[rx_ptr] <= s_axis_tdata<br>Update CRC: crc_accum <= next_crc32(crc, data)"]
    
    WriteBRAM --> CheckLast{"s_axis_tlast OR<br>(rx_ptr == cfg_payload_words - 1)?"}
    
    CheckLast -- No --> IncPtr["rx_ptr <= rx_ptr + 1'b1"]
    IncPtr --> Start
    
    CheckLast -- Yes --> FrameDone["pkt_len <= {1'b0, rx_ptr} + 1'b1<br>rx_ptr <= 0"]
    FrameDone --> CRCState["State <= S_CHECK_CRC"]
    
    CRCState --> ValidateCRC{"(crc_accum == expected_crc)<br>OR (expected_crc == 0)?"}
    
    ValidateCRC -- Pass --> StartCompute["crc_err <= 0<br>State <= S_COMP_RUN"]
    ValidateCRC -- Fail --> ErrorState["crc_err <= 1<br>ip_done_irq <= 1 (Assert Level-High)<br>State <= S_DONE_IRQ"]
```

---

## 3. Pipelined Compute Workflow (`S_COMP_RUN`)

Compute runs in-place: **Port B** acts as the dedicated read port, and **Port A** acts as the dedicated write port. Once the 2-stage pipeline fills, **one word is read, transformed, and written back on every single clock cycle (100 MHz)**.

```mermaid
flowchart TD
    subgraph ComputePipeline["3-Stage Pipelined Execution (Every Clock Cycle)"]
        direction TB
        S1["Stage 1 (Port B):<br>Apply portb_addr <= comp_rd_ptr<br>comp_rd_ptr <= comp_rd_ptr + 1"]
        S2["Stage 2 (ALU):<br>BRAM outputs portb_dout<br>comp_wr_data <= (~portb_dout) + 0x1000"]
        S3["Stage 3 (Port A Write):<br>Write back to bram[comp_wr_addr] <= comp_wr_data<br>comp_wr_en <= 1"]
        
        S1 --> S2 --> S3
    end

    CheckDone{"comp_wr_en &&<br>(comp_wr_addr == pkt_len - 1)?"}
    S3 --> CheckDone
    CheckDone -- No --> S1
    CheckDone -- Yes --> NotifyPS["comp_wr_en <= 0<br>ip_done_irq <= 1 (Assert Level-High GIC IRQ)<br>State <= S_DONE_IRQ"]
```

---

## 4. Egress (TX: BRAM $\rightarrow$ DMA S2MM) with Skid Buffering

The egress path streams processed words from BRAM Port B to `m_axis` at the maximum physical wire rate (1 beat per cycle = 400 MB/s). 

Because BRAM has a **1-clock-cycle read latency**, downstream backpressure (`m_axis_tready == 0`) requires a **Skid Buffer** to capture the in-flight read without dropping data.

```mermaid
flowchart TD
    subgraph EgressPipeline["Egress & Skid Buffer FSM (S_TX_RUN)"]
        direction TB
        
        CanRead{"can_issue_bram_rd?<br>(rd_ptr < pkt_len) && (!skid_valid) &&<br>(tready || !bram_pipe_valid || !tvalid)"}
        
        CanRead -- Yes --> IssueRead["egress_rd_ptr <= egress_rd_ptr + 1<br>bram_pipe_valid <= 1<br>bram_pipe_last <= (rd_ptr == pkt_len - 1)<br>portb_addr <= egress_rd_ptr"]
        CanRead -- No --> StallRead["bram_pipe_valid <= 0<br>bram_pipe_last <= 0<br>(Address Generator Freezes)"]
        
        IssueRead --> CheckSkid{"skid_valid == 1?"}
        StallRead --> CheckSkid
        
        CheckSkid -- "Yes (Buffer Stalled)" --> HandleSkid{"m_axis_tready == 1?"}
        HandleSkid -- Yes --> DrainSkid["m_axis_tdata <= skid_data<br>m_axis_tlast <= skid_last<br>m_axis_tvalid <= 1<br>skid_valid <= bram_pipe_valid<br>skid_data <= portb_dout"]
        HandleSkid -- No --> HoldSkid["Hold m_axis_tdata, skid_data, and tvalid stable"]
        
        CheckSkid -- "No (Normal Flow)" --> HandleDirect{"m_axis_tready || !m_axis_tvalid?"}
        HandleDirect -- Yes --> StreamOut["If bram_pipe_valid:<br>m_axis_tdata <= portb_dout<br>m_axis_tlast <= bram_pipe_last<br>m_axis_tvalid <= 1"]
        HandleDirect -- "No (Consumer Stalled!)" --> ParkInSkid["If bram_pipe_valid:<br>skid_data <= portb_dout<br>skid_last <= bram_pipe_last<br>skid_valid <= 1<br>(Emergency Parking Bay Engaged!)"]
    end

    Complete{"tx_handshake && m_axis_tlast?<br>(Final beat accepted by DMA)"}
    StreamOut --> Complete
    DrainSkid --> Complete
    Complete -- Yes --> ReturnIdle["m_axis_tvalid <= 0<br>m_axis_tlast <= 0<br>skid_valid <= 0<br>State <= S_IDLE"]
```

---

## 5. Skid Buffer State & Timing Analysis (Handling Backpressure)

### Why is `reg [ADDR_WIDTH:0] comp_rd_ptr;` sized to `ADDR_WIDTH + 1` bits?
- BRAM contains $2^{\text{ADDR\_WIDTH}}$ words (e.g., $65,536$ words for $\text{ADDR\_WIDTH}=16$).
- The standard address space `[ADDR_WIDTH-1:0]` can only represent values from $0$ to $65,535$.
- When streaming or computing all $65,536$ words, the fetch pointer increments from $0 \rightarrow 65,535 \rightarrow \mathbf{65,536}$.
- If `comp_rd_ptr` were only $16$ bits, $65,535 + 1$ would **overflow back to 0**, causing the loop condition `comp_rd_ptr < pkt_len` ($0 < 65,536$) to become true again and loop infinitely.
- The 17th bit (`[ADDR_WIDTH]`) allows the counter to reach $65,536$ (`17'h10000`) cleanly without rollover.

---

### Why is `bram_pipe_valid` checked when `skid_valid == 1`?

Look at the skid draining logic in `S_TX_RUN`:
```verilog
if (skid_valid) begin
    if (m_axis_tready) begin
        m_axis_tdata  <= skid_data;
        m_axis_tlast  <= skid_last;
        m_axis_tvalid <= 1'b1;
        if (bram_pipe_valid) begin
            skid_data  <= portb_dout;
            skid_last  <= bram_pipe_last;
            skid_valid <= 1'b1;
        end else begin
            skid_valid <= 1'b0;
        end
    end
end
```

**Scenario: Stalling and Draining without Bubbles**
1. At cycle $T$, downstream deasserts `m_axis_tready = 0`. Word $A$ is held on `m_axis_tdata`.
2. A read was already in flight from cycle $T-1$. That word (Word $B$) emerges into `skid_data` at cycle $T+1$ (`skid_valid <= 1`).
3. Now suppose that on the exact cycle downstream reasserts `m_axis_tready = 1`, another read was finishing:
   - Word $B$ (from `skid_data`) moves to `m_axis_tdata` to satisfy the active handshake.
   - **If `bram_pipe_valid == 1`**: The newly emerging word from BRAM (`portb_dout`, Word $C$) immediately takes the empty parking spot in `skid_data` (`skid_valid <= 1`).
   - **If `bram_pipe_valid == 0`**: No new read was in flight, so the parking spot is now empty (`skid_valid <= 0`).

Checking `bram_pipe_valid` ensures the skid buffer is **re-armed if a read was in flight, or cleared if the pipeline was idle**, guaranteeing zero dropped words and zero bubble cycles.
