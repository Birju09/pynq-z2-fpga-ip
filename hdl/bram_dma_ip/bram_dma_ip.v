`timescale 1ns / 1ps

// =============================================================================
// Module: bram_dma_ip (bram_dma_ip.v)
//
// Description:
//   Store-and-Forward BRAM Hardware Accelerator with AXI4-Stream DMA Ingress,
//   Dedicated AXI4-Lite Control & Status Register (CSR) Space, Full 32-bit
//   On-the-Fly CRC-32 Verification, and AXI4-Stream S2MM Egress.
//
// AXI4-Lite Control & Status Register Map (Byte Offsets):
//   - 0x00 (Word 0) : TRIGGER_ADDR      (Write-Only)
//                     Write: Forces FSM reset/abort back to S_IDLE when in
//                            S_DONE_IRQ or S_WAIT_EGRESS.
//   - 0x04 (Word 1) : STATUS_ADDR       (R/W)
//                     Read : Bit [0]    = ip_done_irq (Interrupt wire state)
//                            Bits [4:1] = state (Current FSM state)
//                            Bit [5]    = crc_err (1 = CRC mismatch)
//                            Bits [31:6]= Reserved (26'b0)
//                     Write: In S_IDLE       -> Bit [0]=1 triggers manual compute
//                            In S_DONE_IRQ   -> Acknowledges & clears ip_done_irq
//                            In S_WAIT_EGRESS-> Kicks M_AXIS S2MM egress stream
//   - 0x08 (Word 2) : COMPUTE_CTRL_ADDR (R/W)
//                     Read/Write: 32-bit compute configuration register (ctrl_reg)
//   - 0x0C (Word 3) : EXPECTED_CRC_ADDR (R/W)
//                     Write: Configures expected_crc[31:0] (0x0 bypasses check)
//                     Read : Returns full 32-bit calculated crc_accum[31:0]
//
// Architecture & Workflow:
//   1. AXI4-Lite Control Plane (S_AXI) & AXI4-Stream Data Plane (S_AXIS):
//      - CPU pre-programs expected_crc at EXPECTED_CRC_ADDR (0x0C) via S_AXI.
//      - DMA MM2S streams payload data directly into BRAM Port A words [0 .. 1023].
//      - As words arrive on S_AXIS, on-the-fly CRC-32 accumulator updates each beat.
//      - Packet completion is signaled by s_axis_tlast (or full BRAM_DEPTH), or
//        manually triggered by writing bit 0 to STATUS_ADDR (0x04).
//
//   2. CRC Verification (S_CHECK_CRC):
//      - If (crc_accum == expected_crc) or (expected_crc == 0):
//          CRC PASSED -> Moves to S_COMP_READ to begin processing.
//      - If (crc_accum != expected_crc):
//          CRC FAILED -> Sets crc_err = 1, aborts compute, and raises ip_done_irq
//          to alert PS of corrupted ingress buffer.
//
//   3. Dual-Port BRAM (Inferred RAMB36E1 Block RAM, 4 KB / 1024 words):
//      - Port A: Dedicated exclusively to S_AXIS MM2S streaming DMA ingress.
//      - Port B: Dedicated to internal compute engine and M_AXIS S2MM egress.
//
//   4. Compute Engine:
//      - Iterates across BRAM data in-place over Port B.
//      - Modifies words and writes back responses.
//
//   5. Interrupt & Egress (M_AXIS to S2MM DMA):
//      - Upon compute completion (or CRC error), asserts Level-High `ip_done_irq`.
//      - Driver ISR writes to STATUS_ADDR (0x04) to acknowledge and clear IRQ.
//      - Driver read() primes S2MM DMA and writes to STATUS_ADDR (0x04) to kick
//        M_AXIS egress streaming with TLAST asserted on the final word.
// =============================================================================

module bram_dma_ip #(
    parameter integer DATA_WIDTH     = 32,
    parameter integer BRAM_DEPTH     = 1024, // 1024 words = 4096 bytes (4 KB)
    parameter integer ADDR_WIDTH     = 10,   // 2^10 = 1024 words
    parameter integer CFG_DEPTH      = 256,  // 256 words (configuration space)
    parameter integer CFG_ADDR_WIDTH = 8     // 2^8 = 256 words (configuration space)
)(
    input  wire                      clk,
    input  wire                      rst_n,

    // -------------------------------------------------------------------------
    // 1. AXI4-Lite Memory-Mapped Slave (Control & Status Registers)
    // -------------------------------------------------------------------------
    input  wire [CFG_ADDR_WIDTH+1:0] s_axi_awaddr,
    input  wire                      s_axi_awvalid,
    output reg                       s_axi_awready,
    input  wire [DATA_WIDTH-1:0]     s_axi_wdata,
    input  wire [(DATA_WIDTH/8)-1:0] s_axi_wstrb,
    input  wire                      s_axi_wvalid,
    output reg                       s_axi_wready,
    output wire [1:0]                s_axi_bresp,
    output reg                       s_axi_bvalid,
    input  wire                      s_axi_bready,

    input  wire [CFG_ADDR_WIDTH+1:0] s_axi_araddr,
    input  wire                      s_axi_arvalid,
    output reg                       s_axi_arready,
    output reg  [DATA_WIDTH-1:0]     s_axi_rdata,
    output wire [1:0]                s_axi_rresp,
    output reg                       s_axi_rvalid,
    input  wire                      s_axi_rready,

    // -------------------------------------------------------------------------
    // 2. AXI4-Stream Slave (Connected to AXI DMA MM2S: DDR RAM -> BRAM)
    // -------------------------------------------------------------------------
    input  wire [DATA_WIDTH-1:0]     s_axis_tdata,
    input  wire [(DATA_WIDTH/8)-1:0] s_axis_tkeep,
    input  wire                      s_axis_tvalid,
    output wire                      s_axis_tready,
    input  wire                      s_axis_tlast,

    // -------------------------------------------------------------------------
    // 3. AXI4-Stream Master (Connected to AXI DMA S2MM: BRAM -> DDR RAM)
    // -------------------------------------------------------------------------
    output reg  [DATA_WIDTH-1:0]     m_axis_tdata,
    output wire [(DATA_WIDTH/8)-1:0] m_axis_tkeep,
    output reg                       m_axis_tvalid,
    input  wire                      m_axis_tready,
    output reg                       m_axis_tlast,

    // -------------------------------------------------------------------------
    // 4. Dedicated Level-High Interrupt Wire to PS GIC (IRQ_F2P[2])
    // -------------------------------------------------------------------------
    output reg                       ip_done_irq
);

    // =========================================================================
    // Address Map Constants (Word Indices: s_axi_a*addr[CFG_ADDR_WIDTH+1:2])
    // =========================================================================
    localparam [CFG_ADDR_WIDTH-1:0] TRIGGER_ADDR      = 8'h0; // Byte offset 0x00
    localparam [CFG_ADDR_WIDTH-1:0] STATUS_ADDR       = 8'h1; // Byte offset 0x04
    localparam [CFG_ADDR_WIDTH-1:0] COMPUTE_CTRL_ADDR = 8'h2; // Byte offset 0x08
    localparam [CFG_ADDR_WIDTH-1:0] EXPECTED_CRC_ADDR = 8'h3; // Byte offset 0x0C
    localparam [CFG_ADDR_WIDTH-1:0] TIMER_ADDR        = 8'h4; // Byte offset 0x10
    localparam [CFG_ADDR_WIDTH-1:0] CFG_PAYLOAD_ADDR  = 8'h5; // Byte offset 0x14

    reg [DATA_WIDTH-1:0] ctrl_reg;
    reg                  status_write_pulse;
    reg                  trigger_detected;
    reg [DATA_WIDTH-1:0] status_write_data;
    reg [DATA_WIDTH-1:0] cfg_worker_timer;
    reg [DATA_WIDTH-1:0] cfg_payload_words;

    // =========================================================================
    // FSM State Definitions
    // =========================================================================
    localparam [3:0]
        S_IDLE        = 4'd0, // Awaiting S_AXIS stream ingress or manual CPU start
        S_CHECK_CRC   = 4'd1, // Validate CRC accumulator against expected CRC
        S_COMP_RUN    = 4'd2, // Pipelined Compute: 1 beat per clock cycle
        S_DONE_IRQ    = 4'd5, // Assert interrupt to PS & await Driver IRQ ACK
        S_WAIT_EGRESS = 4'd6, // Await Driver read kick (write to STATUS_ADDR)
        S_TX_RUN      = 4'd7; // Pipelined Egress: 1 beat per clock cycle over M_AXIS

    reg [3:0] state;

    // =========================================================================
    // Dual-Port Inferred Block RAM
    // Port A: Servicing S_AXIS MM2S Ingress (in S_IDLE) and Compute Writes (in S_COMP_RUN)
    // Port B: Servicing Compute Reads (in S_COMP_RUN) and Egress M_AXIS Stream (in S_TX_RUN)
    // =========================================================================
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] bram [0:BRAM_DEPTH-1];

    // Port A: AXI-Stream Ingress signals & Compute Write signals
    reg  [ADDR_WIDTH-1:0] rx_ptr;
    assign s_axis_tready = (state == S_IDLE);
    wire                  rx_handshake = s_axis_tvalid && s_axis_tready;
    wire                  axis_wr      = rx_handshake && (state == S_IDLE);

    // Pipelined Compute Signals
    reg  [ADDR_WIDTH:0]   comp_rd_ptr;
    reg                   comp_stage1_valid;
    reg  [ADDR_WIDTH-1:0] comp_stage1_addr;
    reg                   comp_wr_en;
    reg  [ADDR_WIDTH-1:0] comp_wr_addr;
    reg  [DATA_WIDTH-1:0] comp_wr_data;

    // Port A MUX: Writes in S_IDLE from stream, writes in S_COMP_RUN from compute
    wire [ADDR_WIDTH-1:0] porta_addr = (state == S_IDLE) ? rx_ptr : comp_wr_addr;
    wire [DATA_WIDTH-1:0] porta_din  = (state == S_IDLE) ? s_axis_tdata : comp_wr_data;
    wire                  porta_we   = (state == S_IDLE) ? axis_wr : comp_wr_en;

    always @(posedge clk) begin
        if (porta_we) begin
            bram[porta_addr] <= porta_din;
        end
    end

    // Pipelined Egress Signals
    reg  [ADDR_WIDTH:0]   egress_rd_ptr;
    reg                   bram_pipe_valid;
    reg                   bram_pipe_last;
    reg  [DATA_WIDTH-1:0] skid_data;
    reg                   skid_last;
    reg                   skid_valid;

    // Port B: Address MUX & Read Data
    wire [ADDR_WIDTH-1:0] portb_addr = (state == S_COMP_RUN) ? comp_rd_ptr[ADDR_WIDTH-1:0] :
                                       (state == S_TX_RUN)   ? egress_rd_ptr[ADDR_WIDTH-1:0] :
                                       {ADDR_WIDTH{1'b0}};
    reg  [DATA_WIDTH-1:0] portb_dout;

    always @(posedge clk) begin
        portb_dout <= bram[portb_addr];
    end

    // =========================================================================
    // Standard IEEE 802.3 Ethernet FCS CRC-32 (2-LUT Parallel Matrix)
    // Polynomial: 0xEDB88320 (reflected / LSB-first)
    // Maximum logic depth: 2 LUT6 levels (sub-nanosecond propagation delay)
    // =========================================================================
    function [31:0] next_crc32;
        input [31:0] current_crc;
        input [31:0] data_in;
        reg   [31:0] d;
        begin
            d = current_crc ^ data_in;
            next_crc32[ 0] = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[7] ^ d[8] ^ d[16] ^ d[20] ^ d[22] ^ d[23] ^ d[26];
            next_crc32[ 1] = d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[7] ^ d[8] ^ d[9] ^ d[17] ^ d[21] ^ d[23] ^ d[24] ^ d[27];
            next_crc32[ 2] = d[0] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[10] ^ d[18] ^ d[22] ^ d[24] ^ d[25] ^ d[28];
            next_crc32[ 3] = d[1] ^ d[3] ^ d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[9] ^ d[10] ^ d[11] ^ d[19] ^ d[23] ^ d[25] ^ d[26] ^ d[29];
            next_crc32[ 4] = d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[8] ^ d[10] ^ d[11] ^ d[12] ^ d[20] ^ d[24] ^ d[26] ^ d[27] ^ d[30];
            next_crc32[ 5] = d[0] ^ d[3] ^ d[5] ^ d[6] ^ d[7] ^ d[8] ^ d[9] ^ d[11] ^ d[12] ^ d[13] ^ d[21] ^ d[25] ^ d[27] ^ d[28] ^ d[31];
            next_crc32[ 6] = d[0] ^ d[2] ^ d[3] ^ d[9] ^ d[10] ^ d[12] ^ d[13] ^ d[14] ^ d[16] ^ d[20] ^ d[23] ^ d[28] ^ d[29];
            next_crc32[ 7] = d[1] ^ d[3] ^ d[4] ^ d[10] ^ d[11] ^ d[13] ^ d[14] ^ d[15] ^ d[17] ^ d[21] ^ d[24] ^ d[29] ^ d[30];
            next_crc32[ 8] = d[0] ^ d[2] ^ d[4] ^ d[5] ^ d[11] ^ d[12] ^ d[14] ^ d[15] ^ d[16] ^ d[18] ^ d[22] ^ d[25] ^ d[30] ^ d[31];
            next_crc32[ 9] = d[0] ^ d[2] ^ d[4] ^ d[5] ^ d[7] ^ d[8] ^ d[12] ^ d[13] ^ d[15] ^ d[17] ^ d[19] ^ d[20] ^ d[22] ^ d[31];
            next_crc32[10] = d[0] ^ d[2] ^ d[4] ^ d[5] ^ d[7] ^ d[9] ^ d[13] ^ d[14] ^ d[18] ^ d[21] ^ d[22] ^ d[26];
            next_crc32[11] = d[1] ^ d[3] ^ d[5] ^ d[6] ^ d[8] ^ d[10] ^ d[14] ^ d[15] ^ d[19] ^ d[22] ^ d[23] ^ d[27];
            next_crc32[12] = d[2] ^ d[4] ^ d[6] ^ d[7] ^ d[9] ^ d[11] ^ d[15] ^ d[16] ^ d[20] ^ d[23] ^ d[24] ^ d[28];
            next_crc32[13] = d[0] ^ d[3] ^ d[5] ^ d[7] ^ d[8] ^ d[10] ^ d[12] ^ d[16] ^ d[17] ^ d[21] ^ d[24] ^ d[25] ^ d[29];
            next_crc32[14] = d[0] ^ d[1] ^ d[4] ^ d[6] ^ d[8] ^ d[9] ^ d[11] ^ d[13] ^ d[17] ^ d[18] ^ d[22] ^ d[25] ^ d[26] ^ d[30];
            next_crc32[15] = d[1] ^ d[2] ^ d[5] ^ d[7] ^ d[9] ^ d[10] ^ d[12] ^ d[14] ^ d[18] ^ d[19] ^ d[23] ^ d[26] ^ d[27] ^ d[31];
            next_crc32[16] = d[1] ^ d[4] ^ d[7] ^ d[10] ^ d[11] ^ d[13] ^ d[15] ^ d[16] ^ d[19] ^ d[22] ^ d[23] ^ d[24] ^ d[26] ^ d[27] ^ d[28];
            next_crc32[17] = d[2] ^ d[5] ^ d[8] ^ d[11] ^ d[12] ^ d[14] ^ d[16] ^ d[17] ^ d[20] ^ d[23] ^ d[24] ^ d[25] ^ d[27] ^ d[28] ^ d[29];
            next_crc32[18] = d[0] ^ d[3] ^ d[6] ^ d[9] ^ d[12] ^ d[13] ^ d[15] ^ d[17] ^ d[18] ^ d[21] ^ d[24] ^ d[25] ^ d[26] ^ d[28] ^ d[29] ^ d[30];
            next_crc32[19] = d[0] ^ d[1] ^ d[4] ^ d[7] ^ d[10] ^ d[13] ^ d[14] ^ d[16] ^ d[18] ^ d[19] ^ d[22] ^ d[25] ^ d[26] ^ d[27] ^ d[29] ^ d[30] ^ d[31];
            next_crc32[20] = d[0] ^ d[3] ^ d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[11] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[19] ^ d[22] ^ d[27] ^ d[28] ^ d[30] ^ d[31];
            next_crc32[21] = d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[12] ^ d[15] ^ d[17] ^ d[18] ^ d[22] ^ d[26] ^ d[28] ^ d[29] ^ d[31];
            next_crc32[22] = d[2] ^ d[7] ^ d[8] ^ d[13] ^ d[18] ^ d[19] ^ d[20] ^ d[22] ^ d[26] ^ d[27] ^ d[29] ^ d[30];
            next_crc32[23] = d[0] ^ d[3] ^ d[8] ^ d[9] ^ d[14] ^ d[19] ^ d[20] ^ d[21] ^ d[23] ^ d[27] ^ d[28] ^ d[30] ^ d[31];
            next_crc32[24] = d[2] ^ d[3] ^ d[6] ^ d[7] ^ d[8] ^ d[9] ^ d[10] ^ d[15] ^ d[16] ^ d[21] ^ d[23] ^ d[24] ^ d[26] ^ d[28] ^ d[29] ^ d[31];
            next_crc32[25] = d[1] ^ d[2] ^ d[6] ^ d[9] ^ d[10] ^ d[11] ^ d[17] ^ d[20] ^ d[23] ^ d[24] ^ d[25] ^ d[26] ^ d[27] ^ d[29] ^ d[30];
            next_crc32[26] = d[2] ^ d[3] ^ d[7] ^ d[10] ^ d[11] ^ d[12] ^ d[18] ^ d[21] ^ d[24] ^ d[25] ^ d[26] ^ d[27] ^ d[28] ^ d[30] ^ d[31];
            next_crc32[27] = d[0] ^ d[1] ^ d[2] ^ d[6] ^ d[7] ^ d[11] ^ d[12] ^ d[13] ^ d[16] ^ d[19] ^ d[20] ^ d[23] ^ d[25] ^ d[27] ^ d[28] ^ d[29] ^ d[31];
            next_crc32[28] = d[0] ^ d[4] ^ d[6] ^ d[12] ^ d[13] ^ d[14] ^ d[16] ^ d[17] ^ d[21] ^ d[22] ^ d[23] ^ d[24] ^ d[28] ^ d[29] ^ d[30];
            next_crc32[29] = d[0] ^ d[1] ^ d[5] ^ d[7] ^ d[13] ^ d[14] ^ d[15] ^ d[17] ^ d[18] ^ d[22] ^ d[23] ^ d[24] ^ d[25] ^ d[29] ^ d[30] ^ d[31];
            next_crc32[30] = d[3] ^ d[4] ^ d[7] ^ d[14] ^ d[15] ^ d[18] ^ d[19] ^ d[20] ^ d[22] ^ d[24] ^ d[25] ^ d[30] ^ d[31];
            next_crc32[31] = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[7] ^ d[15] ^ d[19] ^ d[21] ^ d[22] ^ d[25] ^ d[31];
        end
    endfunction

    reg [31:0] crc_accum;
    reg [31:0] expected_crc;
    reg        crc_err;

    // =========================================================================
    // AXI4-Lite Slave Write Channel (Control & Status Registers)
    // =========================================================================
    assign s_axi_bresp = 2'b00; // OKAY
    assign s_axi_rresp = 2'b00; // OKAY

    wire reg_wr_fire = s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid;

    always @(posedge clk) begin
        if (!rst_n) begin
            s_axi_awready      <= 1'b0;
            s_axi_wready       <= 1'b0;
            s_axi_bvalid       <= 1'b0;
            ctrl_reg           <= 32'h0;
            expected_crc       <= 32'h0;
            status_write_pulse <= 1'b0;
            status_write_data  <= 32'h0;
            trigger_detected   <= 1'b0;
            cfg_worker_timer   <= 16'd100;
            cfg_payload_words   <= BRAM_DEPTH;
        end else begin
            status_write_pulse <= 1'b0;
            trigger_detected   <= 1'b0;

            if (reg_wr_fire) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;

                case (s_axi_awaddr[CFG_ADDR_WIDTH+1:2])
                    STATUS_ADDR: begin
                        status_write_pulse <= 1'b1;
                        status_write_data  <= s_axi_wdata;
                    end
                    TRIGGER_ADDR: begin
                        trigger_detected <= 1'b1;
                    end
                    EXPECTED_CRC_ADDR: begin
                        expected_crc <= s_axi_wdata;
                    end
                    COMPUTE_CTRL_ADDR: begin
                        ctrl_reg <= s_axi_wdata;
                    end
                    TIMER_ADDR: begin
                        cfg_worker_timer <= s_axi_wdata[15:0];
                    end
                    CFG_PAYLOAD_ADDR: begin
                        cfg_payload_words <= (s_axi_wdata == 0 || s_axi_wdata > BRAM_DEPTH) ? BRAM_DEPTH : s_axi_wdata;
                    end
                endcase

                s_axi_bvalid <= 1'b1;
            end else begin
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;
            end

            // Write response handshake
            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    // On-the-Fly CRC-32 Accumulation on S_AXIS Ingress
    always @(posedge clk) begin
        if (!rst_n) begin
            crc_accum <= 32'hFFFFFFFF;
        end else if (axis_wr) begin
            if (rx_ptr == 0)
                crc_accum <= next_crc32(32'hFFFFFFFF, s_axis_tdata);
            else if (rx_ptr < cfg_payload_words)
                crc_accum <= next_crc32(crc_accum, s_axis_tdata);
        end
    end

    // =========================================================================
    // AXI4-Lite Slave Read Channel (Control & Status Registers)
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= 32'd0;
        end else begin
            if (!s_axi_arready && s_axi_arvalid && !s_axi_rvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rvalid  <= 1'b1;
                case (s_axi_araddr[CFG_ADDR_WIDTH+1:2])
                    STATUS_ADDR: begin
                        s_axi_rdata <= {26'd0, crc_err, state, ip_done_irq};
                    end
                    EXPECTED_CRC_ADDR: begin
                        s_axi_rdata <= ~crc_accum;
                    end
                    COMPUTE_CTRL_ADDR: begin
                        s_axi_rdata <= ctrl_reg;
                    end
                    TIMER_ADDR: begin
                        s_axi_rdata <= {16'd0, cfg_worker_timer};
                    end
                    CFG_PAYLOAD_ADDR: begin
                        s_axi_rdata <= cfg_payload_words;
                    end
                    default: begin
                        s_axi_rdata <= 32'd0;
                    end
                endcase
            end else begin
                s_axi_arready <= 1'b0;
            end

            // Read completion handshake
            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Compute & Egress State Machine
    // =========================================================================
    reg [ADDR_WIDTH:0] pkt_len;
    reg [15:0]         work_timer;

    assign m_axis_tkeep = {(DATA_WIDTH/8){1'b1}};
    wire   tx_handshake = m_axis_tvalid && m_axis_tready;
    wire   can_issue_bram_rd = (state == S_TX_RUN) && (egress_rd_ptr < pkt_len) &&
                               (!skid_valid) &&
                               (m_axis_tready || !bram_pipe_valid || !m_axis_tvalid);

    always @(posedge clk) begin
        if (!rst_n) begin
            state             <= S_IDLE;
            rx_ptr            <= 0;
            pkt_len           <= 0;
            work_timer        <= 0;
            crc_err           <= 1'b0;
            ip_done_irq       <= 1'b0;
            comp_rd_ptr       <= 0;
            comp_stage1_valid <= 1'b0;
            comp_stage1_addr  <= 0;
            comp_wr_en        <= 1'b0;
            comp_wr_addr      <= 0;
            comp_wr_data      <= 0;
            egress_rd_ptr     <= 0;
            bram_pipe_valid   <= 1'b0;
            bram_pipe_last    <= 1'b0;
            skid_data         <= 0;
            skid_last         <= 1'b0;
            skid_valid        <= 1'b0;
            m_axis_tvalid     <= 1'b0;
            m_axis_tdata      <= 0;
            m_axis_tlast      <= 1'b0;
        end else begin
            case (state)
                // -------------------------------------------------------------
                // 1. IDLE: Wait for DMA stream (MM2S) or manual CPU start
                // -------------------------------------------------------------
                S_IDLE: begin
                    m_axis_tvalid     <= 1'b0;
                    m_axis_tlast      <= 1'b0;
                    skid_valid        <= 1'b0;
                    ip_done_irq       <= 1'b0;
                    comp_wr_en        <= 1'b0;
                    comp_stage1_valid <= 1'b0;

                    // AXI-Stream Ingress from AXI DMA MM2S:
                    if (rx_handshake) begin
                        if (s_axis_tlast || (rx_ptr == cfg_payload_words - 1)) begin
                            pkt_len  <= {1'b0, rx_ptr} + 1'b1;
                            rx_ptr   <= 0;
                            state    <= S_CHECK_CRC;
                        end else begin
                            rx_ptr   <= rx_ptr + 1'b1;
                        end
                    end
                    // Manual Software Start from CPU (write bit 0 to STATUS_ADDR 0x04):
                    else if (status_write_pulse && status_write_data[0]) begin
                        pkt_len  <= cfg_payload_words; // Process full buffer length
                        rx_ptr   <= 0;
                        state    <= S_CHECK_CRC;
                    end
                end

                // -------------------------------------------------------------
                // 2. CHECK CRC: Validate accumulated CRC vs expected CRC
                // -------------------------------------------------------------
                S_CHECK_CRC: begin
                    // Bypass CRC check if expected_crc is 0, or if accumulator matches
                    if ((~crc_accum == expected_crc) || (expected_crc == 32'h0)) begin
                        crc_err           <= 1'b0;
                        comp_rd_ptr       <= 0;
                        comp_stage1_valid <= 1'b0;
                        comp_wr_en        <= 1'b0;
                        work_timer        <= cfg_worker_timer[15:0];
                        state             <= S_COMP_RUN; // CRC OK: Proceed to pipelined compute!
                    end else begin
                        crc_err     <= 1'b1;     // CRC Failed: Abort compute to protect data!
                        ip_done_irq <= 1'b1;     // Alert PS of error via interrupt
                        state       <= S_DONE_IRQ;
                    end
                end

                // -------------------------------------------------------------
                // 3. PIPELINED COMPUTE: Read -> Compute -> Write (1 cycle/word)
                // -------------------------------------------------------------
                S_COMP_RUN: begin
                    if (trigger_detected) begin
                        comp_wr_en  <= 1'b0;
                        ip_done_irq <= 1'b0;
                        rx_ptr      <= 0;
                        state       <= S_IDLE;
                    end else if (work_timer == 0) begin
                        work_timer <= cfg_worker_timer[15:0];

                        // Stage 1: Port B Read address generation
                        if (comp_rd_ptr < pkt_len) begin
                            comp_rd_ptr       <= comp_rd_ptr + 1'b1;
                            comp_stage1_valid <= 1'b1;
                            comp_stage1_addr  <= comp_rd_ptr[ADDR_WIDTH-1:0];
                        end else begin
                            comp_stage1_valid <= 1'b0;
                        end

                        // Stage 2: Transform data & write to Port A
                        comp_wr_en   <= comp_stage1_valid;
                        comp_wr_addr <= comp_stage1_addr;
                        comp_wr_data <= (~portb_dout) + 32'h0000_1000;

                        // Completion: When the last word is written to Port A
                        if (comp_wr_en && (comp_wr_addr == pkt_len - 1)) begin
                            comp_wr_en  <= 1'b0;
                            ip_done_irq <= 1'b1; // Assert Level-High interrupt
                            state       <= S_DONE_IRQ;
                        end
                    end else begin
                        work_timer <= work_timer - 1'b1;
                    end
                end

                // -------------------------------------------------------------
                // 4. NOTIFY: Hold interrupt high until PS Kernel driver acks
                // -------------------------------------------------------------
                S_DONE_IRQ: begin
                    comp_wr_en <= 1'b0;
                    // Nominal Path: Driver ISR writes to STATUS_ADDR (0x04) to ACK
                    if (status_write_pulse) begin
                        ip_done_irq <= 1'b0; // Lower interrupt line on CPU ACK
                        if (crc_err) begin
                            crc_err <= 1'b0;
                            rx_ptr  <= 0;
                            state   <= S_IDLE;
                        end else begin
                            state   <= S_WAIT_EGRESS;
                        end
                    end
                    // Abort/Reset Path: Driver writes to TRIGGER_ADDR (0x00) to force reset
                    else if (trigger_detected) begin
                        ip_done_irq <= 1'b0;
                        crc_err     <= 1'b0;
                        rx_ptr      <= 0;
                        state       <= S_IDLE; // Always back to IDLE, never re-compute!
                    end
                end

                // -------------------------------------------------------------
                // 5. WAIT EGRESS: Wait for Driver read() to prime DMA and kick
                // -------------------------------------------------------------
                S_WAIT_EGRESS: begin
                    // Nominal Path: Driver read() writes to STATUS_ADDR (0x04) to kick egress
                    if (status_write_pulse) begin
                        egress_rd_ptr   <= 0;
                        bram_pipe_valid <= 1'b0;
                        bram_pipe_last  <= 1'b0;
                        skid_valid      <= 1'b0;
                        m_axis_tvalid   <= 1'b0;
                        m_axis_tlast    <= 1'b0;
                        state           <= S_TX_RUN;
                    end
                    // Abort/Reset Path: Driver abandons read and forces reset (0x00)
                    else if (trigger_detected) begin
                        rx_ptr <= 0;
                        state  <= S_IDLE;
                    end
                end

                // -------------------------------------------------------------
                // 6. PIPELINED EGRESS: Stream processed results from BRAM to S2MM
                // -------------------------------------------------------------
                S_TX_RUN: begin
                    if (trigger_detected) begin
                        m_axis_tvalid <= 1'b0;
                        m_axis_tlast  <= 1'b0;
                        skid_valid    <= 1'b0;
                        rx_ptr        <= 0;
                        state         <= S_IDLE;
                    end else begin
                        // 1. Issue BRAM read if pipeline can accept a beat
                        if (can_issue_bram_rd) begin
                            egress_rd_ptr   <= egress_rd_ptr + 1'b1;
                            bram_pipe_valid <= 1'b1;
                            bram_pipe_last  <= (egress_rd_ptr == pkt_len - 1);
                        end else begin
                            bram_pipe_valid <= 1'b0;
                            bram_pipe_last  <= 1'b0;
                        end

                        // 2. Stream & Skid Buffer State Machine
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
                        end else begin
                            if (m_axis_tready || !m_axis_tvalid) begin
                                if (bram_pipe_valid) begin
                                    m_axis_tdata  <= portb_dout;
                                    m_axis_tlast  <= bram_pipe_last;
                                    m_axis_tvalid <= 1'b1;
                                end else if (m_axis_tready && m_axis_tvalid) begin
                                    m_axis_tvalid <= 1'b0;
                                    m_axis_tlast  <= 1'b0;
                                end
                            end else begin
                                // Downstream stalled while BRAM read was in flight
                                if (bram_pipe_valid) begin
                                    skid_data  <= portb_dout;
                                    skid_last  <= bram_pipe_last;
                                    skid_valid <= 1'b1;
                                end
                            end
                        end

                        // 3. Egress completion check: final beat handshaked
                        if (tx_handshake && m_axis_tlast) begin
                            m_axis_tvalid <= 1'b0;
                            m_axis_tlast  <= 1'b0;
                            skid_valid    <= 1'b0;
                            rx_ptr        <= 0;
                            state         <= S_IDLE; // Frame complete! Return to S_IDLE
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule