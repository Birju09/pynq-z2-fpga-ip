`timescale 1ns / 1ps

// =============================================================================
// Module: ecat_cyclic_master_ip (ecat_cyclic_master_ip.v)
//
// Description:
//   Stack-Agnostic Hybrid PS+PL EtherCAT Cyclic Master Hardware Accelerator
//   with Dynamic Datagram Descriptor Table (DDT), Multi-Frame IEEE 802.3 /
//   EtherCAT (0x88A4) Sequencer, On-the-Fly FCS & Working Counter (WKC)
//   Verification, Zero-Copy AXI4-Stream DMA PDI Interface, and Integrated
//   40-Slave 5 KB Ring Emulator.
//
// Key Invariants:
//   1. Pre-programming CSRs or Descriptor Table registers NEVER triggers FSM
//      transitions out of S_IDLE. Cyclic execution is triggered strictly by
//      s_axis_tlast on the final beat of the MM2S DMA transfer (or explicit
//      manual write of bit 0 to STATUS_ADDR 0x04).
//   2. ip_done_irq is a Level-High interrupt line held asserted until
//      explicitly acknowledged by a CPU write to STATUS_ADDR (0x04).
//   3. S2MM Egress (S_TX_RUN) implements a full skid buffer for 1-cycle BRAM
//      read latency under downstream m_axis_tready backpressure.
//   4. All word pointers and counters iterating through BRAM_DEPTH are sized
//      to [ADDR_WIDTH:0] (N+1 bits) to prevent rollover overflow.
// =============================================================================

module ecat_cyclic_master_ip #(
    parameter integer DATA_WIDTH     = 32,
    parameter integer BRAM_DEPTH     = 2048, // 2048 words = 8192 bytes (8 KB max PDI)
    parameter integer ADDR_WIDTH     = 11,   // 2^11 = 2048 words
    parameter integer CFG_DEPTH      = 256,  // 256 words (1 KB CSR + Descriptor space)
    parameter integer CFG_ADDR_WIDTH = 8,    // 2^8 = 256 words
    parameter integer MAX_DESCS      = 8,    // Up to 8 cyclic EtherCAT frames per cycle
    parameter integer NUM_SLAVES     = 40    // 40-slave ring emulator
)(
    input  wire                      clk,
    input  wire                      rst_n,

    // -------------------------------------------------------------------------
    // 1. AXI4-Lite Memory-Mapped Slave (CSRs + Datagram Descriptor Table)
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
    // 2. AXI4-Stream Slave (AXI DMA MM2S: PS TxPDO Buffer -> PDI BRAM)
    // -------------------------------------------------------------------------
    input  wire [DATA_WIDTH-1:0]     s_axis_tdata,
    input  wire [(DATA_WIDTH/8)-1:0] s_axis_tkeep,
    input  wire                      s_axis_tvalid,
    output wire                      s_axis_tready,
    input  wire                      s_axis_tlast,

    // -------------------------------------------------------------------------
    // 3. AXI4-Stream Master (AXI DMA S2MM: PDI BRAM -> PS RxPDO Buffer)
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
    // AXI4-Lite Register Word Indices (s_axi_a*addr[CFG_ADDR_WIDTH+1:2])
    // =========================================================================
    localparam [CFG_ADDR_WIDTH-1:0]
        TRIGGER_ADDR       = 8'h00, // 0x00: Write forces FSM abort/reset to S_IDLE
        STATUS_ADDR        = 8'h01, // 0x04: Read status / Write ACK IRQ or kick
        COMPUTE_CTRL_ADDR  = 8'h02, // 0x08: [3:0]=num_descs, [4]=emu_en, [15:8]=seq_id
        EXPECTED_CRC_ADDR  = 8'h03, // 0x0C: Write expected DMA CRC / Read calc CRC
        PACING_TIMER_ADDR  = 8'h04, // 0x10: Wire pacing delay cycles per beat (0=sim speedup, >0=100BASE-TX)
        CFG_PAYLOAD_ADDR   = 8'h05, // 0x14: Total PDI words (default 1280 = 5120B)
        SLAVE_MASK_LO_ADDR = 8'h06, // 0x18: Slaves 0..31 online bitmask
        SLAVE_MASK_HI_ADDR = 8'h07, // 0x1C: Slaves 32..39 online bitmask
        LAST_CYCLE_TICKS   = 8'h08; // 0x20: Elapsed clock ticks in last EtherCAT cycle

    // Descriptor Table Base: Word 16 (Byte offset 0x40 .. 0xBF)
    // Descriptor i (0..7):
    //   Word (16 + 4*i + 0): DESC_LOGICAL_ADDR[i]
    //   Word (16 + 4*i + 1): DESC_CTRL[i]     ({idx[7:0], cmd[7:0], 4'd0, words[11:0]})
    //   Word (16 + 4*i + 2): DESC_PDI_WKC[i]  ({expected_wkc[15:0], 4'd0, pdi_word_offset[11:0]})
    //   Word (16 + 4*i + 3): DESC_STATUS[i]   ({13'd0, done, fcs_err, wkc_err, actual_wkc[15:0]})

    // =========================================================================
    // Configuration & Descriptor Registers
    // =========================================================================
    reg [DATA_WIDTH-1:0] ctrl_reg;
    reg [DATA_WIDTH-1:0] expected_crc;
    reg [DATA_WIDTH-1:0] cfg_pacing_timer; // Wire pacing delay cycles per beat (0=sim speedup, >0=100BASE-TX)
    reg [DATA_WIDTH-1:0] cfg_payload_words;
    reg [NUM_SLAVES-1:0] slave_online_mask;

    reg [31:0]           desc_logical_addr [0:MAX_DESCS-1];
    reg [31:0]           desc_ctrl         [0:MAX_DESCS-1];
    reg [31:0]           desc_pdi_wkc      [0:MAX_DESCS-1];
    reg [31:0]           desc_status       [0:MAX_DESCS-1];

    reg                  status_write_pulse;
    reg [DATA_WIDTH-1:0] status_write_data;
    reg                  trigger_detected;

    wire [3:0]           cfg_num_descs = (ctrl_reg[3:0] == 4'd0) ? 4'd4 :
                                         (ctrl_reg[3:0] > MAX_DESCS) ? MAX_DESCS[3:0] :
                                         ctrl_reg[3:0];

    // =========================================================================
    // Master FSM States
    // =========================================================================
    localparam [3:0]
        S_IDLE            = 4'd0, // Awaiting S_AXIS MM2S stream or manual start
        S_CHECK_CRC       = 4'd1, // Verify MM2S DMA CRC-32
        S_ECAT_FRAME_INIT = 4'd2, // Load current Datagram Descriptor
        S_ECAT_XFER       = 4'd3, // Stream TX frame -> 40-Slave Ring -> Parse RX frame
        S_DONE_IRQ        = 4'd5, // Assert Level-High IRQ to PS & wait for ACK
        S_WAIT_EGRESS     = 4'd6, // Wait for Driver read() kick
        S_TX_RUN          = 4'd7; // Stream RxPDO PDI to S2MM DMA with Skid Buffer

    reg [3:0] state;

    // =========================================================================
    // Dual-Port Inferred Block RAM (8 KB Process Data Image)
    // Port A: MM2S Ingress Writes (S_IDLE) & EtherCAT RX Frame Writes (S_ECAT_XFER)
    // Port B: EtherCAT TX Frame Reads (S_ECAT_XFER) & S2MM Egress Reads (S_TX_RUN)
    // =========================================================================
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] bram [0:BRAM_DEPTH-1];

    reg  [ADDR_WIDTH:0]   rx_ptr;
    assign s_axis_tready = (state == S_IDLE);
    wire                  rx_handshake = s_axis_tvalid && s_axis_tready;
    wire                  axis_wr      = rx_handshake && (state == S_IDLE);

    // EtherCAT Frame RX Write signals (Port A)
    reg                   ecat_rx_wr_en;
    reg  [ADDR_WIDTH-1:0] ecat_rx_wr_addr;
    reg  [DATA_WIDTH-1:0] ecat_rx_wr_data;

    wire [ADDR_WIDTH-1:0] porta_addr = (state == S_IDLE) ? rx_ptr[ADDR_WIDTH-1:0] : ecat_rx_wr_addr;
    wire [DATA_WIDTH-1:0] porta_din  = (state == S_IDLE) ? s_axis_tdata : ecat_rx_wr_data;
    wire                  porta_we   = (state == S_IDLE) ? axis_wr : ecat_rx_wr_en;

    always @(posedge clk) begin
        if (porta_we) begin
            bram[porta_addr] <= porta_din;
        end
    end

    // Port B Read Multiplexer (EtherCAT TX Frame Builder vs S2MM Egress)
    reg  [ADDR_WIDTH-1:0] ecat_tx_rd_addr;
    reg  [ADDR_WIDTH:0]   egress_rd_ptr;
    wire [ADDR_WIDTH-1:0] portb_addr = (state == S_TX_RUN) ? egress_rd_ptr[ADDR_WIDTH-1:0] :
                                                             ecat_tx_rd_addr;
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
    reg        crc_err;
    reg        wkc_err;

    // On-the-Fly CRC-32 Accumulation on S_AXIS MM2S Ingress
    always @(posedge clk) begin
        if (!rst_n) begin
            crc_accum <= 32'hFFFF_FFFF;
        end else if (axis_wr) begin
            if (rx_ptr == 0)
                crc_accum <= next_crc32(32'hFFFF_FFFF, s_axis_tdata);
            else if (rx_ptr < cfg_payload_words)
                crc_accum <= next_crc32(crc_accum, s_axis_tdata);
        end
    end

    // =========================================================================
    // Instantiation of In-FPGA 40-Slave, 5 KB Ring Emulator
    // =========================================================================
    reg  [DATA_WIDTH-1:0] eth_tx_tdata;
    reg                   eth_tx_tvalid;
    wire                  eth_tx_tready;
    reg                   eth_tx_tlast;

    wire [DATA_WIDTH-1:0] eth_rx_tdata;
    wire                  eth_rx_tvalid;
    wire                  eth_rx_tready;
    wire                  eth_rx_tlast;

    assign eth_rx_tready = (state == S_ECAT_XFER);

    ecat_40slave_emulator #(
        .DATA_WIDTH  (DATA_WIDTH),
        .NUM_SLAVES  (NUM_SLAVES),
        .SLAVE_BYTES (128),
        .TOTAL_WORDS (BRAM_DEPTH)
    ) u_slave_ring_emulator (
        .clk               (clk),
        .rst_n             (rst_n),
        .slave_online_mask (slave_online_mask),
        .s_axis_tdata      (eth_tx_tdata),
        .s_axis_tvalid     (eth_tx_tvalid),
        .s_axis_tready     (eth_tx_tready),
        .s_axis_tlast      (eth_tx_tlast),
        .m_axis_tdata      (eth_rx_tdata),
        .m_axis_tvalid     (eth_rx_tvalid),
        .m_axis_tready     (eth_rx_tready),
        .m_axis_tlast      (eth_rx_tlast)
    );

    // =========================================================================
    // AXI4-Lite Slave Write Channel (CSRs & Descriptor Table)
    // =========================================================================
    assign s_axi_bresp = 2'b00;
    assign s_axi_rresp = 2'b00;

    wire reg_wr_fire = s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid;
    wire [CFG_ADDR_WIDTH-1:0] wr_word_idx = s_axi_awaddr[CFG_ADDR_WIDTH+1:2];
    wire [CFG_ADDR_WIDTH-1:0] wr_desc_off = wr_word_idx - 8'h10;
    wire [2:0]                wr_desc_idx = wr_desc_off[4:2];
    wire [1:0]                wr_desc_reg = wr_desc_off[1:0];

    integer d_init;
    always @(posedge clk) begin
        if (!rst_n) begin
            s_axi_awready      <= 1'b0;
            s_axi_wready       <= 1'b0;
            s_axi_bvalid       <= 1'b0;
            ctrl_reg           <= 32'h0000_0014; // Default: 4 descriptors, internal emulator enabled
            expected_crc       <= 32'h0000_0000;
            status_write_pulse <= 1'b0;
            status_write_data  <= 32'h0000_0000;
            trigger_detected   <= 1'b0;
            cfg_pacing_timer   <= 32'd0;         // 0 extra delay ticks for max speed / sim speedup by default
            cfg_payload_words  <= 32'd1280;      // Default: 1280 words = 5120 bytes (5 KB)
            slave_online_mask  <= {NUM_SLAVES{1'b1}}; // All 40 slaves online

            // Default 4-Frame Descriptor Table for 40 Slaves, 5 KB (128B/slave):
            // Frame 0: Slaves 0..10  (11 slaves = 1408B = 352 words), Expected WKC = 33
            desc_logical_addr[0] <= 32'd0;
            desc_ctrl[0]         <= {8'd0, 8'h0C, 4'd0, 12'd352};
            desc_pdi_wkc[0]      <= {16'd33, 4'd0, 12'd0};

            // Frame 1: Slaves 11..21 (11 slaves = 1408B = 352 words), Expected WKC = 33
            desc_logical_addr[1] <= 32'd1408;
            desc_ctrl[1]         <= {8'd1, 8'h0C, 4'd0, 12'd352};
            desc_pdi_wkc[1]      <= {16'd33, 4'd0, 12'd352};

            // Frame 2: Slaves 22..32 (11 slaves = 1408B = 352 words), Expected WKC = 33
            desc_logical_addr[2] <= 32'd2816;
            desc_ctrl[2]         <= {8'd2, 8'h0C, 4'd0, 12'd352};
            desc_pdi_wkc[2]      <= {16'd33, 4'd0, 12'd704};

            // Frame 3: Slaves 33..39 (7 slaves  = 896B  = 224 words), Expected WKC = 21
            desc_logical_addr[3] <= 32'd4224;
            desc_ctrl[3]         <= {8'd3, 8'h0C, 4'd0, 12'd224};
            desc_pdi_wkc[3]      <= {16'd21, 4'd0, 12'd1056};

            for (d_init = 4; d_init < MAX_DESCS; d_init = d_init + 1) begin
                desc_logical_addr[d_init] <= 32'd0;
                desc_ctrl[d_init]         <= 32'd0;
                desc_pdi_wkc[d_init]      <= 32'd0;
            end
        end else begin
            status_write_pulse <= 1'b0;
            trigger_detected   <= 1'b0;

            if (reg_wr_fire) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;

                if (wr_word_idx >= 8'h10 && wr_word_idx < 8'h30) begin
                    // Descriptor Table Write (Words 16..47 -> Byte offsets 0x40..0xBF)
                    case (wr_desc_reg)
                        2'd0: desc_logical_addr[wr_desc_idx] <= s_axi_wdata;
                        2'd1: desc_ctrl[wr_desc_idx]         <= s_axi_wdata;
                        2'd2: desc_pdi_wkc[wr_desc_idx]      <= s_axi_wdata;
                        default: ; // Word 3 is read-only status
                    endcase
                end else begin
                    case (wr_word_idx)
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
                        PACING_TIMER_ADDR: begin
                            cfg_pacing_timer <= {16'd0, s_axi_wdata[15:0]};
                        end
                        CFG_PAYLOAD_ADDR: begin
                            cfg_payload_words <= (s_axi_wdata == 0 || s_axi_wdata > BRAM_DEPTH) ?
                                                 BRAM_DEPTH : s_axi_wdata;
                        end
                        SLAVE_MASK_LO_ADDR: begin
                            slave_online_mask[31:0] <= s_axi_wdata;
                        end
                        SLAVE_MASK_HI_ADDR: begin
                            slave_online_mask[39:32] <= s_axi_wdata[7:0];
                        end
                        default: ;
                    endcase
                end

                s_axi_bvalid <= 1'b1;
            end else begin
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;
            end

            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    // =========================================================================
    // AXI4-Lite Slave Read Channel
    // =========================================================================
    reg  [15:0] total_actual_wkc;
    reg  [3:0]  completed_frames;
    reg  [31:0] last_cycle_ticks;

    wire [CFG_ADDR_WIDTH-1:0] rd_word_idx = s_axi_araddr[CFG_ADDR_WIDTH+1:2];
    wire [CFG_ADDR_WIDTH-1:0] rd_desc_off = rd_word_idx - 8'h10;
    wire [2:0]                rd_desc_idx = rd_desc_off[4:2];
    wire [1:0]                rd_desc_reg = rd_desc_off[1:0];

    always @(posedge clk) begin
        if (!rst_n) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= 32'd0;
        end else begin
            if (!s_axi_arready && s_axi_arvalid && !s_axi_rvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rvalid  <= 1'b1;

                if (rd_word_idx >= 8'h10 && rd_word_idx < 8'h30) begin
                    case (rd_desc_reg)
                        2'd0: s_axi_rdata <= desc_logical_addr[rd_desc_idx];
                        2'd1: s_axi_rdata <= desc_ctrl[rd_desc_idx];
                        2'd2: s_axi_rdata <= desc_pdi_wkc[rd_desc_idx];
                        2'd3: s_axi_rdata <= desc_status[rd_desc_idx];
                    endcase
                end else begin
                    case (rd_word_idx)
                        STATUS_ADDR: begin
                            s_axi_rdata <= {
                                total_actual_wkc, // [31:16] Total WKC across cycle
                                cfg_num_descs,    // [15:12] Active descriptor count
                                1'b0,             // [11]    Reserved
                                completed_frames, // [10:7]  Frames completed
                                wkc_err,          // [6]     WKC mismatch error
                                crc_err,          // [5]     Ingress CRC or FCS error
                                state,            // [4:1]   FSM state
                                ip_done_irq       // [0]     Level-High IRQ status
                            };
                        end
                        EXPECTED_CRC_ADDR:  s_axi_rdata <= ~crc_accum;
                        COMPUTE_CTRL_ADDR:  s_axi_rdata <= ctrl_reg;
                        PACING_TIMER_ADDR:  s_axi_rdata <= cfg_pacing_timer;
                        CFG_PAYLOAD_ADDR:   s_axi_rdata <= cfg_payload_words;
                        SLAVE_MASK_LO_ADDR: s_axi_rdata <= slave_online_mask[31:0];
                        SLAVE_MASK_HI_ADDR: s_axi_rdata <= {24'd0, slave_online_mask[39:32]};
                        LAST_CYCLE_TICKS:   s_axi_rdata <= last_cycle_ticks;
                        default:            s_axi_rdata <= 32'd0;
                    endcase
                end
            end else begin
                s_axi_arready <= 1'b0;
            end

            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Multi-Frame EtherCAT Cyclic Sequencer & S2MM Egress FSM
    // =========================================================================
    reg [ADDR_WIDTH:0] pkt_len;
    reg [15:0]         pacing_timer;
    reg [31:0]         cycle_tick_cnt;

    // Active Descriptor Working Registers
    reg [3:0]          cur_desc_idx;
    reg [31:0]         cur_logical_addr;
    reg [11:0]         cur_payload_words;
    reg [7:0]          cur_ecat_cmd;
    reg [7:0]          cur_dgrm_idx;
    reg [11:0]         cur_pdi_offset;
    reg [15:0]         cur_expected_wkc;

    // Frame TX / RX Counters & FCS Calculators
    reg [11:0]         tx_beat_idx;
    reg                tx_frame_done;
    reg [31:0]         tx_fcs_accum;

    reg [11:0]         rx_beat_idx;
    reg [31:0]         rx_fcs_accum;
    reg [15:0]         rx_latched_wkc;
    reg                rx_frame_wkc_err;

    // Skid Buffer & Pipelined Egress Signals (per GEMINI.md Rule 1c)
    reg                bram_pipe_valid;
    reg                bram_pipe_last;
    reg [DATA_WIDTH-1:0] skid_data;
    reg                skid_last;
    reg                skid_valid;

    assign m_axis_tkeep = {(DATA_WIDTH/8){1'b1}};
    wire   tx_handshake = m_axis_tvalid && m_axis_tready;
    wire   can_issue_bram_rd = (state == S_TX_RUN) && (egress_rd_ptr < pkt_len) &&
                               (!skid_valid) &&
                               (m_axis_tready || !bram_pipe_valid || !m_axis_tvalid);

    // Combinational Frame Word Builder for Current TX Beat
    wire [15:0] cur_payload_bytes = {2'b00, cur_payload_words, 2'b00};
    wire [15:0] ecat_hdr_word     = {4'h1, (cur_payload_bytes[11:0] + 12'd12)}; // Type=1, Len=Payload+12

    // Freeze BRAM Port B output on first cycle of beat when pacing_timer > 0
    reg  [DATA_WIDTH-1:0] tx_word_buf;
    always @(posedge clk) begin
        if (!rst_n) begin
            tx_word_buf <= 32'd0;
        end else if (pacing_timer == cfg_pacing_timer[15:0]) begin
            tx_word_buf <= portb_dout;
        end
    end
    wire [31:0] pdo_tx_word = (cfg_pacing_timer == 32'd0) ? portb_dout : tx_word_buf;

    reg  [31:0] next_eth_tx_word;
    always @(*) begin
        case (tx_beat_idx)
            12'd0:   next_eth_tx_word = 32'hFFFF_FFFF; // Dst MAC [31:0]
            12'd1:   next_eth_tx_word = 32'hFFFF_0200; // Dst MAC [47:32], Src MAC [47:32]
            12'd2:   next_eth_tx_word = 32'h5E10_0001; // Src MAC [31:0]
            12'd3:   next_eth_tx_word = {16'h88A4, ecat_hdr_word}; // EtherType 0x88A4 + ECAT Hdr
            12'd4:   next_eth_tx_word = {cur_ecat_cmd, cur_dgrm_idx, cur_payload_bytes};
            12'd5:   next_eth_tx_word = cur_logical_addr;
            default: begin
                if (tx_beat_idx < (12'd6 + cur_payload_words))
                    next_eth_tx_word = pdo_tx_word; // Prefetched TxPDO word from BRAM Port B
                else if (tx_beat_idx == (12'd6 + cur_payload_words))
                    next_eth_tx_word = 32'h0000_0000; // Outgoing WKC initialized to 0
                else
                    next_eth_tx_word = ~tx_fcs_accum; // Trailing Ethernet FCS (CRC-32)
            end
        endcase
    end

    integer s_rst;
    always @(posedge clk) begin
        if (!rst_n) begin
            state             <= S_IDLE;
            rx_ptr            <= 0;
            pkt_len           <= 0;
            pacing_timer      <= 0;
            cycle_tick_cnt    <= 0;
            last_cycle_ticks  <= 0;
            crc_err           <= 1'b0;
            wkc_err           <= 1'b0;
            ip_done_irq       <= 1'b0;
            total_actual_wkc  <= 16'd0;
            completed_frames  <= 4'd0;

            cur_desc_idx      <= 4'd0;
            cur_logical_addr  <= 32'd0;
            cur_payload_words <= 12'd0;
            cur_ecat_cmd      <= 8'h0C;
            cur_dgrm_idx      <= 8'd0;
            cur_pdi_offset    <= 12'd0;
            cur_expected_wkc  <= 16'd0;

            tx_beat_idx       <= 12'd0;
            tx_frame_done     <= 1'b0;
            tx_fcs_accum      <= 32'hFFFF_FFFF;
            eth_tx_tdata      <= 32'd0;
            eth_tx_tvalid     <= 1'b0;
            eth_tx_tlast      <= 1'b0;
            ecat_tx_rd_addr   <= {ADDR_WIDTH{1'b0}};

            rx_beat_idx       <= 12'd0;
            rx_fcs_accum      <= 32'hFFFF_FFFF;
            rx_latched_wkc    <= 16'd0;
            rx_frame_wkc_err  <= 1'b0;
            ecat_rx_wr_en     <= 1'b0;
            ecat_rx_wr_addr   <= {ADDR_WIDTH{1'b0}};
            ecat_rx_wr_data   <= 32'd0;

            egress_rd_ptr     <= 0;
            bram_pipe_valid   <= 1'b0;
            bram_pipe_last    <= 1'b0;
            skid_data         <= 0;
            skid_last         <= 1'b0;
            skid_valid        <= 1'b0;
            m_axis_tvalid     <= 1'b0;
            m_axis_tdata      <= 0;
            m_axis_tlast      <= 1'b0;

            for (s_rst = 0; s_rst < MAX_DESCS; s_rst = s_rst + 1) begin
                desc_status[s_rst] <= 32'd0;
            end
        end else begin
            case (state)
                // -------------------------------------------------------------
                // 1. IDLE: Wait for DMA MM2S stream or explicit CPU start
                // -------------------------------------------------------------
                S_IDLE: begin
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;
                    skid_valid    <= 1'b0;
                    ip_done_irq   <= 1'b0;
                    ecat_rx_wr_en <= 1'b0;
                    eth_tx_tvalid <= 1'b0;
                    eth_tx_tlast  <= 1'b0;

                    if (rx_handshake) begin
                        if (s_axis_tlast || (rx_ptr == cfg_payload_words - 1)) begin
                            pkt_len <= {1'b0, rx_ptr[ADDR_WIDTH-1:0]} + 1'b1;
                            rx_ptr  <= 0;
                            state   <= S_CHECK_CRC;
                        end else begin
                            rx_ptr  <= rx_ptr + 1'b1;
                        end
                    end else if (status_write_pulse && status_write_data[0]) begin
                        pkt_len <= cfg_payload_words[ADDR_WIDTH:0];
                        rx_ptr  <= 0;
                        state   <= S_CHECK_CRC;
                    end
                end

                // -------------------------------------------------------------
                // 2. CHECK CRC: Validate DMA MM2S CRC-32 & initialize cycle
                // -------------------------------------------------------------
                S_CHECK_CRC: begin
                    if ((~crc_accum == expected_crc) || (expected_crc == 32'h0)) begin
                        crc_err          <= 1'b0;
                        wkc_err          <= 1'b0;
                        cur_desc_idx     <= 4'd0;
                        total_actual_wkc <= 16'd0;
                        completed_frames <= 4'd0;
                        cycle_tick_cnt   <= 32'd0;
                        state            <= S_ECAT_FRAME_INIT;
                    end else begin
                        crc_err     <= 1'b1;
                        ip_done_irq <= 1'b1;
                        state       <= S_DONE_IRQ;
                    end
                end

                // -------------------------------------------------------------
                // 3. ECAT FRAME INIT: Load Descriptor[cur_desc_idx]
                // -------------------------------------------------------------
                S_ECAT_FRAME_INIT: begin
                    cycle_tick_cnt    <= cycle_tick_cnt + 32'd1;
                    cur_logical_addr  <= desc_logical_addr[cur_desc_idx[2:0]];
                    cur_payload_words <= (desc_ctrl[cur_desc_idx[2:0]][11:0] == 12'd0) ?
                                         12'd1 : desc_ctrl[cur_desc_idx[2:0]][11:0];
                    cur_ecat_cmd      <= desc_ctrl[cur_desc_idx[2:0]][23:16];
                    cur_dgrm_idx      <= desc_ctrl[cur_desc_idx[2:0]][31:24];
                    cur_pdi_offset    <= desc_pdi_wkc[cur_desc_idx[2:0]][11:0];
                    cur_expected_wkc  <= desc_pdi_wkc[cur_desc_idx[2:0]][31:16];

                    // Prime Port B read address to first word of this frame's PDI slice
                    ecat_tx_rd_addr   <= desc_pdi_wkc[cur_desc_idx[2:0]][ADDR_WIDTH-1:0];

                    tx_beat_idx       <= 12'd0;
                    tx_frame_done     <= 1'b0;
                    tx_fcs_accum      <= 32'hFFFF_FFFF;
                    eth_tx_tvalid     <= 1'b0;
                    eth_tx_tlast      <= 1'b0;

                    rx_beat_idx       <= 12'd0;
                    rx_fcs_accum      <= 32'hFFFF_FFFF;
                    rx_latched_wkc    <= 16'd0;
                    rx_frame_wkc_err  <= 1'b0;
                    ecat_rx_wr_en     <= 1'b0;
                    pacing_timer      <= cfg_pacing_timer[15:0];
                    state             <= S_ECAT_XFER;
                end

                // -------------------------------------------------------------
                // 4. ECAT XFER: Full-Duplex Frame TX & Returning Frame RX
                // -------------------------------------------------------------
                S_ECAT_XFER: begin
                    if (trigger_detected) begin
                        eth_tx_tvalid <= 1'b0;
                        eth_tx_tlast  <= 1'b0;
                        ecat_rx_wr_en <= 1'b0;
                        ip_done_irq   <= 1'b0;
                        rx_ptr        <= 0;
                        state         <= S_IDLE;
                    end else begin
                        cycle_tick_cnt <= cycle_tick_cnt + 32'd1;
                        ecat_rx_wr_en  <= 1'b0;

                        // --- A. Frame Transmitter (Master -> 40-Slave Ring) ---
                        if (!tx_frame_done) begin
                            if (!eth_tx_tvalid || eth_tx_tready) begin
                                if (pacing_timer == 16'd0) begin
                                    pacing_timer  <= cfg_pacing_timer[15:0];
                                    eth_tx_tdata  <= next_eth_tx_word;
                                    eth_tx_tvalid <= 1'b1;
                                    eth_tx_tlast  <= (tx_beat_idx == (12'd7 + cur_payload_words));

                                    if (tx_beat_idx == (12'd7 + cur_payload_words)) begin
                                        tx_frame_done <= 1'b1;
                                    end else begin
                                        tx_beat_idx <= tx_beat_idx + 12'd1;
                                        if (tx_beat_idx == 12'd0)
                                            tx_fcs_accum <= next_crc32(32'hFFFF_FFFF, next_eth_tx_word);
                                        else
                                            tx_fcs_accum <= next_crc32(tx_fcs_accum, next_eth_tx_word);

                                        // Advance BRAM Port B address 1 beat ahead of PDO window
                                        if ((tx_beat_idx >= 12'd5) &&
                                            (tx_beat_idx < (12'd5 + cur_payload_words))) begin
                                            ecat_tx_rd_addr <= ecat_tx_rd_addr + 1'b1;
                                        end
                                    end
                                end else begin
                                    pacing_timer  <= pacing_timer - 16'd1;
                                    eth_tx_tvalid <= 1'b0;
                                    eth_tx_tlast  <= 1'b0;
                                end
                            end
                        end else if (eth_tx_tvalid && eth_tx_tready) begin
                            eth_tx_tvalid <= 1'b0;
                            eth_tx_tlast  <= 1'b0;
                        end

                        // --- B. Frame Receiver (40-Slave Ring -> Master PDI BRAM) ---
                        if (eth_rx_tvalid && eth_rx_tready) begin
                            if (eth_rx_tlast) begin
                                // Final Beat: Verify Ethernet FCS (CRC-32) & finalize descriptor
                                desc_status[cur_desc_idx[2:0]] <= {
                                    13'd0,
                                    1'b1,                             // [18] frame_done
                                    (eth_rx_tdata != ~rx_fcs_accum),  // [17] frame_fcs_err
                                    rx_frame_wkc_err,                 // [16] frame_wkc_err
                                    rx_latched_wkc                    // [15:0] actual_wkc
                                };

                                if (eth_rx_tdata != ~rx_fcs_accum) begin
                                    crc_err <= 1'b1;
                                end
                                if (rx_frame_wkc_err) begin
                                    wkc_err <= 1'b1;
                                end

                                total_actual_wkc <= total_actual_wkc + rx_latched_wkc;
                                completed_frames <= completed_frames + 4'd1;

                                // Check if more descriptors remain in this cycle
                                if ((cur_desc_idx + 4'd1) < cfg_num_descs) begin
                                    cur_desc_idx <= cur_desc_idx + 4'd1;
                                    state        <= S_ECAT_FRAME_INIT;
                                end else begin
                                    last_cycle_ticks <= cycle_tick_cnt + 32'd1;
                                    ip_done_irq      <= 1'b1; // Assert Level-High IRQ to PS!
                                    state            <= S_DONE_IRQ;
                                end
                            end else begin
                                rx_beat_idx <= rx_beat_idx + 12'd1;

                                // Accumulate expected Ethernet FCS across beats 0..6+N
                                if (rx_beat_idx == 12'd0)
                                    rx_fcs_accum <= next_crc32(32'hFFFF_FFFF, eth_rx_tdata);
                                else
                                    rx_fcs_accum <= next_crc32(rx_fcs_accum, eth_rx_tdata);

                                // Beats 6 .. 6+N-1: Extract RxPDO into PDI BRAM Port A
                                if ((rx_beat_idx >= 12'd6) &&
                                    (rx_beat_idx < (12'd6 + cur_payload_words))) begin
                                    ecat_rx_wr_en   <= 1'b1;
                                    ecat_rx_wr_addr <= cur_pdi_offset[ADDR_WIDTH-1:0] +
                                                       (rx_beat_idx[ADDR_WIDTH-1:0] - 11'd6);
                                    ecat_rx_wr_data <= eth_rx_tdata;
                                end
                                // Beat 6+N: Extract Working Counter (WKC) & validate
                                else if (rx_beat_idx == (12'd6 + cur_payload_words)) begin
                                    rx_latched_wkc   <= eth_rx_tdata[15:0];
                                    rx_frame_wkc_err <= (cur_expected_wkc != 16'd0) &&
                                                        (eth_rx_tdata[15:0] != cur_expected_wkc);
                                end
                            end
                        end
                    end
                end

                // -------------------------------------------------------------
                // 5. DONE IRQ: Hold Level-High IRQ until PS Driver ISR ACKs
                // -------------------------------------------------------------
                S_DONE_IRQ: begin
                    ecat_rx_wr_en <= 1'b0;
                    eth_tx_tvalid <= 1'b0;
                    if (status_write_pulse) begin
                        ip_done_irq <= 1'b0;
                        if (crc_err) begin
                            crc_err <= 1'b0;
                            rx_ptr  <= 0;
                            state   <= S_IDLE;
                        end else begin
                            state   <= S_WAIT_EGRESS;
                        end
                    end else if (trigger_detected) begin
                        ip_done_irq <= 1'b0;
                        crc_err     <= 1'b0;
                        wkc_err     <= 1'b0;
                        rx_ptr      <= 0;
                        state       <= S_IDLE;
                    end
                end

                // -------------------------------------------------------------
                // 6. WAIT EGRESS: Wait for Driver read() to prime S2MM & kick
                // -------------------------------------------------------------
                S_WAIT_EGRESS: begin
                    if (status_write_pulse) begin
                        egress_rd_ptr   <= 0;
                        bram_pipe_valid <= 1'b0;
                        bram_pipe_last  <= 1'b0;
                        skid_valid      <= 1'b0;
                        m_axis_tvalid   <= 1'b0;
                        m_axis_tlast    <= 1'b0;
                        state           <= S_TX_RUN;
                    end else if (trigger_detected) begin
                        rx_ptr <= 0;
                        state  <= S_IDLE;
                    end
                end

                // -------------------------------------------------------------
                // 7. PIPELINED EGRESS: Stream 5 KB RxPDO PDI to S2MM DMA
                // -------------------------------------------------------------
                S_TX_RUN: begin
                    if (trigger_detected) begin
                        m_axis_tvalid <= 1'b0;
                        m_axis_tlast  <= 1'b0;
                        skid_valid    <= 1'b0;
                        rx_ptr        <= 0;
                        state         <= S_IDLE;
                    end else begin
                        if (can_issue_bram_rd) begin
                            egress_rd_ptr   <= egress_rd_ptr + 1'b1;
                            bram_pipe_valid <= 1'b1;
                            bram_pipe_last  <= (egress_rd_ptr == pkt_len - 1);
                        end else begin
                            bram_pipe_valid <= 1'b0;
                            bram_pipe_last  <= 1'b0;
                        end

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
                                if (bram_pipe_valid) begin
                                    skid_data  <= portb_dout;
                                    skid_last  <= bram_pipe_last;
                                    skid_valid <= 1'b1;
                                end
                            end
                        end

                        if (tx_handshake && m_axis_tlast) begin
                            m_axis_tvalid <= 1'b0;
                            m_axis_tlast  <= 1'b0;
                            skid_valid    <= 1'b0;
                            rx_ptr        <= 0;
                            state         <= S_IDLE;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

