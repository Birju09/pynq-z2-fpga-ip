`timescale 1ns / 1ps

// =============================================================================
// Module: rtps_fastpath_ip (rtps_fastpath_ip.v)
//
// Description:
//   Hybrid PS+PL RTPS v2.x Best-Effort Cyclic Data-Plane Hardware Accelerator
//   for RTI Connext / Eclipse Cyclone DDS Interoperability.
//   Features:
//     - GUID-Impersonation Endpoint Descriptor Table (EDT): Shadow/Active
//       atomic commit at cycle boundaries (4 Writer slots + 4 Reader CAM slots).
//     - Dual AXI4-Stream Ports:
//         1. Sample Port (DMA0): s_axis (MM2S Command sample in) & m_axis
//            (S2MM Feedback sample out) with on-the-fly IEEE 802.3 CRC-32
//            verification and 1-cycle BRAM read skid buffer.
//         2. Virtual MAC / Wire Port (DMA1): m_axis_wire & s_axis_wire carrying
//            complete IPv4-fragmented Ethernet/IPv4/UDP/RTPS frames, with a
//            CSR-selectable loopback mux to the integrated rtps_peer_emulator.
//     - On-the-Fly UDP Checksum Accumulation during MM2S sample ingress,
//       zero-latency 3-fragment IPv4 header + checksum generation, and
//       Dedicated Secondary Source IP + incrementing 16-bit IP ID.
//     - 3-Slot Store-and-Forward IPv4 Fragment Reassembly Engine (keyed by
//       SrcIP + IP_ID, supporting arbitrary fragment arrival order).
//     - Generic Hardware RTPS v2.x Submessage Walker: validates 'RTPS' magic,
//       matches remote writer GuidPrefix in 4-entry CAM, honors per-submessage
//       E-flag endianness & octetsToNextHeader, latches INFO_TS, skips unknown/
//       vendor submessages (0x80+) and inline QoS (Q=1 -> PID_SENTINEL), and
//       enforces Latest-Value delivery (seqNum > last_rx_seq_num).
//     - 100 ms Hardware Lifecycle Watchdog & RX Sample Timeout Counter.
//
// Key Invariants and Notes:
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

module rtps_fastpath_ip #(
    parameter integer DATA_WIDTH     = 32,
    parameter integer BRAM_DEPTH     = 2048, // 2048 words = 8192 bytes (8 KB max sample)
    parameter integer ADDR_WIDTH     = 11,   // 2^11 = 2048 words
    parameter integer CFG_DEPTH      = 256,  // 256 words (1 KB CSR + EDT space)
    parameter integer CFG_ADDR_WIDTH = 8,    // 2^8 = 256 words
    parameter integer NUM_ENDPOINTS     = 4,    // 4 Writer + 4 Reader slots
    parameter integer REASM_DEPTH       = 1536, // 1536 words (6 KB) per reassembly slot
    parameter integer CLK_PERIOD_NS     = 10,   // Clock period in nanoseconds (e.g. 10 for 100MHz, 8 for 125MHz)
    parameter integer USE_EXT_TIMESTAMP = 0     // 1 = use ext_timestamp_ns input port, 0 = internal timer
)(
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire [63:0]               ext_timestamp_ns,

    // -------------------------------------------------------------------------
    // 1. AXI4-Lite Memory-Mapped Slave (CSRs + Endpoint Descriptor Table)
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
    // 2. AXI4-Stream Slave (DMA0 MM2S: PS Command Sample -> TX Sample BRAM)
    // -------------------------------------------------------------------------
    input  wire [DATA_WIDTH-1:0]     s_axis_tdata,
    input  wire [(DATA_WIDTH/8)-1:0] s_axis_tkeep,
    input  wire                      s_axis_tvalid,
    output wire                      s_axis_tready,
    input  wire                      s_axis_tlast,

    // -------------------------------------------------------------------------
    // 3. AXI4-Stream Master (DMA0 S2MM: RX Reassembly Slot -> PS Feedback Sample)
    // -------------------------------------------------------------------------
    output reg  [DATA_WIDTH-1:0]     m_axis_tdata,
    output wire [(DATA_WIDTH/8)-1:0] m_axis_tkeep,
    output reg                       m_axis_tvalid,
    input  wire                      m_axis_tready,
    output reg                       m_axis_tlast,

    // -------------------------------------------------------------------------
    // 4. AXI4-Stream Wire Port (DMA1 Virtual MAC / External RMII MAC)
    // -------------------------------------------------------------------------
    output wire [DATA_WIDTH-1:0]     m_axis_wire_tdata,
    output wire                      m_axis_wire_tvalid,
    input  wire                      m_axis_wire_tready,
    output wire                      m_axis_wire_tlast,

    input  wire [DATA_WIDTH-1:0]     s_axis_wire_tdata,
    input  wire                      s_axis_wire_tvalid,
    output wire                      s_axis_wire_tready,
    input  wire                      s_axis_wire_tlast,

    // -------------------------------------------------------------------------
    // 5. Dedicated Level-High Interrupt Wire to PS GIC (IRQ_F2P[2])
    // -------------------------------------------------------------------------
    output reg                       ip_done_irq
);

    // =========================================================================
    // AXI4-Lite Register Word Indices (s_axi_a*addr[CFG_ADDR_WIDTH+1:2])
    // =========================================================================
    localparam [CFG_ADDR_WIDTH-1:0]
        TRIGGER_ADDR        = 8'h00, // 0x00: Write forces FSM abort/reset to S_IDLE
        STATUS_ADDR         = 8'h01, // 0x04: Read status / Write ACK IRQ or kick
        CTRL_ADDR           = 8'h02, // 0x08: [0]=en, [3:1]=writers, [4]=emu_en, [5]=commit, [6]=wdog_en, [7]=plugin_rtps_rx_mode
        EXPECTED_CRC_ADDR   = 8'h03, // 0x0C: Write expected DMA0 CRC / Read calc CRC
        PACING_TIMER_ADDR   = 8'h04, // 0x10: Wire pacing delay cycles per beat
        CFG_PAYLOAD_ADDR    = 8'h05, // 0x14: Sample size in 32-bit words (default 1024 = 4096B)
        WDOG_PET_ADDR       = 8'h06, // 0x18: Write pets watchdog / Read remaining ticks
        WDOG_LIMIT_ADDR     = 8'h07, // 0x1C: Watchdog timeout ticks (default 10,000,000 = 100ms @ 100MHz)
        RX_TIMEOUT_ADDR     = 8'h08, // 0x20: RX feedback timeout ticks (default 90,000 = 900us @ 100MHz)
        LAST_CYCLE_TICKS    = 8'h09, // 0x24: Elapsed clock ticks in last RTPS cycle
        LOCAL_GUID_0_ADDR   = 8'h0A, // 0x28: Local Cyclone Participant GuidPrefix[95:64]
        LOCAL_GUID_1_ADDR   = 8'h0B, // 0x2C: Local Cyclone Participant GuidPrefix[63:32]
        LOCAL_GUID_2_ADDR   = 8'h0C, // 0x30: Local Cyclone Participant GuidPrefix[31:0]
        LOCAL_SRC_IP_ADDR   = 8'h0D, // 0x34: Dedicated secondary source IP for PL TX (Q16)
        LOCAL_MAC_LO_ADDR   = 8'h0E, // 0x38: Local MAC [31:0]
        LOCAL_MAC_HI_ADDR   = 8'h0F, // 0x3C: {VendorId[15:0], LocalMAC[47:32]}
        TS_OFFSET_LO_ADDR   = 8'h10, // 0x40: Epoch Timestamp Offset Lo
        TS_OFFSET_HI_ADDR   = 8'h11, // 0x44: Epoch Timestamp Offset Hi
        FAULT_INJECT_ADDR   = 8'h12, // 0x48: Emulator fault injection [0]=drop_frag1, [1]=swap_frag01, [2]=bad_udp_csum, [3]=stale_seq
        STAT_TX_SAMPLES     = 8'h13, // 0x4C: Total TX samples published
        STAT_RX_SAMPLES     = 8'h14, // 0x50: Total valid newer RX samples accepted
        STAT_SEQ_GAPS       = 8'h15, // 0x54: Cumulative seqNum gaps (lost samples)
        STAT_STALE_DROPS    = 8'h16, // 0x58: Duplicate/stale seqNum drops
        STAT_CSUM_ERRS      = 8'h17, // 0x5C: IPv4 or UDP checksum errors
        STAT_SKIPPED_SUBMSG = 8'h18, // 0x60: Unknown/vendor submessages skipped by RX walker
        LAST_RX_SEQ_LO      = 8'h19, // 0x64: Last accepted RX seqNum[31:0]
        LAST_RX_SEQ_HI      = 8'h1A, // 0x68: Last accepted RX seqNum[63:32]
        LAST_RX_TS_LO       = 8'h1B, // 0x6C: Last latched RX INFO_TS[31:0]
        LAST_RX_TS_HI       = 8'h1C, // 0x70: Last latched RX INFO_TS[63:32]
        LAST_RX_RTPS_WORDS  = 8'h1D, // 0x74: Word count of last accepted full RTPS message (e.g. 1039 = 4156B)
        CLK_PERIOD_ADDR     = 8'h1E, // 0x78: Clock period in nanoseconds (parameterized default)
        IDENTITY_CTRL_ADDR  = 8'h1F, // 0x7C: [0]=local_identity_valid, [1]=writer_context_valid, [2]=clear_seq
        GENERATION_ADDR     = 8'h60, // 0x180: Shadow writer generation number
        COMMIT_STATUS_ADDR  = 8'h61, // 0x184: Active writer generation number / commit status
        TYPE_PROFILE_ADDR   = 8'h62, // 0x188: Type profile & strict payload contract
        LAST_RX_SRC_IP_ADDR = 8'h63, // 0x18C: Last accepted RX source IPv4
        LAST_RX_PORTS_ADDR  = 8'h64, // 0x190: Last accepted RX {src_port[15:0], dst_port[15:0]}
        LAST_RX_META_ADDR   = 8'h65; // 0x194: Last accepted RX {24'd0, 6'd0, reader_slot[1:0]}

    // =========================================================================
    // Endpoint Descriptor Table (EDT): Shadow & Active Banks
    // Writer Entry i (i = 0..3), Word base = 0x20 + 8*i (Byte 0x80 .. 0xFF):
    //   +0: WR_CTRL[i]         ([0]=valid, [15:4]=payload_words, [27:16]=bram_offset)
    //   +1: WR_DST_IP[i]       (Remote RTI Peer IPv4 address)
    //   +2: WR_PORTS[i]        ({src_port[15:0], dst_port[15:0]})
    //   +3: WR_DST_MAC_LO[i]   (Dst MAC [31:0])
    //   +4: WR_DST_MAC_HI[i]   ({16'd0, Dst MAC [47:32]})
    //   +5: WR_ENTITY_ID[i]    (Cyclone Writer EntityId, e.g., 0x00000102)
    //   +6: WR_READER_ID[i]    (Target Reader EntityId, default 0x00000000 = ENTITYID_UNKNOWN)
    //   +7: WR_SEQ_SEED[i]     (Initial SequenceNumber[31:0] seed at commit)
    //
    // Reader CAM Entry j (j = 0..3), Word base = 0x40 + 8*j (Byte 0x100 .. 0x17F):
    //   +0: RD_CTRL[j]         ([0]=valid, [15:0]=expected_dst_port)
    //   +1: RD_GUID_0[j]       (Remote RTI Peer GuidPrefix[95:64])
    //   +2: RD_GUID_1[j]       (Remote RTI Peer GuidPrefix[63:32])
    //   +3: RD_GUID_2[j]       (Remote RTI Peer GuidPrefix[31:0])
    //   +4: RD_WRITER_ID[j]    (Remote RTI Feedback Writer EntityId)
    //   +5: RD_READER_ID[j]    (Local Cyclone Reader EntityId, or 0 for any)
    //   +6: RD_SRC_IP[j]       (Expected Remote Peer Source IPv4)
    //   +7: RD_STATUS[j]       (Read-only: matched rx sample count for slot j)
    // =========================================================================

    reg [31:0] shadow_wr_edt [0:NUM_ENDPOINTS-1][0:7];
    reg [31:0] active_wr_edt [0:NUM_ENDPOINTS-1][0:7];
    reg [31:0] shadow_rd_edt [0:NUM_ENDPOINTS-1][0:7];
    reg [31:0] active_rd_edt [0:NUM_ENDPOINTS-1][0:7];

    // Identity and Generation States (Rules Section 1 & 2)
    reg        local_identity_valid;
    reg        writer_context_valid;
    reg [31:0] active_generation;
    reg [31:0] shadow_generation;
    reg        edt_commit_applied;
    reg [31:0] active_wr_generation [0:NUM_ENDPOINTS-1];
    reg [31:0] shadow_wr_generation [0:NUM_ENDPOINTS-1];
    reg [31:0] type_profile_reg;
    reg        identity_clear_seq_pulse;

    // Per-writer sequence number counter (64-bit)
    reg [63:0] writer_seq_num [0:NUM_ENDPOINTS-1];
    reg [31:0] rd_match_count [0:NUM_ENDPOINTS-1];
    reg [63:0] last_rx_seq_num_by_cam [0:NUM_ENDPOINTS-1];
    reg [63:0] last_rx_ts_by_cam      [0:NUM_ENDPOINTS-1];

    // Global CSRs
    reg [31:0] ctrl_reg;
    reg [31:0] expected_crc;
    reg [31:0] cfg_pacing_timer;
    reg [31:0] cfg_payload_words;
    reg [31:0] wdog_counter;
    reg [31:0] wdog_limit;
    reg        wdog_expired;
    reg [31:0] rx_timeout_limit;
    reg [31:0] last_cycle_ticks;

    reg [95:0] local_guid_prefix;
    reg [31:0] local_src_ip;
    reg [47:0] local_mac;
    reg [15:0] local_vendor_id;
    reg [63:0] ts_offset;
    reg [63:0] free_running_ns;
    reg [15:0] cfg_clk_period_ns;
    wire [63:0] active_time_ns = (USE_EXT_TIMESTAMP) ? ext_timestamp_ns : free_running_ns;
    reg [31:0] fault_inject_reg;

    // Statistics & Diagnostics
    reg [31:0] stat_tx_samples;
    reg [31:0] stat_rx_samples;
    reg [31:0] stat_seq_gaps;
    reg [31:0] stat_stale_drops;
    reg [31:0] stat_csum_errs;
    reg [31:0] stat_skipped_submsg;
    reg [63:0] last_rx_seq_num;
    reg [63:0] last_rx_ts;
    reg [11:0] latest_rtps_words;
    reg [31:0] last_rx_src_ip;
    reg [15:0] last_rx_src_port;
    reg [15:0] last_rx_dst_port;
    reg [1:0]  last_rx_cam_idx;

    reg        status_write_pulse;
    reg [31:0] status_write_data;
    reg        trigger_detected;
    reg        edt_commit_pending;
    reg        wdog_pet_pulse;

    // =========================================================================
    // Master FSM States
    // =========================================================================
    localparam [3:0]
        S_IDLE         = 4'd0, // Awaiting DMA0 MM2S sample stream or manual start
        S_CHECK_CRC    = 4'd1, // Validate DMA0 CRC-32, check lifecycle/match, finalize UDP csum
        S_TX_FRAG_INIT = 4'd2, // Initialize current IPv4 fragment (0, 1, 2)
        S_TX_FRAG_SEND = 4'd3, // Stream Ethernet + IPv4 + UDP/RTPS fragment over wire port
        S_WAIT_RX_FB   = 4'd4, // Wait for RX reassembly + RTPS walker to latch newer sample (or timeout)
        S_DONE_IRQ     = 4'd5, // Assert Level-High IRQ to PS & wait for ACK
        S_WAIT_EGRESS  = 4'd6, // Wait for Driver read() kick
        S_TX_RUN       = 4'd7; // Stream Feedback sample from locked reassembly slot to DMA0 S2MM

    reg [3:0] state;

    // Error flags in STATUS_ADDR
    reg err_crc;        // Bit [5]: DMA0 MM2S CRC-32 mismatch
    reg err_no_match;   // Bit [6]: Lifecycle disabled, watchdog expired, or no valid writer descriptor
    reg err_rx_timeout; // Bit [7]: RX feedback sample did not arrive within rx_timeout_limit
    reg err_csum;       // Bit [8]: RX IPv4 or UDP checksum mismatch detected
    reg irq_is_rx;      // Bit [10]: 1 = ip_done_irq is for RX sample ready in BRAM, 0 = TX done / err

    // =========================================================================
    // 1's Complement Checksum & IEEE 802.3 CRC-32 Functions
    // =========================================================================
    // Checksum Utilities (Option 1: 32-bit Additive DMA Checksum + 1's Comp UDP)
    // =========================================================================
    function [15:0] csum_add32;
        input [15:0] sum_in;
        input [31:0] word_in;
        reg   [17:0] tmp;
        reg   [16:0] fold1;
        begin
            tmp   = {2'b00, sum_in} + {2'b00, word_in[31:16]} + {2'b00, word_in[15:0]};
            fold1 = {1'b0, tmp[15:0]} + {15'd0, tmp[17:16]};
            csum_add32 = fold1[15:0] + {15'd0, fold1[16]};
        end
    endfunction

    // =========================================================================
    // TX Command Sample BRAM (2048 x 32-bit = 8 KB) & On-the-Fly Checksums
    // =========================================================================
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] tx_sample_bram [0:BRAM_DEPTH-1];

    reg  [ADDR_WIDTH:0] rx_ptr;
    wire                mode_pub_only = ctrl_reg[8]; // Bit [8]: 1 = Publish-only mode (no RX feedback expected)
    wire                mode_sub_only = ctrl_reg[9]; // Bit [9]: 1 = Subscribe-only mode (unsolicited wire RX from S_IDLE)

    assign s_axis_tready = (state == S_IDLE) && !mode_sub_only;
    wire   rx_handshake  = s_axis_tvalid && s_axis_tready;
    wire   axis_wr       = rx_handshake && (state == S_IDLE);

    reg  [ADDR_WIDTH-1:0] tx_sample_rd_addr;
    reg  [DATA_WIDTH-1:0] tx_sample_dout;

    always @(posedge clk) begin
        if (axis_wr) begin
            tx_sample_bram[rx_ptr[ADDR_WIDTH-1:0]] <= s_axis_tdata;
        end
        tx_sample_dout <= tx_sample_bram[tx_sample_rd_addr];
    end

    // On-the-Fly 32-bit Additive Checksum & 1's Complement Payload Sum during MM2S Ingress
    reg [31:0] crc_accum;
    reg [15:0] payload_csum_accum;

    always @(posedge clk) begin
        if (!rst_n) begin
            crc_accum          <= 32'd0;
            payload_csum_accum <= 16'h0000;
        end else if (axis_wr) begin
            if (rx_ptr == 0) begin
                crc_accum          <= s_axis_tdata;
                payload_csum_accum <= csum_add32(16'h0000, s_axis_tdata);
            end else if (rx_ptr < cfg_payload_words) begin
                crc_accum          <= crc_accum + s_axis_tdata;
                payload_csum_accum <= csum_add32(payload_csum_accum, s_axis_tdata);
            end
        end
    end

    // =========================================================================
    // Wire Port Multiplexer: Internal Peer Emulator (Stage D) vs DMA1 (Stage C)
    // =========================================================================
    reg  [DATA_WIDTH-1:0] eng_tx_tdata;
    reg                   eng_tx_tvalid;
    wire                  eng_tx_tready;
    reg                   eng_tx_tlast;

    wire [DATA_WIDTH-1:0] eng_rx_tdata;
    wire                  eng_rx_tvalid;
    wire                  eng_rx_tready;
    wire                  eng_rx_tlast;

    wire [DATA_WIDTH-1:0] emu_rx_tdata;
    wire                  emu_rx_tvalid;
    wire                  emu_rx_tready;
    wire                  emu_rx_tlast;
    wire                  emu_tx_tready;

    wire                  emu_enable = ctrl_reg[4];

    assign m_axis_wire_tdata  = eng_tx_tdata;
    assign m_axis_wire_tvalid = emu_enable ? 1'b0 : eng_tx_tvalid;
    assign m_axis_wire_tlast  = eng_tx_tlast;
    assign eng_tx_tready      = emu_enable ? emu_tx_tready : m_axis_wire_tready;

    assign eng_rx_tdata       = emu_enable ? emu_rx_tdata  : s_axis_wire_tdata;
    assign eng_rx_tvalid      = emu_enable ? emu_rx_tvalid : s_axis_wire_tvalid;
    assign eng_rx_tlast       = emu_enable ? emu_rx_tlast  : s_axis_wire_tlast;
    assign emu_rx_tready      = emu_enable ? eng_rx_tready : 1'b0;
    assign s_axis_wire_tready = emu_enable ? 1'b0          : eng_rx_tready;

    rtps_peer_emulator #(
        .DATA_WIDTH  (DATA_WIDTH),
        .MAX_DGRAM_W (REASM_DEPTH)
    ) u_rtps_peer_emulator (
        .clk              (clk),
        .rst_n            (rst_n),
        .fault_ctrl       (fault_inject_reg),
        .peer_guid_prefix ({active_rd_edt[0][1], active_rd_edt[0][2], active_rd_edt[0][3]}),
        .peer_writer_id   (active_rd_edt[0][4]),
        .peer_reader_id   (active_rd_edt[0][5]),
        .s_axis_tdata     (eng_tx_tdata),
        .s_axis_tvalid    (emu_enable ? eng_tx_tvalid : 1'b0),
        .s_axis_tready    (emu_tx_tready),
        .s_axis_tlast     (eng_tx_tlast),
        .m_axis_tdata     (emu_rx_tdata),
        .m_axis_tvalid    (emu_rx_tvalid),
        .m_axis_tready    (emu_rx_tready),
        .m_axis_tlast     (emu_rx_tlast)
    );

    // =========================================================================
    // Free-Running Nanosecond Timestamp Counter & 100 ms Lifecycle Watchdog
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            free_running_ns <= 64'd1000_000_000;
            wdog_counter    <= 32'd0;
            wdog_expired    <= 1'b0;
        end else begin
            free_running_ns <= free_running_ns + {48'd0, cfg_clk_period_ns};
            if (wdog_pet_pulse) begin
                wdog_counter <= 32'd0;
                wdog_expired <= 1'b0;
            end else if (ctrl_reg[6] && ctrl_reg[0]) begin // Watchdog enabled & global_enable=1
                if (wdog_counter >= wdog_limit) begin
                    wdog_expired <= 1'b1;
                end else begin
                    wdog_counter <= wdog_counter + 32'd1;
                end
            end else begin
                wdog_counter <= 32'd0;
                wdog_expired <= 1'b0;
            end
        end
    end

    // =========================================================================
    // AXI4-Lite Slave Write Channel (CSRs & Shadow EDT) - Fully Decoupled (Section 9)
    // =========================================================================
    assign s_axi_bresp = 2'b00;
    assign s_axi_rresp = 2'b00;

    reg                      aw_hold_valid;
    reg [CFG_ADDR_WIDTH-1:0] aw_hold_addr;
    reg                      w_hold_valid;
    reg [DATA_WIDTH-1:0]     w_hold_data;

    wire aw_active   = aw_hold_valid || (s_axi_awvalid && s_axi_awready);
    wire w_active    = w_hold_valid  || (s_axi_wvalid  && s_axi_wready);
    wire reg_wr_fire = aw_active && w_active && !s_axi_bvalid;

    wire [CFG_ADDR_WIDTH-1:0] cur_wr_addr = aw_hold_valid ? aw_hold_addr : s_axi_awaddr[CFG_ADDR_WIDTH+1:2];
    wire [DATA_WIDTH-1:0]     cur_wr_data = w_hold_valid  ? w_hold_data  : s_axi_wdata;

    integer ep_i, w_i;
    always @(posedge clk) begin
        if (!rst_n) begin
            s_axi_awready            <= 1'b1;
            s_axi_wready             <= 1'b1;
            s_axi_bvalid             <= 1'b0;
            aw_hold_valid            <= 1'b0;
            aw_hold_addr             <= {CFG_ADDR_WIDTH{1'b0}};
            w_hold_valid             <= 1'b0;
            w_hold_data              <= 32'd0;
            status_write_pulse       <= 1'b0;
            status_write_data        <= 32'd0;
            trigger_detected         <= 1'b0;
            edt_commit_pending       <= 1'b0;
            edt_commit_applied       <= 1'b0;
            identity_clear_seq_pulse <= 1'b0;
            wdog_pet_pulse           <= 1'b0;

            local_identity_valid     <= 1'b0;
            writer_context_valid     <= 1'b0;
            active_generation        <= 32'd0;
            shadow_generation        <= 32'd0;
            type_profile_reg         <= 32'h0007_0101; // [18:16]=3'b111, [19]=0, [15:0]=0x0101

            // Default: global_enable=1, 1 writer, emulator enabled (bit 4), watchdog disabled by default
            ctrl_reg           <= 32'h0000_0013;
            expected_crc       <= 32'd0;
            cfg_pacing_timer   <= 32'd0;
            cfg_payload_words  <= 32'd1024;       // 1,024 words = 4,096 bytes (4 KB FastSample)
            wdog_limit         <= 32'd10_000_000; // 100 ms @ 100 MHz
            rx_timeout_limit   <= 32'd90_000;     // 900 us @ 100 MHz

            local_guid_prefix  <= 96'h0110_CD01_2345_6789_ABCD_EF01;
            local_src_ip       <= 32'hC0A8_01C9; // 192.168.1.201 (Dedicated PL TX alias IP, Q16)
            local_mac          <= 48'h0200_5E10_0001;
            local_vendor_id    <= 16'h0101;      // RTI Connext (0x0101) default
            ts_offset          <= 64'h6530_0000_0000_0000;
            fault_inject_reg   <= 32'd0;
            cfg_clk_period_ns  <= CLK_PERIOD_NS[15:0];

            // Initialize Shadow & Active EDT Entry 0 with ready-to-run defaults
            for (ep_i = 0; ep_i < NUM_ENDPOINTS; ep_i = ep_i + 1) begin
                active_wr_generation[ep_i] <= 32'd0;
                shadow_wr_generation[ep_i] <= 32'd0;
                for (w_i = 0; w_i < 8; w_i = w_i + 1) begin
                    shadow_wr_edt[ep_i][w_i] <= 32'd0;
                    active_wr_edt[ep_i][w_i] <= 32'd0;
                    shadow_rd_edt[ep_i][w_i] <= 32'd0;
                    active_rd_edt[ep_i][w_i] <= 32'd0;
                end
            end

            // Default Writer 0 (Command Topic -> RTI Peer 192.168.1.100:7410)
            shadow_wr_edt[0][0] <= {16'd0, 12'd1024, 3'd2, 1'b1}; // valid=1, state=ACTIVE(2), 1024 words (4096B)
            shadow_wr_edt[0][1] <= 32'hC0A8_0164;                 // 192.168.1.100
            shadow_wr_edt[0][2] <= {16'd7411, 16'd7410};          // src_port=7411, dst_port=7410
            shadow_wr_edt[0][3] <= 32'h5E20_0002;                 // Dst MAC Lo
            shadow_wr_edt[0][4] <= 32'h0000_0200;                 // Dst MAC Hi
            shadow_wr_edt[0][5] <= 32'h0000_0102;                 // Writer EntityId
            shadow_wr_edt[0][6] <= 32'h0000_0000;                 // Reader EntityId = ENTITYID_UNKNOWN
            shadow_wr_edt[0][7] <= 32'h0000_0001;                 // Initial seqNum seed = 1

            active_wr_edt[0][0] <= {16'd0, 12'd1024, 3'd2, 1'b1};
            active_wr_edt[0][1] <= 32'hC0A8_0164;
            active_wr_edt[0][2] <= {16'd7411, 16'd7410};
            active_wr_edt[0][3] <= 32'h5E20_0002;
            active_wr_edt[0][4] <= 32'h0000_0200;
            active_wr_edt[0][5] <= 32'h0000_0102;
            active_wr_edt[0][6] <= 32'h0000_0000;
            active_wr_edt[0][7] <= 32'h0000_0001;

            // Default Reader CAM 0 (Feedback Topic <- RTI Peer 192.168.1.100)
            shadow_rd_edt[0][0] <= {16'd7411, 12'd0, 3'd2, 1'b1}; // valid=1, state=ACTIVE(2), dst_port=7411
            shadow_rd_edt[0][1] <= 32'h0101_AA00;                 // RTI Peer GuidPrefix[95:64]
            shadow_rd_edt[0][2] <= 32'hBB11_CC22;                 // RTI Peer GuidPrefix[63:32]
            shadow_rd_edt[0][3] <= 32'hDD33_EE44;                 // RTI Peer GuidPrefix[31:0]
            shadow_rd_edt[0][4] <= 32'h0000_0202;                 // RTI Feedback Writer EntityId
            shadow_rd_edt[0][5] <= 32'h0000_0107;                 // Local Reader EntityId
            shadow_rd_edt[0][6] <= 32'hC0A8_0164;                 // Expected SrcIP 192.168.1.100

            active_rd_edt[0][0] <= {16'd7411, 12'd0, 3'd2, 1'b1};
            active_rd_edt[0][1] <= 32'h0101_AA00;
            active_rd_edt[0][2] <= 32'hBB11_CC22;
            active_rd_edt[0][3] <= 32'hDD33_EE44;
            active_rd_edt[0][4] <= 32'h0000_0202;
            active_rd_edt[0][5] <= 32'h0000_0107;
            active_rd_edt[0][6] <= 32'hC0A8_0164;
        end else begin
            status_write_pulse       <= 1'b0;
            trigger_detected         <= 1'b0;
            wdog_pet_pulse           <= 1'b0;
            edt_commit_applied       <= 1'b0;
            identity_clear_seq_pulse <= 1'b0;

            // Independent AW address capture
            if (!aw_hold_valid) begin
                if (s_axi_awvalid && s_axi_awready) begin
                    if (!w_active || s_axi_bvalid) begin
                        aw_hold_valid <= 1'b1;
                        aw_hold_addr  <= s_axi_awaddr[CFG_ADDR_WIDTH+1:2];
                        s_axi_awready <= 1'b0;
                    end
                end else begin
                    s_axi_awready <= !s_axi_bvalid;
                end
            end

            // Independent W data capture
            if (!w_hold_valid) begin
                if (s_axi_wvalid && s_axi_wready) begin
                    if (!aw_active || s_axi_bvalid) begin
                        w_hold_valid <= 1'b1;
                        w_hold_data  <= s_axi_wdata;
                        s_axi_wready <= 1'b0;
                    end
                end else begin
                    s_axi_wready <= !s_axi_bvalid;
                end
            end

            // Atomic Shadow -> Active EDT commit when FSM is in S_IDLE (Section 2)
            if (edt_commit_pending && (state == S_IDLE)) begin
                edt_commit_pending <= 1'b0;
                edt_commit_applied <= 1'b1;
                active_generation  <= shadow_generation;
                for (ep_i = 0; ep_i < NUM_ENDPOINTS; ep_i = ep_i + 1) begin
                    for (w_i = 0; w_i < 8; w_i = w_i + 1) begin
                        active_wr_edt[ep_i][w_i] <= shadow_wr_edt[ep_i][w_i];
                        active_rd_edt[ep_i][w_i] <= shadow_rd_edt[ep_i][w_i];
                    end
                    active_wr_generation[ep_i] <= shadow_wr_generation[ep_i];
                end
            end

            if (reg_wr_fire) begin
                s_axi_bvalid  <= 1'b1;
                aw_hold_valid <= 1'b0;
                w_hold_valid  <= 1'b0;
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;

                if (cur_wr_addr >= 8'h20 && cur_wr_addr < 8'h40) begin
                    // Shadow Writer EDT (Words 0x20..0x3F -> Byte 0x80..0xFF)
                    shadow_wr_edt[(cur_wr_addr - 8'h20) >> 3][(cur_wr_addr - 8'h20) & 3'd7] <= cur_wr_data;
                end else if (cur_wr_addr >= 8'h40 && cur_wr_addr < 8'h60) begin
                    // Shadow Reader CAM EDT (Words 0x40..0x5F -> Byte 0x100..0x17F)
                    shadow_rd_edt[(cur_wr_addr - 8'h40) >> 3][(cur_wr_addr - 8'h40) & 3'd7] <= cur_wr_data;
                end else begin
                    case (cur_wr_addr)
                        TRIGGER_ADDR:      trigger_detected   <= 1'b1;
                        STATUS_ADDR: begin
                            status_write_pulse <= 1'b1;
                            status_write_data  <= cur_wr_data;
                        end
                        CTRL_ADDR: begin
                            ctrl_reg <= cur_wr_data;
                            if (cur_wr_data[5]) begin
                                edt_commit_pending <= 1'b1;
                            end
                        end
                        EXPECTED_CRC_ADDR: expected_crc       <= cur_wr_data;
                        PACING_TIMER_ADDR: cfg_pacing_timer   <= {16'd0, cur_wr_data[15:0]};
                        CFG_PAYLOAD_ADDR:  cfg_payload_words  <= (cur_wr_data == 0 || cur_wr_data > BRAM_DEPTH) ?
                                                                 BRAM_DEPTH : cur_wr_data;
                        WDOG_PET_ADDR:     wdog_pet_pulse     <= 1'b1;
                        WDOG_LIMIT_ADDR:   wdog_limit         <= cur_wr_data;
                        RX_TIMEOUT_ADDR:   rx_timeout_limit   <= cur_wr_data;
                        LOCAL_GUID_0_ADDR: local_guid_prefix[95:64] <= cur_wr_data;
                        LOCAL_GUID_1_ADDR: local_guid_prefix[63:32] <= cur_wr_data;
                        LOCAL_GUID_2_ADDR: local_guid_prefix[31:0]  <= cur_wr_data;
                        LOCAL_SRC_IP_ADDR: local_src_ip       <= cur_wr_data;
                        LOCAL_MAC_LO_ADDR: local_mac[31:0]    <= cur_wr_data;
                        LOCAL_MAC_HI_ADDR: begin
                            local_mac[47:32] <= cur_wr_data[15:0];
                            local_vendor_id  <= (cur_wr_data[31:16] == 16'd0) ? 16'h0101 : cur_wr_data[31:16];
                        end
                        TS_OFFSET_LO_ADDR: ts_offset[31:0]    <= cur_wr_data;
                        TS_OFFSET_HI_ADDR: ts_offset[63:32]   <= cur_wr_data;
                        FAULT_INJECT_ADDR: fault_inject_reg   <= cur_wr_data;
                        CLK_PERIOD_ADDR:   cfg_clk_period_ns  <= cur_wr_data[15:0];
                        IDENTITY_CTRL_ADDR: begin
                            local_identity_valid <= cur_wr_data[0];
                            writer_context_valid <= cur_wr_data[1];
                            if (cur_wr_data[2]) begin
                                identity_clear_seq_pulse <= 1'b1;
                            end
                        end
                        GENERATION_ADDR: begin
                            shadow_generation <= cur_wr_data;
                            for (ep_i = 0; ep_i < NUM_ENDPOINTS; ep_i = ep_i + 1) begin
                                shadow_wr_generation[ep_i] <= cur_wr_data;
                            end
                        end
                        TYPE_PROFILE_ADDR: begin
                            type_profile_reg <= cur_wr_data;
                        end
                        default: ;
                    endcase
                end
            end

            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid  <= 1'b0;
                s_axi_awready <= !aw_hold_valid;
                s_axi_wready  <= !w_hold_valid;
            end
        end
    end

    // =========================================================================
    // AXI4-Lite Slave Read Channel
    // =========================================================================
    wire [CFG_ADDR_WIDTH-1:0] rd_word_idx = s_axi_araddr[CFG_ADDR_WIDTH+1:2];
    reg  [3:0]                completed_frags;

    wire payload_contract_ok =
        type_profile_reg[16] && // serialized CDR/XCDR
        type_profile_reg[17] && // little-endian representation
        type_profile_reg[18] && // fixed-size payload required
        !type_profile_reg[19];  // unkeyed topic

    wire wr0_valid        = active_wr_edt[0][0][0];
    wire any_reader_valid = active_rd_edt[0][0][0] || active_rd_edt[1][0][0] ||
                            active_rd_edt[2][0][0] || active_rd_edt[3][0][0];

    wire [2:0] wr0_lifecycle_state = active_wr_edt[0][0][3:1];
    wire wr0_lifecycle_active      = (wr0_lifecycle_state == 3'd2);
    wire wr0_route_valid           = (wr0_dst_ip != 32'd0) && (wr0_dst_port != 16'd0) && (wr0_dst_mac != 48'd0);
    wire generation_match          = (shadow_generation == 32'd0) || (active_generation == shadow_generation);

    wire lifecycle_ok     = ctrl_reg[0] &&
                            !wdog_expired &&
                            local_identity_valid &&
                            payload_contract_ok &&
                            generation_match &&
                            (mode_sub_only ? any_reader_valid :
                                              (wr0_valid && wr0_lifecycle_active && wr0_route_valid && writer_context_valid));

    always @(posedge clk) begin
        if (!rst_n) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= 32'd0;
        end else begin
            if (!s_axi_arready && s_axi_arvalid && !s_axi_rvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rvalid  <= 1'b1;

                if (rd_word_idx >= 8'h20 && rd_word_idx < 8'h40) begin
                    s_axi_rdata <= active_wr_edt[(rd_word_idx - 8'h20) >> 3][(rd_word_idx - 8'h20) & 3'd7];
                end else if (rd_word_idx >= 8'h40 && rd_word_idx < 8'h60) begin
                    if (((rd_word_idx - 8'h40) & 3'd7) == 3'd7)
                        s_axi_rdata <= rd_match_count[(rd_word_idx - 8'h40) >> 3];
                    else
                        s_axi_rdata <= active_rd_edt[(rd_word_idx - 8'h40) >> 3][(rd_word_idx - 8'h40) & 3'd7];
                end else begin
                    case (rd_word_idx)
                        STATUS_ADDR: begin
                            s_axi_rdata <= {
                                stat_rx_samples[15:0], // [31:16] Total accepted RX samples
                                lifecycle_ok,          // [15]    Lifecycle & Writer[0] valid
                                wdog_expired,          // [14]    Watchdog expired flag
                                completed_frags[2:0],  // [13:11] TX fragments sent in cycle
                                irq_is_rx,             // [10]    1 = RX sample ready in BRAM, 0 = TX done / err
                                mode_pub_only,         // [9]     Publish-only mode active
                                err_csum,              // [8]     RX IPv4/UDP checksum error
                                err_rx_timeout,        // [7]     RX sample timeout error
                                err_no_match,          // [6]     No valid endpoint / watchdog disabled
                                err_crc,               // [5]     DMA0 MM2S CRC-32 error
                                state,                 // [4:1]   Master FSM state
                                ip_done_irq            // [0]     Level-High IRQ status
                            };
                        end
                        CTRL_ADDR:           s_axi_rdata <= ctrl_reg;
                        EXPECTED_CRC_ADDR:   s_axi_rdata <= crc_accum;
                        PACING_TIMER_ADDR:   s_axi_rdata <= cfg_pacing_timer;
                        CFG_PAYLOAD_ADDR:    s_axi_rdata <= cfg_payload_words;
                        WDOG_PET_ADDR:       s_axi_rdata <= wdog_limit - wdog_counter;
                        WDOG_LIMIT_ADDR:     s_axi_rdata <= wdog_limit;
                        RX_TIMEOUT_ADDR:     s_axi_rdata <= rx_timeout_limit;
                        LAST_CYCLE_TICKS:    s_axi_rdata <= last_cycle_ticks;
                        LOCAL_GUID_0_ADDR:   s_axi_rdata <= local_guid_prefix[95:64];
                        LOCAL_GUID_1_ADDR:   s_axi_rdata <= local_guid_prefix[63:32];
                        LOCAL_GUID_2_ADDR:   s_axi_rdata <= local_guid_prefix[31:0];
                        LOCAL_SRC_IP_ADDR:   s_axi_rdata <= local_src_ip;
                        LOCAL_MAC_LO_ADDR:   s_axi_rdata <= local_mac[31:0];
                        LOCAL_MAC_HI_ADDR:   s_axi_rdata <= {local_vendor_id, local_mac[47:32]};
                        TS_OFFSET_LO_ADDR:   s_axi_rdata <= ts_offset[31:0];
                        TS_OFFSET_HI_ADDR:   s_axi_rdata <= ts_offset[63:32];
                        FAULT_INJECT_ADDR:   s_axi_rdata <= fault_inject_reg;
                        STAT_TX_SAMPLES:     s_axi_rdata <= stat_tx_samples;
                        STAT_RX_SAMPLES:     s_axi_rdata <= stat_rx_samples;
                        STAT_SEQ_GAPS:       s_axi_rdata <= stat_seq_gaps;
                        STAT_STALE_DROPS:    s_axi_rdata <= stat_stale_drops;
                        STAT_CSUM_ERRS:      s_axi_rdata <= stat_csum_errs;
                        STAT_SKIPPED_SUBMSG: s_axi_rdata <= stat_skipped_submsg;
                        LAST_RX_SEQ_LO:      s_axi_rdata <= last_rx_seq_num[31:0];
                        LAST_RX_SEQ_HI:      s_axi_rdata <= last_rx_seq_num[63:32];
                        LAST_RX_TS_LO:       s_axi_rdata <= last_rx_ts[31:0];
                        LAST_RX_TS_HI:       s_axi_rdata <= last_rx_ts[63:32];
                        LAST_RX_RTPS_WORDS:  s_axi_rdata <= {20'd0, latest_rtps_words};
                        CLK_PERIOD_ADDR:     s_axi_rdata <= {16'd0, cfg_clk_period_ns};
                        IDENTITY_CTRL_ADDR:  s_axi_rdata <= {29'd0, 1'b0, writer_context_valid, local_identity_valid};
                        GENERATION_ADDR:     s_axi_rdata <= shadow_generation;
                        COMMIT_STATUS_ADDR:  s_axi_rdata <= active_generation;
                        TYPE_PROFILE_ADDR:   s_axi_rdata <= type_profile_reg;
                        LAST_RX_SRC_IP_ADDR: s_axi_rdata <= last_rx_src_ip;
                        LAST_RX_PORTS_ADDR:  s_axi_rdata <= {last_rx_src_port, last_rx_dst_port};
                        LAST_RX_META_ADDR:   s_axi_rdata <= {24'd0, 6'd0, last_rx_cam_idx};
                        default:             s_axi_rdata <= 32'd0;
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
    // 3-Slot Store-and-Forward IPv4 Reassembly + Generic RTPS Submessage Walker
    //
    // Why 3 Slots? (per Design Decisions Section 5 Note):
    //   Slot A: Currently filling with incoming IPv4 fragments from the wire.
    //   Slot B: Holds the most recently validated 'latest' sample ready for PS.
    //   Slot C: Locked by an in-flight S2MM DMA read (S_TX_RUN).
    // =========================================================================
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] reasm_ram_0 [0:REASM_DEPTH-1];
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] reasm_ram_1 [0:REASM_DEPTH-1];
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] reasm_ram_2 [0:REASM_DEPTH-1];

    reg [1:0]  fill_slot;         // Slot currently used by RX reassembly (0, 1, 2)
    reg [1:0]  latest_slot;       // Slot holding the newest validated sample
    reg [1:0]  egress_slot;       // Slot locked during S_WAIT_EGRESS / S_TX_RUN
    reg        latest_valid;      // 1 = a new valid feedback sample is ready

    reg        reasm_wr_en;
    reg [11:0] reasm_wr_addr;
    reg [31:0] reasm_wr_data;

    reg [11:0] walker_rd_addr;
    reg [11:0] s2mm_rd_addr;

    reg [31:0] reasm_dout_0, reasm_dout_1, reasm_dout_2;
    wire [11:0] slot0_rd_addr = (state == S_TX_RUN && egress_slot == 2'd0) ? s2mm_rd_addr : walker_rd_addr;
    wire [11:0] slot1_rd_addr = (state == S_TX_RUN && egress_slot == 2'd1) ? s2mm_rd_addr : walker_rd_addr;
    wire [11:0] slot2_rd_addr = (state == S_TX_RUN && egress_slot == 2'd2) ? s2mm_rd_addr : walker_rd_addr;

    always @(posedge clk) begin
        if (reasm_wr_en) begin
            case (fill_slot)
                2'd0: reasm_ram_0[reasm_wr_addr] <= reasm_wr_data;
                2'd1: reasm_ram_1[reasm_wr_addr] <= reasm_wr_data;
                default: reasm_ram_2[reasm_wr_addr] <= reasm_wr_data;
            endcase
        end
        reasm_dout_0 <= reasm_ram_0[slot0_rd_addr];
        reasm_dout_1 <= reasm_ram_1[slot1_rd_addr];
        reasm_dout_2 <= reasm_ram_2[slot2_rd_addr];
    end

    wire [31:0] walker_dout = (fill_slot == 2'd0) ? reasm_dout_0 :
                              (fill_slot == 2'd1) ? reasm_dout_1 : reasm_dout_2;

    wire [31:0] s2mm_dout   = (egress_slot == 2'd0) ? reasm_dout_0 :
                              (egress_slot == 2'd1) ? reasm_dout_1 : reasm_dout_2;

    // Choose next free reassembly slot (avoiding latest_slot and egress_slot)
    function [1:0] pick_free_slot;
        input [1:0] busy_a; // fill slot
        input [1:0] busy_b; // egress slot
        begin
            if (busy_a != 2'd0 && busy_b != 2'd0)
                pick_free_slot = 2'd0;
            else if (busy_a != 2'd1 && busy_b != 2'd1)
                pick_free_slot = 2'd1;
            else
                pick_free_slot = 2'd2;
        end
    endfunction

    // =========================================================================
    // RX Reassembly & RTPS Submessage Walker FSM
    // =========================================================================
    localparam [4:0]
        R_IDLE            = 5'd0,  // Receive IPv4 fragments into fill_slot
        R_CHECK_UDP       = 5'd1,  // Verify reassembled UDP checksum & prime RTPS header read
        R_RTPS_HDR_0      = 5'd2,  // Check "RTPS" magic (Word 2)
        R_RTPS_HDR_1      = 5'd3,  // Check version == 2.x (Word 3)
        R_RTPS_GUID_0     = 5'd4,  // Latch Remote GuidPrefix[95:64] (Word 4)
        R_RTPS_GUID_1     = 5'd5,  // Latch Remote GuidPrefix[63:32] (Word 5)
        R_RTPS_GUID_2     = 5'd6,  // Latch Remote GuidPrefix[31:0] (Word 6) & CAM match
        R_SUBMSG_HDR      = 5'd7,  // Inspect Submessage Header {submsgId, flags, octetsToNext}
        R_SUBMSG_HDR_WAIT = 5'd8,  // 1-cycle BRAM read wait for next Submessage Header
        R_INFO_TS_WAIT    = 5'd9,  // 1-cycle BRAM read wait for INFO_TS Word +1
        R_INFO_TS_0       = 5'd10, // Latch INFO_TS seconds
        R_INFO_TS_1       = 5'd11, // Latch INFO_TS fraction & advance to next submessage
        R_DATA_HDR_1      = 5'd12, // 1-cycle BRAM read wait for DATA Word +3 (writerEntityId)
        R_DATA_HDR_2      = 5'd13, // Check writerEntityId against matched CAM slot
        R_DATA_SEQ_HI     = 5'd14, // Latch writerSeqNum[63:32]
        R_DATA_SEQ_LO     = 5'd15, // Latch writerSeqNum[31:0] & branch on Q-flag (inline QoS)
        R_SKIP_QOS_WAIT   = 5'd16, // 1-cycle BRAM read wait for Inline QoS parameter header
        R_SKIP_QOS        = 5'd17, // Walk inline QoS parameter list until PID_SENTINEL (0x0001)
        R_COMMIT_SAMPLE   = 5'd18, // Registered commit state (breaks BRAM-to-CE critical path)
        R_INFO_DST_WAIT   = 5'd19, // 1-cycle BRAM wait for INFO_DST Word +1
        R_INFO_DST_0      = 5'd20, // Latch INFO_DST GuidPrefix[95:64]
        R_INFO_DST_1      = 5'd21, // Latch INFO_DST GuidPrefix[63:32]
        R_INFO_DST_2      = 5'd22, // Latch INFO_DST GuidPrefix[31:0]
        R_DATA_READER_ID  = 5'd23, // Check DATA readerEntityId against local reader / ENTITYID_UNKNOWN
        R_REASM_CHECK     = 5'd24; // 1-cycle registered evaluation of reassembly coverage (breaks 768-to-1 MUX critical path)

    reg [4:0]  rx_state;
    reg [11:0] rx_wire_beat;
    reg [15:0] rx_ip_len;
    reg [15:0] rx_ip_id;
    reg [15:0] rx_active_ip_id;
    reg        rx_active_valid;
    reg [15:0] rx_flags_frag;
    reg [15:0] rx_hdr_csum_accum;
    reg [31:0] rx_src_ip;
    reg [31:0] rx_dst_ip;
    reg [15:0] rx_prev_half;
    reg [11:0] rx_wr_ptr;
    reg [15:0] rx_bytes_rcvd;
    reg [15:0] rx_bytes_total;
    reg        rx_saw_last;
    reg [15:0] rx_udp_csum_accum;
    reg [15:0] rx_udp_len_bytes;
    reg [15:0] rx_udp_src_port;
    reg [15:0] rx_udp_dst_port;

    // Fragment Coverage Bitmap per Reassembly Slot (Section 6)
    localparam integer REASM_MAX_BYTES   = REASM_DEPTH * 4; // 1536 * 4 = 6144 bytes
    localparam integer REASM_BLOCK_BYTES = 8;
    localparam integer REASM_BLOCKS      = REASM_MAX_BYTES / REASM_BLOCK_BYTES; // 768 blocks

    reg [REASM_BLOCKS-1:0] reasm_coverage [0:2];
    reg [9:0]              reasm_blocks_covered [0:2];
    reg [15:0]             reasm_total_bytes [0:2];
    reg                    reasm_last_seen [0:2];

    // Pipelined RX UDP Pseudo-Header Checksum Engine:
    // Pipelined across 3 stages as headers stream in:
    // - Stage 1 (Beat 8): Pre-sums IP addresses and UDP protocol (0x0011).
    // - Stage 2 (Beat 10): Adds latched UDP datagram length (rx_udp_len_bytes).
    // - Stage 3 (Beat 11): Latches 1's complement folded 16-bit pseudo-header checksum.
    // By Beat 12 (thousands of clock cycles before reassembly completes), rx_pseudo_csum
    // is 100% valid in flip-flops, completely breaking the 31-level CARRY4 timing path.
    reg [19:0] rx_ip_pseudo_sum;
    reg [19:0] rx_pseudo_sum_pipe;
    reg [15:0] rx_pseudo_csum;
    wire [16:0] rx_pseudo_fold1 = {1'b0, rx_pseudo_sum_pipe[15:0]} + {13'd0, rx_pseudo_sum_pipe[19:16]};
    wire [15:0] rx_pseudo_fold2 = rx_pseudo_fold1[15:0] + {15'd0, rx_pseudo_fold1[16]};

    always @(posedge clk) begin
        if (!rst_n) begin
            rx_ip_pseudo_sum   <= 20'd0;
            rx_pseudo_sum_pipe <= 20'd0;
            rx_pseudo_csum     <= 16'd0;
        end else begin
            // Stage 1: Sum of IP addresses + UDP protocol (0x0011)
            rx_ip_pseudo_sum   <= {4'd0, rx_src_ip[31:16]} + {4'd0, rx_src_ip[15:0]} +
                                  {4'd0, rx_dst_ip[31:16]} + {4'd0, rx_dst_ip[15:0]} +
                                  20'h0011;
            // Stage 2: Add UDP length
            rx_pseudo_sum_pipe <= rx_ip_pseudo_sum + {4'd0, rx_udp_len_bytes};
            // Stage 3: Fold into 16 bits (1's complement)
            rx_pseudo_csum     <= rx_pseudo_fold2;
        end
    end

    // UDP Checksum validation fold evaluated in R_CHECK_UDP:
    // Combines accumulated payload checksum and pre-folded pseudo-header checksum.
    wire [16:0] rx_chk_fold1 = {1'b0, rx_udp_csum_accum} + {1'b0, rx_pseudo_csum};
    wire [15:0] rx_chk_final = rx_chk_fold1[15:0] + {15'd0, rx_chk_fold1[16]};
    wire        rx_chk_valid = (&rx_chk_fold1[15:1]) && (rx_chk_fold1[0] ^ rx_chk_fold1[16]);

    wire [12:0] rx_frag_off_8b  = rx_flags_frag[12:0];
    wire        rx_frag_mf      = rx_flags_frag[13];
    wire [15:0] rx_ip_pay_bytes = rx_ip_len - 16'd20;
    wire [11:0] rx_frag_w_start = {rx_frag_off_8b[10:0], 1'b0};

    assign eng_rx_tready = (rx_state == R_IDLE);
    wire   eng_rx_fire   = eng_rx_tvalid && eng_rx_tready;

    // Walker registers
    reg [95:0] walk_guid_prefix;
    reg [1:0]  walk_cam_idx;
    reg        walk_cam_hit;
    reg [11:0] walk_cur_word;
    reg [11:0] walk_dgram_words;
    reg [7:0]  walk_submsg_id;
    reg [7:0]  walk_submsg_flags;
    reg [15:0] walk_octets_next;
    reg [11:0] walk_next_submsg_word;
    reg [63:0] walk_ts;
    reg [31:0] walk_writer_id;
    reg [63:0] walk_seq_num;
    reg [11:0] latest_payload_word_off;

    // INFO_DST Participant Filtering (Section 4)
    reg [95:0] walk_dst_guid_prefix;
    reg        walk_dst_valid;
    wire       info_dst_matches_local = !walk_dst_valid ||
                                        (walk_dst_guid_prefix == 96'd0) ||
                                        (walk_dst_guid_prefix == local_guid_prefix);

    // 4-Entry Reader CAM Match on GuidPrefix, Lifecycle, Expected Port & Expected Source IP
    // Optimization: walker_dout[31:0] is the last word of the RTPS header (GuidPrefix[31:0]), so we can compare all 3 words in a single cycle instead of waiting for 1 cycle to populate the 3rd word of walk_guid_prefix.
    wire rd0_active = active_rd_edt[0][0][0] && (active_rd_edt[0][0][3:1] == 3'd2);
    wire rd1_active = active_rd_edt[1][0][0] && (active_rd_edt[1][0][3:1] == 3'd2);
    wire rd2_active = active_rd_edt[2][0][0] && (active_rd_edt[2][0][3:1] == 3'd2);
    wire rd3_active = active_rd_edt[3][0][0] && (active_rd_edt[3][0][3:1] == 3'd2);

    wire rd0_port_ok = (active_rd_edt[0][0][31:16] == 16'd0) || (rx_udp_dst_port == active_rd_edt[0][0][31:16]);
    wire rd1_port_ok = (active_rd_edt[1][0][31:16] == 16'd0) || (rx_udp_dst_port == active_rd_edt[1][0][31:16]);
    wire rd2_port_ok = (active_rd_edt[2][0][31:16] == 16'd0) || (rx_udp_dst_port == active_rd_edt[2][0][31:16]);
    wire rd3_port_ok = (active_rd_edt[3][0][31:16] == 16'd0) || (rx_udp_dst_port == active_rd_edt[3][0][31:16]);

    wire rd0_ip_ok   = (active_rd_edt[0][6] == 32'd0) || (rx_src_ip == active_rd_edt[0][6]);
    wire rd1_ip_ok   = (active_rd_edt[1][6] == 32'd0) || (rx_src_ip == active_rd_edt[1][6]);
    wire rd2_ip_ok   = (active_rd_edt[2][6] == 32'd0) || (rx_src_ip == active_rd_edt[2][6]);
    wire rd3_ip_ok   = (active_rd_edt[3][6] == 32'd0) || (rx_src_ip == active_rd_edt[3][6]);

    wire cam_hit_0 = rd0_active && rd0_port_ok && rd0_ip_ok && ({active_rd_edt[0][1], active_rd_edt[0][2], active_rd_edt[0][3]} == {walk_guid_prefix[95:32], walker_dout});
    wire cam_hit_1 = rd1_active && rd1_port_ok && rd1_ip_ok && ({active_rd_edt[1][1], active_rd_edt[1][2], active_rd_edt[1][3]} == {walk_guid_prefix[95:32], walker_dout});
    wire cam_hit_2 = rd2_active && rd2_port_ok && rd2_ip_ok && ({active_rd_edt[2][1], active_rd_edt[2][2], active_rd_edt[2][3]} == {walk_guid_prefix[95:32], walker_dout});
    wire cam_hit_3 = rd3_active && rd3_port_ok && rd3_ip_ok && ({active_rd_edt[3][1], active_rd_edt[3][2], active_rd_edt[3][3]} == {walk_guid_prefix[95:32], walker_dout});

    // Helper for endian-aware 16-bit extraction
    wire        submsg_e_flag = walker_dout[16]; // Bit 0 of flags byte (walker_dout[23:16])
    wire [15:0] submsg_octets = submsg_e_flag ? {walker_dout[7:0], walker_dout[15:8]} : walker_dout[15:0];

    // =========================================================================
    // Generic TX Fragmentation Planner Signals (Section 7)
    // =========================================================================
    localparam integer ETH_MTU_BYTES        = 1500;
    localparam integer IPV4_HDR_BYTES       = 20;
    localparam integer IPV4_PAYLOAD_MAX     = ETH_MTU_BYTES - IPV4_HDR_BYTES; // 1480
    localparam integer IPV4_NONFINAL_PAYLOAD = (IPV4_PAYLOAD_MAX / 8) * 8;     // 1480

    reg [ADDR_WIDTH:0] pkt_len;
    reg [15:0]         pacing_timer;
    reg [31:0]         cycle_tick_cnt;
    reg [31:0]         rx_wait_cnt;

    reg [15:0]         tx_ip_id_counter;
    reg [63:0]         latched_tx_ts;
    reg [63:0]         cur_tx_seq;
    reg [15:0]         tx_udp_csum;
    reg [1:0]          tx_frag_idx;
    reg [11:0]         tx_frag_words;
    reg [15:0]         tx_frag_ip_len;
    reg [15:0]         tx_frag_flags;
    reg [15:0]         tx_frag_hdr_csum;
    reg [11:0]         tx_dgram_word_idx;
    reg [11:0]         tx_beat_idx;
    reg [15:0]         tx_prev_half;

    reg [15:0]         tx_frag_payload_bytes;
    reg [15:0]         tx_frag_offset_bytes;
    reg [15:0]         tx_bytes_remaining;
    reg                tx_frag_more;

    // Latch active Writer[0] fields
    wire [31:0] wr0_dst_ip     = active_wr_edt[0][1];
    wire [15:0] wr0_src_port   = active_wr_edt[0][2][31:16];
    wire [15:0] wr0_dst_port   = active_wr_edt[0][2][15:0];
    wire [47:0] wr0_dst_mac    = {active_wr_edt[0][4][15:0], active_wr_edt[0][3]};
    wire [31:0] wr0_writer_id  = active_wr_edt[0][5];
    wire [31:0] wr0_reader_id  = active_wr_edt[0][6];

    wire [15:0] tx_total_dgram_words = pkt_len[11:0] + 12'd17; // 17 header words (68B) + 1024 payload words = 1041 words
    wire [15:0] tx_udp_total_bytes   = {2'b00, tx_total_dgram_words[11:0], 2'b00}; // 4,164 bytes
    wire [15:0] tx_data_octets_le    = {2'b00, pkt_len[11:0], 2'b00} + 16'd24;     // 4,120 bytes (0x1018)
    wire [15:0] tx_data_octets_wire  = {tx_data_octets_le[7:0], tx_data_octets_le[15:8]}; // 0x1810 on wire

    // Freeze tx_sample_dout when pacing_timer > 0 so multi-cycle beats don't lose prefetched data
    reg [DATA_WIDTH-1:0] tx_sample_buf;
    always @(posedge clk) begin
        if (!rst_n) begin
            tx_sample_buf <= 32'd0;
        end else if (pacing_timer == cfg_pacing_timer[15:0]) begin
            tx_sample_buf <= tx_sample_dout;
        end
    end
    wire [31:0] tx_sample_word = (cfg_pacing_timer == 32'd0) ? tx_sample_dout : tx_sample_buf;

    // Combinational UDP/RTPS Datagram Word Mux (Word 0..16 headers, Word 17..1040 sample payload)
    reg [31:0] cur_dgram_word;
    always @(*) begin
        case (tx_dgram_word_idx)
            12'd0:  cur_dgram_word = {wr0_src_port, wr0_dst_port};
            12'd1:  cur_dgram_word = {tx_udp_total_bytes, tx_udp_csum};
            12'd2:  cur_dgram_word = 32'h5254_5053; // "RTPS"
            12'd3:  cur_dgram_word = {16'h0201, local_vendor_id}; // RTPS v2.1, VendorId=0x0110 (Cyclone)
            12'd4:  cur_dgram_word = local_guid_prefix[95:64];
            12'd5:  cur_dgram_word = local_guid_prefix[63:32];
            12'd6:  cur_dgram_word = local_guid_prefix[31:0];
            12'd7:  cur_dgram_word = 32'h0901_0800; // INFO_TS (0x09), flags=0x01 (E=1), octetsToNext=8 (LE)
            12'd8:  cur_dgram_word = {latched_tx_ts[39:32], latched_tx_ts[47:40], latched_tx_ts[55:48], latched_tx_ts[63:56]};
            12'd9:  cur_dgram_word = {latched_tx_ts[7:0],   latched_tx_ts[15:8],  latched_tx_ts[23:16], latched_tx_ts[31:24]};
            12'd10: cur_dgram_word = {8'h15, 8'h05, tx_data_octets_wire}; // DATA (0x15), flags=0x05 (D=1,Q=0,E=1)
            12'd11: cur_dgram_word = 32'h0000_1000; // extraFlags=0, octetsToInlineQos=16 (LE)
            12'd12: cur_dgram_word = wr0_reader_id;
            12'd13: cur_dgram_word = wr0_writer_id;
            12'd14: cur_dgram_word = {cur_tx_seq[39:32], cur_tx_seq[47:40], cur_tx_seq[55:48], cur_tx_seq[63:56]};
            12'd15: cur_dgram_word = {cur_tx_seq[7:0],   cur_tx_seq[15:8],  cur_tx_seq[23:16], cur_tx_seq[31:24]};
            12'd16: cur_dgram_word = 32'h0001_0000; // CDR_LE encapsulation (0x0001, options=0x0000)
            default: cur_dgram_word = tx_sample_word;
        endcase
    end

    // =========================================================================
    // High-Performance Pipelined UDP & IP Checksum Engines (Timing Closure)
    //
    // Instead of chaining 60 carry-chains with per-word end-around carries,
    // we sum the 16-bit half-words into a wide 24-bit binary accumulator tree
    // (max 1 carry-chain per stage, delay < 1.5 ns), and fold the carries once.
    //
    // The result is ready in 4 clock cycles, well before Word 1 is sent on Beat 9.
    // =========================================================================
    reg [23:0] ucs_tree_0, ucs_tree_1, ucs_tree_2;
    reg [15:0] ucs_final_fold;

    // Stage 1: Sum 8 half-words (4 words: payload_csum, local_src_ip, wr0_dst_ip, udp_len/proto)
    wire [23:0] ucs_stg0_sum = {8'd0, payload_csum_accum} +
                               {8'd0, local_src_ip[31:16]} + {8'd0, local_src_ip[15:0]} +
                               {8'd0, wr0_dst_ip[31:16]}   + {8'd0, wr0_dst_ip[15:0]} +
                               {8'd0, 16'h0011}            + {8'd0, tx_udp_total_bytes} +
                               {8'd0, wr0_src_port}        + {8'd0, wr0_dst_port};

    // Stage 2: Sum next 8 half-words (RTPS magic, version/vendor, guidPrefix)
    wire [23:0] ucs_stg1_sum = ucs_tree_0 +
                               {8'd0, tx_udp_total_bytes} + 24'd0 +
                               {8'd0, 16'h5254} + {8'd0, 16'h5053} +
                               {8'd0, 16'h0201} + {8'd0, local_vendor_id} +
                               {8'd0, local_guid_prefix[95:80]} + {8'd0, local_guid_prefix[79:64]} +
                               {8'd0, local_guid_prefix[63:48]} + {8'd0, local_guid_prefix[47:32]} +
                               {8'd0, local_guid_prefix[31:16]} + {8'd0, local_guid_prefix[15:0]};

    // Stage 3: Sum remaining fields (INFO_TS, timestamp, DATA header, writer/reader IDs, seqNum, CDR_LE)
    wire [23:0] ucs_stg2_sum = ucs_tree_1 +
                               {8'd0, 16'h0901} + {8'd0, 16'h0800} +
                               {8'd0, latched_tx_ts[39:32], latched_tx_ts[47:40]} +
                               {8'd0, latched_tx_ts[55:48], latched_tx_ts[63:56]} +
                               {8'd0, latched_tx_ts[7:0],   latched_tx_ts[15:8]}  +
                               {8'd0, latched_tx_ts[23:16], latched_tx_ts[31:24]} +
                               {8'd0, 8'h15, 8'h05} + {8'd0, tx_data_octets_wire} +
                               {8'd0, 16'h0000} + {8'd0, 16'h1000} +
                               {8'd0, wr0_reader_id[31:16]} + {8'd0, wr0_reader_id[15:0]} +
                               {8'd0, wr0_writer_id[31:16]} + {8'd0, wr0_writer_id[15:0]} +
                               {8'd0, cur_tx_seq[39:32], cur_tx_seq[47:40]} +
                               {8'd0, cur_tx_seq[55:48], cur_tx_seq[63:56]} +
                               {8'd0, cur_tx_seq[7:0],   cur_tx_seq[15:8]}  +
                               {8'd0, cur_tx_seq[23:16], cur_tx_seq[31:24]} +
                               {8'd0, 16'h0001};

    // Final Stage 4: 1's complement carry-fold of the 24-bit binary sum into 16 bits
    wire [19:0] ucs_fold_tmp1 = {4'd0, ucs_tree_2[15:0]} + {12'd0, ucs_tree_2[23:16]};
    wire [16:0] ucs_fold_tmp2 = {1'b0, ucs_fold_tmp1[15:0]} + {13'd0, ucs_fold_tmp1[19:16]};
    wire [15:0] ucs_folded    = ucs_fold_tmp2[15:0] + {15'd0, ucs_fold_tmp2[16]};

    always @(posedge clk) begin
        if (!rst_n) begin
            ucs_tree_0     <= 24'd0;
            ucs_tree_1     <= 24'd0;
            ucs_tree_2     <= 24'd0;
            ucs_final_fold <= 16'd0;
        end else begin
            ucs_tree_0     <= ucs_stg0_sum;
            ucs_tree_1     <= ucs_stg1_sum;
            ucs_tree_2     <= ucs_stg2_sum;
            ucs_final_fold <= ucs_folded;
        end
    end
    wire [15:0] calc_tx_udp_csum = (~ucs_final_fold == 16'h0000) ? 16'hFFFF : ~ucs_final_fold;

    // Pipelined IPv4 Header Checksum: computed in 2 stages during Beat 0 and Beat 1
    reg [19:0] ipcs_stg0;
    reg [15:0] ipcs_stg1;
    always @(posedge clk) begin
        if (!rst_n) begin
            ipcs_stg0 <= 20'd0;
            ipcs_stg1 <= 16'd0;
        end else begin
            ipcs_stg0 <= {4'd0, 16'h4500} + {4'd0, tx_frag_ip_len} +
                         {4'd0, tx_ip_id_counter} + {4'd0, tx_frag_flags} +
                         {4'd0, 16'h4011} +
                         {4'd0, local_src_ip[31:16]} + {4'd0, local_src_ip[15:0]} +
                         {4'd0, wr0_dst_ip[31:16]}   + {4'd0, wr0_dst_ip[15:0]};
            ipcs_stg1 <= ~(ipcs_stg0[15:0] + {12'd0, ipcs_stg0[19:16]});
        end
    end
    wire [15:0] calc_tx_ip_hdr_csum = ipcs_stg1;

    // Combinational Ethernet + IPv4 + Fragment Word Builder
    reg [31:0] next_wire_tx_word;
    always @(*) begin
        case (tx_beat_idx)
            12'd0: next_wire_tx_word = wr0_dst_mac[47:16];
            12'd1: next_wire_tx_word = {wr0_dst_mac[15:0], local_mac[47:32]};
            12'd2: next_wire_tx_word = local_mac[31:0];
            12'd3: next_wire_tx_word = 32'h0800_4500; // EtherType 0x0800 (IPv4), Ver=4, IHL=5, TOS=0
            12'd4: next_wire_tx_word = {tx_frag_ip_len, tx_ip_id_counter};
            12'd5: next_wire_tx_word = {tx_frag_flags, 16'h4011}; // TTL=64, Proto=UDP (0x11)
            12'd6: next_wire_tx_word = {tx_frag_hdr_csum, local_src_ip[31:16]};
            12'd7: next_wire_tx_word = {local_src_ip[15:0], wr0_dst_ip[31:16]};
            12'd8: next_wire_tx_word = {wr0_dst_ip[15:0], cur_dgram_word[31:16]};
            default: begin
                if (tx_beat_idx == (12'd8 + tx_frag_words))
                    next_wire_tx_word = {tx_prev_half, 16'h0000};
                else
                    next_wire_tx_word = {tx_prev_half, cur_dgram_word[31:16]};
            end
        endcase
    end

    // =========================================================================
    // Skid Buffer & Pipelined S2MM Egress Signals 
    // Option B (ctrl_reg[7] == 1): S2MM streams from Word 2 ("RTPS" header +
    // INFO_TS + DATA + CDR_LE + 4KB payload = latest_rtps_words, e.g. 1039W)
    // directly into NDDS_Transport_Plugin::receive_rEA() for DataReader::take()!
    // =========================================================================
    reg  [ADDR_WIDTH:0]   egress_rd_ptr;
    reg                   bram_pipe_valid;
    reg                   bram_pipe_last;
    reg  [DATA_WIDTH-1:0] skid_data;
    reg                   skid_last;
    reg                   skid_valid;

    wire                  plugin_rx_rtps_mode = ctrl_reg[7];
    wire [11:0]           egress_start_word   = plugin_rx_rtps_mode ? 12'd2 : latest_payload_word_off;
    wire [ADDR_WIDTH:0]   egress_total_words  = plugin_rx_rtps_mode ? latest_rtps_words : pkt_len;

    assign m_axis_tkeep = {(DATA_WIDTH/8){1'b1}};
    wire   tx_handshake = m_axis_tvalid && m_axis_tready;
    wire   can_issue_bram_rd = (state == S_TX_RUN) && (egress_rd_ptr < egress_total_words) &&
                               (!skid_valid) &&
                               (m_axis_tready || !bram_pipe_valid || !m_axis_tvalid);

    always @(*) begin
        s2mm_rd_addr = egress_start_word + egress_rd_ptr[11:0];
    end

    // =========================================================================
    // Main Sequential Logic: Master Cycle FSM + RX Reassembly & RTPS Walker
    // =========================================================================
    integer rst_ep;
    always @(posedge clk) begin
        if (!rst_n) begin
            state                   <= S_IDLE;
            rx_ptr                  <= 0;
            pkt_len                 <= 0;
            pacing_timer            <= 16'd0;
            cycle_tick_cnt          <= 32'd0;
            last_cycle_ticks        <= 32'd0;
            rx_wait_cnt             <= 32'd0;

            ip_done_irq             <= 1'b0;
            err_crc                 <= 1'b0;
            err_no_match            <= 1'b0;
            err_rx_timeout          <= 1'b0;
            err_csum                <= 1'b0;
            irq_is_rx               <= 1'b0;
            completed_frags         <= 4'd0;

            tx_ip_id_counter        <= 16'h1000;
            latched_tx_ts           <= 64'd0;
            cur_tx_seq              <= 64'd1;
            tx_udp_csum             <= 16'd0;
            tx_frag_idx             <= 2'd0;
            tx_frag_words           <= 12'd0;
            tx_frag_ip_len          <= 16'd0;
            tx_frag_flags           <= 16'd0;
            tx_frag_hdr_csum        <= 16'd0;
            tx_dgram_word_idx       <= 12'd0;
            tx_beat_idx             <= 12'd0;
            tx_prev_half            <= 16'd0;
            tx_sample_rd_addr       <= {ADDR_WIDTH{1'b0}};

            eng_tx_tdata            <= 32'd0;
            eng_tx_tvalid           <= 1'b0;
            eng_tx_tlast            <= 1'b0;

            egress_rd_ptr           <= 0;
            bram_pipe_valid         <= 1'b0;
            bram_pipe_last          <= 1'b0;
            skid_data               <= 32'd0;
            skid_last               <= 1'b0;
            skid_valid              <= 1'b0;
            m_axis_tvalid           <= 1'b0;
            m_axis_tdata            <= 32'd0;
            m_axis_tlast            <= 1'b0;

            // RX Reassembly & Walker Reset
            rx_state                <= R_IDLE;
            fill_slot               <= 2'd0;
            latest_slot             <= 2'd1;
            egress_slot             <= 2'd2;
            latest_valid            <= 1'b0;
            reasm_wr_en             <= 1'b0;
            reasm_wr_addr           <= 12'd0;
            reasm_wr_data           <= 32'd0;
            walker_rd_addr          <= 12'd0;

            rx_wire_beat            <= 12'd0;
            rx_ip_len               <= 16'd0;
            rx_ip_id                <= 16'd0;
            rx_active_ip_id         <= 16'd0;
            rx_active_valid         <= 1'b0;
            rx_flags_frag           <= 16'd0;
            rx_hdr_csum_accum       <= 16'd0;
            rx_src_ip               <= 32'd0;
            rx_dst_ip               <= 32'd0;
            rx_prev_half            <= 16'd0;
            rx_wr_ptr               <= 12'd0;
            rx_bytes_rcvd           <= 16'd0;
            rx_bytes_total          <= 16'd0;
            rx_saw_last             <= 1'b0;
            rx_udp_csum_accum       <= 16'd0;
            rx_udp_len_bytes        <= 16'd0;
            rx_udp_src_port         <= 16'd0;
            rx_udp_dst_port         <= 16'd0;
            last_rx_src_ip          <= 32'd0;
            last_rx_src_port        <= 16'd0;
            last_rx_dst_port        <= 16'd0;
            last_rx_cam_idx         <= 2'd0;

            walk_guid_prefix        <= 96'd0;
            walk_cam_idx            <= 2'd0;
            walk_cam_hit            <= 1'b0;
            walk_cur_word           <= 12'd0;
            walk_dgram_words        <= 12'd0;
            walk_submsg_id          <= 8'd0;
            walk_submsg_flags       <= 8'd0;
            walk_octets_next        <= 16'd0;
            walk_next_submsg_word   <= 12'd0;
            walk_ts                 <= 64'd0;
            walk_writer_id          <= 32'd0;
            walk_seq_num            <= 64'd0;
            latest_payload_word_off <= 12'd17;
            latest_rtps_words       <= 12'd1039;

            stat_tx_samples         <= 32'd0;
            stat_rx_samples         <= 32'd0;
            stat_seq_gaps           <= 32'd0;
            stat_stale_drops        <= 32'd0;
            stat_csum_errs          <= 32'd0;
            stat_skipped_submsg     <= 32'd0;
            last_rx_seq_num         <= 64'd0;
            last_rx_ts              <= 64'd0;

            walk_dst_guid_prefix    <= 96'd0;
            walk_dst_valid          <= 1'b0;
            tx_frag_payload_bytes   <= 16'd0;
            tx_frag_offset_bytes    <= 16'd0;
            tx_bytes_remaining      <= 16'd0;
            tx_frag_more            <= 1'b0;

            for (rst_ep = 0; rst_ep < NUM_ENDPOINTS; rst_ep = rst_ep + 1) begin
                writer_seq_num[rst_ep]         <= 64'd1;
                rd_match_count[rst_ep]         <= 32'd0;
                last_rx_seq_num_by_cam[rst_ep] <= 64'd0;
                last_rx_ts_by_cam[rst_ep]      <= 64'd0;
            end

            for (rst_ep = 0; rst_ep < 3; rst_ep = rst_ep + 1) begin
                reasm_coverage[rst_ep]       <= {REASM_BLOCKS{1'b0}};
                reasm_blocks_covered[rst_ep] <= 10'd0;
                reasm_total_bytes[rst_ep]    <= 16'd0;
                reasm_last_seen[rst_ep]      <= 1'b0;
            end
        end else begin
            // Default single-cycle pulse deassertions
            reasm_wr_en <= 1'b0;

            // Atomic synchronization pulse on commit (Section 2)
            if (edt_commit_applied) begin
                for (rst_ep = 0; rst_ep < NUM_ENDPOINTS; rst_ep = rst_ep + 1) begin
                    if (active_wr_edt[rst_ep][0][0] && (active_wr_edt[rst_ep][7] != 32'd0)) begin
                        writer_seq_num[rst_ep] <= {32'd0, active_wr_edt[rst_ep][7]};
                    end
                    last_rx_seq_num_by_cam[rst_ep] <= 64'd0;
                    last_rx_ts_by_cam[rst_ep]      <= 64'd0;
                end
                last_rx_seq_num <= 64'd0;
                last_rx_ts      <= 64'd0;
            end

            if (identity_clear_seq_pulse) begin
                for (rst_ep = 0; rst_ep < NUM_ENDPOINTS; rst_ep = rst_ep + 1) begin
                    writer_seq_num[rst_ep]         <= 64'd1;
                    last_rx_seq_num_by_cam[rst_ep] <= 64'd0;
                    last_rx_ts_by_cam[rst_ep]      <= 64'd0;
                end
                last_rx_seq_num <= 64'd0;
                last_rx_ts      <= 64'd0;
            end

            // =================================================================
            // PART A: Independent RX Reassembly & Generic RTPS Submessage Walker
            // =================================================================
            case (rx_state)
                R_IDLE: begin
                    if (eng_rx_fire) begin
                        case (rx_wire_beat)
                            12'd0, 12'd1, 12'd2: begin
                                rx_wire_beat <= rx_wire_beat + 12'd1;
                            end
                            12'd3: begin
                                // Beat 3: {EtherType(0x0800), VerIHL_TOS(0x4500)}
                                rx_hdr_csum_accum <= csum_add32(16'h0000, {16'h0000, eng_rx_tdata[15:0]});
                                rx_wire_beat      <= rx_wire_beat + 12'd1;
                            end
                            12'd4: begin
                                // Beat 4: {IPv4_TotalLen, IPv4_ID}
                                rx_ip_len         <= eng_rx_tdata[31:16];
                                rx_ip_id          <= eng_rx_tdata[15:0];
                                rx_hdr_csum_accum <= csum_add32(rx_hdr_csum_accum, eng_rx_tdata);
                                rx_wire_beat      <= rx_wire_beat + 12'd1;
                            end
                            12'd5: begin
                                // Beat 5: {IPv4_FlagsFrag, TTL_Proto}
                                rx_flags_frag     <= eng_rx_tdata[31:16];
                                rx_hdr_csum_accum <= csum_add32(rx_hdr_csum_accum, eng_rx_tdata);
                                rx_wire_beat      <= rx_wire_beat + 12'd1;

                                // If a new IP_ID arrives, reset reassembly context for the new datagram
                                if (!rx_active_valid || (rx_active_ip_id != rx_ip_id)) begin
                                    rx_active_ip_id                <= rx_ip_id;
                                    rx_active_valid                <= 1'b1;
                                    rx_bytes_rcvd                  <= 16'd0;
                                    rx_bytes_total                 <= 16'd0;
                                    rx_saw_last                    <= 1'b0;
                                    rx_udp_csum_accum              <= 16'd0;
                                    reasm_coverage[fill_slot]       <= {REASM_BLOCKS{1'b0}};
                                    reasm_blocks_covered[fill_slot] <= 10'd0;
                                    reasm_total_bytes[fill_slot]    <= 16'd0;
                                    reasm_last_seen[fill_slot]      <= 1'b0;
                                end
                            end
                            12'd6: begin
                                // Beat 6: {IPv4_HdrCsum, SrcIP[31:16]}
                                rx_src_ip[31:16]  <= eng_rx_tdata[15:0];
                                rx_hdr_csum_accum <= csum_add32(rx_hdr_csum_accum, eng_rx_tdata);
                                rx_wire_beat      <= rx_wire_beat + 12'd1;
                            end
                            12'd7: begin
                                // Beat 7: {SrcIP[15:0], DstIP[31:16]}
                                rx_src_ip[15:0]   <= eng_rx_tdata[31:16];
                                rx_dst_ip[31:16]  <= eng_rx_tdata[15:0];
                                rx_hdr_csum_accum <= csum_add32(rx_hdr_csum_accum, eng_rx_tdata);
                                rx_wire_beat      <= rx_wire_beat + 12'd1;
                            end
                            12'd8: begin
                                // Beat 8: {DstIP[15:0], IP_Payload_Half0}
                                rx_dst_ip[15:0]   <= eng_rx_tdata[31:16];
                                rx_hdr_csum_accum <= csum_add32(rx_hdr_csum_accum, {eng_rx_tdata[31:16], 16'h0000});
                                rx_prev_half      <= eng_rx_tdata[15:0];
                                rx_wr_ptr         <= rx_frag_w_start;
                                rx_wire_beat      <= rx_wire_beat + 12'd1;
                            end
                            default: begin
                                // Store reassembled 32-bit word into fill_slot at rx_wr_ptr
                                if (rx_wr_ptr < (rx_frag_w_start + rx_ip_pay_bytes[13:2])) begin
                                    reasm_wr_en       <= 1'b1;
                                    reasm_wr_addr     <= rx_wr_ptr;
                                    reasm_wr_data     <= {rx_prev_half, eng_rx_tdata[31:16]};
                                    rx_udp_csum_accum <= csum_add32(rx_udp_csum_accum, {rx_prev_half, eng_rx_tdata[31:16]});
                                    if (rx_frag_off_8b == 13'd0 && rx_wr_ptr == 12'd0) begin
                                        rx_udp_src_port <= rx_prev_half;
                                        rx_udp_dst_port <= eng_rx_tdata[31:16];
                                    end
                                    if (rx_frag_off_8b == 13'd0 && rx_wr_ptr == 12'd1) begin
                                        rx_udp_len_bytes <= rx_prev_half;
                                    end
                                    rx_wr_ptr <= rx_wr_ptr + 12'd1;

                                    // Mark 8-byte block in coverage bitmap (Section 6)
                                    if (!reasm_coverage[fill_slot][rx_wr_ptr[10:1]]) begin
                                        reasm_coverage[fill_slot][rx_wr_ptr[10:1]] <= 1'b1;
                                        reasm_blocks_covered[fill_slot] <= reasm_blocks_covered[fill_slot] + 10'd1;
                                    end
                                end
                                rx_prev_half <= eng_rx_tdata[15:0];

                                if (eng_rx_tlast) begin
                                    rx_wire_beat <= 12'd0;
                                    // Verify IPv4 header checksum (1's complement sum of valid header is 0xFFFF)
                                    if (rx_hdr_csum_accum != 16'hFFFF) begin
                                        err_csum        <= 1'b1;
                                        stat_csum_errs  <= stat_csum_errs + 32'd1;
                                        rx_active_valid <= 1'b0;
                                    end else begin
                                        // Update reassembly status
                                        if (!rx_frag_mf) begin
                                            rx_bytes_total               <= {rx_frag_off_8b, 3'b000} + rx_ip_pay_bytes;
                                            rx_saw_last                  <= 1'b1;
                                            reasm_last_seen[fill_slot]   <= 1'b1;
                                            reasm_total_bytes[fill_slot] <= {rx_frag_off_8b, 3'b000} + rx_ip_pay_bytes;
                                        end
                                        rx_bytes_rcvd <= rx_bytes_rcvd + rx_ip_pay_bytes;
                                        rx_state      <= R_REASM_CHECK;
                                    end
                                end else begin
                                    rx_wire_beat <= rx_wire_beat + 12'd1;
                                end
                            end
                        endcase
                    end
                end

                // 1-cycle registered evaluation of reassembly coverage (breaks 768-to-1 MUX critical path)
                R_REASM_CHECK: begin
                    if (reasm_last_seen[fill_slot] &&
                        (reasm_total_bytes[fill_slot] > 16'd0) &&
                        (reasm_blocks_covered[fill_slot] == ((reasm_total_bytes[fill_slot] + 16'd7) >> 3))) begin
                        rx_active_valid <= 1'b0;
                        rx_bytes_rcvd   <= 16'd0;
                        rx_saw_last     <= 1'b0;
                        walker_rd_addr  <= 12'd2; // Prime Word 2 ("RTPS")
                        rx_state        <= R_CHECK_UDP;
                    end else begin
                        rx_state        <= R_IDLE;
                    end
                end

                // Fold UDP pseudo-header into rx_udp_csum_accum & check == 0xFFFF
                R_CHECK_UDP: begin
                    walk_dst_valid       <= 1'b0;
                    walk_dst_guid_prefix <= 96'd0;
                    walk_dgram_words     <= rx_udp_len_bytes[13:2];
                    if (!rx_chk_valid) begin
                        err_csum       <= 1'b1;
                        stat_csum_errs <= stat_csum_errs + 32'd1;
                        rx_state       <= R_IDLE;
                    end else begin
                        walker_rd_addr <= 12'd3; // Prime Word 3 (Ver + VendorId)
                        rx_state       <= R_RTPS_HDR_0;
                    end
                end

                R_RTPS_HDR_0: begin
                    // walker_dout holds Word 2 ("RTPS" = 0x5254_5053)
                    if (walker_dout != 32'h5254_5053) begin
                        rx_state <= R_IDLE;
                    end else begin
                        walker_rd_addr <= 12'd4; // Prime Word 4 (GuidPrefix[95:64])
                        rx_state       <= R_RTPS_HDR_1;
                    end
                end

                R_RTPS_HDR_1: begin
                    // walker_dout holds Word 3 ({ProtoVer[15:0], VendorId[15:0]})
                    if (walker_dout[31:24] != 8'h02) begin // Major version must be 2
                        rx_state <= R_IDLE;
                    end else begin
                        walker_rd_addr <= 12'd5; // Prime Word 5 (GuidPrefix[63:32])
                        rx_state       <= R_RTPS_GUID_0;
                    end
                end

                R_RTPS_GUID_0: begin
                    walk_guid_prefix[95:64] <= walker_dout;
                    walker_rd_addr          <= 12'd6; // Prime Word 6 (GuidPrefix[31:0])
                    rx_state                <= R_RTPS_GUID_1;
                end

                R_RTPS_GUID_1: begin
                    walk_guid_prefix[63:32] <= walker_dout;
                    walker_rd_addr          <= 12'd7; // Prime Word 7 (First Submessage Header)
                    rx_state                <= R_RTPS_GUID_2;
                end

                R_RTPS_GUID_2: begin
                    walk_guid_prefix[31:0] <= walker_dout;
                    walk_cur_word          <= 12'd7;
                    if (cam_hit_0) begin
                        walk_cam_hit <= 1'b1;
                        walk_cam_idx <= 2'd0;
                        rx_state     <= R_SUBMSG_HDR;
                    end else if (cam_hit_1) begin
                        walk_cam_hit <= 1'b1;
                        walk_cam_idx <= 2'd1;
                        rx_state     <= R_SUBMSG_HDR;
                    end else if (cam_hit_2) begin
                        walk_cam_hit <= 1'b1;
                        walk_cam_idx <= 2'd2;
                        rx_state     <= R_SUBMSG_HDR;
                    end else if (cam_hit_3) begin
                        walk_cam_hit <= 1'b1;
                        walk_cam_idx <= 2'd3;
                        rx_state     <= R_SUBMSG_HDR;
                    end else begin
                        walk_cam_hit <= 1'b0;
                        rx_state     <= R_IDLE; // Unknown participant GUID -> ignore
                    end
                end

                // Generic Submessage Header Walker
                R_SUBMSG_HDR: begin
                    if (walk_cur_word >= walk_dgram_words) begin
                        rx_state <= R_IDLE;
                    end else begin
                        walk_submsg_id        <= walker_dout[31:24];
                        walk_submsg_flags     <= walker_dout[23:16];
                        walk_octets_next      <= submsg_octets;
                        walk_next_submsg_word <= (submsg_octets == 16'd0) ? walk_dgram_words :
                                                 (walk_cur_word + 12'd1 + submsg_octets[13:2]);

                        case (walker_dout[31:24])
                            8'h09: begin // INFO_TS (0x09)
                                if (walker_dout[17]) begin // I-flag=1 (invalid timestamp) -> skip
                                    walk_cur_word  <= walk_cur_word + 12'd1 + submsg_octets[13:2];
                                    walker_rd_addr <= walk_cur_word + 12'd1 + submsg_octets[13:2];
                                    rx_state       <= R_SUBMSG_HDR_WAIT;
                                end else begin
                                    walker_rd_addr <= walk_cur_word + 12'd1;
                                    rx_state       <= R_INFO_TS_WAIT;
                                end
                            end
                            8'h0E: begin // INFO_DST (0x0E) - Section 4
                                if (submsg_octets != 16'd12) begin
                                    rx_state <= R_IDLE;
                                end else begin
                                    walker_rd_addr <= walk_cur_word + 12'd1;
                                    rx_state       <= R_INFO_DST_WAIT;
                                end
                            end
                            8'h15: begin // DATA (0x15) - Section 5
                                // Body: +1 flags/octetsToInlineQos, +2 readerEntityId, +3 writerEntityId
                                walker_rd_addr <= walk_cur_word + 12'd2; // Prime Word +2: readerEntityId
                                rx_state       <= R_DATA_HDR_1;          // 1-cycle wait state for Word +2
                            end
                            default: begin
                                // Skip HEARTBEAT (0x07), ACKNACK (0x06), DATA_FRAG (0x16), or Vendor (0x80+)
                                stat_skipped_submsg <= stat_skipped_submsg + 32'd1;
                                if (submsg_octets == 16'd0) begin
                                    rx_state <= R_IDLE;
                                end else begin
                                    walk_cur_word  <= walk_cur_word + 12'd1 + submsg_octets[13:2];
                                    walker_rd_addr <= walk_cur_word + 12'd1 + submsg_octets[13:2];
                                    rx_state       <= R_SUBMSG_HDR_WAIT;
                                end
                            end
                        endcase
                    end
                end

                R_SUBMSG_HDR_WAIT: begin
                    rx_state <= R_SUBMSG_HDR;
                end

                R_INFO_TS_WAIT: begin
                    walker_rd_addr <= walk_cur_word + 12'd2; // Prime Word +2 (frac)
                    rx_state       <= R_INFO_TS_0;
                end

                R_INFO_TS_0: begin
                    // walker_dout holds Word +1 (seconds)
                    walk_ts[63:32] <= walk_submsg_flags[0] ?
                                      {walker_dout[7:0], walker_dout[15:8], walker_dout[23:16], walker_dout[31:24]} :
                                      walker_dout;
                    walker_rd_addr <= walk_next_submsg_word; // Prime next Submessage Header
                    rx_state       <= R_INFO_TS_1;
                end

                R_INFO_TS_1: begin
                    // walker_dout holds Word +2 (fraction)
                    walk_ts[31:0]  <= walk_submsg_flags[0] ?
                                      {walker_dout[7:0], walker_dout[15:8], walker_dout[23:16], walker_dout[31:24]} :
                                      walker_dout;
                    walk_cur_word  <= walk_next_submsg_word;
                    rx_state       <= R_SUBMSG_HDR;
                end

                // INFO_DST Participant Filtering (Section 4)
                R_INFO_DST_WAIT: begin
                    walker_rd_addr <= walk_cur_word + 12'd2; // Prime Word +2
                    rx_state       <= R_INFO_DST_0;
                end

                R_INFO_DST_0: begin
                    walk_dst_guid_prefix[95:64] <= walker_dout;
                    walker_rd_addr              <= walk_cur_word + 12'd3; // Prime Word +3
                    rx_state                    <= R_INFO_DST_1;
                end

                R_INFO_DST_1: begin
                    walk_dst_guid_prefix[63:32] <= walker_dout;
                    walker_rd_addr              <= walk_next_submsg_word; // Prime next submessage header
                    rx_state                    <= R_INFO_DST_2;
                end

                R_INFO_DST_2: begin
                    walk_dst_guid_prefix[31:0] <= walker_dout;
                    walk_dst_valid             <= 1'b1;
                    walk_cur_word              <= walk_next_submsg_word;
                    rx_state                   <= R_SUBMSG_HDR;
                end

                R_DATA_HDR_1: begin
                    // 1-cycle BRAM wait for Word +2 (readerEntityId); prime Word +3 (writerEntityId)
                    walker_rd_addr <= walk_cur_word + 12'd3;
                    rx_state       <= R_DATA_READER_ID;
                end

                // Validate DATA.readerId (Section 5)
                R_DATA_READER_ID: begin
                    // walker_dout holds Word +2: readerEntityId
                    // readerId must be ENTITYID_UNKNOWN (0), or match local configured reader in CAM slot
                    if ((walker_dout != 32'h0000_0000) &&
                        (active_rd_edt[walk_cam_idx][5] != 32'h0000_0000) &&
                        (walker_dout != active_rd_edt[walk_cam_idx][5])) begin
                        walk_cur_word  <= walk_next_submsg_word;
                        walker_rd_addr <= walk_next_submsg_word;
                        rx_state       <= R_SUBMSG_HDR_WAIT;
                    end else begin
                        walker_rd_addr <= walk_cur_word + 12'd4; // Prime Word +4: seqNum Hi
                        rx_state       <= R_DATA_HDR_2;
                    end
                end

                R_DATA_HDR_2: begin
                    // walker_dout holds Word +3: writerEntityId
                    walk_writer_id <= walker_dout;
                    if (!info_dst_matches_local || (walker_dout != active_rd_edt[walk_cam_idx][4])) begin
                        walk_cur_word  <= walk_next_submsg_word;
                        walker_rd_addr <= walk_next_submsg_word;
                        rx_state       <= R_SUBMSG_HDR_WAIT;
                    end else begin
                        walker_rd_addr <= walk_cur_word + 12'd5; // Prime Word +5: seqNum Lo
                        rx_state       <= R_DATA_SEQ_HI;
                    end
                end

                R_DATA_SEQ_HI: begin
                    // walker_dout holds Word +4: seqNum Hi
                    walk_seq_num[63:32] <= walk_submsg_flags[0] ?
                                           {walker_dout[7:0], walker_dout[15:8], walker_dout[23:16], walker_dout[31:24]} :
                                           walker_dout;
                    walker_rd_addr      <= walk_cur_word + 12'd5; // Prime Word +5: seqNum Lo
                    rx_state            <= R_DATA_SEQ_LO;
                end

                R_DATA_SEQ_LO: begin
                    // walker_dout holds Word +5: seqNum Lo
                    walk_seq_num[31:0] <= walk_submsg_flags[0] ?
                                          {walker_dout[7:0], walker_dout[15:8], walker_dout[23:16], walker_dout[31:24]} :
                                          walker_dout;
                    walker_rd_addr     <= walk_cur_word + 12'd6; // Prime Word +6 (first QoS PID or Encapsulation)
                    if (walk_submsg_flags[1]) begin // Q-flag == 1: Inline QoS parameter list present
                        walk_cur_word  <= walk_cur_word + 12'd6;
                        rx_state       <= R_SKIP_QOS; // walker_dout already holds Word +6 on next cycle!
                    end else begin
                        // Q-flag == 0: Pipeline into R_COMMIT_SAMPLE to break BRAM-to-CE critical path
                        rx_state <= R_COMMIT_SAMPLE;
                    end
                end

                // Registered commit state: all inputs come from registers (Section 3)
                R_COMMIT_SAMPLE: begin
                    // Word +6 is Encapsulation, Word +7 is Payload start
                    // Check Latest-Value invariant per CAM entry: new seqNum > last_rx_seq_num_by_cam[walk_cam_idx]
                    if (walk_seq_num > last_rx_seq_num_by_cam[walk_cam_idx]) begin
                        if (last_rx_seq_num_by_cam[walk_cam_idx] != 64'd0 &&
                            (walk_seq_num > (last_rx_seq_num_by_cam[walk_cam_idx] + 64'd1))) begin
                            stat_seq_gaps <= stat_seq_gaps +
                                (walk_seq_num[31:0] -
                                 last_rx_seq_num_by_cam[walk_cam_idx][31:0] -
                                 32'd1);
                        end

                        last_rx_seq_num_by_cam[walk_cam_idx] <= walk_seq_num;
                        last_rx_ts_by_cam[walk_cam_idx]      <= walk_ts;
                        last_rx_seq_num                      <= walk_seq_num;
                        last_rx_ts                           <= walk_ts;
                        last_rx_src_ip                       <= rx_src_ip;
                        last_rx_src_port                     <= rx_udp_src_port;
                        last_rx_dst_port                     <= rx_udp_dst_port;
                        last_rx_cam_idx                      <= walk_cam_idx;
                        stat_rx_samples                      <= stat_rx_samples + 32'd1;
                        rd_match_count[walk_cam_idx]         <= rd_match_count[walk_cam_idx] + 32'd1;
                        latest_payload_word_off              <= walk_cur_word + 12'd7; // Skip 4B encapsulation word (+6)
                        latest_rtps_words                    <= walk_dgram_words - 12'd2;
                        latest_slot                          <= fill_slot;
                        latest_valid                         <= 1'b1;
                        fill_slot                            <= pick_free_slot(fill_slot, egress_slot);
                    end else begin
                        stat_stale_drops <= stat_stale_drops + 32'd1;
                    end
                    rx_state <= R_IDLE;
                end

                R_SKIP_QOS_WAIT: begin
                    rx_state <= R_SKIP_QOS;
                end

                // Walk Inline QoS parameter list until PID_SENTINEL (0x0001) (Section 3)
                R_SKIP_QOS: begin
                    // Parameter header: {parameterId[15:0], length[15:0]} (LE if E=1)
                    if ((walk_submsg_flags[0] ? {walker_dout[23:16], walker_dout[31:24]} : walker_dout[31:16]) == 16'h0001) begin
                        // Reached PID_SENTINEL -> next word is Encapsulation, +2 is Payload
                        if (walk_seq_num > last_rx_seq_num_by_cam[walk_cam_idx]) begin
                            if (last_rx_seq_num_by_cam[walk_cam_idx] != 64'd0 &&
                                (walk_seq_num > (last_rx_seq_num_by_cam[walk_cam_idx] + 64'd1))) begin
                                stat_seq_gaps <= stat_seq_gaps +
                                    (walk_seq_num[31:0] -
                                     last_rx_seq_num_by_cam[walk_cam_idx][31:0] -
                                     32'd1);
                            end

                            last_rx_seq_num_by_cam[walk_cam_idx] <= walk_seq_num;
                            last_rx_ts_by_cam[walk_cam_idx]      <= walk_ts;
                            last_rx_seq_num                      <= walk_seq_num;
                            last_rx_ts                           <= walk_ts;
                            last_rx_src_ip                       <= rx_src_ip;
                            last_rx_src_port                     <= rx_udp_src_port;
                            last_rx_dst_port                     <= rx_udp_dst_port;
                            last_rx_cam_idx                      <= walk_cam_idx;
                            stat_rx_samples                      <= stat_rx_samples + 32'd1;
                            rd_match_count[walk_cam_idx]         <= rd_match_count[walk_cam_idx] + 32'd1;
                            latest_payload_word_off              <= walk_cur_word + 12'd2;
                            latest_rtps_words                    <= walk_dgram_words - 12'd2;
                            latest_slot                          <= fill_slot;
                            latest_valid                         <= 1'b1;
                            fill_slot                            <= pick_free_slot(fill_slot, egress_slot);
                        end else begin
                            stat_stale_drops <= stat_stale_drops + 32'd1;
                        end
                        rx_state <= R_IDLE;
                    end else begin
                        walk_cur_word  <= walk_cur_word + 12'd1 +
                                          (walk_submsg_flags[0] ? {walker_dout[3:0], walker_dout[15:10]} : walker_dout[13:2]);
                        walker_rd_addr <= walk_cur_word + 12'd1 +
                                          (walk_submsg_flags[0] ? {walker_dout[3:0], walker_dout[15:10]} : walker_dout[13:2]);
                        rx_state       <= R_SKIP_QOS_WAIT;
                    end
                end

                default: rx_state <= R_IDLE;
            endcase

            // =================================================================
            // PART B: Master Cycle Sequencer & S2MM Egress FSM
            // =================================================================
            case (state)
                // -------------------------------------------------------------
                // 1. S_IDLE: Wait for DMA0 MM2S sample stream or manual CSR kick
                // -------------------------------------------------------------
                S_IDLE: begin
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;
                    skid_valid    <= 1'b0;
                    ip_done_irq   <= 1'b0;
                    eng_tx_tvalid <= 1'b0;
                    eng_tx_tlast  <= 1'b0;

                    if (rx_handshake && !mode_sub_only) begin
                        if (s_axis_tlast || (rx_ptr == cfg_payload_words - 1)) begin
                            pkt_len <= {1'b0, rx_ptr[ADDR_WIDTH-1:0]} + 1'b1;
                            rx_ptr  <= 0;
                            state   <= S_CHECK_CRC;
                        end else begin
                            rx_ptr  <= rx_ptr + 1'b1;
                        end
                    end else if ((trigger_detected || (status_write_pulse && (status_write_data[2] || status_write_data[0]))) && !mode_sub_only) begin
                        pkt_len <= cfg_payload_words[ADDR_WIDTH:0];
                        rx_ptr  <= 0;
                        state   <= S_CHECK_CRC;
                    end else if (latest_valid && (rx_ptr == 0)) begin
                        // Asynchronous wire RX: Walker has verified a complete sample from remote publisher!
                        egress_slot      <= latest_slot;
                        latest_valid     <= 1'b0;
                        irq_is_rx        <= 1'b1;
                        last_cycle_ticks <= 32'd1;
                        ip_done_irq      <= 1'b1; // Level-High IRQ to PS (IRQ_F2P)
                        state            <= S_DONE_IRQ;
                    end
                end

                // -------------------------------------------------------------
                // 2. S_CHECK_CRC: Verify DMA0 CRC, Check Lifecycle, Init UDP Csum
                // -------------------------------------------------------------
                S_CHECK_CRC: begin
                    latched_tx_ts  <= active_time_ns + ts_offset;
                    cur_tx_seq     <= writer_seq_num[0];
                    cycle_tick_cnt <= 32'd0;

                    if (!lifecycle_ok) begin
                        // Q22: Disabled / watchdog expired / no valid writer match -> immediate ERR_NO_MATCH
                        err_no_match   <= 1'b1;
                        err_crc        <= 1'b0;
                        err_rx_timeout <= 1'b0;
                        irq_is_rx      <= 1'b0;
                        ip_done_irq    <= 1'b1;
                        state          <= S_DONE_IRQ;
                    end else if ((crc_accum == expected_crc) || (expected_crc == 32'h0)) begin
                        err_crc            <= 1'b0;
                        err_no_match       <= 1'b0;
                        err_rx_timeout     <= 1'b0;
                        err_csum           <= 1'b0;
                        completed_frags      <= 4'd0;
                        tx_frag_idx          <= 2'd0;
                        tx_bytes_remaining   <= tx_udp_total_bytes;
                        tx_frag_offset_bytes <= 16'd0;
                        tx_ip_id_counter     <= tx_ip_id_counter + 16'd1;
                        writer_seq_num[0]    <= writer_seq_num[0] + 64'd1;
                        latest_valid         <= 1'b0;
                        state                <= S_TX_FRAG_INIT;
                    end else begin
                        err_crc     <= 1'b1;
                        irq_is_rx   <= 1'b0;
                        ip_done_irq <= 1'b1;
                        state       <= S_DONE_IRQ;
                    end
                end

                // -------------------------------------------------------------
                // 3. S_TX_FRAG_INIT: Generic Fragmentation Planner (Section 7)
                // -------------------------------------------------------------
                S_TX_FRAG_INIT: begin
                    cycle_tick_cnt <= cycle_tick_cnt + 32'd1;

                    if (tx_bytes_remaining > IPV4_NONFINAL_PAYLOAD) begin
                        tx_frag_payload_bytes <= IPV4_NONFINAL_PAYLOAD;
                        tx_frag_more          <= 1'b1;
                        tx_frag_words         <= (IPV4_NONFINAL_PAYLOAD >> 2); // 370 words (1480B)
                        tx_frag_ip_len        <= IPV4_HDR_BYTES + IPV4_NONFINAL_PAYLOAD; // 1500 bytes
                        tx_frag_flags         <= {2'b00, 1'b1, tx_frag_offset_bytes[15:3]}; // MF=1
                    end else begin
                        tx_frag_payload_bytes <= tx_bytes_remaining;
                        tx_frag_more          <= 1'b0;
                        tx_frag_words         <= (tx_bytes_remaining + 16'd3) >> 2;
                        tx_frag_ip_len        <= IPV4_HDR_BYTES + tx_bytes_remaining;
                        tx_frag_flags         <= {2'b00, 1'b0, tx_frag_offset_bytes[15:3]}; // MF=0
                    end

                    tx_dgram_word_idx <= tx_frag_offset_bytes >> 2;

                    if ((tx_frag_offset_bytes >> 2) >= 12'd17) begin
                        tx_sample_rd_addr <= (tx_frag_offset_bytes >> 2) - 12'd17;
                    end else begin
                        tx_sample_rd_addr <= 11'd0;
                    end

                    tx_beat_idx   <= 12'd0;
                    eng_tx_tvalid <= 1'b0;
                    eng_tx_tlast  <= 1'b0;
                    pacing_timer  <= cfg_pacing_timer[15:0];
                    state         <= S_TX_FRAG_SEND;
                end

                // -------------------------------------------------------------
                // 4. S_TX_FRAG_SEND: Stream Fragment over Wire Port
                // -------------------------------------------------------------
                S_TX_FRAG_SEND: begin
                    if (trigger_detected) begin
                        eng_tx_tvalid <= 1'b0;
                        eng_tx_tlast  <= 1'b0;
                        ip_done_irq   <= 1'b0;
                        rx_ptr        <= 0;
                        state         <= S_IDLE;
                    end else begin
                        cycle_tick_cnt <= cycle_tick_cnt + 32'd1;
                        if (tx_beat_idx == 12'd3) begin
                            tx_frag_hdr_csum <= calc_tx_ip_hdr_csum;
                        end
                        if (tx_frag_idx == 2'd0 && tx_beat_idx == 12'd4) begin
                            tx_udp_csum <= calc_tx_udp_csum;
                        end

                        if (!eng_tx_tvalid || eng_tx_tready) begin
                            if (pacing_timer == 16'd0) begin
                                pacing_timer  <= cfg_pacing_timer[15:0];
                                eng_tx_tdata  <= next_wire_tx_word;
                                eng_tx_tvalid <= 1'b1;
                                eng_tx_tlast  <= (tx_beat_idx == (12'd8 + tx_frag_words));

                                if (tx_beat_idx == (12'd8 + tx_frag_words)) begin
                                    completed_frags <= completed_frags + 4'd1;
                                    if (!tx_frag_more) begin
                                        stat_tx_samples <= stat_tx_samples + 32'd1;
                                        if (mode_pub_only) begin
                                            // Publish-only mode: all fragments sent, no RX feedback expected!
                                            last_cycle_ticks <= cycle_tick_cnt + 32'd1;
                                            irq_is_rx        <= 1'b0;
                                            ip_done_irq      <= 1'b1;
                                            state            <= S_DONE_IRQ;
                                        end else begin
                                            rx_wait_cnt      <= 32'd0;
                                            state            <= S_WAIT_RX_FB;
                                        end
                                    end else begin
                                        tx_bytes_remaining   <= tx_bytes_remaining - tx_frag_payload_bytes;
                                        tx_frag_offset_bytes <= tx_frag_offset_bytes + tx_frag_payload_bytes;
                                        tx_frag_idx          <= tx_frag_idx + 2'd1;
                                        state                <= S_TX_FRAG_INIT;
                                    end
                                end else begin
                                    tx_beat_idx <= tx_beat_idx + 12'd1;
                                    // For fragments 1 & 2, beat 7 precedes beat 8 (where sample word 353/723 is read),
                                    // so increment tx_sample_rd_addr on beat 7 so beat 9 gets word 354/724!
                                    if (tx_beat_idx == 12'd7 && tx_dgram_word_idx >= 12'd17) begin
                                        tx_sample_rd_addr <= tx_sample_rd_addr + 11'd1;
                                    end else if (tx_beat_idx >= 12'd8) begin
                                        tx_prev_half      <= cur_dgram_word[15:0];
                                        tx_dgram_word_idx <= tx_dgram_word_idx + 12'd1;
                                        // Advance sample BRAM read pointer 1 beat ahead of payload window (tx_dgram_word_idx >= 16)
                                        if (tx_dgram_word_idx >= 12'd16) begin
                                            tx_sample_rd_addr <= tx_sample_rd_addr + 11'd1;
                                        end
                                    end
                                end
                            end else begin
                                pacing_timer  <= pacing_timer - 16'd1;
                                eng_tx_tvalid <= 1'b0;
                                eng_tx_tlast  <= 1'b0;
                            end
                        end
                    end
                end

                // -------------------------------------------------------------
                // 5. S_WAIT_RX_FB: Wait for Latest-Value RX Sample or Timeout
                // -------------------------------------------------------------
                S_WAIT_RX_FB: begin
                    if (eng_tx_tvalid && eng_tx_tready) begin
                        eng_tx_tvalid <= 1'b0;
                        eng_tx_tlast  <= 1'b0;
                    end

                    if (trigger_detected) begin
                        eng_tx_tvalid <= 1'b0;
                        eng_tx_tlast  <= 1'b0;
                        ip_done_irq   <= 1'b0;
                        rx_ptr        <= 0;
                        state         <= S_IDLE;
                    end else if (latest_valid) begin
                        // Lock the validated slot for S2MM Egress
                        egress_slot      <= latest_slot;
                        latest_valid     <= 1'b0;
                        irq_is_rx        <= 1'b1;
                        last_cycle_ticks <= cycle_tick_cnt + 32'd1;
                        ip_done_irq      <= 1'b1; // Assert Level-High IRQ to PS!
                        state            <= S_DONE_IRQ;
                    end else if (rx_wait_cnt >= rx_timeout_limit) begin
                        err_rx_timeout   <= 1'b1;
                        irq_is_rx        <= 1'b0;
                        last_cycle_ticks <= cycle_tick_cnt + 32'd1;
                        ip_done_irq      <= 1'b1; // Wake PS with timeout error bit set
                        state            <= S_DONE_IRQ;
                    end else begin
                        cycle_tick_cnt <= cycle_tick_cnt + 32'd1;
                        rx_wait_cnt    <= rx_wait_cnt + 32'd1;
                    end
                end

                // -------------------------------------------------------------
                // 6. S_DONE_IRQ: Hold Level-High IRQ until PS Driver ACKs
                // -------------------------------------------------------------
                S_DONE_IRQ: begin
                    eng_tx_tvalid <= 1'b0;
                    eng_tx_tlast  <= 1'b0;
                    if (status_write_pulse && status_write_data[0]) begin
                        ip_done_irq <= 1'b0;
                        if (err_crc || err_no_match || err_rx_timeout || (!irq_is_rx && mode_pub_only)) begin
                            irq_is_rx <= 1'b0;
                            rx_ptr    <= 0;
                            state     <= S_IDLE;
                        end else begin
                            state     <= S_WAIT_EGRESS;
                        end
                    end else if (trigger_detected) begin
                        ip_done_irq    <= 1'b0;
                        err_crc        <= 1'b0;
                        err_no_match   <= 1'b0;
                        err_rx_timeout <= 1'b0;
                        irq_is_rx      <= 1'b0;
                        rx_ptr         <= 0;
                        state          <= S_IDLE;
                    end
                end

                // -------------------------------------------------------------
                // 7. S_WAIT_EGRESS: Wait for Driver read() to prime S2MM & kick
                // -------------------------------------------------------------
                S_WAIT_EGRESS: begin
                    if (status_write_pulse && status_write_data[1]) begin
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
                // 8. S_TX_RUN: Pipelined S2MM Egress with Skid Buffer (Rule 1c)
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
                            bram_pipe_last  <= (egress_rd_ptr == egress_total_words - 1);
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
                                    skid_data  <= s2mm_dout;
                                    skid_last  <= bram_pipe_last;
                                    skid_valid <= 1'b1;
                                end else begin
                                    skid_valid <= 1'b0;
                                end
                            end
                        end else begin
                            if (m_axis_tready || !m_axis_tvalid) begin
                                if (bram_pipe_valid) begin
                                    m_axis_tdata  <= s2mm_dout;
                                    m_axis_tlast  <= bram_pipe_last;
                                    m_axis_tvalid <= 1'b1;
                                end else if (m_axis_tready && m_axis_tvalid) begin
                                    m_axis_tvalid <= 1'b0;
                                    m_axis_tlast  <= 1'b0;
                                end
                            end else begin
                                if (bram_pipe_valid) begin
                                    skid_data  <= s2mm_dout;
                                    skid_last  <= bram_pipe_last;
                                    skid_valid <= 1'b1;
                                end
                            end
                        end

                        if (tx_handshake && m_axis_tlast) begin
                            m_axis_tvalid <= 1'b0;
                            m_axis_tlast  <= 1'b0;
                            skid_valid    <= 1'b0;
                            irq_is_rx     <= 1'b0;
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
