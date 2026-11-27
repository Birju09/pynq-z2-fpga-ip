`timescale 1ns / 1ps

// =============================================================================
// Module: rtps_tx_engine
//
// Description:
//   DMA0 MM2S sample ingress, 8 KB TX Sample BRAM, on-the-fly CRC-32 &
//   payload checksum, Option C TX Fragmentation Planner (DATA_FRAG / IP-Frag),
//   and Master Cycle Sequencer FSM (PART B).
// =============================================================================

module rtps_tx_engine #(
    parameter integer DATA_WIDTH    = 32,
    parameter integer BRAM_DEPTH    = 2048,
    parameter integer ADDR_WIDTH    = 11,
    parameter integer NUM_ENDPOINTS = 4
)(
    input  wire                        clk,
    input  wire                        rst_n_local,

    // DMA0 MM2S Sample Stream In
    input  wire [DATA_WIDTH-1:0]       s_axis_tdata,
    input  wire [(DATA_WIDTH/8)-1:0]   s_axis_tkeep,
    input  wire                        s_axis_tvalid,
    output wire                        s_axis_tready,
    input  wire                        s_axis_tlast,

    // Wire / Emulator TX Stream Out
    output reg  [DATA_WIDTH-1:0]       eng_tx_tdata,
    output reg                         eng_tx_tvalid,
    input  wire                        eng_tx_tready,
    output reg                         eng_tx_tlast,

    // Control & Configuration from rtps_csr_edt
    input  wire [31:0]                 ctrl_reg,
    input  wire [31:0]                 expected_crc,
    input  wire [31:0]                 cfg_pacing_timer,
    input  wire [31:0]                 cfg_payload_words,
    input  wire [31:0]                 cfg_frag_size,
    input  wire [95:0]                 local_guid_prefix,
    input  wire [31:0]                 local_src_ip,
    input  wire [47:0]                 local_mac,
    input  wire [15:0]                 local_vendor_id,
    input  wire [63:0]                 ts_offset,
    input  wire [63:0]                 active_time_ns,
    input  wire                        lifecycle_ok_q,

    input  wire                        status_write_pulse,
    input  wire [31:0]                 status_write_data,
    input  wire                        trigger_detected,
    input  wire                        edt_commit_applied,
    input  wire                        identity_clear_seq_pulse,

    input  wire [NUM_ENDPOINTS*8*32-1:0] wr_edt_flat,
    input  wire [47:0]                 wr0_dst_mac,
    input  wire [NUM_ENDPOINTS-1:0]    wr_valid_vec,
    input  wire [NUM_ENDPOINTS*32-1:0] wr_seq_seed_flat,

    // Adaptive Small-TX Writer Select & Large TX DDR Ring Controls
    input  wire [1:0]                  tx_active_writer,
    input  wire [NUM_ENDPOINTS*32-1:0] tx_ring_base_flat,
    input  wire [NUM_ENDPOINTS*32-1:0] tx_ring_slot_size_flat,
    input  wire [NUM_ENDPOINTS*32-1:0] tx_ring_cfg_flat,
    input  wire [NUM_ENDPOINTS*32-1:0] tx_ring_submit_len_flat,
    input  wire [NUM_ENDPOINTS*5-1:0]  tx_ring_prod_idx_flat,
    input  wire [NUM_ENDPOINTS-1:0]    tx_ring_doorbell_pulse,
    input  wire [NUM_ENDPOINTS-1:0]    tx_ring_reset_pulse,
    output wire [NUM_ENDPOINTS*5-1:0]  tx_ring_cons_idx_flat,
    output wire [NUM_ENDPOINTS*32-1:0] tx_ring_stat_done_flat,
    input  wire [NUM_ENDPOINTS-1:0]    large_tx_irq_ack_pulse,
    output reg  [NUM_ENDPOINTS-1:0]    large_tx_done_irq_vec,

    // AXI4 Master Read Channel Interface (rtps_ddr_dma_engine)
    output reg                         ddr_rd_req_valid,
    output reg  [31:0]                 ddr_rd_req_addr,
    output reg  [9:0]                  ddr_rd_req_words,
    input  wire                        ddr_rd_data_valid,
    input  wire [31:0]                 ddr_rd_data_word,
    input  wire [9:0]                  ddr_rd_data_idx,
    input  wire                        ddr_rd_done_pulse,
    input  wire [15:0]                 ddr_rd_csum_fold,
    input  wire                        ddr_rd_resp_err,

    // RX Error Event
    input  wire                        rx_csum_err_pulse,

    // Interrupt & Status Outputs
    output reg                         ip_done_irq,
    output reg  [3:0]                  state,
    output reg  [31:0]                 crc_accum,
    output reg  [31:0]                 last_cycle_ticks,
    output reg  [15:0]                 completed_frags,
    output reg  [31:0]                 stat_tx_samples,
    output reg                         err_crc,
    output reg                         err_no_match,
    output reg                         err_csum
);

    `include "rtps_fastpath_defs.vh"

    localparam integer ETH_MTU_BYTES         = 1500;
    localparam integer IPV4_HDR_BYTES        = 20;
    localparam integer IPV4_PAYLOAD_MAX      = 1472;
    localparam integer IPV4_NONFINAL_PAYLOAD = 1472;
    localparam integer TX_DF_FRAG_MAX        = 1404;

    wire mode_sub_only   = ctrl_reg[9];
    wire tx_ip_frag_mode = ctrl_reg[10];

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

    function [31:0] bswap32;
        input [31:0] word_in;
        begin
            bswap32 = {word_in[7:0], word_in[15:8], word_in[23:16], word_in[31:24]};
        end
    endfunction

    // Unpack 4-Writer Active EDT
    wire [31:0] active_wr_edt [0:NUM_ENDPOINTS-1][0:7];
    genvar g_ep, g_w;
    generate
        for (g_ep = 0; g_ep < NUM_ENDPOINTS; g_ep = g_ep + 1) begin : GEN_WR_UNPACK
            for (g_w = 0; g_w < 8; g_w = g_w + 1) begin : GEN_WR_W
                assign active_wr_edt[g_ep][g_w] = wr_edt_flat[(g_ep*8 + g_w)*32 +: 32];
            end
        end
    endgenerate

    // Selected Active Writer Fields (registered at transaction/fragment start)
    reg  [1:0]  cur_wr_slot;
    reg  [31:0] cur_wr_dst_ip;
    reg  [15:0] cur_wr_src_port;
    reg  [15:0] cur_wr_dst_port;
    reg  [47:0] cur_wr_dst_mac;
    reg  [31:0] cur_wr_writer_id;
    reg  [31:0] cur_wr_reader_id;

    // -------------------------------------------------------------------------
    // TX Command Sample BRAM (2048 x 32-bit = 8 KB) & On-the-Fly Checksums
    // -------------------------------------------------------------------------
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] tx_sample_bram [0:BRAM_DEPTH-1];

    // Large-TX Fragment Staging BRAM (1024 x 32-bit = 4 KB) filled from DDR
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] tx_frag_ram [0:1023];
    reg  [9:0]            tx_frag_ram_rd_addr;
    reg  [DATA_WIDTH-1:0] tx_frag_ram_dout;
    reg                   cur_is_large_tx;
    reg                   small_tx_pending;

    always @(posedge clk) begin
        if (ddr_rd_data_valid) begin
            tx_frag_ram[ddr_rd_data_idx] <= ddr_rd_data_word;
        end
        tx_frag_ram_dout <= tx_frag_ram[tx_frag_ram_rd_addr];
    end

    reg [ADDR_WIDTH:0] rx_ptr;
    assign s_axis_tready = !small_tx_pending && !mode_sub_only && (state != S_CHECK_CRC) && (state != S_DONE_IRQ);
    wire   rx_handshake  = s_axis_tvalid && s_axis_tready;
    wire   axis_wr       = rx_handshake;

    reg  [ADDR_WIDTH-1:0] tx_sample_rd_addr;
    reg  [DATA_WIDTH-1:0] tx_sample_dout;

    always @(posedge clk) begin
        if (axis_wr) begin
            tx_sample_bram[rx_ptr[ADDR_WIDTH-1:0]] <= s_axis_tdata;
        end
    end

    reg [15:0] payload_csum_accum;
    always @(posedge clk) begin
        if (!rst_n_local) begin
            crc_accum          <= 32'd0;
            payload_csum_accum <= 16'h0000;
        end else if (axis_wr) begin
            if (rx_ptr == 0) begin
                crc_accum          <= s_axis_tdata;
                payload_csum_accum <= csum_add32(16'h0000, bswap32(s_axis_tdata));
            end else if (rx_ptr < cfg_payload_words) begin
                crc_accum          <= crc_accum + s_axis_tdata;
                payload_csum_accum <= csum_add32(payload_csum_accum, bswap32(s_axis_tdata));
            end
        end
    end

    // -------------------------------------------------------------------------
    // TX Fragmentation Planner & Sequencer Registers
    // -------------------------------------------------------------------------
    reg [63:0]         writer_seq_num [0:NUM_ENDPOINTS-1];
    reg                err_icmp;

    reg [ADDR_WIDTH:0] pkt_len;

    reg [15:0]         pacing_timer;
    reg [31:0]         cycle_tick_cnt;

    reg [15:0]         tx_ip_id_counter;
    reg [63:0]         latched_tx_ts;
    reg [63:0]         cur_tx_seq;
    reg [15:0]         tx_udp_csum;
    reg [15:0]         tx_frag_idx;
    reg [11:0]         tx_frag_words;
    reg [15:0]         tx_frag_ip_len;
    reg [15:0]         tx_frag_flags;
    reg [15:0]         tx_frag_hdr_csum;
    reg [11:0]         tx_dgram_word_idx;
    reg [11:0]         tx_beat_idx;
    reg [15:0]         tx_prev_half;

    reg [15:0]         tx_frag_payload_bytes;
    reg [15:0]         tx_frag_offset_bytes;
    reg [31:0]         tx_bytes_remaining;
    reg                tx_frag_more;

    reg [1:0]          tx_init_phase;
    reg [11:0]         tx_frag_ser_bytes;
    reg [11:0]         tx_frag_bram_base;
    reg [11:0]         tx_frag_bram_words;
    reg [11:0]         tx_scan_cnt;
    reg [31:0]         tx_scan_csum;
    wire [31:0]        tx_scan_word_be = bswap32(tx_sample_dout);
    reg [15:0]         tx_frag_csum_seed;

    // Per-Endpoint Large TX DDR Ring State (ep = 0..3)
    reg        ltx_active      [0:NUM_ENDPOINTS-1];
    reg [4:0]  ltx_cons_idx    [0:NUM_ENDPOINTS-1];
    reg [31:0] ltx_slot_base   [0:NUM_ENDPOINTS-1];
    reg [31:0] ltx_byte_off    [0:NUM_ENDPOINTS-1];
    reg [31:0] ltx_bytes_rem   [0:NUM_ENDPOINTS-1];
    reg [31:0] ltx_total_bytes [0:NUM_ENDPOINTS-1];
    reg [15:0] ltx_frag_idx    [0:NUM_ENDPOINTS-1];
    reg [63:0] ltx_seq_num     [0:NUM_ENDPOINTS-1];
    reg [63:0] ltx_ts          [0:NUM_ENDPOINTS-1];
    reg [31:0] ltx_stat_done   [0:NUM_ENDPOINTS-1];
    reg [31:0] tx_slot_len_lut [0:NUM_ENDPOINTS-1][0:15];
    reg [1:0]  rr_ptr;
    reg [31:0] cur_ltx_total_bytes;

    // Unpack ring CSRs per endpoint
    wire [31:0] tx_ring_base_ep      [0:NUM_ENDPOINTS-1];
    wire [31:0] tx_ring_slot_size_ep [0:NUM_ENDPOINTS-1];
    wire [4:0]  tx_ring_depth_ep     [0:NUM_ENDPOINTS-1];
    wire        tx_ring_enable_ep    [0:NUM_ENDPOINTS-1];
    wire [31:0] tx_ring_submit_len_ep[0:NUM_ENDPOINTS-1];
    wire [4:0]  tx_ring_prod_idx_ep  [0:NUM_ENDPOINTS-1];
    wire [3:0]  ltx_pend_vec;

    generate
        for (g_ep = 0; g_ep < NUM_ENDPOINTS; g_ep = g_ep + 1) begin : GEN_LTX_CSR
            assign tx_ring_base_ep[g_ep]       = tx_ring_base_flat[g_ep*32 +: 32];
            assign tx_ring_slot_size_ep[g_ep]  = tx_ring_slot_size_flat[g_ep*32 +: 32];
            assign tx_ring_depth_ep[g_ep]      = tx_ring_cfg_flat[g_ep*32 +: 5];
            assign tx_ring_enable_ep[g_ep]     = tx_ring_cfg_flat[g_ep*32 + 8];
            assign tx_ring_submit_len_ep[g_ep] = tx_ring_submit_len_flat[g_ep*32 +: 32];
            assign tx_ring_prod_idx_ep[g_ep]   = tx_ring_prod_idx_flat[g_ep*5 +: 5];
            assign tx_ring_cons_idx_flat[g_ep*5 +: 5]   = ltx_cons_idx[g_ep];
            assign tx_ring_stat_done_flat[g_ep*32 +: 32] = ltx_stat_done[g_ep];
            assign ltx_pend_vec[g_ep] = tx_ring_enable_ep[g_ep] &&
                                        (ltx_active[g_ep] || (tx_ring_prod_idx_ep[g_ep] != ltx_cons_idx[g_ep]));
        end
    endgenerate

    reg [15:0] tx_eff_frag_bytes;
    always @(posedge clk) begin
        tx_eff_frag_bytes <= (cfg_frag_size[15:0] < 16'd4)          ? 16'd4 :
                             (cfg_frag_size[15:0] > TX_DF_FRAG_MAX) ? TX_DF_FRAG_MAX[15:0] :
                             {cfg_frag_size[15:2], 2'b00};
    end

    wire [31:0] small_sample_total_bytes = {18'd0, pkt_len[11:0], 2'b00} + 32'd4;
    wire [31:0] tx_sample_total_bytes    = cur_is_large_tx ? cur_ltx_total_bytes : small_sample_total_bytes;
    wire        tx_use_single_data       = (!cur_is_large_tx) && (!tx_ip_frag_mode) && (small_sample_total_bytes <= {16'd0, tx_eff_frag_bytes});
    wire        tx_use_data_frag         = cur_is_large_tx || ((!tx_ip_frag_mode) && (small_sample_total_bytes > {16'd0, tx_eff_frag_bytes}));

    wire [15:0] tx_total_dgram_words = pkt_len[11:0] + 12'd17;
    wire [15:0] tx_udp_total_bytes   = {2'b00, tx_total_dgram_words[11:0], 2'b00};
    wire [15:0] tx_data_octets_le    = {2'b00, pkt_len[11:0], 2'b00} + 16'd24;
    wire [15:0] tx_data_octets_wire  = {tx_data_octets_le[7:0], tx_data_octets_le[15:8]};

    wire [15:0] tx_df_octets_le        = {4'd0, tx_frag_ser_bytes} + 16'd32;
    wire [15:0] tx_df_octets_wire      = {tx_df_octets_le[7:0], tx_df_octets_le[15:8]};
    wire [31:0] tx_df_start_num        = {16'd0, tx_frag_idx} + 32'd1;
    wire [31:0] tx_df_start_wire       = bswap32(tx_df_start_num);
    wire [15:0] tx_df_frag_size_wire   = {tx_eff_frag_bytes[7:0], tx_eff_frag_bytes[15:8]};
    wire [31:0] tx_df_sample_size_wire = bswap32(tx_sample_total_bytes);

    wire [31:0] raw_sample_dout = cur_is_large_tx ? tx_frag_ram_dout : tx_sample_dout;
    reg [DATA_WIDTH-1:0] tx_sample_buf;
    always @(posedge clk) begin
        if (!rst_n_local) begin
            tx_sample_buf <= 32'd0;
        end else if (pacing_timer == cfg_pacing_timer[15:0]) begin
            tx_sample_buf <= raw_sample_dout;
        end
    end
    wire [31:0] tx_sample_word = (cfg_pacing_timer == 32'd0) ? raw_sample_dout : tx_sample_buf;

    wire tx_beat_load = (state == S_TX_FRAG_SEND) && (!eng_tx_tvalid || eng_tx_tready) && (pacing_timer == 16'd0);
    always @(posedge clk) begin
        if (state != S_TX_FRAG_SEND || tx_beat_load) begin
            tx_sample_dout <= tx_sample_bram[tx_sample_rd_addr];
        end
    end

    reg [31:0] cur_dgram_word;
    always @(*) begin
        if (tx_use_data_frag) begin
            case (tx_dgram_word_idx)
                12'd0:  cur_dgram_word = {cur_wr_src_port, cur_wr_dst_port};
                12'd1:  cur_dgram_word = {tx_frag_payload_bytes, tx_udp_csum};
                12'd2:  cur_dgram_word = 32'h5254_5053;
                12'd3:  cur_dgram_word = {16'h0201, local_vendor_id};
                12'd4:  cur_dgram_word = local_guid_prefix[95:64];
                12'd5:  cur_dgram_word = local_guid_prefix[63:32];
                12'd6:  cur_dgram_word = local_guid_prefix[31:0];
                12'd7:  cur_dgram_word = 32'h0901_0800;
                12'd8:  cur_dgram_word = {latched_tx_ts[39:32], latched_tx_ts[47:40], latched_tx_ts[55:48], latched_tx_ts[63:56]};
                12'd9:  cur_dgram_word = {latched_tx_ts[7:0],   latched_tx_ts[15:8],  latched_tx_ts[23:16], latched_tx_ts[31:24]};
                12'd10: cur_dgram_word = {8'h16, 8'h01, tx_df_octets_wire};
                12'd11: cur_dgram_word = 32'h0000_1C00;
                12'd12: cur_dgram_word = cur_wr_reader_id;
                12'd13: cur_dgram_word = cur_wr_writer_id;
                12'd14: cur_dgram_word = {cur_tx_seq[39:32], cur_tx_seq[47:40], cur_tx_seq[55:48], cur_tx_seq[63:56]};
                12'd15: cur_dgram_word = {cur_tx_seq[7:0],   cur_tx_seq[15:8],  cur_tx_seq[23:16], cur_tx_seq[31:24]};
                12'd16: cur_dgram_word = tx_df_start_wire;
                12'd17: cur_dgram_word = {16'h0100, tx_df_frag_size_wire};
                12'd18: cur_dgram_word = tx_df_sample_size_wire;
                12'd19: cur_dgram_word = (tx_frag_idx == 16'd0) ? 32'h0001_0000 : bswap32(tx_sample_word);
                default: cur_dgram_word = bswap32(tx_sample_word);
            endcase
        end else begin
            case (tx_dgram_word_idx)
                12'd0:  cur_dgram_word = {cur_wr_src_port, cur_wr_dst_port};
                12'd1:  cur_dgram_word = {tx_udp_total_bytes, tx_udp_csum};
                12'd2:  cur_dgram_word = 32'h5254_5053;
                12'd3:  cur_dgram_word = {16'h0201, local_vendor_id};
                12'd4:  cur_dgram_word = local_guid_prefix[95:64];
                12'd5:  cur_dgram_word = local_guid_prefix[63:32];
                12'd6:  cur_dgram_word = local_guid_prefix[31:0];
                12'd7:  cur_dgram_word = 32'h0901_0800;
                12'd8:  cur_dgram_word = {latched_tx_ts[39:32], latched_tx_ts[47:40], latched_tx_ts[55:48], latched_tx_ts[63:56]};
                12'd9:  cur_dgram_word = {latched_tx_ts[7:0],   latched_tx_ts[15:8],  latched_tx_ts[23:16], latched_tx_ts[31:24]};
                12'd10: cur_dgram_word = {8'h15, 8'h05, tx_data_octets_wire};
                12'd11: cur_dgram_word = 32'h0000_1000;
                12'd12: cur_dgram_word = cur_wr_reader_id;
                12'd13: cur_dgram_word = cur_wr_writer_id;
                12'd14: cur_dgram_word = {cur_tx_seq[39:32], cur_tx_seq[47:40], cur_tx_seq[55:48], cur_tx_seq[63:56]};
                12'd15: cur_dgram_word = {cur_tx_seq[7:0],   cur_tx_seq[15:8],  cur_tx_seq[23:16], cur_tx_seq[31:24]};
                12'd16: cur_dgram_word = 32'h0001_0000;
                default: cur_dgram_word = bswap32(tx_sample_word);
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Pipelined UDP & IPv4 Checksum Submodule
    // -------------------------------------------------------------------------
    wire [15:0] calc_tx_udp_csum;
    wire [15:0] calc_tx_ip_hdr_csum;

    rtps_tx_csum_pipe u_tx_csum_pipe (
        .clk                    (clk),
        .tx_use_data_frag       (tx_use_data_frag),
        .tx_ip_frag_mode        (tx_ip_frag_mode),
        .tx_frag_payload_bytes  (tx_frag_payload_bytes),
        .tx_udp_total_bytes     (tx_udp_total_bytes),
        .payload_csum_accum     (payload_csum_accum),
        .tx_frag_csum_seed      (tx_frag_csum_seed),
        .local_src_ip           (local_src_ip),
        .wr0_dst_ip             (cur_wr_dst_ip),
        .wr0_src_port           (cur_wr_src_port),
        .wr0_dst_port           (cur_wr_dst_port),
        .local_vendor_id        (local_vendor_id),
        .local_guid_prefix      (local_guid_prefix),
        .tx_df_octets_wire      (tx_df_octets_wire),
        .tx_df_start_wire       (tx_df_start_wire),
        .tx_df_frag_size_wire   (tx_df_frag_size_wire),
        .tx_df_sample_size_wire (tx_df_sample_size_wire),
        .tx_data_octets_wire    (tx_data_octets_wire),
        .latched_tx_ts          (latched_tx_ts),
        .wr0_reader_id          (cur_wr_reader_id),
        .wr0_writer_id          (cur_wr_writer_id),
        .cur_tx_seq             (cur_tx_seq),
        .tx_frag_ip_len         (tx_frag_ip_len),
        .tx_ip_id_counter       (tx_ip_id_counter),
        .tx_frag_flags          (tx_frag_flags),
        .calc_tx_udp_csum       (calc_tx_udp_csum),
        .calc_tx_ip_hdr_csum    (calc_tx_ip_hdr_csum)
    );

    reg [31:0] next_wire_tx_word;
    always @(*) begin
        case (tx_beat_idx)
            12'd0: next_wire_tx_word = cur_wr_dst_mac[47:16];
            12'd1: next_wire_tx_word = {cur_wr_dst_mac[15:0], local_mac[47:32]};
            12'd2: next_wire_tx_word = local_mac[31:0];
            12'd3: next_wire_tx_word = 32'h0800_4500;
            12'd4: next_wire_tx_word = {tx_frag_ip_len, tx_ip_id_counter};
            12'd5: next_wire_tx_word = {tx_frag_flags, 16'h4011};
            12'd6: next_wire_tx_word = {tx_frag_hdr_csum, local_src_ip[31:16]};
            12'd7: next_wire_tx_word = {local_src_ip[15:0], cur_wr_dst_ip[31:16]};
            12'd8: next_wire_tx_word = {cur_wr_dst_ip[15:0], cur_dgram_word[31:16]};
            default: begin
                if (tx_beat_idx == (12'd8 + tx_frag_words))
                    next_wire_tx_word = {tx_prev_half, 16'h0000};
                else
                    next_wire_tx_word = {tx_prev_half, cur_dgram_word[31:16]};
            end
        endcase
    end

    // -------------------------------------------------------------------------
    // Master Cycle Sequencer FSM (Always Asynchronous TX, Fragment-Interleaved)
    // -------------------------------------------------------------------------
    reg ltx_fetch_issued;

    integer rst_ep;
    always @(posedge clk) begin
        if (!rst_n_local) begin
            state                  <= S_IDLE;
            rx_ptr                 <= 0;
            pkt_len                <= 0;
            small_tx_pending       <= 1'b0;
            cur_is_large_tx        <= 1'b0;
            cur_wr_slot            <= 2'd0;
            cur_wr_dst_ip          <= 32'd0;
            cur_wr_src_port        <= 16'd0;
            cur_wr_dst_port        <= 16'd0;
            cur_wr_dst_mac         <= 48'd0;
            cur_wr_writer_id       <= 32'd0;
            cur_wr_reader_id       <= 32'd0;
            cur_ltx_total_bytes    <= 32'd0;
            rr_ptr                 <= 2'd0;
            ltx_fetch_issued       <= 1'b0;
            ddr_rd_req_valid       <= 1'b0;
            ddr_rd_req_addr        <= 32'd0;
            ddr_rd_req_words       <= 10'd0;
            tx_frag_ram_rd_addr    <= 10'd0;
            large_tx_done_irq_vec  <= {NUM_ENDPOINTS{1'b0}};
            pacing_timer           <= 16'd0;
            cycle_tick_cnt         <= 32'd0;
            last_cycle_ticks       <= 32'd0;

            ip_done_irq            <= 1'b0;
            err_crc                <= 1'b0;
            err_no_match           <= 1'b0;
            err_csum               <= 1'b0;
            completed_frags        <= 16'd0;

            tx_ip_id_counter       <= 16'h1000;
            latched_tx_ts          <= 64'd0;
            cur_tx_seq             <= 64'd1;
            tx_udp_csum            <= 16'd0;
            tx_frag_idx            <= 16'd0;
            tx_frag_words          <= 12'd0;
            tx_frag_ip_len         <= 16'd0;
            tx_frag_flags          <= 16'd0;
            tx_frag_hdr_csum       <= 16'd0;
            tx_dgram_word_idx      <= 12'd0;
            tx_beat_idx            <= 12'd0;
            tx_prev_half           <= 16'd0;
            tx_sample_rd_addr      <= {ADDR_WIDTH{1'b0}};
            tx_init_phase          <= 2'd0;
            tx_frag_ser_bytes      <= 12'd0;
            tx_frag_bram_base      <= 12'd0;
            tx_frag_bram_words     <= 12'd0;
            tx_scan_cnt            <= 12'd0;
            tx_scan_csum           <= 16'd0;
            tx_frag_csum_seed      <= 16'd0;

            eng_tx_tdata           <= 32'd0;
            eng_tx_tvalid          <= 1'b0;
            eng_tx_tlast           <= 1'b0;
            stat_tx_samples        <= 32'd0;

            tx_frag_payload_bytes  <= 16'd0;
            tx_frag_offset_bytes   <= 16'd0;
            tx_bytes_remaining     <= 32'd0;
            tx_frag_more           <= 1'b0;

            for (rst_ep = 0; rst_ep < NUM_ENDPOINTS; rst_ep = rst_ep + 1) begin
                writer_seq_num[rst_ep]  <= 64'd1;
                ltx_active[rst_ep]      <= 1'b0;
                ltx_cons_idx[rst_ep]    <= 5'd0;
                ltx_slot_base[rst_ep]   <= 32'd0;
                ltx_byte_off[rst_ep]    <= 32'd0;
                ltx_bytes_rem[rst_ep]   <= 32'd0;
                ltx_total_bytes[rst_ep] <= 32'd0;
                ltx_frag_idx[rst_ep]    <= 16'd0;
                ltx_seq_num[rst_ep]     <= 64'd1;
                ltx_ts[rst_ep]          <= 64'd0;
                ltx_stat_done[rst_ep]   <= 32'd0;
            end
        end else begin
            ddr_rd_req_valid <= 1'b0;

            if (rx_csum_err_pulse) begin
                err_csum <= 1'b1;
            end

            // Background MM2S small-sample ingress (non-blocking during large TX)
            if (rx_handshake && !mode_sub_only) begin
                if (s_axis_tlast || (rx_ptr == cfg_payload_words - 1)) begin
                    pkt_len          <= {1'b0, rx_ptr[ADDR_WIDTH-1:0]} + 1'b1;
                    rx_ptr           <= 0;
                    small_tx_pending <= 1'b1;
                end else begin
                    rx_ptr           <= rx_ptr + 1'b1;
                end
            end

            // Per-endpoint Large TX ring doorbell, reset, and IRQ ACK
            for (rst_ep = 0; rst_ep < NUM_ENDPOINTS; rst_ep = rst_ep + 1) begin
                if (large_tx_irq_ack_pulse[rst_ep]) begin
                    large_tx_done_irq_vec[rst_ep] <= 1'b0;
                end
                if (tx_ring_reset_pulse[rst_ep]) begin
                    ltx_active[rst_ep]    <= 1'b0;
                    ltx_cons_idx[rst_ep]  <= 5'd0;
                    ltx_slot_base[rst_ep] <= tx_ring_base_ep[rst_ep];
                end else if (tx_ring_doorbell_pulse[rst_ep]) begin : ltx_db_blk
                    reg [3:0] sub_slot;
                    sub_slot = (tx_ring_prod_idx_ep[rst_ep] == 5'd0) ?
                               (tx_ring_depth_ep[rst_ep][3:0] - 4'd1) :
                               (tx_ring_prod_idx_ep[rst_ep][3:0] - 4'd1);
                    tx_slot_len_lut[rst_ep][sub_slot] <= tx_ring_submit_len_ep[rst_ep];
                end
            end

            if (edt_commit_applied) begin
                for (rst_ep = 0; rst_ep < NUM_ENDPOINTS; rst_ep = rst_ep + 1) begin
                    if (wr_valid_vec[rst_ep] && (wr_seq_seed_flat[rst_ep*32 +: 32] != 32'd0)) begin
                        writer_seq_num[rst_ep] <= {32'd0, wr_seq_seed_flat[rst_ep*32 +: 32]};
                    end
                end
            end

            if (identity_clear_seq_pulse) begin
                for (rst_ep = 0; rst_ep < NUM_ENDPOINTS; rst_ep = rst_ep + 1) begin
                    writer_seq_num[rst_ep] <= 64'd1;
                end
            end

            case (state)
                // -------------------------------------------------------------
                // 1. S_IDLE: Wait for DMA0 MM2S, manual CSR kick, or Large TX Ring
                // -------------------------------------------------------------
                S_IDLE: begin
                    ip_done_irq     <= 1'b0;
                    eng_tx_tvalid   <= 1'b0;
                    eng_tx_tlast    <= 1'b0;
                    cur_is_large_tx <= 1'b0;

                    if (small_tx_pending && !mode_sub_only) begin
                        state <= S_TX_ARB;
                    end else if ((trigger_detected || (status_write_pulse && status_write_data[2])) && !mode_sub_only) begin
                        pkt_len          <= cfg_payload_words[ADDR_WIDTH:0];
                        rx_ptr           <= 0;
                        small_tx_pending <= 1'b1;
                        state            <= S_TX_ARB;
                    end else if (|ltx_pend_vec) begin
                        state <= S_TX_ARB;
                    end
                end

                // -------------------------------------------------------------
                // 2. S_TX_ARB: Fragment-Boundary Interleaving Arbiter
                //    Priority 1: Small <=8KB BRAM TX (preempts between fragments)
                //    Priority 2: Round-Robin Large >8KB DDR Ring TX (ep 0..3)
                // -------------------------------------------------------------
                S_TX_ARB: begin
                    if (eng_tx_tvalid && eng_tx_tready) begin
                        eng_tx_tvalid <= 1'b0;
                        eng_tx_tlast  <= 1'b0;
                    end

                    if (trigger_detected) begin
                        small_tx_pending <= 1'b0;
                        state            <= S_IDLE;
                    end else if (small_tx_pending && !mode_sub_only) begin
                        small_tx_pending <= 1'b0;
                        cur_is_large_tx  <= 1'b0;
                        cur_wr_slot      <= tx_active_writer;
                        cur_wr_dst_ip    <= active_wr_edt[tx_active_writer][1];
                        cur_wr_src_port  <= active_wr_edt[tx_active_writer][2][31:16];
                        cur_wr_dst_port  <= active_wr_edt[tx_active_writer][2][15:0];
                        cur_wr_dst_mac   <= (tx_active_writer == 2'd0) ? wr0_dst_mac :
                                            {active_wr_edt[tx_active_writer][4][15:0], active_wr_edt[tx_active_writer][3]};
                        cur_wr_writer_id <= active_wr_edt[tx_active_writer][5];
                        cur_wr_reader_id <= active_wr_edt[tx_active_writer][6];
                        state            <= S_CHECK_CRC;
                    end else if (|ltx_pend_vec) begin : arb_rr_blk
                        reg [1:0] cand;
                        if      (ltx_pend_vec[rr_ptr])              cand = rr_ptr;
                        else if (ltx_pend_vec[rr_ptr + 2'd1])       cand = rr_ptr + 2'd1;
                        else if (ltx_pend_vec[rr_ptr + 2'd2])       cand = rr_ptr + 2'd2;
                        else                                        cand = rr_ptr + 2'd3;

                        cur_is_large_tx  <= 1'b1;
                        cur_wr_slot      <= cand;
                        rr_ptr           <= cand + 2'd1;
                        cur_wr_dst_ip    <= active_wr_edt[cand][1];
                        cur_wr_src_port  <= active_wr_edt[cand][2][31:16];
                        cur_wr_dst_port  <= active_wr_edt[cand][2][15:0];
                        cur_wr_dst_mac   <= (cand == 2'd0) ? wr0_dst_mac :
                                            {active_wr_edt[cand][4][15:0], active_wr_edt[cand][3]};
                        cur_wr_writer_id <= active_wr_edt[cand][5];
                        cur_wr_reader_id <= active_wr_edt[cand][6];

                        if (!ltx_active[cand]) begin
                            ltx_active[cand]      <= 1'b1;
                            if (ltx_cons_idx[cand] == 5'd0)
                                ltx_slot_base[cand] <= tx_ring_base_ep[cand];
                            ltx_byte_off[cand]    <= 32'd0;
                            ltx_total_bytes[cand] <= tx_slot_len_lut[cand][ltx_cons_idx[cand][3:0]] + 32'd4;
                            ltx_bytes_rem[cand]   <= tx_slot_len_lut[cand][ltx_cons_idx[cand][3:0]] + 32'd4;
                            ltx_frag_idx[cand]    <= 16'd0;
                            ltx_seq_num[cand]     <= writer_seq_num[cand];
                            writer_seq_num[cand]  <= writer_seq_num[cand] + 64'd1;
                            ltx_ts[cand]          <= active_time_ns + ts_offset;
                        end
                        ltx_fetch_issued <= 1'b0;
                        state            <= S_LTX_DDR_FETCH;
                    end else begin
                        state <= S_IDLE;
                    end
                end

                // -------------------------------------------------------------
                // 3. S_LTX_DDR_FETCH: Pull 1 Fragment from DDR Ring via m_axi_ddr
                // -------------------------------------------------------------
                S_LTX_DDR_FETCH: begin
                    if (eng_tx_tvalid && eng_tx_tready) begin
                        eng_tx_tvalid <= 1'b0;
                        eng_tx_tlast  <= 1'b0;
                    end

                    if (trigger_detected) begin
                        ltx_fetch_issued <= 1'b0;
                        state            <= S_IDLE;
                    end else if (!ltx_fetch_issued) begin : ltx_req_blk
                        reg [11:0] ser_b;
                        ser_b = (ltx_bytes_rem[cur_wr_slot] > {16'd0, tx_eff_frag_bytes}) ?
                                tx_eff_frag_bytes[11:0] : ltx_bytes_rem[cur_wr_slot][11:0];

                        cur_ltx_total_bytes   <= ltx_total_bytes[cur_wr_slot];
                        cur_tx_seq            <= ltx_seq_num[cur_wr_slot];
                        latched_tx_ts         <= ltx_ts[cur_wr_slot];
                        tx_frag_idx           <= ltx_frag_idx[cur_wr_slot];
                        tx_bytes_remaining    <= ltx_bytes_rem[cur_wr_slot];
                        tx_frag_ser_bytes     <= ser_b;
                        tx_frag_more          <= (ltx_bytes_rem[cur_wr_slot] > {16'd0, tx_eff_frag_bytes});
                        tx_frag_payload_bytes <= 16'd76 + {4'd0, ser_b};
                        tx_frag_words         <= 12'd19 + {2'b00, ser_b[11:2]};
                        tx_frag_ip_len        <= 16'd96 + {4'd0, ser_b};
                        tx_frag_flags         <= 16'd0;

                        ddr_rd_req_valid      <= 1'b1;
                        ddr_rd_req_addr       <= ltx_slot_base[cur_wr_slot] + ltx_byte_off[cur_wr_slot];
                        ddr_rd_req_words      <= (ltx_frag_idx[cur_wr_slot] == 16'd0) ?
                                                 ({1'b0, ser_b[11:2]} - 10'd1) : {1'b0, ser_b[11:2]};
                        ltx_fetch_issued      <= 1'b1;
                    end else if (ddr_rd_done_pulse) begin
                        tx_frag_csum_seed   <= (tx_frag_idx == 16'd0) ?
                                               csum_add32(ddr_rd_csum_fold, 32'h0001_0000) :
                                               ddr_rd_csum_fold;
                        tx_frag_ram_rd_addr <= 10'd0;
                        tx_dgram_word_idx   <= 12'd0;
                        tx_beat_idx         <= 12'd0;
                        eng_tx_tvalid       <= 1'b0;
                        eng_tx_tlast        <= 1'b0;
                        pacing_timer        <= cfg_pacing_timer[15:0];
                        ltx_fetch_issued    <= 1'b0;
                        state               <= S_TX_FRAG_SEND;
                    end
                end

                // -------------------------------------------------------------
                // 4. S_CHECK_CRC: Verify DMA0 CRC, Check Lifecycle, Init UDP Csum
                // -------------------------------------------------------------
                S_CHECK_CRC: begin
                    latched_tx_ts  <= active_time_ns + ts_offset;
                    cur_tx_seq     <= writer_seq_num[cur_wr_slot];
                    cycle_tick_cnt <= 32'd0;

                    if (!lifecycle_ok_q) begin
                        err_no_match   <= 1'b1;
                        err_crc        <= 1'b0;
                        ip_done_irq    <= 1'b1;
                        state          <= S_DONE_IRQ;
                    end else if ((crc_accum == expected_crc) || (expected_crc == 32'h0)) begin
                        err_crc              <= 1'b0;
                        err_no_match         <= 1'b0;
                        err_csum             <= 1'b0;
                        completed_frags      <= 16'd0;
                        tx_frag_idx          <= 16'd0;
                        tx_bytes_remaining   <= tx_ip_frag_mode ? {16'd0, tx_udp_total_bytes} : small_sample_total_bytes;
                        tx_frag_offset_bytes <= 16'd0;
                        tx_init_phase        <= 2'd0;
                        tx_ip_id_counter     <= tx_ip_id_counter + 16'd1;
                        writer_seq_num[cur_wr_slot] <= writer_seq_num[cur_wr_slot] + 64'd1;
                        state                <= S_TX_FRAG_INIT;
                    end else begin
                        err_crc     <= 1'b1;
                        ip_done_irq <= 1'b1;
                        state       <= S_DONE_IRQ;
                    end
                end

                // -------------------------------------------------------------
                // 3. S_TX_FRAG_INIT: Option C Fragmentation Planner (DATA_FRAG / IP-Frag)
                // -------------------------------------------------------------
                S_TX_FRAG_INIT: begin
                    cycle_tick_cnt <= cycle_tick_cnt + 32'd1;

                    if (eng_tx_tvalid && eng_tx_tready) begin
                        eng_tx_tvalid <= 1'b0;
                        eng_tx_tlast  <= 1'b0;
                    end

                    if (!(eng_tx_tvalid && !eng_tx_tready)) begin
                        case (tx_init_phase)
                            2'd0: begin
                                    if (tx_ip_frag_mode) begin
                                    if (tx_bytes_remaining > IPV4_NONFINAL_PAYLOAD) begin
                                        tx_frag_payload_bytes <= IPV4_NONFINAL_PAYLOAD;
                                        tx_frag_more          <= 1'b1;
                                        tx_frag_words         <= (IPV4_NONFINAL_PAYLOAD >> 2);
                                        tx_frag_ip_len        <= IPV4_HDR_BYTES + IPV4_NONFINAL_PAYLOAD;
                                        tx_frag_flags         <= {2'b00, 1'b1, tx_frag_offset_bytes[15:3]};
                                    end else begin
                                        tx_frag_payload_bytes <= tx_bytes_remaining[15:0];
                                        tx_frag_more          <= 1'b0;
                                        tx_frag_words         <= (tx_bytes_remaining[15:0] + 16'd3) >> 2;
                                        tx_frag_ip_len        <= IPV4_HDR_BYTES + tx_bytes_remaining[15:0];
                                        tx_frag_flags         <= {2'b00, 1'b0, tx_frag_offset_bytes[15:3]};
                                    end
                                    tx_dgram_word_idx <= tx_frag_offset_bytes >> 2;
                                    if ((tx_frag_offset_bytes >> 2) >= 12'd17)
                                        tx_sample_rd_addr <= (tx_frag_offset_bytes >> 2) - 12'd17;
                                    else
                                        tx_sample_rd_addr <= 11'd0;
                                    tx_beat_idx   <= 12'd0;
                                    eng_tx_tvalid <= 1'b0;
                                    eng_tx_tlast  <= 1'b0;
                                    pacing_timer  <= cfg_pacing_timer[15:0];
                                    state         <= S_TX_FRAG_SEND;
                                end else if (tx_use_single_data) begin
                                    tx_frag_payload_bytes <= tx_udp_total_bytes;
                                    tx_frag_more          <= 1'b0;
                                    tx_frag_words         <= tx_total_dgram_words[11:0];
                                    tx_frag_ip_len        <= IPV4_HDR_BYTES + tx_udp_total_bytes;
                                    tx_frag_flags         <= 16'd0;
                                    tx_frag_ser_bytes     <= tx_sample_total_bytes[11:0];
                                    tx_frag_bram_base     <= 12'd0;
                                    tx_frag_bram_words    <= pkt_len[11:0];
                                    tx_scan_csum          <= 16'd0;
                                    tx_scan_cnt           <= 12'd0;
                                    tx_sample_rd_addr     <= {ADDR_WIDTH{1'b0}};
                                    tx_init_phase         <= 2'd1;
                                end else begin : df_init_blk
                                    reg [11:0] ser_bytes;
                                    ser_bytes = (tx_bytes_remaining > tx_eff_frag_bytes) ?
                                                tx_eff_frag_bytes[11:0] : tx_bytes_remaining[11:0];
                                    tx_frag_ser_bytes     <= ser_bytes;
                                    tx_frag_more          <= (tx_bytes_remaining > tx_eff_frag_bytes);
                                    tx_frag_payload_bytes <= 16'd76 + {4'd0, ser_bytes};
                                    tx_frag_words         <= 12'd19 + {2'b00, ser_bytes[11:2]};
                                    tx_frag_ip_len        <= 16'd96 + {4'd0, ser_bytes};
                                    tx_frag_flags         <= 16'd0;
                                    tx_frag_bram_base     <= (tx_frag_idx == 16'd0) ? 12'd0 : (tx_frag_offset_bytes[13:2] - 12'd1);
                                    tx_frag_bram_words    <= (tx_frag_idx == 16'd0) ? (ser_bytes[11:2] - 12'd1) : ser_bytes[11:2];
                                    tx_scan_csum          <= (tx_frag_idx == 16'd0) ? 16'h0001 : 16'h0000;
                                    tx_scan_cnt           <= 12'd0;
                                    tx_sample_rd_addr     <= (tx_frag_idx == 16'd0) ? {ADDR_WIDTH{1'b0}} :
                                                             (tx_frag_offset_bytes[ADDR_WIDTH+1:2] - 11'd1);
                                    tx_init_phase         <= 2'd1;
                                end
                            end
                            2'd1: begin
                                if (tx_frag_bram_words == 12'd0) begin
                                    tx_frag_csum_seed <= csum_add32(16'd0, tx_scan_csum);
                                    tx_init_phase     <= 2'd3;
                                end else begin
                                    tx_sample_rd_addr <= tx_sample_rd_addr + 11'd1;
                                    tx_init_phase     <= 2'd2;
                                end
                            end
                            2'd2: begin
                                tx_scan_csum <= tx_scan_csum + {16'd0, tx_scan_word_be[31:16]} + {16'd0, tx_scan_word_be[15:0]};
                                if (tx_scan_cnt == (tx_frag_bram_words - 12'd1)) begin
                                    tx_init_phase <= 2'd3;
                                end else begin
                                    tx_scan_cnt       <= tx_scan_cnt + 12'd1;
                                    tx_sample_rd_addr <= tx_sample_rd_addr + 11'd1;
                                end
                            end
                            default: begin
                                tx_frag_csum_seed <= csum_add32(16'd0, tx_scan_csum);
                                tx_sample_rd_addr <= tx_frag_bram_base[ADDR_WIDTH-1:0];
                                tx_dgram_word_idx <= 12'd0;
                                tx_beat_idx       <= 12'd0;
                                eng_tx_tvalid     <= 1'b0;
                                eng_tx_tlast      <= 1'b0;
                                pacing_timer      <= cfg_pacing_timer[15:0];
                                tx_init_phase     <= 2'd0;
                                state             <= S_TX_FRAG_SEND;
                            end
                        endcase
                    end
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
                        if ((tx_frag_idx == 16'd0 || !tx_ip_frag_mode) && tx_beat_idx == 12'd4) begin
                            tx_udp_csum <= calc_tx_udp_csum;
                        end

                        if (!eng_tx_tvalid || eng_tx_tready) begin
                            if (pacing_timer == 16'd0) begin
                                pacing_timer  <= cfg_pacing_timer[15:0];
                                eng_tx_tdata  <= next_wire_tx_word;
                                eng_tx_tvalid <= 1'b1;
                                eng_tx_tlast  <= (tx_beat_idx == (12'd8 + tx_frag_words));

                                if (tx_beat_idx == (12'd8 + tx_frag_words)) begin
                                    completed_frags <= completed_frags + 16'd1;
                                    if (cur_is_large_tx) begin
                                        tx_ip_id_counter <= tx_ip_id_counter + 16'd1;
                                        if (!tx_frag_more) begin : ltx_done_blk
                                            reg [4:0] next_cons;
                                            next_cons = (ltx_cons_idx[cur_wr_slot] + 5'd1 >= tx_ring_depth_ep[cur_wr_slot]) ?
                                                        5'd0 : (ltx_cons_idx[cur_wr_slot] + 5'd1);
                                            ltx_active[cur_wr_slot]            <= 1'b0;
                                            ltx_cons_idx[cur_wr_slot]          <= next_cons;
                                            ltx_slot_base[cur_wr_slot]         <= (next_cons == 5'd0) ?
                                                                                  tx_ring_base_ep[cur_wr_slot] :
                                                                                  (ltx_slot_base[cur_wr_slot] + tx_ring_slot_size_ep[cur_wr_slot]);
                                            ltx_stat_done[cur_wr_slot]         <= ltx_stat_done[cur_wr_slot] + 32'd1;
                                            stat_tx_samples                    <= stat_tx_samples + 32'd1;
                                            large_tx_done_irq_vec[cur_wr_slot] <= 1'b1;
                                        end else begin
                                            ltx_bytes_rem[cur_wr_slot] <= ltx_bytes_rem[cur_wr_slot] - {20'd0, tx_frag_ser_bytes};
                                            ltx_byte_off[cur_wr_slot]  <= ltx_byte_off[cur_wr_slot] +
                                                                          ((tx_frag_idx == 16'd0) ?
                                                                           ({20'd0, tx_frag_ser_bytes} - 32'd4) :
                                                                           {20'd0, tx_frag_ser_bytes});
                                            ltx_frag_idx[cur_wr_slot]  <= ltx_frag_idx[cur_wr_slot] + 16'd1;
                                        end
                                        state <= S_TX_ARB;
                                    end else if (!tx_frag_more) begin
                                        stat_tx_samples  <= stat_tx_samples + 32'd1;
                                        last_cycle_ticks <= cycle_tick_cnt + 32'd1;
                                        ip_done_irq      <= 1'b1;
                                        state            <= S_DONE_IRQ;
                                    end else begin
                                        if (tx_use_data_frag) begin
                                            tx_bytes_remaining   <= tx_bytes_remaining - {20'd0, tx_frag_ser_bytes};
                                            tx_frag_offset_bytes <= tx_frag_offset_bytes + {4'd0, tx_frag_ser_bytes};
                                            tx_ip_id_counter     <= tx_ip_id_counter + 16'd1;
                                        end else begin
                                            tx_bytes_remaining   <= tx_bytes_remaining - {16'd0, tx_frag_payload_bytes};
                                            tx_frag_offset_bytes <= tx_frag_offset_bytes + tx_frag_payload_bytes;
                                        end
                                        tx_frag_idx   <= tx_frag_idx + 16'd1;
                                        tx_init_phase <= 2'd0;
                                        state         <= S_TX_FRAG_INIT;
                                    end
                                end else begin
                                    tx_beat_idx <= tx_beat_idx + 12'd1;
                                    if (tx_beat_idx == 12'd7 && tx_dgram_word_idx >= (tx_use_data_frag ? 12'd19 : 12'd17)) begin
                                        tx_sample_rd_addr   <= tx_sample_rd_addr + 11'd1;
                                        tx_frag_ram_rd_addr <= tx_frag_ram_rd_addr + 10'd1;
                                    end else if (tx_beat_idx >= 12'd8) begin
                                        tx_prev_half      <= cur_dgram_word[15:0];
                                        tx_dgram_word_idx <= tx_dgram_word_idx + 12'd1;
                                        if (tx_use_data_frag) begin
                                            if ((tx_frag_idx == 16'd0 && tx_dgram_word_idx >= 12'd19) ||
                                                (tx_frag_idx != 16'd0 && tx_dgram_word_idx >= 12'd18)) begin
                                                tx_sample_rd_addr   <= tx_sample_rd_addr + 11'd1;
                                                tx_frag_ram_rd_addr <= tx_frag_ram_rd_addr + 10'd1;
                                            end
                                        end else begin
                                            if (tx_dgram_word_idx >= 12'd16) begin
                                                tx_sample_rd_addr   <= tx_sample_rd_addr + 11'd1;
                                                tx_frag_ram_rd_addr <= tx_frag_ram_rd_addr + 10'd1;
                                            end
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
                // 5. S_DONE_IRQ: Hold Level-High TX IRQ until PS Driver ACKs
                // -------------------------------------------------------------
                S_DONE_IRQ: begin
                    if (eng_tx_tvalid && eng_tx_tready) begin
                        eng_tx_tvalid <= 1'b0;
                        eng_tx_tlast  <= 1'b0;
                    end
                    if ((status_write_pulse && status_write_data[0]) || trigger_detected) begin
                        ip_done_irq    <= 1'b0;
                        err_crc        <= 1'b0;
                        err_no_match   <= 1'b0;
                        rx_ptr         <= 0;
                        state          <= (|ltx_pend_vec && !trigger_detected) ? S_TX_ARB : S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
