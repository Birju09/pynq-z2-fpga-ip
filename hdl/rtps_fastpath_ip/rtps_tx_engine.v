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
    input  wire [31:0]                 rx_timeout_limit,
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

    input  wire [31:0]                 wr0_dst_ip,
    input  wire [15:0]                 wr0_src_port,
    input  wire [15:0]                 wr0_dst_port,
    input  wire [47:0]                 wr0_dst_mac,
    input  wire [31:0]                 wr0_writer_id,
    input  wire [31:0]                 wr0_reader_id,
    input  wire [NUM_ENDPOINTS-1:0]    wr_valid_vec,
    input  wire [NUM_ENDPOINTS*32-1:0] wr_seq_seed_flat,

    // ICMP & RX Error Events
    input  wire                        icmp_err_stb,
    input  wire [31:0]                 icmp_err_info,
    input  wire                        rx_csum_err_pulse,

    // 3-Slot Arbiter & S2MM Egress Handshake
    input  wire                        latest_valid,
    output wire                        slot_consume_pulse,
    output wire                        slot_clear_latest_pulse,
    output wire                        start_egress,
    output wire                        abort_egress,
    output wire [ADDR_WIDTH:0]         egress_pkt_len,
    input  wire                        egress_done,

    // Interrupt & Status Outputs
    output reg                         ip_done_irq,
    output reg  [3:0]                  state,
    output reg  [31:0]                 crc_accum,
    output reg  [31:0]                 last_cycle_ticks,
    output reg  [15:0]                 completed_frags,
    output reg  [31:0]                 stat_tx_samples,
    output reg                         err_crc,
    output reg                         err_no_match,
    output reg                         err_rx_timeout,
    output reg                         err_csum,
    output reg                         irq_is_rx
);

    `include "rtps_fastpath_defs.vh"

    localparam integer ETH_MTU_BYTES         = 1500;
    localparam integer IPV4_HDR_BYTES        = 20;
    localparam integer IPV4_PAYLOAD_MAX      = 1472;
    localparam integer IPV4_NONFINAL_PAYLOAD = 1472;
    localparam integer TX_DF_FRAG_MAX        = 1404;

    wire mode_pub_only   = ctrl_reg[8];
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

    // -------------------------------------------------------------------------
    // TX Command Sample BRAM (2048 x 32-bit = 8 KB) & On-the-Fly Checksums
    // -------------------------------------------------------------------------
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] tx_sample_bram [0:BRAM_DEPTH-1];

    reg [ADDR_WIDTH:0] rx_ptr;
    assign s_axis_tready = (state == S_IDLE) && !mode_sub_only && (!latest_valid || (rx_ptr > 0));
    wire   rx_handshake  = s_axis_tvalid && s_axis_tready;
    wire   axis_wr       = rx_handshake && (state == S_IDLE);

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
    assign egress_pkt_len = pkt_len;

    reg [15:0]         pacing_timer;
    reg [31:0]         cycle_tick_cnt;
    reg [31:0]         rx_wait_cnt;

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
    reg [15:0]         tx_bytes_remaining;
    reg                tx_frag_more;

    reg [1:0]          tx_init_phase;
    reg [11:0]         tx_frag_ser_bytes;
    reg [11:0]         tx_frag_bram_base;
    reg [11:0]         tx_frag_bram_words;
    reg [11:0]         tx_scan_cnt;
    reg [31:0]         tx_scan_csum;
    wire [31:0]        tx_scan_word_be = bswap32(tx_sample_dout);
    reg [15:0]         tx_frag_csum_seed;

    reg [15:0] tx_eff_frag_bytes;
    always @(posedge clk) begin
        tx_eff_frag_bytes <= (cfg_frag_size[15:0] < 16'd4)          ? 16'd4 :
                             (cfg_frag_size[15:0] > TX_DF_FRAG_MAX) ? TX_DF_FRAG_MAX[15:0] :
                             {cfg_frag_size[15:2], 2'b00};
    end

    wire [15:0] tx_sample_total_bytes = {2'b00, pkt_len[11:0], 2'b00} + 16'd4;
    wire        tx_use_single_data    = (!tx_ip_frag_mode) && (tx_sample_total_bytes <= tx_eff_frag_bytes);
    wire        tx_use_data_frag      = (!tx_ip_frag_mode) && (tx_sample_total_bytes > tx_eff_frag_bytes);

    wire [15:0] tx_total_dgram_words = pkt_len[11:0] + 12'd17;
    wire [15:0] tx_udp_total_bytes   = {2'b00, tx_total_dgram_words[11:0], 2'b00};
    wire [15:0] tx_data_octets_le    = {2'b00, pkt_len[11:0], 2'b00} + 16'd24;
    wire [15:0] tx_data_octets_wire  = {tx_data_octets_le[7:0], tx_data_octets_le[15:8]};

    wire [15:0] tx_df_octets_le        = {4'd0, tx_frag_ser_bytes} + 16'd32;
    wire [15:0] tx_df_octets_wire      = {tx_df_octets_le[7:0], tx_df_octets_le[15:8]};
    wire [31:0] tx_df_start_num        = {16'd0, tx_frag_idx} + 32'd1;
    wire [31:0] tx_df_start_wire       = {tx_df_start_num[7:0], tx_df_start_num[15:8],
                                          tx_df_start_num[23:16], tx_df_start_num[31:24]};
    wire [15:0] tx_df_frag_size_wire   = {tx_eff_frag_bytes[7:0], tx_eff_frag_bytes[15:8]};
    wire [31:0] tx_df_sample_size_wire = {tx_sample_total_bytes[7:0], tx_sample_total_bytes[15:8], 16'h0000};

    reg [DATA_WIDTH-1:0] tx_sample_buf;
    always @(posedge clk) begin
        if (!rst_n_local) begin
            tx_sample_buf <= 32'd0;
        end else if (pacing_timer == cfg_pacing_timer[15:0]) begin
            tx_sample_buf <= tx_sample_dout;
        end
    end
    wire [31:0] tx_sample_word = (cfg_pacing_timer == 32'd0) ? tx_sample_dout : tx_sample_buf;

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
                12'd0:  cur_dgram_word = {wr0_src_port, wr0_dst_port};
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
                12'd12: cur_dgram_word = wr0_reader_id;
                12'd13: cur_dgram_word = wr0_writer_id;
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
                12'd0:  cur_dgram_word = {wr0_src_port, wr0_dst_port};
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
                12'd12: cur_dgram_word = wr0_reader_id;
                12'd13: cur_dgram_word = wr0_writer_id;
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
        .wr0_dst_ip             (wr0_dst_ip),
        .wr0_src_port           (wr0_src_port),
        .wr0_dst_port           (wr0_dst_port),
        .local_vendor_id        (local_vendor_id),
        .local_guid_prefix      (local_guid_prefix),
        .tx_df_octets_wire      (tx_df_octets_wire),
        .tx_df_start_wire       (tx_df_start_wire),
        .tx_df_frag_size_wire   (tx_df_frag_size_wire),
        .tx_df_sample_size_wire (tx_df_sample_size_wire),
        .tx_data_octets_wire    (tx_data_octets_wire),
        .latched_tx_ts          (latched_tx_ts),
        .wr0_reader_id          (wr0_reader_id),
        .wr0_writer_id          (wr0_writer_id),
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
            12'd0: next_wire_tx_word = wr0_dst_mac[47:16];
            12'd1: next_wire_tx_word = {wr0_dst_mac[15:0], local_mac[47:32]};
            12'd2: next_wire_tx_word = local_mac[31:0];
            12'd3: next_wire_tx_word = 32'h0800_4500;
            12'd4: next_wire_tx_word = {tx_frag_ip_len, tx_ip_id_counter};
            12'd5: next_wire_tx_word = {tx_frag_flags, 16'h4011};
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

    // -------------------------------------------------------------------------
    // Combinational Handoff Pulses to 3-Slot Arbiter & S2MM Egress
    // -------------------------------------------------------------------------
    assign slot_consume_pulse =
        ((state == S_IDLE)         && latest_valid && (rx_ptr == 0)) ||
        ((state == S_TX_FRAG_SEND) && !trigger_detected && (!eng_tx_tvalid || eng_tx_tready) &&
         (pacing_timer == 16'd0) && (tx_beat_idx == (12'd8 + tx_frag_words)) &&
         !tx_frag_more && mode_pub_only && latest_valid) ||
        ((state == S_WAIT_RX_FB)   && !trigger_detected && latest_valid);

    assign slot_clear_latest_pulse =
        (state == S_CHECK_CRC) && lifecycle_ok_q &&
        ((crc_accum == expected_crc) || (expected_crc == 32'h0)) && !mode_pub_only;

    assign start_egress = (state == S_WAIT_EGRESS) && status_write_pulse && status_write_data[1];
    assign abort_egress = (state == S_TX_RUN) && trigger_detected;

    // -------------------------------------------------------------------------
    // Master Cycle Sequencer FSM
    // -------------------------------------------------------------------------
    integer rst_ep;
    always @(posedge clk) begin
        if (!rst_n_local) begin
            state                <= S_IDLE;
            rx_ptr               <= 0;
            pkt_len              <= 0;
            pacing_timer         <= 16'd0;
            cycle_tick_cnt       <= 32'd0;
            last_cycle_ticks     <= 32'd0;
            rx_wait_cnt          <= 32'd0;

            ip_done_irq          <= 1'b0;
            err_crc              <= 1'b0;
            err_no_match         <= 1'b0;
            err_rx_timeout       <= 1'b0;
            err_csum             <= 1'b0;
            err_icmp             <= 1'b0;
            irq_is_rx            <= 1'b0;
            completed_frags      <= 16'd0;

            tx_ip_id_counter     <= 16'h1000;
            latched_tx_ts        <= 64'd0;
            cur_tx_seq           <= 64'd1;
            tx_udp_csum          <= 16'd0;
            tx_frag_idx          <= 16'd0;
            tx_frag_words        <= 12'd0;
            tx_frag_ip_len       <= 16'd0;
            tx_frag_flags        <= 16'd0;
            tx_frag_hdr_csum     <= 16'd0;
            tx_dgram_word_idx    <= 12'd0;
            tx_beat_idx          <= 12'd0;
            tx_prev_half         <= 16'd0;
            tx_sample_rd_addr    <= {ADDR_WIDTH{1'b0}};
            tx_init_phase        <= 2'd0;
            tx_frag_ser_bytes    <= 12'd0;
            tx_frag_bram_base    <= 12'd0;
            tx_frag_bram_words   <= 12'd0;
            tx_scan_cnt          <= 12'd0;
            tx_scan_csum         <= 16'd0;
            tx_frag_csum_seed    <= 16'd0;

            eng_tx_tdata         <= 32'd0;
            eng_tx_tvalid        <= 1'b0;
            eng_tx_tlast         <= 1'b0;
            stat_tx_samples      <= 32'd0;

            tx_frag_payload_bytes <= 16'd0;
            tx_frag_offset_bytes  <= 16'd0;
            tx_bytes_remaining    <= 16'd0;
            tx_frag_more          <= 1'b0;

            for (rst_ep = 0; rst_ep < NUM_ENDPOINTS; rst_ep = rst_ep + 1) begin
                writer_seq_num[rst_ep] <= 64'd1;
            end
        end else begin
            if (rx_csum_err_pulse) begin
                err_csum <= 1'b1;
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
                // 1. S_IDLE: Wait for DMA0 MM2S sample stream or manual CSR kick
                // -------------------------------------------------------------
                S_IDLE: begin
                    ip_done_irq   <= 1'b0;
                    eng_tx_tvalid <= 1'b0;
                    eng_tx_tlast  <= 1'b0;

                    if (latest_valid && (rx_ptr == 0)) begin
                        // Asynchronous wire RX: Walker has verified a complete sample from remote publisher!
                        irq_is_rx        <= 1'b1;
                        last_cycle_ticks <= 32'd1;
                        ip_done_irq      <= 1'b1; // Level-High IRQ to PS (IRQ_F2P)
                        state            <= S_DONE_IRQ;
                    end else if (rx_handshake && !mode_sub_only) begin
                        if (s_axis_tlast || (rx_ptr == cfg_payload_words - 1)) begin
                            pkt_len <= {1'b0, rx_ptr[ADDR_WIDTH-1:0]} + 1'b1;
                            rx_ptr  <= 0;
                            state   <= S_CHECK_CRC;
                        end else begin
                            rx_ptr  <= rx_ptr + 1'b1;
                        end
                    end else if ((trigger_detected || (status_write_pulse && status_write_data[2])) && !mode_sub_only) begin
                        pkt_len <= cfg_payload_words[ADDR_WIDTH:0];
                        rx_ptr  <= 0;
                        state   <= S_CHECK_CRC;
                    end
                end

                // -------------------------------------------------------------
                // 2. S_CHECK_CRC: Verify DMA0 CRC, Check Lifecycle, Init UDP Csum
                // -------------------------------------------------------------
                S_CHECK_CRC: begin
                    latched_tx_ts  <= active_time_ns + ts_offset;
                    cur_tx_seq     <= writer_seq_num[0];
                    cycle_tick_cnt <= 32'd0;

                    if (!lifecycle_ok_q) begin
                        // Q22: Disabled / watchdog expired / no valid writer match -> immediate ERR_NO_MATCH
                        err_no_match   <= 1'b1;
                        err_crc        <= 1'b0;
                        err_rx_timeout <= 1'b0;
                        irq_is_rx      <= 1'b0;
                        ip_done_irq    <= 1'b1;
                        state          <= S_DONE_IRQ;
                    end else if ((crc_accum == expected_crc) || (expected_crc == 32'h0)) begin
                        err_crc              <= 1'b0;
                        err_no_match         <= 1'b0;
                        err_rx_timeout       <= 1'b0;
                        err_csum             <= 1'b0;
                        completed_frags      <= 16'd0;
                        tx_frag_idx          <= 16'd0;
                        tx_bytes_remaining   <= tx_ip_frag_mode ? tx_udp_total_bytes : tx_sample_total_bytes;
                        tx_frag_offset_bytes <= 16'd0;
                        tx_init_phase        <= 2'd0;
                        tx_ip_id_counter     <= tx_ip_id_counter + 16'd1;
                        writer_seq_num[0]    <= writer_seq_num[0] + 64'd1;
                        if (!mode_pub_only) begin
                            end
                        state                <= S_TX_FRAG_INIT;
                    end else begin
                        err_crc     <= 1'b1;
                        irq_is_rx   <= 1'b0;
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
                                        tx_frag_payload_bytes <= tx_bytes_remaining;
                                        tx_frag_more          <= 1'b0;
                                        tx_frag_words         <= (tx_bytes_remaining + 16'd3) >> 2;
                                        tx_frag_ip_len        <= IPV4_HDR_BYTES + tx_bytes_remaining;
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
                                    if (!tx_frag_more) begin
                                        stat_tx_samples <= stat_tx_samples + 32'd1;
                                        if (mode_pub_only) begin
                                            last_cycle_ticks <= cycle_tick_cnt + 32'd1;
                                            if (latest_valid) begin
                                                irq_is_rx    <= 1'b1;
                                            end else begin
                                                irq_is_rx    <= 1'b0;
                                            end
                                            ip_done_irq <= 1'b1;
                                            state       <= S_DONE_IRQ;
                                        end else begin
                                            rx_wait_cnt <= 32'd0;
                                            state       <= S_WAIT_RX_FB;
                                        end
                                    end else begin
                                        if (tx_use_data_frag) begin
                                            tx_bytes_remaining   <= tx_bytes_remaining - {4'd0, tx_frag_ser_bytes};
                                            tx_frag_offset_bytes <= tx_frag_offset_bytes + {4'd0, tx_frag_ser_bytes};
                                            tx_ip_id_counter     <= tx_ip_id_counter + 16'd1;
                                        end else begin
                                            tx_bytes_remaining   <= tx_bytes_remaining - tx_frag_payload_bytes;
                                            tx_frag_offset_bytes <= tx_frag_offset_bytes + tx_frag_payload_bytes;
                                        end
                                        tx_frag_idx   <= tx_frag_idx + 16'd1;
                                        tx_init_phase <= 2'd0;
                                        state         <= S_TX_FRAG_INIT;
                                    end
                                end else begin
                                    tx_beat_idx <= tx_beat_idx + 12'd1;
                                    if (tx_beat_idx == 12'd7 && tx_dgram_word_idx >= (tx_use_data_frag ? 12'd19 : 12'd17)) begin
                                        tx_sample_rd_addr <= tx_sample_rd_addr + 11'd1;
                                    end else if (tx_beat_idx >= 12'd8) begin
                                        tx_prev_half      <= cur_dgram_word[15:0];
                                        tx_dgram_word_idx <= tx_dgram_word_idx + 12'd1;
                                        if (tx_use_data_frag) begin
                                            if ((tx_frag_idx == 16'd0 && tx_dgram_word_idx >= 12'd19) ||
                                                (tx_frag_idx != 16'd0 && tx_dgram_word_idx >= 12'd18)) begin
                                                tx_sample_rd_addr <= tx_sample_rd_addr + 11'd1;
                                            end
                                        end else begin
                                            if (tx_dgram_word_idx >= 12'd16) begin
                                                tx_sample_rd_addr <= tx_sample_rd_addr + 11'd1;
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
                        irq_is_rx        <= 1'b1;
                        last_cycle_ticks <= cycle_tick_cnt + 32'd1;
                        ip_done_irq      <= 1'b1; // Assert Level-High IRQ to PS!
                        state            <= S_DONE_IRQ;
                    end else if (icmp_err_stb && (icmp_err_info[31:24] == 8'd3 || icmp_err_info[31:24] == 8'd11)) begin
                        err_icmp         <= 1'b1;
                        err_rx_timeout   <= 1'b1;
                        irq_is_rx        <= 1'b0;
                        last_cycle_ticks <= cycle_tick_cnt + 32'd1;
                        ip_done_irq      <= 1'b1; // Wake PS with ICMP error
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
                    if (eng_tx_tvalid && eng_tx_tready) begin
                        eng_tx_tvalid <= 1'b0;
                        eng_tx_tlast  <= 1'b0;
                    end
                    if (status_write_pulse && status_write_data[0]) begin
                        ip_done_irq <= 1'b0;
                        if (err_crc || err_no_match || err_rx_timeout || err_icmp || (!irq_is_rx && mode_pub_only)) begin
                            err_crc        <= 1'b0;
                            err_no_match   <= 1'b0;
                            err_rx_timeout <= 1'b0;
                            err_icmp       <= 1'b0;
                            irq_is_rx      <= 1'b0;
                            rx_ptr         <= 0;
                            state          <= S_IDLE;
                        end else begin
                            state     <= S_WAIT_EGRESS;
                        end
                    end else if (trigger_detected) begin
                        ip_done_irq    <= 1'b0;
                        err_crc        <= 1'b0;
                        err_no_match   <= 1'b0;
                        err_rx_timeout <= 1'b0;
                        err_icmp       <= 1'b0;
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
                        state           <= S_TX_RUN;
                    end else if (trigger_detected) begin
                        rx_ptr <= 0;
                        state  <= S_IDLE;
                    end
                end

                S_TX_RUN: begin
                    if (trigger_detected) begin
                        rx_ptr <= 0;
                        state  <= S_IDLE;
                    end else if (egress_done) begin
                        irq_is_rx <= 1'b0;
                        rx_ptr    <= 0;
                        state     <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
