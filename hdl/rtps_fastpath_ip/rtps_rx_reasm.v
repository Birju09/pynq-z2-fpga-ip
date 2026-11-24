`timescale 1ns / 1ps

// =============================================================================
// Module: rtps_rx_reasm
//
// Description:
//   3-Slot Store-and-Forward IPv4 Reassembly, 1.5 KB Unfragmented Staging RAM,
//   4-Entry Reader CAM, and Generic RTPS v2.x Submessage Walker (PART A).
// =============================================================================

module rtps_rx_reasm #(
    parameter integer DATA_WIDTH    = 32,
    parameter integer NUM_ENDPOINTS = 4,
    parameter integer REASM_DEPTH   = 1536
)(
    input  wire                          clk,
    input  wire                          rst_n_local,

    // Wire / Emulator RX Stream
    input  wire [DATA_WIDTH-1:0]         eng_rx_tdata,
    input  wire                          eng_rx_tvalid,
    output wire                          eng_rx_tready,
    input  wire                          eng_rx_tlast,

    // Configuration & Active Reader CAM EDT
    input  wire [31:0]                   local_src_ip,
    input  wire [95:0]                   local_guid_prefix,
    input  wire [NUM_ENDPOINTS*7*32-1:0] rd_edt_flat,
    input  wire                          edt_commit_applied,
    input  wire                          identity_clear_seq_pulse,

    // 3-Slot Reassembly BRAM Interface (top-level arbiter owns fill_slot)
    output reg                           reasm_wr_en,
    output reg  [3:0]                    reasm_wr_be,
    output reg  [11:0]                   reasm_wr_addr,
    output reg  [31:0]                   reasm_wr_data,
    output reg  [11:0]                   walker_rd_addr,
    input  wire [31:0]                   reasm_walker_dout,

    // Handoff Pulses to Arbiter & Master FSM
    output reg                           rx_commit_pulse,
    output reg                           rx_csum_err_pulse,

    // Telemetry & Status Outputs
    output reg  [5:0]                    rx_state,
    output reg  [11:0]                   latest_payload_word_off,
    output reg  [11:0]                   latest_rtps_words,
    output reg  [31:0]                   stat_rx_samples,
    output reg  [31:0]                   stat_seq_gaps,
    output reg  [31:0]                   stat_stale_drops,
    output reg  [31:0]                   stat_csum_errs,
    output reg  [31:0]                   stat_skipped_submsg,
    output reg  [63:0]                   last_rx_seq_num,
    output reg  [63:0]                   last_rx_ts,
    output reg  [31:0]                   last_rx_src_ip,
    output reg  [15:0]                   last_rx_src_port,
    output reg  [15:0]                   last_rx_dst_port,
    output reg  [1:0]                    last_rx_cam_idx,
    output reg  [47:0]                   last_rx_src_mac,
    output wire [NUM_ENDPOINTS*32-1:0]   rd_match_count_flat
);

    `include "rtps_fastpath_defs.vh"

    // Unpack Active Reader CAM EDT
    wire [31:0] active_rd_edt [0:NUM_ENDPOINTS-1][0:6];
    genvar g_ep, g_w;
    generate
        for (g_ep = 0; g_ep < NUM_ENDPOINTS; g_ep = g_ep + 1) begin : GEN_RD_UNPACK
            for (g_w = 0; g_w < 7; g_w = g_w + 1) begin : GEN_RD_W
                assign active_rd_edt[g_ep][g_w] = rd_edt_flat[(g_ep*7 + g_w)*32 +: 32];
            end
        end
    endgenerate

    reg [31:0] rd_match_count [0:NUM_ENDPOINTS-1];
    reg [63:0] last_rx_seq_num_by_cam [0:NUM_ENDPOINTS-1];
    reg [63:0] last_rx_ts_by_cam      [0:NUM_ENDPOINTS-1];

    generate
        for (g_ep = 0; g_ep < NUM_ENDPOINTS; g_ep = g_ep + 1) begin : GEN_RD_MATCH_OUT
            assign rd_match_count_flat[g_ep*32 +: 32] = rd_match_count[g_ep];
        end
    endgenerate

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

    // 1.5 KB (512 x 32-bit) RX Staging RAM for unfragmented UDP/IPv4 frames
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] rx_pkt_ram [0:511];
    reg        rx_pkt_wr_en;
    reg [8:0]  rx_pkt_wr_addr;
    reg [31:0] rx_pkt_wr_data;
    reg [31:0] rx_pkt_dout;
    reg        walk_from_pkt_ram;

    always @(posedge clk) begin
        if (rx_pkt_wr_en) begin
            rx_pkt_ram[rx_pkt_wr_addr] <= rx_pkt_wr_data;
        end
        rx_pkt_dout <= rx_pkt_ram[walker_rd_addr[8:0]];
    end

    wire [31:0] walker_dout = walk_from_pkt_ram ? rx_pkt_dout : reasm_walker_dout;

    reg [11:0] rx_wire_beat;
    reg [15:0] rx_ip_len;
    reg [15:0] rx_ip_id;
    reg [15:0] rx_active_ip_id;
    reg        rx_active_valid;
    reg [15:0] rx_flags_frag;
    reg [15:0] rx_hdr_csum_accum;
    reg [31:0] rx_hdr_word;
    reg        rx_hdr_add, rx_hdr_init;

    always @(posedge clk) begin
        if (!rst_n_local)
            rx_hdr_csum_accum <= 16'd0;
        else if (rx_hdr_init)
            rx_hdr_csum_accum <= csum_add32(16'h0000, rx_hdr_word);
        else if (rx_hdr_add)
            rx_hdr_csum_accum <= csum_add32(rx_hdr_csum_accum, rx_hdr_word);
    end

    reg        rx_is_ipv4_udp;
    reg        rx_pkt_is_ip_frag;
    reg [31:0] rx_src_ip;
    reg [31:0] rx_dst_ip;
    reg [15:0] rx_prev_half;
    reg [11:0] rx_wr_ptr;
    reg [9:0]  rx_pkt_wr_ptr;
    reg [15:0] rx_bytes_rcvd;
    reg [15:0] rx_bytes_total;
    reg        rx_saw_last;
    reg        rx_dst_match;
    reg [47:0] rx_wire_src_mac;
    reg [47:0] rx_active_src_mac;

    reg [31:0] frag_src_ip, frag_dst_ip;
    reg [15:0] frag_udp_len_bytes, frag_udp_src_port, frag_udp_dst_port, frag_udp_hdr_csum;
    reg [15:0] frag_udp_csum_accum;
    reg        rx_csum_clr;

    reg [31:0] pkt_src_ip, pkt_dst_ip;
    reg [15:0] pkt_ip_pay_bytes, pkt_udp_len_bytes, pkt_udp_src_port, pkt_udp_dst_port, pkt_udp_hdr_csum;
    reg [15:0] pkt_udp_csum_accum;
    reg        pkt_csum_clr;
    reg [47:0] pkt_src_mac;
    reg [31:0] rx_tail_word;
    reg        rx_tail_valid;

    reg [31:0] walk_src_ip, walk_dst_ip;
    reg [15:0] walk_udp_len_bytes, walk_udp_src_port, walk_udp_dst_port, walk_udp_hdr_csum;
    reg [15:0] walk_udp_csum_accum;
    reg [47:0] walk_src_mac;

    localparam integer REASM_MAX_BYTES   = REASM_DEPTH * 4;
    localparam integer REASM_BLOCK_BYTES = 8;
    localparam integer REASM_BLOCKS      = REASM_MAX_BYTES / REASM_BLOCK_BYTES;

    reg [REASM_BLOCKS-1:0] reasm_coverage;
    reg [9:0]              reasm_blocks_covered;
    reg [15:0]             reasm_total_bytes;
    reg                    reasm_last_seen;

    reg [19:0] rx_ip_pseudo_sum;
    reg [19:0] rx_pseudo_sum_pipe;
    reg [15:0] rx_pseudo_csum;
    wire [16:0] rx_pseudo_fold1 = {1'b0, rx_pseudo_sum_pipe[15:0]} + {13'd0, rx_pseudo_sum_pipe[19:16]};
    wire [15:0] rx_pseudo_fold2 = rx_pseudo_fold1[15:0] + {15'd0, rx_pseudo_fold1[16]};

    always @(posedge clk) begin
        if (!rst_n_local) begin
            rx_ip_pseudo_sum   <= 20'd0;
            rx_pseudo_sum_pipe <= 20'd0;
            rx_pseudo_csum     <= 16'd0;
        end else begin
            rx_ip_pseudo_sum   <= {4'd0, walk_src_ip[31:16]} + {4'd0, walk_src_ip[15:0]} +
                                  {4'd0, walk_dst_ip[31:16]} + {4'd0, walk_dst_ip[15:0]} +
                                  20'h0011;
            rx_pseudo_sum_pipe <= rx_ip_pseudo_sum + {4'd0, walk_udp_len_bytes};
            rx_pseudo_csum     <= rx_pseudo_fold2;
        end
    end

    always @(posedge clk) begin
        if (!rst_n_local || rx_csum_clr)
            frag_udp_csum_accum <= 16'd0;
        else if ((rx_state == R_IDLE || rx_state == R_REASM_CHECK) && reasm_wr_en)
            frag_udp_csum_accum <= csum_add32(frag_udp_csum_accum, reasm_wr_data);
    end

    always @(posedge clk) begin
        if (!rst_n_local || pkt_csum_clr)
            pkt_udp_csum_accum <= 16'd0;
        else if (rx_pkt_wr_en)
            pkt_udp_csum_accum <= csum_add32(pkt_udp_csum_accum, rx_pkt_wr_data);
    end

    wire [16:0] rx_chk_fold1 = {1'b0, walk_udp_csum_accum} + {1'b0, rx_pseudo_csum};
    wire [15:0] rx_chk_final = rx_chk_fold1[15:0] + {15'd0, rx_chk_fold1[16]};
    wire        rx_chk_valid = (walk_udp_hdr_csum == 16'h0000) ||
                               ((&rx_chk_fold1[15:1]) && (rx_chk_fold1[0] ^ rx_chk_fold1[16]));

    wire [12:0] rx_frag_off_8b  = rx_flags_frag[12:0];
    wire        rx_frag_mf      = rx_flags_frag[13];
    wire [15:0] rx_ip_pay_bytes = rx_ip_len - 16'd20;
    wire [11:0] rx_frag_w_start = {rx_frag_off_8b[10:0], 1'b0};

    reg [11:0] rx_wr_end;
    always @(posedge clk) rx_wr_end <= rx_frag_w_start + rx_ip_pay_bytes[13:2];

    assign eng_rx_tready = (rx_state == R_IDLE);
    wire   eng_rx_fire   = eng_rx_tvalid && eng_rx_tready;

    reg [95:0] walk_guid_prefix;
    reg [1:0]  walk_cam_idx;
    reg        walk_cam_hit;
    reg [31:0] cam_reader_id_r;
    reg [31:0] cam_writer_id_r;
    reg [63:0] cam_last_seq_r;
    reg [11:0] walk_cur_word;
    reg [11:0] walk_dgram_words;
    reg [7:0]  walk_submsg_id;
    reg [7:0]  walk_submsg_flags;
    reg [15:0] walk_octets_next;
    reg [11:0] walk_next_submsg_word;
    reg [63:0] walk_ts;
    reg [31:0] walk_writer_id;
    reg [31:0] walk_reader_id;
    reg [63:0] walk_seq_num;
    reg [11:0] pending_payload_word_off;
    reg [31:0] walk_rtps_ver_vendor;

    reg        df_in_qos;
    reg [31:0] df_frag_start;
    reg [15:0] df_frags_in_submsg;
    reg [15:0] df_frag_size;
    reg [31:0] df_sample_size;
    reg [11:0] df_pay_start_word;
    reg [1:0]  df_calc_phase;
    reg [15:0] df_calc_rem;
    reg [5:0]  df_calc_cnt;
    reg [15:0] df_calc_acc;
    reg [15:0] df_byte_off;
    reg [15:0] df_submsg_bytes;
    reg [15:0] df_chunk_bytes;
    reg [5:0]  df_total_frags;
    reg [31:0] df_frag_mask_add_r;
    reg [31:0] df_frag_mask_all_r;

    reg        df_active_valid;
    reg [1:0]  df_active_cam_idx;
    reg [63:0] df_active_seq;
    reg [15:0] df_active_sample_size;
    reg [15:0] df_active_frag_size;
    reg [5:0]  df_active_total_frags;
    reg [31:0] df_frag_bitmap;

    reg [15:0] df_dst_byte_addr;
    reg [15:0] df_rem_bytes;
    reg [11:0] df_src_word_ptr;
    reg [31:0] df_prev_word;
    reg        df_first_beat;
    reg [1:0]  df_shift;
    reg [3:0]  df_synth_idx;
    reg [11:0] copy_word_idx;

    reg [95:0] walk_dst_guid_prefix;
    reg        walk_dst_valid;
    wire       info_dst_matches_local = !walk_dst_valid ||
                                        (walk_dst_guid_prefix == 96'd0) ||
                                        (walk_dst_guid_prefix == local_guid_prefix);

    wire rd0_active = active_rd_edt[0][0][0] && (active_rd_edt[0][0][3:1] == 3'd2);
    wire rd1_active = active_rd_edt[1][0][0] && (active_rd_edt[1][0][3:1] == 3'd2);
    wire rd2_active = active_rd_edt[2][0][0] && (active_rd_edt[2][0][3:1] == 3'd2);
    wire rd3_active = active_rd_edt[3][0][0] && (active_rd_edt[3][0][3:1] == 3'd2);

    wire rd0_port_ok = (active_rd_edt[0][0][31:16] == 16'd0) || (walk_udp_dst_port == active_rd_edt[0][0][31:16]);
    wire rd1_port_ok = (active_rd_edt[1][0][31:16] == 16'd0) || (walk_udp_dst_port == active_rd_edt[1][0][31:16]);
    wire rd2_port_ok = (active_rd_edt[2][0][31:16] == 16'd0) || (walk_udp_dst_port == active_rd_edt[2][0][31:16]);
    wire rd3_port_ok = (active_rd_edt[3][0][31:16] == 16'd0) || (walk_udp_dst_port == active_rd_edt[3][0][31:16]);

    wire rd0_ip_ok   = (active_rd_edt[0][6] == 32'd0) || (walk_src_ip == active_rd_edt[0][6]);
    wire rd1_ip_ok   = (active_rd_edt[1][6] == 32'd0) || (walk_src_ip == active_rd_edt[1][6]);
    wire rd2_ip_ok   = (active_rd_edt[2][6] == 32'd0) || (walk_src_ip == active_rd_edt[2][6]);
    wire rd3_ip_ok   = (active_rd_edt[3][6] == 32'd0) || (walk_src_ip == active_rd_edt[3][6]);

    wire cam_hit_0 = rd0_active && rd0_port_ok && rd0_ip_ok && ({active_rd_edt[0][1], active_rd_edt[0][2], active_rd_edt[0][3]} == walk_guid_prefix);
    wire cam_hit_1 = rd1_active && rd1_port_ok && rd1_ip_ok && ({active_rd_edt[1][1], active_rd_edt[1][2], active_rd_edt[1][3]} == walk_guid_prefix);
    wire cam_hit_2 = rd2_active && rd2_port_ok && rd2_ip_ok && ({active_rd_edt[2][1], active_rd_edt[2][2], active_rd_edt[2][3]} == walk_guid_prefix);
    wire cam_hit_3 = rd3_active && rd3_port_ok && rd3_ip_ok && ({active_rd_edt[3][1], active_rd_edt[3][2], active_rd_edt[3][3]} == walk_guid_prefix);

    reg [3:0] cam_hit_r;
    always @(posedge clk) begin
        cam_hit_r <= {cam_hit_3, cam_hit_2, cam_hit_1, cam_hit_0};
    end

    wire        submsg_e_flag = walker_dout[16];
    wire [15:0] submsg_octets = submsg_e_flag ? {walker_dout[7:0], walker_dout[15:8]} : walker_dout[15:0];

    integer rst_ep;
    always @(posedge clk) begin
        if (!rst_n_local) begin
            rx_state                <= R_IDLE;
            reasm_wr_en             <= 1'b0;
            reasm_wr_be             <= 4'b1111;
            rx_csum_clr             <= 1'b0;
            pkt_csum_clr            <= 1'b0;
            rx_pkt_wr_en            <= 1'b0;
            rx_pkt_wr_addr          <= 9'd0;
            rx_pkt_wr_data          <= 32'd0;
            walk_from_pkt_ram       <= 1'b0;
            reasm_wr_addr           <= 12'd0;
            reasm_wr_data           <= 32'd0;
            walker_rd_addr          <= 12'd0;
            rx_commit_pulse         <= 1'b0;
            rx_csum_err_pulse       <= 1'b0;

            rx_wire_beat            <= 12'd0;
            rx_is_ipv4_udp          <= 1'b0;
            rx_pkt_is_ip_frag       <= 1'b0;
            rx_ip_len               <= 16'd0;
            rx_ip_id                <= 16'd0;
            rx_active_ip_id         <= 16'd0;
            rx_active_valid         <= 1'b0;
            rx_flags_frag           <= 16'd0;
            rx_hdr_word             <= 32'd0;
            rx_hdr_add              <= 1'b0;
            rx_hdr_init             <= 1'b0;
            rx_src_ip               <= 32'd0;
            rx_dst_ip               <= 32'd0;
            rx_prev_half            <= 16'd0;
            rx_wr_ptr               <= 12'd0;
            rx_pkt_wr_ptr           <= 10'd0;
            rx_bytes_rcvd           <= 16'd0;
            rx_bytes_total          <= 16'd0;
            rx_saw_last             <= 1'b0;
            rx_dst_match            <= 1'b0;
            frag_src_ip             <= 32'd0;
            frag_dst_ip             <= 32'd0;
            frag_udp_len_bytes      <= 16'd0;
            frag_udp_src_port       <= 16'd0;
            frag_udp_dst_port       <= 16'd0;
            frag_udp_hdr_csum       <= 16'hFFFF;
            pkt_src_ip              <= 32'd0;
            pkt_dst_ip              <= 32'd0;
            pkt_ip_pay_bytes        <= 16'd0;
            pkt_udp_len_bytes       <= 16'd0;
            pkt_udp_src_port        <= 16'd0;
            pkt_udp_dst_port        <= 16'd0;
            pkt_udp_hdr_csum        <= 16'hFFFF;
            pkt_src_mac             <= 48'd0;
            rx_tail_word            <= 32'd0;
            rx_tail_valid           <= 1'b0;
            walk_src_ip             <= 32'd0;
            walk_dst_ip             <= 32'd0;
            walk_udp_len_bytes      <= 16'd0;
            walk_udp_src_port       <= 16'd0;
            walk_udp_dst_port       <= 16'd0;
            walk_udp_hdr_csum       <= 16'hFFFF;
            walk_udp_csum_accum     <= 16'd0;
            walk_src_mac            <= 48'd0;
            last_rx_src_ip          <= 32'd0;
            last_rx_src_port        <= 16'd0;
            last_rx_dst_port        <= 16'd0;
            last_rx_cam_idx         <= 2'd0;
            rx_wire_src_mac         <= 48'd0;
            rx_active_src_mac       <= 48'd0;
            last_rx_src_mac         <= 48'd0;

            walk_guid_prefix        <= 96'd0;
            walk_cam_idx            <= 2'd0;
            walk_cam_hit            <= 1'b0;
            cam_reader_id_r         <= 32'd0;
            cam_writer_id_r         <= 32'd0;
            cam_last_seq_r          <= 64'd0;
            walk_cur_word           <= 12'd0;
            walk_dgram_words        <= 12'd0;
            walk_submsg_id          <= 8'd0;
            walk_submsg_flags       <= 8'd0;
            walk_octets_next        <= 16'd0;
            walk_next_submsg_word   <= 12'd0;
            walk_ts                 <= 64'd0;
            walk_writer_id          <= 32'd0;
            walk_reader_id          <= 32'd0;
            walk_seq_num            <= 64'd0;
            latest_payload_word_off <= 12'd17;
            pending_payload_word_off<= 12'd17;
            walk_rtps_ver_vendor    <= 32'h0203_0101;
            latest_rtps_words       <= 12'd1039;

            df_in_qos               <= 1'b0;
            df_frag_start           <= 32'd0;
            df_frags_in_submsg      <= 16'd0;
            df_frag_size            <= 16'd0;
            df_sample_size          <= 32'd0;
            df_pay_start_word       <= 12'd0;
            df_calc_phase           <= 2'd0;
            df_calc_rem             <= 16'd0;
            df_calc_cnt             <= 6'd0;
            df_calc_acc             <= 16'd0;
            df_byte_off             <= 16'd0;
            df_submsg_bytes         <= 16'd0;
            df_chunk_bytes          <= 16'd0;
            df_total_frags          <= 6'd0;
            df_active_valid         <= 1'b0;
            df_active_cam_idx       <= 2'd0;
            df_active_seq           <= 64'd0;
            df_active_sample_size   <= 16'd0;
            df_active_frag_size     <= 16'd0;
            df_active_total_frags   <= 6'd0;
            df_frag_bitmap          <= 32'd0;
            df_dst_byte_addr        <= 16'd0;
            df_rem_bytes            <= 16'd0;
            df_src_word_ptr         <= 12'd0;
            df_prev_word            <= 32'd0;
            df_first_beat           <= 1'b0;
            df_shift                <= 2'd0;
            df_synth_idx            <= 4'd0;
            copy_word_idx           <= 12'd0;

            stat_rx_samples         <= 32'd0;
            stat_seq_gaps           <= 32'd0;
            stat_stale_drops        <= 32'd0;
            stat_csum_errs          <= 32'd0;
            stat_skipped_submsg     <= 32'd0;
            last_rx_seq_num         <= 64'd0;
            last_rx_ts              <= 64'd0;
            walk_dst_guid_prefix    <= 96'd0;
            walk_dst_valid          <= 1'b0;

            for (rst_ep = 0; rst_ep < NUM_ENDPOINTS; rst_ep = rst_ep + 1) begin
                rd_match_count[rst_ep]         <= 32'd0;
                last_rx_seq_num_by_cam[rst_ep] <= 64'd0;
                last_rx_ts_by_cam[rst_ep]      <= 64'd0;
            end

            reasm_coverage       <= {REASM_BLOCKS{1'b0}};
            reasm_blocks_covered <= 10'd0;
            reasm_total_bytes    <= 16'd0;
            reasm_last_seen      <= 1'b0;
        end else begin
            reasm_wr_en       <= 1'b0;
            reasm_wr_be       <= 4'b1111;
            rx_pkt_wr_en      <= 1'b0;
            rx_csum_clr       <= 1'b0;
            pkt_csum_clr      <= 1'b0;
            rx_hdr_add        <= 1'b0;
            rx_hdr_init       <= 1'b0;
            rx_commit_pulse   <= 1'b0;
            rx_csum_err_pulse <= 1'b0;

            if (edt_commit_applied || identity_clear_seq_pulse) begin
                for (rst_ep = 0; rst_ep < NUM_ENDPOINTS; rst_ep = rst_ep + 1) begin
                    last_rx_seq_num_by_cam[rst_ep] <= 64'd0;
                    last_rx_ts_by_cam[rst_ep]      <= 64'd0;
                end
                last_rx_seq_num <= 64'd0;
                last_rx_ts      <= 64'd0;
            end

            case (rx_state)
                R_IDLE: begin
                    if (eng_rx_fire) begin
                        case (rx_wire_beat)
                            12'd0: begin
                                rx_wire_beat <= rx_wire_beat + 12'd1;
                            end
                            12'd1: begin
                                rx_wire_src_mac[47:32] <= eng_rx_tdata[15:0];
                                rx_wire_beat           <= rx_wire_beat + 12'd1;
                            end
                            12'd2: begin
                                rx_wire_src_mac[31:0]  <= eng_rx_tdata[31:0];
                                rx_wire_beat           <= rx_wire_beat + 12'd1;
                            end
                            12'd3: begin
                                // Beat 3: {EtherType(0x0800), VerIHL_TOS(0x4500)}
                                rx_hdr_word       <= {16'h0000, eng_rx_tdata[15:0]};
                                rx_hdr_init       <= 1'b1;
                                rx_is_ipv4_udp    <= (eng_rx_tdata[31:16] == 16'h0800) && (eng_rx_tdata[15:8] == 8'h45);
                                rx_dst_match      <= 1'b0;
                                rx_wire_beat      <= rx_wire_beat + 12'd1;
                            end
                            12'd4: begin
                                // Beat 4: {IPv4_TotalLen, IPv4_ID}
                                rx_ip_len         <= eng_rx_tdata[31:16];
                                rx_ip_id          <= eng_rx_tdata[15:0];
                                rx_hdr_word       <= eng_rx_tdata;
                                rx_hdr_add        <= 1'b1;
                                rx_wire_beat      <= rx_wire_beat + 12'd1;
                            end
                            12'd5: begin
                                // Beat 5: {IPv4_FlagsFrag, TTL_Proto}
                                rx_flags_frag     <= eng_rx_tdata[31:16];
                                rx_pkt_is_ip_frag <= eng_rx_tdata[29] || (eng_rx_tdata[28:16] != 13'd0);
                                rx_hdr_word       <= eng_rx_tdata;
                                rx_hdr_add        <= 1'b1;
                                rx_wire_beat      <= rx_wire_beat + 12'd1;
                                rx_is_ipv4_udp    <= rx_is_ipv4_udp && (eng_rx_tdata[7:0] == 8'h11);

                                if (rx_is_ipv4_udp && (eng_rx_tdata[7:0] == 8'h11)) begin
                                    if (eng_rx_tdata[29] || (eng_rx_tdata[28:16] != 13'd0)) begin
                                        // IPv4-fragmented path: reset only when a new IP_ID arrives
                                        if (!rx_active_valid || (rx_active_ip_id != rx_ip_id)) begin
                                            rx_active_ip_id      <= rx_ip_id;
                                            rx_active_src_mac    <= rx_wire_src_mac;
                                            rx_active_valid      <= 1'b1;
                                            rx_bytes_rcvd        <= 16'd0;
                                            rx_bytes_total       <= 16'd0;
                                            rx_saw_last          <= 1'b0;
                                            rx_csum_clr          <= 1'b1;
                                            frag_udp_hdr_csum    <= 16'hFFFF;
                                            reasm_coverage       <= {REASM_BLOCKS{1'b0}};
                                            reasm_blocks_covered <= 10'd0;
                                            reasm_total_bytes    <= 16'd0;
                                            reasm_last_seen      <= 1'b0;
                                            df_active_valid      <= 1'b0;
                                            df_frag_bitmap       <= 32'd0;
                                        end
                                    end else begin
                                        // Unfragmented UDP/IPv4 path: buffer in rx_pkt_ram without disturbing IPv4-frag state
                                        pkt_csum_clr      <= 1'b1;
                                        pkt_src_mac       <= rx_wire_src_mac;
                                        pkt_udp_hdr_csum  <= 16'hFFFF;
                                        pkt_ip_pay_bytes  <= rx_ip_len - 16'd20;
                                        rx_tail_valid     <= 1'b0;
                                    end
                                end
                            end
                            12'd6: begin
                                // Beat 6: {IPv4_HdrCsum, SrcIP[31:16]}
                                rx_src_ip[31:16]  <= eng_rx_tdata[15:0];
                                rx_hdr_word       <= eng_rx_tdata;
                                rx_hdr_add        <= 1'b1;
                                rx_wire_beat      <= rx_wire_beat + 12'd1;
                            end
                            12'd7: begin
                                // Beat 7: {SrcIP[15:0], DstIP[31:16]}
                                rx_src_ip[15:0]   <= eng_rx_tdata[31:16];
                                rx_dst_ip[31:16]  <= eng_rx_tdata[15:0];
                                rx_hdr_word       <= eng_rx_tdata;
                                rx_hdr_add        <= 1'b1;
                                rx_wire_beat      <= rx_wire_beat + 12'd1;
                            end
                            12'd8: begin
                                // Beat 8: {DstIP[15:0], IP_Payload_Half0}
                                rx_dst_ip[15:0]   <= eng_rx_tdata[31:16];
                                rx_hdr_word       <= {eng_rx_tdata[31:16], 16'h0000};
                                rx_hdr_add        <= 1'b1;
                                rx_prev_half      <= eng_rx_tdata[15:0];
                                rx_wr_ptr         <= rx_frag_w_start;
                                rx_pkt_wr_ptr     <= 10'd0;
                                rx_wire_beat      <= rx_wire_beat + 12'd1;
                                rx_dst_match      <= ({rx_dst_ip[31:16], eng_rx_tdata[31:16]} == local_src_ip) ||
                                                     ({rx_dst_ip[31:16], eng_rx_tdata[31:16]} == {local_src_ip[31:8], 8'hFF}) ||
                                                     ({rx_dst_ip[31:16], eng_rx_tdata[31:16]} == 32'hFFFF_FFFF);
                                if (rx_pkt_is_ip_frag) begin
                                    frag_src_ip <= rx_src_ip;
                                    frag_dst_ip <= {rx_dst_ip[31:16], eng_rx_tdata[31:16]};
                                end else begin
                                    pkt_src_ip  <= rx_src_ip;
                                    pkt_dst_ip  <= {rx_dst_ip[31:16], eng_rx_tdata[31:16]};
                                end
                            end
                            default: begin
                                if (rx_pkt_is_ip_frag) begin
                                    if (rx_is_ipv4_udp && rx_dst_match && rx_wr_ptr < rx_wr_end) begin
                                        reasm_wr_en   <= 1'b1;
                                        reasm_wr_be   <= 4'b1111;
                                        reasm_wr_addr <= rx_wr_ptr;
                                        reasm_wr_data <= {rx_prev_half, eng_rx_tdata[31:16]};
                                        if (rx_frag_off_8b == 13'd0 && rx_wr_ptr == 12'd0) begin
                                            frag_udp_src_port <= rx_prev_half;
                                            frag_udp_dst_port <= eng_rx_tdata[31:16];
                                        end
                                        if (rx_frag_off_8b == 13'd0 && rx_wr_ptr == 12'd1) begin
                                            frag_udp_len_bytes <= rx_prev_half;
                                            frag_udp_hdr_csum  <= eng_rx_tdata[31:16];
                                        end
                                        rx_wr_ptr <= rx_wr_ptr + 12'd1;

                                        if (!rx_wr_ptr[0] && !reasm_coverage[rx_wr_ptr[10:1]]) begin
                                            reasm_coverage[rx_wr_ptr[10:1]] <= 1'b1;
                                            reasm_blocks_covered            <= reasm_blocks_covered + 10'd1;
                                        end
                                    end
                                end else begin
                                    if (rx_is_ipv4_udp && rx_dst_match && (rx_pkt_wr_ptr < {1'b0, pkt_ip_pay_bytes[10:2]}) && (rx_pkt_wr_ptr < 10'd511)) begin
                                        rx_pkt_wr_en   <= 1'b1;
                                        rx_pkt_wr_addr <= rx_pkt_wr_ptr[8:0];
                                        rx_pkt_wr_data <= {rx_prev_half, eng_rx_tdata[31:16]};
                                        if (rx_pkt_wr_ptr == 10'd0) begin
                                            pkt_udp_src_port <= rx_prev_half;
                                            pkt_udp_dst_port <= eng_rx_tdata[31:16];
                                        end
                                        if (rx_pkt_wr_ptr == 10'd1) begin
                                            pkt_udp_len_bytes <= rx_prev_half;
                                            pkt_udp_hdr_csum  <= eng_rx_tdata[31:16];
                                        end
                                        rx_pkt_wr_ptr <= rx_pkt_wr_ptr + 10'd1;
                                    end
                                end
                                rx_prev_half <= eng_rx_tdata[15:0];

                                if (eng_rx_tlast) begin
                                    rx_wire_beat <= 12'd0;
                                    if (!rx_is_ipv4_udp || !rx_dst_match) begin
                                        if (rx_pkt_is_ip_frag)
                                            rx_active_valid <= 1'b0;
                                    end else if (rx_hdr_csum_accum != 16'hFFFF) begin
                                        rx_csum_err_pulse <= 1'b1;
                                        stat_csum_errs <= stat_csum_errs + 32'd1;
                                        if (rx_pkt_is_ip_frag)
                                            rx_active_valid <= 1'b0;
                                    end else if (rx_pkt_is_ip_frag) begin
                                        if (!rx_frag_mf) begin
                                            rx_bytes_total    <= {rx_frag_off_8b, 3'b000} + rx_ip_pay_bytes;
                                            rx_saw_last       <= 1'b1;
                                            reasm_last_seen   <= 1'b1;
                                            reasm_total_bytes <= {rx_frag_off_8b, 3'b000} + rx_ip_pay_bytes;
                                        end
                                        rx_bytes_rcvd <= rx_bytes_rcvd + rx_ip_pay_bytes;
                                        rx_state      <= R_REASM_CHECK;
                                    end else begin
                                        // Unfragmented UDP/IPv4: capture trailing 1..3 unaligned bytes if any
                                        if (pkt_ip_pay_bytes[1:0] == 2'b01) begin
                                            rx_tail_word  <= {eng_rx_tdata[15:8], 24'd0};
                                            rx_tail_valid <= 1'b1;
                                        end else if (pkt_ip_pay_bytes[1:0] == 2'b10) begin
                                            rx_tail_word  <= {eng_rx_tdata[15:0], 16'd0};
                                            rx_tail_valid <= 1'b1;
                                        end else if (pkt_ip_pay_bytes[1:0] == 2'b11) begin
                                            rx_tail_word  <= {rx_prev_half, eng_rx_tdata[31:24], 8'd0};
                                            rx_tail_valid <= 1'b1;
                                        end else begin
                                            rx_tail_valid <= 1'b0;
                                        end
                                        walk_from_pkt_ram <= 1'b1;
                                        rx_state          <= R_PKT_TAIL_WR;
                                    end
                                end else begin
                                    rx_wire_beat <= rx_wire_beat + 12'd1;
                                end
                            end
                        endcase
                        if (eng_rx_tlast && rx_wire_beat <= 12'd8) rx_wire_beat <= 12'd0;
                    end
                end

                // Write optional unaligned tail word & latch packet metadata into walker
                R_PKT_TAIL_WR: begin
                    if (rx_tail_valid && rx_pkt_wr_ptr < 10'd511) begin
                        rx_pkt_wr_en   <= 1'b1;
                        rx_pkt_wr_addr <= rx_pkt_wr_ptr[8:0];
                        rx_pkt_wr_data <= rx_tail_word;
                    end
                    walk_src_ip        <= pkt_src_ip;
                    walk_dst_ip        <= pkt_dst_ip;
                    walk_udp_len_bytes <= pkt_udp_len_bytes;
                    walk_udp_src_port  <= pkt_udp_src_port;
                    walk_udp_dst_port  <= pkt_udp_dst_port;
                    walk_udp_hdr_csum  <= pkt_udp_hdr_csum;
                    walk_src_mac       <= pkt_src_mac;
                    rx_state           <= R_UDP_CSUM_W0;
                end

                // 1-cycle registered evaluation of IPv4 fragment coverage
                R_REASM_CHECK: begin
                    if (reasm_last_seen && rx_dst_match &&
                        (reasm_total_bytes > 16'd0) &&
                        (reasm_blocks_covered == ((reasm_total_bytes + 16'd7) >> 3))) begin
                        rx_active_valid    <= 1'b0;
                        rx_bytes_rcvd      <= 16'd0;
                        rx_saw_last        <= 1'b0;
                        walk_from_pkt_ram  <= 1'b0;
                        walk_src_ip        <= frag_src_ip;
                        walk_dst_ip        <= frag_dst_ip;
                        walk_udp_len_bytes <= frag_udp_len_bytes;
                        walk_udp_src_port  <= frag_udp_src_port;
                        walk_udp_dst_port  <= frag_udp_dst_port;
                        walk_udp_hdr_csum  <= frag_udp_hdr_csum;
                        walk_src_mac       <= rx_active_src_mac;
                        rx_state           <= R_UDP_CSUM_W0;
                    end else begin
                        if (!rx_dst_match)
                            rx_active_valid <= 1'b0;
                        rx_state <= R_IDLE;
                    end
                end

                // Flush 3-stage rx_pseudo_csum pipeline & latch final payload checksum
                R_UDP_CSUM_W0: begin
                    rx_state <= R_UDP_CSUM_W1;
                end

                R_UDP_CSUM_W1: begin
                    walk_udp_csum_accum <= walk_from_pkt_ram ? pkt_udp_csum_accum : frag_udp_csum_accum;
                    rx_state            <= R_UDP_CSUM_W2;
                end

                R_UDP_CSUM_W2: begin
                    walker_rd_addr <= 12'd2; // Prime Word 2 ("RTPS")
                    rx_state       <= R_CHECK_UDP;
                end

                // Fold UDP pseudo-header & check == 0xFFFF
                R_CHECK_UDP: begin
                    walk_dst_valid       <= 1'b0;
                    walk_dst_guid_prefix <= 96'd0;
                    walk_dgram_words     <= (walk_udp_len_bytes + 16'd3) >> 2;
                    df_in_qos            <= 1'b0;
                    if (!rx_chk_valid) begin
                        rx_csum_err_pulse <= 1'b1;
                        stat_csum_errs <= stat_csum_errs + 32'd1;
                        rx_state       <= R_IDLE;
                    end else begin
                        walker_rd_addr <= 12'd3; // Prime Word 3 (Ver + VendorId)
                        rx_state       <= R_RTPS_HDR_0;
                    end
                end

                R_RTPS_HDR_0: begin
                    walker_rd_addr <= 12'd4; // Prime Word 4 (GuidPrefix[95:64])
                    if (walker_dout != 32'h5254_5053) begin
                        rx_state <= R_IDLE;
                    end else begin
                        rx_state <= R_RTPS_HDR_1;
                    end
                end

                R_RTPS_HDR_1: begin
                    walk_rtps_ver_vendor <= walker_dout;
                    walker_rd_addr       <= 12'd5; // Prime Word 5 (GuidPrefix[63:32])
                    if (walker_dout[31:24] != 8'h02) begin
                        rx_state <= R_IDLE;
                    end else begin
                        rx_state <= R_RTPS_GUID_0;
                    end
                end

                R_RTPS_GUID_0: begin
                    walk_guid_prefix[95:64] <= walker_dout;
                    walker_rd_addr          <= 12'd6;
                    rx_state                <= R_RTPS_GUID_1;
                end

                R_RTPS_GUID_1: begin
                    walk_guid_prefix[63:32] <= walker_dout;
                    walker_rd_addr          <= 12'd7;
                    rx_state                <= R_RTPS_GUID_2;
                end

                R_RTPS_GUID_2: begin
                    walk_guid_prefix[31:0] <= walker_dout;
                    walk_cur_word          <= 12'd7;
                    rx_state               <= R_CAM_WAIT;
                end

                R_CAM_WAIT: begin
                    // 1-cycle wait for registered cam_hit_r
                    rx_state <= R_CAM_MATCH;
                end

                R_CAM_MATCH: begin
                    if (cam_hit_r[0]) begin
                        walk_cam_hit    <= 1'b1;
                        walk_cam_idx    <= 2'd0;
                        cam_reader_id_r <= active_rd_edt[0][5];
                        cam_writer_id_r <= active_rd_edt[0][4];
                        cam_last_seq_r  <= last_rx_seq_num_by_cam[0];
                        rx_state        <= R_SUBMSG_HDR;
                    end else if (cam_hit_r[1]) begin
                        walk_cam_hit    <= 1'b1;
                        walk_cam_idx    <= 2'd1;
                        cam_reader_id_r <= active_rd_edt[1][5];
                        cam_writer_id_r <= active_rd_edt[1][4];
                        cam_last_seq_r  <= last_rx_seq_num_by_cam[1];
                        rx_state        <= R_SUBMSG_HDR;
                    end else if (cam_hit_r[2]) begin
                        walk_cam_hit    <= 1'b1;
                        walk_cam_idx    <= 2'd2;
                        cam_reader_id_r <= active_rd_edt[2][5];
                        cam_writer_id_r <= active_rd_edt[2][4];
                        cam_last_seq_r  <= last_rx_seq_num_by_cam[2];
                        rx_state        <= R_SUBMSG_HDR;
                    end else if (cam_hit_r[3]) begin
                        walk_cam_hit    <= 1'b1;
                        walk_cam_idx    <= 2'd3;
                        cam_reader_id_r <= active_rd_edt[3][5];
                        cam_writer_id_r <= active_rd_edt[3][4];
                        cam_last_seq_r  <= last_rx_seq_num_by_cam[3];
                        rx_state        <= R_SUBMSG_HDR;
                    end else begin
                        walk_cam_hit <= 1'b0;
                        rx_state     <= R_IDLE;
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
                                if (walker_dout[17]) begin
                                    walk_cur_word  <= walk_cur_word + 12'd1 + submsg_octets[13:2];
                                    rx_state       <= R_SUBMSG_HDR_WAIT2;
                                end else begin
                                    walker_rd_addr <= walk_cur_word + 12'd1;
                                    rx_state       <= R_INFO_TS_WAIT;
                                end
                            end
                            8'h0E: begin // INFO_DST (0x0E)
                                if (submsg_octets != 16'd12) begin
                                    rx_state <= R_IDLE;
                                end else begin
                                    walker_rd_addr <= walk_cur_word + 12'd1;
                                    rx_state       <= R_INFO_DST_WAIT;
                                end
                            end
                            8'h15: begin // DATA (0x15)
                                df_in_qos      <= 1'b0;
                                walker_rd_addr <= walk_cur_word + 12'd2;
                                rx_state       <= R_DATA_HDR_1;
                            end
                            8'h16: begin // DATA_FRAG (0x16)
                                if (walk_from_pkt_ram) begin
                                    df_in_qos      <= 1'b1;
                                    walker_rd_addr <= walk_cur_word + 12'd2;
                                    rx_state       <= R_DF_HDR_1;
                                end else begin
                                    stat_skipped_submsg <= stat_skipped_submsg + 32'd1;
                                    if (submsg_octets == 16'd0) begin
                                        rx_state <= R_IDLE;
                                    end else begin
                                        walk_cur_word  <= walk_cur_word + 12'd1 + submsg_octets[13:2];
                                        rx_state       <= R_SUBMSG_HDR_WAIT2;
                                    end
                                end
                            end
                            default: begin
                                stat_skipped_submsg <= stat_skipped_submsg + 32'd1;
                                if (submsg_octets == 16'd0) begin
                                    rx_state <= R_IDLE;
                                end else begin
                                    walk_cur_word  <= walk_cur_word + 12'd1 + submsg_octets[13:2];
                                    rx_state       <= R_SUBMSG_HDR_WAIT2;
                                end
                            end
                        endcase
                    end
                end

                R_SUBMSG_HDR_WAIT2: begin
                    walker_rd_addr <= walk_cur_word;
                    rx_state       <= R_SUBMSG_HDR_WAIT;
                end

                R_SUBMSG_HDR_WAIT: begin
                    rx_state <= R_SUBMSG_HDR;
                end

                R_INFO_TS_WAIT: begin
                    walker_rd_addr <= walk_cur_word + 12'd2;
                    rx_state       <= R_INFO_TS_0;
                end

                R_INFO_TS_0: begin
                    walk_ts[63:32] <= walk_submsg_flags[0] ?
                                      {walker_dout[7:0], walker_dout[15:8], walker_dout[23:16], walker_dout[31:24]} :
                                      walker_dout;
                    walker_rd_addr <= walk_next_submsg_word;
                    rx_state       <= R_INFO_TS_1;
                end

                R_INFO_TS_1: begin
                    walk_ts[31:0]  <= walk_submsg_flags[0] ?
                                      {walker_dout[7:0], walker_dout[15:8], walker_dout[23:16], walker_dout[31:24]} :
                                      walker_dout;
                    walk_cur_word  <= walk_next_submsg_word;
                    rx_state       <= R_SUBMSG_HDR;
                end

                R_INFO_DST_WAIT: begin
                    walker_rd_addr <= walk_cur_word + 12'd2;
                    rx_state       <= R_INFO_DST_0;
                end

                R_INFO_DST_0: begin
                    walk_dst_guid_prefix[95:64] <= walker_dout;
                    walker_rd_addr              <= walk_cur_word + 12'd3;
                    rx_state                    <= R_INFO_DST_1;
                end

                R_INFO_DST_1: begin
                    walk_dst_guid_prefix[63:32] <= walker_dout;
                    walker_rd_addr              <= walk_next_submsg_word;
                    rx_state                    <= R_INFO_DST_2;
                end

                R_INFO_DST_2: begin
                    walk_dst_guid_prefix[31:0] <= walker_dout;
                    walk_dst_valid             <= 1'b1;
                    walk_cur_word              <= walk_next_submsg_word;
                    rx_state                   <= R_SUBMSG_HDR;
                end

                R_DATA_HDR_1: begin
                    walker_rd_addr <= walk_cur_word + 12'd3;
                    rx_state       <= R_DATA_READER_ID;
                end

                R_DATA_READER_ID: begin
                    walk_reader_id <= walker_dout;
                    walker_rd_addr <= walk_cur_word + 12'd4;
                    if ((walker_dout != 32'h0000_0000) &&
                        (cam_reader_id_r != 32'h0000_0000) &&
                        (walker_dout != cam_reader_id_r)) begin
                        walk_cur_word  <= walk_next_submsg_word;
                        rx_state       <= R_SUBMSG_HDR_WAIT2;
                    end else begin
                        rx_state       <= R_DATA_HDR_2;
                    end
                end

                R_DATA_HDR_2: begin
                    walk_writer_id <= walker_dout;
                    walker_rd_addr <= walk_cur_word + 12'd5;
                    if (!info_dst_matches_local || (walker_dout != cam_writer_id_r)) begin
                        walk_cur_word  <= walk_next_submsg_word;
                        rx_state       <= R_SUBMSG_HDR_WAIT2;
                    end else begin
                        rx_state       <= R_DATA_SEQ_HI;
                    end
                end

                R_DATA_SEQ_HI: begin
                    walk_seq_num[63:32] <= walk_submsg_flags[0] ?
                                           {walker_dout[7:0], walker_dout[15:8], walker_dout[23:16], walker_dout[31:24]} :
                                           walker_dout;
                    walker_rd_addr      <= walk_cur_word + 12'd5;
                    rx_state            <= R_DATA_SEQ_LO;
                end

                R_DATA_SEQ_LO: begin
                    walk_seq_num[31:0] <= walk_submsg_flags[0] ?
                                          {walker_dout[7:0], walker_dout[15:8], walker_dout[23:16], walker_dout[31:24]} :
                                          walker_dout;
                    walker_rd_addr     <= walk_cur_word + 12'd6;
                    if (walk_submsg_flags[1]) begin
                        walk_cur_word <= walk_cur_word + 12'd6;
                        rx_state      <= R_SKIP_QOS_WAIT;
                    end else begin
                        pending_payload_word_off <= walk_cur_word + 12'd7;
                        if (walk_from_pkt_ram) begin
                            rx_state <= R_COPY_DATA_PRIME;
                        end else begin
                            rx_state <= R_COMMIT_SAMPLE;
                        end
                    end
                end

                // Copy unfragmented DATA packet from rx_pkt_ram into reasm_ram[fill_slot]
                R_COPY_DATA_PRIME: begin
                    if (walk_seq_num <= cam_last_seq_r) begin
                        stat_stale_drops <= stat_stale_drops + 32'd1;
                        rx_state         <= R_IDLE;
                    end else begin
                        df_active_valid <= 1'b0;
                        df_frag_bitmap  <= 32'd0;
                        copy_word_idx   <= 12'd0;
                        walker_rd_addr  <= 12'd0;
                        rx_state        <= R_COPY_DATA_LOOP;
                    end
                end

                R_COPY_DATA_LOOP: begin
                    if (walker_rd_addr == 12'd0 && copy_word_idx == 12'd0) begin
                        walker_rd_addr <= 12'd1;
                    end else if (copy_word_idx < walk_dgram_words) begin
                        reasm_wr_en    <= 1'b1;
                        reasm_wr_be    <= 4'b1111;
                        reasm_wr_addr  <= copy_word_idx;
                        reasm_wr_data  <= rx_pkt_dout;
                        copy_word_idx  <= copy_word_idx + 12'd1;
                        walker_rd_addr <= walker_rd_addr + 12'd1;
                    end else begin
                        rx_state <= R_COMMIT_SAMPLE;
                    end
                end

                // DATA_FRAG (0x16) Submessage Parser
                R_DF_HDR_1: begin
                    walker_rd_addr <= walk_cur_word + 12'd3;
                    rx_state       <= R_DF_READER_ID;
                end

                R_DF_READER_ID: begin
                    walk_reader_id <= walker_dout;
                    walker_rd_addr <= walk_cur_word + 12'd4;
                    if ((walker_dout != 32'h0000_0000) &&
                        (cam_reader_id_r != 32'h0000_0000) &&
                        (walker_dout != cam_reader_id_r)) begin
                        walk_cur_word  <= walk_next_submsg_word;
                        rx_state       <= R_SUBMSG_HDR_WAIT2;
                    end else begin
                        rx_state       <= R_DF_WRITER_ID;
                    end
                end

                R_DF_WRITER_ID: begin
                    walk_writer_id <= walker_dout;
                    walker_rd_addr <= walk_cur_word + 12'd5;
                    if (!info_dst_matches_local || (walker_dout != cam_writer_id_r)) begin
                        walk_cur_word  <= walk_next_submsg_word;
                        rx_state       <= R_SUBMSG_HDR_WAIT2;
                    end else begin
                        rx_state       <= R_DF_SEQ_HI;
                    end
                end

                R_DF_SEQ_HI: begin
                    walk_seq_num[63:32] <= walk_submsg_flags[0] ?
                                           {walker_dout[7:0], walker_dout[15:8], walker_dout[23:16], walker_dout[31:24]} :
                                           walker_dout;
                    walker_rd_addr      <= walk_cur_word + 12'd6;
                    rx_state            <= R_DF_SEQ_LO;
                end

                R_DF_SEQ_LO: begin
                    walk_seq_num[31:0] <= walk_submsg_flags[0] ?
                                          {walker_dout[7:0], walker_dout[15:8], walker_dout[23:16], walker_dout[31:24]} :
                                          walker_dout;
                    walker_rd_addr     <= walk_cur_word + 12'd7;
                    rx_state           <= R_DF_FRAG_START;
                end

                R_DF_FRAG_START: begin
                    df_frag_start  <= walk_submsg_flags[0] ?
                                      {walker_dout[7:0], walker_dout[15:8], walker_dout[23:16], walker_dout[31:24]} :
                                      walker_dout;
                    walker_rd_addr <= walk_cur_word + 12'd8;
                    rx_state       <= R_DF_FRAG_INFO;
                end

                R_DF_FRAG_INFO: begin
                    df_frags_in_submsg <= walk_submsg_flags[0] ?
                                          {walker_dout[23:16], walker_dout[31:24]} : walker_dout[31:16];
                    df_frag_size       <= walk_submsg_flags[0] ?
                                          {walker_dout[7:0], walker_dout[15:8]} : walker_dout[15:0];
                    walker_rd_addr     <= walk_cur_word + 12'd9;
                    rx_state           <= R_DF_SAMPLE_SIZE;
                end

                R_DF_SAMPLE_SIZE: begin
                    df_sample_size <= walk_submsg_flags[0] ?
                                      {walker_dout[7:0], walker_dout[15:8], walker_dout[23:16], walker_dout[31:24]} :
                                      walker_dout;
                    if (walk_submsg_flags[1]) begin
                        walk_cur_word <= walk_cur_word + 12'd9;
                        rx_state      <= R_SKIP_QOS;
                    end else begin
                        df_pay_start_word <= walk_cur_word + 12'd9;
                        rx_state          <= R_DF_VALIDATE;
                    end
                end

                R_DF_VALIDATE: begin
                    if (walk_seq_num <= cam_last_seq_r) begin
                        stat_stale_drops <= stat_stale_drops + 32'd1;
                        rx_state         <= R_IDLE;
                    end else if ((df_frag_start < 32'd1) || (df_frag_start > 32'd32) ||
                                 (df_frags_in_submsg < 16'd1) ||
                                 ((df_frag_start + {16'd0, df_frags_in_submsg}) > 32'd33) ||
                                 (df_frag_size == 16'd0) ||
                                 (df_sample_size == 32'd0) ||
                                 (df_sample_size > ((REASM_DEPTH - 16) * 4))) begin
                        rx_state <= R_IDLE;
                    end else if (df_active_valid && (df_active_cam_idx == walk_cam_idx) && (walk_seq_num < df_active_seq)) begin
                        stat_stale_drops <= stat_stale_drops + 32'd1;
                        rx_state         <= R_IDLE;
                    end else if (df_active_valid && (df_active_cam_idx == walk_cam_idx) && (walk_seq_num == df_active_seq)) begin
                        if ((df_active_sample_size != df_sample_size[15:0]) || (df_active_frag_size != df_frag_size)) begin
                            df_active_valid <= 1'b0;
                            df_frag_bitmap  <= 32'd0;
                            rx_state        <= R_IDLE;
                        end else begin
                            df_calc_phase <= 2'd1;
                            df_calc_cnt   <= 6'd0;
                            df_calc_acc   <= 16'd0;
                            rx_state      <= R_DF_CALC;
                        end
                    end else begin
                        // New writerSN (or newer sample preempting incomplete older one)
                        rx_active_valid       <= 1'b0;
                        df_active_valid       <= 1'b1;
                        df_active_cam_idx     <= walk_cam_idx;
                        df_active_seq         <= walk_seq_num;
                        df_active_sample_size <= df_sample_size[15:0];
                        df_active_frag_size   <= df_frag_size;
                        df_frag_bitmap        <= 32'd0;
                        df_calc_phase         <= 2'd0;
                        df_calc_rem           <= df_sample_size[15:0];
                        df_calc_cnt           <= 6'd0;
                        df_calc_acc           <= 16'd0;
                        rx_state              <= R_DF_CALC;
                    end
                end

                R_DF_CALC: begin
                    case (df_calc_phase)
                        2'd0: begin
                            // Compute df_active_total_frags = ceil(sampleSize / fragSize)
                            if (df_calc_rem > df_frag_size && df_calc_cnt < 6'd32) begin
                                df_calc_rem <= df_calc_rem - df_frag_size;
                                df_calc_cnt <= df_calc_cnt + 6'd1;
                            end else if (df_calc_cnt >= 6'd32) begin
                                df_active_valid <= 1'b0;
                                rx_state        <= R_IDLE;
                            end else begin
                                df_active_total_frags <= df_calc_cnt + 6'd1;
                                df_calc_cnt           <= 6'd0;
                                df_calc_acc           <= 16'd0;
                                df_calc_phase         <= 2'd1;
                            end
                        end
                        2'd1: begin
                            // Compute df_byte_off = (fragStart - 1) * fragSize
                            if (df_calc_cnt < (df_frag_start[5:0] - 6'd1)) begin
                                df_calc_acc <= df_calc_acc + df_frag_size;
                                df_calc_cnt <= df_calc_cnt + 6'd1;
                            end else begin
                                df_byte_off   <= df_calc_acc;
                                df_calc_cnt   <= 6'd0;
                                df_calc_acc   <= 16'd0;
                                df_calc_phase <= 2'd2;
                            end
                        end
                        2'd2: begin
                            // Compute df_submsg_bytes = fragsInSubmsg * fragSize
                            if (df_calc_cnt < df_frags_in_submsg[5:0]) begin
                                df_calc_acc <= df_calc_acc + df_frag_size;
                                df_calc_cnt <= df_calc_cnt + 6'd1;
                            end else begin
                                df_submsg_bytes <= df_calc_acc;
                                df_calc_phase   <= 2'd3;
                            end
                        end
                        default: begin
                            // Bound chunk size to remaining sampleSize and UDP packet length
                            if (df_byte_off >= df_sample_size[15:0]) begin
                                rx_state <= R_IDLE;
                            end else begin
                                df_chunk_bytes   <= (df_submsg_bytes <= (df_sample_size[15:0] - df_byte_off)) ?
                                                    df_submsg_bytes : (df_sample_size[15:0] - df_byte_off);
                                df_dst_byte_addr <= 16'd64 + df_byte_off; // Word 16 = CDR_LE, Word 17 = FastSample
                                df_shift         <= df_byte_off[1:0];
                                df_src_word_ptr  <= df_pay_start_word;
                                df_prev_word     <= 32'd0;
                                df_first_beat    <= 1'b1;
                                walker_rd_addr   <= df_pay_start_word;
                                rx_state         <= R_DF_COPY_PRIME;
                            end
                        end
                    endcase
                end

                R_DF_COPY_PRIME: begin
                    // Pre-compute fragment bitmasks in R_DF_COPY_PRIME so R_DF_COPY_LOOP has no barrel shifters
                    df_frag_mask_add_r <= (((df_frags_in_submsg >= 16'd32) ? 32'hFFFF_FFFF : ((32'd1 << df_frags_in_submsg[4:0]) - 32'd1))
                                           << (df_frag_start[4:0] - 5'd1));
                    df_frag_mask_all_r <= (df_active_total_frags >= 6'd32) ? 32'hFFFF_FFFF : ((32'd1 << df_active_total_frags[4:0]) - 32'd1);
                    // Check packet isn't truncated past UDP length
                    if (({df_pay_start_word, 2'b00} + df_chunk_bytes) > walk_udp_len_bytes || df_chunk_bytes == 16'd0) begin
                        rx_state <= R_IDLE;
                    end else begin
                        df_rem_bytes   <= df_chunk_bytes;
                        walker_rd_addr <= df_src_word_ptr + 12'd1;
                        rx_state       <= R_DF_COPY_LOOP;
                    end
                end

                R_DF_COPY_LOOP: begin : df_copy_blk
                    reg [1:0]  s_byte;
                    reg [2:0]  avail_bytes;
                    reg [2:0]  cnt_bytes;
                    reg [3:0]  lane_be;
                    reg [31:0] shifted_word;
                    integer    bi;

                    s_byte      = df_first_beat ? df_shift : 2'd0;
                    avail_bytes = 3'd4 - {1'b0, s_byte};
                    cnt_bytes   = (df_rem_bytes < {13'd0, avail_bytes}) ? df_rem_bytes[2:0] : avail_bytes;

                    for (bi = 0; bi < 4; bi = bi + 1) begin
                        lane_be[3 - bi] = (bi >= s_byte) && (bi < (s_byte + cnt_bytes));
                    end

                    case (df_shift)
                        2'd0:    shifted_word = rx_pkt_dout;
                        2'd1:    shifted_word = {df_prev_word[7:0],  rx_pkt_dout[31:8]};
                        2'd2:    shifted_word = {df_prev_word[15:0], rx_pkt_dout[31:16]};
                        default: shifted_word = {df_prev_word[23:0], rx_pkt_dout[31:24]};
                    endcase

                    reasm_wr_en      <= 1'b1;
                    reasm_wr_be      <= lane_be;
                    reasm_wr_addr    <= df_dst_byte_addr[13:2];
                    reasm_wr_data    <= shifted_word;
                    df_prev_word     <= rx_pkt_dout;
                    df_first_beat    <= 1'b0;
                    df_dst_byte_addr <= df_dst_byte_addr + 16'd4;
                    df_rem_bytes     <= df_rem_bytes - {13'd0, cnt_bytes};
                    walker_rd_addr   <= walker_rd_addr + 12'd1;

                    if (df_rem_bytes <= {13'd0, cnt_bytes}) begin
                        // Update received-fragment bitmask using pre-computed masks
                        df_frag_bitmap <= df_frag_bitmap | df_frag_mask_add_r;
                        if (((df_frag_bitmap | df_frag_mask_add_r) & df_frag_mask_all_r) == df_frag_mask_all_r) begin
                            df_synth_idx <= 4'd0;
                            rx_state     <= R_DF_SYNTH_HDR;
                        end else if (walk_next_submsg_word < walk_dgram_words) begin
                            walk_cur_word  <= walk_next_submsg_word;
                            walker_rd_addr <= walk_next_submsg_word;
                            rx_state       <= R_SUBMSG_HDR_WAIT;
                        end else begin
                            rx_state <= R_IDLE;
                        end
                    end
                end

                // Synthesize Words 2..15 (RTPS + INFO_TS + DATA 0x15) for Option B compatibility
                R_DF_SYNTH_HDR: begin : df_synth_blk
                    reg [15:0] synth_oct_le;
                    reg [15:0] synth_oct_wire;
                    synth_oct_le   = df_active_sample_size + 16'd20;
                    synth_oct_wire = {synth_oct_le[7:0], synth_oct_le[15:8]};

                    reasm_wr_en   <= 1'b1;
                    reasm_wr_be   <= 4'b1111;
                    reasm_wr_addr <= 12'd2 + {8'd0, df_synth_idx};
                    case (df_synth_idx)
                        4'd0:    reasm_wr_data <= 32'h5254_5053;
                        4'd1:    reasm_wr_data <= walk_rtps_ver_vendor;
                        4'd2:    reasm_wr_data <= walk_guid_prefix[95:64];
                        4'd3:    reasm_wr_data <= walk_guid_prefix[63:32];
                        4'd4:    reasm_wr_data <= walk_guid_prefix[31:0];
                        4'd5:    reasm_wr_data <= 32'h0901_0800;
                        4'd6:    reasm_wr_data <= {walk_ts[39:32], walk_ts[47:40], walk_ts[55:48], walk_ts[63:56]};
                        4'd7:    reasm_wr_data <= {walk_ts[7:0],   walk_ts[15:8],  walk_ts[23:16], walk_ts[31:24]};
                        4'd8:    reasm_wr_data <= {8'h15, 8'h05, synth_oct_wire};
                        4'd9:    reasm_wr_data <= 32'h0000_1000;
                        4'd10:   reasm_wr_data <= walk_reader_id;
                        4'd11:   reasm_wr_data <= walk_writer_id;
                        4'd12:   reasm_wr_data <= {walk_seq_num[39:32], walk_seq_num[47:40], walk_seq_num[55:48], walk_seq_num[63:56]};
                        default: reasm_wr_data <= {walk_seq_num[7:0],   walk_seq_num[15:8],  walk_seq_num[23:16], walk_seq_num[31:24]};
                    endcase

                    if (df_synth_idx == 4'd13) begin
                        pending_payload_word_off <= 12'd17;
                        walk_dgram_words         <= 12'd16 + ((df_active_sample_size + 16'd3) >> 2);
                        df_active_valid          <= 1'b0;
                        df_frag_bitmap           <= 32'd0;
                        rx_state                 <= R_COMMIT_SAMPLE;
                    end else begin
                        df_synth_idx <= df_synth_idx + 4'd1;
                    end
                end

                // Unified Registered Commit State for all RX paths
                R_COMMIT_SAMPLE: begin
                    if (walk_seq_num > cam_last_seq_r) begin
                        if (cam_last_seq_r != 64'd0 &&
                            (walk_seq_num > (cam_last_seq_r + 64'd1))) begin
                            stat_seq_gaps <= stat_seq_gaps +
                                (walk_seq_num[31:0] -
                                 cam_last_seq_r[31:0] -
                                 32'd1);
                        end

                        last_rx_seq_num_by_cam[walk_cam_idx] <= walk_seq_num;
                        last_rx_ts_by_cam[walk_cam_idx]      <= walk_ts;
                        last_rx_seq_num                      <= walk_seq_num;
                        last_rx_ts                           <= walk_ts;
                        last_rx_src_ip                       <= walk_src_ip;
                        last_rx_src_port                     <= walk_udp_src_port;
                        last_rx_dst_port                     <= walk_udp_dst_port;
                        last_rx_cam_idx                      <= walk_cam_idx;
                        last_rx_src_mac                      <= walk_src_mac;
                        stat_rx_samples                      <= stat_rx_samples + 32'd1;
                        rd_match_count[walk_cam_idx]         <= rd_match_count[walk_cam_idx] + 32'd1;
                        latest_payload_word_off              <= pending_payload_word_off;
                        latest_rtps_words                    <= walk_dgram_words - 12'd2;
                        rx_commit_pulse                      <= 1'b1;
                    end else begin
                        stat_stale_drops <= stat_stale_drops + 32'd1;
                    end
                    rx_state <= R_IDLE;
                end

                R_SKIP_QOS_WAIT2: begin
                    walker_rd_addr <= walk_cur_word;
                    rx_state       <= R_SKIP_QOS_WAIT;
                end

                R_SKIP_QOS_WAIT: begin
                    rx_state <= R_SKIP_QOS;
                end

                // Walk Inline QoS parameter list until PID_SENTINEL (0x0001)
                R_SKIP_QOS: begin
                    if ((walk_submsg_flags[0] ? {walker_dout[23:16], walker_dout[31:24]} : walker_dout[31:16]) == 16'h0001) begin
                        if (df_in_qos) begin
                            df_pay_start_word <= walk_cur_word + 12'd1;
                            rx_state          <= R_DF_VALIDATE;
                        end else begin
                            pending_payload_word_off <= walk_cur_word + 12'd2;
                            if (walk_from_pkt_ram) begin
                                rx_state <= R_COPY_DATA_PRIME;
                            end else begin
                                rx_state <= R_COMMIT_SAMPLE;
                            end
                        end
                    end else begin
                        walk_cur_word  <= walk_cur_word + 12'd1 +
                                          (walk_submsg_flags[0] ? {walker_dout[3:0], walker_dout[15:10]} : walker_dout[13:2]);
                        rx_state       <= R_SKIP_QOS_WAIT2;
                    end
                end

                default: rx_state <= R_IDLE;
            endcase
        end
    end

endmodule
