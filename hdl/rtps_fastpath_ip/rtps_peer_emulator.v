`timescale 1ns / 1ps

// =============================================================================
// Module: rtps_peer_emulator (rtps_peer_emulator.v)
//
// Description:
//   Cycle-Accurate In-FPGA RTI Connext DDS / RTPS Peer Participant Emulator.
//   Supports both RTPS DATA_FRAG (0x16) and IPv4-fragmented / single DATA (0x15)
//   over AXI4-Stream:
//     - RX accepts either IPv4-fragmented frames (MF=1 / FragOffset>0) or
//       unfragmented UDP/RTPS frames carrying DATA (0x15) or DATA_FRAG (0x16).
//     - Transforms FastSample payload deterministically (preserving Word 0..3
//       header: cycle_id, flags, tx_timestamp_ns; transforming data[0..1019] via
//       (~word) + {16'h0000, seq_num[5:0], 10'h200}).
//     - TX emits DATA_FRAG (0x16) frames by default (or DATA if <= cfg_frag_size),
//       or 3 IPv4-fragmented DATA (0x15) frames when tx_ip_frag_mode == 1.
//     - Supports hardware fault injection via fault_ctrl:
//         Bit [0]: Drop Fragment #1 (simulates packet loss / incomplete reassembly)
//         Bit [1]: Swap Fragment #0 and Fragment #1 order (out-of-order test)
//         Bit [2]: Corrupt UDP checksum on outgoing Feedback datagram
//         Bit [3]: Emit stale / duplicate SequenceNumber (seqNum does not advance)
// =============================================================================

module rtps_peer_emulator #(
    parameter integer DATA_WIDTH  = 32,
    parameter integer MAX_DGRAM_W = 1536  // 6 KB max UDP datagram (1,536 words)
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // Fault-injection, mode & peer identity configuration from CSRs
    input  wire [31:0]            fault_ctrl,
    input  wire                   tx_ip_frag_mode,
    input  wire [15:0]            cfg_frag_size,
    input  wire [95:0]            peer_guid_prefix,
    input  wire [31:0]            peer_writer_id,
    input  wire [31:0]            peer_reader_id,

    // Ingress AXI4-Stream (from Master Wire TX port)
    input  wire [DATA_WIDTH-1:0]  s_axis_tdata,
    input  wire                   s_axis_tvalid,
    output wire                   s_axis_tready,
    input  wire                   s_axis_tlast,

    // Egress AXI4-Stream (to Master Wire RX port)
    output reg  [DATA_WIDTH-1:0]  m_axis_tdata,
    output reg                    m_axis_tvalid,
    input  wire                   m_axis_tready,
    output reg                    m_axis_tlast
);

    // =========================================================================
    // 1's Complement 16-bit Adder Helpers for IPv4 & UDP Checksums
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
    // Emulator FSM States
    // =========================================================================
    localparam [2:0]
        E_RX_FRAMES    = 3'd0, // Receive incoming Ethernet/IPv4/UDP/RTPS frame
        E_RX_PKT_STAGE = 3'd1, // Copy/reassemble unfragmented DATA / DATA_FRAG from rx_pkt_ram
        E_BUILD_DGRAM  = 3'd2, // Transform FastSample payload into tx_sample_ram
        E_TX_INIT      = 3'd3, // Compute per-packet/datagram UDP checksum & fragment params
        E_TX_STREAM    = 3'd4; // Stream Ethernet + IPv4 + UDP/RTPS over m_axis

    reg [2:0] state;

    // Reassembly buffer for incoming sample:
    //   Words 8..9  = INFO_TS timestamp
    //   Word 16     = CDR_LE (0x0001_0000)
    //   Words 17..  = FastSample payload
    (* ram_style = "block" *) reg [31:0] rx_dgram_ram  [0:MAX_DGRAM_W-1];
    // 1-packet staging RAM for unfragmented UDP/RTPS frames (up to 512 words = 2 KB)
    (* ram_style = "block" *) reg [31:0] rx_pkt_ram    [0:511];
    // Transformed FastSample payload words (0 .. payload_words_cfg-1)
    (* ram_style = "block" *) reg [31:0] tx_sample_ram [0:MAX_DGRAM_W-1];

    // =========================================================================
    // RX Fragment & Packet Parser Registers
    // =========================================================================
    reg [11:0] rx_beat_idx;
    reg [31:0] latched_src_ip;
    reg [31:0] latched_dst_ip;
    reg [15:0] latched_src_port;
    reg [15:0] latched_dst_port;
    reg [15:0] cur_ip_len;
    reg [15:0] cur_ip_id;
    reg [15:0] cur_flags_frag;
    reg        cur_is_ip_frag;
    reg [15:0] rx_accum_bytes;
    reg [15:0] rx_expected_bytes;
    reg        rx_saw_last_frag;

    reg [15:0] rx_prev_half;
    reg [11:0] rx_dgram_wr_ptr;
    reg [9:0]  rx_pkt_wr_ptr;

    wire [12:0] frag_offset_8b  = cur_flags_frag[12:0];
    wire        frag_mf         = cur_flags_frag[13];
    wire [15:0] ip_payload_len  = cur_ip_len - 16'd20;
    wire [11:0] frag_word_start = {frag_offset_8b[10:0], 1'b0};

    assign s_axis_tready = (state == E_RX_FRAMES);
    wire   rx_fire       = s_axis_tvalid && s_axis_tready;

    // Staging copy & DATA_FRAG reassembly state
    reg [2:0]  pkt_substate;
    reg [9:0]  pkt_rd_addr;
    reg [31:0] pkt_ram_dout;
    reg [7:0]  pkt_submsg_id;
    reg [31:0] pkt_df_frag_start;
    reg [15:0] pkt_df_frag_size;
    reg [31:0] pkt_df_sample_size;
    reg [63:0] pkt_df_seq_num;
    reg [63:0] active_df_seq;
    reg        active_df_valid;
    reg [31:0] active_df_bitmap;
    reg [5:0]  active_df_total_frags;
    reg [15:0] df_rem_calc;
    reg [5:0]  df_tot_cnt;
    reg [11:0] copy_src_idx;
    reg [11:0] copy_dst_idx;
    reg [11:0] copy_end_idx;

    always @(posedge clk) begin
        pkt_ram_dout <= rx_pkt_ram[pkt_rd_addr];
    end

    // =========================================================================
    // Outgoing Feedback State & Fragment Sequencer
    // =========================================================================
    reg [63:0] peer_seq_num;
    reg [15:0] peer_ip_id;
    reg [11:0] build_idx;
    reg        build_prime;
    reg [15:0] payload_words_cfg; // e.g. 1024 words (4096 bytes)
    reg [31:0] echo_ts_0;
    reg [31:0] echo_ts_1;

    reg [11:0] rx_ram_rd_addr;
    reg [31:0] rx_ram_dout;
    reg [11:0] tx_sample_rd_addr;
    reg [31:0] tx_sample_dout;

    always @(posedge clk) begin
        rx_ram_dout <= rx_dgram_ram[rx_ram_rd_addr];
    end

    // Effective TX fragment size (4B-aligned, 4..1404 bytes)
    wire [15:0] eff_frag_bytes = (cfg_frag_size < 16'd4)    ? 16'd4 :
                                 (cfg_frag_size > 16'd1404) ? 16'd1404 :
                                 {cfg_frag_size[15:2], 2'b00};

    wire [15:0] sample_total_bytes = {2'b00, payload_words_cfg[11:0], 2'b00} + 16'd4; // CDR_LE (4B) + payload
    wire        use_single_data    = (!tx_ip_frag_mode) && (sample_total_bytes <= eff_frag_bytes);
    wire        use_data_frag      = (!tx_ip_frag_mode) && (sample_total_bytes > eff_frag_bytes);

    reg [15:0] total_frags_tx;
    reg [15:0] frag_step;
    reg [15:0] active_frag_idx;
    reg [15:0] tx_bytes_remaining;
    reg [15:0] tx_frag_offset_bytes;
    reg [15:0] cur_frag_ser_bytes;
    reg [11:0] cur_frag_ser_words;
    reg [11:0] cur_frag_bram_base;
    reg [11:0] cur_frag_bram_words;
    reg [11:0] cur_frag_words;     // UDP+RTPS words in current IPv4 packet
    reg [15:0] cur_frag_ip_len;
    reg [15:0] cur_frag_flags;
    reg [15:0] cur_frag_hdr_csum;
    reg [15:0] cur_udp_len_bytes;
    reg [15:0] udp_csum_final;

    // TX Init UDP Checksum Scan
    reg [1:0]  tx_init_phase;
    reg [11:0] scan_cnt;
    reg [15:0] scan_csum_accum;
    reg [2:0]  csum_pipe_cnt;
    reg [15:0] tx_frag_csum_seed;

    reg [11:0] tx_dgram_word_idx;
    reg [11:0] tx_beat_idx;
    reg [15:0] tx_prev_half;

    wire tx_beat_load = (state == E_TX_STREAM) && (!m_axis_tvalid || m_axis_tready);
    always @(posedge clk) begin
        if (state != E_TX_STREAM || tx_beat_load) begin
            tx_sample_dout <= tx_sample_ram[tx_sample_rd_addr];
        end
    end

    // Header field helpers for TX UDP/RTPS Mux
    wire [15:0] ipfrag_total_dgram_words = payload_words_cfg[11:0] + 12'd17;
    wire [15:0] ipfrag_udp_total_bytes   = {2'b00, ipfrag_total_dgram_words[11:0], 2'b00};
    wire [15:0] data_octets_le           = {2'b00, payload_words_cfg[11:0], 2'b00} + 16'd24;
    wire [15:0] data_octets_wire         = {data_octets_le[7:0], data_octets_le[15:8]};

    wire [15:0] df_octets_le             = cur_frag_ser_bytes + 16'd32;
    wire [15:0] df_octets_wire           = {df_octets_le[7:0], df_octets_le[15:8]};
    wire [31:0] df_frag_start_num        = {16'd0, active_frag_idx} + 32'd1;
    wire [31:0] df_frag_start_wire       = {df_frag_start_num[7:0], df_frag_start_num[15:8],
                                            df_frag_start_num[23:16], df_frag_start_num[31:24]};
    wire [15:0] df_frag_size_wire        = {eff_frag_bytes[7:0], eff_frag_bytes[15:8]};
    wire [31:0] df_sample_size_wire      = {sample_total_bytes[7:0], sample_total_bytes[15:8], 16'h0000};

    // Combinational UDP/RTPS Datagram Word Mux
    reg [31:0] cur_dgram_word;
    always @(*) begin
        if (use_data_frag) begin
            case (tx_dgram_word_idx)
                12'd0:  cur_dgram_word = {latched_dst_port, latched_src_port};
                12'd1:  cur_dgram_word = {cur_udp_len_bytes, udp_csum_final};
                12'd2:  cur_dgram_word = 32'h5254_5053;
                12'd3:  cur_dgram_word = 32'h0203_0101;
                12'd4:  cur_dgram_word = peer_guid_prefix[95:64];
                12'd5:  cur_dgram_word = peer_guid_prefix[63:32];
                12'd6:  cur_dgram_word = peer_guid_prefix[31:0];
                12'd7:  cur_dgram_word = 32'h0901_0800;
                12'd8:  cur_dgram_word = echo_ts_0;
                12'd9:  cur_dgram_word = echo_ts_1;
                12'd10: cur_dgram_word = {8'h16, 8'h01, df_octets_wire};
                12'd11: cur_dgram_word = 32'h0000_1C00; // extraFlags=0, octetsToInlineQos=28
                12'd12: cur_dgram_word = peer_reader_id;
                12'd13: cur_dgram_word = peer_writer_id;
                12'd14: cur_dgram_word = {peer_seq_num[39:32], peer_seq_num[47:40], peer_seq_num[55:48], peer_seq_num[63:56]};
                12'd15: cur_dgram_word = {peer_seq_num[7:0],   peer_seq_num[15:8],  peer_seq_num[23:16], peer_seq_num[31:24]};
                12'd16: cur_dgram_word = df_frag_start_wire;
                12'd17: cur_dgram_word = {16'h0100, df_frag_size_wire}; // fragmentsInSubmessage=1 (LE), fragmentSize (LE)
                12'd18: cur_dgram_word = df_sample_size_wire;
                12'd19: cur_dgram_word = (active_frag_idx == 16'd0) ? 32'h0001_0000 : bswap32(tx_sample_dout);
                default: cur_dgram_word = bswap32(tx_sample_dout);
            endcase
        end else begin
            case (tx_dgram_word_idx)
                12'd0:  cur_dgram_word = {latched_dst_port, latched_src_port};
                12'd1:  cur_dgram_word = {cur_udp_len_bytes, udp_csum_final};
                12'd2:  cur_dgram_word = 32'h5254_5053;
                12'd3:  cur_dgram_word = 32'h0203_0101;
                12'd4:  cur_dgram_word = peer_guid_prefix[95:64];
                12'd5:  cur_dgram_word = peer_guid_prefix[63:32];
                12'd6:  cur_dgram_word = peer_guid_prefix[31:0];
                12'd7:  cur_dgram_word = 32'h0901_0800;
                12'd8:  cur_dgram_word = echo_ts_0;
                12'd9:  cur_dgram_word = echo_ts_1;
                12'd10: cur_dgram_word = {8'h15, 8'h05, data_octets_wire};
                12'd11: cur_dgram_word = 32'h0000_1000;
                12'd12: cur_dgram_word = peer_reader_id;
                12'd13: cur_dgram_word = peer_writer_id;
                12'd14: cur_dgram_word = {peer_seq_num[39:32], peer_seq_num[47:40], peer_seq_num[55:48], peer_seq_num[63:56]};
                12'd15: cur_dgram_word = {peer_seq_num[7:0],   peer_seq_num[15:8],  peer_seq_num[23:16], peer_seq_num[31:24]};
                12'd16: cur_dgram_word = 32'h0001_0000;
                default: cur_dgram_word = bswap32(tx_sample_dout);
            endcase
        end
    end

    // Pipelined UDP Checksum Engine for TX (runs during E_TX_INIT phase 3)
    reg [23:0] emu_ucs_tree0, emu_ucs_tree1, emu_ucs_tree2;
    reg [15:0] emu_ucs_fold;

    wire [23:0] emu_ucs_stg0 = {8'd0, tx_frag_csum_seed} +
                               {8'd0, latched_dst_ip[31:16]} + {8'd0, latched_dst_ip[15:0]} +
                               {8'd0, latched_src_ip[31:16]} + {8'd0, latched_src_ip[15:0]} +
                               {8'd0, 16'h0011}              + {8'd0, cur_udp_len_bytes} +
                               {8'd0, latched_dst_port}      + {8'd0, latched_src_port};

    wire [23:0] emu_ucs_stg1 = emu_ucs_tree0 +
                               {8'd0, cur_udp_len_bytes} +
                               {8'd0, 16'h5254} + {8'd0, 16'h5053} +
                               {8'd0, 16'h0203} + {8'd0, 16'h0101} +
                               {8'd0, peer_guid_prefix[95:80]} + {8'd0, peer_guid_prefix[79:64]} +
                               {8'd0, peer_guid_prefix[63:48]} + {8'd0, peer_guid_prefix[47:32]} +
                               {8'd0, peer_guid_prefix[31:16]} + {8'd0, peer_guid_prefix[15:0]};

    wire [23:0] emu_ucs_submsg_sum = use_data_frag ?
        ({8'd0, 16'h1601} + {8'd0, df_octets_wire} + {8'd0, 16'h1C00} +
         {8'd0, df_frag_start_wire[31:16]} + {8'd0, df_frag_start_wire[15:0]} +
         {8'd0, 16'h0100} + {8'd0, df_frag_size_wire} +
         {8'd0, df_sample_size_wire[31:16]} + {8'd0, df_sample_size_wire[15:0]}) :
        ({8'd0, 16'h1505} + {8'd0, data_octets_wire} + {8'd0, 16'h1000} +
         {8'd0, 16'h0001});

    reg [23:0] emu_ucs_pre;
    wire [23:0] emu_ucs_pre_sum = {8'd0, 16'h0901} + {8'd0, 16'h0800} +
                                  {8'd0, echo_ts_0[31:16]} + {8'd0, echo_ts_0[15:0]} +
                                  {8'd0, echo_ts_1[31:16]} + {8'd0, echo_ts_1[15:0]} +
                                  {8'd0, peer_reader_id[31:16]} + {8'd0, peer_reader_id[15:0]} +
                                  {8'd0, peer_writer_id[31:16]} + {8'd0, peer_writer_id[15:0]} +
                                  {8'd0, peer_seq_num[39:32], peer_seq_num[47:40]} +
                                  {8'd0, peer_seq_num[55:48], peer_seq_num[63:56]} +
                                  {8'd0, peer_seq_num[7:0],   peer_seq_num[15:8]}  +
                                  {8'd0, peer_seq_num[23:16], peer_seq_num[31:24]} +
                                  emu_ucs_submsg_sum;

    wire [23:0] emu_ucs_stg2  = emu_ucs_tree1 + emu_ucs_pre;
    wire [19:0] emu_ucs_f1    = {4'd0, emu_ucs_tree2[15:0]} + {12'd0, emu_ucs_tree2[23:16]};
    wire [16:0] emu_ucs_f2    = {1'b0, emu_ucs_f1[15:0]} + {13'd0, emu_ucs_f1[19:16]};
    wire [15:0] emu_ucs_f3    = emu_ucs_f2[15:0] + {15'd0, emu_ucs_f2[16]};

    always @(posedge clk) begin
        if (!rst_n) begin
            emu_ucs_tree0 <= 24'd0;
            emu_ucs_tree1 <= 24'd0;
            emu_ucs_tree2 <= 24'd0;
            emu_ucs_pre   <= 24'd0;
            emu_ucs_fold  <= 16'd0;
        end else begin
            emu_ucs_tree0 <= emu_ucs_stg0;
            emu_ucs_tree1 <= emu_ucs_stg1;
            emu_ucs_pre   <= emu_ucs_pre_sum;
            emu_ucs_tree2 <= emu_ucs_stg2;
            emu_ucs_fold  <= emu_ucs_f3;
        end
    end

    // Pipelined IPv4 header checksum engine
    reg [19:0] emu_ipcs_stg0;
    reg [15:0] emu_ipcs_stg1;
    wire [16:0] emu_ipcs_fold1 = {1'b0, emu_ipcs_stg0[15:0]} + {13'd0, emu_ipcs_stg0[19:16]};
    wire [15:0] emu_ipcs_fold2 = emu_ipcs_fold1[15:0] + {15'd0, emu_ipcs_fold1[16]};
    always @(posedge clk) begin
        if (!rst_n) begin
            emu_ipcs_stg0 <= 20'd0;
            emu_ipcs_stg1 <= 16'd0;
        end else begin
            emu_ipcs_stg0 <= {4'd0, 16'h4500} + {4'd0, cur_frag_ip_len} +
                             {4'd0, peer_ip_id} + {4'd0, cur_frag_flags} +
                             {4'd0, 16'h4011} +
                             {4'd0, latched_dst_ip[31:16]} + {4'd0, latched_dst_ip[15:0]} +
                             {4'd0, latched_src_ip[31:16]} + {4'd0, latched_src_ip[15:0]};
            emu_ipcs_stg1 <= ~emu_ipcs_fold2;
        end
    end

    // Outgoing Ethernet + IPv4 + Fragment word mux
    reg [31:0] tx_wire_word;
    always @(*) begin
        case (tx_beat_idx)
            12'd0: tx_wire_word = 32'h0200_5E10;
            12'd1: tx_wire_word = 32'h0001_0200;
            12'd2: tx_wire_word = 32'h5E20_0002;
            12'd3: tx_wire_word = 32'h0800_4500;
            12'd4: tx_wire_word = {cur_frag_ip_len, peer_ip_id};
            12'd5: tx_wire_word = {cur_frag_flags, 16'h4011};
            12'd6: tx_wire_word = {cur_frag_hdr_csum, latched_dst_ip[31:16]};
            12'd7: tx_wire_word = {latched_dst_ip[15:0], latched_src_ip[31:16]};
            12'd8: tx_wire_word = {latched_src_ip[15:0], cur_dgram_word[31:16]};
            default: begin
                if (tx_beat_idx == (12'd8 + cur_frag_words))
                    tx_wire_word = {tx_prev_half, 16'h0000};
                else
                    tx_wire_word = {tx_prev_half, cur_dgram_word[31:16]};
            end
        endcase
    end

    // Helper: endian-swap 16/32
    function [15:0] bswap16;
        input [15:0] v;
        begin
            bswap16 = {v[7:0], v[15:8]};
        end
    endfunction

    function [31:0] bswap32;
        input [31:0] v;
        begin
            bswap32 = {v[7:0], v[15:8], v[23:16], v[31:24]};
        end
    endfunction

    always @(posedge clk) begin
        if (!rst_n) begin
            state                 <= E_RX_FRAMES;
            rx_beat_idx           <= 12'd0;
            latched_src_ip        <= 32'hC0A8_01C9;
            latched_dst_ip        <= 32'hC0A8_0164;
            latched_src_port      <= 16'd7410;
            latched_dst_port      <= 16'd7411;
            cur_ip_len            <= 16'd0;
            cur_ip_id             <= 16'd0;
            cur_flags_frag        <= 16'd0;
            cur_is_ip_frag        <= 1'b0;
            rx_accum_bytes        <= 16'd0;
            rx_expected_bytes     <= 16'd0;
            rx_saw_last_frag      <= 1'b0;
            rx_prev_half          <= 16'd0;
            rx_dgram_wr_ptr       <= 12'd0;
            rx_pkt_wr_ptr         <= 10'd0;

            pkt_substate          <= 3'd0;
            pkt_rd_addr           <= 10'd0;
            pkt_submsg_id         <= 8'd0;
            pkt_df_frag_start     <= 32'd0;
            pkt_df_frag_size      <= 16'd0;
            pkt_df_sample_size    <= 32'd0;
            pkt_df_seq_num        <= 64'd0;
            active_df_seq         <= 64'd0;
            active_df_valid       <= 1'b0;
            active_df_bitmap      <= 32'd0;
            active_df_total_frags <= 6'd0;
            df_rem_calc           <= 16'd0;
            df_tot_cnt            <= 6'd0;
            copy_src_idx          <= 12'd0;
            copy_dst_idx          <= 12'd0;
            copy_end_idx          <= 12'd0;

            peer_seq_num          <= 64'd0;
            peer_ip_id            <= 16'hA000;
            build_idx             <= 12'd0;
            build_prime           <= 1'b0;
            payload_words_cfg     <= 16'd1024;
            echo_ts_0             <= 32'd0;
            echo_ts_1             <= 32'd0;
            rx_ram_rd_addr        <= 12'd0;
            tx_sample_rd_addr     <= 12'd0;

            total_frags_tx        <= 16'd3;
            frag_step             <= 16'd0;
            active_frag_idx       <= 16'd0;
            tx_bytes_remaining    <= 16'd0;
            tx_frag_offset_bytes  <= 16'd0;
            cur_frag_ser_bytes    <= 16'd0;
            cur_frag_ser_words    <= 12'd0;
            cur_frag_bram_base    <= 12'd0;
            cur_frag_bram_words   <= 12'd0;
            cur_frag_words        <= 12'd0;
            cur_frag_ip_len       <= 16'd0;
            cur_frag_flags        <= 16'd0;
            cur_frag_hdr_csum     <= 16'd0;
            cur_udp_len_bytes     <= 16'd0;
            udp_csum_final        <= 16'd0;
            tx_init_phase         <= 2'd0;
            scan_cnt              <= 12'd0;
            scan_csum_accum       <= 16'd0;
            csum_pipe_cnt         <= 3'd0;
            tx_frag_csum_seed     <= 16'd0;

            tx_dgram_word_idx     <= 12'd0;
            tx_beat_idx           <= 12'd0;
            tx_prev_half          <= 16'd0;

            m_axis_tdata          <= 32'd0;
            m_axis_tvalid         <= 1'b0;
            m_axis_tlast          <= 1'b0;
        end else begin
            case (state)
                // -------------------------------------------------------------
                // 1. E_RX_FRAMES: Receive IPv4-Frag or Unfragmented RTPS Frame
                // -------------------------------------------------------------
                E_RX_FRAMES: begin
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;

                    if (rx_fire) begin
                        case (rx_beat_idx)
                            12'd0, 12'd1, 12'd2, 12'd3: begin
                                rx_beat_idx <= rx_beat_idx + 12'd1;
                            end
                            12'd4: begin
                                cur_ip_len  <= s_axis_tdata[31:16];
                                cur_ip_id   <= s_axis_tdata[15:0];
                                rx_beat_idx <= rx_beat_idx + 12'd1;
                            end
                            12'd5: begin
                                cur_flags_frag <= s_axis_tdata[31:16];
                                cur_is_ip_frag <= s_axis_tdata[29] || (s_axis_tdata[28:16] != 13'd0);
                                rx_beat_idx    <= rx_beat_idx + 12'd1;
                                if ((s_axis_tdata[29] || (s_axis_tdata[28:16] != 13'd0)) &&
                                    (s_axis_tdata[28:16] == 13'd0 && rx_accum_bytes == 16'd0)) begin
                                    rx_saw_last_frag  <= 1'b0;
                                    rx_expected_bytes <= 16'd0;
                                end
                            end
                            12'd6: begin
                                latched_src_ip[31:16] <= s_axis_tdata[15:0];
                                rx_beat_idx           <= rx_beat_idx + 12'd1;
                            end
                            12'd7: begin
                                latched_src_ip[15:0]  <= s_axis_tdata[31:16];
                                latched_dst_ip[31:16] <= s_axis_tdata[15:0];
                                rx_beat_idx           <= rx_beat_idx + 12'd1;
                            end
                            12'd8: begin
                                latched_dst_ip[15:0] <= s_axis_tdata[31:16];
                                rx_prev_half         <= s_axis_tdata[15:0];
                                rx_dgram_wr_ptr      <= frag_word_start;
                                rx_pkt_wr_ptr        <= 10'd0;
                                if (frag_offset_8b == 13'd0) begin
                                    latched_src_port <= s_axis_tdata[15:0];
                                end
                                rx_beat_idx          <= rx_beat_idx + 12'd1;
                            end
                            default: begin
                                if (cur_is_ip_frag) begin
                                    if (rx_dgram_wr_ptr < (frag_word_start + ip_payload_len[13:2])) begin
                                        rx_dgram_ram[rx_dgram_wr_ptr] <= {rx_prev_half, s_axis_tdata[31:16]};
                                        rx_dgram_wr_ptr               <= rx_dgram_wr_ptr + 12'd1;
                                        if (frag_offset_8b == 13'd0 && rx_dgram_wr_ptr == 12'd0)
                                            latched_dst_port <= s_axis_tdata[31:16];
                                        if (frag_offset_8b == 13'd0 && rx_dgram_wr_ptr == 12'd1)
                                            payload_words_cfg <= {4'd0, rx_prev_half[13:2]} - 16'd17;
                                    end
                                end else begin
                                    if (rx_pkt_wr_ptr < {1'b0, ip_payload_len[10:2]} && rx_pkt_wr_ptr < 10'd511) begin
                                        rx_pkt_ram[rx_pkt_wr_ptr] <= {rx_prev_half, s_axis_tdata[31:16]};
                                        rx_pkt_wr_ptr             <= rx_pkt_wr_ptr + 10'd1;
                                        if (rx_pkt_wr_ptr == 10'd0)
                                            latched_dst_port <= s_axis_tdata[31:16];
                                    end
                                end
                                rx_prev_half <= s_axis_tdata[15:0];

                                if (s_axis_tlast) begin
                                    rx_beat_idx <= 12'd0;
                                    if (!cur_is_ip_frag) begin
                                        pkt_substate <= 3'd0;
                                        pkt_rd_addr  <= 10'd8; // Prime INFO_TS sec
                                        state        <= E_RX_PKT_STAGE;
                                    end else if (!frag_mf) begin
                                        if ((rx_accum_bytes + ip_payload_len) == ({frag_offset_8b, 3'b000} + ip_payload_len)) begin
                                            rx_accum_bytes <= 16'd0;
                                            peer_seq_num   <= fault_ctrl[3] ? ((peer_seq_num == 64'd0) ? 64'd1 : peer_seq_num)
                                                                            : (peer_seq_num + 64'd1);
                                            peer_ip_id     <= peer_ip_id + 16'd1;
                                            build_idx      <= 12'd0;
                                            build_prime    <= 1'b1;
                                            rx_ram_rd_addr <= 12'd8; // Echo INFO_TS sec
                                            state          <= E_BUILD_DGRAM;
                                        end else begin
                                            rx_accum_bytes    <= rx_accum_bytes + ip_payload_len;
                                            rx_expected_bytes <= {frag_offset_8b, 3'b000} + ip_payload_len;
                                            rx_saw_last_frag  <= 1'b1;
                                        end
                                    end else begin
                                        if (rx_saw_last_frag && ((rx_accum_bytes + ip_payload_len) == rx_expected_bytes)) begin
                                            rx_accum_bytes   <= 16'd0;
                                            rx_saw_last_frag <= 1'b0;
                                            peer_seq_num     <= fault_ctrl[3] ? ((peer_seq_num == 64'd0) ? 64'd1 : peer_seq_num)
                                                                              : (peer_seq_num + 64'd1);
                                            peer_ip_id       <= peer_ip_id + 16'd1;
                                            build_idx        <= 12'd0;
                                            build_prime      <= 1'b1;
                                            rx_ram_rd_addr   <= 12'd8;
                                            state            <= E_BUILD_DGRAM;
                                        end else begin
                                            rx_accum_bytes <= rx_accum_bytes + ip_payload_len;
                                        end
                                    end
                                end else begin
                                    rx_beat_idx <= rx_beat_idx + 12'd1;
                                end
                            end
                        endcase
                    end
                end

                // -------------------------------------------------------------
                // 2. E_RX_PKT_STAGE: Unpack DATA (0x15) or DATA_FRAG (0x16)
                // -------------------------------------------------------------
                E_RX_PKT_STAGE: begin
                    case (pkt_substate)
                        3'd0: begin
                            // Wait 1 cycle for Word 8; prime Word 9
                            pkt_rd_addr  <= 10'd9;
                            pkt_substate <= 3'd1;
                        end
                        3'd1: begin
                            // pkt_ram_dout = Word 8 (INFO_TS sec); prime Word 10 (SubmsgId)
                            echo_ts_0    <= pkt_ram_dout;
                            pkt_rd_addr  <= 10'd10;
                            pkt_substate <= 3'd2;
                        end
                        3'd2: begin
                            // pkt_ram_dout = Word 9 (INFO_TS frac); prime Word 14 (SeqNum Hi)
                            echo_ts_1    <= pkt_ram_dout;
                            pkt_rd_addr  <= 10'd14;
                            pkt_substate <= 3'd3;
                        end
                        3'd3: begin
                            // pkt_ram_dout = Word 10 (SubmsgId)
                            pkt_submsg_id <= pkt_ram_dout[31:24];
                            if (pkt_ram_dout[31:24] == 8'h15) begin
                                // Single DATA packet: copy Words 17 .. ip_payload_len[13:2]-1
                                payload_words_cfg <= {4'd0, ip_payload_len[13:2]} - 16'd17;
                                copy_src_idx      <= 12'd17;
                                copy_dst_idx      <= 12'd17;
                                copy_end_idx      <= ip_payload_len[13:2];
                                pkt_rd_addr       <= 10'd17;
                                pkt_substate      <= 3'd7; // Single DATA copy wait
                            end else if (pkt_ram_dout[31:24] == 8'h16) begin
                                // DATA_FRAG: prime Word 15 (SeqNum Lo)
                                pkt_rd_addr  <= 10'd15;
                                pkt_substate <= 3'd4;
                            end else begin
                                state <= E_RX_FRAMES;
                            end
                        end
                        3'd4: begin
                            // pkt_ram_dout = Word 14 (SeqNum Hi); prime Word 16 (fragmentStartingNum)
                            pkt_df_seq_num[63:32] <= bswap32(pkt_ram_dout);
                            pkt_rd_addr           <= 10'd16;
                            pkt_substate          <= 3'd5;
                        end
                        3'd5: begin
                            // pkt_ram_dout = Word 15 (SeqNum Lo); prime Word 17 ({fragsInSubmsg, fragSize})
                            pkt_df_seq_num[31:0] <= bswap32(pkt_ram_dout);
                            pkt_rd_addr          <= 10'd17;
                            pkt_substate         <= 3'd6;
                        end
                        3'd6: begin
                            // pkt_ram_dout = Word 16 (fragmentStartingNum LE); prime Word 18 (sampleSize)
                            pkt_df_frag_start <= bswap32(pkt_ram_dout);
                            pkt_rd_addr       <= 10'd18;
                            pkt_substate      <= 3'd7;
                        end
                        default: begin
                            if (pkt_submsg_id == 8'h15) begin
                                // Single DATA copy loop
                                if (pkt_rd_addr == 10'd17 && copy_src_idx == 12'd17 && copy_dst_idx == 12'd17) begin
                                    // Prime +1 for 1-cycle BRAM latency
                                    pkt_rd_addr <= 10'd18;
                                end else if (copy_src_idx < copy_end_idx) begin
                                    rx_dgram_ram[copy_dst_idx] <= pkt_ram_dout;
                                    copy_src_idx               <= copy_src_idx + 12'd1;
                                    copy_dst_idx               <= copy_dst_idx + 12'd1;
                                    pkt_rd_addr                <= pkt_rd_addr + 10'd1;
                                end else begin
                                    // Done copying single DATA payload!
                                    rx_dgram_ram[8] <= echo_ts_0;
                                    rx_dgram_ram[9] <= echo_ts_1;
                                    peer_seq_num    <= fault_ctrl[3] ? ((peer_seq_num == 64'd0) ? 64'd1 : peer_seq_num)
                                                                     : (peer_seq_num + 64'd1);
                                    peer_ip_id      <= peer_ip_id + 16'd1;
                                    build_idx       <= 12'd0;
                                    build_prime     <= 1'b1;
                                    rx_ram_rd_addr  <= 12'd8;
                                    state           <= E_BUILD_DGRAM;
                                end
                            end else begin
                                // DATA_FRAG multi-cycle handling using extra helper states via copy_src_idx
                                if (pkt_rd_addr == 10'd18) begin
                                    // pkt_ram_dout = Word 17 ({fragsInSubmsg, fragSize})
                                    pkt_df_frag_size <= bswap16(pkt_ram_dout[15:0]);
                                    pkt_rd_addr      <= 10'd19; // Next cycle gives Word 18 (sampleSize)
                                end else if (pkt_rd_addr == 10'd19 && copy_src_idx == 12'd0) begin
                                    // pkt_ram_dout = Word 18 (sampleSize LE)
                                    pkt_df_sample_size <= bswap32(pkt_ram_dout);
                                    df_rem_calc        <= bswap32(pkt_ram_dout[31:0]);
                                    df_tot_cnt         <= 6'd0;
                                    copy_src_idx       <= 12'd1; // Signal calc phase
                                    if (!active_df_valid || active_df_seq != pkt_df_seq_num) begin
                                        active_df_valid  <= 1'b1;
                                        active_df_seq    <= pkt_df_seq_num;
                                        active_df_bitmap <= 32'd0;
                                    end
                                end else if (copy_src_idx == 12'd1) begin
                                    // Iterative total_frags = ceil(sampleSize / fragSize)
                                    if (df_rem_calc > pkt_df_frag_size && df_tot_cnt < 6'd32) begin
                                        df_rem_calc <= df_rem_calc - pkt_df_frag_size;
                                        df_tot_cnt  <= df_tot_cnt + 6'd1;
                                    end else begin
                                        active_df_total_frags <= df_tot_cnt + 6'd1;
                                        copy_src_idx          <= 12'd2;
                                        copy_dst_idx          <= 12'd16 + ((pkt_df_frag_start[4:0] - 5'd1) * pkt_df_frag_size[13:2]);
                                        copy_end_idx          <= ip_payload_len[13:2]; // Total UDP words in this packet
                                        pkt_rd_addr           <= 10'd19;
                                    end
                                end else if (copy_src_idx == 12'd2) begin
                                    // 1-cycle BRAM prime for Word 19
                                    copy_src_idx <= 12'd19;
                                    pkt_rd_addr  <= 10'd20;
                                end else if (copy_src_idx < copy_end_idx) begin
                                    rx_dgram_ram[copy_dst_idx] <= pkt_ram_dout;
                                    copy_src_idx               <= copy_src_idx + 12'd1;
                                    copy_dst_idx               <= copy_dst_idx + 12'd1;
                                    pkt_rd_addr                <= pkt_rd_addr + 10'd1;
                                end else begin
                                    // Mark fragment bit & check completion
                                    copy_src_idx <= 12'd0;
                                    if ((active_df_bitmap | (32'd1 << (pkt_df_frag_start[4:0] - 5'd1))) ==
                                        ((active_df_total_frags == 6'd32) ? 32'hFFFF_FFFF : ((32'd1 << active_df_total_frags) - 32'd1))) begin
                                        active_df_valid   <= 1'b0;
                                        active_df_bitmap  <= 32'd0;
                                        payload_words_cfg <= pkt_df_sample_size[13:2] - 16'd1;
                                        rx_dgram_ram[8]   <= echo_ts_0;
                                        rx_dgram_ram[9]   <= echo_ts_1;
                                        peer_seq_num      <= fault_ctrl[3] ? ((peer_seq_num == 64'd0) ? 64'd1 : peer_seq_num)
                                                                           : (peer_seq_num + 64'd1);
                                        peer_ip_id        <= peer_ip_id + 16'd1;
                                        build_idx         <= 12'd0;
                                        build_prime       <= 1'b1;
                                        rx_ram_rd_addr    <= 12'd8;
                                        state             <= E_BUILD_DGRAM;
                                    end else begin
                                        active_df_bitmap <= active_df_bitmap | (32'd1 << (pkt_df_frag_start[4:0] - 5'd1));
                                        state            <= E_RX_FRAMES;
                                    end
                                end
                            end
                        end
                    endcase
                end

                // -------------------------------------------------------------
                // 3. E_BUILD_DGRAM: Transform FastSample into tx_sample_ram
                // -------------------------------------------------------------
                E_BUILD_DGRAM: begin
                    if (build_prime) begin
                        rx_ram_rd_addr <= 12'd9;
                        build_prime    <= 1'b0;
                        build_idx      <= 12'd0;
                    end else if (build_idx == 12'd0 && rx_ram_rd_addr == 12'd9) begin
                        echo_ts_0      <= rx_ram_dout;
                        rx_ram_rd_addr <= 12'd17;
                    end else if (build_idx == 12'd0 && rx_ram_rd_addr == 12'd17) begin
                        echo_ts_1      <= rx_ram_dout;
                        rx_ram_rd_addr <= 12'd18;
                        build_idx      <= 12'd1; // Signal payload loop
                    end else begin
                        // Use copy_dst_idx as tx_sample_ram write pointer (starting at 0)
                        if (build_idx == 12'd1) begin
                            tx_sample_ram[0] <= bswap32(rx_ram_dout); // Word 0: host-LE cycle_id intact
                            copy_dst_idx     <= 12'd1;
                            rx_ram_rd_addr   <= 12'd19;
                            build_idx        <= 12'd2;
                        end else if (copy_dst_idx < payload_words_cfg[11:0]) begin
                            if (copy_dst_idx < 12'd4)
                                tx_sample_ram[copy_dst_idx] <= bswap32(rx_ram_dout);
                            else
                                tx_sample_ram[copy_dst_idx] <= (~bswap32(rx_ram_dout)) + {16'h0000, peer_seq_num[5:0], 10'h200};
                            copy_dst_idx   <= copy_dst_idx + 12'd1;
                            rx_ram_rd_addr <= rx_ram_rd_addr + 12'd1;
                        end else begin
                            copy_dst_idx         <= 12'd0;
                            frag_step            <= 16'd0;
                            tx_bytes_remaining   <= tx_ip_frag_mode ? ipfrag_udp_total_bytes : sample_total_bytes;
                            tx_frag_offset_bytes <= 16'd0;
                            tx_init_phase        <= 2'd0;
                            // Compute total_frags_tx for DATA_FRAG (or 3 for IP-frag, 1 for single DATA)
                            if (tx_ip_frag_mode) begin
                                total_frags_tx <= 16'd3;
                            end else if (use_single_data) begin
                                total_frags_tx <= 16'd1;
                            end else begin
                                total_frags_tx <= (sample_total_bytes + eff_frag_bytes - 16'd1) / eff_frag_bytes;
                            end
                            state <= E_TX_INIT;
                        end
                    end
                end

                // -------------------------------------------------------------
                // 4. E_TX_INIT: Plan Fragment & Compute UDP Checksum
                // -------------------------------------------------------------
                E_TX_INIT: begin
                    if (m_axis_tvalid && m_axis_tready) begin
                        m_axis_tvalid <= 1'b0;
                        m_axis_tlast  <= 1'b0;
                    end

                    if (!(m_axis_tvalid && !m_axis_tready)) begin
                    // Fault [0]: Drop Fragment #1 when multiple fragments exist
                    if (fault_ctrl[0] && (frag_step == 16'd1) && (total_frags_tx > 16'd1)) begin
                        frag_step <= 16'd2;
                    end else begin
                        case (tx_init_phase)
                            2'd0: begin
                                // Determine active_frag_idx (Fault [1]: swap #0 and #1)
                                if (fault_ctrl[1] && (total_frags_tx >= 16'd2)) begin
                                    active_frag_idx <= (frag_step == 16'd0) ? 16'd1 :
                                                       (frag_step == 16'd1) ? 16'd0 : frag_step;
                                end else begin
                                    active_frag_idx <= frag_step;
                                end

                                if (tx_ip_frag_mode) begin
                                    // 3 IPv4 fragments: 0 (368W @ 0), 1 (368W @ 184), 2 (rem @ 368)
                                    case (fault_ctrl[1] ? ((frag_step == 16'd0) ? 16'd1 : (frag_step == 16'd1) ? 16'd0 : frag_step) : frag_step)
                                        16'd0: begin
                                            cur_frag_words     <= 12'd368;
                                            cur_frag_ip_len    <= 16'd1492;
                                            cur_frag_flags     <= {3'b001, 13'd0};
                                            cur_frag_bram_base <= 12'd0; // Word 0..367
                                        end
                                        16'd1: begin
                                            cur_frag_words     <= 12'd368;
                                            cur_frag_ip_len    <= 16'd1492;
                                            cur_frag_flags     <= {3'b001, 13'd184};
                                            cur_frag_bram_base <= 12'd351; // 368 - 17
                                        end
                                        default: begin
                                            cur_frag_words     <= ipfrag_total_dgram_words[11:0] - 12'd736;
                                            cur_frag_ip_len    <= 16'd20 + {2'b00, (ipfrag_total_dgram_words[11:0] - 12'd736), 2'b00};
                                            cur_frag_flags     <= {3'b000, 13'd368};
                                            cur_frag_bram_base <= 12'd719; // 736 - 17
                                        end
                                    endcase
                                    cur_udp_len_bytes   <= ipfrag_udp_total_bytes;
                                    cur_frag_bram_words <= payload_words_cfg[11:0]; // Full sample checksum on frag_step==0
                                    if (frag_step == 16'd0) begin
                                        scan_csum_accum   <= 16'd0;
                                        scan_cnt          <= 12'd0;
                                        tx_sample_rd_addr <= 12'd0;
                                        tx_init_phase     <= 2'd1;
                                    end else begin
                                        // Already computed udp_csum_final on frag_step == 0
                                        tx_sample_rd_addr <= (fault_ctrl[1] && frag_step == 16'd1) ? 12'd0 :
                                                             (frag_step == 16'd1) ? 12'd351 : 12'd719;
                                        tx_dgram_word_idx <= (fault_ctrl[1] && frag_step == 16'd1) ? 12'd0 :
                                                             (frag_step == 16'd1) ? 12'd368 : 12'd736;
                                        tx_beat_idx       <= 12'd0;
                                        m_axis_tvalid     <= 1'b0;
                                        m_axis_tlast      <= 1'b0;
                                        state             <= E_TX_STREAM;
                                    end
                                end else if (use_single_data) begin
                                    cur_frag_ser_bytes  <= sample_total_bytes;
                                    cur_frag_ser_words  <= payload_words_cfg[11:0] + 12'd1;
                                    cur_udp_len_bytes   <= ipfrag_udp_total_bytes;
                                    cur_frag_words      <= ipfrag_total_dgram_words[11:0];
                                    cur_frag_ip_len     <= 16'd20 + ipfrag_udp_total_bytes;
                                    cur_frag_flags      <= 16'd0;
                                    cur_frag_bram_base  <= 12'd0;
                                    cur_frag_bram_words <= payload_words_cfg[11:0];
                                    scan_csum_accum     <= 16'd0;
                                    scan_cnt            <= 12'd0;
                                    tx_sample_rd_addr   <= 12'd0;
                                    tx_init_phase       <= 2'd1;
                                end else begin
                                    // DATA_FRAG (0x16)
                                    // Compute active fragment's byte offset & size (handles Fault [1] swap cleanly!)
                                    if (fault_ctrl[1] && (total_frags_tx >= 16'd2) && (frag_step == 16'd0)) begin
                                        // Emitting Frag #1 first
                                        cur_frag_ser_bytes  <= ((sample_total_bytes - eff_frag_bytes) > eff_frag_bytes) ?
                                                               eff_frag_bytes : (sample_total_bytes - eff_frag_bytes);
                                        cur_frag_bram_base  <= eff_frag_bytes[13:2] - 12'd1;
                                        cur_frag_bram_words <= (((sample_total_bytes - eff_frag_bytes) > eff_frag_bytes) ?
                                                                eff_frag_bytes[13:2] : ((sample_total_bytes - eff_frag_bytes) >> 2));
                                        cur_frag_ser_words  <= (((sample_total_bytes - eff_frag_bytes) > eff_frag_bytes) ?
                                                                eff_frag_bytes[13:2] : ((sample_total_bytes - eff_frag_bytes) >> 2));
                                        cur_udp_len_bytes   <= 16'd76 + (((sample_total_bytes - eff_frag_bytes) > eff_frag_bytes) ?
                                                               eff_frag_bytes : (sample_total_bytes - eff_frag_bytes));
                                        cur_frag_words      <= 12'd19 + (((sample_total_bytes - eff_frag_bytes) > eff_frag_bytes) ?
                                                               eff_frag_bytes[13:2] : ((sample_total_bytes - eff_frag_bytes) >> 2));
                                        cur_frag_ip_len     <= 16'd96 + (((sample_total_bytes - eff_frag_bytes) > eff_frag_bytes) ?
                                                               eff_frag_bytes : (sample_total_bytes - eff_frag_bytes));
                                        scan_csum_accum     <= 16'd0;
                                        tx_sample_rd_addr   <= eff_frag_bytes[13:2] - 12'd1;
                                    end else if (fault_ctrl[1] && (total_frags_tx >= 16'd2) && (frag_step == 16'd1)) begin
                                        // Emitting Frag #0 second
                                        cur_frag_ser_bytes  <= eff_frag_bytes;
                                        cur_frag_ser_words  <= eff_frag_bytes[13:2];
                                        cur_frag_bram_base  <= 12'd0;
                                        cur_frag_bram_words <= eff_frag_bytes[13:2] - 12'd1;
                                        cur_udp_len_bytes   <= 16'd76 + eff_frag_bytes;
                                        cur_frag_words      <= 12'd19 + eff_frag_bytes[13:2];
                                        cur_frag_ip_len     <= 16'd96 + eff_frag_bytes;
                                        scan_csum_accum     <= 16'h0001; // Include CDR_LE
                                        tx_sample_rd_addr   <= 12'd0;
                                    end else begin
                                        // Normal in-order fragment (or frag_step >= 2)
                                        if (tx_bytes_remaining > eff_frag_bytes) begin
                                            cur_frag_ser_bytes  <= eff_frag_bytes;
                                            cur_frag_ser_words  <= eff_frag_bytes[13:2];
                                            cur_udp_len_bytes   <= 16'd76 + eff_frag_bytes;
                                            cur_frag_words      <= 12'd19 + eff_frag_bytes[13:2];
                                            cur_frag_ip_len     <= 16'd96 + eff_frag_bytes;
                                            cur_frag_bram_words <= (frag_step == 16'd0) ? (eff_frag_bytes[13:2] - 12'd1) : eff_frag_bytes[13:2];
                                        end else begin
                                            cur_frag_ser_bytes  <= tx_bytes_remaining;
                                            cur_frag_ser_words  <= tx_bytes_remaining[13:2];
                                            cur_udp_len_bytes   <= 16'd76 + tx_bytes_remaining;
                                            cur_frag_words      <= 12'd19 + tx_bytes_remaining[13:2];
                                            cur_frag_ip_len     <= 16'd96 + tx_bytes_remaining;
                                            cur_frag_bram_words <= (frag_step == 16'd0) ? (tx_bytes_remaining[13:2] - 12'd1) : tx_bytes_remaining[13:2];
                                        end
                                        cur_frag_bram_base <= (frag_step == 16'd0) ? 12'd0 : (tx_frag_offset_bytes[13:2] - 12'd1);
                                        scan_csum_accum    <= (frag_step == 16'd0) ? 16'h0001 : 16'h0000;
                                        tx_sample_rd_addr  <= (frag_step == 16'd0) ? 12'd0 : (tx_frag_offset_bytes[13:2] - 12'd1);
                                    end
                                    cur_frag_flags <= 16'd0;
                                    scan_cnt       <= 12'd0;
                                    tx_init_phase  <= 2'd1;
                                end
                            end
                            2'd1: begin
                                // 1-cycle BRAM read wait
                                if (cur_frag_bram_words == 12'd0) begin
                                    tx_frag_csum_seed <= scan_csum_accum;
                                    csum_pipe_cnt     <= 3'd0;
                                    tx_init_phase     <= 2'd3;
                                end else begin
                                    tx_sample_rd_addr <= tx_sample_rd_addr + 12'd1;
                                    tx_init_phase     <= 2'd2;
                                end
                            end
                            2'd2: begin
                                scan_csum_accum <= csum_add32(scan_csum_accum, bswap32(tx_sample_dout));
                                if (scan_cnt == (cur_frag_bram_words - 12'd1)) begin
                                    tx_init_phase <= 2'd3;
                                    csum_pipe_cnt <= 3'd0;
                                end else begin
                                    scan_cnt          <= scan_cnt + 12'd1;
                                    tx_sample_rd_addr <= tx_sample_rd_addr + 12'd1;
                                end
                            end
                            2'd3: begin
                                // Latch scan_csum_accum into tx_frag_csum_seed on cycle 0, wait 4 cycles for emu_ucs_fold
                                if (csum_pipe_cnt == 3'd0) begin
                                    tx_frag_csum_seed <= scan_csum_accum;
                                end
                                if (csum_pipe_cnt < 3'd5) begin
                                    csum_pipe_cnt <= csum_pipe_cnt + 3'd1;
                                end else begin
                                    udp_csum_final <= fault_ctrl[2] ? (~emu_ucs_fold ^ 16'hDEAD) :
                                        ((~emu_ucs_fold == 16'h0000) ? 16'hFFFF : ~emu_ucs_fold);
                                    tx_sample_rd_addr <= cur_frag_bram_base;
                                    tx_dgram_word_idx <= (tx_ip_frag_mode && active_frag_idx == 16'd1) ? 12'd368 :
                                                         (tx_ip_frag_mode && active_frag_idx == 16'd2) ? 12'd736 : 12'd0;
                                    tx_beat_idx       <= 12'd0;
                                    m_axis_tvalid     <= 1'b0;
                                    m_axis_tlast      <= 1'b0;
                                    tx_init_phase     <= 2'd0;
                                    state             <= E_TX_STREAM;
                                end
                            end
                        endcase
                    end
                    end
                end

                // -------------------------------------------------------------
                // 5. E_TX_STREAM: Stream Ethernet + IPv4 + Fragment over m_axis
                // -------------------------------------------------------------
                E_TX_STREAM: begin
                    if (tx_beat_idx == 12'd3) begin
                        cur_frag_hdr_csum <= emu_ipcs_stg1;
                    end

                    if (!m_axis_tvalid || m_axis_tready) begin
                        m_axis_tdata  <= tx_wire_word;
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= (tx_beat_idx == (12'd8 + cur_frag_words));

                        if (tx_beat_idx == (12'd8 + cur_frag_words)) begin
                            if (frag_step >= (total_frags_tx - 16'd1)) begin
                                state <= E_RX_FRAMES;
                            end else begin
                                frag_step <= frag_step + 16'd1;
                                if (!tx_ip_frag_mode) begin
                                    if (fault_ctrl[1] && frag_step == 16'd0 && total_frags_tx >= 16'd2) begin
                                        // Swapped #1 first, #0 next; after #0 completes, step 2 skips past both #0 and #1!
                                        tx_bytes_remaining   <= sample_total_bytes - eff_frag_bytes;
                                        tx_frag_offset_bytes <= eff_frag_bytes;
                                    end else if (fault_ctrl[1] && frag_step == 16'd1 && total_frags_tx >= 16'd2) begin
                                        tx_bytes_remaining   <= sample_total_bytes - {eff_frag_bytes[14:0], 1'b0};
                                        tx_frag_offset_bytes <= {eff_frag_bytes[14:0], 1'b0};
                                    end else begin
                                        tx_bytes_remaining   <= tx_bytes_remaining - cur_frag_ser_bytes;
                                        tx_frag_offset_bytes <= tx_frag_offset_bytes + cur_frag_ser_bytes;
                                    end
                                end
                                tx_init_phase <= 2'd0;
                                state         <= E_TX_INIT;
                            end
                        end else begin
                            tx_beat_idx <= tx_beat_idx + 12'd1;
                            if (tx_beat_idx == 12'd7 && tx_dgram_word_idx >= (use_data_frag ? 12'd19 : 12'd17)) begin
                                tx_sample_rd_addr <= tx_sample_rd_addr + 12'd1;
                            end else if (tx_beat_idx >= 12'd8) begin
                                tx_prev_half      <= cur_dgram_word[15:0];
                                tx_dgram_word_idx <= tx_dgram_word_idx + 12'd1;
                                if (use_data_frag) begin
                                    if ((active_frag_idx == 16'd0 && tx_dgram_word_idx >= 12'd19) ||
                                        (active_frag_idx != 16'd0 && tx_dgram_word_idx >= 12'd18)) begin
                                        tx_sample_rd_addr <= tx_sample_rd_addr + 12'd1;
                                    end
                                end else begin
                                    if (tx_dgram_word_idx >= 12'd16) begin
                                        tx_sample_rd_addr <= tx_sample_rd_addr + 12'd1;
                                    end
                                end
                            end
                        end
                    end
                end

                default: state <= E_RX_FRAMES;
            endcase
        end
    end

endmodule
