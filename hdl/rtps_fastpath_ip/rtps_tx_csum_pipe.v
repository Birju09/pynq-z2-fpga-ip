`timescale 1ns / 1ps

// =============================================================================
// Module: rtps_tx_csum_pipe
//
// Description:
//   High-performance pipelined UDP & IPv4 header checksum engines for TX.
//   Pipeline registers omit synchronous resets per UG949 timing closure
//   guidelines since they overwrite unconditionally every cycle and self-flush.
// =============================================================================

module rtps_tx_csum_pipe (
    input  wire        clk,

    // Mode & Length Inputs
    input  wire        tx_use_data_frag,
    input  wire        tx_ip_frag_mode,
    input  wire [15:0] tx_frag_payload_bytes,
    input  wire [15:0] tx_udp_total_bytes,

    // Payload Checksum Seeds
    input  wire [15:0] payload_csum_accum,
    input  wire [15:0] tx_frag_csum_seed,

    // Network & Endpoint Header Fields
    input  wire [31:0] local_src_ip,
    input  wire [31:0] wr0_dst_ip,
    input  wire [15:0] wr0_src_port,
    input  wire [15:0] wr0_dst_port,
    input  wire [15:0] local_vendor_id,
    input  wire [95:0] local_guid_prefix,

    // RTPS Submessage Header Fields
    input  wire [15:0] tx_df_octets_wire,
    input  wire [31:0] tx_df_start_wire,
    input  wire [15:0] tx_df_frag_size_wire,
    input  wire [31:0] tx_df_sample_size_wire,
    input  wire [15:0] tx_data_octets_wire,
    input  wire [63:0] latched_tx_ts,
    input  wire [31:0] wr0_reader_id,
    input  wire [31:0] wr0_writer_id,
    input  wire [63:0] cur_tx_seq,

    // IPv4 Header Fields
    input  wire [15:0] tx_frag_ip_len,
    input  wire [15:0] tx_ip_id_counter,
    input  wire [15:0] tx_frag_flags,

    // Final Folded Checksums
    output wire [15:0] calc_tx_udp_csum,
    output wire [15:0] calc_tx_ip_hdr_csum
);

    reg [23:0] ucs_tree_0, ucs_tree_1, ucs_tree_2;
    reg [15:0] ucs_final_fold;

    reg  [15:0] ucs_udp_len_r;
    wire [15:0] ucs_udp_len   = tx_use_data_frag ? tx_frag_payload_bytes : tx_udp_total_bytes;
    always @(posedge clk) ucs_udp_len_r <= ucs_udp_len;
    wire [15:0] ucs_seed_csum = tx_ip_frag_mode  ? payload_csum_accum    : tx_frag_csum_seed;

    // Stage 1: Sum payload checksum, IPs, UDP len/proto, ports
    wire [23:0] ucs_stg0_sum = {8'd0, ucs_seed_csum} +
                               {8'd0, local_src_ip[31:16]} + {8'd0, local_src_ip[15:0]} +
                               {8'd0, wr0_dst_ip[31:16]}   + {8'd0, wr0_dst_ip[15:0]} +
                               {8'd0, 16'h0011}            + {8'd0, ucs_udp_len_r} +
                               {8'd0, wr0_src_port}        + {8'd0, wr0_dst_port};

    // Stage 2: Sum UDP len + RTPS magic, version/vendor, guidPrefix
    wire [23:0] ucs_stg1_sum = ucs_tree_0 +
                               {8'd0, ucs_udp_len_r} + 24'd0 +
                               {8'd0, 16'h5254} + {8'd0, 16'h5053} +
                               {8'd0, 16'h0201} + {8'd0, local_vendor_id} +
                               {8'd0, local_guid_prefix[95:80]} + {8'd0, local_guid_prefix[79:64]} +
                               {8'd0, local_guid_prefix[63:48]} + {8'd0, local_guid_prefix[47:32]} +
                               {8'd0, local_guid_prefix[31:16]} + {8'd0, local_guid_prefix[15:0]};

    reg  [23:0] ucs_submsg_r;
    wire [23:0] ucs_submsg_sum = tx_use_data_frag ?
        ({8'd0, 8'h16, 8'h01} + {8'd0, tx_df_octets_wire} + {8'd0, 16'h1C00} +
         {8'd0, tx_df_start_wire[31:16]} + {8'd0, tx_df_start_wire[15:0]} +
         {8'd0, 16'h0100} + {8'd0, tx_df_frag_size_wire} +
         {8'd0, tx_df_sample_size_wire[31:16]} + {8'd0, tx_df_sample_size_wire[15:0]}) :
        ({8'd0, 8'h15, 8'h05} + {8'd0, tx_data_octets_wire} + {8'd0, 16'h1000} +
         {8'd0, 16'h0001});

    // Stage 3a: terms that do not depend on tree_1, summed in parallel with stage 2
    reg [23:0] ucs_pre;
    wire [23:0] ucs_pre_sum = {8'd0, 16'h0901} + {8'd0, 16'h0800} +
                              {8'd0, latched_tx_ts[39:32], latched_tx_ts[47:40]} +
                              {8'd0, latched_tx_ts[55:48], latched_tx_ts[63:56]} +
                              {8'd0, latched_tx_ts[7:0],   latched_tx_ts[15:8]}  +
                              {8'd0, latched_tx_ts[23:16], latched_tx_ts[31:24]} +
                              {8'd0, wr0_reader_id[31:16]} + {8'd0, wr0_reader_id[15:0]} +
                              {8'd0, wr0_writer_id[31:16]} + {8'd0, wr0_writer_id[15:0]} +
                              {8'd0, cur_tx_seq[39:32], cur_tx_seq[47:40]} +
                              {8'd0, cur_tx_seq[55:48], cur_tx_seq[63:56]} +
                              {8'd0, cur_tx_seq[7:0],   cur_tx_seq[15:8]}  +
                              {8'd0, cur_tx_seq[23:16], cur_tx_seq[31:24]} +
                              ucs_submsg_r;
    always @(posedge clk) begin
        ucs_submsg_r <= ucs_submsg_sum;
        ucs_pre      <= ucs_pre_sum;
    end

    // Stage 3: combine tree_1 with the pre-summed terms
    wire [23:0] ucs_stg2_sum = ucs_tree_1 + ucs_pre;

    // Final Stage 4: 1's complement carry-fold of the 24-bit binary sum into 16 bits
    wire [19:0] ucs_fold_tmp1 = {4'd0, ucs_tree_2[15:0]} + {12'd0, ucs_tree_2[23:16]};
    wire [16:0] ucs_fold_tmp2 = {1'b0, ucs_fold_tmp1[15:0]} + {13'd0, ucs_fold_tmp1[19:16]};
    wire [15:0] ucs_folded    = ucs_fold_tmp2[15:0] + {15'd0, ucs_fold_tmp2[16]};

    always @(posedge clk) begin
        ucs_tree_0     <= ucs_stg0_sum;
        ucs_tree_1     <= ucs_stg1_sum;
        ucs_tree_2     <= ucs_stg2_sum;
        ucs_final_fold <= ucs_folded;
    end
    assign calc_tx_udp_csum = (~ucs_final_fold == 16'h0000) ? 16'hFFFF : ~ucs_final_fold;

    // Pipelined IPv4 Header Checksum: computed in 2 stages during Beat 0 and Beat 1
    reg [19:0] ipcs_stg0;
    reg [15:0] ipcs_stg1;
    wire [16:0] ipcs_fold1 = {1'b0, ipcs_stg0[15:0]} + {13'd0, ipcs_stg0[19:16]};
    wire [15:0] ipcs_fold2 = ipcs_fold1[15:0] + {15'd0, ipcs_fold1[16]};
    always @(posedge clk) begin
        ipcs_stg0 <= {4'd0, 16'h4500} + {4'd0, tx_frag_ip_len} +
                     {4'd0, tx_ip_id_counter} + {4'd0, tx_frag_flags} +
                     {4'd0, 16'h4011} +
                     {4'd0, local_src_ip[31:16]} + {4'd0, local_src_ip[15:0]} +
                     {4'd0, wr0_dst_ip[31:16]}   + {4'd0, wr0_dst_ip[15:0]};
        ipcs_stg1 <= ~ipcs_fold2;
    end
    assign calc_tx_ip_hdr_csum = ipcs_stg1;

endmodule
