`timescale 1ns / 1ps

// =============================================================================
// Module: icmp_engine
//
// Description:
//   Hardware RFC 792 ICMPv4 Processor for the PYNQ-Z2 RTPS Fast-Path IP.
//   Operates in Network Big-Endian word order at 100 MHz (10 ns period) behind
//   the L2/L3 Protocol Selector in arp_reply_generator.v.
//
// Capabilities:
//   1. Hardware ICMP Echo Reply Generator (RFC 792 Type 8 -> Type 0):
//      - Validates incoming IPv4 header checksum and target IP address
//        (matches local_ip or secondary_ip).
//      - Buffers up to 64 words (256 bytes, covering standard 64B-128B Linux,
//        macOS, and Windows `ping` packets) in distributed LUTRAM.
//      - Swaps DstMAC <-> SrcMAC and DstIP <-> SrcIP.
//      - Recomputes the 16-bit IPv4 header checksum with a 2-stage carry fold.
//      - Updates ICMP Type (8 -> 0) and adjusts the 16-bit ICMP checksum via
//        RFC 1624 1's-complement differential update (~(~old_csum - 16'h0800)).
//      - Enforces IEEE 802.3 60-byte (15-word) minimum Ethernet frame length.
//
//   2. Inbound ICMP Error Parser (RFC 792 Type 3 & Type 11):
//      - Parses Destination Unreachable (Type 3: e.g. Code 1 Host Unreachable,
//        Code 3 Port Unreachable, Code 4 Fragmentation Needed) and Time
//        Exceeded (Type 11: Code 0 TTL Exceeded, Code 1 Fragment Reassembly
//        Time Exceeded).
//      - Extracts the embedded offending IPv4 header (Identification, Dst IP)
//        and offending 8-byte UDP header (Src Port, Dst Port) plus Next-Hop MTU.
//      - Pulses `icmp_err_stb` and updates telemetry registers for the PS
//        Linux driver (`axi_rtps.c`) and hardware CSRs.
// =============================================================================

module icmp_engine #(
    parameter [31:0] SECONDARY_IP = 32'h0000_0000
)(
    input  wire        clk,
    input  wire        rst_n,

    // -------------------------------------------------------------------------
    // 1. Filtered ICMP RX Stream (Network Big-Endian Order from Selector)
    // -------------------------------------------------------------------------
    input  wire [31:0] s_axis_rx_tdata,
    input  wire        s_axis_rx_tvalid,
    output wire        s_axis_rx_tready,
    input  wire        s_axis_rx_tlast,

    // -------------------------------------------------------------------------
    // 2. Generated ICMP Echo Reply TX Stream (Network Big-Endian Order)
    // -------------------------------------------------------------------------
    output reg  [31:0] m_axis_tx_tdata,
    output reg         m_axis_tx_tvalid,
    input  wire        m_axis_tx_tready,
    output reg         m_axis_tx_tlast,

    // -------------------------------------------------------------------------
    // 3. Local Network Identity & Passive MAC Learning
    // -------------------------------------------------------------------------
    input  wire [31:0] local_ip,
    input  wire [47:0] local_mac,
    input  wire [31:0] remote_client_ip,
    output reg         learn_mac_stb,
    output reg  [47:0] learned_client_mac,

    // -------------------------------------------------------------------------
    // 4. ICMP Telemetry & Error Reporting to RTPS CSRs
    // -------------------------------------------------------------------------
    output reg  [31:0] stat_icmp_echo_replies,
    output reg  [31:0] stat_icmp_errors,
    output reg         icmp_err_stb,
    output reg  [31:0] icmp_err_info,    // {icmp_type[7:0], icmp_code[7:0], next_hop_mtu[15:0]}
    output reg  [31:0] icmp_err_dst_ip,  // Offending IPv4 destination IP
    output reg  [31:0] icmp_err_ports,   // Offending UDP {src_port[15:0], dst_port[15:0]}
    output reg  [15:0] icmp_err_ip_id    // Offending IPv4 Identification
);

    // =========================================================================
    // Distributed LUTRAM Buffer for Echo Request / Reply (64 words = 256 bytes)
    // =========================================================================
    localparam integer BUF_DEPTH = 64;
    reg [31:0] pkt_buf [0:BUF_DEPTH-1];

    // =========================================================================
    // FSM States
    // =========================================================================
    localparam [1:0]
        S_RX_COLLECT = 2'd0,
        S_PREP_CSUM  = 2'd1,
        S_TX_REPLY   = 2'd2;

    reg [1:0]  state;
    reg [6:0]  rx_beat;
    reg [5:0]  tx_beat;
    reg [5:0]  tx_last_beat;
    reg [5:0]  rx_last_beat_saved;

    // Parsed RX Frame Metadata
    reg [47:0] rx_src_mac;
    reg        rx_valid_ipv4_icmp;
    reg [15:0] rx_ip_tos_ver;
    reg [15:0] rx_ip_tot_len;
    reg [15:0] rx_ip_id;
    reg [31:0] rx_src_ip;
    reg [31:0] rx_dst_ip;
    reg        rx_dst_ip_match;
    reg [7:0]  rx_icmp_type;
    reg [7:0]  rx_icmp_code;
    reg [15:0] rx_icmp_csum;
    reg [15:0] rx_icmp_id;
    reg [15:0] rx_next_hop_mtu;
    reg [15:0] rx_off_ip_id;
    reg [31:0] rx_off_dst_ip;
    reg [15:0] rx_off_src_port;
    reg [15:0] rx_off_dst_port;
    reg        rx_overflow;

    // Running 1's-complement sum of incoming 20B IPv4 header
    reg [19:0] rx_ip_csum_acc;
    wire [19:0] rx_ip_csum_beat8 = rx_ip_csum_acc + {4'd0, s_axis_rx_tdata[31:16]};
    wire [16:0] rx_ip_fold1      = {1'b0, rx_ip_csum_beat8[15:0]} + {13'd0, rx_ip_csum_beat8[19:16]};
    wire [15:0] rx_ip_fold2      = rx_ip_fold1[15:0] + {15'd0, rx_ip_fold1[16]};

    // Reply Checksum Pipeline Registers (S_PREP_CSUM)
    reg [19:0] reply_ip_sum_stg0;
    reg [15:0] reply_ip_csum;
    reg [15:0] reply_icmp_csum;

    wire [16:0] reply_ip_fold1 = {1'b0, reply_ip_sum_stg0[15:0]} + {13'd0, reply_ip_sum_stg0[19:16]};
    wire [15:0] reply_ip_fold2 = reply_ip_fold1[15:0] + {15'd0, reply_ip_fold1[16]};

    // RFC 1624 Differential ICMP Checksum Update for Type 8 (0x0800) -> Type 0 (0x0000):
    // C_new = C_old + 16'h0800 (1's-complement addition with end-around carry)
    wire [16:0] icmp_csum_diff = {1'b0, rx_icmp_csum} + 17'h0_0800;
    wire [15:0] icmp_csum_next = icmp_csum_diff[15:0] + {15'd0, icmp_csum_diff[16]};

    // Accept RX beats whenever not actively transmitting an Echo Reply
    assign s_axis_rx_tready = (state == S_RX_COLLECT);
    wire rx_hs = s_axis_rx_tvalid && s_axis_rx_tready;

    // Helper for target IP match
    wire [31:0] beat8_dst_ip = {rx_dst_ip[31:16], s_axis_rx_tdata[31:16]};
    wire        beat8_ip_ok  = (beat8_dst_ip == local_ip) ||
                               (SECONDARY_IP != 32'd0 && beat8_dst_ip == SECONDARY_IP);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                  <= S_RX_COLLECT;
            rx_beat                <= 7'd0;
            tx_beat                <= 6'd0;
            tx_last_beat           <= 6'd14;
            rx_last_beat_saved     <= 6'd0;
            rx_src_mac             <= 48'd0;
            rx_valid_ipv4_icmp     <= 1'b0;
            rx_ip_tos_ver          <= 16'h4500;
            rx_ip_tot_len          <= 16'd0;
            rx_ip_id               <= 16'd0;
            rx_src_ip              <= 32'd0;
            rx_dst_ip              <= 32'd0;
            rx_dst_ip_match        <= 1'b0;
            rx_icmp_type           <= 8'd0;
            rx_icmp_code           <= 8'd0;
            rx_icmp_csum           <= 16'd0;
            rx_icmp_id             <= 16'd0;
            rx_next_hop_mtu        <= 16'd0;
            rx_off_ip_id           <= 16'd0;
            rx_off_dst_ip          <= 32'd0;
            rx_off_src_port        <= 16'd0;
            rx_off_dst_port        <= 16'd0;
            rx_overflow            <= 1'b0;
            rx_ip_csum_acc         <= 20'd0;
            reply_ip_sum_stg0      <= 20'd0;
            reply_ip_csum          <= 16'd0;
            reply_icmp_csum        <= 16'd0;
            learn_mac_stb          <= 1'b0;
            learned_client_mac     <= 48'd0;
            stat_icmp_echo_replies <= 32'd0;
            stat_icmp_errors       <= 32'd0;
            icmp_err_stb           <= 1'b0;
            icmp_err_info          <= 32'd0;
            icmp_err_dst_ip        <= 32'd0;
            icmp_err_ports         <= 32'd0;
            icmp_err_ip_id         <= 16'd0;
        end else begin
            learn_mac_stb <= 1'b0;
            icmp_err_stb  <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // S_RX_COLLECT: Parse incoming Ethernet + IPv4 + ICMPv4 packet
                // -------------------------------------------------------------
                S_RX_COLLECT: begin
                    if (rx_hs) begin
                        if (rx_beat < BUF_DEPTH) begin
                            pkt_buf[rx_beat[5:0]] <= s_axis_rx_tdata;
                        end else begin
                            rx_overflow <= 1'b1;
                        end

                        case (rx_beat)
                            7'd0: begin
                                rx_valid_ipv4_icmp <= 1'b0;
                                rx_dst_ip_match    <= 1'b0;
                                rx_overflow        <= 1'b0;
                                rx_ip_csum_acc     <= 20'd0;
                                rx_icmp_type       <= 8'hFF;
                                rx_icmp_code       <= 8'hFF;
                            end

                            7'd1: begin
                                rx_src_mac[47:32] <= s_axis_rx_tdata[15:0];
                            end

                            7'd2: begin
                                rx_src_mac[31:0] <= s_axis_rx_tdata;
                            end

                            7'd3: begin
                                // EtherType == 0x0800 and IPv4 Version/IHL == 0x45 (20B header)
                                if (s_axis_rx_tdata[31:16] == 16'h0800 &&
                                    s_axis_rx_tdata[15:8]  == 8'h45) begin
                                    rx_valid_ipv4_icmp <= 1'b1;
                                end
                                rx_ip_tos_ver  <= s_axis_rx_tdata[15:0];
                                rx_ip_csum_acc <= {4'd0, s_axis_rx_tdata[15:0]};
                            end

                            7'd4: begin
                                rx_ip_tot_len  <= s_axis_rx_tdata[31:16];
                                rx_ip_id       <= s_axis_rx_tdata[15:0];
                                rx_ip_csum_acc <= rx_ip_csum_acc +
                                                  {4'd0, s_axis_rx_tdata[31:16]} +
                                                  {4'd0, s_axis_rx_tdata[15:0]};
                            end

                            7'd5: begin
                                // Must be non-fragmented (MF=0, offset=0) and Protocol == 0x01 (ICMP)
                                if (s_axis_rx_tdata[29:16] != 14'd0 ||
                                    s_axis_rx_tdata[7:0]   != 8'h01) begin
                                    rx_valid_ipv4_icmp <= 1'b0;
                                end
                                rx_ip_csum_acc <= rx_ip_csum_acc +
                                                  {4'd0, s_axis_rx_tdata[31:16]} +
                                                  {4'd0, s_axis_rx_tdata[15:0]};
                            end

                            7'd6: begin
                                rx_src_ip[31:16] <= s_axis_rx_tdata[15:0];
                                rx_ip_csum_acc   <= rx_ip_csum_acc +
                                                    {4'd0, s_axis_rx_tdata[31:16]} +
                                                    {4'd0, s_axis_rx_tdata[15:0]};
                            end

                            7'd7: begin
                                rx_src_ip[15:0]  <= s_axis_rx_tdata[31:16];
                                rx_dst_ip[31:16] <= s_axis_rx_tdata[15:0];
                                rx_ip_csum_acc   <= rx_ip_csum_acc +
                                                    {4'd0, s_axis_rx_tdata[31:16]} +
                                                    {4'd0, s_axis_rx_tdata[15:0]};
                            end

                            7'd8: begin
                                rx_dst_ip[15:0] <= s_axis_rx_tdata[31:16];
                                rx_icmp_type    <= s_axis_rx_tdata[15:8];
                                rx_icmp_code    <= s_axis_rx_tdata[7:0];
                                rx_dst_ip_match <= beat8_ip_ok;
                                // Verify IPv4 header checksum (1's-complement sum must equal 0xFFFF)
                                if (rx_ip_fold2 != 16'hFFFF) begin
                                    rx_valid_ipv4_icmp <= 1'b0;
                                end
                            end

                            7'd9: begin
                                rx_icmp_csum <= s_axis_rx_tdata[31:16];
                                rx_icmp_id   <= s_axis_rx_tdata[15:0];
                            end

                            7'd10: begin
                                // For Type 3/11: [31:16] is Next-Hop MTU, [15:0] is Offending IPv4 Ver/IHL/TOS
                                rx_next_hop_mtu <= s_axis_rx_tdata[31:16];
                            end

                            7'd11: begin
                                // Offending IPv4 Total Length [31:16], Identification [15:0]
                                rx_off_ip_id <= s_axis_rx_tdata[15:0];
                            end

                            7'd14: begin
                                // Offending IPv4 SrcIP[15:0] in [31:16], DstIP[31:16] in [15:0]
                                rx_off_dst_ip[31:16] <= s_axis_rx_tdata[15:0];
                            end

                            7'd15: begin
                                // Offending IPv4 DstIP[15:0] in [31:16], Offending UDP SrcPort[15:0] in [15:0]
                                rx_off_dst_ip[15:0] <= s_axis_rx_tdata[31:16];
                                rx_off_src_port     <= s_axis_rx_tdata[15:0];
                            end

                            7'd16: begin
                                // Offending UDP DstPort[15:0] in [31:16]
                                rx_off_dst_port <= s_axis_rx_tdata[31:16];
                            end

                            default: ;
                        endcase

                        if (s_axis_rx_tlast) begin
                            rx_beat            <= 7'd0;
                            rx_last_beat_saved <= rx_beat[5:0];

                            if (rx_valid_ipv4_icmp && rx_dst_ip_match) begin
                                // Passive MAC learning on valid unicast ICMP from remote peer
                                if (remote_client_ip == 32'd0 || rx_src_ip == remote_client_ip) begin
                                    learn_mac_stb      <= 1'b1;
                                    learned_client_mac <= rx_src_mac;
                                end

                                // Case A: ICMP Echo Request (Type 8, Code 0)
                                if (rx_icmp_type == 8'd8 && rx_icmp_code == 8'd0 &&
                                    rx_beat >= 7'd10 && !rx_overflow) begin
                                    tx_last_beat      <= (rx_beat < 7'd14) ? 6'd14 : rx_beat[5:0];
                                    reply_ip_sum_stg0 <= {4'd0, rx_ip_tos_ver} +
                                                         {4'd0, rx_ip_tot_len} +
                                                         {4'd0, rx_ip_id}      +
                                                         {4'd0, 16'h0000}      + // Flags/FragOffset = 0
                                                         {4'd0, 16'h4001}      + // TTL=64 (0x40), Proto=ICMP (0x01)
                                                         {4'd0, rx_dst_ip[31:16]} +
                                                         {4'd0, rx_dst_ip[15:0]}  +
                                                         {4'd0, rx_src_ip[31:16]} +
                                                         {4'd0, rx_src_ip[15:0]};
                                    reply_icmp_csum   <= icmp_csum_next;
                                    state             <= S_PREP_CSUM;
                                end
                                // Case B: ICMP Destination Unreachable (Type 3) or Time Exceeded (Type 11)
                                else if ((rx_icmp_type == 8'd3 || rx_icmp_type == 8'd11) && rx_beat >= 7'd15) begin
                                    stat_icmp_errors <= stat_icmp_errors + 32'd1;
                                    icmp_err_stb     <= 1'b1;
                                    icmp_err_info    <= {rx_icmp_type, rx_icmp_code, rx_next_hop_mtu};
                                    icmp_err_dst_ip  <= (rx_beat == 7'd15) ?
                                                        {rx_off_dst_ip[31:16], s_axis_rx_tdata[31:16]} :
                                                        rx_off_dst_ip;
                                    icmp_err_ports   <= (rx_beat == 7'd16) ?
                                                        {rx_off_src_port, s_axis_rx_tdata[31:16]} :
                                                        {rx_off_src_port, rx_off_dst_port};
                                    icmp_err_ip_id   <= rx_off_ip_id;
                                end
                            end
                        end else begin
                            rx_beat <= rx_beat + 7'd1;
                        end
                    end
                end

                // -------------------------------------------------------------
                // S_PREP_CSUM: 1-cycle pipeline stage for 2-stage carry fold
                // -------------------------------------------------------------
                S_PREP_CSUM: begin
                    reply_ip_csum          <= ~reply_ip_fold2;
                    tx_beat                <= 6'd0;
                    stat_icmp_echo_replies <= stat_icmp_echo_replies + 32'd1;
                    state                  <= S_TX_REPLY;
                end

                // -------------------------------------------------------------
                // S_TX_REPLY: Stream out Echo Reply on m_axis_tx
                // -------------------------------------------------------------
                S_TX_REPLY: begin
                    if (m_axis_tx_tready) begin
                        if (tx_beat == tx_last_beat) begin
                            tx_beat <= 6'd0;
                            state   <= S_RX_COLLECT;
                        end else begin
                            tx_beat <= tx_beat + 6'd1;
                        end
                    end
                end

                default: state <= S_RX_COLLECT;
            endcase
        end
    end

    // =========================================================================
    // Combinational TX Frame Builder (Network Big-Endian Order)
    // =========================================================================
    always @(*) begin
        m_axis_tx_tvalid = (state == S_TX_REPLY);
        m_axis_tx_tlast  = (state == S_TX_REPLY) && (tx_beat == tx_last_beat);

        case (tx_beat)
            6'd0: m_axis_tx_tdata = rx_src_mac[47:16];
            6'd1: m_axis_tx_tdata = {rx_src_mac[15:0], local_mac[47:32]};
            6'd2: m_axis_tx_tdata = local_mac[31:0];
            6'd3: m_axis_tx_tdata = {16'h0800, rx_ip_tos_ver};
            6'd4: m_axis_tx_tdata = {rx_ip_tot_len, rx_ip_id};
            6'd5: m_axis_tx_tdata = 32'h0000_4001; // Flags=0, FragOffset=0, TTL=64, Proto=ICMP(1)
            6'd6: m_axis_tx_tdata = {reply_ip_csum, rx_dst_ip[31:16]};
            6'd7: m_axis_tx_tdata = {rx_dst_ip[15:0], rx_src_ip[31:16]};
            6'd8: m_axis_tx_tdata = {rx_src_ip[15:0], 8'h00, 8'h00}; // ICMP Type 0 (Echo Reply), Code 0
            6'd9: m_axis_tx_tdata = {reply_icmp_csum, rx_icmp_id};
            default: m_axis_tx_tdata = (tx_beat <= rx_last_beat_saved) ? pkt_buf[tx_beat] : 32'd0;
        endcase
    end

endmodule

