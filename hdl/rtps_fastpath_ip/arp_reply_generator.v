`timescale 1ns / 1ps

// =============================================================================
// Module: arp_reply_generator
//
// Description:
//   Zero-Stall L2/L3 Protocol Selector, Hardware ARP Engine (RFC 826),
//   Hardware ICMP Engine (RFC 792), and 3-Way Atomic Packet TX Arbiter
//   for the RTPS Fast-Path Hardware Accelerator.
//
// Functionality:
//   1. Zero-Stall Ingress Protocol Selector (s_arp_rx):
//      - Buffers up to 6 beats (60 ns @ 100 MHz) in an elastic lookahead FIFO.
//      - Beat 3:
//        * If EtherType == 0x0806 (ARP):
//          Filters and absorbs the ARP frame into the RFC 826 ARP engine.
//          Flushes the RTPS lookahead FIFO so ARP never perturbs RTPS.
//        * If EtherType == 0x0800 (IPv4):
//          Proceeds to Beat 5 inspection.
//        * Other EtherTypes:
//          Immediately marks pass-through to m_rtps_tx.
//      - Beat 5:
//        * If IPv4 Protocol == 0x01 (ICMP):
//          Filters and absorbs into u_icmp_engine (hardware ping echo reply and
//          ICMP error parsing). Flushes the RTPS lookahead FIFO so ICMP never
//          perturbs RTPS.
//        * If IPv4 Protocol == 0x11 (UDP) or other:
//          Marks pass-through (rx_passing_pkt <= 1), draining the lookahead FIFO
//          to m_rtps_tx (RTPS accelerator) with full AXI-Stream flow control.
//
//   2. 3-Way Atomic Packet TX Arbiter (m_arp_tx):
//      - Arbitrates between:
//        * Priority 0: RFC 826 ARP Replies (generated locally)
//        * Priority 1: RFC 792 ICMP Echo Replies (from u_icmp_engine)
//        * Priority 2: RTPS FastPath Egress Frames (from s_rtps_rx)
//      - Enforces strict atomic packet framing (zero interleaving, zero truncation).
//
//   3. Bidirectional MAC Resolution & Passive Client Learning:
//      - Learns remote client MAC from both incoming ARP requests and ICMP pings/errors.
// =============================================================================

module arp_reply_generator #(
    parameter integer DATA_WIDTH     = 32,
    parameter integer AXIS_BYTE_SWAP = 1,          // 1 = AXI-Stream byte order (LSB byte 0), 0 = Network big-endian
    parameter [31:0]  ARP_REPLY_IP_2 = 32'h00000000 // Optional secondary IP to answer (e.g. 192.168.1.201)
)(
    input  wire                      clk,
    input  wire                      rst_n,

    // -------------------------------------------------------------------------
    // 1. Connection with Upstream IP (RTPS FastPath / PS DMA)
    // -------------------------------------------------------------------------
    // Forwarded non-ARP, non-ICMP ingress traffic to RTPS IP (connects to RTPS s_axis_wire)
    output wire [DATA_WIDTH-1:0]     m_rtps_tx_tdata,
    output wire                      m_rtps_tx_tvalid,
    input  wire                      m_rtps_tx_tready,
    output wire                      m_rtps_tx_tlast,

    // Outgoing egress traffic from RTPS IP (connects to RTPS m_axis_wire)
    input  wire [DATA_WIDTH-1:0]     s_rtps_rx_tdata,
    input  wire                      s_rtps_rx_tvalid,
    output wire                      s_rtps_rx_tready,
    input  wire                      s_rtps_rx_tlast,

    // -------------------------------------------------------------------------
    // 2. Connection with Downstream Network (Ethernet MAC / RMII PHY)
    // -------------------------------------------------------------------------
    // Outgoing egress traffic to Ethernet MAC TX
    output wire [DATA_WIDTH-1:0]     m_arp_tx_tdata,
    output wire                      m_arp_tx_tvalid,
    input  wire                      m_arp_tx_tready,
    output wire                      m_arp_tx_tlast,

    // Incoming ingress traffic from Ethernet MAC RX
    input  wire [DATA_WIDTH-1:0]     s_arp_rx_tdata,
    input  wire                      s_arp_rx_tvalid,
    output wire                      s_arp_rx_tready,
    input  wire                      s_arp_rx_tlast,

    // -------------------------------------------------------------------------
    // 3. Network Configuration & Client Learning
    // -------------------------------------------------------------------------
    input  wire [31:0]               arp_reply_ip,
    input  wire [47:0]               arp_reply_mac,
    input  wire [31:0]               remote_client_ip,
    output reg  [47:0]               remote_client_mac,

    // -------------------------------------------------------------------------
    // 4. ICMP Telemetry & Error Reporting
    // -------------------------------------------------------------------------
    output wire [31:0]               stat_icmp_echo_replies,
    output wire [31:0]               stat_icmp_errors,
    output wire                      icmp_err_stb,
    output wire [31:0]               icmp_err_info,
    output wire [31:0]               icmp_err_dst_ip,
    output wire [31:0]               icmp_err_ports,
    output wire [15:0]               icmp_err_ip_id
);

    // =========================================================================
    // Endianness Conversion Helper Function
    // =========================================================================
    function [31:0] byte_swap32;
        input [31:0] in;
        begin
            byte_swap32 = {in[7:0], in[15:8], in[23:16], in[31:24]};
        end
    endfunction

    // =========================================================================
    // Part 1: Incoming Ethernet Frame Parsing & Protocol Demux
    // =========================================================================
    wire [31:0] rx_net_word  = AXIS_BYTE_SWAP ? byte_swap32(s_arp_rx_tdata) : s_arp_rx_tdata;
    wire        rx_handshake = s_arp_rx_tvalid && s_arp_rx_tready;

    reg [3:0]  rx_wire_beat;
    reg        rx_is_arp_frame;
    reg        rx_is_arp_req;
    reg        rx_is_ipv4_frame;
    reg        rx_target_ip_match;
    reg [47:0] rx_sha_mac;
    reg [31:0] rx_spa_ip;
    reg [15:0] rx_tpa_hi;
    reg [31:0] matched_spa_ip;

    reg        arp_reply_trigger;
    reg [47:0] latched_reply_dst_mac;
    reg [31:0] latched_reply_dst_ip;
    reg [31:0] latched_reply_spa_ip;

    // Filter flags: set when the current incoming frame is filtered out of RTPS
    reg rx_filtering_arp;
    reg rx_filtering_icmp;
    reg rx_drop_fifo;

    // Passive learning from ICMP engine
    wire        icmp_learn_mac_stb;
    wire [47:0] icmp_learned_client_mac;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_wire_beat          <= 4'd0;
            rx_is_arp_frame       <= 1'b0;
            rx_is_arp_req         <= 1'b0;
            rx_is_ipv4_frame      <= 1'b0;
            rx_target_ip_match    <= 1'b0;
            rx_sha_mac            <= 48'd0;
            rx_spa_ip             <= 32'd0;
            rx_tpa_hi             <= 16'd0;
            matched_spa_ip        <= 32'd0;
            remote_client_mac     <= 48'd0;
            arp_reply_trigger     <= 1'b0;
            latched_reply_dst_mac <= 48'd0;
            latched_reply_dst_ip  <= 32'd0;
            latched_reply_spa_ip  <= 32'd0;
            rx_filtering_arp      <= 1'b0;
            rx_filtering_icmp     <= 1'b0;
            rx_drop_fifo          <= 1'b0;
        end else begin
            arp_reply_trigger <= 1'b0;
            rx_drop_fifo      <= 1'b0;

            // If a previous pass-through frame was stalled and downstream RTPS is not consuming,
            // drop stale FIFO contents when a new frame begins so ARP/ICMP lookahead is never polluted.
            if (s_arp_rx_tvalid && rx_wire_beat == 4'd0 && fifo_count > 0 && !m_rtps_tx_tready) begin
                rx_drop_fifo <= 1'b1;
            end

            // Passive MAC learning from ICMP Engine
            if (icmp_learn_mac_stb) begin
                remote_client_mac <= icmp_learned_client_mac;
            end

            if (rx_handshake) begin
                case (rx_wire_beat)
                    4'd0: begin
                        // Beat 0: Ethernet Dst MAC [47:16]
                        rx_is_arp_frame    <= 1'b0;
                        rx_is_arp_req      <= 1'b0;
                        rx_is_ipv4_frame   <= 1'b0;
                        rx_target_ip_match <= 1'b0;
                        rx_filtering_arp   <= 1'b0;
                        rx_filtering_icmp  <= 1'b0;
                    end

                    4'd1: begin
                        // Beat 1: Dst MAC [15:0] + Src MAC [47:32]
                    end

                    4'd2: begin
                        // Beat 2: Src MAC [31:0]
                    end

                    4'd3: begin
                        // Beat 3: EtherType (bits [31:16])
                        if (rx_net_word[31:16] == 16'h0806 && rx_net_word[15:0] == 16'h0001) begin
                            // ARP Request/Reply (0x0806 / HTYPE 0x0001)
                            rx_is_arp_frame  <= 1'b1;
                            rx_filtering_arp <= 1'b1;
                            rx_drop_fifo     <= 1'b1; // Flush pre-buffered beats from RTPS FIFO
                        end else if (rx_net_word[31:16] == 16'h0800) begin
                            // IPv4 Packet (0x0800)
                            rx_is_ipv4_frame <= 1'b1;
                        end else begin
                            rx_is_arp_frame  <= 1'b0;
                            rx_filtering_arp <= 1'b0;
                        end
                    end

                    4'd4: begin
                        // Beat 4: ARP PTYPE/HLEN/PLEN or IPv4 TotalLen/IP_ID
                        if (rx_is_arp_frame &&
                            rx_net_word[31:16] == 16'h0800 &&
                            rx_net_word[15:8]  == 8'h06    &&
                            rx_net_word[7:0]   == 8'h04) begin
                            // ARP parameters valid
                        end else if (rx_is_arp_frame) begin
                            rx_is_arp_frame <= 1'b0;
                        end
                    end

                    4'd5: begin
                        // Beat 5:
                        // For ARP:  OPER (bits [31:16]) + SHA [47:32] (bits [15:0])
                        // For IPv4: Flags/Offset [31:16] + TTL [15:8] + Protocol [7:0]
                        if (rx_is_arp_frame && rx_net_word[31:16] == 16'h0001) begin
                            rx_is_arp_req     <= 1'b1;
                            rx_sha_mac[47:32] <= rx_net_word[15:0];
                        end else if (rx_is_arp_frame) begin
                            rx_is_arp_frame <= 1'b0;
                            rx_is_arp_req   <= 1'b0;
                        end

                        if (rx_is_ipv4_frame && rx_net_word[7:0] == 8'h01) begin
                            // IPv4 Protocol 0x01: ICMP!
                            rx_filtering_icmp <= 1'b1;
                            rx_drop_fifo      <= 1'b1; // Flush pre-buffered beats from RTPS FIFO
                        end
                    end

                    4'd6: begin
                        // Beat 6: SHA [31:0] (Sender MAC low 32 bits)
                        if (rx_is_arp_req) begin
                            rx_sha_mac[31:0] <= rx_net_word[31:0];
                        end
                    end

                    4'd7: begin
                        // Beat 7: SPA [31:0] (Sender Protocol Address / Sender IPv4)
                        if (rx_is_arp_req) begin
                            rx_spa_ip <= rx_net_word[31:0];
                        end
                    end

                    4'd8: begin
                        // Beat 8: THA [47:16] (Target MAC high 32 bits - ignored in ARP request)
                    end

                    4'd9: begin
                        // Beat 9: THA [15:0] (bits [31:16]) + TPA [31:16] (bits [15:0] - Target IP high)
                        if (rx_is_arp_req) begin
                            rx_tpa_hi <= rx_net_word[15:0];
                        end
                    end

                    4'd10: begin
                        // Beat 10: TPA [15:0] (bits [31:16] - Target IP low) + Padding (bits [15:0])
                        if (rx_is_arp_req) begin
                            if ({rx_tpa_hi, rx_net_word[31:16]} == arp_reply_ip) begin
                                rx_target_ip_match <= 1'b1;
                                matched_spa_ip     <= arp_reply_ip;
                            end else if (ARP_REPLY_IP_2 != 32'd0 && {rx_tpa_hi, rx_net_word[31:16]} == ARP_REPLY_IP_2) begin
                                rx_target_ip_match <= 1'b1;
                                matched_spa_ip     <= ARP_REPLY_IP_2;
                            end else begin
                                rx_target_ip_match <= 1'b0;
                            end
                        end
                    end

                    default: begin
                        // Beats 11..15: Frame padding
                    end
                endcase

                if (s_arp_rx_tlast) begin
                    rx_wire_beat      <= 4'd0;
                    rx_filtering_arp  <= 1'b0;
                    rx_filtering_icmp <= 1'b0;
                    rx_is_ipv4_frame  <= 1'b0;

                    // Evaluate ARP reply trigger on end of frame
                    if (rx_is_arp_req &&
                        ((rx_wire_beat >= 4'd10 && rx_target_ip_match) ||
                         (rx_wire_beat == 4'd10 && ({rx_tpa_hi, rx_net_word[31:16]} == arp_reply_ip ||
                                                    (ARP_REPLY_IP_2 != 32'd0 && {rx_tpa_hi, rx_net_word[31:16]} == ARP_REPLY_IP_2))))) begin
                        arp_reply_trigger     <= 1'b1;
                        latched_reply_dst_mac <= rx_sha_mac;
                        latched_reply_dst_ip  <= rx_spa_ip;
                        latched_reply_spa_ip  <= rx_target_ip_match ? matched_spa_ip :
                                                 (ARP_REPLY_IP_2 != 32'd0 && {rx_tpa_hi, rx_net_word[31:16]} == ARP_REPLY_IP_2) ? ARP_REPLY_IP_2 :
                                                 arp_reply_ip;

                        if (remote_client_ip == 32'd0 || rx_spa_ip == remote_client_ip) begin
                            remote_client_mac <= rx_sha_mac;
                        end
                    end

                    rx_is_arp_frame    <= 1'b0;
                    rx_is_arp_req      <= 1'b0;
                    rx_target_ip_match <= 1'b0;
                end else if (rx_wire_beat != 4'd15) begin
                    rx_wire_beat <= rx_wire_beat + 4'd1;
                end
            end
        end
    end

    // =========================================================================
    // Part 2: Elastic Ingress Buffer (Passes Non-ARP, Non-ICMP traffic to RTPS)
    // =========================================================================
    // Buffers Beats 0..5 in an elastic circular FIFO.
    // If Beat 3 is ARP: flushed immediately.
    // If Beat 5 is ICMP: flushed immediately.
    // Otherwise (e.g. UDP RTPS): drains directly to m_rtps_tx.
    localparam FIFO_DEPTH = 16;
    reg [DATA_WIDTH-1:0] fifo_data [0:FIFO_DEPTH-1];
    reg                  fifo_last [0:FIFO_DEPTH-1];
    reg [3:0]            fifo_wr_ptr;
    reg [3:0]            fifo_rd_ptr;
    reg [4:0]            fifo_count;
    reg                  rx_passing_pkt;

    wire fifo_push = s_arp_rx_tvalid && s_arp_rx_tready && !rx_filtering_arp && !rx_filtering_icmp && !rx_drop_fifo && (fifo_count < FIFO_DEPTH);
    wire fifo_pop  = m_rtps_tx_tvalid && m_rtps_tx_tready;

    // Always accept Beats 0..5 (header lookahead window) and all ARP/ICMP beats so
    // network discovery / ping can NEVER be deadlocked by a full or stalled RTPS FIFO.
    assign s_arp_rx_tready = (rx_filtering_arp || rx_filtering_icmp || rx_wire_beat < 4'd6) ? 1'b1 :
                             (fifo_count < (FIFO_DEPTH - 2));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_wr_ptr    <= 4'd0;
            fifo_rd_ptr    <= 4'd0;
            fifo_count     <= 5'd0;
            rx_passing_pkt <= 1'b0;
        end else begin
            // 1. Write Port
            if (fifo_push) begin
                fifo_data[fifo_wr_ptr] <= s_arp_rx_tdata;
                fifo_last[fifo_wr_ptr] <= s_arp_rx_tlast;
                fifo_wr_ptr            <= fifo_wr_ptr + 4'd1;
            end

            // 2. Read Port
            if (fifo_pop) begin
                fifo_rd_ptr <= fifo_rd_ptr + 4'd1;
                if (fifo_last[fifo_rd_ptr]) begin
                    rx_passing_pkt <= 1'b0;
                end
            end

            // 3. FIFO Level Tracking & Dropping
            if (rx_drop_fifo) begin
                fifo_wr_ptr    <= fifo_rd_ptr;
                fifo_count     <= 5'd0;
                rx_passing_pkt <= 1'b0;
            end else begin
                // Activate pass-through once confirmed non-ARP and non-ICMP
                if (rx_handshake) begin
                    if (rx_wire_beat == 4'd3 && rx_net_word[31:16] != 16'h0806 && rx_net_word[31:16] != 16'h0800) begin
                        // Non-ARP, non-IPv4 frame (e.g. raw Ethernet)
                        rx_passing_pkt <= 1'b1;
                    end else if (rx_wire_beat == 4'd5 && rx_is_ipv4_frame && rx_net_word[7:0] != 8'h01) begin
                        // IPv4 frame that is NOT ICMP (e.g. UDP RTPS!)
                        rx_passing_pkt <= 1'b1;
                    end else if (s_arp_rx_tlast && !rx_is_arp_frame && (!rx_is_ipv4_frame || rx_wire_beat < 4'd5)) begin
                        // Early short frame termination
                        rx_passing_pkt <= 1'b1;
                    end
                end

                case ({fifo_push, fifo_pop})
                    2'b10:   fifo_count <= fifo_count + 5'd1;
                    2'b01:   fifo_count <= fifo_count - 5'd1;
                    default: fifo_count <= fifo_count;
                endcase
            end
        end
    end

    assign m_rtps_tx_tvalid = rx_passing_pkt && (fifo_count > 0);
    assign m_rtps_tx_tdata  = fifo_data[fifo_rd_ptr];
    assign m_rtps_tx_tlast  = fifo_last[fifo_rd_ptr];

    // =========================================================================
    // Part 3: RFC 826 ARP Reply Frame Generator
    // =========================================================================
    reg [3:0]  tx_arp_beat;
    reg [31:0] arp_reply_word;

    always @(*) begin
        case (tx_arp_beat)
            // Ethernet Destination MAC [47:16]
            4'd0: arp_reply_word = latched_reply_dst_mac[47:16];

            // Ethernet Destination MAC [15:0] + Ethernet Source MAC [47:32]
            4'd1: arp_reply_word = {latched_reply_dst_mac[15:0], arp_reply_mac[47:32]};

            // Ethernet Source MAC [31:0]
            4'd2: arp_reply_word = arp_reply_mac[31:0];

            // EtherType 0x0806 (ARP) + Hardware Type HTYPE 0x0001 (Ethernet)
            4'd3: arp_reply_word = {16'h0806, 16'h0001};

            // Protocol Type PTYPE 0x0800 (IPv4) + HLEN 0x06 + PLEN 0x04
            4'd4: arp_reply_word = {16'h0800, 8'h06, 8'h04};

            // Operation OPER 0x0002 (ARP Reply) + Sender Hardware Address SHA [47:32]
            4'd5: arp_reply_word = {16'h0002, arp_reply_mac[47:32]};

            // Sender Hardware Address SHA [31:0] (FPGA MAC low 32 bits)
            4'd6: arp_reply_word = arp_reply_mac[31:0];

            // Sender Protocol Address SPA [31:0] (FPGA IP)
            4'd7: arp_reply_word = latched_reply_spa_ip;

            // Target Hardware Address THA [47:16] (Laptop MAC high 32 bits)
            4'd8: arp_reply_word = latched_reply_dst_mac[47:16];

            // Target Hardware Address THA [15:0] + Target Protocol Address TPA [31:16]
            4'd9: arp_reply_word = {latched_reply_dst_mac[15:0], latched_reply_dst_ip[31:16]};

            // Target Protocol Address TPA [15:0] + Ethernet Padding [15:0]
            4'd10: arp_reply_word = {latched_reply_dst_ip[15:0], 16'h0000};

            // Padding words 11..14 (Zeros to meet 60-byte minimum Ethernet frame size)
            4'd11: arp_reply_word = 32'h0000_0000;
            4'd12: arp_reply_word = 32'h0000_0000;
            4'd13: arp_reply_word = 32'h0000_0000;
            4'd14: arp_reply_word = 32'h0000_0000;

            default: arp_reply_word = 32'h0000_0000;
        endcase
    end

    // =========================================================================
    // Part 4: Hardware ICMPv4 Engine Instance (RFC 792)
    // =========================================================================
    wire [31:0] icmp_tx_tdata;
    wire        icmp_tx_tvalid;
    reg         icmp_tx_tready;
    wire        icmp_tx_tlast;

    icmp_engine #(
        .SECONDARY_IP (ARP_REPLY_IP_2)
    ) u_icmp_engine (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .s_axis_rx_tdata        (rx_net_word),
        .s_axis_rx_tvalid       (s_arp_rx_tvalid),
        .s_axis_rx_tready       (), // Always ready in S_RX_COLLECT
        .s_axis_rx_tlast        (s_arp_rx_tlast),
        .m_axis_tx_tdata        (icmp_tx_tdata),
        .m_axis_tx_tvalid       (icmp_tx_tvalid),
        .m_axis_tx_tready       (icmp_tx_tready),
        .m_axis_tx_tlast        (icmp_tx_tlast),
        .local_ip               (arp_reply_ip),
        .local_mac              (arp_reply_mac),
        .remote_client_ip       (remote_client_ip),
        .learn_mac_stb          (icmp_learn_mac_stb),
        .learned_client_mac     (icmp_learned_client_mac),
        .stat_icmp_echo_replies (stat_icmp_echo_replies),
        .stat_icmp_errors       (stat_icmp_errors),
        .icmp_err_stb           (icmp_err_stb),
        .icmp_err_info          (icmp_err_info),
        .icmp_err_dst_ip        (icmp_err_dst_ip),
        .icmp_err_ports         (icmp_err_ports),
        .icmp_err_ip_id         (icmp_err_ip_id)
    );

    // =========================================================================
    // Part 5: 3-Way Atomic Packet TX Arbiter (ARP > ICMP > RTPS)
    // =========================================================================
    localparam [1:0]
        TX_IDLE      = 2'd0,
        TX_ARP_RESP  = 2'd1,
        TX_ICMP_RESP = 2'd2,
        TX_RTPS_FWD  = 2'd3;

    reg [1:0] tx_state;
    reg       arp_reply_pending;

    // Track pending ARP replies
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arp_reply_pending <= 1'b0;
        end else begin
            if (arp_reply_trigger) begin
                arp_reply_pending <= 1'b1;
            end else if (tx_state == TX_ARP_RESP && m_arp_tx_tready && (tx_arp_beat == 4'd14)) begin
                arp_reply_pending <= 1'b0;
            end
        end
    end

    // 3-Way Atomic Arbiter FSM
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state    <= TX_IDLE;
            tx_arp_beat <= 4'd0;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    if (arp_reply_pending) begin
                        tx_state    <= TX_ARP_RESP;
                        tx_arp_beat <= 4'd0;
                    end else if (icmp_tx_tvalid) begin
                        tx_state    <= TX_ICMP_RESP;
                    end else if (s_rtps_rx_tvalid) begin
                        if (m_arp_tx_tready && s_rtps_rx_tlast) begin
                            tx_state <= TX_IDLE;
                        end else begin
                            tx_state <= TX_RTPS_FWD;
                        end
                    end
                end

                TX_ARP_RESP: begin
                    if (m_arp_tx_tready) begin
                        if (tx_arp_beat == 4'd14) begin
                            tx_arp_beat <= 4'd0;
                            tx_state    <= TX_IDLE;
                        end else begin
                            tx_arp_beat <= tx_arp_beat + 4'd1;
                        end
                    end
                end

                TX_ICMP_RESP: begin
                    if (icmp_tx_tvalid && m_arp_tx_tready && icmp_tx_tlast) begin
                        tx_state <= TX_IDLE;
                    end
                end

                TX_RTPS_FWD: begin
                    if (s_rtps_rx_tvalid && m_arp_tx_tready && s_rtps_rx_tlast) begin
                        tx_state <= TX_IDLE;
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    // Combinational TX Multiplexing
    reg [DATA_WIDTH-1:0] mux_arp_tx_tdata;
    reg                  mux_arp_tx_tvalid;
    reg                  mux_arp_tx_tlast;
    reg                  mux_rtps_rx_tready;

    always @(*) begin
        icmp_tx_tready     = 1'b0;
        mux_rtps_rx_tready = 1'b0;

        case (tx_state)
            TX_ARP_RESP: begin
                mux_arp_tx_tvalid  = 1'b1;
                mux_arp_tx_tdata   = AXIS_BYTE_SWAP ? byte_swap32(arp_reply_word) : arp_reply_word;
                mux_arp_tx_tlast   = (tx_arp_beat == 4'd14);
                icmp_tx_tready     = 1'b0;
                mux_rtps_rx_tready = 1'b0;
            end

            TX_ICMP_RESP: begin
                mux_arp_tx_tvalid  = icmp_tx_tvalid;
                mux_arp_tx_tdata   = AXIS_BYTE_SWAP ? byte_swap32(icmp_tx_tdata) : icmp_tx_tdata;
                mux_arp_tx_tlast   = icmp_tx_tlast;
                icmp_tx_tready     = m_arp_tx_tready;
                mux_rtps_rx_tready = 1'b0;
            end

            TX_RTPS_FWD: begin
                mux_arp_tx_tvalid  = s_rtps_rx_tvalid;
                mux_arp_tx_tdata   = s_rtps_rx_tdata;
                mux_arp_tx_tlast   = s_rtps_rx_tlast;
                icmp_tx_tready     = 1'b0;
                mux_rtps_rx_tready = m_arp_tx_tready;
            end

            default: begin // TX_IDLE
                if (arp_reply_pending) begin
                    mux_arp_tx_tvalid  = 1'b0;
                    mux_arp_tx_tdata   = 32'd0;
                    mux_arp_tx_tlast   = 1'b0;
                    icmp_tx_tready     = 1'b0;
                    mux_rtps_rx_tready = 1'b0;
                end else if (icmp_tx_tvalid) begin
                    mux_arp_tx_tvalid  = 1'b0;
                    mux_arp_tx_tdata   = 32'd0;
                    mux_arp_tx_tlast   = 1'b0;
                    icmp_tx_tready     = 1'b0;
                    mux_rtps_rx_tready = 1'b0;
                end else begin
                    mux_arp_tx_tvalid  = s_rtps_rx_tvalid;
                    mux_arp_tx_tdata   = s_rtps_rx_tdata;
                    mux_arp_tx_tlast   = s_rtps_rx_tlast;
                    icmp_tx_tready     = 1'b0;
                    mux_rtps_rx_tready = m_arp_tx_tready;
                end
            end
        endcase
    end

    assign m_arp_tx_tvalid  = mux_arp_tx_tvalid;
    assign m_arp_tx_tdata   = mux_arp_tx_tdata;
    assign m_arp_tx_tlast   = mux_arp_tx_tlast;
    assign s_rtps_rx_tready = mux_rtps_rx_tready;

endmodule
