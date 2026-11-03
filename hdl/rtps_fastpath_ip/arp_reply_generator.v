`timescale 1ns / 1ps

// =============================================================================
// Module: arp_reply_generator
//
// Description:
//   Hardware ARP (Address Resolution Protocol - RFC 826) Reply Generator and
//   L2 Ethernet Shim for the RTPS Fast-Path Hardware Accelerator.
//
// Functionality:
//   1. Shim Multiplexer between Downstream Network Port (Ethernet MAC / Wire)
//      and Upstream Accelerator (rtps_fastpath_ip / PS DMA):
//      - Wire RX (s_arp_rx): Inspects incoming Ethernet frames.
//        * If ARP Request targeting our IP (arp_reply_ip or ARP_REPLY_IP_2):
//          Filters and absorbs the ARP frame so it never leaks into RTPS.
//          Learns the remote peer's MAC (remote_client_mac) and triggers
//          an authentic RFC 826 ARP Reply.
//        * If Non-ARP Traffic (e.g. IPv4 UDP RTPS frames):
//          Streams through to m_rtps_tx with full AXI-Stream backpressure.
//      - Wire TX (m_arp_tx): Arbitrates between outgoing RTPS accelerator
//        traffic (s_rtps_rx) and generated ARP replies with atomic packet
//        framing (no interleaving, zero frame truncation).
//   2. Protocol Alignment & Endianness:
//      - Configurable AXIS_BYTE_SWAP parameter:
//        * 1 (default): external wire is standard AXI-Stream little-endian byte
//          order (tdata[7:0] is wire byte 0).
//        * 0: external wire is big-endian / network order (tdata[31:24] is wire byte 0).
//      - Generates standard IEEE 802.3 60-byte minimum padded ARP reply frames
//        (15 32-bit words), preventing runt frame drops by Ethernet PHYs and switches.
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
    // Forwarded non-ARP ingress traffic to RTPS IP (connects to RTPS s_axis_wire)
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
    output wire                     s_arp_rx_tready,
    input  wire                      s_arp_rx_tlast,

    // -------------------------------------------------------------------------
    // 3. Network Configuration & Client Learning
    // -------------------------------------------------------------------------
    // Local FPGA IP and MAC address
    input  wire [31:0]               arp_reply_ip,
    input  wire [47:0]               arp_reply_mac,

    // Expected remote client IP (e.g. laptop). If 0, learns from any valid client
    input  wire [31:0]               remote_client_ip,

    // Dynamically learned MAC address of the connected remote client
    output reg  [47:0]               remote_client_mac
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
    // Part 1: Incoming Ethernet Frame Parsing & ARP Request Detection
    // =========================================================================
    wire [31:0] rx_net_word = AXIS_BYTE_SWAP ? byte_swap32(s_arp_rx_tdata) : s_arp_rx_tdata;
    wire        rx_handshake = s_arp_rx_tvalid && s_arp_rx_tready;

    reg [3:0]  rx_wire_beat;
    reg        rx_is_arp_frame;
    reg        rx_is_arp_req;
    reg        rx_target_ip_match;
    reg [47:0] rx_sha_mac;
    reg [31:0] rx_spa_ip;
    reg [15:0] rx_tpa_hi;
    reg [31:0] matched_spa_ip;

    reg        arp_reply_trigger;
    reg [47:0] latched_reply_dst_mac;
    reg [31:0] latched_reply_dst_ip;
    reg [31:0] latched_reply_spa_ip;

    // Filter flag: set when the current incoming frame is confirmed to be ARP
    reg rx_filtering_arp;
    reg rx_drop_fifo;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_wire_beat          <= 4'd0;
            rx_is_arp_frame       <= 1'b0;
            rx_is_arp_req         <= 1'b0;
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
            rx_drop_fifo          <= 1'b0;
        end else begin
            arp_reply_trigger <= 1'b0;
            rx_drop_fifo      <= 1'b0;

            if (rx_handshake) begin
                case (rx_wire_beat)
                    4'd0: begin
                        // Beat 0: Ethernet Dst MAC [47:16]
                        rx_is_arp_frame    <= 1'b0;
                        rx_is_arp_req      <= 1'b0;
                        rx_target_ip_match <= 1'b0;
                        rx_filtering_arp   <= 1'b0;
                    end

                    4'd1: begin
                        // Beat 1: Dst MAC [15:0] (bits [31:16]) + Ethernet Src MAC [47:32] (bits [15:0])
                    end

                    4'd2: begin
                        // Beat 2: Ethernet Src MAC [31:0]
                    end

                    4'd3: begin
                        // Beat 3: EtherType (bits [31:16]) + Hardware Type HTYPE (bits [15:0])
                        // ARP EtherType = 0x0806, Ethernet HTYPE = 0x0001
                        if (rx_net_word[31:16] == 16'h0806 && rx_net_word[15:0] == 16'h0001) begin
                            rx_is_arp_frame  <= 1'b1;
                            rx_filtering_arp <= 1'b1;
                            rx_drop_fifo     <= 1'b1; // Flush pre-buffered beats from RTPS egress FIFO
                        end else begin
                            rx_is_arp_frame  <= 1'b0;
                            rx_filtering_arp <= 1'b0;
                        end
                    end

                    4'd4: begin
                        // Beat 4: Protocol Type PTYPE (bits [31:16]) + HLEN (bits [15:8]) + PLEN (bits [7:0])
                        // IPv4 = 0x0800, HLEN = 6 (MAC len), PLEN = 4 (IPv4 len)
                        if (rx_is_arp_frame &&
                            rx_net_word[31:16] == 16'h0800 &&
                            rx_net_word[15:8]  == 8'h06    &&
                            rx_net_word[7:0]   == 8'h04) begin
                            // Header valid
                        end else begin
                            rx_is_arp_frame <= 1'b0;
                        end
                    end

                    4'd5: begin
                        // Beat 5: Operation OPER (bits [31:16]) + SHA [47:32] (bits [15:0])
                        // OPER: 0x0001 = ARP Request, 0x0002 = ARP Reply
                        if (rx_is_arp_frame && rx_net_word[31:16] == 16'h0001) begin
                            rx_is_arp_req     <= 1'b1;
                            rx_sha_mac[47:32] <= rx_net_word[15:0];
                        end else begin
                            rx_is_arp_frame <= 1'b0;
                            rx_is_arp_req   <= 1'b0;
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
                        // Beats 11..15: Frame padding (RFC 826 / IEEE 802.3 60-byte Ethernet padding)
                    end
                endcase

                if (s_arp_rx_tlast) begin
                    rx_wire_beat     <= 4'd0;
                    rx_filtering_arp <= 1'b0;

                    // Evaluate match on end of frame (supports both unpadded and padded ARP frames)
                    if (rx_is_arp_req &&
                        ((rx_wire_beat >= 4'd10 && rx_target_ip_match) ||
                         (rx_wire_beat == 4'd10 && ({rx_tpa_hi, rx_net_word[31:16]} == arp_reply_ip ||
                                                    (ARP_REPLY_IP_2 != 32'd0 && {rx_tpa_hi, rx_net_word[31:16]} == ARP_REPLY_IP_2))))) begin
                        arp_reply_trigger     <= 1'b1;
                        latched_reply_dst_mac <= rx_sha_mac;
                        latched_reply_dst_ip  <= rx_spa_ip;
                        latched_reply_spa_ip  <= (rx_wire_beat == 4'd10 && ARP_REPLY_IP_2 != 32'd0 && {rx_tpa_hi, rx_net_word[31:16]} == ARP_REPLY_IP_2) ?
                                                  ARP_REPLY_IP_2 :
                                                 (rx_target_ip_match ? matched_spa_ip : arp_reply_ip);

                        // Learn remote client MAC: if remote_client_ip == 0, learn from any sender;
                        // otherwise learn only when sender IP matches remote_client_ip
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
    // Part 2: Elastic Ingress Buffer (Passes Non-ARP traffic to RTPS IP)
    // =========================================================================
    // A 16-word circular FIFO buffers the first 4 beats of each incoming frame.
    // If beat 3 reveals EtherType == 0x0806 (ARP), the buffer is flushed
    // immediately without emitting any beats to m_rtps_tx.
    // If EtherType != 0x0806 (e.g. UDP/IPv4), the FIFO drains to m_rtps_tx.
    localparam FIFO_DEPTH = 16;
    reg [DATA_WIDTH-1:0] fifo_data [0:FIFO_DEPTH-1];
    reg                  fifo_last [0:FIFO_DEPTH-1];
    reg [3:0]            fifo_wr_ptr;
    reg [3:0]            fifo_rd_ptr;
    reg [4:0]            fifo_count;
    reg                  rx_passing_pkt;

    wire fifo_push = s_arp_rx_tvalid && s_arp_rx_tready && !rx_filtering_arp && !rx_drop_fifo;
    wire fifo_pop  = m_rtps_tx_tvalid && m_rtps_tx_tready;

    // Upstream ready: absorb line-rate ARP immediately; otherwise backpressure when FIFO fills
    assign s_arp_rx_tready = rx_filtering_arp ? 1'b1 : (fifo_count < (FIFO_DEPTH - 2));

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
                // ARP frame detected: erase pre-buffered beats from FIFO
                fifo_wr_ptr    <= fifo_rd_ptr;
                fifo_count     <= 5'd0;
                rx_passing_pkt <= 1'b0;
            end else begin
                // Activate pass-through once confirmed non-ARP or on early short frame termination
                if (rx_handshake) begin
                    if (rx_wire_beat == 4'd3 && (rx_net_word[31:16] != 16'h0806 || rx_net_word[15:0] != 16'h0001)) begin
                        rx_passing_pkt <= 1'b1;
                    end else if (s_arp_rx_tlast && rx_wire_beat < 4'd3) begin
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
    // Layout in Network Big-Endian order (60-byte IEEE 802.3 standard Ethernet frame):
    // Words 0..3:  Ethernet Header (14 bytes) + HTYPE (2 bytes)
    // Words 4..10: ARP Payload (26 bytes)
    // Words 11..14:Zero Padding (16 bytes) -> Total 60 bytes (15 beats)
    reg [3:0]  tx_beat;
    reg [31:0] arp_reply_word;

    always @(*) begin
        case (tx_beat)
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
    // Part 4: Wire TX Multiplexer & Arbiter (RTPS Egress vs ARP Reply)
    // =========================================================================
    localparam [1:0]
        TX_IDLE     = 2'd0,
        TX_ARP_RESP = 2'd1,
        TX_RTPS_FWD = 2'd2;

    reg [1:0] tx_state;
    reg       arp_reply_pending;

    // Track pending reply requests
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arp_reply_pending <= 1'b0;
        end else begin
            if (arp_reply_trigger) begin
                arp_reply_pending <= 1'b1;
            end else if (tx_state == TX_ARP_RESP && m_arp_tx_tready && (tx_beat == 4'd14)) begin
                arp_reply_pending <= 1'b0;
            end
        end
    end

    // Arbiter FSM
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state <= TX_IDLE;
            tx_beat  <= 4'd0;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    if (arp_reply_pending) begin
                        tx_state <= TX_ARP_RESP;
                        tx_beat  <= 4'd0;
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
                        if (tx_beat == 4'd14) begin
                            tx_beat  <= 4'd0;
                            tx_state <= TX_IDLE;
                        end else begin
                            tx_beat <= tx_beat + 4'd1;
                        end
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
        case (tx_state)
            TX_ARP_RESP: begin
                mux_arp_tx_tvalid  = 1'b1;
                mux_arp_tx_tdata   = AXIS_BYTE_SWAP ? byte_swap32(arp_reply_word) : arp_reply_word;
                mux_arp_tx_tlast   = (tx_beat == 4'd14);
                mux_rtps_rx_tready = 1'b0; // Backpressure RTPS while transmitting ARP reply
            end

            TX_RTPS_FWD: begin
                mux_arp_tx_tvalid  = s_rtps_rx_tvalid;
                mux_arp_tx_tdata   = s_rtps_rx_tdata;
                mux_arp_tx_tlast   = s_rtps_rx_tlast;
                mux_rtps_rx_tready = m_arp_tx_tready;
            end

            default: begin // TX_IDLE
                if (arp_reply_pending) begin
                    mux_arp_tx_tvalid  = 1'b0;
                    mux_arp_tx_tdata   = 32'd0;
                    mux_arp_tx_tlast   = 1'b0;
                    mux_rtps_rx_tready = 1'b0;
                end else begin
                    mux_arp_tx_tvalid  = s_rtps_rx_tvalid;
                    mux_arp_tx_tdata   = s_rtps_rx_tdata;
                    mux_arp_tx_tlast   = s_rtps_rx_tlast;
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
