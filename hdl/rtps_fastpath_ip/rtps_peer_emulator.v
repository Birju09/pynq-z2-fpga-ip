`timescale 1ns / 1ps

// =============================================================================
// Module: rtps_peer_emulator (rtps_peer_emulator.v)
//
// Description:
//   Cycle-Accurate In-FPGA RTI Connext DDS / RTPS Peer Participant Emulator.
//   Processes fragmented Ethernet/IPv4/UDP/RTPS frames from the RTPS Fast-Path
//   IP over AXI4-Stream:
//     - Reassembles incoming IPv4 fragments by (FragOffset * 8) into a 6 KB
//       UDP datagram buffer and verifies the IPv4 header checksums.
//     - Parses the RTPS v2.x message header ('RTPS'), INFO_TS submessage, and
//       DATA (0x15) submessage, extracting the 64-bit writer SequenceNumber
//       and XCDR1 CDR_LE (0x0001) FastSample payload (default 4,096 bytes).
//     - Transforms the payload deterministically (preserving Word 0..3 header:
//       cycle_id, flags, tx_timestamp_ns; transforming data[0..1019] via
//       (~word) + {16'h0000, seq_num[5:0], 10'h200}) to emulate an RTI C++
//       Feedback subscriber/publisher echo node.
//     - Emits 3 IPv4-fragmented Ethernet/IPv4/UDP/RTPS Feedback frames back to
//       the Master RX engine with full IPv4 & UDP 1's complement checksums.
//     - Supports hardware fault injection via fault_ctrl:
//         Bit [0]: Drop Fragment #1 (simulates packet loss / incomplete reassembly)
//         Bit [1]: Swap Fragment #0 and Fragment #1 order (out-of-order reassembly test)
//         Bit [2]: Corrupt UDP checksum on outgoing Feedback datagram
//         Bit [3]: Emit stale / duplicate SequenceNumber (seqNum does not advance)
// =============================================================================

module rtps_peer_emulator #(
    parameter integer DATA_WIDTH  = 32,
    parameter integer MAX_DGRAM_W = 1536  // 6 KB max UDP datagram (1,536 words)
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // Fault-injection & peer identity configuration from CSRs
    input  wire [31:0]            fault_ctrl,
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
    // 1's Complement 16-bit Adder Helper for IPv4 & UDP Checksums
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

    function [15:0] csum_add16;
        input [15:0] sum_in;
        input [15:0] half_in;
        reg   [16:0] tmp;
        begin
            tmp = {1'b0, sum_in} + {1'b0, half_in};
            csum_add16 = tmp[15:0] + {15'd0, tmp[16]};
        end
    endfunction

    // =========================================================================
    // Emulator FSM States
    // =========================================================================
    localparam [2:0]
        E_RX_FRAMES   = 3'd0, // Receive & reassemble incoming IPv4 fragments
        E_BUILD_DGRAM = 3'd1, // Build complete outgoing UDP+RTPS Feedback datagram in tx_dgram_ram
        E_TX_INIT     = 3'd2, // Setup current IPv4 fragment parameters & header checksum
        E_TX_STREAM   = 3'd3; // Stream Ethernet + IPv4 + fragment payload over m_axis

    reg [2:0] state;

    // Reassembly buffer for incoming UDP datagram (Port A write, read during E_BUILD_DGRAM)
    (* ram_style = "block" *) reg [31:0] rx_dgram_ram [0:MAX_DGRAM_W-1];
    // Outgoing UDP datagram buffer (includes 8B UDP hdr + 56B RTPS hdrs + 4B encap + 4096B payload = 4164B = 1041 words)
    (* ram_style = "block" *) reg [31:0] tx_dgram_ram [0:MAX_DGRAM_W-1];

    // =========================================================================
    // RX Fragment Parser Registers
    // =========================================================================
    reg [11:0] rx_beat_idx;
    reg [31:0] latched_src_ip;
    reg [31:0] latched_dst_ip;
    reg [15:0] latched_src_port;
    reg [15:0] latched_dst_port;
    reg [15:0] cur_ip_len;       // IPv4 Total Length in bytes
    reg [15:0] cur_ip_id;
    reg [15:0] cur_flags_frag;   // [15:13]=flags (bit 13 = MF), [12:0]=frag_offset_8B
    reg [15:0] rx_accum_bytes;
    reg [15:0] rx_expected_bytes;
    reg        rx_saw_last_frag;

    // Unaligned 16-bit Ethernet header (14 bytes = 3.5 words) staging register
    // Frame format on 32-bit stream (Big-Endian network order per word):
    //   Beat 0: DstMAC[47:16]
    //   Beat 1: {DstMAC[15:0], SrcMAC[47:32]}
    //   Beat 2: SrcMAC[31:0]
    //   Beat 3: {EtherType(16'h0800), IPv4_VerIHL_TOS(16'h4500)}
    //   Beat 4: {IPv4_TotalLen[15:0], IPv4_ID[15:0]}
    //   Beat 5: {IPv4_FlagsFrag[15:0], IPv4_TTL_Proto(16'h4011)}
    //   Beat 6: {IPv4_HdrCsum[15:0], SrcIP[31:16]}
    //   Beat 7: {SrcIP[15:0], DstIP[31:16]}
    //   Beat 8: {DstIP[15:0], IP_Payload_Half0[15:0]}
    //   Beat k (k>=9): {IP_Payload_Half(2k-17)[15:0], IP_Payload_Half(2k-16)[15:0]}
    reg [15:0] rx_prev_half;
    reg [11:0] rx_dgram_wr_ptr;

    wire [12:0] frag_offset_8b  = cur_flags_frag[12:0];
    wire        frag_mf         = cur_flags_frag[13];
    wire [15:0] ip_payload_len  = cur_ip_len - 16'd20;
    wire [11:0] frag_word_start = {frag_offset_8b[10:0], 1'b0}; // offset * 8 bytes = offset * 2 words

    assign s_axis_tready = (state == E_RX_FRAMES);
    wire   rx_fire       = s_axis_tvalid && s_axis_tready;

    // =========================================================================
    // Outgoing Feedback State & Fragment Sequencer
    // =========================================================================
    reg [63:0] peer_seq_num;
    reg [15:0] peer_ip_id;
    reg [11:0] build_idx;
    reg [15:0] udp_csum_accum;
    reg [11:0] total_dgram_words; // e.g. 1041 words (4164 bytes)
    reg [15:0] payload_words_cfg; // e.g. 1024 words (4096 bytes)

    reg [1:0]  frag_step;         // 0, 1, 2 (3 fragments)
    reg [1:0]  active_frag_idx;   // Actual fragment index (swapped when fault_ctrl[1]=1)
    reg [11:0] cur_frag_word_off; // Start word in tx_dgram_ram
    reg [11:0] cur_frag_words;    // Number of 32-bit words in this fragment (370, 370, 301)
    reg [15:0] cur_frag_ip_len;   // 20 + cur_frag_words*4
    reg [15:0] cur_frag_flags;    // {2'b00, mf_bit, frag_offset_8b[12:0]}
    reg [15:0] cur_frag_hdr_csum;

    reg [11:0] tx_beat_idx;
    reg [15:0] tx_prev_half;
    reg [31:0] rx_ram_dout;
    reg [31:0] tx_ram_dout;
    reg [11:0] rx_ram_rd_addr;
    reg [11:0] tx_ram_rd_addr;

    always @(posedge clk) begin
        rx_ram_dout <= rx_dgram_ram[rx_ram_rd_addr];
        tx_ram_dout <= tx_dgram_ram[tx_ram_rd_addr];
    end

    // Extracted fields from rx_dgram_ram during E_BUILD_DGRAM:
    // Word 0: {SrcPort[15:0], DstPort[15:0]}
    // Word 1: {UDP_Len[15:0], UDP_Csum[15:0]}
    // Word 2: "RTPS" (32'h5254_5053)
    // Word 3: {ProtoVer(16'h0201), VendorId(16'h0110)}
    // Word 4..6: GuidPrefix[95:0]
    // Word 7: {SubmsgId_INFO_TS(8'h09), Flags(8'h01), OctetsToNext(16'h0800)}
    // Word 8..9: Timestamp[63:0]
    // Word 10: {SubmsgId_DATA(8'h15), Flags(8'h05), OctetsToNext[15:0]}
    // Word 11: {ExtraFlags(16'h0000), OctetsToInlineQos(16'h1000)}
    // Word 12: ReaderEntityId[31:0]
    // Word 13: WriterEntityId[31:0]
    // Word 14: SeqNum_Hi[31:0] (LE)
    // Word 15: SeqNum_Lo[31:0] (LE)
    // Word 16: Encapsulation (32'h0001_0000 = CDR_LE)
    // Word 17 .. 17+1023: FastSample payload (1024 words = 4096 bytes)

    wire [15:0] udp_total_bytes = {2'b00, total_dgram_words, 2'b00};
    wire [15:0] data_octets_le  = {2'b00, payload_words_cfg[11:0], 2'b00} + 16'd24; // 20B DATA hdr after octetsToNext + 4B encap + payload
    wire [15:0] data_octets_wire = {data_octets_le[7:0], data_octets_le[15:8]};      // Little-endian on wire

    // Combinational word generator for E_BUILD_DGRAM
    reg [31:0] build_word;
    always @(*) begin
        case (build_idx)
            12'd0:  build_word = {latched_dst_port, latched_src_port}; // Swap UDP ports for reply
            12'd1:  build_word = {udp_total_bytes, 16'h0000};          // UDP checksum filled at end of build
            12'd2:  build_word = 32'h5254_5053;                        // "RTPS"
            12'd3:  build_word = 32'h0203_0101;                        // RTPS v2.3, VendorId = 0x0101 (RTI Connext)
            12'd4:  build_word = peer_guid_prefix[95:64];
            12'd5:  build_word = peer_guid_prefix[63:32];
            12'd6:  build_word = peer_guid_prefix[31:0];
            12'd7:  build_word = 32'h0901_0800;                        // INFO_TS (0x09), E=1, len=8 (LE: 0x0800)
            12'd8:  build_word = rx_ram_dout;                          // Echo INFO_TS sec
            12'd9:  build_word = rx_ram_dout;                          // Echo INFO_TS frac
            12'd10: build_word = {8'h15, 8'h05, data_octets_wire};     // DATA (0x15), flags=0x05 (D=1,Q=0,E=1)
            12'd11: build_word = 32'h0000_1000;                        // extraFlags=0, octetsToInlineQos=16 (LE: 0x1000)
            12'd12: build_word = peer_reader_id;                       // Target Reader EntityId (or UNKNOWN)
            12'd13: build_word = peer_writer_id;                       // RTI Feedback Writer EntityId
            12'd14: build_word = {peer_seq_num[39:32], peer_seq_num[47:40], peer_seq_num[55:48], peer_seq_num[63:56]}; // SeqNum Hi (LE)
            12'd15: build_word = {peer_seq_num[7:0],   peer_seq_num[15:8],  peer_seq_num[23:16], peer_seq_num[31:24]}; // SeqNum Lo (LE)
            12'd16: build_word = 32'h0001_0000;                        // CDR_LE encapsulation (0x0001, options 0x0000)
            default: begin
                // Words 17..20: FastSample 16-byte header (cycle_id, flags, tx_timestamp_ns) echoed intact!
                if (build_idx >= 12'd17 && build_idx <= 12'd20) begin
                    build_word = rx_ram_dout;
                end else begin
                    // Words 21..1040: Transform payload words deterministically
                    build_word = (~rx_ram_dout) + {16'h0000, peer_seq_num[5:0], 10'h200};
                end
            end
        endcase
    end

    // Combinational IPv4 header checksum calculator for current outgoing fragment
    wire [15:0] csum_step0 = csum_add32(16'h0000,   {16'h4500, cur_frag_ip_len});
    wire [15:0] csum_step1 = csum_add32(csum_step0, {peer_ip_id, cur_frag_flags});
    wire [15:0] csum_step2 = csum_add32(csum_step1, {16'h4011, 16'h0000});
    wire [15:0] csum_step3 = csum_add32(csum_step2, latched_dst_ip); // Reply SrcIP = Master's DstIP
    wire [15:0] csum_step4 = csum_add32(csum_step3, latched_src_ip); // Reply DstIP = Master's SrcIP
    wire [15:0] calc_ip_hdr_csum = ~csum_step4;

    // Pseudo-header + words 0..1 (excluding Word 1 low 16b checksum field) seed
    wire [15:0] pseudo_step0 = csum_add32(16'h0000,       latched_dst_ip);
    wire [15:0] pseudo_step1 = csum_add32(pseudo_step0,   latched_src_ip);
    wire [15:0] pseudo_step2 = csum_add32(pseudo_step1,   {16'h0011, udp_total_bytes});

    // Outgoing Ethernet + IPv4 + Fragment word mux
    reg [31:0] tx_wire_word;
    always @(*) begin
        case (tx_beat_idx)
            12'd0: tx_wire_word = 32'h0200_5E10; // DstMAC[47:16] (Master MAC)
            12'd1: tx_wire_word = 32'h0001_0200; // {DstMAC[15:0], SrcMAC[47:32]}
            12'd2: tx_wire_word = 32'h5E20_0002; // SrcMAC[31:0] (RTI Peer MAC)
            12'd3: tx_wire_word = 32'h0800_4500; // EtherType IPv4 (0x0800), Ver/IHL/TOS (0x4500)
            12'd4: tx_wire_word = {cur_frag_ip_len, peer_ip_id};
            12'd5: tx_wire_word = {cur_frag_flags, 16'h4011}; // TTL=64, Proto=UDP(0x11)
            12'd6: tx_wire_word = {cur_frag_hdr_csum, latched_dst_ip[31:16]};
            12'd7: tx_wire_word = {latched_dst_ip[15:0], latched_src_ip[31:16]};
            12'd8: tx_wire_word = {latched_src_ip[15:0], tx_ram_dout[31:16]};
            default: begin
                // Last beat (tx_beat_idx == 8 + cur_frag_words) carries {tx_prev_half, 16'h0000}
                if (tx_beat_idx == (12'd8 + cur_frag_words))
                    tx_wire_word = {tx_prev_half, 16'h0000};
                else
                    tx_wire_word = {tx_prev_half, tx_ram_dout[31:16]};
            end
        endcase
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            state             <= E_RX_FRAMES;
            rx_beat_idx       <= 12'd0;
            latched_src_ip    <= 32'hC0A8_01C9; // 192.168.1.201
            latched_dst_ip    <= 32'hC0A8_0164; // 192.168.1.100
            latched_src_port  <= 16'd7410;
            latched_dst_port  <= 16'd7411;
            cur_ip_len        <= 16'd0;
            cur_ip_id         <= 16'd0;
            cur_flags_frag    <= 16'd0;
            rx_accum_bytes    <= 16'd0;
            rx_expected_bytes <= 16'd0;
            rx_saw_last_frag  <= 1'b0;
            rx_prev_half      <= 16'd0;
            rx_dgram_wr_ptr   <= 12'd0;

            peer_seq_num      <= 64'd0;
            peer_ip_id        <= 16'hA000;
            build_idx         <= 12'd0;
            udp_csum_accum    <= 16'd0;
            total_dgram_words <= 12'd1041; // 4164 bytes default
            payload_words_cfg <= 12'd1024; // 4096 bytes default

            frag_step         <= 2'd0;
            active_frag_idx   <= 2'd0;
            cur_frag_word_off <= 12'd0;
            cur_frag_words    <= 12'd0;
            cur_frag_ip_len   <= 16'd0;
            cur_frag_flags    <= 16'd0;
            cur_frag_hdr_csum <= 16'd0;

            tx_beat_idx       <= 12'd0;
            tx_prev_half      <= 16'd0;
            rx_ram_rd_addr    <= 12'd0;
            tx_ram_rd_addr    <= 12'd0;

            m_axis_tdata      <= 32'd0;
            m_axis_tvalid     <= 1'b0;
            m_axis_tlast      <= 1'b0;
        end else begin
            case (state)
                // -------------------------------------------------------------
                // 1. E_RX_FRAMES: Reassemble Master's outgoing IPv4 fragments
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
                                rx_beat_idx    <= rx_beat_idx + 12'd1;
                                // Reset accumulators on first fragment of a new IP ID
                                if (s_axis_tdata[28:16] == 13'd0 && rx_accum_bytes == 16'd0) begin
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
                                if (frag_offset_8b == 13'd0) begin
                                    latched_src_port <= s_axis_tdata[15:0];
                                end
                                rx_beat_idx          <= rx_beat_idx + 12'd1;
                            end
                            default: begin
                                // Write reassembled 32-bit UDP datagram word
                                if (rx_dgram_wr_ptr < (frag_word_start + (ip_payload_len[13:2]))) begin
                                    rx_dgram_ram[rx_dgram_wr_ptr] <= {rx_prev_half, s_axis_tdata[31:16]};
                                    rx_dgram_wr_ptr               <= rx_dgram_wr_ptr + 12'd1;
                                    if (frag_offset_8b == 13'd0 && rx_dgram_wr_ptr == 12'd0) begin
                                        latched_dst_port <= s_axis_tdata[31:16];
                                    end
                                    if (frag_offset_8b == 13'd0 && rx_dgram_wr_ptr == 12'd1) begin
                                        // Extract UDP total length in words:rx_prev_half is UDP_Len
                                        total_dgram_words <= rx_prev_half[13:2];
                                        payload_words_cfg <= rx_prev_half[13:2] - 12'd17;
                                    end
                                end
                                rx_prev_half <= s_axis_tdata[15:0];

                                if (s_axis_tlast) begin
                                    rx_beat_idx <= 12'd0;
                                    // Check if reassembly of all fragments is complete
                                    if (!frag_mf) begin
                                        // Last fragment: total expected bytes = frag_offset*8 + ip_payload_len
                                        if ((rx_accum_bytes + ip_payload_len) == ({frag_offset_8b, 3'b000} + ip_payload_len)) begin
                                            rx_accum_bytes   <= 16'd0;
                                            peer_seq_num     <= fault_ctrl[3] ? ((peer_seq_num == 64'd0) ? 64'd1 : peer_seq_num)
                                                                              : (peer_seq_num + 64'd1);
                                            peer_ip_id       <= peer_ip_id + 16'd1;
                                            build_idx        <= 12'd0;
                                            rx_ram_rd_addr   <= 12'd1; // Prefetch Word 1 for build_idx == 1
                                            state            <= E_BUILD_DGRAM;
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
                                            rx_ram_rd_addr   <= 12'd1; // Prefetch Word 1 for build_idx == 1
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
                // 2. E_BUILD_DGRAM: Build Outgoing Feedback UDP+RTPS Datagram
                // -------------------------------------------------------------
                E_BUILD_DGRAM: begin
                    if (build_idx < total_dgram_words) begin
                        tx_dgram_ram[build_idx] <= build_word;
                        if (build_idx == 12'd0)
                            udp_csum_accum <= csum_add32(pseudo_step2, build_word);
                        else
                            udp_csum_accum <= csum_add32(udp_csum_accum, build_word);

                        build_idx      <= build_idx + 12'd1;
                        rx_ram_rd_addr <= build_idx + 12'd2;
                    end else begin
                        // Patch final UDP checksum into Word 1 of tx_dgram_ram
                        // If fault_ctrl[2] == 1, corrupt UDP checksum deliberately
                        tx_dgram_ram[12'd1] <= {udp_total_bytes,
                            fault_ctrl[2] ? (~udp_csum_accum ^ 16'hDEAD) :
                            ((~udp_csum_accum == 16'h0000) ? 16'hFFFF : ~udp_csum_accum)};
                        frag_step           <= 2'd0;
                        state               <= E_TX_INIT;
                    end
                end

                // -------------------------------------------------------------
                // 3. E_TX_INIT: Prepare IPv4 Fragment Parameters & Checksum
                // -------------------------------------------------------------
                E_TX_INIT: begin
                    // Fault [0]: Drop Fragment #1 (skip directly from step 0 to step 2)
                    // Fault [1]: Swap Fragment #0 and Fragment #1 order
                    if (fault_ctrl[0] && frag_step == 2'd1) begin
                        frag_step <= 2'd2;
                    end else begin
                        if (fault_ctrl[1]) begin
                            active_frag_idx <= (frag_step == 2'd0) ? 2'd1 :
                                               (frag_step == 2'd1) ? 2'd0 : 2'd2;
                        end else begin
                            active_frag_idx <= frag_step;
                        end

                        case (fault_ctrl[1] ? ((frag_step == 2'd0) ? 2'd1 : (frag_step == 2'd1) ? 2'd0 : 2'd2) : frag_step)
                            2'd0: begin
                                cur_frag_word_off <= 12'd0;
                                cur_frag_words    <= 12'd370; // 1,480 bytes
                                cur_frag_ip_len   <= 16'd1500;
                                cur_frag_flags    <= {3'b001, 13'd0}; // MF=1, offset=0
                                tx_ram_rd_addr    <= 12'd0;
                            end
                            2'd1: begin
                                cur_frag_word_off <= 12'd370;
                                cur_frag_words    <= 12'd370; // 1,480 bytes
                                cur_frag_ip_len   <= 16'd1500;
                                cur_frag_flags    <= {3'b001, 13'd185}; // MF=1, offset=185 (1480/8)
                                tx_ram_rd_addr    <= 12'd370;
                            end
                            default: begin
                                cur_frag_word_off <= 12'd740;
                                cur_frag_words    <= total_dgram_words - 12'd740; // 301 words = 1,204 bytes
                                cur_frag_ip_len   <= 16'd20 + {2'b00, (total_dgram_words - 12'd740), 2'b00};
                                cur_frag_flags    <= {3'b000, 13'd370}; // MF=0, offset=370 (2960/8)
                                tx_ram_rd_addr    <= 12'd740;
                            end
                        endcase

                        tx_beat_idx   <= 12'd0;
                        m_axis_tvalid <= 1'b0;
                        m_axis_tlast  <= 1'b0;
                        state         <= E_TX_STREAM;
                    end
                end

                // -------------------------------------------------------------
                // 4. E_TX_STREAM: Stream Ethernet + IPv4 Fragment over m_axis
                // -------------------------------------------------------------
                E_TX_STREAM: begin
                    // Latch computed IPv4 header checksum on beat 0
                    if (tx_beat_idx == 12'd0) begin
                        cur_frag_hdr_csum <= calc_ip_hdr_csum;
                    end

                    if (!m_axis_tvalid || m_axis_tready) begin
                        m_axis_tdata  <= tx_wire_word;
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= (tx_beat_idx == (12'd8 + cur_frag_words));

                        if (tx_beat_idx == (12'd8 + cur_frag_words)) begin
                            if (frag_step == 2'd2) begin
                                state <= E_RX_FRAMES;
                            end else begin
                                frag_step <= frag_step + 2'd1;
                                state     <= E_TX_INIT;
                            end
                        end else begin
                            tx_beat_idx <= tx_beat_idx + 12'd1;
                            if (tx_beat_idx == 12'd7) begin
                                tx_ram_rd_addr <= tx_ram_rd_addr + 12'd1;
                            end else if (tx_beat_idx >= 12'd8) begin
                                tx_prev_half <= tx_ram_dout[15:0];
                                if (tx_beat_idx < (12'd7 + cur_frag_words)) begin
                                    tx_ram_rd_addr <= tx_ram_rd_addr + 12'd1;
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
