`timescale 1ns / 1ps

// =============================================================================
// Module: ecat_40slave_emulator (ecat_40slave_emulator.v)
//
// Description:
//   Cycle-Accurate In-FPGA 40-Slave, 5 KB (5,120-Byte) EtherCAT Ring Emulator.
//   Processes EtherCAT frames (EtherType 0x88A4) on-the-fly over AXI4-Stream:
//     - Logical Address Space: 0x0000_0000 .. 0x0000_13FF (5,120 bytes)
//     - Slave Partitioning:    40 Slaves x 128 Bytes (32 words) per slave
//                              slave_id = cur_byte_addr[12:7] (0 .. 39)
//     - On-the-Fly Processing:
//         1. Inspects EtherCAT Datagram Header (CMD, LOGICAL_ADDR, LEN).
//         2. For each PDO word, maps logical address to slave_id (0..39).
//         3. If slave_online_mask[slave_id] == 1:
//              - Writes outgoing Master word (RxPDO) into Slave BRAM.
//              - Replaces wire word with Slave's deterministic TxPDO response:
//                (~s_axis_tdata) + {16'h0000, slave_id[5:0], 10'h100}
//              - Marks slave_visited[slave_id] = 1 and adds WKC delta (+3 for
//                LRW 0x0C, +1 for LRD 0x0A, +2 for LWR 0x0B) once per slave.
//         4. Replaces trailing Working Counter (WKC) beat with updated WKC.
//         5. Recalculates and outputs final 32-bit Ethernet FCS (CRC-32) beat.
// =============================================================================

module ecat_40slave_emulator #(
    parameter integer DATA_WIDTH   = 32,
    parameter integer NUM_SLAVES   = 40,
    parameter integer SLAVE_BYTES  = 128,  // 128 bytes (32 words) per slave
    parameter integer TOTAL_WORDS  = 2048  // 8 KB max BRAM backing store
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // Configuration from PS CSRs (for fault-injection testing)
    input  wire [NUM_SLAVES-1:0]  slave_online_mask,

    // Ingress AXI4-Stream (from Master TX port)
    input  wire [DATA_WIDTH-1:0]  s_axis_tdata,
    input  wire                   s_axis_tvalid,
    output wire                   s_axis_tready,
    input  wire                   s_axis_tlast,

    // Egress AXI4-Stream (to Master RX port)
    output reg  [DATA_WIDTH-1:0]  m_axis_tdata,
    output reg                    m_axis_tvalid,
    input  wire                   m_axis_tready,
    output reg                    m_axis_tlast
);

    // =========================================================================
    // EtherCAT Command Codes
    // =========================================================================
    localparam [7:0]
        ECAT_CMD_LRD = 8'h0A, // Logical Read        (WKC +1 per slave)
        ECAT_CMD_LWR = 8'h0B, // Logical Write       (WKC +2 per slave)
        ECAT_CMD_LRW = 8'h0C; // Logical Read/Write  (WKC +3 per slave)

    // =========================================================================
    // Internal Slave Process Data RAM (Stores Master->Slave outputs)
    // =========================================================================
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] slave_pdo_ram [0:TOTAL_WORDS-1];

    // =========================================================================
    // Standard IEEE 802.3 Ethernet FCS CRC-32 (2-LUT Parallel Matrix)
    // Polynomial: 0xEDB88320 (reflected / LSB-first)
    // Maximum logic depth: 2 LUT6 levels (sub-nanosecond propagation delay)
    // =========================================================================
    function [31:0] next_crc32;
        input [31:0] current_crc;
        input [31:0] data_in;
        reg   [31:0] d;
        begin
            d = current_crc ^ data_in;
            next_crc32[ 0] = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[7] ^ d[8] ^ d[16] ^ d[20] ^ d[22] ^ d[23] ^ d[26];
            next_crc32[ 1] = d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[7] ^ d[8] ^ d[9] ^ d[17] ^ d[21] ^ d[23] ^ d[24] ^ d[27];
            next_crc32[ 2] = d[0] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6] ^ d[8] ^ d[9] ^ d[10] ^ d[18] ^ d[22] ^ d[24] ^ d[25] ^ d[28];
            next_crc32[ 3] = d[1] ^ d[3] ^ d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[9] ^ d[10] ^ d[11] ^ d[19] ^ d[23] ^ d[25] ^ d[26] ^ d[29];
            next_crc32[ 4] = d[2] ^ d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[8] ^ d[10] ^ d[11] ^ d[12] ^ d[20] ^ d[24] ^ d[26] ^ d[27] ^ d[30];
            next_crc32[ 5] = d[0] ^ d[3] ^ d[5] ^ d[6] ^ d[7] ^ d[8] ^ d[9] ^ d[11] ^ d[12] ^ d[13] ^ d[21] ^ d[25] ^ d[27] ^ d[28] ^ d[31];
            next_crc32[ 6] = d[0] ^ d[2] ^ d[3] ^ d[9] ^ d[10] ^ d[12] ^ d[13] ^ d[14] ^ d[16] ^ d[20] ^ d[23] ^ d[28] ^ d[29];
            next_crc32[ 7] = d[1] ^ d[3] ^ d[4] ^ d[10] ^ d[11] ^ d[13] ^ d[14] ^ d[15] ^ d[17] ^ d[21] ^ d[24] ^ d[29] ^ d[30];
            next_crc32[ 8] = d[0] ^ d[2] ^ d[4] ^ d[5] ^ d[11] ^ d[12] ^ d[14] ^ d[15] ^ d[16] ^ d[18] ^ d[22] ^ d[25] ^ d[30] ^ d[31];
            next_crc32[ 9] = d[0] ^ d[2] ^ d[4] ^ d[5] ^ d[7] ^ d[8] ^ d[12] ^ d[13] ^ d[15] ^ d[17] ^ d[19] ^ d[20] ^ d[22] ^ d[31];
            next_crc32[10] = d[0] ^ d[2] ^ d[4] ^ d[5] ^ d[7] ^ d[9] ^ d[13] ^ d[14] ^ d[18] ^ d[21] ^ d[22] ^ d[26];
            next_crc32[11] = d[1] ^ d[3] ^ d[5] ^ d[6] ^ d[8] ^ d[10] ^ d[14] ^ d[15] ^ d[19] ^ d[22] ^ d[23] ^ d[27];
            next_crc32[12] = d[2] ^ d[4] ^ d[6] ^ d[7] ^ d[9] ^ d[11] ^ d[15] ^ d[16] ^ d[20] ^ d[23] ^ d[24] ^ d[28];
            next_crc32[13] = d[0] ^ d[3] ^ d[5] ^ d[7] ^ d[8] ^ d[10] ^ d[12] ^ d[16] ^ d[17] ^ d[21] ^ d[24] ^ d[25] ^ d[29];
            next_crc32[14] = d[0] ^ d[1] ^ d[4] ^ d[6] ^ d[8] ^ d[9] ^ d[11] ^ d[13] ^ d[17] ^ d[18] ^ d[22] ^ d[25] ^ d[26] ^ d[30];
            next_crc32[15] = d[1] ^ d[2] ^ d[5] ^ d[7] ^ d[9] ^ d[10] ^ d[12] ^ d[14] ^ d[18] ^ d[19] ^ d[23] ^ d[26] ^ d[27] ^ d[31];
            next_crc32[16] = d[1] ^ d[4] ^ d[7] ^ d[10] ^ d[11] ^ d[13] ^ d[15] ^ d[16] ^ d[19] ^ d[22] ^ d[23] ^ d[24] ^ d[26] ^ d[27] ^ d[28];
            next_crc32[17] = d[2] ^ d[5] ^ d[8] ^ d[11] ^ d[12] ^ d[14] ^ d[16] ^ d[17] ^ d[20] ^ d[23] ^ d[24] ^ d[25] ^ d[27] ^ d[28] ^ d[29];
            next_crc32[18] = d[0] ^ d[3] ^ d[6] ^ d[9] ^ d[12] ^ d[13] ^ d[15] ^ d[17] ^ d[18] ^ d[21] ^ d[24] ^ d[25] ^ d[26] ^ d[28] ^ d[29] ^ d[30];
            next_crc32[19] = d[0] ^ d[1] ^ d[4] ^ d[7] ^ d[10] ^ d[13] ^ d[14] ^ d[16] ^ d[18] ^ d[19] ^ d[22] ^ d[25] ^ d[26] ^ d[27] ^ d[29] ^ d[30] ^ d[31];
            next_crc32[20] = d[0] ^ d[3] ^ d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[11] ^ d[14] ^ d[15] ^ d[16] ^ d[17] ^ d[19] ^ d[22] ^ d[27] ^ d[28] ^ d[30] ^ d[31];
            next_crc32[21] = d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[12] ^ d[15] ^ d[17] ^ d[18] ^ d[22] ^ d[26] ^ d[28] ^ d[29] ^ d[31];
            next_crc32[22] = d[2] ^ d[7] ^ d[8] ^ d[13] ^ d[18] ^ d[19] ^ d[20] ^ d[22] ^ d[26] ^ d[27] ^ d[29] ^ d[30];
            next_crc32[23] = d[0] ^ d[3] ^ d[8] ^ d[9] ^ d[14] ^ d[19] ^ d[20] ^ d[21] ^ d[23] ^ d[27] ^ d[28] ^ d[30] ^ d[31];
            next_crc32[24] = d[2] ^ d[3] ^ d[6] ^ d[7] ^ d[8] ^ d[9] ^ d[10] ^ d[15] ^ d[16] ^ d[21] ^ d[23] ^ d[24] ^ d[26] ^ d[28] ^ d[29] ^ d[31];
            next_crc32[25] = d[1] ^ d[2] ^ d[6] ^ d[9] ^ d[10] ^ d[11] ^ d[17] ^ d[20] ^ d[23] ^ d[24] ^ d[25] ^ d[26] ^ d[27] ^ d[29] ^ d[30];
            next_crc32[26] = d[2] ^ d[3] ^ d[7] ^ d[10] ^ d[11] ^ d[12] ^ d[18] ^ d[21] ^ d[24] ^ d[25] ^ d[26] ^ d[27] ^ d[28] ^ d[30] ^ d[31];
            next_crc32[27] = d[0] ^ d[1] ^ d[2] ^ d[6] ^ d[7] ^ d[11] ^ d[12] ^ d[13] ^ d[16] ^ d[19] ^ d[20] ^ d[23] ^ d[25] ^ d[27] ^ d[28] ^ d[29] ^ d[31];
            next_crc32[28] = d[0] ^ d[4] ^ d[6] ^ d[12] ^ d[13] ^ d[14] ^ d[16] ^ d[17] ^ d[21] ^ d[22] ^ d[23] ^ d[24] ^ d[28] ^ d[29] ^ d[30];
            next_crc32[29] = d[0] ^ d[1] ^ d[5] ^ d[7] ^ d[13] ^ d[14] ^ d[15] ^ d[17] ^ d[18] ^ d[22] ^ d[23] ^ d[24] ^ d[25] ^ d[29] ^ d[30] ^ d[31];
            next_crc32[30] = d[3] ^ d[4] ^ d[7] ^ d[14] ^ d[15] ^ d[18] ^ d[19] ^ d[20] ^ d[22] ^ d[24] ^ d[25] ^ d[30] ^ d[31];
            next_crc32[31] = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[7] ^ d[15] ^ d[19] ^ d[21] ^ d[22] ^ d[25] ^ d[31];
        end
    endfunction

    // =========================================================================
    // Streaming Frame Parser State & Counters
    // =========================================================================
    reg [11:0]           beat_idx;
    reg [7:0]            latched_cmd;
    reg [15:0]           latched_payload_bytes;
    reg [11:0]           latched_payload_words;
    reg [31:0]           latched_logical_addr;
    reg [15:0]           wkc_accum;
    reg [NUM_SLAVES-1:0] slave_visited;
    reg [31:0]           fcs_accum;
    reg [31:0]           cur_byte_addr;

    // Ready whenever downstream can accept (or output register is empty)
    assign s_axis_tready = m_axis_tready || !m_axis_tvalid;
    wire   beat_fire     = s_axis_tvalid && s_axis_tready;

    // Registered byte address & slave mapping for current PDO beat
    wire [5:0]  cur_slave_id  = cur_byte_addr[12:7]; // 128 bytes per slave (2^7)
    wire [10:0] ram_word_addr = cur_byte_addr[12:2]; // 32-bit word index in 8KB space

    wire        valid_slave   = (cur_byte_addr < (NUM_SLAVES * SLAVE_BYTES)) &&
                                (cur_slave_id < NUM_SLAVES);
    wire        slave_online  = valid_slave && slave_online_mask[cur_slave_id];
    wire        first_hit     = slave_online && !slave_visited[cur_slave_id];

    wire [15:0] wkc_step      = (latched_cmd == ECAT_CMD_LRW) ? 16'd3 :
                                (latched_cmd == ECAT_CMD_LWR) ? 16'd2 :
                                (latched_cmd == ECAT_CMD_LRD) ? 16'd1 : 16'd0;

    wire [31:0] slave_tx_word = (~s_axis_tdata) + {16'h0000, cur_slave_id[5:0], 10'h100};

    // Determine outgoing word on the wire for current beat
    reg  [31:0] wire_out_word;
    always @(*) begin
        if (beat_idx < 12'd6) begin
            // Beats 0..5: Ethernet + EtherCAT + Datagram Headers pass through
            wire_out_word = s_axis_tdata;
        end else if (beat_idx < (12'd6 + latched_payload_words)) begin
            // Beats 6 .. 6+N-1: PDO Payload
            if (slave_online && (latched_cmd == ECAT_CMD_LRW || latched_cmd == ECAT_CMD_LRD))
                wire_out_word = slave_tx_word;
            else
                wire_out_word = s_axis_tdata;
        end else if (beat_idx == (12'd6 + latched_payload_words)) begin
            // Beat 6+N: Working Counter (WKC) trailer
            wire_out_word = {16'h0000, s_axis_tdata[15:0] + wkc_accum};
        end else begin
            // Beat 7+N (tlast): Ethernet FCS (CRC-32) over all preceding beats
            wire_out_word = ~fcs_accum;
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            beat_idx              <= 12'd0;
            latched_cmd           <= 8'd0;
            latched_payload_bytes <= 16'd0;
            latched_payload_words <= 12'd0;
            latched_logical_addr  <= 32'd0;
            cur_byte_addr         <= 32'd0;
            wkc_accum             <= 16'd0;
            slave_visited         <= {NUM_SLAVES{1'b0}};
            fcs_accum             <= 32'hFFFF_FFFF;
            m_axis_tdata          <= 32'd0;
            m_axis_tvalid         <= 1'b0;
            m_axis_tlast          <= 1'b0;
        end else begin
            if (beat_fire) begin
                m_axis_tdata  <= wire_out_word;
                m_axis_tvalid <= 1'b1;
                m_axis_tlast  <= s_axis_tlast;

                // Update Slave RAM during PDO payload beats
                if ((beat_idx >= 12'd6) &&
                    (beat_idx < (12'd6 + latched_payload_words)) &&
                    slave_online &&
                    (latched_cmd == ECAT_CMD_LRW || latched_cmd == ECAT_CMD_LWR)) begin
                    slave_pdo_ram[ram_word_addr] <= s_axis_tdata;
                end

                // Frame state progression
                if (s_axis_tlast) begin
                    beat_idx      <= 12'd0;
                    cur_byte_addr <= 32'd0;
                    wkc_accum     <= 16'd0;
                    slave_visited <= {NUM_SLAVES{1'b0}};
                    fcs_accum     <= 32'hFFFF_FFFF;
                end else begin
                    beat_idx <= beat_idx + 12'd1;

                    // Update running Ethernet FCS over outgoing modified wire words
                    if (beat_idx == 12'd0)
                        fcs_accum <= next_crc32(32'hFFFF_FFFF, wire_out_word);
                    else
                        fcs_accum <= next_crc32(fcs_accum, wire_out_word);

                    // Parse Datagram Headers on Beats 4 & 5
                    if (beat_idx == 12'd4) begin
                        latched_cmd           <= s_axis_tdata[31:24];
                        latched_payload_bytes <= s_axis_tdata[15:0];
                        latched_payload_words <= s_axis_tdata[13:2];
                        wkc_accum             <= 16'd0;
                        slave_visited         <= {NUM_SLAVES{1'b0}};
                    end else if (beat_idx == 12'd5) begin
                        latched_logical_addr  <= s_axis_tdata;
                        cur_byte_addr         <= s_axis_tdata;
                    end else if ((beat_idx >= 12'd6) &&
                                 (beat_idx < (12'd6 + latched_payload_words))) begin
                        cur_byte_addr <= cur_byte_addr + 32'd4;
                        if (first_hit) begin
                            wkc_accum                    <= wkc_accum + wkc_step;
                            slave_visited[cur_slave_id] <= 1'b1;
                        end
                    end
                end
            end else if (m_axis_tready && m_axis_tvalid) begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast  <= 1'b0;
            end
        end
    end

endmodule

