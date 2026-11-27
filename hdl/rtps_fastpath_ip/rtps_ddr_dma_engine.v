`timescale 1ns / 1ps

// =============================================================================
// Module: rtps_ddr_dma_engine
//
// Description:
//   AXI4 Full Master (m_axi_ddr, 32-bit addr/data) for direct PL <-> PS DDR
//   streaming of large (>8 KB) RTPS DATA_FRAG payloads via Zynq S_AXI_HP1:
//     - Read Channel:  Fetches <=1,404-byte fragments from TX DDR Ring slots,
//                      accumulating 32-bit binary payload checksum and folding
//                      once at completion (100 MHz timing-safe).
//     - Write Channel: Streams validated fragment payloads from rx_pkt_ram
//                      directly to their final DDR offset (Direct Data
//                      Placement), splitting bursts at 4 KB AXI4 boundaries and
//                      waiting for BVALID before asserting wr_done_pulse.
// =============================================================================

module rtps_ddr_dma_engine #(
    parameter integer DATA_WIDTH    = 32,
    parameter integer ADDR_WIDTH    = 32,
    parameter integer MAX_BURST_LEN = 32   // 32 words = 128 bytes per AXI4 burst
)(
    input  wire                   clk,
    input  wire                   rst_n_local,

    // -------------------------------------------------------------------------
    // Read Request Interface (from rtps_tx_engine)
    // -------------------------------------------------------------------------
    input  wire                   rd_req_valid,
    input  wire [ADDR_WIDTH-1:0]  rd_req_addr,
    input  wire [9:0]             rd_req_words,   // 1 .. 351 words
    output reg                    rd_data_valid,
    output reg  [DATA_WIDTH-1:0]  rd_data_word,   // Little-endian host word
    output reg  [9:0]             rd_data_idx,    // 0 .. rd_req_words-1
    output reg                    rd_done_pulse,
    output reg  [15:0]            rd_csum_fold,   // 1's-complement folded sum of bswap32(rdata)
    output reg                    rd_resp_err,

    // -------------------------------------------------------------------------
    // Write Request Interface (from rtps_rx_reasm -> rx_pkt_ram)
    // -------------------------------------------------------------------------
    input  wire                   wr_req_valid,
    input  wire [ADDR_WIDTH-1:0]  wr_req_addr,
    input  wire [9:0]             wr_req_words,   // 1 .. 351 words
    input  wire [8:0]             wr_pkt_base_word,
    output reg  [8:0]             wr_pkt_rd_addr,
    input  wire [DATA_WIDTH-1:0]  wr_pkt_rd_data,
    output reg                    wr_done_pulse,
    output reg                    wr_resp_err,

    // -------------------------------------------------------------------------
    // AXI4 Full Master Interface (m_axi_ddr -> Zynq S_AXI_HP1)
    // -------------------------------------------------------------------------
    output wire [0:0]             m_axi_ddr_awid,
    output reg  [ADDR_WIDTH-1:0]  m_axi_ddr_awaddr,
    output reg  [7:0]             m_axi_ddr_awlen,
    output wire [2:0]             m_axi_ddr_awsize,
    output wire [1:0]             m_axi_ddr_awburst,
    output wire                   m_axi_ddr_awlock,
    output wire [3:0]             m_axi_ddr_awcache,
    output wire [2:0]             m_axi_ddr_awprot,
    output wire [3:0]             m_axi_ddr_awqos,
    output reg                    m_axi_ddr_awvalid,
    input  wire                   m_axi_ddr_awready,

    output reg  [DATA_WIDTH-1:0]  m_axi_ddr_wdata,
    output wire [3:0]             m_axi_ddr_wstrb,
    output reg                    m_axi_ddr_wlast,
    output reg                    m_axi_ddr_wvalid,
    input  wire                   m_axi_ddr_wready,

    input  wire [0:0]             m_axi_ddr_bid,
    input  wire [1:0]             m_axi_ddr_bresp,
    input  wire                   m_axi_ddr_bvalid,
    output reg                    m_axi_ddr_bready,

    output wire [0:0]             m_axi_ddr_arid,
    output reg  [ADDR_WIDTH-1:0]  m_axi_ddr_araddr,
    output reg  [7:0]             m_axi_ddr_arlen,
    output wire [2:0]             m_axi_ddr_arsize,
    output wire [1:0]             m_axi_ddr_arburst,
    output wire                   m_axi_ddr_arlock,
    output wire [3:0]             m_axi_ddr_arcache,
    output wire [2:0]             m_axi_ddr_arprot,
    output wire [3:0]             m_axi_ddr_arqos,
    output reg                    m_axi_ddr_arvalid,
    input  wire                   m_axi_ddr_arready,

    input  wire [0:0]             m_axi_ddr_rid,
    input  wire [DATA_WIDTH-1:0]  m_axi_ddr_rdata,
    input  wire [1:0]             m_axi_ddr_rresp,
    input  wire                   m_axi_ddr_rlast,
    input  wire                   m_axi_ddr_rvalid,
    output reg                    m_axi_ddr_rready
);

    // Static AXI4 burst attributes (32-bit INCR, Normal Non-Cacheable Bufferable)
    assign m_axi_ddr_awid    = 1'b0;
    assign m_axi_ddr_awsize  = 3'b010; // 4 bytes
    assign m_axi_ddr_awburst = 2'b01;  // INCR
    assign m_axi_ddr_awlock  = 1'b0;
    assign m_axi_ddr_awcache = 4'b0011;
    assign m_axi_ddr_awprot  = 3'b000;
    assign m_axi_ddr_awqos   = 4'b0000;
    assign m_axi_ddr_wstrb   = 4'b1111;

    assign m_axi_ddr_arid    = 1'b0;
    assign m_axi_ddr_arsize  = 3'b010; // 4 bytes
    assign m_axi_ddr_arburst = 2'b01;  // INCR
    assign m_axi_ddr_arlock  = 1'b0;
    assign m_axi_ddr_arcache = 4'b0011;
    assign m_axi_ddr_arprot  = 3'b000;
    assign m_axi_ddr_arqos   = 4'b0000;

    function [31:0] bswap32;
        input [31:0] w;
        begin
            bswap32 = {w[7:0], w[15:8], w[23:16], w[31:24]};
        end
    endfunction

    function [15:0] fold32_to_16;
        input [31:0] sum_in;
        reg   [16:0] s1;
        begin
            s1           = {1'b0, sum_in[15:0]} + {1'b0, sum_in[31:16]};
            fold32_to_16 = s1[15:0] + {15'd0, s1[16]};
        end
    endfunction

    // =========================================================================
    // 1. AXI4 Read Channel FSM (TX DDR Ring -> Ping-Pong Fragment BRAM)
    // =========================================================================
    localparam [2:0]
        RD_IDLE       = 3'd0,
        RD_CALC_BURST = 3'd1,
        RD_AR         = 3'd2,
        RD_DATA       = 3'd3,
        RD_DONE       = 3'd4;

    reg [2:0]  rd_state;
    reg [31:0] rd_cur_addr;
    reg [9:0]  rd_words_rem;
    reg [9:0]  rd_total_idx;
    reg [7:0]  rd_burst_words;
    reg [31:0] rd_csum_acc;

    wire [10:0] rd_words_to_4k = 11'd1024 - {1'b0, rd_cur_addr[11:2]};
    wire [10:0] rd_cap_burst   = (rd_words_rem < MAX_BURST_LEN) ? {1'b0, rd_words_rem} : MAX_BURST_LEN[10:0];
    wire [10:0] rd_next_burst  = (rd_cap_burst < rd_words_to_4k) ? rd_cap_burst : rd_words_to_4k;

    wire [31:0] rd_word_be = bswap32(m_axi_ddr_rdata);

    always @(posedge clk) begin
        if (!rst_n_local) begin
            rd_state          <= RD_IDLE;
            rd_cur_addr       <= 32'd0;
            rd_words_rem      <= 10'd0;
            rd_total_idx      <= 10'd0;
            rd_burst_words    <= 8'd0;
            rd_csum_acc       <= 32'd0;
            rd_data_valid     <= 1'b0;
            rd_data_word      <= 32'd0;
            rd_data_idx       <= 10'd0;
            rd_done_pulse     <= 1'b0;
            rd_csum_fold      <= 16'd0;
            rd_resp_err       <= 1'b0;
            m_axi_ddr_araddr  <= 32'd0;
            m_axi_ddr_arlen   <= 8'd0;
            m_axi_ddr_arvalid <= 1'b0;
            m_axi_ddr_rready  <= 1'b0;
        end else begin
            rd_data_valid <= 1'b0;
            rd_done_pulse <= 1'b0;

            case (rd_state)
                RD_IDLE: begin
                    m_axi_ddr_arvalid <= 1'b0;
                    m_axi_ddr_rready  <= 1'b0;
                    if (rd_req_valid) begin
                        rd_cur_addr  <= rd_req_addr;
                        rd_words_rem <= rd_req_words;
                        rd_total_idx <= 10'd0;
                        rd_csum_acc  <= 32'd0;
                        rd_resp_err  <= 1'b0;
                        if (rd_req_words == 10'd0)
                            rd_state <= RD_DONE;
                        else
                            rd_state <= RD_CALC_BURST;
                    end
                end

                RD_CALC_BURST: begin
                    rd_burst_words    <= rd_next_burst[7:0];
                    m_axi_ddr_araddr  <= rd_cur_addr;
                    m_axi_ddr_arlen   <= rd_next_burst[7:0] - 8'd1;
                    m_axi_ddr_arvalid <= 1'b1;
                    rd_state          <= RD_AR;
                end

                RD_AR: begin
                    if (m_axi_ddr_arvalid && m_axi_ddr_arready) begin
                        m_axi_ddr_arvalid <= 1'b0;
                        m_axi_ddr_rready  <= 1'b1;
                        rd_state          <= RD_DATA;
                    end
                end

                RD_DATA: begin
                    if (m_axi_ddr_rvalid && m_axi_ddr_rready) begin
                        rd_data_valid <= 1'b1;
                        rd_data_word  <= m_axi_ddr_rdata;
                        rd_data_idx   <= rd_total_idx;
                        rd_total_idx  <= rd_total_idx + 10'd1;
                        rd_csum_acc   <= rd_csum_acc + {16'd0, rd_word_be[31:16]} + {16'd0, rd_word_be[15:0]};
                        if (m_axi_ddr_rresp != 2'b00) begin
                            rd_resp_err <= 1'b1;
                        end

                        if (m_axi_ddr_rlast) begin
                            m_axi_ddr_rready <= 1'b0;
                            rd_cur_addr      <= rd_cur_addr + {22'd0, rd_burst_words, 2'b00};
                            rd_words_rem     <= rd_words_rem - {2'b00, rd_burst_words};
                            if (rd_words_rem <= {2'b00, rd_burst_words})
                                rd_state <= RD_DONE;
                            else
                                rd_state <= RD_CALC_BURST;
                        end
                    end
                end

                RD_DONE: begin
                    rd_csum_fold  <= fold32_to_16(rd_csum_acc);
                    rd_done_pulse <= 1'b1;
                    rd_state      <= RD_IDLE;
                end

                default: rd_state <= RD_IDLE;
            endcase
        end
    end

    // =========================================================================
    // 2. AXI4 Write Channel FSM (rx_pkt_ram -> RX DDR Ring DDP)
    // =========================================================================
    localparam [2:0]
        WR_IDLE       = 3'd0,
        WR_CALC_BURST = 3'd1,
        WR_AW         = 3'd2,
        WR_DATA_PRIME = 3'd3,
        WR_DATA       = 3'd4,
        WR_BWAIT      = 3'd5,
        WR_DONE       = 3'd6;

    reg [2:0]  wr_state;
    reg [31:0] wr_cur_addr;
    reg [9:0]  wr_words_rem;
    reg [8:0]  wr_pkt_ptr;
    reg [7:0]  wr_burst_words;
    reg [7:0]  wr_beat_rem;

    wire [10:0] wr_words_to_4k = 11'd1024 - {1'b0, wr_cur_addr[11:2]};
    wire [10:0] wr_cap_burst   = (wr_words_rem < MAX_BURST_LEN) ? {1'b0, wr_words_rem} : MAX_BURST_LEN[10:0];
    wire [10:0] wr_next_burst  = (wr_cap_burst < wr_words_to_4k) ? wr_cap_burst : wr_words_to_4k;

    always @(posedge clk) begin
        if (!rst_n_local) begin
            wr_state          <= WR_IDLE;
            wr_cur_addr       <= 32'd0;
            wr_words_rem      <= 10'd0;
            wr_pkt_ptr        <= 9'd0;
            wr_burst_words    <= 8'd0;
            wr_beat_rem       <= 8'd0;
            wr_pkt_rd_addr    <= 9'd0;
            wr_done_pulse     <= 1'b0;
            wr_resp_err       <= 1'b0;
            m_axi_ddr_awaddr  <= 32'd0;
            m_axi_ddr_awlen   <= 8'd0;
            m_axi_ddr_awvalid <= 1'b0;
            m_axi_ddr_wdata   <= 32'd0;
            m_axi_ddr_wlast   <= 1'b0;
            m_axi_ddr_wvalid  <= 1'b0;
            m_axi_ddr_bready  <= 1'b0;
        end else begin
            wr_done_pulse <= 1'b0;

            case (wr_state)
                WR_IDLE: begin
                    m_axi_ddr_awvalid <= 1'b0;
                    m_axi_ddr_wvalid  <= 1'b0;
                    m_axi_ddr_wlast   <= 1'b0;
                    m_axi_ddr_bready  <= 1'b0;
                    if (wr_req_valid) begin
                        wr_cur_addr  <= wr_req_addr;
                        wr_words_rem <= wr_req_words;
                        wr_pkt_ptr   <= wr_pkt_base_word;
                        wr_resp_err  <= 1'b0;
                        if (wr_req_words == 10'd0)
                            wr_state <= WR_DONE;
                        else
                            wr_state <= WR_CALC_BURST;
                    end
                end

                WR_CALC_BURST: begin
                    wr_burst_words    <= wr_next_burst[7:0];
                    m_axi_ddr_awaddr  <= wr_cur_addr;
                    m_axi_ddr_awlen   <= wr_next_burst[7:0] - 8'd1;
                    m_axi_ddr_awvalid <= 1'b1;
                    wr_pkt_rd_addr    <= wr_pkt_ptr;
                    wr_state          <= WR_AW;
                end

                WR_AW: begin
                    if (m_axi_ddr_awvalid && m_axi_ddr_awready) begin
                        m_axi_ddr_awvalid <= 1'b0;
                        wr_beat_rem       <= wr_burst_words;
                        wr_pkt_ptr        <= wr_pkt_ptr + 9'd1;
                        wr_pkt_rd_addr    <= wr_pkt_ptr + 9'd1;
                        wr_state          <= WR_DATA_PRIME;
                    end
                end

                WR_DATA_PRIME: begin
                    // wr_pkt_rd_data is now valid for word 0; wr_pkt_rd_addr already points to word 1
                    m_axi_ddr_wdata  <= bswap32(wr_pkt_rd_data);
                    m_axi_ddr_wvalid <= 1'b1;
                    m_axi_ddr_wlast  <= (wr_beat_rem == 8'd1);
                    if (wr_beat_rem > 8'd1) begin
                        wr_pkt_ptr     <= wr_pkt_ptr + 9'd1;
                        wr_pkt_rd_addr <= wr_pkt_ptr + 9'd1;
                    end
                    wr_state         <= WR_DATA;
                end

                WR_DATA: begin
                    if (m_axi_ddr_wvalid && m_axi_ddr_wready) begin
                        if (m_axi_ddr_wlast) begin
                            m_axi_ddr_wvalid <= 1'b0;
                            m_axi_ddr_wlast  <= 1'b0;
                            m_axi_ddr_bready <= 1'b1;
                            wr_state         <= WR_BWAIT;
                        end else begin
                            wr_beat_rem      <= wr_beat_rem - 8'd1;
                            m_axi_ddr_wdata  <= bswap32(wr_pkt_rd_data);
                            m_axi_ddr_wlast  <= (wr_beat_rem == 8'd2);
                            if (wr_beat_rem > 8'd2) begin
                                wr_pkt_ptr     <= wr_pkt_ptr + 9'd1;
                                wr_pkt_rd_addr <= wr_pkt_ptr + 9'd1;
                            end
                        end
                    end
                end

                WR_BWAIT: begin
                    if (m_axi_ddr_bvalid && m_axi_ddr_bready) begin
                        m_axi_ddr_bready <= 1'b0;
                        if (m_axi_ddr_bresp != 2'b00) begin
                            wr_resp_err <= 1'b1;
                        end
                        wr_cur_addr  <= wr_cur_addr + {22'd0, wr_burst_words, 2'b00};
                        wr_words_rem <= wr_words_rem - {2'b00, wr_burst_words};
                        if (wr_words_rem <= {2'b00, wr_burst_words})
                            wr_state <= WR_DONE;
                        else
                            wr_state <= WR_CALC_BURST;
                    end
                end

                WR_DONE: begin
                    wr_done_pulse <= 1'b1;
                    wr_state      <= WR_IDLE;
                end

                default: wr_state <= WR_IDLE;
            endcase
        end
    end

endmodule
