`timescale 1ns / 1ps

// =============================================================================
// Module: rtps_s2mm_egress
//
// Description:
//   Decoupled RX Egress Controller & Pipelined BRAM-to-AXI4-Stream S2MM Engine.
//   Runs independently of rtps_tx_engine so MM2S TX never blocks on RX egress.
//   Includes a full skid buffer to absorb 1-cycle synchronous BRAM read latency
//   under downstream m_axis_tready backpressure.
// =============================================================================

module rtps_s2mm_egress #(
    parameter integer DATA_WIDTH = 32,
    parameter integer ADDR_WIDTH = 11
)(
    input  wire                      clk,
    input  wire                      rst_n_local,

    // Decoupled RX Egress Control & CSR Handshake
    input  wire                      latest_valid,
    input  wire                      status_write_pulse,
    input  wire [31:0]               status_write_data,
    input  wire                      trigger_detected,
    output wire                      slot_consume_pulse,
    output reg                       rx_done_irq,
    output reg  [1:0]                rx_egress_state,
    output reg                       egress_active,

    // Egress Addressing
    input  wire [11:0]               egress_start_word,
    input  wire [ADDR_WIDTH:0]       egress_total_words,

    // Reassembly BRAM Read Port
    output reg  [11:0]               s2mm_rd_addr,
    input  wire [DATA_WIDTH-1:0]     s2mm_dout,

    // External AXI4-Stream Master (DMA0 S2MM)
    output reg  [DATA_WIDTH-1:0]     m_axis_tdata,
    output wire [(DATA_WIDTH/8)-1:0] m_axis_tkeep,
    output reg                       m_axis_tvalid,
    input  wire                      m_axis_tready,
    output reg                       m_axis_tlast
);

    `include "rtps_fastpath_defs.vh"

    reg  [ADDR_WIDTH:0]   egress_rd_ptr;
    reg                   bram_pipe_valid;
    reg                   bram_pipe_last;
    reg  [DATA_WIDTH-1:0] skid_data;
    reg                   skid_last;
    reg                   skid_valid;

    // Convert network-order engine words back to host little-endian DDR byte order for S2MM.
    wire [31:0] s2mm_dout_host = {s2mm_dout[7:0], s2mm_dout[15:8], s2mm_dout[23:16], s2mm_dout[31:24]};

    assign m_axis_tkeep = {(DATA_WIDTH/8){1'b1}};
    wire   tx_handshake = m_axis_tvalid && m_axis_tready;
    wire   egress_done  = egress_active && tx_handshake && m_axis_tlast;

    assign slot_consume_pulse = (rx_egress_state == RX_EGRESS_IDLE) && latest_valid && !trigger_detected;
    wire   ack_rx_irq   = status_write_pulse && status_write_data[3];
    wire   start_egress = (rx_egress_state == RX_EGRESS_WAIT) && status_write_pulse && status_write_data[1] && !trigger_detected;
    wire   abort_egress = trigger_detected;

    wire can_issue_bram_rd = egress_active && (egress_rd_ptr < egress_total_words) &&
                             (!skid_valid) &&
                             (m_axis_tready || !bram_pipe_valid || !m_axis_tvalid);

    always @(*) begin
        s2mm_rd_addr = egress_start_word + egress_rd_ptr[11:0];
    end

    always @(posedge clk) begin
        if (!rst_n_local) begin
            rx_egress_state <= RX_EGRESS_IDLE;
            rx_done_irq     <= 1'b0;
            egress_active   <= 1'b0;
            egress_rd_ptr   <= {(ADDR_WIDTH+1){1'b0}};
            bram_pipe_valid <= 1'b0;
            bram_pipe_last  <= 1'b0;
            skid_data       <= {DATA_WIDTH{1'b0}};
            skid_last       <= 1'b0;
            skid_valid      <= 1'b0;
            m_axis_tvalid   <= 1'b0;
            m_axis_tdata    <= {DATA_WIDTH{1'b0}};
            m_axis_tlast    <= 1'b0;
        end else if (abort_egress) begin
            rx_egress_state <= RX_EGRESS_IDLE;
            rx_done_irq     <= 1'b0;
            egress_active   <= 1'b0;
            bram_pipe_valid <= 1'b0;
            bram_pipe_last  <= 1'b0;
            skid_valid      <= 1'b0;
            m_axis_tvalid   <= 1'b0;
            m_axis_tlast    <= 1'b0;
        end else begin
            case (rx_egress_state)
                RX_EGRESS_IDLE: begin
                    egress_active <= 1'b0;
                    if (latest_valid) begin
                        rx_done_irq     <= 1'b1;
                        rx_egress_state <= RX_EGRESS_IRQ;
                    end
                end
                RX_EGRESS_IRQ: begin
                    if (ack_rx_irq) begin
                        rx_done_irq     <= 1'b0;
                        rx_egress_state <= RX_EGRESS_WAIT;
                    end
                end
                RX_EGRESS_WAIT: begin
                    if (start_egress) begin
                        rx_egress_state <= RX_EGRESS_RUN;
                    end
                end
                RX_EGRESS_RUN: begin
                    if (egress_done) begin
                        rx_egress_state <= RX_EGRESS_IDLE;
                    end
                end
            endcase

            if (start_egress) begin
                egress_active   <= 1'b1;
                egress_rd_ptr   <= {(ADDR_WIDTH+1){1'b0}};
                bram_pipe_valid <= 1'b0;
                bram_pipe_last  <= 1'b0;
                skid_valid      <= 1'b0;
                m_axis_tvalid   <= 1'b0;
                m_axis_tlast    <= 1'b0;
            end else if (egress_active) begin
                if (can_issue_bram_rd) begin
                    egress_rd_ptr   <= egress_rd_ptr + 1'b1;
                    bram_pipe_valid <= 1'b1;
                    bram_pipe_last  <= (egress_rd_ptr == egress_total_words - 1);
                end else begin
                    bram_pipe_valid <= 1'b0;
                    bram_pipe_last  <= 1'b0;
                end

                if (skid_valid) begin
                    if (m_axis_tready) begin
                        m_axis_tdata  <= skid_data;
                        m_axis_tlast  <= skid_last;
                        m_axis_tvalid <= 1'b1;
                        if (bram_pipe_valid) begin
                            skid_data  <= s2mm_dout_host;
                            skid_last  <= bram_pipe_last;
                            skid_valid <= 1'b1;
                        end else begin
                            skid_valid <= 1'b0;
                        end
                    end
                end else begin
                    if (m_axis_tready || !m_axis_tvalid) begin
                        if (bram_pipe_valid) begin
                            m_axis_tdata  <= s2mm_dout_host;
                            m_axis_tlast  <= bram_pipe_last;
                            m_axis_tvalid <= 1'b1;
                        end else if (m_axis_tready && m_axis_tvalid) begin
                            m_axis_tvalid <= 1'b0;
                            m_axis_tlast  <= 1'b0;
                        end
                    end else begin
                        if (bram_pipe_valid) begin
                            skid_data  <= s2mm_dout_host;
                            skid_last  <= bram_pipe_last;
                            skid_valid <= 1'b1;
                        end
                    end
                end

                if (tx_handshake && m_axis_tlast) begin
                    egress_active <= 1'b0;
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;
                    skid_valid    <= 1'b0;
                end
            end else begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast  <= 1'b0;
                skid_valid    <= 1'b0;
            end
        end
    end

endmodule
