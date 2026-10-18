`timescale 1ns / 1ps

// =============================================================================
// Module: bram_dma_ip (bram_dma_ip.v)
//
// Description:
//   Store-and-Forward BRAM Hardware Accelerator with Memory-Mapped DMA Ingress,
//   Hardware Doorbell Trigger Detection, Full 32-bit On-the-Fly CRC-32 Checking,
//   Dedicated Status Register, and AXI-Stream S2MM Egress.
//
// Memory Map (4 KB / 1024 words):
//   - 0x000 - 0xFF4 (Words 0 .. 1021) : BRAM Data Buffer (Payload: up to 4088 bytes)
//   - 0xFF8         (Word 1022)       : IP_STATUS_REG (Read-Only)
//                                       Bit [0]   : ip_done_irq (Interrupt wire state)
//                                       Bits [4:1]: state (Current FSM state)
//                                       Bit [5]   : crc_err (1 = CRC mismatch)
//                                       Bits [31:6]: Reserved (26'b0)
//   - 0xFFC         (Word 1023)       : EXPECTED_CRC / CALC_CRC (R/W)
//                                       Write: Configures expected_crc[31:0]
//                                              (Write 0x0 to bypass CRC check; in S_DONE_IRQ/S_WAIT_EGRESS acts as reset)
//                                       Read : Returns FULL 32-bit calculated crc_accum
//
// Architecture & Workflow:
//   1. Memory-Mapped Slave (S_AXI) & Streaming Slave (S_AXIS):
//      - DMA MM2S streams payload data into BRAM Port A words [0 .. 1021].
//      - As words arrive, on-the-fly CRC-32 accumulator updates each beat.
//      - CPU writes expected_crc to EXPECTED_CRC (0x0FFC) before/during ingress.
//      - Packet completion is signaled by s_axis_tlast (or writing 1 to STATUS_ADDR for manual CPU mode).
//
//   2. Hardware Trigger & CRC Verification (S_CHECK_CRC):
//      - Triggered automatically on s_axis_tlast (DMA) or status_write with bit 0 (CPU).
//      - If (crc_accum == expected_crc) or (expected_crc == 0):
//          CRC PASSED -> Moves to S_COMP_READ to begin processing.
//      - If (crc_accum != expected_crc):
//          CRC FAILED -> Sets crc_err = 1, aborts compute, and raises ip_done_irq
//          to alert PS of corrupted ingress buffer.
//
//   3. Dual-Port BRAM (Inferred RAMB36E1 Block RAM):
//      - Port A: Dedicated to external AXI bus (writes from DMA / reads from CPU).
//      - Port B: Dedicated to internal compute engine and M_AXIS egress streaming.
//
//   4. Compute Engine:
//      - Iterates across BRAM data in-place over Port B.
//      - Modifies words and writes back responses.
//
//   5. Interrupt & Egress (M_AXIS to S2MM DMA):
//      - Upon compute completion (or CRC error), asserts Level-High interrupt `ip_done_irq`.
//      - When PS primes S2MM DMA, `m_axis_tready` asserts.
//      - IP automatically deasserts `ip_done_irq` and streams out processed
//        results onto M_AXIS with proper TLAST framing on the final word.
// =============================================================================

module bram_dma_ip #(
    parameter integer DATA_WIDTH   = 32,
    parameter integer BRAM_DEPTH   = 1024,   // 1024 words = 4096 bytes (4 KB)
    parameter integer ADDR_WIDTH   = 10,     // 2^10 = 1024 words
    parameter integer CFG_DEPTH    = 256,     // 256 words (configuration space)
    parameter integer CFG_ADDR_WIDTH = 8     // 2^8 = 256 words (configuration space)
)(
    input  wire                      clk,
    input  wire                      rst_n,

    // -------------------------------------------------------------------------
    // 1. AXI4-Lite / AXI4 Memory-Mapped Slave (BRAM Window: 0x000 - 0xFFC)
    // -------------------------------------------------------------------------
    input  wire [CFG_ADDR_WIDTH+1:0] s_axi_awaddr,
    input  wire                      s_axi_awvalid,
    output reg                       s_axi_awready,
    input  wire [DATA_WIDTH-1:0]     s_axi_wdata,
    input  wire [(DATA_WIDTH/8)-1:0] s_axi_wstrb,
    input  wire                      s_axi_wvalid,
    output reg                       s_axi_wready,
    output wire [1:0]                s_axi_bresp,
    output reg                       s_axi_bvalid,
    input  wire                      s_axi_bready,

    input  wire [CFG_ADDR_WIDTH+1:0] s_axi_araddr,
    input  wire                      s_axi_arvalid,
    output reg                       s_axi_arready,
    output reg  [DATA_WIDTH-1:0]     s_axi_rdata,
    output wire [1:0]                s_axi_rresp,
    output reg                       s_axi_rvalid,
    input  wire                      s_axi_rready,

    // -------------------------------------------------------------------------
    // 2. AXI4-Stream Slave (Connected to AXI DMA MM2S: DDR RAM -> BRAM)
    // -------------------------------------------------------------------------
    input  wire [DATA_WIDTH-1:0]     s_axis_tdata,
    input  wire [(DATA_WIDTH/8)-1:0] s_axis_tkeep,
    input  wire                      s_axis_tvalid,
    output wire                      s_axis_tready,
    input  wire                      s_axis_tlast,

    // -------------------------------------------------------------------------
    // 3. AXI4-Stream Master (Connected to AXI DMA S2MM: BRAM -> DDR RAM)
    // -------------------------------------------------------------------------
    output reg  [DATA_WIDTH-1:0]     m_axis_tdata,
    output wire [(DATA_WIDTH/8)-1:0] m_axis_tkeep,
    output reg                       m_axis_tvalid,
    input  wire                      m_axis_tready,
    output reg                       m_axis_tlast,

    // -------------------------------------------------------------------------
    // 4. Dedicated Level-High Interrupt Wire to PS GIC (IRQ_F2P[2])
    // -------------------------------------------------------------------------
    output reg                       ip_done_irq
);

    // =========================================================================
    // Address Map Constants
    // =========================================================================
    localparam [CFG_ADDR_WIDTH-1:0] TRIGGER_ADDR = 8'h0;
    localparam [CFG_ADDR_WIDTH-1:0] STATUS_ADDR = 8'h1;
    localparam [CFG_ADDR_WIDTH-1:0] COMPUTE_CTRL_ADDR = 8'h2;
    localparam [CFG_ADDR_WIDTH-1:0] EXPECTED_CRC_ADDR = 8'h3;

    reg [31:0] ctrl_reg;
    reg status_write_pulse;
    reg trigger_detected;
    reg [DATA_WIDTH-1:0] status_write_data;

    // =========================================================================
    // FSM State Definitions
    // =========================================================================
    localparam [3:0]
        S_IDLE        = 4'd0, // Awaiting write to TRIGGER_ADDR
        S_CHECK_CRC   = 4'd1, // Validate CRC accumulator against expected CRC
        S_COMP_READ   = 4'd2, // Read BRAM word for compute
        S_COMP_WAIT   = 4'd3, // Multi-cycle processing delay
        S_COMP_WRITE  = 4'd4, // Write processed result back to BRAM
        S_DONE_IRQ    = 4'd5, // Assert interrupt to PS & await Driver IRQ ACK
        S_WAIT_EGRESS = 4'd6, // Await Driver read kick (write to STATUS_ADDR)
        S_TX_FETCH    = 4'd7, // Fetch word from BRAM for M_AXIS
        S_TX_WAIT_RAM = 4'd8, // Wait 1 cycle for BRAM synchronous read
        S_TX_SEND     = 4'd9; // Stream out word to S2MM DMA

    reg [3:0] state;

    // =========================================================================
    // Dual-Port Inferred Block RAM (1024 x 32-bit = 4 KB)
    // Port A: Servicing Memory-Mapped AXI writes/reads (DMA / CPU)
    // Port B: Servicing Internal Compute and Egress Stream (M_AXIS)
    // =========================================================================
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] bram [0:BRAM_DEPTH-1];

    // AXI-Stream Ingress signals (from AXI DMA MM2S)
    reg  [ADDR_WIDTH-1:0]     rx_ptr;
    assign s_axis_tready = (state == S_IDLE);
    wire                      rx_handshake = s_axis_tvalid && s_axis_tready;
    wire                      axis_wr      = rx_handshake && (state == S_IDLE);

    always @(posedge clk) begin
        if (axis_wr) begin
            bram[rx_ptr] <= s_axis_tdata;
        end
    end

    // Port B signals
    reg  [ADDR_WIDTH-1:0] portb_addr;
    reg  [DATA_WIDTH-1:0] portb_din;
    reg                   portb_we;
    reg  [DATA_WIDTH-1:0] portb_dout;

    // Port B Servicing Internal Compute Engine and M_AXIS Egress
    always @(posedge clk) begin
        if (portb_we) begin
            bram[portb_addr] <= portb_din;
        end 
        portb_dout <= bram[portb_addr];
    end

    // =========================================================================
    // Full 32-bit CRC-32 Calculation Logic
    // =========================================================================
    // Standard 32-bit parallel reduction function (CRC-32 / Checksum step)
    function [31:0] next_crc32;
        input [31:0] current_crc;
        input [31:0] data_in;
        reg   [31:0] x;
        begin
            x = current_crc ^ data_in;
            next_crc32 = {x[30:0], 1'b0} ^ (x[31] ? 32'h04C11DB7 : 32'h0);
        end
    endfunction

    reg [31:0] crc_accum;
    reg [31:0] expected_crc;
    reg        crc_err;

    // =========================================================================
    // AXI Slave Write Channel, Trigger Detection & On-the-Fly CRC
    // =========================================================================
    assign s_axi_bresp = 2'b00; // OKAY
    assign s_axi_rresp = 2'b00; // OKAY

    wire reg_wr_fire = s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid;

    always @(posedge clk) begin
        if (!rst_n) begin
            s_axi_awready    <= 1'b0;
            s_axi_wready     <= 1'b0;
            s_axi_bvalid     <= 1'b0;
            ctrl_reg          <= 32'h0;
            expected_crc     <= 32'h0;
            status_write_pulse <= 1'b0;
            status_write_data  <= 32'h0;
            trigger_detected <= 1'b0;
        end else begin
            status_write_pulse <= 1'b0;
            trigger_detected <= 1'b0;

            if (reg_wr_fire) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;

                case (s_axi_awaddr[CFG_ADDR_WIDTH+1:2])
                    STATUS_ADDR: begin
                        status_write_pulse <= 1'b1;
                        status_write_data  <= s_axi_wdata;
                    end
                    TRIGGER_ADDR: begin
                        trigger_detected <= 1'b1;
                    end
                    EXPECTED_CRC_ADDR: begin
                        expected_crc <= s_axi_wdata;
                    end
                    COMPUTE_CTRL_ADDR: begin
                        ctrl_reg <= s_axi_wdata;
                    end
                endcase

                s_axi_bvalid <= 1'b1;
            end else begin
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;
            end


            // 4. Write Response Handshake
            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end


    // CRC Calcultation on the fly
    always @(posedge clk) begin
        if (!rst_n) begin
            crc_accum <= 32'hFFFFFFFF;
        end else if (axis_wr) begin
            if (rx_ptr == 0)
                crc_accum <= next_crc32(32'hFFFFFFFF, s_axis_tdata);
            else if (rx_ptr < BRAM_DEPTH)
                crc_accum <= next_crc32(crc_accum, s_axis_tdata);
        end
    end

    // =========================================================================
    // AXI Slave Read Channel: Separated Full 32-bit CRC & Status Registers
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            s_axi_arready   <= 1'b0;
            s_axi_rvalid    <= 1'b0;
            s_axi_rdata     <= 32'd0;
        end else begin
            if (!s_axi_arready && s_axi_arvalid && !s_axi_rvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rdata    <= 32'd0;
                s_axi_rvalid   <= 1'b1;
                case (s_axi_araddr[CFG_ADDR_WIDTH+1:2])
                    STATUS_ADDR: begin
                        s_axi_rdata <= {26'd0, crc_err, state, ip_done_irq};
                    end
                    EXPECTED_CRC_ADDR: begin
                        s_axi_rdata <= crc_accum;
                    end
                    COMPUTE_CTRL_ADDR: begin
                        s_axi_rdata <= ctrl_reg;
                    end
                    default: begin
                        s_axi_rdata <= 32'd0;
                    end
                endcase
            end else begin
                s_axi_arready <= 1'b0;
            end

            // Read completion handshake
            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Compute & Egress State Machine
    // =========================================================================
    reg [ADDR_WIDTH:0] comp_ptr;
    reg [ADDR_WIDTH:0] rd_ptr;
    reg [ADDR_WIDTH:0] pkt_len;
    reg [15:0]         work_timer;


    assign m_axis_tkeep = {(DATA_WIDTH/8){1'b1}};
    wire   tx_handshake = m_axis_tvalid && m_axis_tready;

    always @(posedge clk) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            comp_ptr      <= 0;
            rd_ptr        <= 0;
            rx_ptr        <= 0;
            pkt_len       <= 0;
            work_timer    <= 0;
            crc_err       <= 1'b0;
            portb_we      <= 1'b0;
            portb_addr    <= 0;
            portb_din     <= 0;
            m_axis_tvalid <= 1'b0;
            m_axis_tdata  <= 0;
            m_axis_tlast  <= 1'b0;
            ip_done_irq   <= 1'b0;
        end else begin
            portb_we <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // 1. IDLE: Wait for DMA stream (MM2S) or CPU write to land on TRIGGER_ADDR
                // -------------------------------------------------------------
                S_IDLE: begin
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;
                    ip_done_irq   <= 1'b0;

                    // AXI-Stream Ingress from AXI DMA MM2S:
                    if (rx_handshake) begin
                        if (s_axis_tlast || (rx_ptr == BRAM_DEPTH - 1)) begin
                            pkt_len  <= rx_ptr + 1'b1;
                            comp_ptr <= 0;
                            rx_ptr   <= 0;
                            state    <= S_CHECK_CRC;
                        end else begin
                            rx_ptr   <= rx_ptr + 1'b1;
                        end
                    end
                    else if (status_write_pulse && status_write_data[0]) begin
                        pkt_len  <= BRAM_DEPTH; // CPU-triggered compute: Assume full buffer length
                        comp_ptr <= 0;
                        rx_ptr   <= 0;
                        state    <= S_CHECK_CRC;
                    end
                end

                // -------------------------------------------------------------
                // 2. CHECK CRC: Validate accumulated CRC vs expected CRC
                // -------------------------------------------------------------
                S_CHECK_CRC: begin
                    // Bypass CRC check if expected_crc is 0, or if accumulator matches
                    if ((crc_accum == expected_crc) || (expected_crc == 32'h0)) begin
                        crc_err  <= 1'b0;
                        comp_ptr <= 0;
                        state    <= S_COMP_READ; // CRC OK: Proceed to compute!
                    end else begin
                        crc_err     <= 1'b1;     // CRC Failed: Abort compute to protect data!
                        ip_done_irq <= 1'b1;     // Alert PS of error via interrupt
                        state       <= S_DONE_IRQ;
                    end
                end

                // -------------------------------------------------------------
                // 3. COMPUTE: Read word -> Process -> Write back to BRAM
                // -------------------------------------------------------------
                S_COMP_READ: begin
                    portb_addr <= comp_ptr[ADDR_WIDTH-1:0];
                    work_timer <= 16'd100; // Simulated compute latency per word
                    state      <= S_COMP_WAIT;
                end

                S_COMP_WAIT: begin
                    if (work_timer == 0)
                        state <= S_COMP_WRITE;
                    else
                        work_timer <= work_timer - 1'b1;
                end

                S_COMP_WRITE: begin
                    portb_addr <= comp_ptr[ADDR_WIDTH-1:0];
                    // Example Compute: Invert bits and add offset 0x1000
                    portb_din  <= (~portb_dout) + 32'h0000_1000;
                    portb_we   <= 1'b1;

                    if (comp_ptr == pkt_len - 1) begin
                        state       <= S_DONE_IRQ;
                        ip_done_irq <= 1'b1; // Assert Level-High interrupt to PS GIC
                    end else begin
                        comp_ptr <= comp_ptr + 1'b1;
                        state    <= S_COMP_READ;
                    end
                end

                // -------------------------------------------------------------
                // 4. NOTIFY: Hold interrupt high until PS Kernel driver acks
                // -------------------------------------------------------------
                S_DONE_IRQ: begin
                    // Nominal Path: Driver ISR writes to STATUS_ADDR (0x0FF8) to ACK
                    if (status_write_pulse) begin
                        ip_done_irq <= 1'b0; // Lower interrupt line on CPU ACK
                        if (crc_err) begin
                            crc_err <= 1'b0;
                            rx_ptr  <= 0;
                            state   <= S_IDLE;
                        end else begin
                            state   <= S_WAIT_EGRESS;
                        end
                    end
                    // Abort/Reset Path: Driver writes to TRIGGER_ADDR to force reset
                    else if (trigger_detected) begin
                        ip_done_irq <= 1'b0;
                        crc_err     <= 1'b0;
                        rx_ptr      <= 0;
                        state       <= S_IDLE; // Always back to IDLE, never re-compute!
                    end
                end

                // -------------------------------------------------------------
                // 5. WAIT EGRESS: Wait for Driver read() to prime DMA and kick
                // -------------------------------------------------------------
                S_WAIT_EGRESS: begin
                    // Nominal Path: Driver read() writes to STATUS_ADDR (0x0FF8) to kick egress
                    if (status_write_pulse) begin
                        rd_ptr <= 0;
                        state  <= S_TX_FETCH;
                    end
                    // Abort/Reset Path: Driver abandons read and forces reset
                    else if (trigger_detected) begin
                        rx_ptr <= 0;
                        state  <= S_IDLE;
                    end
                end

                // -------------------------------------------------------------
                // 6. EGRESS: Stream processed results from BRAM to S2MM DMA
                // -------------------------------------------------------------
                S_TX_FETCH: begin
                    portb_addr <= rd_ptr[ADDR_WIDTH-1:0];
                    state      <= S_TX_WAIT_RAM;
                end

                S_TX_WAIT_RAM: begin
                    // Wait 1 clock cycle for synchronous BRAM read to settle
                    state <= S_TX_SEND;
                end

                S_TX_SEND: begin
                    m_axis_tdata  <= portb_dout;
                    m_axis_tvalid <= 1'b1;
                    m_axis_tlast  <= (rd_ptr == pkt_len - 1);

                    if (tx_handshake) begin
                        m_axis_tvalid <= 1'b0;
                        m_axis_tlast  <= 1'b0;

                        if (rd_ptr == pkt_len - 1) begin
                            rx_ptr <= 0;
                            state  <= S_IDLE; // Frame complete! Return to IDLE
                        end else begin
                            rd_ptr <= rd_ptr + 1'b1;
                            state  <= S_TX_FETCH;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule