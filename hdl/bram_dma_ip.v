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
//   - 0xFFC         (Word 1023)       : DOORBELL_CRC_REG (R/W)
//                                       Write: Arms trigger, passes expected_crc[31:0]
//                                              (Write 0x0 to bypass CRC check)
//                                       Read : Returns FULL 32-bit calculated crc_accum
//
// Architecture & Workflow:
//   1. Memory-Mapped Slave (S_AXI):
//      - Exposes 4 KB BRAM address window (0x000 to 0xFFC) directly to DMA / CPU.
//      - DMA / CPU writes payload data into BRAM words [0 .. 1021].
//      - As words arrive on Port A, an on-the-fly CRC-32 accumulator updates each beat.
//      - When DMA / CPU writes to DOORBELL_CRC_REG (0x0FFC / word 1023), the written
//        word is captured as EXPECTED_CRC (or 0x0 to bypass CRC check).
//
//   2. Hardware Doorbell & CRC Verification (S_CHECK_CRC):
//      - Writing to TRIGGER_ADDR fires `trigger_detected`.
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
    parameter integer TRIGGER_ADDR = 10'h3FF // Word 1023 (byte offset 0x0FFC)
)(
    input  wire                      clk,
    input  wire                      rst_n,

    // -------------------------------------------------------------------------
    // 1. AXI4-Lite / AXI4 Memory-Mapped Slave (BRAM Window: 0x000 - 0xFFC)
    // -------------------------------------------------------------------------
    input  wire [ADDR_WIDTH+1:0]     s_axi_awaddr,
    input  wire                      s_axi_awvalid,
    output reg                       s_axi_awready,
    input  wire [DATA_WIDTH-1:0]     s_axi_wdata,
    input  wire [(DATA_WIDTH/8)-1:0] s_axi_wstrb,
    input  wire                      s_axi_wvalid,
    output reg                       s_axi_wready,
    output wire [1:0]                s_axi_bresp,
    output reg                       s_axi_bvalid,
    input  wire                      s_axi_bready,

    input  wire [ADDR_WIDTH+1:0]     s_axi_araddr,
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
    localparam [ADDR_WIDTH-1:0] STATUS_ADDR = TRIGGER_ADDR - 1'b1; // Word 1022 (0x0FF8)

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

    // Port A signals
    reg  [ADDR_WIDTH-1:0]     porta_write_addr;
    reg  [ADDR_WIDTH-1:0]     porta_read_addr;
    reg  [DATA_WIDTH-1:0]     porta_din;
    reg  [(DATA_WIDTH/8)-1:0] porta_strb;
    reg                       porta_we;
    reg  [DATA_WIDTH-1:0]     porta_dout;

    // AXI-Stream Ingress signals (from AXI DMA MM2S)
    reg  [ADDR_WIDTH-1:0]     rx_ptr;
    assign s_axis_tready = (state == S_IDLE);
    wire                      rx_handshake = s_axis_tvalid && s_axis_tready;
    wire                      axis_wr      = rx_handshake && (state == S_IDLE);

    // Port A arbitration between AXI-Stream DMA and AXI-Lite Memory-Mapped
    wire [ADDR_WIDTH-1:0]     porta_ram_addr   = axis_wr ? rx_ptr : (porta_we ? porta_write_addr : porta_read_addr);
    wire [DATA_WIDTH-1:0]     actual_porta_din  = axis_wr ? s_axis_tdata : porta_din;
    wire                      actual_porta_we   = axis_wr || porta_we;
    wire [(DATA_WIDTH/8)-1:0] actual_porta_strb = axis_wr ? {(DATA_WIDTH/8){1'b1}} : porta_strb;

    // Port A Synchronous Process (Synthesis template with byte-write enables)
    always @(posedge clk) begin
        if (actual_porta_we) begin
            if (actual_porta_strb[0]) bram[porta_ram_addr][7:0]   <= actual_porta_din[7:0];
            if (actual_porta_strb[1]) bram[porta_ram_addr][15:8]  <= actual_porta_din[15:8];
            if (actual_porta_strb[2]) bram[porta_ram_addr][23:16] <= actual_porta_din[23:16];
            if (actual_porta_strb[3]) bram[porta_ram_addr][31:24] <= actual_porta_din[31:24];
        end
        porta_dout <= bram[porta_ram_addr];
    end

    // Port B signals
    reg  [ADDR_WIDTH-1:0] portb_addr;
    reg  [DATA_WIDTH-1:0] portb_din;
    reg                   portb_we;
    reg  [DATA_WIDTH-1:0] portb_dout;

    // Port B Synchronous Process (Full word write/read for Compute & M_AXIS)
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

    reg [ADDR_WIDTH-1:0]     aw_addr_reg;
    reg                      aw_captured;
    reg [DATA_WIDTH-1:0]     w_data_reg;
    reg [(DATA_WIDTH/8)-1:0] w_strb_reg;
    reg                      w_captured;
    reg                      trigger_detected;
    reg [DATA_WIDTH-1:0]     trigger_val;

    always @(posedge clk) begin
        if (!rst_n) begin
            s_axi_awready    <= 1'b0;
            s_axi_wready     <= 1'b0;
            s_axi_bvalid     <= 1'b0;
            aw_captured      <= 1'b0;
            w_captured       <= 1'b0;
            aw_addr_reg      <= 0;
            w_data_reg       <= 0;
            w_strb_reg       <= 0;
            porta_we         <= 1'b0;
            porta_write_addr <= 0;
            porta_din        <= 0;
            porta_strb       <= 0;
            trigger_detected <= 1'b0;
            trigger_val      <= 0;
            crc_accum        <= 32'hFFFFFFFF;
            expected_crc     <= 0;
        end else begin
            porta_we         <= 1'b0;
            trigger_detected <= 1'b0;

            // --- On-the-Fly CRC Accumulation from AXI-Stream MM2S DMA ---
            if (axis_wr) begin
                if (rx_ptr == 0)
                    crc_accum <= next_crc32(32'hFFFFFFFF, s_axis_tdata);
                else if (rx_ptr < STATUS_ADDR)
                    crc_accum <= next_crc32(crc_accum, s_axis_tdata);
            end

            // 1. Capture Write Address
            if (~s_axi_awready && s_axi_awvalid && ~aw_captured) begin
                s_axi_awready <= 1'b1;
                aw_addr_reg   <= s_axi_awaddr[ADDR_WIDTH+1:2];
                aw_captured   <= 1'b1;
            end else begin
                s_axi_awready <= 1'b0;
            end

            // 2. Capture Write Data
            if (~s_axi_wready && s_axi_wvalid && ~w_captured) begin
                s_axi_wready <= 1'b1;
                w_data_reg   <= s_axi_wdata;
                w_strb_reg   <= s_axi_wstrb;
                w_captured   <= 1'b1;
            end else begin
                s_axi_wready <= 1'b0;
            end

            // 3. Commit Write to BRAM Port A when both Address & Data are available
            if ((aw_captured || (s_axi_awvalid && ~s_axi_awready)) &&
                (w_captured  || (s_axi_wvalid  && ~s_axi_wready))  &&
                ~s_axi_bvalid) begin

                porta_write_addr <= aw_captured ? aw_addr_reg : s_axi_awaddr[ADDR_WIDTH+1:2];
                porta_din        <= w_captured  ? w_data_reg  : s_axi_wdata;
                porta_strb       <= w_captured  ? w_strb_reg  : s_axi_wstrb;
                porta_we         <= 1'b1;

                // --- On-the-Fly CRC Accumulation on Port A Ingress ---
                // Only payload words (0 to STATUS_ADDR - 1) are accumulated into CRC
                if ((aw_captured ? aw_addr_reg : s_axi_awaddr[ADDR_WIDTH+1:2]) == 0) begin
                    // First word resets and initializes CRC
                    crc_accum <= next_crc32(32'hFFFFFFFF, w_captured ? w_data_reg : s_axi_wdata);
                end else if ((aw_captured ? aw_addr_reg : s_axi_awaddr[ADDR_WIDTH+1:2]) < STATUS_ADDR) begin
                    // Middle words accumulate running CRC
                    crc_accum <= next_crc32(crc_accum, w_captured ? w_data_reg : s_axi_wdata);
                end

                // --- Hardware Doorbell Trigger at TRIGGER_ADDR ---
                if ((aw_captured ? aw_addr_reg : s_axi_awaddr[ADDR_WIDTH+1:2]) == TRIGGER_ADDR) begin
                    trigger_detected <= 1'b1;
                    trigger_val      <= w_captured ? w_data_reg : s_axi_wdata;
                    expected_crc     <= w_captured ? w_data_reg : s_axi_wdata; // Doorbell payload is expected CRC
                end

                aw_captured  <= 1'b0;
                w_captured   <= 1'b0;
                s_axi_bvalid <= 1'b1;
            end

            // 4. Write Response Handshake
            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    // =========================================================================
    // AXI Slave Read Channel: Separated Full 32-bit CRC & Status Registers
    // =========================================================================
    reg [1:0] ar_state;

    // Multiplex read data between internal registers and BRAM
    wire [31:0] rdata_mux = (porta_read_addr == STATUS_ADDR)  ? {26'b0, crc_err, state, ip_done_irq} :
                            (porta_read_addr == TRIGGER_ADDR) ? crc_accum :
                            porta_dout;

    always @(posedge clk) begin
        if (!rst_n) begin
            s_axi_arready   <= 1'b0;
            s_axi_rvalid    <= 1'b0;
            s_axi_rdata     <= 32'd0;
            porta_read_addr <= 0;
            ar_state        <= 2'd0;
        end else begin
            case (ar_state)
                2'd0: begin
                    // Idle: Accept incoming read address
                    if (~s_axi_arready && s_axi_arvalid && ~s_axi_rvalid) begin
                        s_axi_arready   <= 1'b1;
                        porta_read_addr <= s_axi_araddr[ADDR_WIDTH+1:2];
                        ar_state        <= 2'd1;
                    end else begin
                        s_axi_arready <= 1'b0;
                    end
                end

                2'd1: begin
                    s_axi_arready <= 1'b0;
                    // Wait 1 clock cycle: BRAM synchronous read latches into porta_dout
                    ar_state      <= 2'd2;
                end

                2'd2: begin
                    // BRAM memory data is now fully clocked and settled on porta_dout
                    s_axi_rdata  <= rdata_mux;
                    s_axi_rvalid <= 1'b1;
                    ar_state     <= 2'd3;
                end

                2'd3: begin
                    // Hold rvalid until master completes the handshake
                    if (s_axi_rvalid && s_axi_rready) begin
                        s_axi_rvalid <= 1'b0;
                        ar_state     <= 2'd0;
                    end
                end
            endcase
        end
    end

    // =========================================================================
    // Compute & Egress State Machine
    // =========================================================================
    reg [ADDR_WIDTH:0] comp_ptr;
    reg [ADDR_WIDTH:0] rd_ptr;
    reg [ADDR_WIDTH:0] pkt_len;
    reg [15:0]         work_timer;

    wire status_write  = porta_we && (porta_write_addr == STATUS_ADDR);
    wire trigger_write = porta_we && (porta_write_addr == TRIGGER_ADDR);

    assign m_axis_tkeep = {(DATA_WIDTH/8){1'b1}};
    wire   tx_handshake = m_axis_tvalid && m_axis_tready;

    always @(posedge clk) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            comp_ptr      <= 0;
            rd_ptr        <= 0;
            rx_ptr        <= 0;
            pkt_len       <= STATUS_ADDR;
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
                        if (s_axis_tlast || (rx_ptr == STATUS_ADDR - 1)) begin
                            pkt_len  <= rx_ptr + 1'b1;
                            comp_ptr <= 0;
                            rx_ptr   <= 0;
                            state    <= S_CHECK_CRC;
                        end else begin
                            rx_ptr   <= rx_ptr + 1'b1;
                        end
                    end
                    // Or Doorbell Trigger from CPU AXI-Lite write:
                    else if (trigger_detected || trigger_write) begin
                        pkt_len  <= STATUS_ADDR; // Process all payload words [0 .. 1021]
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
                    if (status_write) begin
                        ip_done_irq <= 1'b0; // Lower interrupt line on CPU ACK
                        if (crc_err) begin
                            crc_err <= 1'b0;
                            rx_ptr  <= 0;
                            state   <= S_IDLE;
                        end else begin
                            state   <= S_WAIT_EGRESS;
                        end
                    end
                    // Abort/Reset Path: Driver writes to TRIGGER_ADDR (0x0FFC) to force reset
                    else if (trigger_detected || trigger_write) begin
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
                    if (status_write) begin
                        rd_ptr <= 0;
                        state  <= S_TX_FETCH;
                    end
                    // Abort/Reset Path: Driver abandons read and forces reset
                    else if (trigger_detected || trigger_write) begin
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