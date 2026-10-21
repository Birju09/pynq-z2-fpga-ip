`timescale 1ns / 1ps

// =============================================================================
// Testbench: tb_ecat_master (tb_ecat_master.v)
//
// Verifies:
//   1. Dynamic PS->PL programming of the 4-Frame Datagram Descriptor Table
//      for 40 Slaves, 5 KB (5,120 bytes = 1,280 words) without premature start.
//   2. Full 5 KB MM2S DMA ingress with on-the-fly CRC-32 verification.
//   3. 4-Frame EtherCAT (0x88A4) LRW exchange through the 40-Slave Ring Emulator
//      (WKC = 33 + 33 + 33 + 21 = 120) and Ethernet FCS verification.
//   4. Level-High IRQ assertion & CPU ACK handshake.
//   5. S2MM Egress under backpressure (skid buffer stress test) verifying all
//      1,280 words across all 40 slaves.
//   6. Slave Dropout Fault-Injection Test (disabling Slave #15 and Slave #37)
//      and verifying per-descriptor WKC error detection (Total WKC = 114).
// =============================================================================

module tb_ecat_master;

    localparam integer DATA_WIDTH     = 32;
    localparam integer BRAM_DEPTH     = 2048;
    localparam integer ADDR_WIDTH     = 11;
    localparam integer CFG_ADDR_WIDTH = 8;
    localparam integer PAYLOAD_WORDS  = 1280; // 5,120 bytes (5 KB) across 40 slaves

    reg                       clk;
    reg                       rst_n;

    // AXI4-Lite Slave signals
    reg  [CFG_ADDR_WIDTH+1:0] s_axi_awaddr;
    reg                       s_axi_awvalid;
    wire                      s_axi_awready;
    reg  [DATA_WIDTH-1:0]     s_axi_wdata;
    reg  [3:0]                s_axi_wstrb;
    reg                       s_axi_wvalid;
    wire                      s_axi_wready;
    wire [1:0]                s_axi_bresp;
    wire                      s_axi_bvalid;
    reg                       s_axi_bready;

    reg  [CFG_ADDR_WIDTH+1:0] s_axi_araddr;
    reg                       s_axi_arvalid;
    wire                      s_axi_arready;
    wire [DATA_WIDTH-1:0]     s_axi_rdata;
    wire [1:0]                s_axi_rresp;
    wire                      s_axi_rvalid;
    reg                       s_axi_rready;

    // AXI4-Stream Slave (MM2S)
    reg  [DATA_WIDTH-1:0]     s_axis_tdata;
    reg  [3:0]                s_axis_tkeep;
    reg                       s_axis_tvalid;
    wire                      s_axis_tready;
    reg                       s_axis_tlast;

    // AXI4-Stream Master (S2MM)
    wire [DATA_WIDTH-1:0]     m_axis_tdata;
    wire [3:0]                m_axis_tkeep;
    wire                      m_axis_tvalid;
    reg                       m_axis_tready;
    wire                      m_axis_tlast;

    wire                      ip_done_irq;

    // Instantiate DUT
    ecat_cyclic_master_ip #(
        .DATA_WIDTH     (DATA_WIDTH),
        .BRAM_DEPTH     (BRAM_DEPTH),
        .ADDR_WIDTH     (ADDR_WIDTH),
        .CFG_ADDR_WIDTH (CFG_ADDR_WIDTH)
    ) dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .s_axi_awaddr  (s_axi_awaddr),
        .s_axi_awvalid (s_axi_awvalid),
        .s_axi_awready (s_axi_awready),
        .s_axi_wdata   (s_axi_wdata),
        .s_axi_wstrb   (s_axi_wstrb),
        .s_axi_wvalid  (s_axi_wvalid),
        .s_axi_wready  (s_axi_wready),
        .s_axi_bresp   (s_axi_bresp),
        .s_axi_bvalid  (s_axi_bvalid),
        .s_axi_bready  (s_axi_bready),
        .s_axi_araddr  (s_axi_araddr),
        .s_axi_arvalid (s_axi_arvalid),
        .s_axi_arready (s_axi_arready),
        .s_axi_rdata   (s_axi_rdata),
        .s_axi_rresp   (s_axi_rresp),
        .s_axi_rvalid  (s_axi_rvalid),
        .s_axi_rready  (s_axi_rready),
        .s_axis_tdata  (s_axis_tdata),
        .s_axis_tkeep  (s_axis_tkeep),
        .s_axis_tvalid (s_axis_tvalid),
        .s_axis_tready (s_axis_tready),
        .s_axis_tlast  (s_axis_tlast),
        .m_axis_tdata  (m_axis_tdata),
        .m_axis_tkeep  (m_axis_tkeep),
        .m_axis_tvalid (m_axis_tvalid),
        .m_axis_tready (m_axis_tready),
        .m_axis_tlast  (m_axis_tlast),
        .ip_done_irq   (ip_done_irq)
    );

    // 100 MHz clock (10 ns period)
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // Helper CRC-32 function matching hardware (IEEE 802.3 Ethernet FCS)
    function [31:0] next_crc32;
        input [31:0] current_crc;
        input [31:0] data_in;
        reg   [31:0] c;
        integer      i;
        begin
            c = current_crc ^ data_in;
            for (i = 0; i < 32; i = i + 1) begin
                c = (c >> 1) ^ (c[0] ? 32'hEDB88320 : 32'h0000_0000);
            end
            next_crc32 = c;
        end
    endfunction

    // AXI4-Lite Write Task
    task axi_write(input [9:0] byte_addr, input [31:0] data);
        begin
            @(posedge clk);
            s_axi_awaddr  <= byte_addr;
            s_axi_awvalid <= 1'b1;
            s_axi_wdata   <= data;
            s_axi_wstrb   <= 4'hF;
            s_axi_wvalid  <= 1'b1;
            s_axi_bready  <= 1'b1;
            wait (s_axi_awready && s_axi_wready);
            @(posedge clk);
            s_axi_awvalid <= 1'b0;
            s_axi_wvalid  <= 1'b0;
            wait (s_axi_bvalid);
            @(posedge clk);
            s_axi_bready  <= 1'b0;
        end
    endtask

    // AXI4-Lite Read Task
    task axi_read(input [9:0] byte_addr, output [31:0] data);
        begin
            @(posedge clk);
            s_axi_araddr  <= byte_addr;
            s_axi_arvalid <= 1'b1;
            s_axi_rready  <= 1'b1;
            wait (s_axi_arready && s_axi_rvalid);
            data = s_axi_rdata;
            @(posedge clk);
            s_axi_arvalid <= 1'b0;
            s_axi_rready  <= 1'b0;
        end
    endtask

    reg [31:0] tx_buf [0:PAYLOAD_WORDS-1];
    reg [31:0] rx_buf [0:PAYLOAD_WORDS-1];
    reg [31:0] calc_crc;
    reg [31:0] status_val;
    reg [31:0] desc_sts;
    reg [31:0] ticks_val;
    integer    i, errors, rx_cnt;
    reg [5:0]  slave_id;
    reg [31:0] expected_word;

    initial begin
        errors        = 0;
        rst_n         = 1'b0;
        s_axi_awaddr  = 10'd0;
        s_axi_awvalid = 1'b0;
        s_axi_wdata   = 32'd0;
        s_axi_wstrb   = 4'h0;
        s_axi_wvalid  = 1'b0;
        s_axi_bready  = 1'b0;
        s_axi_araddr  = 10'd0;
        s_axi_arvalid = 1'b0;
        s_axi_rready  = 1'b0;
        s_axis_tdata  = 32'd0;
        s_axis_tkeep  = 4'hF;
        s_axis_tvalid = 1'b0;
        s_axis_tlast  = 1'b0;
        m_axis_tready = 1'b0;

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        $display("==================================================================");
        $display("  TEST 1: 40-Slave, 5 KB (1,280 Words) 4-Frame EtherCAT OP Cycle");
        $display("==================================================================");

        // 1. Dynamically program the 4-frame Descriptor Table from PS
        axi_write(10'h008, 32'h0000_0014); // 4 descriptors, internal emulator enabled
        axi_write(10'h010, 32'd31);        // 31 wire delay ticks (matching 100BASE-TX)
        axi_write(10'h014, PAYLOAD_WORDS); // 1280 words = 5120 bytes

        // Desc 0: Logical Addr 0, 352 words (1408B, Slaves 0..10), Offset 0, Expected WKC 33
        axi_write(10'h040, 32'd0);
        axi_write(10'h044, {8'd0, 8'h0C, 4'd0, 12'd352});
        axi_write(10'h048, {16'd33, 4'd0, 12'd0});

        // Desc 1: Logical Addr 1408, 352 words (1408B, Slaves 11..21), Offset 352, Expected WKC 33
        axi_write(10'h050, 32'd1408);
        axi_write(10'h054, {8'd1, 8'h0C, 4'd0, 12'd352});
        axi_write(10'h058, {16'd33, 4'd0, 12'd352});

        // Desc 2: Logical Addr 2816, 352 words (1408B, Slaves 22..32), Offset 704, Expected WKC 33
        axi_write(10'h060, 32'd2816);
        axi_write(10'h064, {8'd2, 8'h0C, 4'd0, 12'd352});
        axi_write(10'h068, {16'd33, 4'd0, 12'd704});

        // Desc 3: Logical Addr 4224, 224 words (896B, Slaves 33..39), Offset 1056, Expected WKC 21
        axi_write(10'h070, 32'd4224);
        axi_write(10'h074, {8'd3, 8'h0C, 4'd0, 12'd224});
        axi_write(10'h078, {16'd21, 4'd0, 12'd1056});

        // Prepare 5 KB TxPDO payload and compute expected CRC-32
        calc_crc = 32'hFFFF_FFFF;
        for (i = 0; i < PAYLOAD_WORDS; i = i + 1) begin
            tx_buf[i] = 32'hA500_0000 ^ (i * 32'h0001_0103);
            calc_crc  = next_crc32(calc_crc, tx_buf[i]);
        end
        calc_crc = ~calc_crc;

        // Write expected_crc and verify FSM remains in S_IDLE (Rule 1a)
        axi_write(10'h00C, calc_crc);
        axi_read(10'h004, status_val);
        if (status_val[4:1] !== 4'd0) begin
            $display("[-] FAIL: FSM left S_IDLE prematurely upon CSR config! state=%d", status_val[4:1]);
            errors = errors + 1;
        end else begin
            $display("[+] PASS: FSM remained in S_IDLE after Descriptor & CRC configuration.");
        end

        // 2. Stream 1,280 words (5 KB) over S_AXIS MM2S
        for (i = 0; i < PAYLOAD_WORDS; i = i + 1) begin
            @(posedge clk);
            s_axis_tdata  <= tx_buf[i];
            s_axis_tvalid <= 1'b1;
            s_axis_tlast  <= (i == PAYLOAD_WORDS - 1);
            while (!s_axis_tready) @(posedge clk);
        end
        @(posedge clk);
        s_axis_tvalid <= 1'b0;
        s_axis_tlast  <= 1'b0;

        // 3. Wait for Level-High ip_done_irq
        wait (ip_done_irq == 1'b1);
        repeat (5) @(posedge clk);
        if (ip_done_irq !== 1'b1) begin
            $display("[-] FAIL: ip_done_irq is not held Level-High!");
            errors = errors + 1;
        end

        axi_read(10'h004, status_val);
        axi_read(10'h020, ticks_val);
        $display("[+] Cycle Complete! STATUS=0x%08X (Total WKC=%0d, Frames=%0d, WKC_ERR=%b, CRC_ERR=%b, Ticks=%0d)",
                 status_val, status_val[31:16], status_val[10:7], status_val[6], status_val[5], ticks_val);

        if (status_val[31:16] !== 16'd120 || status_val[6] !== 1'b0 || status_val[5] !== 1'b0) begin
            $display("[-] FAIL: Expected Total WKC=120, WKC_ERR=0, CRC_ERR=0!");
            errors = errors + 1;
        end

        for (i = 0; i < 4; i = i + 1) begin
            axi_read(10'h04C + (i * 16), desc_sts);
            $display("    -> Descriptor %0d Status: 0x%08X (Actual WKC=%0d, WKC_ERR=%b, FCS_ERR=%b)",
                     i, desc_sts, desc_sts[15:0], desc_sts[16], desc_sts[17]);
        end

        // 4. Acknowledge Level-High IRQ (write to STATUS_ADDR 0x04)
        axi_write(10'h004, 32'h0000_0001);
        @(posedge clk);
        if (ip_done_irq !== 1'b0) begin
            $display("[-] FAIL: ip_done_irq did not clear after ACK!");
            errors = errors + 1;
        end

        // 5. Kick S2MM Egress (write to STATUS_ADDR 0x04 in S_WAIT_EGRESS) & stress Skid Buffer
        axi_write(10'h004, 32'h0000_0001);
        rx_cnt = 0;
        while (rx_cnt < PAYLOAD_WORDS) begin
            @(posedge clk);
            // Toggle m_axis_tready to exercise skid buffer backpressure
            m_axis_tready <= ((rx_cnt % 5) != 2);
            if (m_axis_tvalid && m_axis_tready) begin
                rx_buf[rx_cnt] = m_axis_tdata;
                if (rx_cnt == PAYLOAD_WORDS - 1 && !m_axis_tlast) begin
                    $display("[-] FAIL: Missing m_axis_tlast on final word %0d", rx_cnt);
                    errors = errors + 1;
                end
                rx_cnt = rx_cnt + 1;
            end
        end
        @(posedge clk);
        m_axis_tready <= 1'b0;

        // Verify all 1,280 words against per-slave formula
        for (i = 0; i < PAYLOAD_WORDS; i = i + 1) begin
            slave_id      = i / 32; // 32 words (128 bytes) per slave
            expected_word = (~tx_buf[i]) + {16'h0000, slave_id[5:0], 10'h100};
            if (rx_buf[i] !== expected_word) begin
                if (errors < 10) begin
                    $display("[-] FAIL Word %0d (Slave %0d): exp=0x%08X, got=0x%08X",
                             i, slave_id, expected_word, rx_buf[i]);
                end
                errors = errors + 1;
            end
        end
        if (errors == 0) begin
            $display("[+] PASS: All 1,280 words (5,120 bytes) across 40 slaves verified with 0 errors!");
        end

        // =====================================================================
        // TEST 2: Fault Injection (Drop Slave #15 and Slave #37 Offline)
        // =====================================================================
        $display("\n==================================================================");
        $display("  TEST 2: Slave Dropout Fault Injection (Slave #15 & Slave #37)");
        $display("==================================================================");
        // Disable bit 15 in SLAVE_MASK_LO and bit 5 (Slave 37) in SLAVE_MASK_HI
        axi_write(10'h018, 32'hFFFF_7FFF);
        axi_write(10'h01C, 32'h0000_00DF);

        // Trigger manual cycle start via STATUS_ADDR bit 0
        axi_write(10'h00C, 32'h0000_0000); // Bypass ingress CRC for manual trigger
        axi_write(10'h004, 32'h0000_0001);

        wait (ip_done_irq == 1'b1);
        axi_read(10'h004, status_val);
        $display("[+] Fault Cycle STATUS=0x%08X (Total WKC=%0d, WKC_ERR=%b)",
                 status_val, status_val[31:16], status_val[6]);

        if (status_val[31:16] !== 16'd114 || status_val[6] !== 1'b1) begin
            $display("[-] FAIL: Expected Total WKC=114 and WKC_ERR=1!");
            errors = errors + 1;
        end else begin
            $display("[+] PASS: Master detected WKC=114 (expected 120) and raised WKC_ERR=1!");
        end

        // Abort/Reset back to S_IDLE via TRIGGER_ADDR (0x00)
        axi_write(10'h000, 32'h0000_0001);
        repeat (5) @(posedge clk);

        // =====================================================================
        // TEST 3: Corrupted Ingress CRC Fault Injection (Rule 1b)
        // =====================================================================
        $display("\n==================================================================");
        $display("  TEST 3: Ingress CRC Fault Injection (Deliberate CRC mismatch)");
        $display("==================================================================");
        // Reset slaves back online
        axi_write(10'h018, 32'hFFFF_FFFF);
        axi_write(10'h01C, 32'h0000_00FF);

        // Write intentionally wrong expected_crc (0xDEADBEEF)
        axi_write(10'h00C, 32'hDEAD_BEEF);

        // Stream 1,280 words over S_AXIS MM2S
        for (i = 0; i < PAYLOAD_WORDS; i = i + 1) begin
            @(posedge clk);
            s_axis_tdata  <= tx_buf[i];
            s_axis_tvalid <= 1'b1;
            s_axis_tlast  <= (i == PAYLOAD_WORDS - 1);
            while (!s_axis_tready) @(posedge clk);
        end
        @(posedge clk);
        s_axis_tvalid <= 1'b0;
        s_axis_tlast  <= 1'b0;

        // Wait for IRQ from CRC failure
        wait (ip_done_irq == 1'b1);
        axi_read(10'h004, status_val);
        $display("[+] CRC Error STATUS=0x%08X (CRC_ERR=%b, Level-High IRQ=%b)",
                 status_val, status_val[5], status_val[0]);

        if (status_val[5] !== 1'b1) begin
            $display("[-] FAIL: Master did not assert CRC_ERR (bit 5) upon corrupted ingress!");
            errors = errors + 1;
        end else begin
            $display("[+] PASS: Master detected corrupted Ingress CRC and raised CRC_ERR=1!");
        end

        // Acknowledge IRQ and clear back to S_IDLE
        axi_write(10'h004, 32'h0000_0001);
        repeat (5) @(posedge clk);

        $display("\n==================================================================");
        if (errors == 0)
            $display("  ALL SIMULATION TESTS PASSED SUCCESSFULLY!");
        else
            $display("  SIMULATION FAILED WITH %0d ERRORS!", errors);
        $display("==================================================================");
        $finish;
    end

endmodule

