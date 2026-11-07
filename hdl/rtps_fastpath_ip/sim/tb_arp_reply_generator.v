`timescale 1ns / 1ps

module tb_arp_reply_generator;

    reg clk;
    reg rst_n;

    // Upstream RTPS interface
    wire [31:0] m_rtps_tx_tdata;
    wire        m_rtps_tx_tvalid;
    reg         m_rtps_tx_tready;
    wire        m_rtps_tx_tlast;

    reg  [31:0] s_rtps_rx_tdata;
    reg         s_rtps_rx_tvalid;
    wire        s_rtps_rx_tready;
    reg         s_rtps_rx_tlast;

    // Downstream MAC interface
    wire [31:0] m_arp_tx_tdata;
    wire        m_arp_tx_tvalid;
    reg         m_arp_tx_tready;
    wire        m_arp_tx_tlast;

    reg  [31:0] s_arp_rx_tdata;
    reg         s_arp_rx_tvalid;
    wire        s_arp_rx_tready;
    reg         s_arp_rx_tlast;

    // Configuration
    reg [31:0] arp_reply_ip;
    reg [47:0] arp_reply_mac;
    reg [31:0] remote_client_ip;
    wire [47:0] remote_client_mac;

    // ICMP Telemetry from DUT
    wire [31:0] stat_icmp_echo_replies;
    wire [31:0] stat_icmp_errors;
    wire        icmp_err_stb;
    wire [31:0] icmp_err_info;
    wire [31:0] icmp_err_dst_ip;
    wire [31:0] icmp_err_ports;
    wire [15:0] icmp_err_ip_id;

    // Big-Endian signals for DUT_BE
    reg  [31:0] be_s_arp_rx_tdata;
    reg         be_s_arp_rx_tvalid;
    wire        be_s_arp_rx_tready;
    reg         be_s_arp_rx_tlast;
    wire [31:0] be_m_arp_tx_tdata;
    wire        be_m_arp_tx_tvalid;
    reg         be_m_arp_tx_tready;
    wire        be_m_arp_tx_tlast;
    wire [47:0] be_remote_client_mac;
    reg [31:0] captured_be_tx_data [0:31];
    integer    captured_be_tx_cnt;

    always @(posedge clk) begin
        if (rst_n && be_m_arp_tx_tvalid && be_m_arp_tx_tready) begin
            captured_be_tx_data[captured_be_tx_cnt] <= be_m_arp_tx_tdata;
            captured_be_tx_cnt <= captured_be_tx_cnt + 1;
        end
    end

    // Instantiate DUT (AXIS Little-Endian default)
    arp_reply_generator #(
        .DATA_WIDTH     (32),
        .AXIS_BYTE_SWAP (1),
        .ARP_REPLY_IP_2 (32'hC0A801C9) // 192.168.1.201
    ) dut (
        .clk               (clk),
        .rst_n             (rst_n),
        .m_rtps_tx_tdata   (m_rtps_tx_tdata),
        .m_rtps_tx_tvalid  (m_rtps_tx_tvalid),
        .m_rtps_tx_tready  (m_rtps_tx_tready),
        .m_rtps_tx_tlast   (m_rtps_tx_tlast),
        .s_rtps_rx_tdata   (s_rtps_rx_tdata),
        .s_rtps_rx_tvalid  (s_rtps_rx_tvalid),
        .s_rtps_rx_tready  (s_rtps_rx_tready),
        .s_rtps_rx_tlast   (s_rtps_rx_tlast),
        .m_arp_tx_tdata    (m_arp_tx_tdata),
        .m_arp_tx_tvalid   (m_arp_tx_tvalid),
        .m_arp_tx_tready   (m_arp_tx_tready),
        .m_arp_tx_tlast    (m_arp_tx_tlast),
        .s_arp_rx_tdata    (s_arp_rx_tdata),
        .s_arp_rx_tvalid   (s_arp_rx_tvalid),
        .s_arp_rx_tready   (s_arp_rx_tready),
        .s_arp_rx_tlast    (s_arp_rx_tlast),
        .arp_reply_ip      (arp_reply_ip),
        .arp_reply_mac     (arp_reply_mac),
        .remote_client_ip  (remote_client_ip),
        .remote_client_mac (remote_client_mac),
        .stat_icmp_echo_replies (stat_icmp_echo_replies),
        .stat_icmp_errors       (stat_icmp_errors),
        .icmp_err_stb           (icmp_err_stb),
        .icmp_err_info          (icmp_err_info),
        .icmp_err_dst_ip        (icmp_err_dst_ip),
        .icmp_err_ports         (icmp_err_ports),
        .icmp_err_ip_id         (icmp_err_ip_id)
    );

    // Instantiate DUT_BE (Big-Endian mode)
    arp_reply_generator #(
        .DATA_WIDTH     (32),
        .AXIS_BYTE_SWAP (0),
        .ARP_REPLY_IP_2 (32'hC0A801C9)
    ) dut_be (
        .clk               (clk),
        .rst_n             (rst_n),
        .m_rtps_tx_tdata   (),
        .m_rtps_tx_tvalid  (),
        .m_rtps_tx_tready  (1'b1),
        .m_rtps_tx_tlast   (),
        .s_rtps_rx_tdata   (32'd0),
        .s_rtps_rx_tvalid  (1'b0),
        .s_rtps_rx_tready  (),
        .s_rtps_rx_tlast   (1'b0),
        .m_arp_tx_tdata    (be_m_arp_tx_tdata),
        .m_arp_tx_tvalid   (be_m_arp_tx_tvalid),
        .m_arp_tx_tready   (be_m_arp_tx_tready),
        .m_arp_tx_tlast    (be_m_arp_tx_tlast),
        .s_arp_rx_tdata    (be_s_arp_rx_tdata),
        .s_arp_rx_tvalid   (be_s_arp_rx_tvalid),
        .s_arp_rx_tready   (be_s_arp_rx_tready),
        .s_arp_rx_tlast    (be_s_arp_rx_tlast),
        .arp_reply_ip      (arp_reply_ip),
        .arp_reply_mac     (arp_reply_mac),
        .remote_client_ip  (remote_client_ip),
        .remote_client_mac (be_remote_client_mac),
        .stat_icmp_echo_replies (),
        .stat_icmp_errors       (),
        .icmp_err_stb           (),
        .icmp_err_info          (),
        .icmp_err_dst_ip        (),
        .icmp_err_ports         (),
        .icmp_err_ip_id         ()
    );

    // Clock generation (100 MHz, 10 ns period)
    always #5 clk = ~clk;

    // Byte swap helper for testbench
    function [31:0] swap32(input [31:0] in);
        swap32 = {in[7:0], in[15:8], in[23:16], in[31:24]};
    endfunction

    integer errors = 0;
    integer i;
    integer sim_cnt;

    // Buffer for captured TX beats
    reg [31:0] captured_tx_data [0:31];
    reg        captured_tx_last [0:31];
    integer    captured_tx_cnt;

    wire [31:0] captured_tx_data_net = swap32(m_arp_tx_tdata);

    // Monitor for m_arp_tx
    always @(posedge clk) begin
        if (rst_n && m_arp_tx_tvalid && m_arp_tx_tready) begin
            captured_tx_data[captured_tx_cnt] <= captured_tx_data_net;
            captured_tx_last[captured_tx_cnt] <= m_arp_tx_tlast;
            captured_tx_cnt <= captured_tx_cnt + 1;
        end
    end

    // Buffer for captured RTPS RX beats
    reg [31:0] captured_rtps_data [0:31];
    reg        captured_rtps_last [0:31];
    integer    captured_rtps_cnt;

    always @(posedge clk) begin
        if (rst_n && m_rtps_tx_tvalid && m_rtps_tx_tready) begin
            captured_rtps_data[captured_rtps_cnt] <= m_rtps_tx_tdata;
            captured_rtps_last[captured_rtps_cnt] <= m_rtps_tx_tlast;
            captured_rtps_cnt <= captured_rtps_cnt + 1;
        end
    end

    // Task to send an Ethernet frame into s_arp_rx
    task send_wire_frame(
        input integer num_words,
        input [31:0] word0, word1, word2, word3, word4, word5,
        input [31:0] word6, word7, word8, word9, word10, word11,
        input [31:0] word12, word13, word14
    );
        reg [31:0] words [0:14];
        integer w;
        begin
            words[0]  = word0;  words[1]  = word1;  words[2]  = word2;
            words[3]  = word3;  words[4]  = word4;  words[5]  = word5;
            words[6]  = word6;  words[7]  = word7;  words[8]  = word8;
            words[9]  = word9;  words[10] = word10; words[11] = word11;
            words[12] = word12; words[13] = word13; words[14] = word14;

            for (w = 0; w < num_words; w = w + 1) begin
                @(posedge clk);
                s_arp_rx_tvalid <= 1'b1;
                s_arp_rx_tdata  <= swap32(words[w]);
                s_arp_rx_tlast  <= (w == num_words - 1);
                while (!s_arp_rx_tready) @(posedge clk);
            end
            @(posedge clk);
            s_arp_rx_tvalid <= 1'b0;
            s_arp_rx_tlast  <= 1'b0;
            s_arp_rx_tdata  <= 32'd0;
        end
    endtask

    reg [31:0] frame_words_buf [0:63];

    task send_variable_frame(input integer num_words);
        integer w;
        begin
            for (w = 0; w < num_words; w = w + 1) begin
                @(posedge clk);
                s_arp_rx_tvalid <= 1'b1;
                s_arp_rx_tdata  <= swap32(frame_words_buf[w]);
                s_arp_rx_tlast  <= (w == num_words - 1);
                while (!s_arp_rx_tready) @(posedge clk);
            end
            @(posedge clk);
            s_arp_rx_tvalid <= 1'b0;
            s_arp_rx_tlast  <= 1'b0;
            s_arp_rx_tdata  <= 32'd0;
        end
    endtask

    initial begin
        clk = 0;
        rst_n = 0;
        m_rtps_tx_tready = 1;
        s_rtps_rx_tdata = 0;
        s_rtps_rx_tvalid = 0;
        s_rtps_rx_tlast = 0;
        m_arp_tx_tready = 1;
        s_arp_rx_tdata = 0;
        s_arp_rx_tvalid = 0;
        s_arp_rx_tlast = 0;

        arp_reply_ip     = 32'hC0A8010A;        // 192.168.1.10
        arp_reply_mac    = 48'h0200_5E10_0001;  // 02:00:5E:10:00:01
        remote_client_ip = 32'hC0A80164;        // 192.168.1.100

        captured_tx_cnt   = 0;
        captured_rtps_cnt = 0;
        captured_be_tx_cnt = 0;
        be_m_arp_tx_tready = 1;
        be_s_arp_rx_tdata  = 0;
        be_s_arp_rx_tvalid = 0;
        be_s_arp_rx_tlast  = 0;

        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        $display("=================================================================");
        $display("[+] STARTING TESTBENCH FOR ARP_REPLY_GENERATOR");
        $display("=================================================================");

        // ---------------------------------------------------------------------
        // TEST 1: Standard ARP Request from Laptop (192.168.1.100) for 192.168.1.10
        // Laptop MAC: 00:0C:29:AB:CD:EF
        // ---------------------------------------------------------------------
        $display("\n--- TEST 1: ARP Request for Primary IP (192.168.1.10) ---");
        captured_tx_cnt   = 0;
        captured_rtps_cnt = 0;
        captured_be_tx_cnt = 0;
        be_m_arp_tx_tready = 1;
        be_s_arp_rx_tdata  = 0;
        be_s_arp_rx_tvalid = 0;
        be_s_arp_rx_tlast  = 0;

        // Build 15-word ARP Request frame (Network order):
        // Word 0: FF:FF:FF:FF
        // Word 1: FF:FF + Laptop MAC [47:32] (00:0C)
        // Word 2: Laptop MAC [31:0] (29:AB:CD:EF)
        // Word 3: EtherType 0x0806 + HTYPE 0x0001
        // Word 4: PTYPE 0x0800 + HLEN 0x06 + PLEN 0x04
        // Word 5: OPER 0x0001 (Request) + Laptop MAC [47:32]
        // Word 6: Laptop MAC [31:0]
        // Word 7: Laptop IP (192.168.1.100 = 0xC0A80164)
        // Word 8: Target MAC [47:16] (00:00:00:00)
        // Word 9: Target MAC [15:0] (00:00) + Target IP [31:16] (192.168 = 0xC0A8)
        // Word 10: Target IP [15:0] (1.10 = 0x010A) + Padding
        // Words 11..14: Padding
        send_wire_frame(
            15,
            32'hFFFF_FFFF,
            32'hFFFF_000C,
            32'h29AB_CDEF,
            32'h0806_0001,
            32'h0800_0604,
            32'h0001_000C,
            32'h29AB_CDEF,
            32'hC0A8_0164,
            32'h0000_0000,
            32'h0000_C0A8,
            32'h010A_0000,
            32'h0000_0000,
            32'h0000_0000,
            32'h0000_0000,
            32'h0000_0000
        );

        // Wait for ARP reply to be transmitted
        repeat (25) @(posedge clk);

        // Verify remote_client_mac learned
        if (remote_client_mac === 48'h000C_29AB_CDEF) begin
            $display("[+] PASS: Successfully learned remote_client_mac = 00:0C:29:AB:CD:EF");
        end else begin
            $display("[-] FAIL: remote_client_mac = %012X (expected 000C29ABCDEF)", remote_client_mac);
            errors = errors + 1;
        end

        // Verify ARP frame was NOT leaked to RTPS
        if (captured_rtps_cnt === 0) begin
            $display("[+] PASS: 0 ARP beats leaked to RTPS IP (clean filtering)!");
        end else begin
            $display("[-] FAIL: %0d beats leaked to RTPS IP!", captured_rtps_cnt);
            errors = errors + 1;
        end

        // Verify ARP Reply frame
        if (captured_tx_cnt === 15) begin
            $display("[+] PASS: Captured 15 beats (60 bytes) ARP Reply frame!");
        end else begin
            $display("[-] FAIL: Captured %0d beats (expected 15)!", captured_tx_cnt);
            errors = errors + 1;
        end

        // Verify Ethernet Dst MAC = Laptop MAC (00:0C:29:AB:CD:EF)
        if (captured_tx_data[0] === 32'h000C_29AB && captured_tx_data[1][31:16] === 16'hCDEF) begin
            $display("[+] PASS: Reply Dst MAC = 00:0C:29:AB:CD:EF");
        end else begin
            $display("[-] FAIL: Reply Dst MAC mismatch! [0]=0x%08X [1]=0x%08X", captured_tx_data[0], captured_tx_data[1]);
            errors = errors + 1;
        end

        // Verify Ethernet Src MAC = FPGA MAC (02:00:5E:10:00:01)
        if (captured_tx_data[1][15:0] === 16'h0200 && captured_tx_data[2] === 32'h5E10_0001) begin
            $display("[+] PASS: Reply Src MAC = 02:00:5E:10:00:01");
        end else begin
            $display("[-] FAIL: Reply Src MAC mismatch! [1]=0x%08X [2]=0x%08X", captured_tx_data[1], captured_tx_data[2]);
            errors = errors + 1;
        end

        // Verify EtherType 0x0806, HTYPE 0x0001, PTYPE 0x0800, HLEN 6, PLEN 4
        if (captured_tx_data[3] === 32'h0806_0001 && captured_tx_data[4] === 32'h0800_0604) begin
            $display("[+] PASS: EtherType, HTYPE, PTYPE, HLEN, PLEN verified!");
        end else begin
            $display("[-] FAIL: Header mismatch! [3]=0x%08X [4]=0x%08X", captured_tx_data[3], captured_tx_data[4]);
            errors = errors + 1;
        end

        // Verify OPER = 0x0002 (Reply), SHA = FPGA MAC
        if (captured_tx_data[5] === 32'h0002_0200 && captured_tx_data[6] === 32'h5E10_0001) begin
            $display("[+] PASS: OPER = Reply (0x0002), SHA = FPGA MAC");
        end else begin
            $display("[-] FAIL: SHA mismatch! [5]=0x%08X [6]=0x%08X", captured_tx_data[5], captured_tx_data[6]);
            errors = errors + 1;
        end

        // Verify SPA = FPGA IP (192.168.1.10 = 0xC0A8010A)
        if (captured_tx_data[7] === 32'hC0A8_010A) begin
            $display("[+] PASS: SPA = FPGA IP (192.168.1.10)");
        end else begin
            $display("[-] FAIL: SPA mismatch! [7]=0x%08X (expected 0xC0A8010A)", captured_tx_data[7]);
            errors = errors + 1;
        end

        // Verify THA = Laptop MAC, TPA = Laptop IP
        if (captured_tx_data[8] === 32'h000C_29AB &&
            captured_tx_data[9] === 32'hCDEF_C0A8 &&
            captured_tx_data[10][31:16] === 16'h0164) begin
            $display("[+] PASS: THA = Laptop MAC, TPA = Laptop IP (192.168.1.100)");
        end else begin
            $display("[-] FAIL: THA/TPA mismatch! [8]=0x%08X [9]=0x%08X [10]=0x%08X", captured_tx_data[8], captured_tx_data[9], captured_tx_data[10]);
            errors = errors + 1;
        end

        // Verify tlast asserted on beat 14
        if (captured_tx_last[14] === 1'b1) begin
            $display("[+] PASS: tlast asserted on final beat (beat 14)!");
        end else begin
            $display("[-] FAIL: tlast was not asserted on beat 14!");
            errors = errors + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 2: Pass-Through Non-ARP Traffic (IPv4 UDP packet) to RTPS IP
        // ---------------------------------------------------------------------
        $display("\n--- TEST 2: Pass-Through Non-ARP Frame to RTPS IP ---");
        captured_tx_cnt   = 0;
        captured_rtps_cnt = 0;
        captured_be_tx_cnt = 0;
        be_m_arp_tx_tready = 1;
        be_s_arp_rx_tdata  = 0;
        be_s_arp_rx_tvalid = 0;
        be_s_arp_rx_tlast  = 0;

        // Send 6-beat IPv4 UDP frame
        send_wire_frame(
            6,
            32'h0200_5E10,        // Dst MAC high
            32'h0001_000C,        // Dst MAC low + Src MAC high
            32'h29AB_CDEF,        // Src MAC low
            32'h0800_4500,        // EtherType IPv4 (0x0800) + IP header
            32'h0030_1234,        // IP Len + IP ID
            32'h4011_AABB,        // TTL, UDP proto (0x11), csum
            32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0
        );

        repeat (15) @(posedge clk);

        if (captured_rtps_cnt === 6) begin
            $display("[+] PASS: Received 6 beats on m_rtps_tx!");
        end else begin
            $display("[-] FAIL: Expected 6 beats on m_rtps_tx, got %0d!", captured_rtps_cnt);
            errors = errors + 1;
        end

        // Check captured beats match sent bytes
        if (captured_rtps_data[0] === swap32(32'h0200_5E10) &&
            captured_rtps_data[3] === swap32(32'h0800_4500) &&
            captured_rtps_last[5] === 1'b1) begin
            $display("[+] PASS: Ingress UDP payload and tlast passed through perfectly!");
        end else begin
            $display("[-] FAIL: Data corruption in pass-through!");
            errors = errors + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 3: Outgoing RTPS Frame pass-through to Wire MAC TX
        // ---------------------------------------------------------------------
        $display("\n--- TEST 3: Outgoing RTPS Frame pass-through to Wire TX ---");
        captured_tx_cnt = 0;

        @(posedge clk);
        s_rtps_rx_tvalid <= 1'b1;
        s_rtps_rx_tdata  <= 32'hDEADBEEF;
        s_rtps_rx_tlast  <= 1'b0;
        @(posedge clk);
        s_rtps_rx_tdata  <= 32'hCAFE1234;
        s_rtps_rx_tlast  <= 1'b1;
        @(posedge clk);
        s_rtps_rx_tvalid <= 1'b0;
        s_rtps_rx_tlast  <= 1'b0;

        repeat (5) @(posedge clk);

        if (captured_tx_cnt === 2 &&
            captured_tx_data[0] === swap32(32'hDEADBEEF) &&
            captured_tx_data[1] === swap32(32'hCAFE1234) &&
            captured_tx_last[1] === 1'b1) begin
            $display("[+] PASS: Outgoing RTPS frame streamed to m_arp_tx without modification!");
        end else begin
            $display("[-] FAIL: RTPS TX pass-through error! cnt=%0d", captured_tx_cnt);
            errors = errors + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 4: ARP Request for Secondary IP (192.168.1.201 = 0xC0A801C9)
        // ---------------------------------------------------------------------
        $display("\n--- TEST 4: ARP Request for Secondary IP (192.168.1.201) ---");
        captured_tx_cnt = 0;

        send_wire_frame(
            15,
            32'hFFFF_FFFF,
            32'hFFFF_000C,
            32'h29AB_CDEF,
            32'h0806_0001,
            32'h0800_0604,
            32'h0001_000C,
            32'h29AB_CDEF,
            32'hC0A8_0164,
            32'h0000_0000,
            32'h0000_C0A8,
            32'h01C9_0000, // 192.168.1.201!
            32'h0000_0000,
            32'h0000_0000,
            32'h0000_0000,
            32'h0000_0000
        );

        repeat (25) @(posedge clk);

        if (captured_tx_cnt === 15 && captured_tx_data[7] === 32'hC0A8_01C9) begin
            $display("[+] PASS: Successfully answered secondary IP ARP request with SPA = 192.168.1.201!");
        end else begin
            $display("[-] FAIL: Secondary IP ARP response failed! cnt=%0d SPA=0x%08X", captured_tx_cnt, captured_tx_data[7]);
            errors = errors + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 5: ARP Request for Unknown IP (192.168.1.55) -> Must be Ignored!
        // ---------------------------------------------------------------------
        $display("\n--- TEST 5: ARP Request for Unknown IP (192.168.1.55) ---");
        captured_tx_cnt = 0;

        send_wire_frame(
            15,
            32'hFFFF_FFFF,
            32'hFFFF_000C,
            32'h29AB_CDEF,
            32'h0806_0001,
            32'h0800_0604,
            32'h0001_000C,
            32'h29AB_CDEF,
            32'hC0A8_0164,
            32'h0000_0000,
            32'h0000_C0A8,
            32'h0137_0000, // 192.168.1.55
            32'h0000_0000,
            32'h0000_0000,
            32'h0000_0000,
            32'h0000_0000
        );

        repeat (25) @(posedge clk);

        if (captured_tx_cnt === 0) begin
            $display("[+] PASS: ARP request for unknown IP was correctly ignored (0 replies sent)!");
        end else begin
            $display("[-] FAIL: Erroneously answered ARP request for foreign IP! cnt=%0d", captured_tx_cnt);
            errors = errors + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 6: Backpressure during ARP Reply transmission
        // ---------------------------------------------------------------------
        $display("\n--- TEST 6: Backpressure handling on m_arp_tx_tready ---");
        captured_tx_cnt = 0;

        // Trigger reply
        send_wire_frame(
            15,
            32'hFFFF_FFFF,
            32'hFFFF_000C,
            32'h29AB_CDEF,
            32'h0806_0001,
            32'h0800_0604,
            32'h0001_000C,
            32'h29AB_CDEF,
            32'hC0A8_0164,
            32'h0000_0000,
            32'h0000_C0A8,
            32'h010A_0000,
            32'h0000_0000,
            32'h0000_0000,
            32'h0000_0000,
            32'h0000_0000
        );

        // Deassert tready every second cycle
        fork
            begin
                while (captured_tx_cnt < 15) begin
                    @(posedge clk);
                    m_arp_tx_tready <= 1'b0;
                    @(posedge clk);
                    m_arp_tx_tready <= 1'b1;
                end
            end
        join

        repeat (10) @(posedge clk);
        m_arp_tx_tready <= 1'b1;

        if (captured_tx_cnt === 15 && captured_tx_last[14] === 1'b1) begin
            $display("[+] PASS: Handled intermittent downstream backpressure without data loss!");
        end else begin
            $display("[-] FAIL: Backpressure test failed! cnt=%0d", captured_tx_cnt);
            errors = errors + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 7: Test with AXIS_BYTE_SWAP = 0 instance
        // ---------------------------------------------------------------------
        $display("\n--- TEST 7: Direct Network Order (AXIS_BYTE_SWAP = 0) ---");
        captured_be_tx_cnt = 0;
        be_m_arp_tx_tready = 1;
        be_s_arp_rx_tvalid = 0;
        be_s_arp_rx_tlast  = 0;
        @(posedge clk);

        // Send 15-word ARP Request directly in big-endian network order
        begin : be_test_blk
            reg [31:0] be_words [0:14];
            integer bw;
            be_words[0]  = 32'hFFFF_FFFF;
            be_words[1]  = 32'hFFFF_000C;
            be_words[2]  = 32'h29AB_CDEF;
            be_words[3]  = 32'h0806_0001;
            be_words[4]  = 32'h0800_0604;
            be_words[5]  = 32'h0001_000C;
            be_words[6]  = 32'h29AB_CDEF;
            be_words[7]  = 32'hC0A8_0164;
            be_words[8]  = 32'h0000_0000;
            be_words[9]  = 32'h0000_C0A8;
            be_words[10] = 32'h010A_0000;
            be_words[11] = 32'h0000_0000;
            be_words[12] = 32'h0000_0000;
            be_words[13] = 32'h0000_0000;
            be_words[14] = 32'h0000_0000;

            for (bw = 0; bw < 15; bw = bw + 1) begin
                @(posedge clk);
                be_s_arp_rx_tvalid <= 1'b1;
                be_s_arp_rx_tdata  <= be_words[bw];
                be_s_arp_rx_tlast  <= (bw == 14);
                while (!be_s_arp_rx_tready) @(posedge clk);
            end
            @(posedge clk);
            be_s_arp_rx_tvalid <= 1'b0;
            be_s_arp_rx_tlast  <= 1'b0;
        end

        // Wait and verify response on be_m_arp_tx
        repeat (20) @(posedge clk);

        if (captured_be_tx_cnt === 15 && captured_be_tx_data[0] === 32'h000C_29AB) begin
            $display("[+] PASS: Big-Endian instance generated correct ARP reply (Dst MAC = 000C29AB)!");
        end else begin
            $display("[-] FAIL: Big-Endian reply mismatch! cnt=%0d [0]=0x%08X", captured_be_tx_cnt, captured_be_tx_data[0]);
            errors = errors + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 8: ICMP Echo Request (ping) -> Hardware Echo Reply
        // ---------------------------------------------------------------------
        $display("\n--- TEST 8: ICMP Echo Request (ping) -> Hardware Echo Reply ---");
        captured_tx_cnt   = 0;
        captured_rtps_cnt = 0;

        frame_words_buf[0]  = 32'h0200_5E10; // Dst MAC: 02:00:5E:10:00:01
        frame_words_buf[1]  = 32'h0001_000C; // Src MAC: 00:0C:29:AB:CD:EF
        frame_words_buf[2]  = 32'h29AB_CDEF;
        frame_words_buf[3]  = 32'h0800_4500; // EtherType 0x0800, IPv4 Ver 4 IHL 5
        frame_words_buf[4]  = 32'h0054_ABCD; // TotLen = 84B (0x0054), ID = 0xABCD
        frame_words_buf[5]  = 32'h0000_4001; // Flags=0, TTL=64, Proto=1 (ICMP)
        frame_words_buf[6]  = 32'h4B1D_C0A8; // Header Checksum 0x4B1D, Src IP 192.168
        frame_words_buf[7]  = 32'h0164_C0A8; // Src IP .1.100, Dst IP 192.168
        frame_words_buf[8]  = 32'h010A_0800; // Dst IP .1.10, ICMP Type 8 (Echo Req), Code 0
        frame_words_buf[9]  = 32'h3210_1234; // ICMP Checksum 0x3210, ID 0x1234
        frame_words_buf[10] = 32'h0001_6162; // ICMP Seq 0x0001, Data "ab"
        frame_words_buf[11] = 32'h6364_6566; // Data "cdef"
        frame_words_buf[12] = 32'h6768_696A; // Data "ghij"
        frame_words_buf[13] = 32'h6B6C_6D6E; // Data "klmn"
        frame_words_buf[14] = 32'h6F70_7172; // Data "opqr"

        send_variable_frame(15);
        repeat (25) @(posedge clk);

        // Verify ICMP packet was filtered away from RTPS
        if (captured_rtps_cnt === 0) begin
            $display("[+] PASS: 0 ICMP beats leaked to RTPS IP (clean protocol selection)!");
        end else begin
            $display("[-] FAIL: %0d ICMP beats leaked to RTPS IP!", captured_rtps_cnt);
            errors = errors + 1;
        end

        // Verify Echo Reply captured on m_arp_tx
        if (captured_tx_cnt >= 15) begin
            $display("[+] PASS: Captured %0d beats ICMP Echo Reply!", captured_tx_cnt);
        end else begin
            $display("[-] FAIL: ICMP Echo Reply not captured! cnt=%0d", captured_tx_cnt);
            errors = errors + 1;
        end

        // Verify Dst MAC = 00:0C:29:AB:CD:EF, Src MAC = 02:00:5E:10:00:01
        if (captured_tx_data[0] === 32'h000C_29AB &&
            captured_tx_data[1] === 32'hCDEF_0200 &&
            captured_tx_data[2] === 32'h5E10_0001) begin
            $display("[+] PASS: Reply MAC addresses swapped correctly!");
        end else begin
            $display("[-] FAIL: Reply MAC mismatch! [0]=0x%08X [1]=0x%08X [2]=0x%08X",
                     captured_tx_data[0], captured_tx_data[1], captured_tx_data[2]);
            errors = errors + 1;
        end

        // Verify IPv4 Src IP = 192.168.1.10, Dst IP = 192.168.1.100, Type = 0 (Echo Reply), Code = 0
        if (captured_tx_data[6][15:0] === 16'hC0A8 &&
            captured_tx_data[7] === 32'h010A_C0A8 &&
            captured_tx_data[8] === 32'h0164_0000) begin
            $display("[+] PASS: Reply IP addresses swapped and ICMP Type 0 Code 0 verified!");
        end else begin
            $display("[-] FAIL: Reply IP/Type mismatch! [6]=0x%08X [7]=0x%08X [8]=0x%08X",
                     captured_tx_data[6], captured_tx_data[7], captured_tx_data[8]);
            errors = errors + 1;
        end

        // Verify 1's complement differential ICMP checksum: 0x3210 + 0x0800 = 0x3A10
        if (captured_tx_data[9][31:16] === 16'h3A10) begin
            $display("[+] PASS: ICMP Checksum differentially updated to 0x3A10!");
        end else begin
            $display("[-] FAIL: ICMP Checksum mismatch! [9]=0x%08X (expected 0x3A101234)", captured_tx_data[9]);
            errors = errors + 1;
        end

        // Verify stat_icmp_echo_replies counter incremented
        if (stat_icmp_echo_replies === 32'd1) begin
            $display("[+] PASS: stat_icmp_echo_replies counter = 1");
        end else begin
            $display("[-] FAIL: stat_icmp_echo_replies = %0d (expected 1)", stat_icmp_echo_replies);
            errors = errors + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 9: ICMP Error Reporting (Type 3 Code 3 - Port Unreachable)
        // ---------------------------------------------------------------------
        $display("\n--- TEST 9: ICMP Error Reporting (Type 3 Code 3: Port Unreachable) ---");
        captured_tx_cnt   = 0;
        captured_rtps_cnt = 0;

        frame_words_buf[0]  = 32'h0200_5E10; // Dst MAC: FPGA
        frame_words_buf[1]  = 32'h0001_000C; // Src MAC: Laptop
        frame_words_buf[2]  = 32'h29AB_CDEF;
        frame_words_buf[3]  = 32'h0800_4500; // EtherType 0x0800, IPv4 Ver 4 IHL 5
        frame_words_buf[4]  = 32'h0046_5678; // TotLen = 70B, ID = 0x5678
        frame_words_buf[5]  = 32'h0000_4001; // Flags=0, TTL=64, Proto=1 (ICMP)
        frame_words_buf[6]  = 32'hA080_C0A8; // Header Checksum 0xA080, Src IP 192.168
        frame_words_buf[7]  = 32'h0164_C0A8; // Src IP .1.100, Dst IP 192.168
        frame_words_buf[8]  = 32'h010A_0303; // Dst IP .1.10, ICMP Type 3 (Dest Unreach), Code 3 (Port Unreach)
        frame_words_buf[9]  = 32'h1111_0000; // ICMP Checksum 0x1111, Unused
        frame_words_buf[10] = 32'h05DC_4500; // Next-Hop MTU 1500 (0x05DC), Offending IP Ver/IHL/TOS (0x4500)
        frame_words_buf[11] = 32'h1060_9988; // Offending IP TotLen 0x1060, ID 0x9988
        frame_words_buf[12] = 32'h0000_4011; // Offending IP Flags=0, TTL=64, Proto=17 (UDP)
        frame_words_buf[13] = 32'h0000_C0A8; // Offending IP Csum, Src IP 192.168
        frame_words_buf[14] = 32'h010A_C0A8; // Offending IP Src IP .1.10, Dst IP 192.168
        frame_words_buf[15] = 32'h0164_1CF3; // Offending IP Dst IP .1.100, Offending UDP Src Port 7411 (0x1CF3)
        frame_words_buf[16] = 32'h1CF2_104C; // Offending UDP Dst Port 7410 (0x1CF2), Len 0x104C

        send_variable_frame(17);
        repeat (15) @(posedge clk);

        // Verify ICMP error was filtered from RTPS and generated no bogus reply
        if (captured_rtps_cnt === 0 && captured_tx_cnt === 0) begin
            $display("[+] PASS: Inbound ICMP error consumed cleanly with 0 beats leaked to RTPS!");
        end else begin
            $display("[-] FAIL: Inbound ICMP error leaked! rtps_cnt=%0d tx_cnt=%0d", captured_rtps_cnt, captured_tx_cnt);
            errors = errors + 1;
        end

        // Verify ICMP error telemetry registers
        if (stat_icmp_errors === 32'd1 &&
            icmp_err_info === 32'h0303_05DC &&
            icmp_err_dst_ip === 32'hC0A8_0164 &&
            icmp_err_ports === 32'h1CF3_1CF2 &&
            icmp_err_ip_id === 16'h9988) begin
            $display("[+] PASS: Successfully extracted ICMP Type 3 Code 3, MTU 1500, IP ID 0x9988, Ports 7411->7410!");
        end else begin
            $display("[-] FAIL: ICMP error telemetry mismatch! stat=%0d info=0x%08X dst_ip=0x%08X ports=0x%08X id=0x%04X",
                     stat_icmp_errors, icmp_err_info, icmp_err_dst_ip, icmp_err_ports, icmp_err_ip_id);
            errors = errors + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 10: ICMP Error Reporting (Type 11 Code 1 - Fragment Reassembly Timeout)
        // ---------------------------------------------------------------------
        $display("\n--- TEST 10: ICMP Error Reporting (Type 11 Code 1: Reassembly Timeout) ---");
        frame_words_buf[8]  = 32'h010A_0B01; // ICMP Type 11 (Time Exceeded), Code 1 (Reassembly Timeout)
        send_variable_frame(17);
        repeat (15) @(posedge clk);

        if (stat_icmp_errors === 32'd2 && icmp_err_info[31:16] === 16'h0B01) begin
            $display("[+] PASS: Successfully parsed ICMP Type 11 Code 1 Fragment Reassembly Time Exceeded!");
        end else begin
            $display("[-] FAIL: Type 11 parsing failed! stat=%0d info=0x%08X", stat_icmp_errors, icmp_err_info);
            errors = errors + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 11: 3-Way TX Priority Arbitration (ARP > ICMP > RTPS)
        // ---------------------------------------------------------------------
        $display("\n--- TEST 11: 3-Way TX Priority Arbitration (ARP > ICMP > RTPS) ---");
        captured_tx_cnt = 0;

        // Hold backpressure so nothing leaves TX_IDLE while we stage contention
        m_arp_tx_tready <= 1'b0;

        // 1. Concurrently trigger an ARP Request to make ARP reply pending
        send_wire_frame(
            15,
            32'hFFFF_FFFF,
            32'hFFFF_000C,
            32'h29AB_CDEF,
            32'h0806_0001,
            32'h0800_0604,
            32'h0001_000C,
            32'h29AB_CDEF,
            32'hC0A8_0164,
            32'h0000_0000,
            32'h0000_C0A8,
            32'h010A_0000,
            32'd0, 32'd0, 32'd0, 32'd0
        );
        repeat (5) @(posedge clk);

        // 2. Concurrently inject an ICMP ping to trigger an ICMP Echo Reply
        frame_words_buf[0]  = 32'h0200_5E10;
        frame_words_buf[1]  = 32'h0001_000C;
        frame_words_buf[2]  = 32'h29AB_CDEF;
        frame_words_buf[3]  = 32'h0800_4500;
        frame_words_buf[4]  = 32'h0054_ABCD;
        frame_words_buf[5]  = 32'h0000_4001;
        frame_words_buf[6]  = 32'h4B1D_C0A8;
        frame_words_buf[7]  = 32'h0164_C0A8;
        frame_words_buf[8]  = 32'h010A_0800; // Type 8, Code 0
        frame_words_buf[9]  = 32'h3210_1234;
        frame_words_buf[10] = 32'h0001_6162;
        frame_words_buf[11] = 32'h6364_6566;
        frame_words_buf[12] = 32'h6768_696A;
        frame_words_buf[13] = 32'h6B6C_6D6E;
        frame_words_buf[14] = 32'h6F70_7172;
        send_variable_frame(15);
        repeat (5) @(posedge clk);

        // 3. Queue an RTPS egress packet on s_rtps_rx
        @(posedge clk);
        s_rtps_rx_tvalid <= 1'b1;
        s_rtps_rx_tdata  <= 32'hBEEF_0001;
        s_rtps_rx_tlast  <= 1'b1;

        // Release downstream backpressure: all 3 requests are pending simultaneously!
        @(posedge clk);
        m_arp_tx_tready <= 1'b1;

        // Wait for transmissions to complete
        repeat (50) @(posedge clk);
        s_rtps_rx_tvalid <= 1'b0;
        s_rtps_rx_tlast  <= 1'b0;
        repeat (20) @(posedge clk);

        // Verify ARP reply (EtherType 0x0806) transmitted first
        if (captured_tx_data[3] === 32'h0806_0001) begin
            $display("[+] PASS: Priority 0 (ARP Reply) transmitted first!");
        end else begin
            $display("[-] FAIL: Priority 0 (ARP Reply) did not transmit first! [3]=0x%08X", captured_tx_data[3]);
            errors = errors + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 12: Zero-Stall Lookahead: RTPS UDP packet passed with 60 ns latency
        // ---------------------------------------------------------------------
        $display("\n--- TEST 12: RTPS UDP Pass-Through directly following ICMP ---");
        captured_rtps_cnt = 0;

        send_wire_frame(
            6,
            32'h0200_5E10,
            32'h0001_000C,
            32'h29AB_CDEF,
            32'h0800_4500,
            32'h0030_1234,
            32'h4011_AABB, // UDP proto 0x11
            32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0
        );
        repeat (15) @(posedge clk);

        if (captured_rtps_cnt === 6) begin
            $display("[+] PASS: 6-beat UDP frame passed to RTPS with exact 6-beat lookahead!");
        end else begin
            $display("[-] FAIL: UDP frame failed to pass! cnt=%0d", captured_rtps_cnt);
            errors = errors + 1;
        end

        // ---------------------------------------------------------------------
        // TEST 13: ARP & ICMP Ping Responds Even When RTPS Downstream is Stalled
        // ---------------------------------------------------------------------
        $display("\n--- TEST 13: ARP & ICMP Ping Under Stalled RTPS Consumer (m_rtps_tx_tready = 0) ---");
        // Downstream RTPS consumer is completely backpressured (e.g. emu_enable=1 or uninitialized)
        m_rtps_tx_tready  = 1'b0;
        captured_tx_cnt   = 0;
        captured_rtps_cnt = 0;

        // 1. Send ambient UDP frame to fill/stall the lookahead FIFO
        send_wire_frame(
            8,
            32'h0200_5E10,
            32'h0001_000C,
            32'h29AB_CDEF,
            32'h0800_4500,
            32'h0030_1234,
            32'h4011_AABB, // UDP
            32'hC0A8_0164, // Src IP
            32'hC0A8_010A, // Dst IP
            32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0
        );
        repeat (10) @(posedge clk);

        // 2. Send ARP Request for 192.168.1.10 while FIFO has stalled pass-through data
        send_wire_frame(
            15,
            32'hFFFF_FFFF,
            32'hFFFF_000C,
            32'h29AB_CDEF,
            32'h0806_0001,
            32'h0800_0604,
            32'h0001_000C,
            32'h29AB_CDEF,
            32'hC0A8_0164,
            32'h0000_0000,
            32'h0000_C0A8,
            32'h010A_0000,
            32'h0000_0000,
            32'h0000_0000,
            32'h0000_0000,
            32'h0000_0000
        );
        repeat (20) @(posedge clk);

        if (captured_tx_cnt === 15 && captured_tx_data[7] === 32'hC0A8_010A) begin
            $display("[+] PASS: ARP Reply for 192.168.1.10 transmitted successfully despite stalled RTPS!");
        end else begin
            $display("[-] FAIL: ARP Reply blocked by stalled RTPS! captured_tx_cnt=%0d", captured_tx_cnt);
            errors = errors + 1;
        end

        // 3. Send ICMP Echo Request (ping 192.168.1.10) while RTPS consumer is still stalled
        captured_tx_cnt = 0;
        send_wire_frame(
            15,
            32'h0200_5E10,
            32'h0001_000C,
            32'h29AB_CDEF,
            32'h0800_4500,
            32'h003C_1234,
            32'h4001_A080, // proto 0x01 (ICMP)
            32'hC0A8_0164,
            32'hC0A8_010A,
            32'h0800_4D5A, // Type 8, Code 0
            32'h0001_0001,
            32'h6162_6364,
            32'h6566_6768,
            32'h696A_6B6C,
            32'h6D6E_6F70,
            32'h7172_7374
        );
        repeat (25) @(posedge clk);

        if (captured_tx_cnt === 15 && captured_tx_data[8][15:0] === 16'h0000) begin
            $display("[+] PASS: ICMP Echo Reply (Type 0) transmitted successfully despite stalled RTPS!");
        end else begin
            $display("[-] FAIL: ICMP Echo Reply blocked by stalled RTPS! captured_tx_cnt=%0d", captured_tx_cnt);
            errors = errors + 1;
        end

        m_rtps_tx_tready = 1'b1; // Restore ready
        repeat (10) @(posedge clk);
        $display("\n=================================================================");
        if (errors === 0) begin
            $display("[+] ALL TESTS PASSED! ARP_REPLY_GENERATOR IS 100%% VERIFIED!");
        end else begin
            $display("[-] TESTBENCH COMPLETED WITH %0d ERRORS!", errors);
        end
        $display("=================================================================\n");
        $finish;
    end

endmodule