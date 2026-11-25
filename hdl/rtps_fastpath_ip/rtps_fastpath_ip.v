`timescale 1ns / 1ps

// =============================================================================
// Module: rtps_fastpath_ip (rtps_fastpath_ip.v)
//
// Description:
//   Thin structural top-level wrapper for the Hybrid PS+PL RTPS v2.x
//   Best-Effort Cyclic Data-Plane Hardware Accelerator.
//   Hosts:
//     - 2-stage local reset pipeline (isolates global proc_sys_reset fanout)
//     - 3-Slot Reassembly BRAM & single-writer Slot Arbiter
//     - Submodules:
//         * u_csr_edt      (rtps_csr_edt)
//         * u_wire_mux     (rtps_wire_mux -> u_rtps_peer_emulator)
//         * u_tx_engine    (rtps_tx_engine -> u_tx_csum_pipe)
//         * u_rx_reasm     (rtps_rx_reasm)
//         * u_s2mm_egress  (rtps_s2mm_egress)
// =============================================================================

module rtps_fastpath_ip #(
    parameter integer DATA_WIDTH        = 32,
    parameter integer BRAM_DEPTH        = 2048,
    parameter integer ADDR_WIDTH        = 11,
    parameter integer CFG_DEPTH         = 256,
    parameter integer CFG_ADDR_WIDTH    = 8,
    parameter integer NUM_ENDPOINTS     = 4,
    parameter integer REASM_DEPTH       = 1536,
    parameter integer CLK_PERIOD_NS     = 10,
    parameter integer USE_EXT_TIMESTAMP = 0
)(
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire [63:0]               ext_timestamp_ns,

    // -------------------------------------------------------------------------
    // 1. AXI4-Lite Memory-Mapped Slave (CSRs + Endpoint Descriptor Table)
    // -------------------------------------------------------------------------
    input  wire [CFG_ADDR_WIDTH+1:0] s_axi_awaddr,
    input  wire                      s_axi_awvalid,
    output wire                      s_axi_awready,
    input  wire [DATA_WIDTH-1:0]     s_axi_wdata,
    input  wire [(DATA_WIDTH/8)-1:0] s_axi_wstrb,
    input  wire                      s_axi_wvalid,
    output wire                      s_axi_wready,
    output wire [1:0]                s_axi_bresp,
    output wire                      s_axi_bvalid,
    input  wire                      s_axi_bready,

    input  wire [CFG_ADDR_WIDTH+1:0] s_axi_araddr,
    input  wire                      s_axi_arvalid,
    output wire                      s_axi_arready,
    output wire [DATA_WIDTH-1:0]     s_axi_rdata,
    output wire [1:0]                s_axi_rresp,
    output wire                      s_axi_rvalid,
    input  wire                      s_axi_rready,

    // -------------------------------------------------------------------------
    // 2. AXI4-Stream Slave (DMA0 MM2S: PS Command Sample -> TX Sample BRAM)
    // -------------------------------------------------------------------------
    input  wire [DATA_WIDTH-1:0]     s_axis_tdata,
    input  wire [(DATA_WIDTH/8)-1:0] s_axis_tkeep,
    input  wire                      s_axis_tvalid,
    output wire                      s_axis_tready,
    input  wire                      s_axis_tlast,

    // -------------------------------------------------------------------------
    // 3. AXI4-Stream Master (DMA0 S2MM: RX Reassembly Slot -> PS Feedback Sample)
    // -------------------------------------------------------------------------
    output wire [DATA_WIDTH-1:0]     m_axis_tdata,
    output wire [(DATA_WIDTH/8)-1:0] m_axis_tkeep,
    output wire                      m_axis_tvalid,
    input  wire                      m_axis_tready,
    output wire                      m_axis_tlast,

    // -------------------------------------------------------------------------
    // 4. AXI4-Stream Wire Port (DMA1 Virtual MAC / External RMII MAC)
    // -------------------------------------------------------------------------
    output wire [DATA_WIDTH-1:0]     m_axis_wire_tdata,
    output wire                      m_axis_wire_tvalid,
    input  wire                      m_axis_wire_tready,
    output wire                      m_axis_wire_tlast,

    input  wire [DATA_WIDTH-1:0]     s_axis_wire_tdata,
    input  wire                      s_axis_wire_tvalid,
    output wire                      s_axis_wire_tready,
    input  wire                      s_axis_wire_tlast,

    // -------------------------------------------------------------------------
    // 5. Dedicated Level-High Interrupt Wire to PS GIC (IRQ_F2P[2])
    // -------------------------------------------------------------------------
    output wire                      ip_done_irq,

    // -------------------------------------------------------------------------
    // 6. ARP Offload & Peer MAC Resolution Interface
    // -------------------------------------------------------------------------
    output wire [31:0]               arp_reply_ip,
    output wire [47:0]               arp_reply_mac,
    output wire [31:0]               remote_client_ip,
    input  wire [47:0]               remote_client_mac,
    output wire [47:0]               ingress_remote_client_mac,

    // -------------------------------------------------------------------------
    // 7. ICMP Engine Telemetry & Error Reporting Interface
    // -------------------------------------------------------------------------
    input  wire [31:0]               stat_icmp_echo_replies,
    input  wire [31:0]               stat_icmp_errors,
    input  wire                      icmp_err_stb,
    input  wire [31:0]               icmp_err_info,
    input  wire [31:0]               icmp_err_dst_ip,
    input  wire [31:0]               icmp_err_ports,
    input  wire [15:0]               icmp_err_ip_id
);

    `include "rtps_fastpath_defs.vh"

    // =========================================================================
    // Local 2-stage reset pipeline to isolate global proc_sys_reset fanout
    // =========================================================================
    (* DONT_TOUCH = "yes" *) reg rst_n_pipe1;
    (* max_fanout = 64 *)    reg rst_n_local;
    always @(posedge clk) begin
        rst_n_pipe1 <= rst_n;
        rst_n_local <= rst_n_pipe1;
    end

    // =========================================================================
    // Inter-Module Control, Telemetry, and Stream Wires
    // =========================================================================
    wire [31:0] ctrl_reg;
    wire [31:0] expected_crc;
    wire [31:0] cfg_pacing_timer;
    wire [31:0] cfg_payload_words;
    wire [31:0] cfg_frag_size;
    wire [31:0] rx_timeout_limit;
    wire [95:0] local_guid_prefix;
    wire [31:0] local_src_ip;
    wire [47:0] local_mac;
    wire [15:0] local_vendor_id;
    wire [63:0] ts_offset;
    wire [31:0] fault_inject_reg;
    wire [63:0] active_time_ns;
    wire        lifecycle_ok_q;

    wire        status_write_pulse;
    wire [31:0] status_write_data;
    wire        trigger_detected;
    wire        edt_commit_applied;
    wire        identity_clear_seq_pulse;

    wire [31:0]                   wr0_dst_ip;
    wire [15:0]                   wr0_src_port;
    wire [15:0]                   wr0_dst_port;
    wire [47:0]                   wr0_dst_mac;
    wire [31:0]                   wr0_writer_id;
    wire [31:0]                   wr0_reader_id;
    wire [NUM_ENDPOINTS-1:0]      wr_valid_vec;
    wire [NUM_ENDPOINTS*32-1:0]   wr_seq_seed_flat;
    wire [NUM_ENDPOINTS*7*32-1:0] rd_edt_flat;

    wire [3:0]                    state;
    wire [1:0]                    rx_egress_state;
    wire                          egress_active;
    wire                          tx_done_irq;
    wire                          rx_done_irq;
    assign ip_done_irq = tx_done_irq | rx_done_irq;
    wire [5:0]                    rx_state;
    wire [31:0]                   crc_accum;
    wire [31:0]                   last_cycle_ticks;
    wire [15:0]                   completed_frags;
    wire [31:0]                   stat_tx_samples;
    wire                          err_crc;
    wire                          err_no_match;
    wire                          err_csum;
    wire                          irq_is_rx = rx_done_irq;

    wire [11:0]                   latest_payload_word_off;
    wire [11:0]                   latest_rtps_words;
    wire [31:0]                   stat_rx_samples;
    wire [31:0]                   stat_seq_gaps;
    wire [31:0]                   stat_stale_drops;
    wire [31:0]                   stat_csum_errs;
    wire [31:0]                   stat_skipped_submsg;
    wire [63:0]                   last_rx_seq_num;
    wire [63:0]                   last_rx_ts;
    wire [31:0]                   last_rx_src_ip;
    wire [15:0]                   last_rx_src_port;
    wire [15:0]                   last_rx_dst_port;
    wire [1:0]                    last_rx_cam_idx;
    wire [47:0]                   last_rx_src_mac;
    wire [NUM_ENDPOINTS*32-1:0]   rd_match_count_flat;

    wire [DATA_WIDTH-1:0] eng_tx_tdata;
    wire                  eng_tx_tvalid;
    wire                  eng_tx_tready;
    wire                  eng_tx_tlast;

    wire [DATA_WIDTH-1:0] eng_rx_tdata;
    wire                  eng_rx_tvalid;
    wire                  eng_rx_tready;
    wire                  eng_rx_tlast;

    // External ARP / MAC Resolution Outputs
    assign arp_reply_ip              = local_src_ip;
    assign arp_reply_mac             = local_mac;
    assign remote_client_ip          = wr0_dst_ip;
    assign ingress_remote_client_mac = last_rx_src_mac;

    // =========================================================================
    // 3-Slot Store-and-Forward Reassembly BRAM & Single-Writer Slot Arbiter
    // =========================================================================
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] reasm_ram_0 [0:REASM_DEPTH-1];
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] reasm_ram_1 [0:REASM_DEPTH-1];
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] reasm_ram_2 [0:REASM_DEPTH-1];

    reg [1:0] fill_slot;
    reg [1:0] latest_slot;
    reg [1:0] egress_slot;
    reg       latest_valid;

    wire        reasm_wr_en;
    wire [3:0]  reasm_wr_be;
    wire [11:0] reasm_wr_addr;
    wire [31:0] reasm_wr_data;
    wire [11:0] walker_rd_addr;
    wire [11:0] s2mm_rd_addr;

    reg  [31:0] reasm_dout_0, reasm_dout_1, reasm_dout_2;
    wire [11:0] slot0_rd_addr = (egress_active && egress_slot == 2'd0) ? s2mm_rd_addr : walker_rd_addr;
    wire [11:0] slot1_rd_addr = (egress_active && egress_slot == 2'd1) ? s2mm_rd_addr : walker_rd_addr;
    wire [11:0] slot2_rd_addr = (egress_active && egress_slot == 2'd2) ? s2mm_rd_addr : walker_rd_addr;

    always @(posedge clk) begin
        if (reasm_wr_en) begin
            case (fill_slot)
                2'd0: begin
                    if (reasm_wr_be[3]) reasm_ram_0[reasm_wr_addr][31:24] <= reasm_wr_data[31:24];
                    if (reasm_wr_be[2]) reasm_ram_0[reasm_wr_addr][23:16] <= reasm_wr_data[23:16];
                    if (reasm_wr_be[1]) reasm_ram_0[reasm_wr_addr][15:8]  <= reasm_wr_data[15:8];
                    if (reasm_wr_be[0]) reasm_ram_0[reasm_wr_addr][7:0]   <= reasm_wr_data[7:0];
                end
                2'd1: begin
                    if (reasm_wr_be[3]) reasm_ram_1[reasm_wr_addr][31:24] <= reasm_wr_data[31:24];
                    if (reasm_wr_be[2]) reasm_ram_1[reasm_wr_addr][23:16] <= reasm_wr_data[23:16];
                    if (reasm_wr_be[1]) reasm_ram_1[reasm_wr_addr][15:8]  <= reasm_wr_data[15:8];
                    if (reasm_wr_be[0]) reasm_ram_1[reasm_wr_addr][7:0]   <= reasm_wr_data[7:0];
                end
                default: begin
                    if (reasm_wr_be[3]) reasm_ram_2[reasm_wr_addr][31:24] <= reasm_wr_data[31:24];
                    if (reasm_wr_be[2]) reasm_ram_2[reasm_wr_addr][23:16] <= reasm_wr_data[23:16];
                    if (reasm_wr_be[1]) reasm_ram_2[reasm_wr_addr][15:8]  <= reasm_wr_data[15:8];
                    if (reasm_wr_be[0]) reasm_ram_2[reasm_wr_addr][7:0]   <= reasm_wr_data[7:0];
                end
            endcase
        end
        reasm_dout_0 <= reasm_ram_0[slot0_rd_addr];
        reasm_dout_1 <= reasm_ram_1[slot1_rd_addr];
        reasm_dout_2 <= reasm_ram_2[slot2_rd_addr];
    end

    wire [31:0] reasm_walker_dout = (fill_slot == 2'd0) ? reasm_dout_0 :
                                    (fill_slot == 2'd1) ? reasm_dout_1 : reasm_dout_2;
    wire [31:0] s2mm_dout         = (egress_slot == 2'd0) ? reasm_dout_0 :
                                    (egress_slot == 2'd1) ? reasm_dout_1 : reasm_dout_2;

    function [1:0] pick_free_slot;
        input [1:0] busy_a;
        input [1:0] busy_b;
        begin
            if (busy_a != 2'd0 && busy_b != 2'd0)
                pick_free_slot = 2'd0;
            else if (busy_a != 2'd1 && busy_b != 2'd1)
                pick_free_slot = 2'd1;
            else
                pick_free_slot = 2'd2;
        end
    endfunction

    wire rx_commit_pulse;
    wire rx_csum_err_pulse;
    wire slot_consume_pulse;

    always @(posedge clk) begin
        if (!rst_n_local) begin
            fill_slot    <= 2'd0;
            latest_slot  <= 2'd1;
            egress_slot  <= 2'd2;
            latest_valid <= 1'b0;
        end else begin
            if (rx_commit_pulse && slot_consume_pulse) begin
                egress_slot  <= latest_slot;
                latest_slot  <= fill_slot;
                latest_valid <= 1'b1;
                fill_slot    <= pick_free_slot(fill_slot, latest_slot);
            end else if (rx_commit_pulse) begin
                latest_slot  <= fill_slot;
                latest_valid <= 1'b1;
                fill_slot    <= pick_free_slot(fill_slot, egress_slot);
            end else if (slot_consume_pulse) begin
                egress_slot  <= latest_slot;
                latest_valid <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Submodule Instantiations
    // =========================================================================
    rtps_csr_edt #(
        .DATA_WIDTH        (DATA_WIDTH),
        .BRAM_DEPTH        (BRAM_DEPTH),
        .CFG_ADDR_WIDTH    (CFG_ADDR_WIDTH),
        .NUM_ENDPOINTS     (NUM_ENDPOINTS),
        .CLK_PERIOD_NS     (CLK_PERIOD_NS),
        .USE_EXT_TIMESTAMP (USE_EXT_TIMESTAMP)
    ) u_csr_edt (
        .clk                      (clk),
        .rst_n_local              (rst_n_local),
        .ext_timestamp_ns         (ext_timestamp_ns),
        .s_axi_awaddr             (s_axi_awaddr),
        .s_axi_awvalid            (s_axi_awvalid),
        .s_axi_awready            (s_axi_awready),
        .s_axi_wdata              (s_axi_wdata),
        .s_axi_wstrb              (s_axi_wstrb),
        .s_axi_wvalid             (s_axi_wvalid),
        .s_axi_wready             (s_axi_wready),
        .s_axi_bresp              (s_axi_bresp),
        .s_axi_bvalid             (s_axi_bvalid),
        .s_axi_bready             (s_axi_bready),
        .s_axi_araddr             (s_axi_araddr),
        .s_axi_arvalid            (s_axi_arvalid),
        .s_axi_arready            (s_axi_arready),
        .s_axi_rdata              (s_axi_rdata),
        .s_axi_rresp              (s_axi_rresp),
        .s_axi_rvalid             (s_axi_rvalid),
        .s_axi_rready             (s_axi_rready),
        .ctrl_reg                 (ctrl_reg),
        .expected_crc             (expected_crc),
        .cfg_pacing_timer         (cfg_pacing_timer),
        .cfg_payload_words        (cfg_payload_words),
        .cfg_frag_size            (cfg_frag_size),
        .local_guid_prefix        (local_guid_prefix),
        .local_src_ip             (local_src_ip),
        .local_mac                (local_mac),
        .local_vendor_id          (local_vendor_id),
        .ts_offset                (ts_offset),
        .fault_inject_reg         (fault_inject_reg),
        .active_time_ns           (active_time_ns),
        .lifecycle_ok_q           (lifecycle_ok_q),
        .status_write_pulse       (status_write_pulse),
        .status_write_data        (status_write_data),
        .trigger_detected         (trigger_detected),
        .edt_commit_applied       (edt_commit_applied),
        .identity_clear_seq_pulse (identity_clear_seq_pulse),
        .wr0_dst_ip               (wr0_dst_ip),
        .wr0_src_port             (wr0_src_port),
        .wr0_dst_port             (wr0_dst_port),
        .wr0_dst_mac              (wr0_dst_mac),
        .wr0_writer_id            (wr0_writer_id),
        .wr0_reader_id            (wr0_reader_id),
        .wr_valid_vec             (wr_valid_vec),
        .wr_seq_seed_flat         (wr_seq_seed_flat),
        .rd_edt_flat              (rd_edt_flat),
        .state                    (state),
        .rx_egress_state          (rx_egress_state),
        .crc_accum                (crc_accum),
        .last_cycle_ticks         (last_cycle_ticks),
        .completed_frags          (completed_frags),
        .ip_done_irq              (ip_done_irq),
        .irq_is_rx                (irq_is_rx),
        .err_crc                  (err_crc),
        .err_no_match             (err_no_match),
        .err_csum                 (err_csum),
        .stat_tx_samples          (stat_tx_samples),
        .stat_rx_samples          (stat_rx_samples),
        .stat_seq_gaps            (stat_seq_gaps),
        .stat_stale_drops         (stat_stale_drops),
        .stat_csum_errs           (stat_csum_errs),
        .stat_skipped_submsg      (stat_skipped_submsg),
        .last_rx_seq_num          (last_rx_seq_num),
        .last_rx_ts               (last_rx_ts),
        .latest_rtps_words        (latest_rtps_words),
        .last_rx_src_ip           (last_rx_src_ip),
        .last_rx_src_port         (last_rx_src_port),
        .last_rx_dst_port         (last_rx_dst_port),
        .last_rx_cam_idx          (last_rx_cam_idx),
        .last_rx_src_mac          (last_rx_src_mac),
        .rd_match_count_flat      (rd_match_count_flat),
        .remote_client_mac        (remote_client_mac),
        .stat_icmp_echo_replies   (stat_icmp_echo_replies),
        .stat_icmp_errors         (stat_icmp_errors),
        .icmp_err_info            (icmp_err_info),
        .icmp_err_dst_ip          (icmp_err_dst_ip),
        .icmp_err_ports           (icmp_err_ports),
        .icmp_err_ip_id           (icmp_err_ip_id)
    );

    rtps_wire_mux #(
        .DATA_WIDTH  (DATA_WIDTH),
        .REASM_DEPTH (REASM_DEPTH)
    ) u_wire_mux (
        .clk                (clk),
        .rst_n_local        (rst_n_local),
        .emu_enable         (ctrl_reg[4]),
        .fault_inject_reg   (fault_inject_reg),
        .tx_ip_frag_mode    (ctrl_reg[10]),
        .cfg_frag_size      (cfg_frag_size[15:0]),
        .peer_guid_prefix   ({rd_edt_flat[1*32 +: 32], rd_edt_flat[2*32 +: 32], rd_edt_flat[3*32 +: 32]}),
        .peer_writer_id     (rd_edt_flat[4*32 +: 32]),
        .peer_reader_id     (rd_edt_flat[5*32 +: 32]),
        .eng_tx_tdata       (eng_tx_tdata),
        .eng_tx_tvalid      (eng_tx_tvalid),
        .eng_tx_tready      (eng_tx_tready),
        .eng_tx_tlast       (eng_tx_tlast),
        .eng_rx_tdata       (eng_rx_tdata),
        .eng_rx_tvalid      (eng_rx_tvalid),
        .eng_rx_tready      (eng_rx_tready),
        .eng_rx_tlast       (eng_rx_tlast),
        .m_axis_wire_tdata  (m_axis_wire_tdata),
        .m_axis_wire_tvalid (m_axis_wire_tvalid),
        .m_axis_wire_tready (m_axis_wire_tready),
        .m_axis_wire_tlast  (m_axis_wire_tlast),
        .s_axis_wire_tdata  (s_axis_wire_tdata),
        .s_axis_wire_tvalid (s_axis_wire_tvalid),
        .s_axis_wire_tready (s_axis_wire_tready),
        .s_axis_wire_tlast  (s_axis_wire_tlast)
    );

    rtps_tx_engine #(
        .DATA_WIDTH    (DATA_WIDTH),
        .BRAM_DEPTH    (BRAM_DEPTH),
        .ADDR_WIDTH    (ADDR_WIDTH),
        .NUM_ENDPOINTS (NUM_ENDPOINTS)
    ) u_tx_engine (
        .clk                      (clk),
        .rst_n_local              (rst_n_local),
        .s_axis_tdata             (s_axis_tdata),
        .s_axis_tkeep             (s_axis_tkeep),
        .s_axis_tvalid            (s_axis_tvalid),
        .s_axis_tready            (s_axis_tready),
        .s_axis_tlast             (s_axis_tlast),
        .eng_tx_tdata             (eng_tx_tdata),
        .eng_tx_tvalid            (eng_tx_tvalid),
        .eng_tx_tready            (eng_tx_tready),
        .eng_tx_tlast             (eng_tx_tlast),
        .ctrl_reg                 (ctrl_reg),
        .expected_crc             (expected_crc),
        .cfg_pacing_timer         (cfg_pacing_timer),
        .cfg_payload_words        (cfg_payload_words),
        .cfg_frag_size            (cfg_frag_size),
        .local_guid_prefix        (local_guid_prefix),
        .local_src_ip             (local_src_ip),
        .local_mac                (local_mac),
        .local_vendor_id          (local_vendor_id),
        .ts_offset                (ts_offset),
        .active_time_ns           (active_time_ns),
        .lifecycle_ok_q           (lifecycle_ok_q),
        .status_write_pulse       (status_write_pulse),
        .status_write_data        (status_write_data),
        .trigger_detected         (trigger_detected),
        .edt_commit_applied       (edt_commit_applied),
        .identity_clear_seq_pulse (identity_clear_seq_pulse),
        .wr0_dst_ip               (wr0_dst_ip),
        .wr0_src_port             (wr0_src_port),
        .wr0_dst_port             (wr0_dst_port),
        .wr0_dst_mac              (wr0_dst_mac),
        .wr0_writer_id            (wr0_writer_id),
        .wr0_reader_id            (wr0_reader_id),
        .wr_valid_vec             (wr_valid_vec),
        .wr_seq_seed_flat         (wr_seq_seed_flat),
        .rx_csum_err_pulse        (rx_csum_err_pulse),
        .ip_done_irq              (tx_done_irq),
        .state                    (state),
        .crc_accum                (crc_accum),
        .last_cycle_ticks         (last_cycle_ticks),
        .completed_frags          (completed_frags),
        .stat_tx_samples          (stat_tx_samples),
        .err_crc                  (err_crc),
        .err_no_match             (err_no_match),
        .err_csum                 (err_csum)
    );

    rtps_rx_reasm #(
        .DATA_WIDTH    (DATA_WIDTH),
        .NUM_ENDPOINTS (NUM_ENDPOINTS),
        .REASM_DEPTH   (REASM_DEPTH)
    ) u_rx_reasm (
        .clk                      (clk),
        .rst_n_local              (rst_n_local),
        .eng_rx_tdata             (eng_rx_tdata),
        .eng_rx_tvalid            (eng_rx_tvalid),
        .eng_rx_tready            (eng_rx_tready),
        .eng_rx_tlast             (eng_rx_tlast),
        .local_src_ip             (local_src_ip),
        .local_guid_prefix        (local_guid_prefix),
        .rd_edt_flat              (rd_edt_flat),
        .edt_commit_applied       (edt_commit_applied),
        .identity_clear_seq_pulse (identity_clear_seq_pulse),
        .reasm_wr_en              (reasm_wr_en),
        .reasm_wr_be              (reasm_wr_be),
        .reasm_wr_addr            (reasm_wr_addr),
        .reasm_wr_data            (reasm_wr_data),
        .walker_rd_addr           (walker_rd_addr),
        .reasm_walker_dout        (reasm_walker_dout),
        .rx_commit_pulse          (rx_commit_pulse),
        .rx_csum_err_pulse        (rx_csum_err_pulse),
        .rx_state                 (rx_state),
        .latest_payload_word_off  (latest_payload_word_off),
        .latest_rtps_words        (latest_rtps_words),
        .stat_rx_samples          (stat_rx_samples),
        .stat_seq_gaps            (stat_seq_gaps),
        .stat_stale_drops         (stat_stale_drops),
        .stat_csum_errs           (stat_csum_errs),
        .stat_skipped_submsg      (stat_skipped_submsg),
        .last_rx_seq_num          (last_rx_seq_num),
        .last_rx_ts               (last_rx_ts),
        .last_rx_src_ip           (last_rx_src_ip),
        .last_rx_src_port         (last_rx_src_port),
        .last_rx_dst_port         (last_rx_dst_port),
        .last_rx_cam_idx          (last_rx_cam_idx),
        .last_rx_src_mac          (last_rx_src_mac),
        .rd_match_count_flat      (rd_match_count_flat)
    );

    wire                  plugin_rx_rtps_mode = ctrl_reg[7];
    wire [11:0]           egress_start_word   = plugin_rx_rtps_mode ? 12'd2 : latest_payload_word_off;
    wire [ADDR_WIDTH:0]   egress_total_words  = plugin_rx_rtps_mode ? latest_rtps_words : cfg_payload_words[ADDR_WIDTH:0];

    rtps_s2mm_egress #(
        .DATA_WIDTH (DATA_WIDTH),
        .ADDR_WIDTH (ADDR_WIDTH)
    ) u_s2mm_egress (
        .clk                (clk),
        .rst_n_local        (rst_n_local),
        .latest_valid       (latest_valid),
        .status_write_pulse (status_write_pulse),
        .status_write_data  (status_write_data),
        .trigger_detected   (trigger_detected),
        .slot_consume_pulse (slot_consume_pulse),
        .rx_done_irq        (rx_done_irq),
        .rx_egress_state    (rx_egress_state),
        .egress_active      (egress_active),
        .egress_start_word  (egress_start_word),
        .egress_total_words (egress_total_words),
        .s2mm_rd_addr       (s2mm_rd_addr),
        .s2mm_dout          (s2mm_dout),
        .m_axis_tdata       (m_axis_tdata),
        .m_axis_tkeep       (m_axis_tkeep),
        .m_axis_tvalid      (m_axis_tvalid),
        .m_axis_tready      (m_axis_tready),
        .m_axis_tlast       (m_axis_tlast)
    );

endmodule
