`timescale 1ns / 1ps

// =============================================================================
// Module: rtps_csr_edt
//
// Description:
//   AXI4-Lite Slave (CSRs + Shadow/Active Endpoint Descriptor Table),
//   free-running RTPS Time_t clock, 100 ms lifecycle watchdog, and
//   lifecycle_ok_q validation logic.
// =============================================================================

module rtps_csr_edt #(
    parameter integer DATA_WIDTH     = 32,
    parameter integer BRAM_DEPTH     = 2048,
    parameter integer CFG_ADDR_WIDTH = 10,
    parameter integer NUM_ENDPOINTS  = 4,
    parameter integer CLK_PERIOD_NS  = 10,
    parameter integer USE_EXT_TIMESTAMP = 0
)(
    input  wire                          clk,
    input  wire                          rst_n_local,
    input  wire [63:0]                   ext_timestamp_ns,

    // -------------------------------------------------------------------------
    // AXI4-Lite Memory-Mapped Slave
    // -------------------------------------------------------------------------
    input  wire [CFG_ADDR_WIDTH+1:0]     s_axi_awaddr,
    input  wire                          s_axi_awvalid,
    output reg                           s_axi_awready,
    input  wire [DATA_WIDTH-1:0]         s_axi_wdata,
    input  wire [(DATA_WIDTH/8)-1:0]     s_axi_wstrb,
    input  wire                          s_axi_wvalid,
    output reg                           s_axi_wready,
    output wire [1:0]                    s_axi_bresp,
    output reg                           s_axi_bvalid,
    input  wire                          s_axi_bready,

    input  wire [CFG_ADDR_WIDTH+1:0]     s_axi_araddr,
    input  wire                          s_axi_arvalid,
    output reg                           s_axi_arready,
    output reg  [DATA_WIDTH-1:0]         s_axi_rdata,
    output wire [1:0]                    s_axi_rresp,
    output reg                           s_axi_rvalid,
    input  wire                          s_axi_rready,

    // -------------------------------------------------------------------------
    // Configuration & Command Outputs to Datapath
    // -------------------------------------------------------------------------
    output reg  [31:0]                   ctrl_reg,
    output reg  [31:0]                   expected_crc,
    output reg  [31:0]                   cfg_pacing_timer,
    output reg  [31:0]                   cfg_payload_words,
    output reg  [31:0]                   cfg_frag_size,
    output reg  [31:0]                   rx_timeout_limit,
    output reg  [95:0]                   local_guid_prefix,
    output reg  [31:0]                   local_src_ip,
    output reg  [47:0]                   local_mac,
    output reg  [15:0]                   local_vendor_id,
    output reg  [63:0]                   ts_offset,
    output reg  [31:0]                   fault_inject_reg,
    output wire [63:0]                   active_time_ns,
    output reg                           lifecycle_ok_q,

    // Command / Event Pulses
    output reg                           status_write_pulse,
    output reg  [31:0]                   status_write_data,
    output reg                           trigger_detected,
    output reg                           edt_commit_applied,
    output reg                           identity_clear_seq_pulse,

    // Active Writer 0 & Seed Outputs
    output wire [31:0]                   wr0_dst_ip,
    output wire [15:0]                   wr0_src_port,
    output wire [15:0]                   wr0_dst_port,
    output wire [47:0]                   wr0_dst_mac,
    output wire [31:0]                   wr0_writer_id,
    output wire [31:0]                   wr0_reader_id,
    output wire [NUM_ENDPOINTS-1:0]      wr_valid_vec,
    output wire [NUM_ENDPOINTS*32-1:0]   wr_seq_seed_flat,
    output wire [NUM_ENDPOINTS*8*32-1:0] wr_edt_flat,

    // Active Reader CAM EDT Outputs (4 endpoints * 7 words * 32 bits)
    output wire [NUM_ENDPOINTS*7*32-1:0] rd_edt_flat,

    // Adaptive Small-TX Writer Select & Large DDR Ring Configuration Outputs
    output reg  [1:0]                    tx_active_writer,
    output reg  [31:0]                   large_rx_timeout_limit,
    output reg  [11:0]                   bmap_rd_ctrl,
    input  wire [31:0]                   bmap_rd_data,

    output wire [NUM_ENDPOINTS*32-1:0]   tx_ring_base_flat,
    output wire [NUM_ENDPOINTS*32-1:0]   tx_ring_slot_size_flat,
    output wire [NUM_ENDPOINTS*32-1:0]   tx_ring_cfg_flat,
    output wire [NUM_ENDPOINTS*32-1:0]   tx_ring_submit_len_flat,
    output wire [NUM_ENDPOINTS*5-1:0]    tx_ring_prod_idx_flat,
    output reg  [NUM_ENDPOINTS-1:0]      tx_ring_doorbell_pulse,
    output reg  [NUM_ENDPOINTS-1:0]      tx_ring_reset_pulse,
    input  wire [NUM_ENDPOINTS*5-1:0]    tx_ring_cons_idx_flat,
    input  wire [NUM_ENDPOINTS*32-1:0]   tx_ring_stat_done_flat,

    output wire [NUM_ENDPOINTS*32-1:0]   rx_ring_base_flat,
    output wire [NUM_ENDPOINTS*32-1:0]   rx_ring_slot_size_flat,
    output wire [NUM_ENDPOINTS*32-1:0]   rx_ring_cfg_flat,
    output wire [NUM_ENDPOINTS*5-1:0]    rx_ring_cons_idx_flat,
    output reg  [NUM_ENDPOINTS-1:0]      rx_ring_reset_pulse,
    input  wire [NUM_ENDPOINTS*5-1:0]    rx_ring_prod_idx_flat,
    input  wire [NUM_ENDPOINTS*32-1:0]   rx_ring_comp_bytes_flat,
    input  wire [NUM_ENDPOINTS*64-1:0]   rx_ring_comp_seq_flat,
    input  wire [NUM_ENDPOINTS*64-1:0]   rx_ring_comp_ts_flat,
    input  wire [NUM_ENDPOINTS*32-1:0]   rx_ring_comp_src_ip_flat,
    input  wire [NUM_ENDPOINTS*32-1:0]   rx_ring_comp_ports_flat,
    input  wire [NUM_ENDPOINTS*32-1:0]   rx_ring_stat_done_flat,
    input  wire [NUM_ENDPOINTS*32-1:0]   rx_ring_stat_drops_flat,

    input  wire [NUM_ENDPOINTS-1:0]      large_tx_done_irq_vec,
    input  wire [NUM_ENDPOINTS-1:0]      large_rx_done_irq_vec,
    input  wire [NUM_ENDPOINTS-1:0]      large_rx_overflow_vec,
    input  wire [NUM_ENDPOINTS-1:0]      large_rx_timeout_vec,
    output reg  [NUM_ENDPOINTS-1:0]      large_tx_irq_ack_pulse,
    output reg  [NUM_ENDPOINTS-1:0]      large_rx_irq_ack_pulse,

    // -------------------------------------------------------------------------
    // Status & Telemetry Inputs from Datapath & External Interfaces
    // -------------------------------------------------------------------------
    input  wire [3:0]                    state,
    input  wire [1:0]                    rx_egress_state,
    input  wire [31:0]                   crc_accum,
    input  wire [31:0]                   last_cycle_ticks,
    input  wire [15:0]                   completed_frags,
    input  wire                          ip_done_irq,
    input  wire                          irq_is_rx,
    input  wire                          err_crc,
    input  wire                          err_no_match,
    input  wire                          err_csum,

    input  wire [31:0]                   stat_tx_samples,
    input  wire [31:0]                   stat_rx_samples,
    input  wire [31:0]                   stat_seq_gaps,
    input  wire [31:0]                   stat_stale_drops,
    input  wire [31:0]                   stat_csum_errs,
    input  wire [31:0]                   stat_skipped_submsg,
    input  wire [63:0]                   last_rx_seq_num,
    input  wire [63:0]                   last_rx_ts,
    input  wire [11:0]                   latest_rtps_words,
    input  wire [31:0]                   last_rx_src_ip,
    input  wire [15:0]                   last_rx_src_port,
    input  wire [15:0]                   last_rx_dst_port,
    input  wire [1:0]                    last_rx_cam_idx,
    input  wire [47:0]                   last_rx_src_mac,
    input  wire [NUM_ENDPOINTS*32-1:0]   rd_match_count_flat,

    // ARP & ICMP External Status Inputs
    input  wire [47:0]                   remote_client_mac,
    input  wire [31:0]                   stat_icmp_echo_replies,
    input  wire [31:0]                   stat_icmp_errors,
    input  wire [31:0]                   icmp_err_info,
    input  wire [31:0]                   icmp_err_dst_ip,
    input  wire [31:0]                   icmp_err_ports,
    input  wire [15:0]                   icmp_err_ip_id
);

    `include "rtps_fastpath_defs.vh"

    // -------------------------------------------------------------------------
    // Shadow & Active EDT Banks
    // -------------------------------------------------------------------------
    reg [31:0] shadow_wr_edt [0:NUM_ENDPOINTS-1][0:7];
    reg [31:0] active_wr_edt [0:NUM_ENDPOINTS-1][0:7];
    reg [31:0] shadow_rd_edt [0:NUM_ENDPOINTS-1][0:7];
    reg [31:0] active_rd_edt [0:NUM_ENDPOINTS-1][0:7];

    reg        local_identity_valid;
    reg        writer_context_valid;
    reg [31:0] active_generation;
    reg [31:0] shadow_generation;
    reg [31:0] active_wr_generation [0:NUM_ENDPOINTS-1];
    reg [31:0] shadow_wr_generation [0:NUM_ENDPOINTS-1];
    reg [31:0] type_profile_reg;

    reg [31:0] wdog_counter;
    reg [31:0] wdog_limit;
    reg        wdog_expired;
    reg        wdog_pet_pulse;
    reg        edt_commit_pending;

    // Per-endpoint TX & RX DDR Ring configuration registers
    reg [31:0] tx_ring_base_r       [0:NUM_ENDPOINTS-1];
    reg [31:0] tx_ring_slot_size_r  [0:NUM_ENDPOINTS-1];
    reg [31:0] tx_ring_cfg_r        [0:NUM_ENDPOINTS-1];
    reg [31:0] tx_ring_submit_len_r [0:NUM_ENDPOINTS-1];
    reg [4:0]  tx_ring_prod_idx_r   [0:NUM_ENDPOINTS-1];

    reg [31:0] rx_ring_base_r       [0:NUM_ENDPOINTS-1];
    reg [31:0] rx_ring_slot_size_r  [0:NUM_ENDPOINTS-1];
    reg [31:0] rx_ring_cfg_r        [0:NUM_ENDPOINTS-1];
    reg [4:0]  rx_ring_cons_idx_r   [0:NUM_ENDPOINTS-1];

    reg [31:0] free_running_sec;
    reg [47:0] free_running_frac_acc; // Q32.16: [47:16] = RTPS 2^-32 s fraction
    reg [15:0] cfg_clk_period_ns;
    reg [47:0] clk_frac_step;
    reg        sec_inc_q;
    assign active_time_ns = (USE_EXT_TIMESTAMP) ? ext_timestamp_ns :
                            {free_running_sec, free_running_frac_acc[47:16]};

    wire emu_enable    = ctrl_reg[4];
    wire mode_sub_only = ctrl_reg[9];

    // -------------------------------------------------------------------------
    // Free-Running RTPS Time_t & 100 ms Watchdog
    // -------------------------------------------------------------------------
    wire [48:0] next_frac_acc = {1'b0, free_running_frac_acc} + {1'b0, clk_frac_step};
    always @(posedge clk) begin
        if (!rst_n_local) begin
            free_running_sec      <= 32'd1;
            free_running_frac_acc <= 48'd0;
            clk_frac_step         <= CLK_PERIOD_NS * 48'd281475;
            sec_inc_q             <= 1'b0;
            wdog_counter          <= 32'd0;
            wdog_expired          <= 1'b0;
        end else begin
            clk_frac_step         <= {32'd0, cfg_clk_period_ns} * 48'd281475;
            free_running_frac_acc <= next_frac_acc[47:0];
            sec_inc_q             <= next_frac_acc[48];
            free_running_sec      <= free_running_sec + {31'd0, sec_inc_q};
            if (wdog_pet_pulse) begin
                wdog_counter <= 32'd0;
                wdog_expired <= 1'b0;
            end else if (ctrl_reg[6] && ctrl_reg[0]) begin
                if (wdog_counter >= wdog_limit) begin
                    wdog_expired <= 1'b1;
                end else begin
                    wdog_counter <= wdog_counter + 32'd1;
                end
            end else begin
                wdog_counter <= 32'd0;
                wdog_expired <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // AXI4-Lite Slave Write Channel (CSRs & Shadow EDT)
    // -------------------------------------------------------------------------
    assign s_axi_bresp = 2'b00;
    assign s_axi_rresp = 2'b00;

    reg                      aw_hold_valid;
    reg [CFG_ADDR_WIDTH-1:0] aw_hold_addr;
    reg                      w_hold_valid;
    reg [DATA_WIDTH-1:0]     w_hold_data;

    wire aw_active   = aw_hold_valid || (s_axi_awvalid && s_axi_awready);
    wire w_active    = w_hold_valid  || (s_axi_wvalid  && s_axi_wready);
    wire reg_wr_fire = aw_active && w_active && !s_axi_bvalid;

    wire [CFG_ADDR_WIDTH-1:0] cur_wr_addr = aw_hold_valid ? aw_hold_addr : s_axi_awaddr[CFG_ADDR_WIDTH+1:2];
    wire [DATA_WIDTH-1:0]     cur_wr_data = w_hold_valid  ? w_hold_data  : s_axi_wdata;

    integer ep_i, w_i;
    always @(posedge clk) begin
        if (!rst_n_local) begin
            s_axi_awready            <= 1'b1;
            s_axi_wready             <= 1'b1;
            s_axi_bvalid             <= 1'b0;
            aw_hold_valid            <= 1'b0;
            aw_hold_addr             <= {CFG_ADDR_WIDTH{1'b0}};
            w_hold_valid             <= 1'b0;
            w_hold_data              <= 32'd0;
            status_write_pulse       <= 1'b0;
            status_write_data        <= 32'd0;
            trigger_detected         <= 1'b0;
            edt_commit_pending       <= 1'b0;
            edt_commit_applied       <= 1'b0;
            identity_clear_seq_pulse <= 1'b0;
            wdog_pet_pulse           <= 1'b0;

            local_identity_valid     <= 1'b1;
            writer_context_valid     <= 1'b1;
            active_generation        <= 32'd0;
            shadow_generation        <= 32'd0;
            type_profile_reg         <= 32'h0007_0101;

            ctrl_reg                 <= 32'h0000_0193;
            expected_crc             <= 32'd0;
            cfg_pacing_timer         <= 32'd0;
            cfg_payload_words        <= 32'd1024;
            cfg_frag_size            <= 32'd1400;
            wdog_limit               <= 32'd10_000_000;
            rx_timeout_limit         <= 32'd90_000;

            local_guid_prefix        <= 96'h0110_CD01_2345_6789_ABCD_EF01;
            local_src_ip             <= 32'hC0A8_01C9;
            local_mac                <= 48'h0200_5E10_0001;
            local_vendor_id          <= 16'h0101;
            ts_offset                <= 64'h6530_0000_0000_0000;
            fault_inject_reg         <= 32'd0;
            cfg_clk_period_ns        <= CLK_PERIOD_NS[15:0];
            tx_active_writer         <= 2'd0;
            large_rx_timeout_limit   <= 32'd50_000_000;
            bmap_rd_ctrl             <= 12'd0;
            tx_ring_doorbell_pulse   <= {NUM_ENDPOINTS{1'b0}};
            tx_ring_reset_pulse      <= {NUM_ENDPOINTS{1'b0}};
            rx_ring_reset_pulse      <= {NUM_ENDPOINTS{1'b0}};
            large_tx_irq_ack_pulse   <= {NUM_ENDPOINTS{1'b0}};
            large_rx_irq_ack_pulse   <= {NUM_ENDPOINTS{1'b0}};

            for (ep_i = 0; ep_i < NUM_ENDPOINTS; ep_i = ep_i + 1) begin
                active_wr_generation[ep_i] <= 32'd0;
                shadow_wr_generation[ep_i] <= 32'd0;
                tx_ring_base_r[ep_i]       <= 32'd0;
                tx_ring_slot_size_r[ep_i]  <= 32'd0;
                tx_ring_cfg_r[ep_i]        <= 32'd0;
                tx_ring_submit_len_r[ep_i] <= 32'd0;
                tx_ring_prod_idx_r[ep_i]   <= 5'd0;
                rx_ring_base_r[ep_i]       <= 32'd0;
                rx_ring_slot_size_r[ep_i]  <= 32'd0;
                rx_ring_cfg_r[ep_i]        <= 32'd0;
                rx_ring_cons_idx_r[ep_i]   <= 5'd0;
                for (w_i = 0; w_i < 8; w_i = w_i + 1) begin
                    shadow_wr_edt[ep_i][w_i] <= 32'd0;
                    active_wr_edt[ep_i][w_i] <= 32'd0;
                    shadow_rd_edt[ep_i][w_i] <= 32'd0;
                    active_rd_edt[ep_i][w_i] <= 32'd0;
                end
            end

            shadow_wr_edt[0][0] <= {16'd0, 12'd1024, 3'd2, 1'b1};
            shadow_wr_edt[0][1] <= 32'hC0A8_0164;
            shadow_wr_edt[0][2] <= {16'd7411, 16'd7410};
            shadow_wr_edt[0][3] <= 32'h5E20_0002;
            shadow_wr_edt[0][4] <= 32'h0000_0200;
            shadow_wr_edt[0][5] <= 32'h0000_0102;
            shadow_wr_edt[0][6] <= 32'h0000_0000;
            shadow_wr_edt[0][7] <= 32'h0000_0001;

            active_wr_edt[0][0] <= {16'd0, 12'd1024, 3'd2, 1'b1};
            active_wr_edt[0][1] <= 32'hC0A8_0164;
            active_wr_edt[0][2] <= {16'd7411, 16'd7410};
            active_wr_edt[0][3] <= 32'h5E20_0002;
            active_wr_edt[0][4] <= 32'h0000_0200;
            active_wr_edt[0][5] <= 32'h0000_0102;
            active_wr_edt[0][6] <= 32'h0000_0000;
            active_wr_edt[0][7] <= 32'h0000_0001;

            shadow_rd_edt[0][0] <= {16'd7411, 12'd0, 3'd2, 1'b1};
            shadow_rd_edt[0][1] <= 32'h0101_AA00;
            shadow_rd_edt[0][2] <= 32'hBB11_CC22;
            shadow_rd_edt[0][3] <= 32'hDD33_EE44;
            shadow_rd_edt[0][4] <= 32'h0000_0202;
            shadow_rd_edt[0][5] <= 32'h0000_0107;
            shadow_rd_edt[0][6] <= 32'hC0A8_0164;

            active_rd_edt[0][0] <= {16'd7411, 12'd0, 3'd2, 1'b1};
            active_rd_edt[0][1] <= 32'h0101_AA00;
            active_rd_edt[0][2] <= 32'hBB11_CC22;
            active_rd_edt[0][3] <= 32'hDD33_EE44;
            active_rd_edt[0][4] <= 32'h0000_0202;
            active_rd_edt[0][5] <= 32'h0000_0107;
            active_rd_edt[0][6] <= 32'hC0A8_0164;
        end else begin
            status_write_pulse       <= 1'b0;
            trigger_detected         <= 1'b0;
            wdog_pet_pulse           <= 1'b0;
            edt_commit_applied       <= 1'b0;
            identity_clear_seq_pulse <= 1'b0;
            tx_ring_doorbell_pulse   <= {NUM_ENDPOINTS{1'b0}};
            tx_ring_reset_pulse      <= {NUM_ENDPOINTS{1'b0}};
            rx_ring_reset_pulse      <= {NUM_ENDPOINTS{1'b0}};
            large_tx_irq_ack_pulse   <= {NUM_ENDPOINTS{1'b0}};
            large_rx_irq_ack_pulse   <= {NUM_ENDPOINTS{1'b0}};

            if (!aw_hold_valid) begin
                if (s_axi_awvalid && s_axi_awready) begin
                    if (!w_active || s_axi_bvalid) begin
                        aw_hold_valid <= 1'b1;
                        aw_hold_addr  <= s_axi_awaddr[CFG_ADDR_WIDTH+1:2];
                        s_axi_awready <= 1'b0;
                    end
                end else begin
                    s_axi_awready <= !s_axi_bvalid;
                end
            end

            if (!w_hold_valid) begin
                if (s_axi_wvalid && s_axi_wready) begin
                    if (!aw_active || s_axi_bvalid) begin
                        w_hold_valid <= 1'b1;
                        w_hold_data  <= s_axi_wdata;
                        s_axi_wready <= 1'b0;
                    end
                end else begin
                    s_axi_wready <= !s_axi_bvalid;
                end
            end

            if (edt_commit_pending && (state == S_IDLE)) begin
                edt_commit_pending <= 1'b0;
                edt_commit_applied <= 1'b1;
                active_generation  <= shadow_generation;
                for (ep_i = 0; ep_i < NUM_ENDPOINTS; ep_i = ep_i + 1) begin
                    for (w_i = 0; w_i < 8; w_i = w_i + 1) begin
                        active_wr_edt[ep_i][w_i] <= shadow_wr_edt[ep_i][w_i];
                        active_rd_edt[ep_i][w_i] <= shadow_rd_edt[ep_i][w_i];
                    end
                    active_wr_generation[ep_i] <= shadow_wr_generation[ep_i];
                end
            end

            if (reg_wr_fire) begin
                s_axi_bvalid  <= 1'b1;
                aw_hold_valid <= 1'b0;
                w_hold_valid  <= 1'b0;
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;

                if (cur_wr_addr >= 10'h020 && cur_wr_addr < 10'h040) begin
                    shadow_wr_edt[(cur_wr_addr - 10'h020) >> 3][(cur_wr_addr - 10'h020) & 3'd7] <= cur_wr_data;
                end else if (cur_wr_addr >= 10'h040 && cur_wr_addr < 10'h060) begin
                    shadow_rd_edt[(cur_wr_addr - 10'h040) >> 3][(cur_wr_addr - 10'h040) & 3'd7] <= cur_wr_data;
                end else if (cur_wr_addr >= 10'h0C0 && cur_wr_addr < 10'h0E0) begin : tx_ring_wr_blk
                    reg [1:0] ep;
                    ep = cur_wr_addr[4:3];
                    case (cur_wr_addr[2:0])
                        3'd0: tx_ring_base_r[ep]       <= cur_wr_data;
                        3'd1: tx_ring_slot_size_r[ep]  <= cur_wr_data;
                        3'd2: begin
                            tx_ring_cfg_r[ep] <= {22'd0, 1'b0, cur_wr_data[8:0]};
                            if (cur_wr_data[9]) begin
                                tx_ring_reset_pulse[ep] <= 1'b1;
                                tx_ring_prod_idx_r[ep]  <= 5'd0;
                            end
                        end
                        3'd3: tx_ring_submit_len_r[ep] <= cur_wr_data;
                        3'd4: begin
                            tx_ring_prod_idx_r[ep]     <= cur_wr_data[4:0];
                            tx_ring_doorbell_pulse[ep] <= 1'b1;
                        end
                        default: ;
                    endcase
                end else if (cur_wr_addr >= 10'h100 && cur_wr_addr < 10'h140) begin : rx_ring_wr_blk
                    reg [1:0] ep;
                    ep = cur_wr_addr[5:4];
                    case (cur_wr_addr[3:0])
                        4'd0: rx_ring_base_r[ep]       <= cur_wr_data;
                        4'd1: rx_ring_slot_size_r[ep]  <= cur_wr_data;
                        4'd2: begin
                            rx_ring_cfg_r[ep] <= {22'd0, 1'b0, cur_wr_data[8:0]};
                            if (cur_wr_data[9]) begin
                                rx_ring_reset_pulse[ep] <= 1'b1;
                                rx_ring_cons_idx_r[ep]  <= 5'd0;
                            end
                        end
                        4'd4: rx_ring_cons_idx_r[ep]   <= cur_wr_data[4:0];
                        default: ;
                    endcase
                end else begin
                    case (cur_wr_addr)
                        TRIGGER_ADDR:      trigger_detected   <= 1'b1;
                        STATUS_ADDR: begin
                            status_write_pulse <= 1'b1;
                            status_write_data  <= cur_wr_data;
                            if (cur_wr_data[4])
                                large_tx_irq_ack_pulse <= {NUM_ENDPOINTS{1'b1}};
                            if (cur_wr_data[5])
                                large_rx_irq_ack_pulse <= {NUM_ENDPOINTS{1'b1}};
                        end
                        CTRL_ADDR: begin
                            ctrl_reg <= cur_wr_data;
                            if (cur_wr_data[5]) begin
                                edt_commit_pending <= 1'b1;
                            end
                        end
                        EXPECTED_CRC_ADDR: expected_crc       <= cur_wr_data;
                        PACING_TIMER_ADDR: cfg_pacing_timer   <= {16'd0, cur_wr_data[15:0]};
                        CFG_PAYLOAD_ADDR:  cfg_payload_words  <= (cur_wr_data == 0 || cur_wr_data > BRAM_DEPTH) ?
                                                                 BRAM_DEPTH : cur_wr_data;
                        WDOG_PET_ADDR:     wdog_pet_pulse     <= 1'b1;
                        WDOG_LIMIT_ADDR:   wdog_limit         <= cur_wr_data;
                        RX_TIMEOUT_ADDR:   rx_timeout_limit   <= cur_wr_data;
                        LOCAL_GUID_0_ADDR: local_guid_prefix[95:64] <= cur_wr_data;
                        LOCAL_GUID_1_ADDR: local_guid_prefix[63:32] <= cur_wr_data;
                        LOCAL_GUID_2_ADDR: local_guid_prefix[31:0]  <= cur_wr_data;
                        LOCAL_SRC_IP_ADDR: local_src_ip       <= cur_wr_data;
                        LOCAL_MAC_LO_ADDR: local_mac[31:0]    <= cur_wr_data;
                        LOCAL_MAC_HI_ADDR: begin
                            local_mac[47:32] <= cur_wr_data[15:0];
                            local_vendor_id  <= (cur_wr_data[31:16] == 16'd0) ? 16'h0101 : cur_wr_data[31:16];
                        end
                        TS_OFFSET_LO_ADDR: ts_offset[31:0]    <= cur_wr_data;
                        TS_OFFSET_HI_ADDR: ts_offset[63:32]   <= cur_wr_data;
                        FAULT_INJECT_ADDR: fault_inject_reg   <= cur_wr_data;
                        CLK_PERIOD_ADDR:   cfg_clk_period_ns  <= cur_wr_data[15:0];
                        IDENTITY_CTRL_ADDR: begin
                            local_identity_valid <= cur_wr_data[0];
                            writer_context_valid <= cur_wr_data[1];
                            if (cur_wr_data[2]) begin
                                identity_clear_seq_pulse <= 1'b1;
                            end
                        end
                        GENERATION_ADDR: begin
                            shadow_generation <= cur_wr_data;
                            for (ep_i = 0; ep_i < NUM_ENDPOINTS; ep_i = ep_i + 1) begin
                                shadow_wr_generation[ep_i] <= cur_wr_data;
                            end
                        end
                        TYPE_PROFILE_ADDR:     type_profile_reg       <= cur_wr_data;
                        CFG_FRAG_SIZE_ADDR:    cfg_frag_size          <= {16'd0, cur_wr_data[15:0]};
                        TX_ACTIVE_WRITER_ADDR: tx_active_writer       <= cur_wr_data[1:0];
                        LARGE_IRQ_STATUS_ADDR: begin
                            large_tx_irq_ack_pulse <= cur_wr_data[3:0];
                            large_rx_irq_ack_pulse <= cur_wr_data[7:4];
                        end
                        LARGE_RX_TIMEOUT_ADDR: large_rx_timeout_limit <= cur_wr_data;
                        BMAP_RD_CTRL_ADDR:     bmap_rd_ctrl           <= cur_wr_data[11:0];
                        default: ;
                    endcase
                end
            end

            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid  <= 1'b0;
                s_axi_awready <= !aw_hold_valid;
                s_axi_wready  <= !w_hold_valid;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Active Endpoint Exports & Lifecycle Evaluation
    // -------------------------------------------------------------------------
    assign wr0_dst_ip    = active_wr_edt[0][1];
    assign wr0_src_port  = active_wr_edt[0][2][31:16];
    assign wr0_dst_port  = active_wr_edt[0][2][15:0];
    assign wr0_writer_id = active_wr_edt[0][5];
    assign wr0_reader_id = active_wr_edt[0][6];

    wire [47:0] wr0_configured_dst_mac = {active_wr_edt[0][4][15:0], active_wr_edt[0][3]};
    wire [47:0] wr0_learned_dst_mac    = (remote_client_mac != 48'd0) ? remote_client_mac : last_rx_src_mac;
    wire [47:0] wr0_dst_mac_comb       = (wr0_configured_dst_mac != 48'd0) ? wr0_configured_dst_mac :
                                         (wr0_learned_dst_mac    != 48'd0) ? wr0_learned_dst_mac :
                                         (emu_enable ? 48'h0200_5E20_0002 : 48'hFFFF_FFFF_FFFF);
    reg  [47:0] wr0_dst_mac_q;
    always @(posedge clk) begin
        if (!rst_n_local)
            wr0_dst_mac_q <= 48'h0200_5E20_0002;
        else
            wr0_dst_mac_q <= wr0_dst_mac_comb;
    end
    assign wr0_dst_mac = wr0_dst_mac_q;

    genvar g_ep, g_w;
    generate
        for (g_ep = 0; g_ep < NUM_ENDPOINTS; g_ep = g_ep + 1) begin : GEN_EDT_OUT
            assign wr_valid_vec[g_ep]                       = active_wr_edt[g_ep][0][0];
            assign wr_seq_seed_flat[g_ep*32 +: 32]          = active_wr_edt[g_ep][7];
            assign tx_ring_base_flat[g_ep*32 +: 32]         = tx_ring_base_r[g_ep];
            assign tx_ring_slot_size_flat[g_ep*32 +: 32]    = tx_ring_slot_size_r[g_ep];
            assign tx_ring_cfg_flat[g_ep*32 +: 32]          = tx_ring_cfg_r[g_ep];
            assign tx_ring_submit_len_flat[g_ep*32 +: 32]   = tx_ring_submit_len_r[g_ep];
            assign tx_ring_prod_idx_flat[g_ep*5 +: 5]       = tx_ring_prod_idx_r[g_ep];
            assign rx_ring_base_flat[g_ep*32 +: 32]         = rx_ring_base_r[g_ep];
            assign rx_ring_slot_size_flat[g_ep*32 +: 32]    = rx_ring_slot_size_r[g_ep];
            assign rx_ring_cfg_flat[g_ep*32 +: 32]          = rx_ring_cfg_r[g_ep];
            assign rx_ring_cons_idx_flat[g_ep*5 +: 5]       = rx_ring_cons_idx_r[g_ep];
            for (g_w = 0; g_w < 8; g_w = g_w + 1) begin : GEN_WR_WORDS
                assign wr_edt_flat[(g_ep*8 + g_w)*32 +: 32] = active_wr_edt[g_ep][g_w];
            end
            for (g_w = 0; g_w < 7; g_w = g_w + 1) begin : GEN_RD_WORDS
                assign rd_edt_flat[(g_ep*7 + g_w)*32 +: 32] = active_rd_edt[g_ep][g_w];
            end
        end
    endgenerate

    wire payload_contract_ok =
        type_profile_reg[16] &&
        type_profile_reg[17] &&
        type_profile_reg[18] &&
        !type_profile_reg[19];

    wire wr0_valid            = active_wr_edt[0][0][0];
    wire any_reader_valid     = active_rd_edt[0][0][0] || active_rd_edt[1][0][0] ||
                                active_rd_edt[2][0][0] || active_rd_edt[3][0][0];
    wire [2:0] wr0_lifecycle_state = active_wr_edt[0][0][3:1];
    wire wr0_lifecycle_active      = (wr0_lifecycle_state == 3'd2);
    wire wr0_route_valid           = (wr0_dst_ip != 32'd0) && (wr0_dst_port != 16'd0) && (wr0_dst_mac != 48'd0);
    wire generation_match          = (shadow_generation == 32'd0) || (active_generation == shadow_generation);

    wire lifecycle_ok     = ctrl_reg[0] &&
                            !wdog_expired &&
                            local_identity_valid &&
                            payload_contract_ok &&
                            generation_match &&
                            (mode_sub_only ? any_reader_valid :
                                              (wr0_valid && wr0_lifecycle_active && wr0_route_valid && writer_context_valid));

    always @(posedge clk) lifecycle_ok_q <= lifecycle_ok;

    // -------------------------------------------------------------------------
    // AXI4-Lite Slave Read Channel
    // -------------------------------------------------------------------------
    wire [CFG_ADDR_WIDTH-1:0] rd_word_idx = s_axi_araddr[CFG_ADDR_WIDTH+1:2];

    always @(posedge clk) begin
        if (!rst_n_local) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= 32'd0;
        end else begin
            if (!s_axi_arready && s_axi_arvalid && !s_axi_rvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rvalid  <= 1'b1;

                if (rd_word_idx >= 10'h020 && rd_word_idx < 10'h040) begin
                    s_axi_rdata <= active_wr_edt[(rd_word_idx - 10'h020) >> 3][(rd_word_idx - 10'h020) & 3'd7];
                end else if (rd_word_idx >= 10'h040 && rd_word_idx < 10'h060) begin
                    if (((rd_word_idx - 10'h040) & 3'd7) == 3'd7)
                        s_axi_rdata <= rd_match_count_flat[((rd_word_idx - 10'h040) >> 3)*32 +: 32];
                    else
                        s_axi_rdata <= active_rd_edt[(rd_word_idx - 10'h040) >> 3][(rd_word_idx - 10'h040) & 3'd7];
                end else if (rd_word_idx >= 10'h0C0 && rd_word_idx < 10'h0E0) begin : tx_ring_rd_blk
                    reg [1:0] ep;
                    ep = rd_word_idx[4:3];
                    case (rd_word_idx[2:0])
                        3'd0: s_axi_rdata <= tx_ring_base_r[ep];
                        3'd1: s_axi_rdata <= tx_ring_slot_size_r[ep];
                        3'd2: s_axi_rdata <= tx_ring_cfg_r[ep];
                        3'd3: s_axi_rdata <= tx_ring_submit_len_r[ep];
                        3'd4: s_axi_rdata <= {27'd0, tx_ring_prod_idx_r[ep]};
                        3'd5: s_axi_rdata <= {27'd0, tx_ring_cons_idx_flat[ep*5 +: 5]};
                        3'd6: s_axi_rdata <= tx_ring_stat_done_flat[ep*32 +: 32];
                        default: s_axi_rdata <= 32'd0;
                    endcase
                end else if (rd_word_idx >= 10'h100 && rd_word_idx < 10'h140) begin : rx_ring_rd_blk
                    reg [1:0] ep;
                    ep = rd_word_idx[5:4];
                    case (rd_word_idx[3:0])
                        4'd0:  s_axi_rdata <= rx_ring_base_r[ep];
                        4'd1:  s_axi_rdata <= rx_ring_slot_size_r[ep];
                        4'd2:  s_axi_rdata <= rx_ring_cfg_r[ep];
                        4'd3:  s_axi_rdata <= {27'd0, rx_ring_prod_idx_flat[ep*5 +: 5]};
                        4'd4:  s_axi_rdata <= {27'd0, rx_ring_cons_idx_r[ep]};
                        4'd5:  s_axi_rdata <= rx_ring_comp_bytes_flat[ep*32 +: 32];
                        4'd6:  s_axi_rdata <= rx_ring_comp_seq_flat[ep*64 +: 32];
                        4'd7:  s_axi_rdata <= rx_ring_comp_seq_flat[ep*64 + 32 +: 32];
                        4'd8:  s_axi_rdata <= rx_ring_comp_ts_flat[ep*64 +: 32];
                        4'd9:  s_axi_rdata <= rx_ring_comp_ts_flat[ep*64 + 32 +: 32];
                        4'd10: s_axi_rdata <= rx_ring_comp_src_ip_flat[ep*32 +: 32];
                        4'd11: s_axi_rdata <= rx_ring_comp_ports_flat[ep*32 +: 32];
                        4'd12: s_axi_rdata <= rx_ring_stat_done_flat[ep*32 +: 32];
                        4'd13: s_axi_rdata <= rx_ring_stat_drops_flat[ep*32 +: 32];
                        default: s_axi_rdata <= 32'd0;
                    endcase
                end else begin
                    case (rd_word_idx)
                        STATUS_ADDR: begin
                            s_axi_rdata <= {
                                rx_egress_state,
                                (|large_rx_done_irq_vec),
                                (|large_tx_done_irq_vec),
                                stat_rx_samples[11:0],
                                lifecycle_ok,
                                wdog_expired,
                                completed_frags[2:0],
                                irq_is_rx,
                                1'b1,
                                err_csum,
                                1'b0,
                                err_no_match,
                                err_crc,
                                state,
                                ip_done_irq
                            };
                        end
                        CTRL_ADDR:                   s_axi_rdata <= ctrl_reg;
                        EXPECTED_CRC_ADDR:           s_axi_rdata <= crc_accum;
                        PACING_TIMER_ADDR:           s_axi_rdata <= cfg_pacing_timer;
                        CFG_PAYLOAD_ADDR:            s_axi_rdata <= cfg_payload_words;
                        WDOG_PET_ADDR:               s_axi_rdata <= wdog_limit - wdog_counter;
                        WDOG_LIMIT_ADDR:             s_axi_rdata <= wdog_limit;
                        RX_TIMEOUT_ADDR:             s_axi_rdata <= rx_timeout_limit;
                        LAST_CYCLE_TICKS:            s_axi_rdata <= last_cycle_ticks;
                        LOCAL_GUID_0_ADDR:           s_axi_rdata <= local_guid_prefix[95:64];
                        LOCAL_GUID_1_ADDR:           s_axi_rdata <= local_guid_prefix[63:32];
                        LOCAL_GUID_2_ADDR:           s_axi_rdata <= local_guid_prefix[31:0];
                        LOCAL_SRC_IP_ADDR:           s_axi_rdata <= local_src_ip;
                        LOCAL_MAC_LO_ADDR:           s_axi_rdata <= local_mac[31:0];
                        LOCAL_MAC_HI_ADDR:           s_axi_rdata <= {local_vendor_id, local_mac[47:32]};
                        TS_OFFSET_LO_ADDR:           s_axi_rdata <= ts_offset[31:0];
                        TS_OFFSET_HI_ADDR:           s_axi_rdata <= ts_offset[63:32];
                        FAULT_INJECT_ADDR:           s_axi_rdata <= fault_inject_reg;
                        STAT_TX_SAMPLES:             s_axi_rdata <= stat_tx_samples;
                        STAT_RX_SAMPLES:             s_axi_rdata <= stat_rx_samples;
                        STAT_SEQ_GAPS:               s_axi_rdata <= stat_seq_gaps;
                        STAT_STALE_DROPS:            s_axi_rdata <= stat_stale_drops;
                        STAT_CSUM_ERRS:              s_axi_rdata <= stat_csum_errs;
                        STAT_SKIPPED_SUBMSG:         s_axi_rdata <= stat_skipped_submsg;
                        LAST_RX_SEQ_LO:              s_axi_rdata <= last_rx_seq_num[31:0];
                        LAST_RX_SEQ_HI:              s_axi_rdata <= last_rx_seq_num[63:32];
                        LAST_RX_TS_LO:               s_axi_rdata <= last_rx_ts[31:0];
                        LAST_RX_TS_HI:               s_axi_rdata <= last_rx_ts[63:32];
                        LAST_RX_RTPS_WORDS:          s_axi_rdata <= {20'd0, latest_rtps_words};
                        CLK_PERIOD_ADDR:             s_axi_rdata <= {16'd0, cfg_clk_period_ns};
                        IDENTITY_CTRL_ADDR:          s_axi_rdata <= {29'd0, 1'b0, writer_context_valid, local_identity_valid};
                        GENERATION_ADDR:             s_axi_rdata <= shadow_generation;
                        COMMIT_STATUS_ADDR:          s_axi_rdata <= active_generation;
                        TYPE_PROFILE_ADDR:           s_axi_rdata <= type_profile_reg;
                        LAST_RX_SRC_IP_ADDR:         s_axi_rdata <= last_rx_src_ip;
                        LAST_RX_PORTS_ADDR:          s_axi_rdata <= {last_rx_src_port, last_rx_dst_port};
                        LAST_RX_META_ADDR:           s_axi_rdata <= {24'd0, 6'd0, last_rx_cam_idx};
                        LAST_RX_SRC_MAC_LO:          s_axi_rdata <= last_rx_src_mac[31:0];
                        LAST_RX_SRC_MAC_HI:          s_axi_rdata <= {16'd0, last_rx_src_mac[47:32]};
                        ARP_CLIENT_MAC_LO:           s_axi_rdata <= remote_client_mac[31:0];
                        ARP_CLIENT_MAC_HI:           s_axi_rdata <= {16'd0, remote_client_mac[47:32]};
                        STAT_ICMP_ECHO_REPLIES_ADDR: s_axi_rdata <= stat_icmp_echo_replies;
                        STAT_ICMP_ERRORS_ADDR:       s_axi_rdata <= stat_icmp_errors;
                        LAST_ICMP_ERR_INFO_ADDR:     s_axi_rdata <= icmp_err_info;
                        LAST_ICMP_ERR_DST_IP_ADDR:   s_axi_rdata <= icmp_err_dst_ip;
                        LAST_ICMP_ERR_PORTS_ADDR:    s_axi_rdata <= icmp_err_ports;
                        LAST_ICMP_ERR_IP_ID_ADDR:    s_axi_rdata <= {16'd0, icmp_err_ip_id};
                        CFG_FRAG_SIZE_ADDR:          s_axi_rdata <= cfg_frag_size;
                        TX_ACTIVE_WRITER_ADDR:       s_axi_rdata <= {30'd0, tx_active_writer};
                        LARGE_IRQ_STATUS_ADDR:       s_axi_rdata <= {16'd0, large_rx_timeout_vec, large_rx_overflow_vec, large_rx_done_irq_vec, large_tx_done_irq_vec};
                        LARGE_RX_TIMEOUT_ADDR:       s_axi_rdata <= large_rx_timeout_limit;
                        BMAP_RD_CTRL_ADDR:           s_axi_rdata <= {20'd0, bmap_rd_ctrl};
                        BMAP_RD_DATA_ADDR:           s_axi_rdata <= bmap_rd_data;
                        default:                     s_axi_rdata <= 32'd0;
                    endcase
                end
            end else begin
                s_axi_arready <= 1'b0;
            end

            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

endmodule
