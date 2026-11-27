`timescale 1ns / 1ps

// =============================================================================
// Module: rtps_wire_mux
//
// Description:
//   Wire port multiplexer between the internal rtps_peer_emulator (when
//   emu_enable == 1) and the external AXI4-Stream wire port (DMA1 / RMII MAC).
//   Includes a 1-stage AXI4-Stream skid buffer on s_axis_wire to isolate
//   external FIFO pointer timing from internal RX reassembly CEs.
// =============================================================================

module rtps_wire_mux #(
    parameter integer DATA_WIDTH  = 32,
    parameter integer REASM_DEPTH = 1536
)(
    input  wire                  clk,
    input  wire                  rst_n_local,

    // Control & Emulator Configuration
    input  wire                  emu_enable,
    input  wire                  wire_swap_loopback,
    input  wire [31:0]           fault_inject_reg,
    input  wire                  tx_ip_frag_mode,
    input  wire [15:0]           cfg_frag_size,
    input  wire [95:0]           peer_guid_prefix,
    input  wire [31:0]           peer_writer_id,
    input  wire [31:0]           peer_reader_id,

    // Internal Engine TX -> Wire/Emulator
    input  wire [DATA_WIDTH-1:0] eng_tx_tdata,
    input  wire                  eng_tx_tvalid,
    output wire                  eng_tx_tready,
    input  wire                  eng_tx_tlast,

    // Wire/Emulator -> Internal Engine RX
    output wire [DATA_WIDTH-1:0] eng_rx_tdata,
    output wire                  eng_rx_tvalid,
    input  wire                  eng_rx_tready,
    output wire                  eng_rx_tlast,

    // External AXI4-Stream Wire Port (DMA1 / RMII MAC)
    output wire [DATA_WIDTH-1:0] m_axis_wire_tdata,
    output wire                  m_axis_wire_tvalid,
    input  wire                  m_axis_wire_tready,
    output wire                  m_axis_wire_tlast,

    input  wire [DATA_WIDTH-1:0] s_axis_wire_tdata,
    input  wire                  s_axis_wire_tvalid,
    output wire                  s_axis_wire_tready,
    input  wire                  s_axis_wire_tlast
);

    wire [DATA_WIDTH-1:0] emu_rx_tdata;
    wire                  emu_rx_tvalid;
    wire                  emu_rx_tready;
    wire                  emu_rx_tlast;
    wire                  emu_tx_tready;

    // Engine words are network-order; the 8<->32 converters are LSB-first.
    // When wire_swap_loopback == 1 (and emu_enable == 0), loop eng_tx directly
    // back to eng_rx through the 1-stage skid buffer (zero BRAM, 100+ MB ready).
    wire use_wire_loop = !emu_enable && wire_swap_loopback;

    assign m_axis_wire_tdata  = {eng_tx_tdata[7:0], eng_tx_tdata[15:8], eng_tx_tdata[23:16], eng_tx_tdata[31:24]};
    assign m_axis_wire_tvalid = (emu_enable || wire_swap_loopback) ? 1'b0 : eng_tx_tvalid;
    assign m_axis_wire_tlast  = eng_tx_tlast;

    // AXI4-Stream 1-stage skid buffer on s_axis_wire (or loopback TX)
    reg                   wire_in_valid, wire_skid_valid;
    reg  [DATA_WIDTH-1:0] wire_in_data,  wire_skid_data;
    reg                   wire_in_last,  wire_skid_last;

    wire [DATA_WIDTH-1:0] ext_in_data  = use_wire_loop ? eng_tx_tdata  : s_axis_wire_tdata;
    wire                  ext_in_valid = use_wire_loop ? eng_tx_tvalid : s_axis_wire_tvalid;
    wire                  ext_in_last  = use_wire_loop ? eng_tx_tlast  : s_axis_wire_tlast;

    wire wire_in_accept = !emu_enable && ext_in_valid && !wire_skid_valid;
    wire wire_out_fire  = !emu_enable && wire_in_valid && eng_rx_tready;

    assign eng_tx_tready = emu_enable         ? emu_tx_tready :
                           wire_swap_loopback ? !wire_skid_valid :
                                                m_axis_wire_tready;

    always @(posedge clk) begin
        if (!rst_n_local) begin
            wire_in_valid   <= 1'b0;
            wire_skid_valid <= 1'b0;
            wire_in_data    <= {DATA_WIDTH{1'b0}};
            wire_skid_data  <= {DATA_WIDTH{1'b0}};
            wire_in_last    <= 1'b0;
            wire_skid_last  <= 1'b0;
        end else begin
            if (wire_in_accept) begin
                if (!wire_in_valid || wire_out_fire) begin
                    wire_in_valid <= 1'b1;
                    wire_in_data  <= ext_in_data;
                    wire_in_last  <= ext_in_last;
                end else begin
                    wire_skid_valid <= 1'b1;
                    wire_skid_data  <= ext_in_data;
                    wire_skid_last  <= ext_in_last;
                end
            end else if (wire_out_fire) begin
                if (wire_skid_valid) begin
                    wire_in_valid   <= 1'b1;
                    wire_in_data    <= wire_skid_data;
                    wire_in_last    <= wire_skid_last;
                    wire_skid_valid <= 1'b0;
                end else begin
                    wire_in_valid   <= 1'b0;
                end
            end
        end
    end

    assign eng_rx_tdata       = emu_enable    ? emu_rx_tdata  :
                                use_wire_loop ? wire_in_data  :
                                {wire_in_data[7:0], wire_in_data[15:8], wire_in_data[23:16], wire_in_data[31:24]};
    assign eng_rx_tvalid      = emu_enable ? emu_rx_tvalid : wire_in_valid;
    assign eng_rx_tlast       = emu_enable ? emu_rx_tlast  : wire_in_last;
    assign emu_rx_tready      = emu_enable ? eng_rx_tready : 1'b0;
    assign s_axis_wire_tready = (emu_enable || wire_swap_loopback) ? 1'b1 : !wire_skid_valid;

    rtps_peer_emulator #(
        .DATA_WIDTH  (DATA_WIDTH),
        .MAX_DGRAM_W (REASM_DEPTH)
    ) u_rtps_peer_emulator (
        .clk              (clk),
        .rst_n            (rst_n_local && emu_enable),
        .fault_ctrl       (fault_inject_reg),
        .tx_ip_frag_mode  (tx_ip_frag_mode),
        .cfg_frag_size    (cfg_frag_size),
        .peer_guid_prefix (peer_guid_prefix),
        .peer_writer_id   (peer_writer_id),
        .peer_reader_id   (peer_reader_id),
        .s_axis_tdata     (eng_tx_tdata),
        .s_axis_tvalid    (emu_enable ? eng_tx_tvalid : 1'b0),
        .s_axis_tready    (emu_tx_tready),
        .s_axis_tlast     (eng_tx_tlast),
        .m_axis_tdata     (emu_rx_tdata),
        .m_axis_tvalid    (emu_rx_tvalid),
        .m_axis_tready    (emu_rx_tready),
        .m_axis_tlast     (emu_rx_tlast)
    );

endmodule
