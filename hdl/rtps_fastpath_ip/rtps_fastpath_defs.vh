// =============================================================================
// Shared Definitions Header: rtps_fastpath_defs.vh
//
// Contains AXI4-Lite CSR word indices, Master FSM states, and RX Reassembly /
// RTPS Submessage Walker FSM states shared across rtps_fastpath_ip submodules.
// =============================================================================

// -----------------------------------------------------------------------------
// AXI4-Lite Register Word Indices (s_axi_a*addr[CFG_ADDR_WIDTH+1:2])
// -----------------------------------------------------------------------------
localparam [7:0]
    TRIGGER_ADDR                = 8'h00, // 0x00: Write forces FSM abort/reset to S_IDLE
    STATUS_ADDR                 = 8'h01, // 0x04: Read status / Write ACK IRQ or kick
    CTRL_ADDR                   = 8'h02, // 0x08: [0]=en, [3:1]=writers, [4]=emu_en, [5]=commit, [6]=wdog_en, [7]=plugin_rtps_rx_mode
    EXPECTED_CRC_ADDR           = 8'h03, // 0x0C: Write expected DMA0 CRC / Read calc CRC
    PACING_TIMER_ADDR           = 8'h04, // 0x10: Wire pacing delay cycles per beat
    CFG_PAYLOAD_ADDR            = 8'h05, // 0x14: Sample size in 32-bit words (default 1024 = 4096B)
    WDOG_PET_ADDR               = 8'h06, // 0x18: Write pets watchdog / Read remaining ticks
    WDOG_LIMIT_ADDR             = 8'h07, // 0x1C: Watchdog timeout ticks (default 10,000,000 = 100ms @ 100MHz)
    RX_TIMEOUT_ADDR             = 8'h08, // 0x20: RX feedback timeout ticks (default 90,000 = 900us @ 100MHz)
    LAST_CYCLE_TICKS            = 8'h09, // 0x24: Elapsed clock ticks in last RTPS cycle
    LOCAL_GUID_0_ADDR           = 8'h0A, // 0x28: Local Cyclone Participant GuidPrefix[95:64]
    LOCAL_GUID_1_ADDR           = 8'h0B, // 0x2C: Local Cyclone Participant GuidPrefix[63:32]
    LOCAL_GUID_2_ADDR           = 8'h0C, // 0x30: Local Cyclone Participant GuidPrefix[31:0]
    LOCAL_SRC_IP_ADDR           = 8'h0D, // 0x34: Dedicated secondary source IP for PL TX (Q16)
    LOCAL_MAC_LO_ADDR           = 8'h0E, // 0x38: Local MAC [31:0]
    LOCAL_MAC_HI_ADDR           = 8'h0F, // 0x3C: {VendorId[15:0], LocalMAC[47:32]}
    TS_OFFSET_LO_ADDR           = 8'h10, // 0x40: Epoch Timestamp Offset Lo
    TS_OFFSET_HI_ADDR           = 8'h11, // 0x44: Epoch Timestamp Offset Hi
    FAULT_INJECT_ADDR           = 8'h12, // 0x48: Emulator fault injection
    STAT_TX_SAMPLES             = 8'h13, // 0x4C: Total TX samples published
    STAT_RX_SAMPLES             = 8'h14, // 0x50: Total valid newer RX samples accepted
    STAT_SEQ_GAPS               = 8'h15, // 0x54: Cumulative seqNum gaps (lost samples)
    STAT_STALE_DROPS            = 8'h16, // 0x58: Duplicate/stale seqNum drops
    STAT_CSUM_ERRS              = 8'h17, // 0x5C: IPv4 or UDP checksum errors
    STAT_SKIPPED_SUBMSG         = 8'h18, // 0x60: Unknown/vendor submessages skipped by RX walker
    LAST_RX_SEQ_LO              = 8'h19, // 0x64: Last accepted RX seqNum[31:0]
    LAST_RX_SEQ_HI              = 8'h1A, // 0x68: Last accepted RX seqNum[63:32]
    LAST_RX_TS_LO               = 8'h1B, // 0x6C: Last latched RX INFO_TS[31:0]
    LAST_RX_TS_HI               = 8'h1C, // 0x70: Last latched RX INFO_TS[63:32]
    LAST_RX_RTPS_WORDS          = 8'h1D, // 0x74: Word count of last accepted full RTPS message
    CLK_PERIOD_ADDR             = 8'h1E, // 0x78: Clock period in nanoseconds
    IDENTITY_CTRL_ADDR          = 8'h1F, // 0x7C: [0]=local_identity_valid, [1]=writer_context_valid, [2]=clear_seq
    GENERATION_ADDR             = 8'h60, // 0x180: Shadow writer generation number
    COMMIT_STATUS_ADDR          = 8'h61, // 0x184: Active writer generation number / commit status
    TYPE_PROFILE_ADDR           = 8'h62, // 0x188: Type profile & strict payload contract
    LAST_RX_SRC_IP_ADDR         = 8'h63, // 0x18C: Last accepted RX source IPv4
    LAST_RX_PORTS_ADDR          = 8'h64, // 0x190: Last accepted RX {src_port[15:0], dst_port[15:0]}
    LAST_RX_META_ADDR           = 8'h65, // 0x194: Last accepted RX {24'd0, 6'd0, reader_slot[1:0]}
    LAST_RX_SRC_MAC_LO          = 8'h66, // 0x198: Ingress learned Remote Client MAC [31:0]
    LAST_RX_SRC_MAC_HI          = 8'h67, // 0x19C: {16'd0, Ingress learned Remote Client MAC [47:32]}
    ARP_CLIENT_MAC_LO           = 8'h68, // 0x1A0: ARP learned Remote Client MAC [31:0]
    ARP_CLIENT_MAC_HI           = 8'h69, // 0x1A4: {16'd0, ARP learned Remote Client MAC [47:32]}
    STAT_ICMP_ECHO_REPLIES_ADDR = 8'h6A, // 0x1A8: Total ICMP Echo Replies sent
    STAT_ICMP_ERRORS_ADDR       = 8'h6B, // 0x1AC: Total ICMP Errors received
    LAST_ICMP_ERR_INFO_ADDR     = 8'h6C, // 0x1B0: {icmp_type[7:0], icmp_code[7:0], next_hop_mtu[15:0]}
    LAST_ICMP_ERR_DST_IP_ADDR   = 8'h6D, // 0x1B4: Offending IPv4 destination IP
    LAST_ICMP_ERR_PORTS_ADDR    = 8'h6E, // 0x1B8: Offending UDP {src_port[15:0], dst_port[15:0]}
    LAST_ICMP_ERR_IP_ID_ADDR    = 8'h6F, // 0x1BC: {16'd0, offending IPv4 Identification}
    CFG_FRAG_SIZE_ADDR          = 8'h70; // 0x1C0: TX DATA_FRAG fragmentSize in bytes (default 1400)

// -----------------------------------------------------------------------------
// Master TX FSM States (rtps_tx_engine, always asynchronous)
// -----------------------------------------------------------------------------
localparam [3:0]
    S_IDLE         = 4'd0,
    S_CHECK_CRC    = 4'd1,
    S_TX_FRAG_INIT = 4'd2,
    S_TX_FRAG_SEND = 4'd3,
    S_DONE_IRQ     = 4'd5;

// -----------------------------------------------------------------------------
// Decoupled RX Egress FSM States (rtps_s2mm_egress)
// -----------------------------------------------------------------------------
localparam [1:0]
    RX_EGRESS_IDLE = 2'd0,
    RX_EGRESS_IRQ  = 2'd1,
    RX_EGRESS_WAIT = 2'd2,
    RX_EGRESS_RUN  = 2'd3;

// -----------------------------------------------------------------------------
// RX Reassembly & RTPS Submessage Walker FSM States (PART A)
// -----------------------------------------------------------------------------
localparam [5:0]
    R_IDLE            = 6'd0,
    R_CHECK_UDP       = 6'd1,
    R_RTPS_HDR_0      = 6'd2,
    R_RTPS_HDR_1      = 6'd3,
    R_RTPS_GUID_0     = 6'd4,
    R_RTPS_GUID_1     = 6'd5,
    R_RTPS_GUID_2     = 6'd6,
    R_SUBMSG_HDR      = 6'd7,
    R_SUBMSG_HDR_WAIT = 6'd8,
    R_INFO_TS_WAIT    = 6'd9,
    R_INFO_TS_0       = 6'd10,
    R_INFO_TS_1       = 6'd11,
    R_DATA_HDR_1      = 6'd12,
    R_DATA_HDR_2      = 6'd13,
    R_DATA_SEQ_HI     = 6'd14,
    R_DATA_SEQ_LO     = 6'd15,
    R_SKIP_QOS_WAIT   = 6'd16,
    R_SKIP_QOS        = 6'd17,
    R_COMMIT_SAMPLE   = 6'd18,
    R_INFO_DST_WAIT   = 6'd19,
    R_INFO_DST_0      = 6'd20,
    R_INFO_DST_1      = 6'd21,
    R_INFO_DST_2      = 6'd22,
    R_DATA_READER_ID  = 6'd23,
    R_REASM_CHECK     = 6'd24,
    R_CAM_MATCH       = 6'd25,
    R_PKT_TAIL_WR     = 6'd26,
    R_UDP_CSUM_W0     = 6'd27,
    R_UDP_CSUM_W1     = 6'd28,
    R_COPY_DATA_PRIME = 6'd29,
    R_COPY_DATA_LOOP  = 6'd30,
    R_DF_HDR_1        = 6'd31,
    R_DF_READER_ID    = 6'd32,
    R_DF_WRITER_ID    = 6'd33,
    R_DF_SEQ_HI       = 6'd34,
    R_DF_SEQ_LO       = 6'd35,
    R_DF_FRAG_START   = 6'd36,
    R_DF_FRAG_INFO    = 6'd37,
    R_DF_SAMPLE_SIZE  = 6'd38,
    R_DF_VALIDATE     = 6'd39,
    R_DF_CALC         = 6'd40,
    R_DF_COPY_PRIME   = 6'd41,
    R_DF_COPY_LOOP    = 6'd42,
    R_DF_SYNTH_HDR    = 6'd43,
    R_UDP_CSUM_W2     = 6'd44,
    R_CAM_WAIT        = 6'd45,
    R_SUBMSG_HDR_WAIT2 = 6'd46,
    R_SKIP_QOS_WAIT2   = 6'd47;
