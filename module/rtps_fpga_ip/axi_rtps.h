#ifndef _AXI_RTPS_H_
#define _AXI_RTPS_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#define XILINX_RTPS_DEV_NAME   "axi_dma_dev"
#define XILINX_RTPS_CLASS_NAME "axi_dma"

/* AXI DMA0 Register Space (Sample Path) */
#define XAXIDMA_DEFAULT_BASEADDR     0x40400000
#define XAXIDMA_DEFAULT_REG_SIZE     0x10000

/* Hybrid PS+PL RTPS v2.x Fast-Path IP Register Space */
#define RTPS_IP_DEFAULT_BASEADDR     0x40000000
#define RTPS_IP_DEFAULT_REG_SIZE     0x1000 /* 4 KB CSR & Endpoint Descriptor Table aperture */

/* Interrupt numbers (ARM GIC SPI numbers) */
#define XAXIDMA_DEFAULT_RX_IRQ       29 /* SPI 29 -> HW IRQ 61 (S2MM) */
#define XAXIDMA_DEFAULT_TX_IRQ       30 /* SPI 30 -> HW IRQ 62 (MM2S) */
#define RTPS_IP_DEFAULT_IRQ          31 /* SPI 31 -> HW IRQ 63 (IP Done / Error) */

/* =========================================================================
 * RTPS Fast-Path IP Global CSR Offsets (matching rtps_fastpath_ip.v)
 * ========================================================================= */
#define RTPS_IP_TRIGGER_OFFSET           0x00 /* Word 0: Abort / Force Reset to S_IDLE */
#define RTPS_IP_STATUS_OFFSET            0x04 /* Word 1: Read status / Write IRQ ACK & S2MM kick */
#define RTPS_IP_CTRL_OFFSET              0x08 /* Word 2: [0]=enable, [3:1]=writers, [4]=emu_en, [5]=commit, [6]=wdog_en, [7]=plugin_rx_rtps_mode */
#define RTPS_IP_CRC_OFFSET               0x0C /* Word 3: Write expected_crc / Read calc_crc */
#define RTPS_IP_PACING_TIMER_OFFSET      0x10 /* Word 4: Wire pacing delay ticks (0 = sim/loopback max speed) */
#define RTPS_IP_PAYLOAD_WORDS_OFFSET     0x14 /* Word 5: Sample payload words (1024 = 4,096 bytes) */
#define RTPS_IP_WDOG_HEARTBEAT_OFFSET    0x18 /* Word 6: Write any value to reset hardware watchdog */
#define RTPS_IP_WDOG_LIMIT_OFFSET        0x1C /* Word 7: Watchdog timeout ticks (default 10,000,000 = 100 ms) */
#define RTPS_IP_RX_TIMEOUT_OFFSET        0x20 /* Word 8: Max wait ticks for peer Feedback in S_WAIT_RX_FB */
#define RTPS_IP_CYCLE_TICKS_OFFSET       0x24 /* Word 9: Hardware clock ticks elapsed in last cycle */
#define RTPS_IP_LOCAL_GUID_0_OFFSET      0x28 /* Word 10: Local Participant guidPrefix[95:64] */
#define RTPS_IP_LOCAL_GUID_1_OFFSET      0x2C /* Word 11: Local Participant guidPrefix[63:32] */
#define RTPS_IP_LOCAL_GUID_2_OFFSET      0x30 /* Word 12: Local Participant guidPrefix[31:0]  */
#define RTPS_IP_LOCAL_IP_OFFSET          0x34 /* Word 13: Local IPv4 Address */
#define RTPS_IP_LOCAL_MAC_LO_OFFSET      0x38 /* Word 14: Local Source MAC[31:0] */
#define RTPS_IP_LOCAL_MAC_HI_OFFSET      0x3C /* Word 15: Local Source MAC[47:32] */
#define RTPS_IP_TS_OFFSET_LO_OFFSET      0x40 /* Word 16: Timestamp clock offset [31:0] */
#define RTPS_IP_TS_OFFSET_HI_OFFSET      0x44 /* Word 17: Timestamp clock offset [63:32] */
#define RTPS_IP_FAULT_INJECT_OFFSET      0x48 /* Word 18: Emulator fault injection bits [3:0] */
#define RTPS_IP_STAT_TX_SAMPLES_OFFSET   0x4C /* Word 19: Total RTPS DATA samples transmitted */
#define RTPS_IP_STAT_RX_SAMPLES_OFFSET   0x50 /* Word 20: Total valid RTPS DATA samples accepted */
#define RTPS_IP_STAT_SEQ_GAPS_OFFSET     0x54 /* Word 21: Total sequence number gaps detected */
#define RTPS_IP_STAT_STALE_DROPS_OFFSET  0x58 /* Word 22: Total duplicate/stale samples dropped */
#define RTPS_IP_STAT_CSUM_ERRS_OFFSET    0x5C /* Word 23: Total IPv4/UDP checksum errors */
#define RTPS_IP_STAT_SKIP_SUBMSG_OFFSET  0x60 /* Word 24: Total non-DATA/vendor submessages skipped */
#define RTPS_IP_LAST_RX_SEQ_LO_OFFSET    0x64 /* Word 25: Last accepted RX SequenceNumber[31:0] */
#define RTPS_IP_LAST_RX_SEQ_HI_OFFSET    0x68 /* Word 26: Last accepted RX SequenceNumber[63:32] */
#define RTPS_IP_LAST_RX_TS_LO_OFFSET     0x6C /* Word 27: Last accepted RX INFO_TS fraction */
#define RTPS_IP_LAST_RX_TS_HI_OFFSET     0x70 /* Word 28: Last accepted RX INFO_TS seconds */
#define RTPS_IP_LAST_RX_RTPS_WORDS       0x74 /* Word 29: Total 32-bit words in latest RX RTPS message (default 1039 = 4,156 B) */
#define RTPS_IP_CLK_PERIOD_OFFSET        0x78 /* Word 30: Clock period in nanoseconds (e.g. 10 for 100MHz, 8 for 125MHz) */

/* CTRL_ADDR (0x08) Bit Masks */
#define RTPS_CTRL_ENABLE                 (1U << 0)
#define RTPS_CTRL_NUM_WRITERS_SHIFT      1
#define RTPS_CTRL_EMU_EN                 (1U << 4)
#define RTPS_CTRL_COMMIT_EDT             (1U << 5)
#define RTPS_CTRL_WDOG_EN                (1U << 6)
#define RTPS_CTRL_PLUGIN_RX_RTPS_MODE    (1U << 7) /* Option B: S2MM outputs full 4,156B RTPS message */
#define RTPS_CTRL_PUB_ONLY               (1U << 8) /* Publish-only mode: do not wait for feedback */
#define RTPS_CTRL_SUB_ONLY               (1U << 9) /* Subscribe-only mode: unsolicited wire RX from S_IDLE */

/* STATUS_ADDR (0x04) Bit Masks */
#define RTPS_STATUS_ACK_IRQ              (1U << 0) /* Write Bit 0: Acknowledge Level-High IRQ */
#define RTPS_STATUS_KICK_EGRESS          (1U << 1) /* Write Bit 1: Kick S2MM Egress Streaming */
#define RTPS_STATUS_IRQ_MASK             (1U << 0)
#define RTPS_STATUS_STATE_MASK           0x0000001EU
#define RTPS_STATUS_ERR_CRC              (1U << 5)
#define RTPS_STATUS_ERR_NO_MATCH         (1U << 6)
#define RTPS_STATUS_ERR_RX_TIMEOUT       (1U << 7)
#define RTPS_STATUS_ERR_CSUM             (1U << 8)
#define RTPS_STATUS_PUB_ONLY             (1U << 9)
#define RTPS_STATUS_IRQ_IS_RX            (1U << 10) /* 1 = RX sample ready in BRAM, 0 = TX done / err */
#define RTPS_STATUS_FRAGS_SHIFT          11
#define RTPS_STATUS_FRAGS_MASK           0x00003800U
#define RTPS_STATUS_WDOG_EXPIRED         (1U << 14)
#define RTPS_STATUS_LIFECYCLE_OK         (1U << 15)
#define RTPS_STATUS_RX_SAMPLES_SHIFT     16
#define RTPS_STATUS_ERR_ALL_MASK         0x000001E0U

/* Shadow Endpoint Descriptor Table (EDT) Offsets */
#define RTPS_MAX_ENDPOINTS               4
#define RTPS_SHADOW_WR_BASE              0x080
#define RTPS_SHADOW_WR_STRIDE            0x020
#define RTPS_SHADOW_RD_BASE              0x100
#define RTPS_SHADOW_RD_STRIDE            0x020
#define RTPS_ACTIVE_WR_BASE              0x200
#define RTPS_ACTIVE_RD_BASE              0x280

#define RTPS_WR_CTRL(i)                  (RTPS_SHADOW_WR_BASE + ((i) * RTPS_SHADOW_WR_STRIDE) + 0x00)
#define RTPS_WR_DST_IP(i)                (RTPS_SHADOW_WR_BASE + ((i) * RTPS_SHADOW_WR_STRIDE) + 0x04)
#define RTPS_WR_PORTS(i)                 (RTPS_SHADOW_WR_BASE + ((i) * RTPS_SHADOW_WR_STRIDE) + 0x08)
#define RTPS_WR_DST_MAC_LO(i)            (RTPS_SHADOW_WR_BASE + ((i) * RTPS_SHADOW_WR_STRIDE) + 0x0C)
#define RTPS_WR_DST_MAC_HI(i)            (RTPS_SHADOW_WR_BASE + ((i) * RTPS_SHADOW_WR_STRIDE) + 0x10)
#define RTPS_WR_ENTITY_ID(i)             (RTPS_SHADOW_WR_BASE + ((i) * RTPS_SHADOW_WR_STRIDE) + 0x14)
#define RTPS_WR_READER_ID(i)             (RTPS_SHADOW_WR_BASE + ((i) * RTPS_SHADOW_WR_STRIDE) + 0x18)
#define RTPS_WR_SEQ_SEED(i)              (RTPS_SHADOW_WR_BASE + ((i) * RTPS_SHADOW_WR_STRIDE) + 0x1C)

#define RTPS_RD_CTRL(i)                  (RTPS_SHADOW_RD_BASE + ((i) * RTPS_SHADOW_RD_STRIDE) + 0x00)
#define RTPS_RD_GUID_0(i)                (RTPS_SHADOW_RD_BASE + ((i) * RTPS_SHADOW_RD_STRIDE) + 0x04)
#define RTPS_RD_GUID_1(i)                (RTPS_SHADOW_RD_BASE + ((i) * RTPS_SHADOW_RD_STRIDE) + 0x08)
#define RTPS_RD_GUID_2(i)                (RTPS_SHADOW_RD_BASE + ((i) * RTPS_SHADOW_RD_STRIDE) + 0x0C)
#define RTPS_RD_WRITER_ID(i)             (RTPS_SHADOW_RD_BASE + ((i) * RTPS_SHADOW_RD_STRIDE) + 0x10)
#define RTPS_RD_READER_ID(i)             (RTPS_SHADOW_RD_BASE + ((i) * RTPS_SHADOW_RD_STRIDE) + 0x14)

/* Sample & RTPS Frame Sizing Constants */
#define RTPS_FAST_SAMPLE_WORDS           1024
#define RTPS_FAST_SAMPLE_BYTES           (RTPS_FAST_SAMPLE_WORDS * 4) /* 4,096 B */
#define RTPS_WIRE_HEADER_WORDS           15
#define RTPS_WIRE_HEADER_BYTES           (RTPS_WIRE_HEADER_WORDS * 4) /* 60 B: RTPS(20)+INFO_TS(12)+DATA(24)+CDR(4) */
#define RTPS_FULL_MSG_WORDS              (RTPS_WIRE_HEADER_WORDS + RTPS_FAST_SAMPLE_WORDS) /* 1,039 W */
#define RTPS_FULL_MSG_BYTES              (RTPS_FULL_MSG_WORDS * 4)    /* 4,156 B */

#define RTPS_IP_MAX_WORDS                65536
#define RTPS_IP_MAX_BYTES                (RTPS_IP_MAX_WORDS * 4)
#define XAXIDMA_DEFAULT_BUF_SIZE         RTPS_IP_MAX_BYTES

/* AXI DMA Register Offsets */
#define XAXIDMA_MM2S_DMACR_OFFSET        0x00
#define XAXIDMA_MM2S_DMASR_OFFSET        0x04
#define XAXIDMA_MM2S_CURDESC_OFFSET      0x08
#define XAXIDMA_MM2S_TAILDESC_OFFSET     0x10

#define XAXIDMA_S2MM_DMACR_OFFSET        0x30
#define XAXIDMA_S2MM_DMASR_OFFSET        0x34
#define XAXIDMA_S2MM_CURDESC_OFFSET      0x38
#define XAXIDMA_S2MM_TAILDESC_OFFSET     0x40

/* DMACR control bits */
#define XAXIDMA_CR_RUNSTOP_MASK          0x00000001
#define XAXIDMA_CR_RESET_MASK            0x00000004
#define XAXIDMA_CR_IRQ_IOC_MASK          0x00001000
#define XAXIDMA_CR_IRQ_DLY_MASK          0x00002000
#define XAXIDMA_CR_IRQ_ERR_MASK          0x00004000
#define XAXIDMA_CR_IRQ_ALL_MASK          0x00007000

/* DMASR status bits */
#define XAXIDMA_SR_HALTED_MASK           0x00000001
#define XAXIDMA_SR_IDLE_MASK             0x00000002
#define XAXIDMA_SR_SG_INCL_MASK          0x00000008
#define XAXIDMA_SR_ERR_INTERNAL_MASK     0x00000100
#define XAXIDMA_SR_ERR_SLAVE_MASK        0x00000200
#define XAXIDMA_SR_ERR_DECODE_MASK       0x00000400
#define XAXIDMA_SR_ERR_ALL_MASK          0x00000700
#define XAXIDMA_SR_IRQ_IOC_MASK          0x00001000
#define XAXIDMA_SR_IRQ_DLY_MASK          0x00002000
#define XAXIDMA_SR_IRQ_ERR_MASK          0x00004000
#define XAXIDMA_SR_IRQ_ALL_MASK          0x00007000

/* Buffer Descriptor (BD) Control & Status bits */
#define XAXIDMA_BD_CTRL_TXSOF_MASK       0x08000000
#define XAXIDMA_BD_CTRL_TXEOF_MASK       0x04000000
#define XAXIDMA_BD_CTRL_LENGTH_MASK      0x007FFFFF

#define XAXIDMA_BD_STS_CMPLT_MASK        0x80000000
#define XAXIDMA_BD_STS_ERROR_MASK        0x70000000
#define XAXIDMA_BD_STS_RXSOF_MASK        0x08000000
#define XAXIDMA_BD_STS_RXEOF_MASK        0x04000000
#define XAXIDMA_BD_STS_ACTUAL_LEN_MASK   0x007FFFFF

/* MMAP Offsets for TX and RX coherent buffers (Rule 2b) */
#define AXI_DMA_MMAP_TX_OFFSET           0x00000000
#define AXI_DMA_MMAP_RX_OFFSET           0x00100000 /* 1MB page offset to map RX buffer */

/* =========================================================================
 * Option B Control-Plane Hook & Endpoint Descriptor Table Structures
 * Populated by on_publication_matched() / on_subscription_matched()
 * ========================================================================= */
struct rtps_writer_edt_entry {
    __u8  valid;           /* 1 = matched & active */
    __u8  use_multicast;   /* 1 = multicast locator */
    __u16 payload_words;   /* 1024 words = 4,096 bytes */
    __u32 dst_ip;          /* Peer/Multicast IPv4 address (host order) */
    __u16 src_port;        /* Local RTPS UDP source port (e.g., 7411) */
    __u16 dst_port;        /* Remote RTPS user-unicast/multicast port (e.g., 7410) */
    __u32 dst_mac_lo;      /* Resolved Next-Hop / Multicast MAC [31:0] */
    __u16 dst_mac_hi;      /* Resolved Next-Hop / Multicast MAC [47:32] */
    __u16 reserved;
    __u32 writer_entity_id;/* Local DDS DataWriter entityId (e.g., 0x00000102) */
    __u32 reader_entity_id;/* Remote DDS DataReader entityId or ENTITYID_UNKNOWN (0) */
    __u32 seq_seed;        /* Starting SequenceNumber Lo after discovery handoff */
};

struct rtps_reader_cam_entry {
    __u8  valid;           /* 1 = CAM entry active */
    __u8  reserved[3];
    __u32 guid_prefix[3];  /* Remote Participant 96-bit guidPrefix[95:0] */
    __u32 writer_entity_id;/* Remote DDS DataWriter entityId (e.g., 0x00000202) */
    __u32 reader_entity_id;/* Local DDS DataReader entityId (e.g., 0x00000207) */
};

struct rtps_endpoint_cfg {
    __u32 local_guid_prefix[3]; /* Local Participant 96-bit guidPrefix */
    __u32 local_ip;             /* Local IPv4 address */
    __u32 local_mac_lo;         /* Local MAC [31:0] */
    __u16 local_mac_hi;         /* Local MAC [47:32] */
    __u8  num_writers;          /* Number of active TX writers (1..4) */
    __u8  plugin_rx_rtps_mode;  /* 1 = Option B NDDS_Transport_Plugin mode (4,156 B RX) */
    __u32 payload_words;        /* Sample payload words (1024 = 4,096 B) */
    __u32 pacing_timer_ticks;   /* Inter-beat pacing delay (0 = max speed) */
    __u32 rx_timeout_ticks;     /* S_WAIT_RX_FB timeout in 100 MHz clock ticks */
    struct rtps_writer_edt_entry writers[RTPS_MAX_ENDPOINTS];
    struct rtps_reader_cam_entry readers[RTPS_MAX_ENDPOINTS];
};

struct rtps_cycle_diag {
    __u32 status_reg;
    __u32 ctrl_reg;
    __u32 last_cycle_ticks;
    __u32 stat_tx_samples;
    __u32 stat_rx_samples;
    __u32 stat_seq_gaps;
    __u32 stat_stale_drops;
    __u32 stat_csum_errs;
    __u32 stat_skipped_submsg;
    __u64 last_rx_seq_num;
    __u64 last_rx_timestamp;
    __u32 last_rx_rtps_words;
    __u8  completed_frags;
    __u8  err_crc;
    __u8  err_no_match;
    __u8  err_rx_timeout;
    __u8  err_csum;
    __u8  reserved[3];

    /* Extended Driver & AXI DMA Hardware Phase Diagnostics */
    __u32 pre_write_ip_status;
    __u32 isr_pre_ack_ip_status;
    __u32 isr_post_ack_ip_status;
    __u32 isr_ack_polls;
    __u32 post_write_ip_status;
    __u32 pre_read_ip_status;
    __u32 post_kick_ip_status;
    __u32 post_50us_ip_status;
    __u32 post_50us_s2mm_dmasr;
    __u32 post_50us_bd_status;
    __u32 s2mm_dmacr;
    __u32 s2mm_dmasr;
    __u32 s2mm_curdesc;
    __u32 s2mm_taildesc;
    __u32 rx_bd_phys;
    __u32 rx_buf_phys;
    __u32 rx_bd_control;
    __u32 rx_bd_status;
    __u32 rx_buf_words_written;
    __u32 rx_buf_total_words;
    __u32 rx_buf_first_mod_idx;
    __u32 rx_buf_last_mod_idx;
    __u32 rx_buf_sample_words[7]; /* words[0, 1, 2, 8, 14, 15, last] */
    __u32 irq_cnt_tx;
    __u32 irq_cnt_rx;
    __u32 irq_cnt_ip;
    __s32 tx_virq;
    __s32 rx_virq;
    __s32 ip_virq;
};

/* IOCTL Definitions */
#define AXI_RTPS_IOC_MAGIC               'r'
#define AXI_RTPS_IOC_CONFIG_ENDPOINTS    _IOW(AXI_RTPS_IOC_MAGIC, 1, struct rtps_endpoint_cfg)
#define AXI_RTPS_IOC_GET_DIAG            _IOR(AXI_RTPS_IOC_MAGIC, 2, struct rtps_cycle_diag)
#define AXI_RTPS_IOC_PET_WATCHDOG        _IOW(AXI_RTPS_IOC_MAGIC, 3, __u32)
#define AXI_RTPS_IOC_SET_PLUGIN_MODE     _IOW(AXI_RTPS_IOC_MAGIC, 4, __u32)
#define AXI_RTPS_IOC_SET_CRC_BYPASS      _IOW(AXI_RTPS_IOC_MAGIC, 5, __u32)
#define AXI_RTPS_IOC_SET_FAULT_INJECT    _IOW(AXI_RTPS_IOC_MAGIC, 6, __u32)

/* AXI DMA SG Buffer Descriptor Structure (64 bytes aligned) */
struct axi_dma_bd {
    __u32 next_desc;
    __u32 next_desc_msb;
    __u32 buf_addr;
    __u32 buf_addr_msb;
    __u32 reserved0;
    __u32 reserved1;
    __u32 control;
    __u32 status;
    __u32 app[5];
    __u32 reserved2[3];
} __attribute__((aligned(64)));

#endif /* _AXI_RTPS_H_ */

