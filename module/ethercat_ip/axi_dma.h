#ifndef _AXI_DMA_H_
#define _AXI_DMA_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#define XILINX_DMA_DEV_NAME "axi_dma_dev"
#define XILINX_DMA_CLASS_NAME "axi_dma"

/* AXI DMA Register Space */
#define XAXIDMA_DEFAULT_BASEADDR 0x40400000
#define XAXIDMA_DEFAULT_REG_SIZE 0x10000

/* Custom Hybrid EtherCAT Master IP Register Space */
#define BRAM_IP_DEFAULT_BASEADDR 0x40000000
#define BRAM_IP_DEFAULT_REG_SIZE 0x1000 /* 4 KB CSR & Descriptor aperture */

/* Interrupt numbers (SPI numbers) */
#define XAXIDMA_DEFAULT_RX_IRQ 29 /* SPI 29 -> HW IRQ 61 (S2MM) */
#define XAXIDMA_DEFAULT_TX_IRQ 30 /* SPI 30 -> HW IRQ 62 (MM2S) */
#define BRAM_IP_DEFAULT_IRQ 31    /* SPI 31 -> HW IRQ 63 (IP Done / Error) */

/* EtherCAT Master IP Global CSR Offsets (matching ecat_cyclic_master_ip.v) */
#define BRAM_IP_TRIGGER_OFFSET       0x00 /* Word 0: Abort / Force Reset to S_IDLE */
#define BRAM_IP_STATUS_OFFSET        0x04 /* Word 1: Read status / Write IRQ ACK & kick */
#define BRAM_IP_CTRL_OFFSET          0x08 /* Word 2: [3:0]=num_descs, [4]=emu_en */
#define BRAM_IP_CRC_OFFSET           0x0C /* Word 3: Write expected_crc / Read calc_crc */
#define ECAT_IP_PACING_TIMER_OFFSET  0x10 /* Word 4: Wire pacing timer (0=sim speedup, >0=100BASE-TX) */
#define BRAM_IP_TIMER_OFFSET         ECAT_IP_PACING_TIMER_OFFSET /* Legacy alias */
#define BRAM_IP_PAYLOAD_LEN_OFFSET   0x14 /* Word 5: Configurable total PDI words */
#define ECAT_IP_SLAVE_MASK_LO_OFFSET 0x18 /* Word 6: Emulator Slaves 0..31 online mask */
#define ECAT_IP_SLAVE_MASK_HI_OFFSET 0x1C /* Word 7: Emulator Slaves 32..39 online mask */
#define ECAT_IP_CYCLE_TICKS_OFFSET   0x20 /* Word 8: Hardware clock ticks in last cycle */

/* Datagram Descriptor Table Base (8 Descriptors x 16 Bytes = 0x40 .. 0xBF) */
#define ECAT_MAX_DESCRIPTORS         8
#define ECAT_DESC_BASE_OFFSET        0x40
#define ECAT_DESC_STRIDE             0x10
#define ECAT_DESC_LOGICAL_ADDR(i)    (ECAT_DESC_BASE_OFFSET + ((i) * ECAT_DESC_STRIDE) + 0x00)
#define ECAT_DESC_CTRL(i)            (ECAT_DESC_BASE_OFFSET + ((i) * ECAT_DESC_STRIDE) + 0x04)
#define ECAT_DESC_PDI_WKC(i)         (ECAT_DESC_BASE_OFFSET + ((i) * ECAT_DESC_STRIDE) + 0x08)
#define ECAT_DESC_STATUS(i)          (ECAT_DESC_BASE_OFFSET + ((i) * ECAT_DESC_STRIDE) + 0x0C)

/* EtherCAT Command Codes */
#define ECAT_CMD_LRD                 0x0A /* Logical Read       (+1 WKC/slave) */
#define ECAT_CMD_LWR                 0x0B /* Logical Write      (+2 WKC/slave) */
#define ECAT_CMD_LRW                 0x0C /* Logical Read/Write (+3 WKC/slave) */

/* Maximum Ethernet/EtherCAT Single-Datagram Payload */
#define ECAT_MAX_FRAME_PAYLOAD_BYTES 1484 /* Word-aligned max payload <= 1486B */

/* Process Data Image (PDI) Buffer Limits */
#define BRAM_IP_MAX_WORDS            65536
#define BRAM_IP_MAX_BYTES            (BRAM_IP_MAX_WORDS * 4)
#define XAXIDMA_DEFAULT_BUF_SIZE     BRAM_IP_MAX_BYTES

/* STATUS_ADDR Bit Masks */
#define BRAM_IP_STATUS_IRQ_MASK      0x00000001
#define BRAM_IP_STATUS_STATE_MASK    0x0000001E
#define BRAM_IP_STATUS_CRC_ERR       0x00000020
#define ECAT_IP_STATUS_WKC_ERR       0x00000040
#define ECAT_IP_STATUS_FRAMES_SHIFT  7
#define ECAT_IP_STATUS_FRAMES_MASK   0x00000780
#define ECAT_IP_STATUS_WKC_SHIFT     16

/* AXI DMA Register Offsets */
#define XAXIDMA_MM2S_DMACR_OFFSET    0x00
#define XAXIDMA_MM2S_DMASR_OFFSET    0x04
#define XAXIDMA_MM2S_CURDESC_OFFSET  0x08
#define XAXIDMA_MM2S_TAILDESC_OFFSET 0x10

#define XAXIDMA_S2MM_DMACR_OFFSET    0x30
#define XAXIDMA_S2MM_DMASR_OFFSET    0x34
#define XAXIDMA_S2MM_CURDESC_OFFSET  0x38
#define XAXIDMA_S2MM_TAILDESC_OFFSET 0x40

/* DMACR control bits */
#define XAXIDMA_CR_RUNSTOP_MASK      0x00000001
#define XAXIDMA_CR_RESET_MASK        0x00000004
#define XAXIDMA_CR_IRQ_IOC_MASK      0x00001000
#define XAXIDMA_CR_IRQ_DLY_MASK      0x00002000
#define XAXIDMA_CR_IRQ_ERR_MASK      0x00004000
#define XAXIDMA_CR_IRQ_ALL_MASK      0x00007000

/* DMASR status bits */
#define XAXIDMA_SR_HALTED_MASK       0x00000001
#define XAXIDMA_SR_IDLE_MASK         0x00000002
#define XAXIDMA_SR_SG_INCL_MASK      0x00000008
#define XAXIDMA_SR_ERR_INTERNAL_MASK 0x00000100
#define XAXIDMA_SR_ERR_SLAVE_MASK    0x00000200
#define XAXIDMA_SR_ERR_DECODE_MASK   0x00000400
#define XAXIDMA_SR_ERR_ALL_MASK      0x00000700
#define XAXIDMA_SR_IRQ_IOC_MASK      0x00001000
#define XAXIDMA_SR_IRQ_DLY_MASK      0x00002000
#define XAXIDMA_SR_IRQ_ERR_MASK      0x00004000
#define XAXIDMA_SR_IRQ_ALL_MASK      0x00007000

/* Buffer Descriptor (BD) Control & Status bits */
#define XAXIDMA_BD_CTRL_TXSOF_MASK      0x08000000
#define XAXIDMA_BD_CTRL_TXEOF_MASK      0x04000000
#define XAXIDMA_BD_CTRL_LENGTH_MASK     0x007FFFFF

#define XAXIDMA_BD_STS_CMPLT_MASK       0x80000000
#define XAXIDMA_BD_STS_ERROR_MASK       0x70000000
#define XAXIDMA_BD_STS_RXSOF_MASK       0x08000000
#define XAXIDMA_BD_STS_RXEOF_MASK       0x04000000
#define XAXIDMA_BD_STS_ACTUAL_LEN_MASK  0x007FFFFF

/* MMAP Offsets for TX and RX coherent buffers */
#define AXI_DMA_MMAP_TX_OFFSET          0x00000000
#define AXI_DMA_MMAP_RX_OFFSET          0x00100000 /* 1MB page offset to map RX buffer */

/* =========================================================================
 * Stack-Agnostic EtherCAT OP-State Handoff & Diagnostic Structures
 * (Used by SOEM, IgH EtherLab, or standalone robotics control loops)
 * ========================================================================= */
struct ecat_datagram_desc {
    __u32 logical_addr;    /* Starting FMMU logical byte address */
    __u16 payload_words;   /* Number of 32-bit words in this frame (1..371) */
    __u8  ecat_cmd;        /* ECAT_CMD_LRW (0x0C), LRD (0x0A), LWR (0x0B) */
    __u8  datagram_idx;    /* Frame sequence index */
    __u16 pdi_word_offset; /* Word offset inside TxPDO/RxPDO PDI buffer */
    __u16 expected_wkc;    /* Expected Working Counter from slaves */
};

struct ecat_master_cfg {
    __u32 num_descriptors; /* Active frames per cycle (1..8) */
    __u32 total_pdo_words; /* Total Process Data Image size in 32-bit words */
    __u32 wire_timer_ticks;/* Pacing delay ticks per beat (0 = max speed) */
    __u32 slave_mask_lo;   /* Slaves 0..31 online bitmask (for emulator) */
    __u32 slave_mask_hi;   /* Slaves 32..39 online bitmask (for emulator) */
    struct ecat_datagram_desc descs[ECAT_MAX_DESCRIPTORS];
};

struct ecat_cycle_diag {
    __u32 status_reg;
    __u32 last_cycle_ticks;
    __u16 total_actual_wkc;
    __u8  completed_frames;
    __u8  wkc_err;
    __u8  crc_err;
    __u8  reserved[3];
    __u32 desc_status[ECAT_MAX_DESCRIPTORS];
};

/* IOCTL Definitions */
#define AXI_DMA_IOC_MAGIC           'd'
#define AXI_DMA_IOC_SET_TIMER       _IOW(AXI_DMA_IOC_MAGIC, 1, __u32)
#define AXI_DMA_IOC_GET_TIMER       _IOR(AXI_DMA_IOC_MAGIC, 2, __u32)
#define AXI_DMA_IOC_SET_PAYLOAD_LEN _IOW(AXI_DMA_IOC_MAGIC, 3, __u32)
#define AXI_DMA_IOC_GET_PAYLOAD_LEN _IOR(AXI_DMA_IOC_MAGIC, 4, __u32)
#define AXI_DMA_IOC_SET_CRC_BYPASS  _IOW(AXI_DMA_IOC_MAGIC, 5, __u32)
#define AXI_DMA_IOC_GET_CRC_BYPASS  _IOR(AXI_DMA_IOC_MAGIC, 6, __u32)
#define AXI_DMA_IOC_CONFIG_ECAT     _IOW(AXI_DMA_IOC_MAGIC, 7, struct ecat_master_cfg)
#define AXI_DMA_IOC_GET_ECAT_DIAG   _IOR(AXI_DMA_IOC_MAGIC, 8, struct ecat_cycle_diag)
#define AXI_DMA_IOC_SET_SLAVE_MASK  _IOW(AXI_DMA_IOC_MAGIC, 9, __u64)

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

#endif /* _AXI_DMA_H_ */

