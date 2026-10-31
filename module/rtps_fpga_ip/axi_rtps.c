#include "axi_rtps.h"

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/wait.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Birju Vachhani");
MODULE_DESCRIPTION(
    "Hybrid PS+PL RTPS v2.x Cyclic Fast-Path & Option B RTI Transport Plugin DMA Driver for PYNQ-Z2");
MODULE_VERSION("1.0");

struct axi_rtps_dev {
    dev_t dev_num;
    struct cdev cdev;
    struct class* class;
    struct device* device;
    struct platform_device* pdev;

    void __iomem* regs;     /* AXI DMA0 registers (Sample Path) */
    void __iomem* ip_regs;  /* Custom RTPS Fast-Path IP CSRs & EDT */
    bool ip_regs_allocated; /* True if manual ioremap was used */

    /* Coherent DMA buffers (Rule 2b) */
    struct axi_dma_bd* tx_bd;
    dma_addr_t tx_bd_phys;
    void* tx_buf;
    dma_addr_t tx_buf_phys;

    struct axi_dma_bd* rx_bd;
    dma_addr_t rx_bd_phys;
    void* rx_buf;
    dma_addr_t rx_buf_phys;

    /* Zero-copy mmap user virtual addresses */
    unsigned long tx_user_addr;
    unsigned long rx_user_addr;
    u32 bypass_crc;
    u32 plugin_rx_rtps_mode;

    /* Synchronization & Diagnostics */
    struct mutex tx_lock;
    struct mutex rx_lock;
    wait_queue_head_t tx_waitq;
    wait_queue_head_t rx_waitq;
    wait_queue_head_t ip_waitq;
    wait_queue_head_t tx_ip_waitq;
    wait_queue_head_t rx_packet_waitq;
    u32 mode_pub_only;
    int tx_done;
    int rx_done;
    int ip_done;
    int tx_ip_done;
    int rx_packet_avail;
    int tx_err;
    int rx_err;
    int crc_err;
    int no_match_err;
    int rx_timeout_err;
    int csum_err;
    u32 last_ip_status;

    /* Extended Phase & DMA Hardware Diagnostics */
    u32 pre_write_ip_status;
    u32 isr_pre_ack_ip_status;
    u32 isr_post_ack_ip_status;
    u32 isr_ack_polls;
    u32 post_write_ip_status;
    u32 pre_read_ip_status;
    u32 post_kick_ip_status;
    u32 post_50us_ip_status;
    u32 post_50us_s2mm_dmasr;
    u32 post_50us_bd_status;
    u32 last_s2mm_dmacr;
    u32 last_s2mm_dmasr;
    u32 last_s2mm_curdesc;
    u32 last_s2mm_taildesc;
    u32 last_rx_bd_control;
    u32 last_rx_bd_status;
    u32 rx_buf_words_written;
    u32 rx_buf_total_words;
    u32 rx_buf_first_mod_idx;
    u32 rx_buf_last_mod_idx;
    u32 rx_buf_sample_words[7];
    u32 irq_cnt_tx;
    u32 irq_cnt_rx;
    u32 irq_cnt_ip;

    /* Mapped Linux Virtual IRQs */
    int tx_virq;
    int rx_virq;
    int ip_virq;
};

static struct axi_rtps_dev rtps_device;

static inline void dma_write(struct axi_rtps_dev* dev, u32 offset, u32 val) {
    iowrite32(val, dev->regs + offset);
}

static inline u32 dma_read(struct axi_rtps_dev* dev, u32 offset) {
    return ioread32(dev->regs + offset);
}

/* Fast 32-bit Additive Checksum for strictly DMA ingress validation (Option 1) */
static u32 calc_crc32(const u32* data, size_t num_words) {
    u32 sum = 0;
    size_t i;
    for (i = 0; i < num_words; i++) {
        sum += data[i];
    }
    return sum;
}

/* Dynamic ARM GIC SPI IRQ mapper (fallback when not in device tree) */
static int map_gic_spi(int irq_val) {
    struct of_phandle_args oirq;
    int spi_num = (irq_val >= 32) ? (irq_val - 32) : irq_val;

    oirq.np = of_find_compatible_node(NULL, NULL, "arm,cortex-a9-gic");
    if (!oirq.np)
        return -ENODEV;

    oirq.args_count = 3;
    oirq.args[0] = 0;       /* 0 = SPI */
    oirq.args[1] = spi_num; /* 29, 30, or 31 */
    oirq.args[2] = 4;       /* 4 = IRQ_TYPE_LEVEL_HIGH */

    return irq_create_of_mapping(&oirq);
}

static int axi_dma_reset(struct axi_rtps_dev* dev) {
    int timeout = 10000;

    dma_write(dev, XAXIDMA_MM2S_DMACR_OFFSET, XAXIDMA_CR_RESET_MASK);
    dma_write(dev, XAXIDMA_S2MM_DMACR_OFFSET, XAXIDMA_CR_RESET_MASK);

    while (timeout--) {
        u32 mm2s_cr = dma_read(dev, XAXIDMA_MM2S_DMACR_OFFSET);
        u32 s2mm_cr = dma_read(dev, XAXIDMA_S2MM_DMACR_OFFSET);
        if (!(mm2s_cr & XAXIDMA_CR_RESET_MASK) &&
            !(s2mm_cr & XAXIDMA_CR_RESET_MASK))
            return 0;
        udelay(1);
    }

    pr_err("axi_rtps: DMA reset timeout\n");
    return -ETIMEDOUT;
}

/* Unified Interrupt Service Routine for S2MM, MM2S, and RTPS Fast-Path IP (Rule 2c) */
static irqreturn_t axi_rtps_isr(int irq, void* dev_id) {
    struct axi_rtps_dev* dev = (struct axi_rtps_dev*)dev_id;
    u32 tx_status = dev->regs ? dma_read(dev, XAXIDMA_MM2S_DMASR_OFFSET) : 0;
    u32 rx_status = dev->regs ? dma_read(dev, XAXIDMA_S2MM_DMASR_OFFSET) : 0;
    irqreturn_t handled = IRQ_NONE;

    /* 1. Handle TX (MM2S) interrupt if active */
    if (tx_status & XAXIDMA_SR_IRQ_ALL_MASK) {
        dma_write(dev, XAXIDMA_MM2S_DMASR_OFFSET,
                  tx_status & XAXIDMA_SR_IRQ_ALL_MASK);
        dev->irq_cnt_tx++;
        if (tx_status & XAXIDMA_SR_ERR_ALL_MASK) {
            dev->tx_err = 1;
            pr_err("axi_rtps: MM2S error status: 0x%08x\n", tx_status);
        }
        dev->tx_done = 1;
        wake_up_interruptible(&dev->tx_waitq);
        handled = IRQ_HANDLED;
    }

    /* 2. Handle RX (S2MM) interrupt if active */
    if (rx_status & XAXIDMA_SR_IRQ_ALL_MASK) {
        dma_write(dev, XAXIDMA_S2MM_DMASR_OFFSET,
                  rx_status & XAXIDMA_SR_IRQ_ALL_MASK);
        dev->irq_cnt_rx++;
        if (rx_status & XAXIDMA_SR_ERR_ALL_MASK) {
            dev->rx_err = 1;
            pr_err("axi_rtps: S2MM error status: 0x%08x\n", rx_status);
        }
        dev->rx_done = 1;
        wake_up_interruptible(&dev->rx_waitq);
        handled = IRQ_HANDLED;
    }

    /* 3. Handle Custom RTPS Fast-Path IP Level-High interrupt (Rule 1b & 2c) */
    if (dev->ip_regs) {
        u32 ip_status = ioread32(dev->ip_regs + RTPS_IP_STATUS_OFFSET);

        if (ip_status & (RTPS_STATUS_IRQ_MASK | RTPS_STATUS_ERR_ALL_MASK)) {
            int ack_polls = 0;
            u32 post_ack = ip_status;

            dev->last_ip_status = ip_status;
            dev->isr_pre_ack_ip_status = ip_status;
            dev->irq_cnt_ip++;

            /* Rule 2c: Acknowledge and drop PL Level-High interrupt line immediately (Bit 0) */
            iowrite32(RTPS_STATUS_ACK_IRQ, dev->ip_regs + RTPS_IP_STATUS_OFFSET);

            /*
             * Flush posted AXI-Lite write across M_AXI_GP0 and wait for PL Level-High
             * ip_done_irq (Bit 0) to clear before returning IRQ_HANDLED to GIC.
             * This prevents spurious GIC re-entry while the posted write is in flight.
             */
            for (ack_polls = 1; ack_polls <= 32; ack_polls++) {
                post_ack = ioread32(dev->ip_regs + RTPS_IP_STATUS_OFFSET);
                if (!(post_ack & RTPS_STATUS_IRQ_MASK))
                    break;
            }
            dev->isr_post_ack_ip_status = post_ack;
            dev->isr_ack_polls = (u32)ack_polls;

            /* Separate IP compute/validation error flags cleanly */
            if (ip_status & RTPS_STATUS_ERR_CRC) {
                u32 calc_crc = ioread32(dev->ip_regs + RTPS_IP_CRC_OFFSET);
                dev->crc_err = 1;
                pr_err("axi_rtps: DMA0 ingress CRC-32 mismatch! Calc: 0x%08x, Status: 0x%08x\n",
                       calc_crc, ip_status);
                iowrite32(0, dev->ip_regs + RTPS_IP_TRIGGER_OFFSET);
            } else {
                dev->crc_err = 0;
            }

            dev->no_match_err   = (ip_status & RTPS_STATUS_ERR_NO_MATCH) ? 1 : 0;
            dev->rx_timeout_err = (ip_status & RTPS_STATUS_ERR_RX_TIMEOUT) ? 1 : 0;
            dev->csum_err       = (ip_status & RTPS_STATUS_ERR_CSUM) ? 1 : 0;

            if (ip_status & RTPS_STATUS_IRQ_IS_RX) {
                dev->rx_packet_avail = 1;
                wake_up_interruptible(&dev->rx_packet_waitq);
            } else {
                dev->tx_ip_done = 1;
                wake_up_interruptible(&dev->tx_ip_waitq);
            }

            dev->ip_done = 1;
            wake_up_interruptible(&dev->ip_waitq);
            handled = IRQ_HANDLED;
        }
    }

    return handled;
}

static long axi_rtps_unlocked_ioctl(struct file* file, unsigned int cmd,
                                    unsigned long arg) {
    struct axi_rtps_dev* dev = file->private_data;
    u32 val;
    u32 ctrl_word;
    struct rtps_endpoint_cfg cfg;
    struct rtps_cycle_diag diag;
    int i;

    if (!dev->ip_regs)
        return -ENODEV;

    switch (cmd) {
        case AXI_RTPS_IOC_CONFIG_ENDPOINTS:
            if (copy_from_user(&cfg, (void __user*)arg, sizeof(cfg)))
                return -EFAULT;
            if (cfg.num_writers == 0 || cfg.num_writers > RTPS_MAX_ENDPOINTS)
                return -EINVAL;
            if (cfg.payload_words == 0 || cfg.payload_words > RTPS_IP_MAX_WORDS)
                return -EINVAL;

            /* Program Local Participant Identity CSRs (does NOT trigger S_IDLE transition) */
            {
                u32 mac_hi = ((u32)(cfg.local_vendor_id ? cfg.local_vendor_id : 0x0101) << 16) |
                             ((u32)cfg.local_mac_hi & 0xFFFFU);
                u32 type_prof = cfg.type_profile ? cfg.type_profile : RTPS_TYPE_PROFILE_RTI_DEFAULT;
                u32 id_ctrl = (cfg.local_identity_valid ? RTPS_IDENTITY_LOCAL_VALID : 0U) |
                              (cfg.writer_context_valid ? RTPS_IDENTITY_WRITER_VALID : 0U);

                iowrite32(cfg.local_guid_prefix[0], dev->ip_regs + RTPS_IP_LOCAL_GUID_0_OFFSET);
                iowrite32(cfg.local_guid_prefix[1], dev->ip_regs + RTPS_IP_LOCAL_GUID_1_OFFSET);
                iowrite32(cfg.local_guid_prefix[2], dev->ip_regs + RTPS_IP_LOCAL_GUID_2_OFFSET);
                iowrite32(cfg.local_ip,             dev->ip_regs + RTPS_IP_LOCAL_IP_OFFSET);
                iowrite32(cfg.local_mac_lo,         dev->ip_regs + RTPS_IP_LOCAL_MAC_LO_OFFSET);
                iowrite32(mac_hi,                   dev->ip_regs + RTPS_IP_LOCAL_MAC_HI_OFFSET);
                iowrite32(cfg.pacing_timer_ticks,   dev->ip_regs + RTPS_IP_PACING_TIMER_OFFSET);
                iowrite32(cfg.payload_words,        dev->ip_regs + RTPS_IP_PAYLOAD_WORDS_OFFSET);
                if (cfg.rx_timeout_ticks > 0)
                    iowrite32(cfg.rx_timeout_ticks, dev->ip_regs + RTPS_IP_RX_TIMEOUT_OFFSET);

                /* Program Type Profile & Strict Payload Contract (Section 8) */
                iowrite32(type_prof, dev->ip_regs + RTPS_IP_TYPE_PROFILE_OFFSET);

                /* Program Shadow Generation (Section 2) */
                iowrite32(cfg.generation, dev->ip_regs + RTPS_IP_GENERATION_OFFSET);

                /* Program Identity Control (Section 1) */
                iowrite32(id_ctrl, dev->ip_regs + RTPS_IP_IDENTITY_CTRL_OFFSET);
            }

            /* Program Shadow Writer EDT [0..3] */
            for (i = 0; i < RTPS_MAX_ENDPOINTS; i++) {
                u8 wr_state = cfg.writers[i].lifecycle_state ? cfg.writers[i].lifecycle_state :
                              (cfg.writers[i].valid ? FPGA_LIFECYCLE_ACTIVE : FPGA_LIFECYCLE_DISABLED);
                u32 wr_ctrl = ((u32)(cfg.writers[i].payload_words & 0x0FFFU) << 4) |
                              ((u32)(wr_state & 0x7U) << 1) |
                              (cfg.writers[i].valid ? 1U : 0U);
                u32 wr_ports = ((u32)cfg.writers[i].src_port << 16) |
                               ((u32)cfg.writers[i].dst_port);

                iowrite32(wr_ctrl,                        dev->ip_regs + RTPS_WR_CTRL(i));
                iowrite32(cfg.writers[i].dst_ip,          dev->ip_regs + RTPS_WR_DST_IP(i));
                iowrite32(wr_ports,                       dev->ip_regs + RTPS_WR_PORTS(i));
                iowrite32(cfg.writers[i].dst_mac_lo,      dev->ip_regs + RTPS_WR_DST_MAC_LO(i));
                iowrite32(cfg.writers[i].dst_mac_hi,      dev->ip_regs + RTPS_WR_DST_MAC_HI(i));
                iowrite32(cfg.writers[i].writer_entity_id,dev->ip_regs + RTPS_WR_ENTITY_ID(i));
                iowrite32(cfg.writers[i].reader_entity_id,dev->ip_regs + RTPS_WR_READER_ID(i));
                iowrite32(cfg.writers[i].seq_seed,        dev->ip_regs + RTPS_WR_SEQ_SEED(i));
            }

            /* Program Shadow Reader CAM [0..3] */
            for (i = 0; i < RTPS_MAX_ENDPOINTS; i++) {
                u8 rd_state = cfg.readers[i].lifecycle_state ? cfg.readers[i].lifecycle_state :
                              (cfg.readers[i].valid ? FPGA_LIFECYCLE_ACTIVE : FPGA_LIFECYCLE_DISABLED);
                u32 rd_ctrl = ((u32)cfg.readers[i].expected_dst_port << 16) |
                              ((u32)(rd_state & 0x7U) << 1) |
                              (cfg.readers[i].valid ? 1U : 0U);
                iowrite32(rd_ctrl,                          dev->ip_regs + RTPS_RD_CTRL(i));
                iowrite32(cfg.readers[i].guid_prefix[0],    dev->ip_regs + RTPS_RD_GUID_0(i));
                iowrite32(cfg.readers[i].guid_prefix[1],    dev->ip_regs + RTPS_RD_GUID_1(i));
                iowrite32(cfg.readers[i].guid_prefix[2],    dev->ip_regs + RTPS_RD_GUID_2(i));
                iowrite32(cfg.readers[i].writer_entity_id,  dev->ip_regs + RTPS_RD_WRITER_ID(i));
                iowrite32(cfg.readers[i].reader_entity_id,  dev->ip_regs + RTPS_RD_READER_ID(i));
                iowrite32(cfg.readers[i].expected_src_ip,   dev->ip_regs + RTPS_RD_SRC_IP(i));
            }

            /* Atomically commit Shadow -> Active EDT at S_IDLE boundary */
            dev->plugin_rx_rtps_mode = cfg.plugin_rx_rtps_mode ? 1 : 0;
            dev->mode_pub_only       = cfg.mode_pub_only ? 1 : 0;
            ctrl_word = RTPS_CTRL_ENABLE |
                        ((cfg.num_writers & 0x07U) << RTPS_CTRL_NUM_WRITERS_SHIFT) |
                        (cfg.enable_peer_emu ? RTPS_CTRL_EMU_EN : 0U) |
                        (cfg.mode_pub_only ? RTPS_CTRL_PUB_ONLY : 0U) |
                        RTPS_CTRL_COMMIT_EDT |
                        (dev->plugin_rx_rtps_mode ? RTPS_CTRL_PLUGIN_RX_RTPS_MODE : 0U);
            iowrite32(ctrl_word, dev->ip_regs + RTPS_IP_CTRL_OFFSET);

            /* Poll COMMIT_STATUS until active generation matches cfg.generation (Section 10) */
            if (cfg.generation > 0) {
                int poll_i;
                for (poll_i = 0; poll_i < 100; poll_i++) {
                    if (ioread32(dev->ip_regs + RTPS_IP_COMMIT_STATUS_OFFSET) == cfg.generation)
                        break;
                    udelay(5);
                }
            }
            return 0;

        case AXI_RTPS_IOC_GET_DIAG:
            memset(&diag, 0, sizeof(diag));
            diag.status_reg          = ioread32(dev->ip_regs + RTPS_IP_STATUS_OFFSET);
            diag.ctrl_reg            = ioread32(dev->ip_regs + RTPS_IP_CTRL_OFFSET);
            diag.last_cycle_ticks    = ioread32(dev->ip_regs + RTPS_IP_CYCLE_TICKS_OFFSET);
            diag.stat_tx_samples     = ioread32(dev->ip_regs + RTPS_IP_STAT_TX_SAMPLES_OFFSET);
            diag.stat_rx_samples     = ioread32(dev->ip_regs + RTPS_IP_STAT_RX_SAMPLES_OFFSET);
            diag.stat_seq_gaps       = ioread32(dev->ip_regs + RTPS_IP_STAT_SEQ_GAPS_OFFSET);
            diag.stat_stale_drops    = ioread32(dev->ip_regs + RTPS_IP_STAT_STALE_DROPS_OFFSET);
            diag.stat_csum_errs      = ioread32(dev->ip_regs + RTPS_IP_STAT_CSUM_ERRS_OFFSET);
            diag.stat_skipped_submsg = ioread32(dev->ip_regs + RTPS_IP_STAT_SKIP_SUBMSG_OFFSET);
            diag.last_rx_seq_num     = ((u64)ioread32(dev->ip_regs + RTPS_IP_LAST_RX_SEQ_HI_OFFSET) << 32) |
                                       ioread32(dev->ip_regs + RTPS_IP_LAST_RX_SEQ_LO_OFFSET);
            diag.last_rx_timestamp   = ((u64)ioread32(dev->ip_regs + RTPS_IP_LAST_RX_TS_HI_OFFSET) << 32) |
                                       ioread32(dev->ip_regs + RTPS_IP_LAST_RX_TS_LO_OFFSET);
            diag.last_rx_rtps_words  = ioread32(dev->ip_regs + RTPS_IP_LAST_RX_RTPS_WORDS);
            diag.last_rx_src_ip      = ioread32(dev->ip_regs + RTPS_IP_LAST_RX_SRC_IP_OFFSET);
            {
                u32 ports_word       = ioread32(dev->ip_regs + RTPS_IP_LAST_RX_PORTS_OFFSET);
                u32 meta_word        = ioread32(dev->ip_regs + RTPS_IP_LAST_RX_META_OFFSET);
                diag.last_rx_src_port= (u16)(ports_word >> 16);
                diag.last_rx_dst_port= (u16)(ports_word & 0xFFFFU);
                diag.last_rx_cam_idx = meta_word & 0x03U;
            }
            diag.active_generation   = ioread32(dev->ip_regs + RTPS_IP_COMMIT_STATUS_OFFSET);
            diag.commit_status       = diag.active_generation;
            diag.type_profile        = ioread32(dev->ip_regs + RTPS_IP_TYPE_PROFILE_OFFSET);
            diag.identity_ctrl       = ioread32(dev->ip_regs + RTPS_IP_IDENTITY_CTRL_OFFSET);
            diag.completed_frags     = (u8)((diag.status_reg & RTPS_STATUS_FRAGS_MASK) >>
                                            RTPS_STATUS_FRAGS_SHIFT);
            diag.err_crc             = (diag.status_reg & RTPS_STATUS_ERR_CRC) ? 1 : 0;
            diag.err_no_match        = (diag.status_reg & RTPS_STATUS_ERR_NO_MATCH) ? 1 : 0;
            diag.err_rx_timeout      = (diag.status_reg & RTPS_STATUS_ERR_RX_TIMEOUT) ? 1 : 0;
            diag.err_csum            = (diag.status_reg & RTPS_STATUS_ERR_CSUM) ? 1 : 0;

            /* Populate Extended Phase & DMA Hardware Diagnostics */
            diag.pre_write_ip_status    = dev->pre_write_ip_status;
            diag.isr_pre_ack_ip_status  = dev->isr_pre_ack_ip_status;
            diag.isr_post_ack_ip_status = dev->isr_post_ack_ip_status;
            diag.isr_ack_polls          = dev->isr_ack_polls;
            diag.post_write_ip_status   = dev->post_write_ip_status;
            diag.pre_read_ip_status     = dev->pre_read_ip_status;
            diag.post_kick_ip_status    = dev->post_kick_ip_status;
            diag.post_50us_ip_status    = dev->post_50us_ip_status;
            diag.post_50us_s2mm_dmasr   = dev->post_50us_s2mm_dmasr;
            diag.post_50us_bd_status    = dev->post_50us_bd_status;
            diag.s2mm_dmacr             = dev->last_s2mm_dmacr;
            diag.s2mm_dmasr             = dev->last_s2mm_dmasr;
            diag.s2mm_curdesc           = dev->last_s2mm_curdesc;
            diag.s2mm_taildesc          = dev->last_s2mm_taildesc;
            diag.rx_bd_phys             = (u32)dev->rx_bd_phys;
            diag.rx_buf_phys            = (u32)dev->rx_buf_phys;
            diag.rx_bd_control          = dev->last_rx_bd_control;
            diag.rx_bd_status           = dev->last_rx_bd_status;
            diag.rx_buf_words_written   = dev->rx_buf_words_written;
            diag.rx_buf_total_words     = dev->rx_buf_total_words;
            diag.rx_buf_first_mod_idx   = dev->rx_buf_first_mod_idx;
            diag.rx_buf_last_mod_idx    = dev->rx_buf_last_mod_idx;
            for (i = 0; i < 7; i++)
                diag.rx_buf_sample_words[i] = dev->rx_buf_sample_words[i];
            diag.irq_cnt_tx             = dev->irq_cnt_tx;
            diag.irq_cnt_rx             = dev->irq_cnt_rx;
            diag.irq_cnt_ip             = dev->irq_cnt_ip;
            diag.tx_virq                = dev->tx_virq;
            diag.rx_virq                = dev->rx_virq;
            diag.ip_virq                = dev->ip_virq;

            if (copy_to_user((void __user*)arg, &diag, sizeof(diag)))
                return -EFAULT;
            return 0;

        case AXI_RTPS_IOC_PET_WATCHDOG:
            iowrite32(1, dev->ip_regs + RTPS_IP_WDOG_HEARTBEAT_OFFSET);
            return 0;

        case AXI_RTPS_IOC_SET_PLUGIN_MODE:
            if (copy_from_user(&val, (u32 __user*)arg, sizeof(u32)))
                return -EFAULT;
            dev->plugin_rx_rtps_mode = val ? 1 : 0;
            ctrl_word = ioread32(dev->ip_regs + RTPS_IP_CTRL_OFFSET);
            if (dev->plugin_rx_rtps_mode)
                ctrl_word |= RTPS_CTRL_PLUGIN_RX_RTPS_MODE;
            else
                ctrl_word &= ~RTPS_CTRL_PLUGIN_RX_RTPS_MODE;
            iowrite32(ctrl_word, dev->ip_regs + RTPS_IP_CTRL_OFFSET);
            return 0;

        case AXI_RTPS_IOC_SET_CRC_BYPASS:
            if (copy_from_user(&val, (u32 __user*)arg, sizeof(u32)))
                return -EFAULT;
            dev->bypass_crc = val ? 1 : 0;
            return 0;

        case AXI_RTPS_IOC_SET_FAULT_INJECT:
            if (copy_from_user(&val, (u32 __user*)arg, sizeof(u32)))
                return -EFAULT;
            iowrite32(val & 0x0FU, dev->ip_regs + RTPS_IP_FAULT_INJECT_OFFSET);
            return 0;

        case AXI_RTPS_IOC_CONFIGURE_WRITER: {
            struct fpga_writer_endpoint wep;
            u32 wr_ctrl, wr_ports, dst_mac_lo, dst_mac_hi, id_ctrl;
            if (copy_from_user(&wep, (void __user*)arg, sizeof(wep)))
                return -EFAULT;
            if (wep.slot_idx >= RTPS_MAX_ENDPOINTS)
                return -EINVAL;
            i = wep.slot_idx;
            wr_ctrl = ((u32)(wep.payload_words & 0x0FFFU) << 4) |
                      ((u32)(wep.lifecycle_state & 0x7U) << 1) |
                      (wep.lifecycle_state != FPGA_LIFECYCLE_DISABLED ? 1U : 0U);
            wr_ports = ((u32)wep.src_port << 16) | ((u32)wep.dst_port);
            dst_mac_hi = ((u32)wep.dst_mac[0] << 8) | ((u32)wep.dst_mac[1]);
            dst_mac_lo = ((u32)wep.dst_mac[2] << 24) |
                         ((u32)wep.dst_mac[3] << 16) |
                         ((u32)wep.dst_mac[4] << 8) |
                         ((u32)wep.dst_mac[5]);

            iowrite32(wr_ctrl,             dev->ip_regs + RTPS_WR_CTRL(i));
            iowrite32(wep.dst_ip,          dev->ip_regs + RTPS_WR_DST_IP(i));
            iowrite32(wr_ports,            dev->ip_regs + RTPS_WR_PORTS(i));
            iowrite32(dst_mac_lo,          dev->ip_regs + RTPS_WR_DST_MAC_LO(i));
            iowrite32(dst_mac_hi,          dev->ip_regs + RTPS_WR_DST_MAC_HI(i));
            iowrite32(wep.writer_entity_id,dev->ip_regs + RTPS_WR_ENTITY_ID(i));
            iowrite32(wep.reader_entity_id,dev->ip_regs + RTPS_WR_READER_ID(i));
            iowrite32(wep.seq_seed,        dev->ip_regs + RTPS_WR_SEQ_SEED(i));
            if (wep.generation > 0)
                iowrite32(wep.generation,  dev->ip_regs + RTPS_IP_GENERATION_OFFSET);

            /* Ensure local identity and writer context flags match writer state */
            id_ctrl = ioread32(dev->ip_regs + RTPS_IP_IDENTITY_CTRL_OFFSET);
            id_ctrl |= RTPS_IDENTITY_LOCAL_VALID;
            if (wep.lifecycle_state == FPGA_LIFECYCLE_ACTIVE)
                id_ctrl |= RTPS_IDENTITY_WRITER_VALID;
            else if (wep.lifecycle_state == FPGA_LIFECYCLE_DISABLED && i == 0)
                id_ctrl &= ~RTPS_IDENTITY_WRITER_VALID;
            iowrite32(id_ctrl, dev->ip_regs + RTPS_IP_IDENTITY_CTRL_OFFSET);

            /* Ensure Type Profile is set to valid contract if uninitialized */
            val = ioread32(dev->ip_regs + RTPS_IP_TYPE_PROFILE_OFFSET);
            if ((val & 0x000F0000U) == 0)
                iowrite32(RTPS_TYPE_PROFILE_RTI_DEFAULT, dev->ip_regs + RTPS_IP_TYPE_PROFILE_OFFSET);

            return 0;
        }

        case AXI_RTPS_IOC_CONFIGURE_READER: {
            struct fpga_reader_endpoint rep;
            u32 rd_ctrl, g0, g1, g2;
            if (copy_from_user(&rep, (void __user*)arg, sizeof(rep)))
                return -EFAULT;
            if (rep.slot_idx >= RTPS_MAX_ENDPOINTS)
                return -EINVAL;
            i = rep.slot_idx;
            rd_ctrl = ((u32)rep.expected_dst_port << 16) |
                      ((u32)(rep.lifecycle_state & 0x7U) << 1) |
                      (rep.lifecycle_state != FPGA_LIFECYCLE_DISABLED ? 1U : 0U);
            g0 = ((u32)rep.remote_guid_prefix[0] << 24) |
                 ((u32)rep.remote_guid_prefix[1] << 16) |
                 ((u32)rep.remote_guid_prefix[2] << 8)  |
                 ((u32)rep.remote_guid_prefix[3]);
            g1 = ((u32)rep.remote_guid_prefix[4] << 24) |
                 ((u32)rep.remote_guid_prefix[5] << 16) |
                 ((u32)rep.remote_guid_prefix[6] << 8)  |
                 ((u32)rep.remote_guid_prefix[7]);
            g2 = ((u32)rep.remote_guid_prefix[8] << 24) |
                 ((u32)rep.remote_guid_prefix[9] << 16) |
                 ((u32)rep.remote_guid_prefix[10] << 8) |
                 ((u32)rep.remote_guid_prefix[11]);

            iowrite32(rd_ctrl,                   dev->ip_regs + RTPS_RD_CTRL(i));
            iowrite32(g0,                         dev->ip_regs + RTPS_RD_GUID_0(i));
            iowrite32(g1,                         dev->ip_regs + RTPS_RD_GUID_1(i));
            iowrite32(g2,                         dev->ip_regs + RTPS_RD_GUID_2(i));
            iowrite32(rep.remote_writer_entity_id,dev->ip_regs + RTPS_RD_WRITER_ID(i));
            iowrite32(rep.local_reader_entity_id, dev->ip_regs + RTPS_RD_READER_ID(i));
            iowrite32(rep.expected_remote_src_ip, dev->ip_regs + RTPS_RD_SRC_IP(i));
            if (rep.generation > 0)
                iowrite32(rep.generation,         dev->ip_regs + RTPS_IP_GENERATION_OFFSET);
            return 0;
        }

        case AXI_RTPS_IOC_COMMIT_ENDPOINTS: {
            u32 gen, id_ctrl;
            int poll_i;
            if (copy_from_user(&gen, (u32 __user*)arg, sizeof(u32)))
                return -EFAULT;
            if (gen > 0)
                iowrite32(gen, dev->ip_regs + RTPS_IP_GENERATION_OFFSET);

            /* Ensure Identity Control reflects active writer 0 */
            id_ctrl = RTPS_IDENTITY_LOCAL_VALID;
            val = ioread32(dev->ip_regs + RTPS_WR_CTRL(0));
            if ((val & 1U) && (((val >> 1) & 7U) == FPGA_LIFECYCLE_ACTIVE))
                id_ctrl |= RTPS_IDENTITY_WRITER_VALID;
            iowrite32(id_ctrl, dev->ip_regs + RTPS_IP_IDENTITY_CTRL_OFFSET);

            /* Ensure Type Profile is set to valid contract if uninitialized */
            val = ioread32(dev->ip_regs + RTPS_IP_TYPE_PROFILE_OFFSET);
            if ((val & 0x000F0000U) == 0)
                iowrite32(RTPS_TYPE_PROFILE_RTI_DEFAULT, dev->ip_regs + RTPS_IP_TYPE_PROFILE_OFFSET);

            ctrl_word = ioread32(dev->ip_regs + RTPS_IP_CTRL_OFFSET);
            ctrl_word |= RTPS_CTRL_ENABLE | RTPS_CTRL_EMU_EN | RTPS_CTRL_COMMIT_EDT;
            iowrite32(ctrl_word, dev->ip_regs + RTPS_IP_CTRL_OFFSET);

            if (gen > 0) {
                for (poll_i = 0; poll_i < 100; poll_i++) {
                    if (ioread32(dev->ip_regs + RTPS_IP_COMMIT_STATUS_OFFSET) == gen)
                        break;
                    udelay(5);
                }
            }
            return 0;
        }

        default:
            return -ENOTTY;
    }
}

static int axi_rtps_open(struct inode* inode, struct file* file) {
    file->private_data = &rtps_device;
    return 0;
}

static int axi_rtps_release(struct inode* inode, struct file* file) {
    struct axi_rtps_dev* dev = file->private_data;
    if (dev) {
        dev->tx_user_addr = 0;
        dev->rx_user_addr = 0;
    }
    return 0;
}

static void axi_rtps_snapshot_s2mm_and_rxbuf(struct axi_rtps_dev* dev,
                                             size_t transfer_len) {
    const u32* rx_words = (const u32*)dev->rx_buf;
    u32 num_words = (u32)(transfer_len / sizeof(u32));
    u32 written = 0;
    u32 first_idx = 0xFFFFFFFFU;
    u32 last_idx = 0xFFFFFFFFU;
    u32 i;

    rmb();
    dev->last_s2mm_dmacr    = dev->regs ? dma_read(dev, XAXIDMA_S2MM_DMACR_OFFSET) : 0;
    dev->last_s2mm_dmasr    = dev->regs ? dma_read(dev, XAXIDMA_S2MM_DMASR_OFFSET) : 0;
    dev->last_s2mm_curdesc  = dev->regs ? dma_read(dev, XAXIDMA_S2MM_CURDESC_OFFSET) : 0;
    dev->last_s2mm_taildesc = dev->regs ? dma_read(dev, XAXIDMA_S2MM_TAILDESC_OFFSET) : 0;
    dev->last_rx_bd_control = dev->rx_bd ? dev->rx_bd->control : 0;
    dev->last_rx_bd_status  = dev->rx_bd ? dev->rx_bd->status : 0;
    dev->rx_buf_total_words = num_words;

    /*
     * TEMPORARY LATENCY OPTIMIZATION:
     * Full DDR memory scan commented out for testing.
     * Iterating over 1,039 words in uncached DDR memory costs ~15-20 us on ARM Cortex-A9.
     */
    /*
    if (rx_words && num_words > 0) {
        for (i = 0; i < num_words; i++) {
            if (rx_words[i] != 0xDEADBEEFU) {
                written++;
                if (first_idx == 0xFFFFFFFFU)
                    first_idx = i;
                last_idx = i;
            }
        }
        dev->rx_buf_sample_words[0] = (num_words > 0)  ? rx_words[0]  : 0;
        dev->rx_buf_sample_words[1] = (num_words > 1)  ? rx_words[1]  : 0;
        dev->rx_buf_sample_words[2] = (num_words > 2)  ? rx_words[2]  : 0;
        dev->rx_buf_sample_words[3] = (num_words > 8)  ? rx_words[8]  : 0;
        dev->rx_buf_sample_words[4] = (num_words > 14) ? rx_words[14] : 0;
        dev->rx_buf_sample_words[5] = (num_words > 15) ? rx_words[15] : 0;
        dev->rx_buf_sample_words[6] = rx_words[num_words - 1];
    }
    */
    (void)rx_words;
    (void)i;
    (void)written;
    (void)first_idx;
    (void)last_idx;
    dev->rx_buf_words_written = written;
    dev->rx_buf_first_mod_idx = first_idx;
    dev->rx_buf_last_mod_idx  = last_idx;
}

/*
 * S2MM Egress Transfer:
 *   - In Option A (raw FastSample mode): streams 4,096 B (1,024 words) from latest_slot.
 *   - In Option B (NDDS_Transport_Plugin mode, CTRL[7]=1): streams 4,156 B (1,039 words)
 *     containing the hardware-validated 60B RTPS header + 4,096B FastSample payload
 *     directly into FpgaTransport_receive_rEA().
 */
static ssize_t axi_rtps_read(struct file* file, char __user* buf, size_t count,
                             loff_t* ppos) {
    struct axi_rtps_dev* dev = file->private_data;
    size_t transfer_len;
    long ret;
    u32 rx_bytes;

    if (count == 0)
        return 0;

    transfer_len = min_t(size_t, count, RTPS_IP_MAX_BYTES);

    if (mutex_lock_interruptible(&dev->rx_lock))
        return -ERESTARTSYS;

    /*
     * TEMPORARY LATENCY OPTIMIZATION:
     * Sentinel pre-fill commented out for testing.
     * Writing 1,039 words to uncached DDR costs ~15-20 us on ARM Cortex-A9.
     */
    /*
    rx_words = (u32*)dev->rx_buf;
    for (i = 0; i < num_words; i++) {
        rx_words[i] = 0xDEADBEEFU;
    }
    */

    dev->pre_read_ip_status = dev->ip_regs ? ioread32(dev->ip_regs + RTPS_IP_STATUS_OFFSET) : 0;

    /*
     * In asynchronous or decoupled subscriber mode, wait for an incoming packet
     * if the IP is not already holding a completed sample in S_DONE_IRQ (5) or S_WAIT_EGRESS (6).
     */
    if (dev->ip_regs) {
        u32 initial_status = dev->pre_read_ip_status;
        u8 initial_state = (initial_status & RTPS_STATUS_STATE_MASK) >> 1;
        if (initial_state != 5 && initial_state != 6 && !dev->rx_packet_avail) {
            ret = wait_event_interruptible_timeout(dev->rx_packet_waitq,
                                                   dev->rx_packet_avail != 0,
                                                   msecs_to_jiffies(3000));
            if (ret == 0) {
                mutex_unlock(&dev->rx_lock);
                return -ETIMEDOUT;
            } else if (ret < 0) {
                mutex_unlock(&dev->rx_lock);
                return ret;
            }
        }
        dev->rx_packet_avail = 0;
    }

    /* Setup S2MM Buffer Descriptor */
    memset(dev->rx_bd, 0, sizeof(*dev->rx_bd));
    dev->rx_bd->next_desc = (u32)dev->rx_bd_phys;
    dev->rx_bd->buf_addr  = (u32)dev->rx_buf_phys;
    dev->rx_bd->control   = transfer_len & XAXIDMA_BD_CTRL_LENGTH_MASK;
    dev->rx_bd->status    = 0;

    /* Ensure memory writes to descriptor and sentinel buffer are visible to DMA */
    wmb();

    dev->rx_done = 0;
    dev->rx_err  = 0;

    /* Rule 2a.1: Clear pending channel interrupts in DMASR before starting */
    dma_write(dev, XAXIDMA_S2MM_DMASR_OFFSET, XAXIDMA_SR_IRQ_ALL_MASK);

    /* Rule 2a.2: Program CURDESC before setting DMACR.RS = 1 */
    dma_write(dev, XAXIDMA_S2MM_CURDESC_OFFSET, (u32)dev->rx_bd_phys);

    /* Rule 2a.3: Set DMACR.RS = 1 and enable interrupts */
    dma_write(dev, XAXIDMA_S2MM_DMACR_OFFSET,
              XAXIDMA_CR_RUNSTOP_MASK | XAXIDMA_CR_IRQ_IOC_MASK |
                  XAXIDMA_CR_IRQ_ERR_MASK);

    /* Rule 2a.4: Flush memory barrier before writing TAILDESC */
    wmb();
    dma_write(dev, XAXIDMA_S2MM_TAILDESC_OFFSET, (u32)dev->rx_bd_phys);

    /* Guarantee DMA descriptor commit to memory before signalling IP */
    mb();

    /* Kick S2MM egress streaming from RTPS Fast-Path IP */
    if (dev->ip_regs) {
        int poll_us;
        u32 live_status = ioread32(dev->ip_regs + RTPS_IP_STATUS_OFFSET);
        u8 cur_state = (live_status & RTPS_STATUS_STATE_MASK) >> 1;
        if (cur_state == 5) {
            int ack_wait;
            /* If IP is still holding S_DONE_IRQ, ACK IRQ first to transition to S_WAIT_EGRESS */
            iowrite32(RTPS_STATUS_ACK_IRQ, dev->ip_regs + RTPS_IP_STATUS_OFFSET);
            for (ack_wait = 0; ack_wait < 32; ack_wait++) {
                live_status = ioread32(dev->ip_regs + RTPS_IP_STATUS_OFFSET);
                cur_state = (live_status & RTPS_STATUS_STATE_MASK) >> 1;
                if (cur_state != 5)
                    break;
            }
        }
        /*
         * Kick egress streaming using strictly Bit 1 (RTPS_STATUS_KICK_EGRESS = 0x2).
         * Never assert Bit 0 here so that if the IP is in S_IDLE it cannot accidentally
         * trigger a new manual TX/RX cycle!
         */
        iowrite32(RTPS_STATUS_KICK_EGRESS, dev->ip_regs + RTPS_IP_STATUS_OFFSET);
        dev->post_kick_ip_status = ioread32(dev->ip_regs + RTPS_IP_STATUS_OFFSET);

        /*
         * Poll for S2MM completion (up to 2000 us in 1 us steps; 1039 words @ 100MHz = 10.4 us).
         * If m_axis_tlast is not wired to s_axis_s2mm_tlast in the Vivado Block Design,
         * AXI DMA S2MM bursts all 1,039 words into dev->rx_buf in DDR memory and the IP
         * FSM returns from S_TX_RUN to S_IDLE (State 0), while S2MM SG awaits TLAST before
         * setting BD_STS.CMPLT or firing S2MM IOC_Irq. We detect both normal BD completion
         * and TLAST-unwired DDR completion cleanly.
         */
        for (poll_us = 0; poll_us < 2000; poll_us++) {
            rmb();
            live_status = ioread32(dev->ip_regs + RTPS_IP_STATUS_OFFSET);
            cur_state = (live_status & RTPS_STATUS_STATE_MASK) >> 1;

            if (dev->rx_done || (dev->rx_bd->status & XAXIDMA_BD_STS_CMPLT_MASK)) {
                dev->rx_done = 1;
                break;
            }
            if (cur_state == 0) {
                /* IP has completed egress streaming (S_TX_RUN -> S_IDLE) */
                break;
            }
            udelay(1);
        }

        dev->post_50us_ip_status  = live_status;
        dev->post_50us_s2mm_dmasr = dma_read(dev, XAXIDMA_S2MM_DMASR_OFFSET);
        dev->post_50us_bd_status  = dev->rx_bd->status;

        if (!dev->rx_done) {
            if ((dev->post_50us_bd_status & XAXIDMA_BD_STS_CMPLT_MASK) ||
                (dev->post_50us_s2mm_dmasr & XAXIDMA_SR_IRQ_IOC_MASK)) {
                dma_write(dev, XAXIDMA_S2MM_DMASR_OFFSET,
                          dev->post_50us_s2mm_dmasr & XAXIDMA_SR_IRQ_ALL_MASK);
                axi_rtps_snapshot_s2mm_and_rxbuf(dev, transfer_len);
                dev->rx_done = 1;
            } else if (((live_status & RTPS_STATUS_STATE_MASK) >> 1) == 0) {
                axi_rtps_snapshot_s2mm_and_rxbuf(dev, transfer_len);
                axi_dma_reset(dev);
                dev->rx_done = 1;
            }
        } else {
            axi_rtps_snapshot_s2mm_and_rxbuf(dev, transfer_len);
        }
    }

    /* Wait for S2MM completion interrupt if not already completed above */
    if (!dev->rx_done) {
        ret = wait_event_interruptible_timeout(dev->rx_waitq, dev->rx_done != 0,
                                               msecs_to_jiffies(3000));
        axi_rtps_snapshot_s2mm_and_rxbuf(dev, transfer_len);
    } else {
        ret = 1;
    }

    if (ret == 0) {
        u32 live_ip_status = dev->ip_regs ? ioread32(dev->ip_regs + RTPS_IP_STATUS_OFFSET) : 0;
        u32 live_ip_ctrl   = dev->ip_regs ? ioread32(dev->ip_regs + RTPS_IP_CTRL_OFFSET) : 0;
        u32 tx_samples     = dev->ip_regs ? ioread32(dev->ip_regs + RTPS_IP_STAT_TX_SAMPLES_OFFSET) : 0;
        u32 rx_samples     = dev->ip_regs ? ioread32(dev->ip_regs + RTPS_IP_STAT_RX_SAMPLES_OFFSET) : 0;
        u32 rtps_words     = dev->ip_regs ? ioread32(dev->ip_regs + RTPS_IP_LAST_RX_RTPS_WORDS) : 0;

        pr_err("axi_rtps: ==================== S2MM WAIT TIMED OUT ====================\n");
        pr_err("axi_rtps: [Timeline] PreWrite=0x%08x(S%u) -> ISR_Pre=0x%08x(S%u) -> ISR_Post=0x%08x(S%u,polls=%u) -> PostWrite=0x%08x(S%u)\n",
               dev->pre_write_ip_status, (dev->pre_write_ip_status & RTPS_STATUS_STATE_MASK) >> 1,
               dev->isr_pre_ack_ip_status, (dev->isr_pre_ack_ip_status & RTPS_STATUS_STATE_MASK) >> 1,
               dev->isr_post_ack_ip_status, (dev->isr_post_ack_ip_status & RTPS_STATUS_STATE_MASK) >> 1,
               dev->isr_ack_polls,
               dev->post_write_ip_status, (dev->post_write_ip_status & RTPS_STATUS_STATE_MASK) >> 1);
        pr_err("axi_rtps: [Timeline] PreRead=0x%08x(S%u) -> PostKick=0x%08x(S%u) -> Post50us=0x%08x(S%u) -> Timeout=0x%08x(S%u)\n",
               dev->pre_read_ip_status, (dev->pre_read_ip_status & RTPS_STATUS_STATE_MASK) >> 1,
               dev->post_kick_ip_status, (dev->post_kick_ip_status & RTPS_STATUS_STATE_MASK) >> 1,
               dev->post_50us_ip_status, (dev->post_50us_ip_status & RTPS_STATUS_STATE_MASK) >> 1,
               live_ip_status, (live_ip_status & RTPS_STATUS_STATE_MASK) >> 1);
        pr_err("axi_rtps: [IP CSRs ] CTRL=0x%08x, TX_Samples=%u, RX_Samples=%u, Latest_RTPS_Words=%u\n",
               live_ip_ctrl, tx_samples, rx_samples, rtps_words);
        pr_err("axi_rtps: [S2MM DMA] DMACR=0x%08x, DMASR=0x%08x (50us: 0x%08x), CURDESC=0x%08x, TAILDESC=0x%08x (bd_phys=0x%08x)\n",
               dev->last_s2mm_dmacr, dev->last_s2mm_dmasr, dev->post_50us_s2mm_dmasr,
               dev->last_s2mm_curdesc, dev->last_s2mm_taildesc, (u32)dev->rx_bd_phys);
        pr_err("axi_rtps: [S2MM BD ] CTRL=0x%08x (%zu B), STS=0x%08x (50us: 0x%08x), IRQs(tx=%u[v%d], rx=%u[v%d], ip=%u[v%d])\n",
               dev->last_rx_bd_control, transfer_len, dev->last_rx_bd_status, dev->post_50us_bd_status,
               dev->irq_cnt_tx, dev->tx_virq, dev->irq_cnt_rx, dev->rx_virq, dev->irq_cnt_ip, dev->ip_virq);
        pr_err("axi_rtps: [DDR Scan] Written=%u/%u words (first_idx=%u, last_idx=%u) w[0]=0x%08x w[1]=0x%08x w[2]=0x%08x w[8]=0x%08x w[14]=0x%08x w[15]=0x%08x w[last]=0x%08x\n",
               dev->rx_buf_words_written, dev->rx_buf_total_words,
               dev->rx_buf_first_mod_idx, dev->rx_buf_last_mod_idx,
               dev->rx_buf_sample_words[0], dev->rx_buf_sample_words[1],
               dev->rx_buf_sample_words[2], dev->rx_buf_sample_words[3],
               dev->rx_buf_sample_words[4], dev->rx_buf_sample_words[5],
               dev->rx_buf_sample_words[6]);
        pr_err("axi_rtps: =============================================================\n");

        axi_dma_reset(dev);
        mutex_unlock(&dev->rx_lock);
        return -ETIMEDOUT;
    } else if (ret < 0) {
        mutex_unlock(&dev->rx_lock);
        return ret;
    }

    if (dev->rx_err || (dev->rx_bd->status & XAXIDMA_BD_STS_ERROR_MASK)) {
        pr_err("axi_rtps: S2MM hardware transfer error\n");
        mutex_unlock(&dev->rx_lock);
        return -EIO;
    }

    rmb();
    rx_bytes = dev->rx_bd->status & XAXIDMA_BD_STS_ACTUAL_LEN_MASK;
    if (rx_bytes == 0)
        rx_bytes = transfer_len;

    /* Rule 2b: Zero-Copy check: bypass copy_to_user() if user pointer is inside mmap'd VMA */
    if (buf != NULL && !(dev->rx_user_addr &&
                         (unsigned long)buf >= dev->rx_user_addr &&
                         (unsigned long)buf < dev->rx_user_addr + XAXIDMA_DEFAULT_BUF_SIZE)) {
        if (copy_to_user(buf, dev->rx_buf, rx_bytes)) {
            mutex_unlock(&dev->rx_lock);
            return -EFAULT;
        }
    }

    mutex_unlock(&dev->rx_lock);
    return rx_bytes;
}

/*
 * MM2S Ingress Transfer:
 *   Streams 4,096 B (1,024 words) FastSample payload from FpgaTransport_send() into
 *   tx_sample_bram -> hardware prepends RTPS + INFO_TS + DATA + CDR_LE, fragments into
 *   3 IPv4 packets, reassembles & filters peer Feedback, and raises ip_done_irq.
 */
static ssize_t axi_rtps_write(struct file* file, const char __user* buf,
                              size_t count, loff_t* ppos) {
    struct axi_rtps_dev* dev = file->private_data;
    size_t transfer_len;
    size_t num_words;
    u32 crc;
    long ret;

    if (count == 0)
        return 0;

    transfer_len = min_t(size_t, count, RTPS_IP_MAX_BYTES);
    num_words = transfer_len / sizeof(u32);

    if (mutex_lock_interruptible(&dev->tx_lock))
        return -ERESTARTSYS;

    /* Rule 2b: Zero-Copy check: bypass copy_from_user() if user pointer is inside mmap'd VMA */
    if (buf != NULL && !(dev->tx_user_addr &&
                         (unsigned long)buf >= dev->tx_user_addr &&
                         (unsigned long)buf < dev->tx_user_addr + XAXIDMA_DEFAULT_BUF_SIZE)) {
        if (copy_from_user(dev->tx_buf, buf, transfer_len)) {
            mutex_unlock(&dev->tx_lock);
            return -EFAULT;
        }
    }

    if (!dev->ip_regs) {
        pr_err("axi_rtps: RTPS Fast-Path IP registers not mapped\n");
        mutex_unlock(&dev->tx_lock);
        return -ENODEV;
    }

    if (!dev->bypass_crc) {
        crc = calc_crc32((const u32*)dev->tx_buf, num_words);
    } else {
        crc = 0;
    }

    dev->last_ip_status        = 0;
    dev->pre_write_ip_status   = ioread32(dev->ip_regs + RTPS_IP_STATUS_OFFSET);
    dev->isr_pre_ack_ip_status = 0;
    dev->isr_post_ack_ip_status= 0;
    dev->isr_ack_polls         = 0;
    dev->post_write_ip_status  = 0;
    dev->ip_done               = 0;
    dev->tx_ip_done            = 0;
    dev->crc_err               = 0;
    dev->no_match_err          = 0;
    dev->rx_timeout_err        = 0;
    dev->csum_err              = 0;
    dev->tx_done               = 0;
    dev->tx_err                = 0;

    /* Rule 1a: Program expected CRC into IP (does NOT leave S_IDLE) */
    iowrite32(crc, dev->ip_regs + RTPS_IP_CRC_OFFSET);

    /* Setup MM2S Buffer Descriptor */
    memset(dev->tx_bd, 0, sizeof(*dev->tx_bd));
    dev->tx_bd->next_desc = (u32)dev->tx_bd_phys;
    dev->tx_bd->buf_addr  = (u32)dev->tx_buf_phys;
    dev->tx_bd->control   = (transfer_len & XAXIDMA_BD_CTRL_LENGTH_MASK) |
                            XAXIDMA_BD_CTRL_TXSOF_MASK |
                            XAXIDMA_BD_CTRL_TXEOF_MASK;
    dev->tx_bd->status    = 0;
    wmb();

    /* Rule 2a.1: Clear pending TX interrupts in DMASR before starting */
    dma_write(dev, XAXIDMA_MM2S_DMASR_OFFSET, XAXIDMA_SR_IRQ_ALL_MASK);

    /* Rule 2a.2: Program CURDESC before DMACR.RS = 1 */
    dma_write(dev, XAXIDMA_MM2S_CURDESC_OFFSET, (u32)dev->tx_bd_phys);

    /* Rule 2a.3: Set DMACR.RS = 1 and enable interrupts */
    dma_write(dev, XAXIDMA_MM2S_DMACR_OFFSET,
              XAXIDMA_CR_RUNSTOP_MASK | XAXIDMA_CR_IRQ_IOC_MASK |
                  XAXIDMA_CR_IRQ_ERR_MASK);

    /* Rule 2a.4: Flush memory barrier before writing TAILDESC */
    wmb();
    dma_write(dev, XAXIDMA_MM2S_TAILDESC_OFFSET, (u32)dev->tx_bd_phys);

    /* Wait for MM2S DMA transfer completion */
    ret = wait_event_interruptible_timeout(dev->tx_waitq, dev->tx_done != 0,
                                           msecs_to_jiffies(3000));
    if (ret == 0) {
        pr_err("axi_rtps: MM2S TX DMA timed out (DMASR: 0x%08x, BD STS: 0x%08x)\n",
               dma_read(dev, XAXIDMA_MM2S_DMASR_OFFSET), dev->tx_bd->status);
        axi_dma_reset(dev);
        mutex_unlock(&dev->tx_lock);
        return -ETIMEDOUT;
    } else if (ret < 0) {
        mutex_unlock(&dev->tx_lock);
        return ret;
    }

    if (dev->tx_err || (dev->tx_bd->status & XAXIDMA_BD_STS_ERROR_MASK)) {
        pr_err("axi_rtps: MM2S hardware transfer error\n");
        mutex_unlock(&dev->tx_lock);
        return -EIO;
    }

    /* Wait for RTPS Fast-Path 3-Fragment Publication + Peer Reassembly & Validation */
    if (dev->mode_pub_only) {
        ret = wait_event_interruptible_timeout(dev->tx_ip_waitq, dev->tx_ip_done != 0,
                                               msecs_to_jiffies(3000));
    } else {
        ret = wait_event_interruptible_timeout(dev->ip_waitq, dev->ip_done != 0,
                                               msecs_to_jiffies(3000));
    }
    dev->post_write_ip_status = ioread32(dev->ip_regs + RTPS_IP_STATUS_OFFSET);
    if (ret == 0) {
        u32 status = dev->post_write_ip_status;
        pr_err("axi_rtps: RTPS cycle wait timed out (status: 0x%08x [State %u], pre_write: 0x%08x)\n",
               status, (status & RTPS_STATUS_STATE_MASK) >> 1, dev->pre_write_ip_status);
        iowrite32(0, dev->ip_regs + RTPS_IP_TRIGGER_OFFSET);
        mutex_unlock(&dev->tx_lock);
        return -ETIMEDOUT;
    } else if (ret < 0) {
        mutex_unlock(&dev->tx_lock);
        return ret;
    }

    if (dev->crc_err) {
        pr_err("axi_rtps: RTPS IP reported DMA0 CRC-32 error\n");
        mutex_unlock(&dev->tx_lock);
        return -EBADMSG;
    }
    if (dev->no_match_err) {
        pr_err("axi_rtps: RTPS IP reported ERR_NO_MATCH (endpoint disabled or watchdog expired)\n");
        mutex_unlock(&dev->tx_lock);
        return -ENOTCONN;
    }
    if (dev->rx_timeout_err) {
        pr_err("axi_rtps: RTPS IP reported ERR_RX_TIMEOUT waiting for peer Feedback\n");
        mutex_unlock(&dev->tx_lock);
        return -ETIMEDOUT;
    }

    mutex_unlock(&dev->tx_lock);
    return transfer_len;
}

/* Rule 2b: MMAP Interface: Map TX or RX coherent DMA buffer directly to user space */
static int axi_rtps_mmap(struct file* file, struct vm_area_struct* vma) {
    struct axi_rtps_dev* dev = file->private_data;
    unsigned long size = vma->vm_end - vma->vm_start;
    unsigned long rx_pgoff = AXI_DMA_MMAP_RX_OFFSET >> PAGE_SHIFT;

    if (size > XAXIDMA_DEFAULT_BUF_SIZE) {
        dev_err(&dev->pdev->dev, "mmap: requested size %lu exceeds buffer size %u\n",
                size, XAXIDMA_DEFAULT_BUF_SIZE);
        return -EINVAL;
    }

    if (vma->vm_pgoff == (AXI_DMA_MMAP_TX_OFFSET >> PAGE_SHIFT)) {
        dev->tx_user_addr = vma->vm_start;
        return dma_mmap_coherent(&dev->pdev->dev, vma, dev->tx_buf,
                                 dev->tx_buf_phys, XAXIDMA_DEFAULT_BUF_SIZE);
    } else if (vma->vm_pgoff == rx_pgoff) {
        dev->rx_user_addr = vma->vm_start;
        vma->vm_pgoff = 0;
        return dma_mmap_coherent(&dev->pdev->dev, vma, dev->rx_buf,
                                 dev->rx_buf_phys, XAXIDMA_DEFAULT_BUF_SIZE);
    }

    dev_err(&dev->pdev->dev, "mmap: invalid offset 0x%lx\n",
            vma->vm_pgoff << PAGE_SHIFT);
    return -EINVAL;
}

static const struct file_operations axi_rtps_fops = {
    .owner          = THIS_MODULE,
    .open           = axi_rtps_open,
    .release        = axi_rtps_release,
    .read           = axi_rtps_read,
    .write          = axi_rtps_write,
    .mmap           = axi_rtps_mmap,
    .unlocked_ioctl = axi_rtps_unlocked_ioctl,
};

/* Sysfs attribute: /sys/class/axi_dma/axi_dma_dev/option_b_mode */
static ssize_t option_b_mode_show(struct device* dev, struct device_attribute* attr, char* buf) {
    struct axi_rtps_dev* dmadev = dev_get_drvdata(dev->parent);
    if (!dmadev)
        return -ENODEV;
    return scnprintf(buf, PAGE_SIZE, "%u\n", dmadev->plugin_rx_rtps_mode);
}

static ssize_t option_b_mode_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count) {
    struct axi_rtps_dev* dmadev = dev_get_drvdata(dev->parent);
    unsigned int val;
    u32 ctrl_word;
    if (!dmadev || !dmadev->ip_regs)
        return -ENODEV;
    if (kstrtouint(buf, 0, &val))
        return -EINVAL;
    dmadev->plugin_rx_rtps_mode = val ? 1 : 0;
    ctrl_word = ioread32(dmadev->ip_regs + RTPS_IP_CTRL_OFFSET);
    if (dmadev->plugin_rx_rtps_mode)
        ctrl_word |= RTPS_CTRL_PLUGIN_RX_RTPS_MODE;
    else
        ctrl_word &= ~RTPS_CTRL_PLUGIN_RX_RTPS_MODE;
    iowrite32(ctrl_word, dmadev->ip_regs + RTPS_IP_CTRL_OFFSET);
    return count;
}
static DEVICE_ATTR_RW(option_b_mode);

static int axi_rtps_probe(struct platform_device* pdev) {
    struct device* dev = &pdev->dev;
    struct axi_rtps_dev* dmadev = &rtps_device;
    struct resource* res0;
    struct resource* res1;
    int ret;

    dmadev->pdev = pdev;

    mutex_init(&dmadev->tx_lock);
    mutex_init(&dmadev->rx_lock);
    init_waitqueue_head(&dmadev->tx_waitq);
    init_waitqueue_head(&dmadev->rx_waitq);
    init_waitqueue_head(&dmadev->ip_waitq);
    init_waitqueue_head(&dmadev->tx_ip_waitq);
    init_waitqueue_head(&dmadev->rx_packet_waitq);

    res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res0) {
        dev_err(dev, "failed to get DMA memory resource\n");
        return -ENODEV;
    }

    dmadev->regs = devm_ioremap_resource(dev, res0);
    if (IS_ERR(dmadev->regs)) {
        dev_err(dev, "failed to ioremap DMA registers\n");
        return PTR_ERR(dmadev->regs);
    }

    res1 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    if (res1) {
        dmadev->ip_regs = devm_ioremap_resource(dev, res1);
        if (IS_ERR(dmadev->ip_regs)) {
            dev_err(dev, "failed to ioremap RTPS Fast-Path IP registers\n");
            return PTR_ERR(dmadev->ip_regs);
        }
        dmadev->ip_regs_allocated = false;
    } else {
        dmadev->ip_regs =
            ioremap(RTPS_IP_DEFAULT_BASEADDR, RTPS_IP_DEFAULT_REG_SIZE);
        if (!dmadev->ip_regs) {
            dev_err(dev, "failed to ioremap static RTPS IP window\n");
            return -ENOMEM;
        }
        dmadev->ip_regs_allocated = true;
    }

    dmadev->rx_virq = platform_get_irq_byname(pdev, "rx");
    if (dmadev->rx_virq < 0) {
        dmadev->rx_virq = platform_get_irq(pdev, 0);
        if (dmadev->rx_virq < 0) {
            dmadev->rx_virq = map_gic_spi(XAXIDMA_DEFAULT_RX_IRQ);
            if (dmadev->rx_virq < 0) {
                ret = dmadev->rx_virq;
                goto err_unmap_ip;
            }
        }
    }

    dmadev->tx_virq = platform_get_irq_byname(pdev, "tx");
    if (dmadev->tx_virq < 0) {
        dmadev->tx_virq = platform_get_irq(pdev, 1);
        if (dmadev->tx_virq < 0) {
            dmadev->tx_virq = map_gic_spi(XAXIDMA_DEFAULT_TX_IRQ);
        }
    }

    dmadev->ip_virq = platform_get_irq_byname(pdev, "ip_done");
    if (dmadev->ip_virq < 0) {
        dmadev->ip_virq = platform_get_irq(pdev, 2);
        if (dmadev->ip_virq < 0) {
            dmadev->ip_virq = map_gic_spi(RTPS_IP_DEFAULT_IRQ);
            if (dmadev->ip_virq < 0) {
                ret = dmadev->ip_virq;
                goto err_unmap_ip;
            }
        }
    }

    ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
    if (ret) {
        pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
        pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
    }

    dmadev->tx_bd = dma_alloc_coherent(dev, sizeof(struct axi_dma_bd),
                                       &dmadev->tx_bd_phys, GFP_KERNEL);
    dmadev->rx_bd = dma_alloc_coherent(dev, sizeof(struct axi_dma_bd),
                                       &dmadev->rx_bd_phys, GFP_KERNEL);
    dmadev->tx_buf = dma_alloc_coherent(dev, XAXIDMA_DEFAULT_BUF_SIZE,
                                        &dmadev->tx_buf_phys, GFP_KERNEL);
    dmadev->rx_buf = dma_alloc_coherent(dev, XAXIDMA_DEFAULT_BUF_SIZE,
                                        &dmadev->rx_buf_phys, GFP_KERNEL);

    if (!dmadev->tx_bd || !dmadev->rx_bd || !dmadev->tx_buf ||
        !dmadev->rx_buf) {
        ret = -ENOMEM;
        goto err_free_dma_buffers;
    }

    ret = alloc_chrdev_region(&dmadev->dev_num, 0, 1, XILINX_RTPS_DEV_NAME);
    if (ret < 0)
        goto err_free_dma_buffers;

    cdev_init(&dmadev->cdev, &axi_rtps_fops);
    dmadev->cdev.owner = THIS_MODULE;
    ret = cdev_add(&dmadev->cdev, dmadev->dev_num, 1);
    if (ret < 0)
        goto err_unregister_chrdev;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
    dmadev->class = class_create(XILINX_RTPS_CLASS_NAME);
#else
    dmadev->class = class_create(THIS_MODULE, XILINX_RTPS_CLASS_NAME);
#endif
    if (IS_ERR(dmadev->class)) {
        ret = PTR_ERR(dmadev->class);
        goto err_cdev_del;
    }

    dmadev->device = device_create(dmadev->class, dev, dmadev->dev_num, NULL,
                                   XILINX_RTPS_DEV_NAME);
    if (IS_ERR(dmadev->device)) {
        ret = PTR_ERR(dmadev->device);
        goto err_class_destroy;
    }

    if (device_create_file(dmadev->device, &dev_attr_option_b_mode))
        dev_warn(dev, "failed to create option_b_mode sysfs attribute\n");

    ret = request_irq(dmadev->rx_virq, axi_rtps_isr, IRQF_SHARED, "axi_rtps_rx",
                      dmadev);
    if (ret)
        goto err_device_destroy;

    if (dmadev->tx_virq > 0) {
        ret = request_irq(dmadev->tx_virq, axi_rtps_isr, IRQF_SHARED,
                          "axi_rtps_tx", dmadev);
        if (ret)
            dev_warn(dev, "could not request TX IRQ %d\n", dmadev->tx_virq);
    }

    if (dmadev->ip_virq > 0) {
        ret = request_irq(dmadev->ip_virq, axi_rtps_isr, IRQF_SHARED,
                          "rtps_ip_done", dmadev);
        if (ret)
            goto err_free_rx_irq;
    }

    ret = axi_dma_reset(dmadev);
    if (ret)
        goto err_free_ip_irq;

    platform_set_drvdata(pdev, dmadev);

    dev_info(dev,
             "Hybrid RTPS v2.x Fast-Path & Option B Transport Plugin initialized: DMA at 0x%pa, IP at 0x%p\n",
             &res0->start, dmadev->ip_regs);
    return 0;

err_free_ip_irq:
    if (dmadev->ip_virq > 0)
        free_irq(dmadev->ip_virq, dmadev);
err_free_rx_irq:
    if (dmadev->tx_virq > 0)
        free_irq(dmadev->tx_virq, dmadev);
    free_irq(dmadev->rx_virq, dmadev);
err_device_destroy:
    device_remove_file(dmadev->device, &dev_attr_option_b_mode);
    device_destroy(dmadev->class, dmadev->dev_num);
err_class_destroy:
    class_destroy(dmadev->class);
err_cdev_del:
    cdev_del(&dmadev->cdev);
err_unregister_chrdev:
    unregister_chrdev_region(dmadev->dev_num, 1);
err_free_dma_buffers:
    if (dmadev->tx_bd)
        dma_free_coherent(dev, sizeof(struct axi_dma_bd), dmadev->tx_bd,
                          dmadev->tx_bd_phys);
    if (dmadev->rx_bd)
        dma_free_coherent(dev, sizeof(struct axi_dma_bd), dmadev->rx_bd,
                          dmadev->rx_bd_phys);
    if (dmadev->tx_buf)
        dma_free_coherent(dev, XAXIDMA_DEFAULT_BUF_SIZE, dmadev->tx_buf,
                          dmadev->tx_buf_phys);
    if (dmadev->rx_buf)
        dma_free_coherent(dev, XAXIDMA_DEFAULT_BUF_SIZE, dmadev->rx_buf,
                          dmadev->rx_buf_phys);
err_unmap_ip:
    if (dmadev->ip_regs && dmadev->ip_regs_allocated)
        iounmap(dmadev->ip_regs);
    return ret;
}

static int axi_rtps_remove(struct platform_device* pdev) {
    struct axi_rtps_dev* dmadev = platform_get_drvdata(pdev);
    struct device* dev = &pdev->dev;

    axi_dma_reset(dmadev);

    if (dmadev->ip_virq > 0)
        free_irq(dmadev->ip_virq, dmadev);
    if (dmadev->tx_virq > 0)
        free_irq(dmadev->tx_virq, dmadev);
    if (dmadev->rx_virq > 0)
        free_irq(dmadev->rx_virq, dmadev);

    device_remove_file(dmadev->device, &dev_attr_option_b_mode);
    device_destroy(dmadev->class, dmadev->dev_num);
    class_destroy(dmadev->class);
    cdev_del(&dmadev->cdev);
    unregister_chrdev_region(dmadev->dev_num, 1);

    if (dmadev->tx_bd)
        dma_free_coherent(dev, sizeof(struct axi_dma_bd), dmadev->tx_bd,
                          dmadev->tx_bd_phys);
    if (dmadev->rx_bd)
        dma_free_coherent(dev, sizeof(struct axi_dma_bd), dmadev->rx_bd,
                          dmadev->rx_bd_phys);
    if (dmadev->tx_buf)
        dma_free_coherent(dev, XAXIDMA_DEFAULT_BUF_SIZE, dmadev->tx_buf,
                          dmadev->tx_buf_phys);
    if (dmadev->rx_buf)
        dma_free_coherent(dev, XAXIDMA_DEFAULT_BUF_SIZE, dmadev->rx_buf,
                          dmadev->rx_buf_phys);

    if (dmadev->ip_regs && dmadev->ip_regs_allocated)
        iounmap(dmadev->ip_regs);

    return 0;
}

static const struct of_device_id axi_rtps_of_match[] = {
    {.compatible = "xlnx,axi-dma-custom"},
    {},
};
MODULE_DEVICE_TABLE(of, axi_rtps_of_match);

static struct platform_driver axi_rtps_driver = {
    .driver =
        {
            .name = "axi_dma_custom",
            .of_match_table = axi_rtps_of_match,
        },
    .probe = axi_rtps_probe,
    .remove = axi_rtps_remove,
};

module_platform_driver(axi_rtps_driver);

