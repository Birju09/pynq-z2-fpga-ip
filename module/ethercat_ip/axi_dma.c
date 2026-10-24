#include "axi_dma.h"

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
    "Hybrid PS+PL EtherCAT Cyclic Master & Zero-Copy AXI DMA Driver for PYNQ-Z2");
MODULE_VERSION("4.0");

struct axi_dma_dev {
    dev_t dev_num;
    struct cdev cdev;
    struct class* class;
    struct device* device;
    struct platform_device* pdev;

    void __iomem* regs;     /* AXI DMA registers */
    void __iomem* ip_regs;  /* Custom EtherCAT Master IP CSRs & Descriptors */
    bool ip_regs_allocated; /* True if manual ioremap was used */

    /* Coherent buffers */
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

    /* Synchronization & Diagnostics */
    struct mutex tx_lock;
    struct mutex rx_lock;
    wait_queue_head_t tx_waitq;
    wait_queue_head_t rx_waitq;
    wait_queue_head_t ip_waitq;
    int tx_done;
    int rx_done;
    int ip_done;
    int tx_err;
    int rx_err;
    int crc_err;
    int wkc_err;
    u32 last_ip_status;

    /* Mapped Linux Virtual IRQs */
    int tx_virq;
    int rx_virq;
    int ip_virq;
};

static struct axi_dma_dev dma_device;

static inline void dma_write(struct axi_dma_dev* dev, u32 offset, u32 val) {
    iowrite32(val, dev->regs + offset);
}

static inline u32 dma_read(struct axi_dma_dev* dev, u32 offset) {
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

static int axi_dma_reset(struct axi_dma_dev* dev) {
    int timeout = 1000;

    dma_write(dev, XAXIDMA_MM2S_DMACR_OFFSET, XAXIDMA_CR_RESET_MASK);
    dma_write(dev, XAXIDMA_S2MM_DMACR_OFFSET, XAXIDMA_CR_RESET_MASK);

    while (timeout--) {
        u32 mm2s_cr = dma_read(dev, XAXIDMA_MM2S_DMACR_OFFSET);
        u32 s2mm_cr = dma_read(dev, XAXIDMA_S2MM_DMACR_OFFSET);
        if (!(mm2s_cr & XAXIDMA_CR_RESET_MASK) &&
            !(s2mm_cr & XAXIDMA_CR_RESET_MASK))
            return 0;
        udelay(10);
    }

    pr_err("axi_dma: reset timeout\n");
    return -ETIMEDOUT;
}

/* Unified Interrupt Service Routine for S2MM, MM2S, and EtherCAT Master IP */
static irqreturn_t axi_dma_isr(int irq, void* dev_id) {
    struct axi_dma_dev* dev = (struct axi_dma_dev*)dev_id;
    u32 tx_status = dev->regs ? dma_read(dev, XAXIDMA_MM2S_DMASR_OFFSET) : 0;
    u32 rx_status = dev->regs ? dma_read(dev, XAXIDMA_S2MM_DMASR_OFFSET) : 0;
    irqreturn_t handled = IRQ_NONE;

    /* Handle TX (MM2S) interrupt if active */
    if (tx_status & XAXIDMA_SR_IRQ_ALL_MASK) {
        dma_write(dev, XAXIDMA_MM2S_DMASR_OFFSET,
                  tx_status & XAXIDMA_SR_IRQ_ALL_MASK);
        if (tx_status & XAXIDMA_SR_ERR_ALL_MASK) {
            dev->tx_err = 1;
            pr_err("axi_dma: MM2S error status: 0x%08x\n", tx_status);
        }
        dev->tx_done = 1;
        wake_up_interruptible(&dev->tx_waitq);
        handled = IRQ_HANDLED;
    }

    /* Handle RX (S2MM) interrupt */
    if (rx_status & XAXIDMA_SR_IRQ_ALL_MASK) {
        dma_write(dev, XAXIDMA_S2MM_DMASR_OFFSET,
                  rx_status & XAXIDMA_SR_IRQ_ALL_MASK);
        if (rx_status & XAXIDMA_SR_ERR_ALL_MASK) {
            dev->rx_err = 1;
            pr_err("axi_dma: S2MM error status: 0x%08x\n", rx_status);
        }
        dev->rx_done = 1;
        wake_up_interruptible(&dev->rx_waitq);
        handled = IRQ_HANDLED;
    }

    /* Handle Custom EtherCAT Master IP interrupt (IRQ_F2P[2]) */
    if (dev->ip_regs) {
        u32 ip_status = ioread32(dev->ip_regs + BRAM_IP_STATUS_OFFSET);

        if (ip_status & (BRAM_IP_STATUS_IRQ_MASK | BRAM_IP_STATUS_CRC_ERR)) {
            dev->last_ip_status = ip_status;

            /* Acknowledge and clear IP Level-High interrupt immediately */
            iowrite32(1, dev->ip_regs + BRAM_IP_STATUS_OFFSET);

            if (ip_status & BRAM_IP_STATUS_CRC_ERR) {
                u32 calc_crc = ioread32(dev->ip_regs + BRAM_IP_CRC_OFFSET);
                dev->crc_err = 1;
                pr_err("axi_dma: IP CRC/FCS mismatch! Calc CRC: 0x%08x, Status: 0x%08x\n",
                       calc_crc, ip_status);
                iowrite32(0, dev->ip_regs + BRAM_IP_TRIGGER_OFFSET);
            } else {
                dev->crc_err = 0;
            }

            if (ip_status & ECAT_IP_STATUS_WKC_ERR) {
                dev->wkc_err = 1;
            } else {
                dev->wkc_err = 0;
            }

            dev->ip_done = 1;
            wake_up_interruptible(&dev->ip_waitq);
            handled = IRQ_HANDLED;
        }
    }

    return handled;
}

static long axi_dma_unlocked_ioctl(struct file* file, unsigned int cmd,
                                   unsigned long arg) {
    struct axi_dma_dev* dev = file->private_data;
    u32 val;
    u64 mask64;
    struct ecat_master_cfg cfg;
    struct ecat_cycle_diag diag;
    int i;

    if (!dev->ip_regs)
        return -ENODEV;

    switch (cmd) {
        case AXI_DMA_IOC_SET_TIMER:
            if (copy_from_user(&val, (u32 __user*)arg, sizeof(u32)))
                return -EFAULT;
            iowrite32(val, dev->ip_regs + BRAM_IP_TIMER_OFFSET);
            return 0;

        case AXI_DMA_IOC_GET_TIMER:
            val = ioread32(dev->ip_regs + BRAM_IP_TIMER_OFFSET);
            if (copy_to_user((u32 __user*)arg, &val, sizeof(u32)))
                return -EFAULT;
            return 0;

        case AXI_DMA_IOC_SET_PAYLOAD_LEN:
            if (copy_from_user(&val, (u32 __user*)arg, sizeof(u32)))
                return -EFAULT;
            if (val > BRAM_IP_MAX_WORDS)
                return -EINVAL;
            iowrite32(val, dev->ip_regs + BRAM_IP_PAYLOAD_LEN_OFFSET);
            return 0;

        case AXI_DMA_IOC_GET_PAYLOAD_LEN:
            val = ioread32(dev->ip_regs + BRAM_IP_PAYLOAD_LEN_OFFSET);
            if (copy_to_user((u32 __user*)arg, &val, sizeof(u32)))
                return -EFAULT;
            return 0;

        case AXI_DMA_IOC_SET_CRC_BYPASS:
            if (copy_from_user(&val, (u32 __user*)arg, sizeof(u32)))
                return -EFAULT;
            dev->bypass_crc = val ? 1 : 0;
            return 0;

        case AXI_DMA_IOC_GET_CRC_BYPASS:
            val = dev->bypass_crc;
            if (copy_to_user((u32 __user*)arg, &val, sizeof(u32)))
                return -EFAULT;
            return 0;

        case AXI_DMA_IOC_CONFIG_ECAT:
            if (copy_from_user(&cfg, (void __user*)arg, sizeof(cfg)))
                return -EFAULT;
            if (cfg.num_descriptors == 0 || cfg.num_descriptors > ECAT_MAX_DESCRIPTORS)
                return -EINVAL;
            if (cfg.total_pdo_words == 0 || cfg.total_pdo_words > BRAM_IP_MAX_WORDS)
                return -EINVAL;

            /* Program global Master CSRs */
            iowrite32(0x10 | (cfg.num_descriptors & 0x0F),
                      dev->ip_regs + BRAM_IP_CTRL_OFFSET);
            iowrite32(cfg.wire_timer_ticks, dev->ip_regs + BRAM_IP_TIMER_OFFSET);
            iowrite32(cfg.total_pdo_words, dev->ip_regs + BRAM_IP_PAYLOAD_LEN_OFFSET);
            iowrite32(cfg.slave_mask_lo, dev->ip_regs + ECAT_IP_SLAVE_MASK_LO_OFFSET);
            iowrite32(cfg.slave_mask_hi, dev->ip_regs + ECAT_IP_SLAVE_MASK_HI_OFFSET);

            /* Program Datagram Descriptor Table [0 .. num_descriptors-1] */
            for (i = 0; i < (int)cfg.num_descriptors; i++) {
                u32 ctrl_word = ((u32)cfg.descs[i].datagram_idx << 24) |
                                ((u32)cfg.descs[i].ecat_cmd << 16) |
                                ((u32)cfg.descs[i].payload_words & 0x0FFF);
                u32 pdi_wkc_word = ((u32)cfg.descs[i].expected_wkc << 16) |
                                   ((u32)cfg.descs[i].pdi_word_offset & 0x0FFF);

                iowrite32(cfg.descs[i].logical_addr,
                          dev->ip_regs + ECAT_DESC_LOGICAL_ADDR(i));
                iowrite32(ctrl_word,
                          dev->ip_regs + ECAT_DESC_CTRL(i));
                iowrite32(pdi_wkc_word,
                          dev->ip_regs + ECAT_DESC_PDI_WKC(i));
            }
            return 0;

        case AXI_DMA_IOC_GET_ECAT_DIAG:
            memset(&diag, 0, sizeof(diag));
            diag.status_reg = dev->last_ip_status ? dev->last_ip_status :
                              ioread32(dev->ip_regs + BRAM_IP_STATUS_OFFSET);
            diag.last_cycle_ticks = ioread32(dev->ip_regs + ECAT_IP_CYCLE_TICKS_OFFSET);
            diag.total_actual_wkc = (u16)(diag.status_reg >> ECAT_IP_STATUS_WKC_SHIFT);
            diag.completed_frames = (u8)((diag.status_reg & ECAT_IP_STATUS_FRAMES_MASK) >>
                                         ECAT_IP_STATUS_FRAMES_SHIFT);
            diag.wkc_err = (diag.status_reg & ECAT_IP_STATUS_WKC_ERR) ? 1 : 0;
            diag.crc_err = (diag.status_reg & BRAM_IP_STATUS_CRC_ERR) ? 1 : 0;
            for (i = 0; i < ECAT_MAX_DESCRIPTORS; i++) {
                diag.desc_status[i] = ioread32(dev->ip_regs + ECAT_DESC_STATUS(i));
            }
            if (copy_to_user((void __user*)arg, &diag, sizeof(diag)))
                return -EFAULT;
            return 0;

        case AXI_DMA_IOC_SET_SLAVE_MASK:
            if (copy_from_user(&mask64, (u64 __user*)arg, sizeof(u64)))
                return -EFAULT;
            iowrite32((u32)(mask64 & 0xFFFFFFFFULL),
                      dev->ip_regs + ECAT_IP_SLAVE_MASK_LO_OFFSET);
            iowrite32((u32)((mask64 >> 32) & 0xFFULL),
                      dev->ip_regs + ECAT_IP_SLAVE_MASK_HI_OFFSET);
            return 0;

        default:
            return -ENOTTY;
    }
}

static int axi_dma_open(struct inode* inode, struct file* file) {
    file->private_data = &dma_device;
    return 0;
}

static int axi_dma_release(struct inode* inode, struct file* file) {
    struct axi_dma_dev* dev = file->private_data;
    if (dev) {
        dev->tx_user_addr = 0;
        dev->rx_user_addr = 0;
    }
    return 0;
}

/* S2MM transfer (Receive updated RxPDO Process Data Image from BRAM over AXI-Stream) */
static ssize_t axi_dma_read(struct file* file, char __user* buf, size_t count,
                            loff_t* ppos) {
    struct axi_dma_dev* dev = file->private_data;
    size_t transfer_len;
    u32 rx_bytes;
    long ret;

    if (count == 0)
        return 0;

    transfer_len = min_t(size_t, count, BRAM_IP_MAX_BYTES);

    if (mutex_lock_interruptible(&dev->rx_lock))
        return -ERESTARTSYS;

    /* Setup S2MM Buffer Descriptor */
    memset(dev->rx_bd, 0, sizeof(*dev->rx_bd));
    dev->rx_bd->next_desc = (u32)dev->rx_bd_phys;
    dev->rx_bd->buf_addr = (u32)dev->rx_buf_phys;
    dev->rx_bd->control = transfer_len & XAXIDMA_BD_CTRL_LENGTH_MASK;
    dev->rx_bd->status = 0;

    /* Ensure memory writes to descriptor are visible to DMA */
    wmb();

    dev->rx_done = 0;
    dev->rx_err = 0;

    /* 1. Clear pending interrupts before starting */
    dma_write(dev, XAXIDMA_S2MM_DMASR_OFFSET, XAXIDMA_SR_IRQ_ALL_MASK);

    /* 2. Program CURDESC first (hardware requirement before RS=1) */
    dma_write(dev, XAXIDMA_S2MM_CURDESC_OFFSET, (u32)dev->rx_bd_phys);

    /* 3. Start channel and enable interrupts */
    dma_write(dev, XAXIDMA_S2MM_DMACR_OFFSET,
              XAXIDMA_CR_RUNSTOP_MASK | XAXIDMA_CR_IRQ_IOC_MASK |
                  XAXIDMA_CR_IRQ_ERR_MASK);

    /* 4. Flush barrier before writing TAILDESC */
    wmb();
    dma_write(dev, XAXIDMA_S2MM_TAILDESC_OFFSET, (u32)dev->rx_bd_phys);

    /* 5. Kick egress streaming from Custom IP (S_WAIT_EGRESS -> S_TX_RUN) */
    if (dev->ip_regs)
        iowrite32(1, dev->ip_regs + BRAM_IP_STATUS_OFFSET);

    /* Wait for S2MM completion interrupt */
    ret = wait_event_interruptible_timeout(dev->rx_waitq, dev->rx_done != 0,
                                           msecs_to_jiffies(3000));
    if (ret == 0) {
        pr_err("axi_dma: S2MM wait timed out (DMASR: 0x%08x, BD STS: 0x%08x)\n",
               dma_read(dev, XAXIDMA_S2MM_DMASR_OFFSET), dev->rx_bd->status);
        axi_dma_reset(dev);
        mutex_unlock(&dev->rx_lock);
        return -ETIMEDOUT;
    } else if (ret < 0) {
        mutex_unlock(&dev->rx_lock);
        return ret;
    }

    if (dev->rx_err || (dev->rx_bd->status & XAXIDMA_BD_STS_ERROR_MASK)) {
        pr_err("axi_dma: S2MM hardware transfer error\n");
        mutex_unlock(&dev->rx_lock);
        return -EIO;
    }

    rmb();
    rx_bytes = dev->rx_bd->status & XAXIDMA_BD_STS_ACTUAL_LEN_MASK;
    if (rx_bytes == 0)
        rx_bytes = transfer_len;

    /* Zero-Copy check: bypass copy_to_user() if user pointer is inside mmap'd VMA */
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

/* Ingress transfer: Stream TxPDO PDI -> Multi-Frame EtherCAT Exchange -> Wait for IRQ */
static ssize_t axi_dma_write(struct file* file, const char __user* buf,
                             size_t count, loff_t* ppos) {
    struct axi_dma_dev* dev = file->private_data;
    size_t transfer_len;
    size_t num_words;
    u32 crc;
    long ret;

    if (count == 0)
        return 0;

    transfer_len = min_t(size_t, count, BRAM_IP_MAX_BYTES);
    num_words = transfer_len / sizeof(u32);

    if (mutex_lock_interruptible(&dev->tx_lock))
        return -ERESTARTSYS;

    /* Zero-Copy check: bypass copy_from_user() if user pointer is inside mmap'd VMA */
    if (buf != NULL && !(dev->tx_user_addr &&
                         (unsigned long)buf >= dev->tx_user_addr &&
                         (unsigned long)buf < dev->tx_user_addr + XAXIDMA_DEFAULT_BUF_SIZE)) {
        if (copy_from_user(dev->tx_buf, buf, transfer_len)) {
            mutex_unlock(&dev->tx_lock);
            return -EFAULT;
        }
    }

    if (!dev->ip_regs) {
        pr_err("axi_dma: EtherCAT Master IP registers not mapped\n");
        mutex_unlock(&dev->tx_lock);
        return -ENODEV;
    }

    if (!dev->bypass_crc) {
        crc = calc_crc32((const u32*)dev->tx_buf, num_words);
    } else {
        crc = 0;
    }

    dev->ip_done = 0;
    dev->crc_err = 0;
    dev->wkc_err = 0;
    dev->tx_done = 0;
    dev->tx_err = 0;

    /* Program expected CRC into IP (does NOT leave S_IDLE) */
    iowrite32(crc, dev->ip_regs + BRAM_IP_CRC_OFFSET);

    /* Setup MM2S Buffer Descriptor */
    memset(dev->tx_bd, 0, sizeof(*dev->tx_bd));
    dev->tx_bd->next_desc = (u32)dev->tx_bd_phys;
    dev->tx_bd->buf_addr = (u32)dev->tx_buf_phys;
    dev->tx_bd->control = (transfer_len & XAXIDMA_BD_CTRL_LENGTH_MASK) |
                          XAXIDMA_BD_CTRL_TXSOF_MASK |
                          XAXIDMA_BD_CTRL_TXEOF_MASK;
    dev->tx_bd->status = 0;
    wmb();

    /* 1. Clear pending TX interrupts before starting */
    dma_write(dev, XAXIDMA_MM2S_DMASR_OFFSET, XAXIDMA_SR_IRQ_ALL_MASK);

    /* 2. Program CURDESC before DMACR.RS=1 */
    dma_write(dev, XAXIDMA_MM2S_CURDESC_OFFSET, (u32)dev->tx_bd_phys);

    /* 3. Start channel and enable interrupts */
    dma_write(dev, XAXIDMA_MM2S_DMACR_OFFSET,
              XAXIDMA_CR_RUNSTOP_MASK | XAXIDMA_CR_IRQ_IOC_MASK |
                  XAXIDMA_CR_IRQ_ERR_MASK);

    /* 4. Flush barrier before writing TAILDESC */
    wmb();
    dma_write(dev, XAXIDMA_MM2S_TAILDESC_OFFSET, (u32)dev->tx_bd_phys);

    /* 5. Wait for MM2S DMA transfer completion */
    ret = wait_event_interruptible_timeout(dev->tx_waitq, dev->tx_done != 0,
                                           msecs_to_jiffies(3000));
    if (ret == 0) {
        pr_err("axi_dma: MM2S TX DMA timed out (DMASR: 0x%08x, BD STS: 0x%08x)\n",
               dma_read(dev, XAXIDMA_MM2S_DMASR_OFFSET), dev->tx_bd->status);
        axi_dma_reset(dev);
        mutex_unlock(&dev->tx_lock);
        return -ETIMEDOUT;
    } else if (ret < 0) {
        mutex_unlock(&dev->tx_lock);
        return ret;
    }

    if (dev->tx_err || (dev->tx_bd->status & XAXIDMA_BD_STS_ERROR_MASK)) {
        pr_err("axi_dma: MM2S hardware transfer error\n");
        mutex_unlock(&dev->tx_lock);
        return -EIO;
    }

    /* 6. Wait for EtherCAT Multi-Frame Cyclic Exchange completion */
    ret = wait_event_interruptible_timeout(dev->ip_waitq, dev->ip_done != 0,
                                           msecs_to_jiffies(3000));
    if (ret == 0) {
        u32 status = ioread32(dev->ip_regs + BRAM_IP_STATUS_OFFSET);
        pr_err("axi_dma: EtherCAT cycle wait timed out (status: 0x%08x)\n", status);
        iowrite32(0, dev->ip_regs + BRAM_IP_TRIGGER_OFFSET);
        mutex_unlock(&dev->tx_lock);
        return -ETIMEDOUT;
    } else if (ret < 0) {
        mutex_unlock(&dev->tx_lock);
        return ret;
    }

    if (dev->crc_err) {
        pr_err("axi_dma: EtherCAT Master reported CRC/FCS error\n");
        mutex_unlock(&dev->tx_lock);
        return -EBADMSG;
    }

    mutex_unlock(&dev->tx_lock);
    return transfer_len;
}

/* MMAP Interface: Map TX or RX coherent DMA buffer directly to user space */
static int axi_dma_mmap(struct file* file, struct vm_area_struct* vma) {
    struct axi_dma_dev* dev = file->private_data;
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

static const struct file_operations axi_dma_fops = {
    .owner = THIS_MODULE,
    .open = axi_dma_open,
    .release = axi_dma_release,
    .read = axi_dma_read,
    .write = axi_dma_write,
    .mmap = axi_dma_mmap,
    .unlocked_ioctl = axi_dma_unlocked_ioctl,
};

static int axi_dma_probe(struct platform_device* pdev) {
    struct device* dev = &pdev->dev;
    struct axi_dma_dev* dmadev = &dma_device;
    struct resource* res0;
    struct resource* res1;
    int ret;

    dmadev->pdev = pdev;

    mutex_init(&dmadev->tx_lock);
    mutex_init(&dmadev->rx_lock);
    init_waitqueue_head(&dmadev->tx_waitq);
    init_waitqueue_head(&dmadev->rx_waitq);
    init_waitqueue_head(&dmadev->ip_waitq);

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
            dev_err(dev, "failed to ioremap EtherCAT Master IP registers\n");
            return PTR_ERR(dmadev->ip_regs);
        }
        dmadev->ip_regs_allocated = false;
    } else {
        dmadev->ip_regs =
            ioremap(BRAM_IP_DEFAULT_BASEADDR, BRAM_IP_DEFAULT_REG_SIZE);
        if (!dmadev->ip_regs) {
            dev_err(dev, "failed to ioremap static EtherCAT IP window\n");
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
            dmadev->ip_virq = map_gic_spi(BRAM_IP_DEFAULT_IRQ);
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

    ret = alloc_chrdev_region(&dmadev->dev_num, 0, 1, XILINX_DMA_DEV_NAME);
    if (ret < 0)
        goto err_free_dma_buffers;

    cdev_init(&dmadev->cdev, &axi_dma_fops);
    dmadev->cdev.owner = THIS_MODULE;
    ret = cdev_add(&dmadev->cdev, dmadev->dev_num, 1);
    if (ret < 0)
        goto err_unregister_chrdev;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
    dmadev->class = class_create(XILINX_DMA_CLASS_NAME);
#else
    dmadev->class = class_create(THIS_MODULE, XILINX_DMA_CLASS_NAME);
#endif
    if (IS_ERR(dmadev->class)) {
        ret = PTR_ERR(dmadev->class);
        goto err_cdev_del;
    }

    dmadev->device = device_create(dmadev->class, dev, dmadev->dev_num, NULL,
                                   XILINX_DMA_DEV_NAME);
    if (IS_ERR(dmadev->device)) {
        ret = PTR_ERR(dmadev->device);
        goto err_class_destroy;
    }

    ret = request_irq(dmadev->rx_virq, axi_dma_isr, IRQF_SHARED, "axi_dma_rx",
                      dmadev);
    if (ret)
        goto err_device_destroy;

    if (dmadev->tx_virq > 0) {
        ret = request_irq(dmadev->tx_virq, axi_dma_isr, IRQF_SHARED,
                          "axi_dma_tx", dmadev);
        if (ret)
            dev_warn(dev, "could not request TX IRQ %d\n", dmadev->tx_virq);
    }

    if (dmadev->ip_virq > 0) {
        ret = request_irq(dmadev->ip_virq, axi_dma_isr, IRQF_SHARED,
                          "ecat_ip_done", dmadev);
        if (ret)
            goto err_free_rx_irq;
    }

    ret = axi_dma_reset(dmadev);
    if (ret)
        goto err_free_ip_irq;

    platform_set_drvdata(pdev, dmadev);

    dev_info(dev,
             "Hybrid EtherCAT Master initialized: DMA at 0x%pa, IP at 0x%p\n",
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

static int axi_dma_remove(struct platform_device* pdev) {
    struct axi_dma_dev* dmadev = platform_get_drvdata(pdev);
    struct device* dev = &pdev->dev;

    axi_dma_reset(dmadev);

    if (dmadev->ip_virq > 0)
        free_irq(dmadev->ip_virq, dmadev);
    if (dmadev->tx_virq > 0)
        free_irq(dmadev->tx_virq, dmadev);
    if (dmadev->rx_virq > 0)
        free_irq(dmadev->rx_virq, dmadev);

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

static const struct of_device_id axi_dma_of_match[] = {
    {.compatible = "xlnx,axi-dma-custom"},
    {},
};
MODULE_DEVICE_TABLE(of, axi_dma_of_match);

static struct platform_driver axi_dma_driver = {
    .driver =
        {
            .name = "axi_dma_custom",
            .of_match_table = axi_dma_of_match,
        },
    .probe = axi_dma_probe,
    .remove = axi_dma_remove,
};

module_platform_driver(axi_dma_driver);

