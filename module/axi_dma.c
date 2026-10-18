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
    "Store-and-Forward BRAM Accelerator & AXI DMA Driver for PYNQ-Z2");
MODULE_VERSION("3.0");

struct axi_dma_dev {
    dev_t dev_num;
    struct cdev cdev;
    struct class* class;
    struct device* device;
    struct platform_device* pdev;

    void __iomem* regs;     /* AXI DMA registers */
    void __iomem* ip_regs;  /* Custom BRAM IP registers & window */
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

    /* Synchronization */
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

/* Calculate 32-bit CRC matching bram_dma_ip.v polynomial reduction */
static inline u32 next_crc32(u32 current_crc, u32 data_in) {
    u32 x = current_crc ^ data_in;
    return (x << 1) ^ (x & 0x80000000 ? 0x04C11DB7 : 0);
}

static u32 calc_crc32(const u32* data, size_t num_words) {
    u32 crc = 0xFFFFFFFF;
    size_t i;
    for (i = 0; i < num_words; i++) {
        crc = next_crc32(crc, data[i]);
    }
    return crc;
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

/* Unified Interrupt Service Routine for S2MM, MM2S, and Custom BRAM IP */
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

    /* Handle Custom BRAM IP interrupt (IRQ_F2P[2]) */
    if (dev->ip_regs) {
        u32 ip_status = ioread32(dev->ip_regs + BRAM_IP_STATUS_OFFSET);

        if (ip_status & (BRAM_IP_STATUS_IRQ_MASK | BRAM_IP_STATUS_CRC_ERR)) {
            /* Acknowledge and clear IP Level-High interrupt */
            iowrite32(1, dev->ip_regs + BRAM_IP_STATUS_OFFSET);

            if (ip_status & BRAM_IP_STATUS_CRC_ERR) {
                u32 calc_crc = ioread32(dev->ip_regs + BRAM_IP_TRIGGER_OFFSET);
                dev->crc_err = 1;
                pr_err(
                    "axi_dma: IP CRC mismatch! Calc CRC: 0x%08x, Status: "
                    "0x%08x\n",
                    calc_crc, ip_status);
                /* Reset IP state by writing 0 to trigger offset */
                iowrite32(0, dev->ip_regs + BRAM_IP_TRIGGER_OFFSET);
            } else {
                dev->crc_err = 0;
            }

            dev->ip_done = 1;
            wake_up_interruptible(&dev->ip_waitq);
            handled = IRQ_HANDLED;
        }
    }

    return handled;
}

static int axi_dma_open(struct inode* inode, struct file* file) {
    file->private_data = &dma_device;
    return 0;
}

static int axi_dma_release(struct inode* inode, struct file* file) {
    return 0;
}

/* S2MM transfer (Receive processed results from BRAM over AXI-Stream) */
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

    /* Clear pending interrupts before starting */
    dma_write(dev, XAXIDMA_S2MM_DMASR_OFFSET, XAXIDMA_SR_IRQ_ALL_MASK);

    /* 1. Program CURDESC first (hardware requirement before RS=1) */
    dma_write(dev, XAXIDMA_S2MM_CURDESC_OFFSET, (u32)dev->rx_bd_phys);

    /* 2. Start channel and enable interrupts */
    dma_write(dev, XAXIDMA_S2MM_DMACR_OFFSET,
              XAXIDMA_CR_RUNSTOP_MASK | XAXIDMA_CR_IRQ_IOC_MASK |
                  XAXIDMA_CR_IRQ_ERR_MASK);

    /* 3. Write TAILDESC to prime receiver (asserts m_axis_tready to IP) */
    dma_write(dev, XAXIDMA_S2MM_TAILDESC_OFFSET, (u32)dev->rx_bd_phys);

    /* 4. Kick egress streaming from Custom IP (transitions S_WAIT_EGRESS ->
     * S_TX_FETCH) */
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

    if (copy_to_user(buf, dev->rx_buf, rx_bytes)) {
        mutex_unlock(&dev->rx_lock);
        return -EFAULT;
    }

    mutex_unlock(&dev->rx_lock);
    return rx_bytes;
}

/* Ingress transfer: Copy to BRAM -> On-the-fly CRC -> Doorbell Trigger -> Wait
 * for compute */
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

    if (copy_from_user(dev->tx_buf, buf, transfer_len)) {
        mutex_unlock(&dev->tx_lock);
        return -EFAULT;
    }

    if (!dev->ip_regs) {
        pr_err("axi_dma: BRAM IP registers not mapped\n");
        mutex_unlock(&dev->tx_lock);
        return -ENODEV;
    }

    /* 1. Calculate 32-bit CRC across payload words */
    crc = calc_crc32((const u32*)dev->tx_buf, num_words);

    dev->ip_done = 0;
    dev->crc_err = 0;
    dev->tx_done = 0;
    dev->tx_err = 0;

    /* 2. Program expected CRC into BRAM IP Trigger register (0x0FFC) */
    iowrite32(crc, dev->ip_regs + BRAM_IP_TRIGGER_OFFSET);

    /* 3. Setup MM2S Buffer Descriptor */
    memset(dev->tx_bd, 0, sizeof(*dev->tx_bd));
    dev->tx_bd->next_desc = (u32)dev->tx_bd_phys;
    dev->tx_bd->buf_addr = (u32)dev->tx_buf_phys;
    dev->tx_bd->control = (transfer_len & XAXIDMA_BD_CTRL_LENGTH_MASK) |
                          XAXIDMA_BD_CTRL_TXSOF_MASK |
                          XAXIDMA_BD_CTRL_TXEOF_MASK;
    dev->tx_bd->status = 0;
    wmb();

    /* 4. Clear pending TX interrupts before starting */
    dma_write(dev, XAXIDMA_MM2S_DMASR_OFFSET, XAXIDMA_SR_IRQ_ALL_MASK);

    /* 5. Program CURDESC first (hardware requirement before RS=1) */
    dma_write(dev, XAXIDMA_MM2S_CURDESC_OFFSET, (u32)dev->tx_bd_phys);

    /* 6. Start channel and enable interrupts */
    dma_write(dev, XAXIDMA_MM2S_DMACR_OFFSET,
              XAXIDMA_CR_RUNSTOP_MASK | XAXIDMA_CR_IRQ_IOC_MASK |
                  XAXIDMA_CR_IRQ_ERR_MASK);

    /* 7. Write TAILDESC to kick MM2S DMA transfer */
    dma_write(dev, XAXIDMA_MM2S_TAILDESC_OFFSET, (u32)dev->tx_bd_phys);

    /* 8. Wait for MM2S DMA transfer completion */
    ret = wait_event_interruptible_timeout(dev->tx_waitq, dev->tx_done != 0,
                                           msecs_to_jiffies(3000));
    if (ret == 0) {
        pr_err(
            "axi_dma: MM2S TX DMA timed out (DMASR: 0x%08x, BD STS: 0x%08x)\n",
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

    /* 9. Wait for Custom IP to complete compute */
    ret = wait_event_interruptible_timeout(dev->ip_waitq, dev->ip_done != 0,
                                           msecs_to_jiffies(3000));
    if (ret == 0) {
        u32 status = ioread32(dev->ip_regs + BRAM_IP_STATUS_OFFSET);
        pr_err("axi_dma: IP compute wait timed out (status: 0x%08x)\n", status);
        /* Attempt to reset IP to IDLE */
        iowrite32(0, dev->ip_regs + BRAM_IP_TRIGGER_OFFSET);
        mutex_unlock(&dev->tx_lock);
        return -ETIMEDOUT;
    } else if (ret < 0) {
        mutex_unlock(&dev->tx_lock);
        return ret;
    }

    if (dev->crc_err) {
        pr_err("axi_dma: IP reported CRC error on ingress buffer\n");
        mutex_unlock(&dev->tx_lock);
        return -EBADMSG;
    }

    mutex_unlock(&dev->tx_lock);
    return transfer_len;
}

static const struct file_operations axi_dma_fops = {
    .owner = THIS_MODULE,
    .open = axi_dma_open,
    .release = axi_dma_release,
    .read = axi_dma_read,
    .write = axi_dma_write,
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

    /* 1. Map AXI DMA registers from Resource 0 */
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

    /* 2. Map Custom BRAM IP registers from Resource 1 (or fallback) */
    res1 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    if (res1) {
        dmadev->ip_regs = devm_ioremap_resource(dev, res1);
        if (IS_ERR(dmadev->ip_regs)) {
            dev_err(dev, "failed to ioremap BRAM IP registers\n");
            return PTR_ERR(dmadev->ip_regs);
        }
        dmadev->ip_regs_allocated = false;
    } else {
        dev_info(dev,
                 "BRAM IP not in DT resource, using static fallback 0x%08x\n",
                 BRAM_IP_DEFAULT_BASEADDR);
        dmadev->ip_regs =
            ioremap(BRAM_IP_DEFAULT_BASEADDR, BRAM_IP_DEFAULT_REG_SIZE);
        if (!dmadev->ip_regs) {
            dev_err(dev, "failed to ioremap static BRAM IP window\n");
            return -ENOMEM;
        }
        dmadev->ip_regs_allocated = true;
    }

    /* 3. Retrieve IRQs from Device Tree or Dynamic Mapping */
    dmadev->rx_virq = platform_get_irq_byname(pdev, "rx");
    if (dmadev->rx_virq < 0) {
        dmadev->rx_virq = platform_get_irq(pdev, 0);
        if (dmadev->rx_virq < 0) {
            dmadev->rx_virq = map_gic_spi(XAXIDMA_DEFAULT_RX_IRQ);
            if (dmadev->rx_virq < 0) {
                dev_err(dev, "failed to map RX IRQ\n");
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
            dev_info(dev, "Mapping dynamic GIC SPI %d for IP interrupt\n",
                     BRAM_IP_DEFAULT_IRQ);
            dmadev->ip_virq = map_gic_spi(BRAM_IP_DEFAULT_IRQ);
            if (dmadev->ip_virq < 0) {
                dev_err(dev, "failed to map IP IRQ\n");
                ret = dmadev->ip_virq;
                goto err_unmap_ip;
            }
        }
    }

    /* 4. Setup DMA coherent mask */
    ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
    if (ret) {
        pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
        pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
    }

    /* 5. Allocate coherent BDs and data buffers */
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
        dev_err(dev, "coherent buffer allocation failed\n");
        ret = -ENOMEM;
        goto err_free_dma_buffers;
    }

    /* 6. Create character device */
    ret = alloc_chrdev_region(&dmadev->dev_num, 0, 1, XILINX_DMA_DEV_NAME);
    if (ret < 0) {
        dev_err(dev, "failed to allocate chrdev region\n");
        goto err_free_dma_buffers;
    }

    cdev_init(&dmadev->cdev, &axi_dma_fops);
    dmadev->cdev.owner = THIS_MODULE;
    ret = cdev_add(&dmadev->cdev, dmadev->dev_num, 1);
    if (ret < 0) {
        dev_err(dev, "failed to add cdev\n");
        goto err_unregister_chrdev;
    }

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

    /* 7. Request Shared IRQs */
    ret = request_irq(dmadev->rx_virq, axi_dma_isr, IRQF_SHARED, "axi_dma_rx",
                      dmadev);
    if (ret) {
        dev_err(dev, "failed to request RX IRQ %d\n", dmadev->rx_virq);
        goto err_device_destroy;
    }

    if (dmadev->tx_virq > 0) {
        ret = request_irq(dmadev->tx_virq, axi_dma_isr, IRQF_SHARED,
                          "axi_dma_tx", dmadev);
        if (ret)
            dev_warn(dev, "could not request TX IRQ %d\n", dmadev->tx_virq);
    }

    if (dmadev->ip_virq > 0) {
        ret = request_irq(dmadev->ip_virq, axi_dma_isr, IRQF_SHARED,
                          "bram_ip_done", dmadev);
        if (ret) {
            dev_err(dev, "failed to request IP IRQ %d\n", dmadev->ip_virq);
            goto err_free_rx_irq;
        }
    }

    /* 8. Soft reset DMA */
    ret = axi_dma_reset(dmadev);
    if (ret) {
        dev_err(dev, "failed to reset DMA\n");
        goto err_free_ip_irq;
    }

    platform_set_drvdata(pdev, dmadev);

    dev_info(dev,
             "BRAM DMA Accelerator initialized: DMA at 0x%pa, IP at 0x%p (RX "
             "IRQ %d, IP IRQ %d)\n",
             &res0->start, dmadev->ip_regs, dmadev->rx_virq, dmadev->ip_virq);
    return 0;

err_free_ip_irq:
    if (dmadev->ip_virq > 0)
        free_irq(dmadev->ip_virq, dmadev);
err_free_rx_irq:
    free_irq(dmadev->rx_virq, dmadev);
    if (dmadev->tx_virq > 0)
        free_irq(dmadev->tx_virq, dmadev);
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
    if (dmadev->ip_regs_allocated && dmadev->ip_regs)
        iounmap(dmadev->ip_regs);
    return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 11, 0)
static void axi_dma_remove(struct platform_device* pdev)
#else
static int axi_dma_remove(struct platform_device* pdev)
#endif
{
    struct device* dev = &pdev->dev;
    struct axi_dma_dev* dmadev = platform_get_drvdata(pdev);

    axi_dma_reset(dmadev);

    if (dmadev->ip_virq > 0)
        free_irq(dmadev->ip_virq, dmadev);
    free_irq(dmadev->rx_virq, dmadev);
    if (dmadev->tx_virq > 0)
        free_irq(dmadev->tx_virq, dmadev);

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

    if (dmadev->ip_regs_allocated && dmadev->ip_regs)
        iounmap(dmadev->ip_regs);

    dev_info(dev, "BRAM DMA Accelerator device removed\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
    return 0;
#endif
}

static const struct of_device_id axi_dma_of_match[] = {
    {
        .compatible = "xlnx,axi-dma-custom",
    },
    {/* sentinel */}};
MODULE_DEVICE_TABLE(of, axi_dma_of_match);

static struct platform_driver axi_dma_driver = {
    .probe = axi_dma_probe,
    .remove = axi_dma_remove,
    .driver =
        {
            .name = "axi_dma_custom",
            .of_match_table = axi_dma_of_match,
        },
};

module_platform_driver(axi_dma_driver);