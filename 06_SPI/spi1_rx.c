#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include "main.h"

#define DRIVER_NAME "spi1_device_driver"
#define DEVICE_NAME "spi1"
#define SLK 500000

#define SPI1_BASE 0x481a0000

// Register offsets for OMAP2 McSPI (AM33xx SPI1, base 0x481a0000)
#define MCSPI_SYSCONFIG   0x110
#define MCSPI_SYSSTATUS   0x114

// #define MCSPI_CHCONF1     0x140
// #define MCSPI_CHSTAT1     0x144
// #define MCSPI_CHCTRL1     0x148
// #define MCSPI_TX1         0x14c
// #define MCSPI_RX1         0x150

#define MCSPI_CHCONF0     0x12c
#define MCSPI_CHSTAT0     0x130
#define MCSPI_CHCTRL0     0x134
#define MCSPI_TX0         0x138
#define MCSPI_RX0         0x13c

#define MCSPI_MODULCTRL   0x128
#define MCSPI_IRQSTATUS   0x118
#define MCSPI_IRQENABLE   0x11c

#define MCSPI_DAFRX       0x1a0
#define MCSPI_XFERLEVEL   0x17c

// Bitmasks for CHCONF1
#define MCSPI_CHCONF_POL      BIT(1)  // Clock polarity
#define MCSPI_CHCONF_PHA      BIT(0)  // Clock phase
#define MCSPI_CHCONF_WL_MASK  (0x1f << 7)  // Word length mask
#define MCSPI_CHCONF_TRM_RX_ONLY BIT(12)  // Receive-only mode
#define MCSPI_CHCONF_DPE0     BIT(16) // Data pin 0 enable
#define MCSPI_CHCONF_DPE1     BIT(17) // Data pin 1 enable
#define MCSPI_CHCONF_IS       BIT(18) // Input select (MISO)
#define MCSPI_CHCONF_FORCE    BIT(20) // Force SPI_EN (CS)
#define MCSPI_CHCONF_EPOL     BIT(6)  // Chip select polarity

// Bitmasks for CHSTAT1
#define MCSPI_CHSTAT_EOT      BIT(2)  // End of frame
#define MCSPI_CHSTAT_TXS      BIT(1)  // TX register empty
#define MCSPI_CHSTAT_RXS      BIT(0)  // RX register full

// Bitmasks for CHCTRL1
#define MCSPI_CHCTRL_EN       BIT(0)  // Channel enable

// Bitmasks for MODULCTRL
#define MCSPI_MODULCTRL_MS    BIT(2)  // Master/Slave mode

// Bitmasks for IRQSTATUS and IRQENABLE
#define MCSPI_CHSTAT_RX1_FULL BIT(6)  // RX1 full interrupt
#define MCSPI_CHSTAT_RX0_FULL BIT(2)  // RX0 full interrupt
#define MCSPI_CHSTAT_RX0_OVERFLOW BIT(3)  // RX0 overflow interrupt
#define MCSPI_CHSTAT_EOWKE BIT(17)  

// Receive interval for logging (2 seconds)
#define SPI_RECV_INTERVAL (2 * HZ)
#define MS_DELAY 3000 // increarw the time from 2000 to 3000 to make sure the completion of DMA received data


struct spi_device_data {
    dev_t dev_num;
    struct cdev cdev;
    struct class *class;
    struct device *dev;
    void __iomem *base;  // Mapped base address of SPI1 registers
    struct clk *clk;
    int irq;
    struct task_struct *recv_thread;
    wait_queue_head_t recv_wait;
    u8 rx_buffer[256];  // Buffer for received data
    size_t rx_count;    // Number of bytes received
    bool data_ready;    // Flag for new data

    u8 index;
    u8 jump_num;
    u8 is_get_count;

    /* DMA-related fields */
    struct dma_chan *rx_chan;
    struct completion rx_completion;
    bool use_dma; /* Flag to indicate if DMA is used */

    size_t count;

    struct work_struct re_request_work;
};

void spi_hw_init(struct spi_device_data *data);

static void spi1_rx_dma_callback(void *data)
{
    struct spi_device_data *spi1 = data;
    complete(&spi1->rx_completion);
    // dev_info(spi1->dev, "RX DMA callback called\n");
    dev_info(spi1->dev, "[DMA] DMA read is done -> /dev/spi1\n");
}

void Dma_read(struct spi_device_data* data){
    void *kbuf;
    dma_addr_t dma_dst_addr;
    struct dma_async_tx_descriptor *rx_desc;
    int ret;
    u32 ch0cfg;
#if (PRINT_DEBUG_RX)
    dev_info(data->dev, "Size to allocate: %d\n", data->count);
#endif
    /* Allocate DMA-coherent buffer */
    kbuf = dma_alloc_coherent(data->dev, data->count, &dma_dst_addr, GFP_KERNEL | GFP_DMA);
    if (!kbuf) {
        dev_err(data->dev, "Failed to allocate DMA-coherent buffer\n");
        return;
    }
    memset(kbuf, 0x40, data->count); // @

    reinit_completion(&data->rx_completion);
    rx_desc = dmaengine_prep_slave_single(data->rx_chan, dma_dst_addr, data->count,
                                            DMA_DEV_TO_MEM, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
    if (!rx_desc) {
        dev_err(data->dev, "Failed to prepare RX DMA descriptor\n");
        goto free_buf;
    }

    rx_desc->callback = spi1_rx_dma_callback;
    rx_desc->callback_param = data;

    dev_info(data->dev, "Submitting RX DMA descriptor\n");
    dmaengine_submit(rx_desc);

    dev_info(data->dev, "Issuing RX DMA pending\n");
    dma_async_issue_pending(data->rx_chan);

    /* Do not need to read the first byte manually here due to it's already done by reading data->count */
    //((char*)kbuf)[data->index++] = ioread32(data->base + MCSPI_RX0);

    // due to every 2ms, msg is sent, so the timeout should be 2ms
    ret = wait_for_completion_timeout(&data->rx_completion, msecs_to_jiffies(MS_DELAY));
    if (ret == 0) {
        dev_err(data->dev, "DMA RX timeout\n");
        dmaengine_terminate_sync(data->rx_chan);
        goto free_buf;
    }

    goto free_buf;

free_buf:
    dma_free_coherent(data->dev, data->count, kbuf, dma_dst_addr);

    // dev_info(data->dev, "Buffer received (%d):\n", data->count);
    // for (i = 0; i < data->count; i++){
    //     dev_info(data->dev, "'%c' ", ((char*)kbuf)[i]);
    // }
    // dev_info(data->dev, "===== \n");

    // update value to read /dev/spi1
    data->index = data->count;
    memcpy(data->rx_buffer, ((char*)kbuf), data->count);

    ch0cfg = ioread32(data->base + MCSPI_CHCONF0);
    ch0cfg |= 0u << 15; // Disable DMA request read;
    iowrite32(ch0cfg, data->base + MCSPI_CHCONF0);

    spi_hw_init(data);

    kfree(kbuf);

    iowrite32(MCSPI_CHSTAT_RX0_FULL, data->base + MCSPI_IRQENABLE); // enable IRQ for next time
}

static void re_request_irq_work(struct work_struct *work)
{
    struct spi_device_data *data = container_of(work, struct spi_device_data, re_request_work);
    Dma_read(data);
}

static irqreturn_t irqHandler(int irq, void *d)
{
    struct spi_device_data *data = d;
    u32 irqsts;
#if (DMA_USED_RX)
    u32 ch0cfg;
#endif
#if (FIFO_DRAINED)
    unsigned long timeout;
#endif

    irqsts = ioread32(data->base + MCSPI_IRQSTATUS);
    //dev_info(data->dev, "XXX\n");

    if (irqsts & MCSPI_CHSTAT_RX0_FULL) {

#if (!DMA_USED_RX)
        if (data->is_get_count == 0){
            data->count = ioread32(data->base + MCSPI_RX0);
            data->is_get_count = 1;
#if (PRINT_DEBUG_RX)
            printk("->count = %d\n", data->count);
#endif /* PRINT_DEBUG_RX */
        }
        else {
#if (FIFO_DRAINED)
            timeout = jiffies + msecs_to_jiffies(3000);
            // use drain FIFO to read data
            do {
                if (ioread32(data->base + MCSPI_CHSTAT0) & MCSPI_CHSTAT_RXS) {
                    data->rx_buffer[data->index++] = ioread32(data->base + MCSPI_RX0);
                }    

                if (time_after(jiffies, timeout)) {
                    dev_err(data->dev, "Timeout draining FIFO\n");
                    break;
                }

            } while(data->rx_buffer[data->index-1] != 0x0a); // '\n'

            data->is_get_count = 0;
            spi_hw_init(data);

            dev_info(data->dev, "[CPU] DMA read is done -> /dev/spi1\n");
#else
            if (ioread32(data->base + MCSPI_CHSTAT0) & MCSPI_CHSTAT_RXS) {
                data->rx_buffer[data->index++] = ioread32(data->base + MCSPI_RX0);
            }   
#endif /* FIFO_DRAINED */
            spi_hw_init(data);

        }
#else
        data->count = ioread32(data->base + MCSPI_RX0);

        // disable all interrupt
        iowrite32(0, data->base + MCSPI_IRQENABLE);

        ch0cfg = ioread32(data->base + MCSPI_CHCONF0);
        ch0cfg |= 1u << 15; // DMA request read;
        iowrite32(ch0cfg, data->base + MCSPI_CHCONF0);

        schedule_work(&data->re_request_work);
#endif /* DMA_USED_RX */

        // Clear RX1_FULL interrupt
        iowrite32(MCSPI_CHSTAT_RX0_FULL, data->base + MCSPI_IRQSTATUS);
    }
    else if (irqsts & MCSPI_CHSTAT_RX0_OVERFLOW) {
        // Clear RX1_FULL interrupt
        iowrite32(MCSPI_CHSTAT_RX0_OVERFLOW, data->base + MCSPI_IRQSTATUS);      
    }
    else if ((irqsts & MCSPI_CHSTAT_EOWKE) == MCSPI_CHSTAT_EOWKE) {
        // Clear RX1_FULL interrupt
        iowrite32(MCSPI_CHSTAT_EOWKE, data->base + MCSPI_IRQSTATUS);      
    }

    data->jump_num++;

    return IRQ_HANDLED;
}

static int spi_device_open(struct inode *inode, struct file *file)
{
    struct spi_device_data *data = container_of(inode->i_cdev, struct spi_device_data, cdev);
    file->private_data = data;
    return 0;
}

static ssize_t spi_device_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct spi_device_data *data = filp->private_data;
    int i;

    // used for reading data in non-DMA mode
    for (i = 0; i < data->index; i++){
        dev_info(data->dev, "[%d]-> '%c'\n", i, data->rx_buffer[i]);
    }

    // back up
    data->index = 0;
    data->is_get_count = 0;

    return 0;
}

static const struct file_operations spi_device_fops = {
    .owner = THIS_MODULE,
    .open = spi_device_open,
    .read = spi_device_read,
};

void spi_hw_init(struct spi_device_data *data){
    u32 chconf = 0;

    // Software reset
    iowrite32(BIT(1), data->base + MCSPI_SYSCONFIG);
    while (!(ioread32(data->base + MCSPI_SYSSTATUS) & BIT(0)))
        cpu_relax();

    // Configure SPI controller (slave mode)
    iowrite32(MCSPI_MODULCTRL_MS, data->base + MCSPI_MODULCTRL);

    chconf = ioread32(data->base + MCSPI_CHCONF0);
    // Configure CHCONF1 for SPI1 (CS1)
    chconf |= (8 - 1) << 7; // 8-bit word length
    chconf &= ~MCSPI_CHCONF_POL; // SPI Mode 0
    chconf &= ~MCSPI_CHCONF_PHA; // SPI Mode 0

    chconf &= ~MCSPI_CHCONF_IS; // D0 as input (MISO)
    chconf |= MCSPI_CHCONF_DPE0; // D0 not driven (MISO input)
    chconf &= ~MCSPI_CHCONF_DPE1; // D1 as output (not use here)

    chconf |= MCSPI_CHCONF_EPOL; // Active-low CS
    chconf |= MCSPI_CHCONF_TRM_RX_ONLY; // Receive-only mode

    chconf |= 1u << 28; // The FIFO buffer is used to receive data.

    iowrite32(chconf, data->base + MCSPI_CHCONF0);


    iowrite32(RX_TRIGGER << 8, data->base + MCSPI_XFERLEVEL); // interrupt at least 12 bytes

    // Enable channel
    iowrite32(MCSPI_CHCTRL_EN, data->base + MCSPI_CHCTRL0);

    // Enable RX0_FULL interrupt
    iowrite32(MCSPI_CHSTAT_RX0_FULL, data->base + MCSPI_IRQENABLE);

        // chconf = ioread32(data->base + MCSPI_CHCONF0);
        // chconf |= 1u << 15; // DMA request read;
        // iowrite32(chconf, data->base + MCSPI_CHCONF0);

        // schedule_work(&data->re_request_work);
}
#if (DMA_USED_RX)
static int spi1_configure_dma(struct spi_device_data *data)
{
    struct dma_slave_config rx_conf = {0};
    int ret;

    if (data->rx_chan) {
        rx_conf.direction = DMA_DEV_TO_MEM;
        rx_conf.src_addr = (dma_addr_t)(SPI1_BASE + MCSPI_RX0);
        rx_conf.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE; /* UART uses 8-bit transfers */
        rx_conf.src_maxburst = 1; /* Single byte per transfer */
        ret = dmaengine_slave_config(data->rx_chan, &rx_conf);
        if (ret) {
            dev_err(data->dev, "Failed to configure RX DMA channel: %d\n", ret);
            return ret;
        }
        dev_info(data->dev, "RX DMA channel configured\n");
    }

    return 0;
}
#endif
static int spi_device_probe(struct platform_device *pdev)
{
    struct spi_device_data *data;
    int ret;
#if (DMA_USED_RX)
    struct clk *edma_clk;
#endif
    dev_info(&pdev->dev, "SPI1 device probed successfully with compatible 'spi1-based'\n");

    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    platform_set_drvdata(pdev, data);

    data->index = 0;
    data->jump_num = 0;
    data->is_get_count = 0;

    // Map SPI controller registers
    data->base = ioremap(SPI1_BASE, 0x400);
    if (!data->base) {
        dev_err(&pdev->dev, "Failed to map resource\n");
        return -ENOMEM;
    }
    data->dev = &pdev->dev;

    // Clock setup
    data->clk = devm_clk_get(&pdev->dev, "fck-spi1");
    if (IS_ERR(data->clk)) {
        dev_err(&pdev->dev, "Failed to get clock: %ld\n", PTR_ERR(data->clk));
        iounmap(data->base);
        return PTR_ERR(data->clk);
    }
    ret = clk_prepare_enable(data->clk);
    if (ret) {
        dev_err(&pdev->dev, "Failed to enable clock: %d\n", ret);
        iounmap(data->base);
        return ret;
    }
    dev_info(&pdev->dev, "SPI1 clock rate: %lu Hz\n", clk_get_rate(data->clk));

    // Initialize workqueue
    INIT_WORK(&data->re_request_work, re_request_irq_work);

    // hw init using registers
    spi_hw_init(data);

    // ========== Request IRQ (hwirq 125 for SPI1 on AM33xx) ==========
    data->irq = platform_get_irq(pdev, 0);
    if (data->irq < 0) {
        dev_err(&pdev->dev, "Failed to get IRQ: %d\n", data->irq);
        clk_disable_unprepare(data->clk);
        iounmap(data->base);
        return data->irq;
    }
    dev_info(data->dev, "swirq of SPI1 = %d\n", data->irq);
    ret = devm_request_irq(&pdev->dev, data->irq, irqHandler, 0, "spi1", data);
    if (ret < 0) {
        dev_err(&pdev->dev, "Unable to request IRQ %d: %d\n", data->irq, ret);
        clk_disable_unprepare(data->clk);
        iounmap(data->base);
        return ret;
    }

#if (DMA_USED_RX)
    /* =============== DMA ================= */
    edma_clk = devm_clk_get(&pdev->dev, "fck-dma");
    if (IS_ERR(edma_clk)) {
        dev_err(&pdev->dev, "Failed to get eDMA clock: %ld\n", PTR_ERR(edma_clk));
        return PTR_ERR(edma_clk);
    }
    ret = clk_prepare_enable(edma_clk);
    if (ret) {
        dev_err(&pdev->dev, "Failed to enable eDMA clock: %d\n", ret);
        return ret;
    }
    dev_info(&pdev->dev, "Edma clock rate: %lu Hz\n", clk_get_rate(edma_clk));

    /* Request DMA channels */
    data->rx_chan = dma_request_chan(&pdev->dev, "rx1");
    if (IS_ERR(data->rx_chan)) {
        dev_info(&pdev->dev, "No TX DMA channel, falling back to non-DMA mode: %ld\n",
                 PTR_ERR(data->rx_chan));
        data->rx_chan = NULL;
    }

    /* COnfig parameters for DMA */
    data->use_dma = (data->rx_chan);
    if (data->use_dma){
        dev_info(&pdev->dev, "Using DMA for SPI transfers\n");
        ret = spi1_configure_dma(data);
        if (ret) {
            dev_info(&pdev->dev, "Failed to configure DMA, falling back to non-DMA mode\n");
            if (data->rx_chan)
                dma_release_channel(data->rx_chan);
            data->rx_chan = NULL;
            data->use_dma = false;
        }
        else {
            dev_info(&pdev->dev, "Succeed to configure DMA\n");
        }
    }
    else
        dev_info(&pdev->dev, "Using interrupt-driven transfers\n");

    init_completion(&data->rx_completion);
#endif

    // ========== Create character device ==========
    ret = alloc_chrdev_region(&data->dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to allocate chrdev region: %d\n", ret);
        kthread_stop(data->recv_thread);
        clk_disable_unprepare(data->clk);
        iounmap(data->base);
        return ret;
    }

    cdev_init(&data->cdev, &spi_device_fops);
    data->cdev.owner = THIS_MODULE;
    ret = cdev_add(&data->cdev, data->dev_num, 1);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to add cdev: %d\n", ret);
        unregister_chrdev_region(data->dev_num, 1);
        kthread_stop(data->recv_thread);
        clk_disable_unprepare(data->clk);
        iounmap(data->base);
        return ret;
    }

    data->class = class_create(THIS_MODULE, "spi1_class");
    if (IS_ERR(data->class)) {
        dev_err(&pdev->dev, "Failed to create class: %ld\n", PTR_ERR(data->class));
        cdev_del(&data->cdev);
        unregister_chrdev_region(data->dev_num, 1);
        kthread_stop(data->recv_thread);
        clk_disable_unprepare(data->clk);
        iounmap(data->base);
        return PTR_ERR(data->class);
    }

    data->dev = device_create(data->class, &pdev->dev, data->dev_num, NULL, "spi1");
    if (IS_ERR(data->dev)) {
        dev_err(&pdev->dev, "Failed to create device: %ld\n", PTR_ERR(data->dev));
        class_destroy(data->class);
        cdev_del(&data->cdev);
        unregister_chrdev_region(data->dev_num, 1);
        kthread_stop(data->recv_thread);
        clk_disable_unprepare(data->clk);
        iounmap(data->base);
        return PTR_ERR(data->dev);
    }

    dev_info(&pdev->dev, "Created /dev/%s\n", DEVICE_NAME);
    return 0;
}

static int spi_device_remove(struct platform_device *pdev)
{
    struct spi_device_data *data = platform_get_drvdata(pdev);

    // Cancel any pending workqueue tasks
    cancel_work_sync(&data->re_request_work);

    // Disable interrupts
    iowrite32(0, data->base + MCSPI_IRQENABLE);

    if (data->rx_chan)
        dma_release_channel(data->rx_chan);

    // Stop receive thread
    if (data->recv_thread)
        kthread_stop(data->recv_thread);

    // Disable channel
    iowrite32(0, data->base + MCSPI_CHCTRL0);

    // Clean up character device
    if (data->dev)
        device_destroy(data->class, data->dev_num);
    if (data->class)
        class_destroy(data->class);
    cdev_del(&data->cdev);
    unregister_chrdev_region(data->dev_num, 1);

    // Disable clock and unmap registers
    clk_disable_unprepare(data->clk);
    iounmap(data->base);

    dev_info(&pdev->dev, "SPI1 device removed\n");
    return 0;
}

static const struct of_device_id spi_device_of_match[] = {
    { .compatible = "spi1-based" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, spi_device_of_match);

static struct platform_driver spi_device_driver = {
    .probe = spi_device_probe,
    .remove = spi_device_remove,
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = spi_device_of_match,
    },
};

module_platform_driver(spi_device_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Custom SPI Slave Device Driver for BeagleBone Black SPI1");
