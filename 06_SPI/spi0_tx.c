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
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include "main.h"

#define DRIVER_NAME "spi0_device_driver"
#define DEVICE_NAME "spi0"
#define SLK 500000

#define SPI0_BASE 0x48030000

// Register offsets for OMAP2 McSPI (corrected for AM33xx SPI0, base 0x48030000)
#define MCSPI_CHCONF0   0x12c
#define MCSPI_SYSCONFIG   0x110
#define MCSPI_SYSSTATUS 0x114
#define MCSPI_CHSTAT0   0x130
#define MCSPI_CHCTRL0   0x134
#define MCSPI_TX0       0x138
#define MCSPI_RX0       0x13c
#define MCSPI_MODULCTRL 0x128
#define MCSPI_IRQSTATUS 0x118
#define MCSPI_IRQENABLE 0x11c
#define MCSPI_XFERLEVEL 0x17c

// Bitmasks for CHCONF0
#define MCSPI_CHCONF_POL    BIT(1)  // Clock polarity
#define MCSPI_CHCONF_PHA    BIT(0)  // Clock phase
#define MCSPI_CHCONF_WL_MASK (0x1f << 7)  // Word length mask
#define MCSPI_CHCONF_CLKD_MASK (0x0f << 2)  // Word length mask
#define MCSPI_CHCONF_DPE0   BIT(16) // Data pin 0 enable (MOSI)
#define MCSPI_CHCONF_DPE1   BIT(17) // Data pin 1 enable
#define MCSPI_CHCONF_IS     BIT(18) // Input select (MISO)
#define MCSPI_CHCONF_FORCE  BIT(20) // Force SPI_EN (CS)
#define MCSPI_CHCONF_EPOL	  BIT(6)

// Bitmasks for CHSTAT0
#define MCSPI_CHSTAT_EOT    BIT(2)  // end of frame
#define MCSPI_CHSTAT_TXS    BIT(1)  // TX register empty
#define MCSPI_CHSTAT_RXS    BIT(0)  // RX register full

// Bitmasks for CHCTRL0
#define MCSPI_CHCTRL_EN     BIT(0)  // Channel enable

// Bitmasks for MODULCTRL
#define MCSPI_MODULCTRL_SINGLE BIT(0) // Single-channel mode

// Bitmasks for IRQSTATUS and IRQENABLE
#define MCSPI_CHSTAT_RX0_FULL BIT(2) // RX0 full interrupt (same as CHSTAT0 RXS)
#define MCSPI_CHSTAT_TX0_EMPTY BIT(0)

#define MAX_IRQ_JUMPS 300
#define MS_DELAY 1000
#define FIFO_USED 1

// /*************** < CONTROL > *****************/
// #define DMA_USED_TX 1

// #if (DMA_USED_TX)
// #define TX_TRIGGER 0 /* For DMA, it's must be fixed */
// #else
// #define TX_TRIGGER 0 /* For non-DMA, we can adjust it */
// #endif
// /*********************************************/

/* 
NOTE:
+ for transmitting only -> choose transmit-only mode
+ active low = when CS becomes 0, data is sent
*/

struct spi_device_data {
    dev_t dev_num;
    struct cdev cdev;
    struct class *class;
    struct device *dev;

    void __iomem *base;  // Mapped base address of SPI0 registers
    struct clk *clk;
    struct clk *edma_clk;

    int irq;

    u8 TX[256];
    u8 jump;
    u8 jump_irq;
    u8 wait_TX;

    char* tx_buf;
    size_t count;
    size_t count_check;

    /* DMA-related fields */
    struct dma_chan *tx_chan;
    struct completion tx_completion;
    bool use_dma; /* Flag to indicate if DMA is used */
    bool DMA_IRQ;

    u8* src1;
};

static irqreturn_t irqHandler(int irq, void *d)
{
    struct spi_device_data *data = d;
    u32 irqsts;

    irqsts = ioread32(data->base + MCSPI_IRQSTATUS);

    if ((irqsts & MCSPI_CHSTAT_TX0_EMPTY) == MCSPI_CHSTAT_TX0_EMPTY) {

        data->wait_TX = 1;

        data->jump_irq++;

        //dev_info(data->dev, "irq - 0x%x, j = %d\n", ioread32(data->base + MCSPI_IRQSTATUS), data->count);
        //if (data->jump_irq >= data->count) iowrite32(0, data->base + MCSPI_IRQENABLE);
        // Clear the RX0_FULL interrupt
        iowrite32(MCSPI_CHSTAT_TX0_EMPTY, data->base + MCSPI_IRQSTATUS);
    }

    data->count_check++;
    if (data->count > MAX_IRQ_JUMPS){
        dev_info(data->dev, "Too many interrupts\n");
        iowrite32(0, data->base + MCSPI_IRQENABLE);
        data->count_check = 0;
    }

    return IRQ_HANDLED;
}

static void spi0_tx_dma_callback(void *data)
{
    struct spi_device_data *spi0 = data;
    complete(&spi0->tx_completion);
    dev_info(spi0->dev, "[DMA] DMA write is done\n");
    spi0->DMA_IRQ = 1;
}

static int spi_device_open(struct inode *inode, struct file *file)
{
    struct spi_device_data *data = container_of(inode->i_cdev, struct spi_device_data, cdev);
    file->private_data = data;
    return 0;
}

void Non_dma_write(struct spi_device_data* data){
    int i, j;
    unsigned long timeout;
    u32 ch0cfg;
#if (PRINT_DEBUG_TX)
    dev_info(data->dev, "Non-DMA transmitting, count = %d\n", data->count);
#endif
    // low CS 
    ch0cfg = ioread32(data->base + MCSPI_CHCONF0);
    ch0cfg |= MCSPI_CHCONF_FORCE; 
    iowrite32(ch0cfg, data->base + MCSPI_CHCONF0);

    iowrite32(data->count, data->base + MCSPI_TX0);
    msleep(1000);

    // high CS
    ch0cfg = ioread32(data->base + MCSPI_CHCONF0);
    ch0cfg &=~ MCSPI_CHCONF_FORCE; 
    iowrite32(ch0cfg, data->base + MCSPI_CHCONF0);

    //data->count = 0;
    msleep(MS_DELAY); // delay 2s before sending read data


    iowrite32(MCSPI_CHSTAT_TX0_EMPTY, data->base + MCSPI_IRQENABLE);

    // low CS - after it, one TX empty interupt is raised
    ch0cfg = ioread32(data->base + MCSPI_CHCONF0);
    ch0cfg |= MCSPI_CHCONF_FORCE; 
    iowrite32(ch0cfg, data->base + MCSPI_CHCONF0);
//data->count = 0;
    timeout = jiffies + msecs_to_jiffies(3000);

    // Perform the transfer
    for (i = 0; i < data->count; i++) {
        // Wait for TX register to be empty
        while ((ioread32(data->base + MCSPI_CHSTAT0) & MCSPI_CHSTAT_TXS) != MCSPI_CHSTAT_TXS) {
            if (time_after(jiffies, timeout)) {
                dev_err(data->dev, "Timeout writing at byte %d, chstst0 = 0x%x\n", i, ioread32(data->base + MCSPI_CHSTAT0));

                // Deassert CS on timeout
                ch0cfg = ioread32(data->base + MCSPI_CHCONF0); 
                 ch0cfg &=~ MCSPI_CHCONF_EPOL; 
                iowrite32(ch0cfg, data->base + MCSPI_CHCONF0);
                // Disable the channel
                iowrite32(0, data->base + MCSPI_CHCTRL0);
            }
            cpu_relax();
        }

        // Then, write the number of writes defined by MCSPI_XFERLEVEL[AEL] + 1
        for (j = 0; j < TX_TRIGGER + 1; j++){

            data->jump++;

            iowrite32(data->tx_buf[j + i], data->base + MCSPI_TX0);

            if (data->jump == data->count) break;
        }
        if (data->jump == data->count) data->wait_TX = 1;
        while(data->wait_TX != 1);

        udelay(100); // need a small break to after interrupt handler

        data->wait_TX = 0;
        i += TX_TRIGGER;

        //iowrite32(data->tx_buf[i], data->base + MCSPI_TX0);

    }
    iowrite32(0, data->base + MCSPI_IRQENABLE); // disable TX interrupt


    // high CS
    ch0cfg = ioread32(data->base + MCSPI_CHCONF0);
    ch0cfg &=~ MCSPI_CHCONF_FORCE; 
    iowrite32(ch0cfg, data->base + MCSPI_CHCONF0);

    data->count_check = 0;
}

void Dma_write(struct spi_device_data* data){
    dma_addr_t dma_addr;
    struct dma_async_tx_descriptor *tx_desc;
    int ret;
    int count = data->count;
    u32 ch0cfg;

    dev_info(data->dev, "DMA transmitting\n");

    // data->src1 = devm_kmalloc(data->dev, 3, GFP_KERNEL | GFP_DMA);
    // if (!data->src1) {
    //     dev_err(data->dev, "Failed to allocate src buffer\n");
    //     return;
    // }

    // dma_addr = dma_map_single(data->dev, data->src1, 3, DMA_TO_DEVICE);
    // if (dma_mapping_error(data->dev, dma_addr)) {
    //     dev_err(data->dev, "Failed to map source buffer for DMA\n");
    //     //return -ENOMEM;
    // }

    /* Allocate DMA-coherent buffer */
    data->src1 = dma_alloc_coherent(data->dev, count, &dma_addr, GFP_KERNEL | GFP_DMA);
    if (!data->src1) {
        dev_err(data->dev, "Failed to allocate DMA-coherent buffer\n");
        return;
    }

    //memset(data->src1, 0x40, count);
    memcpy(data->src1, data->tx_buf, count);

    // low CS 
    ch0cfg = ioread32(data->base + MCSPI_CHCONF0);
    ch0cfg |= MCSPI_CHCONF_FORCE; 
    iowrite32(ch0cfg, data->base + MCSPI_CHCONF0);

    msleep(500);
    iowrite32(count, data->base + MCSPI_TX0);
    msleep(500);

    // high CS
    ch0cfg = ioread32(data->base + MCSPI_CHCONF0);
    ch0cfg &=~ MCSPI_CHCONF_FORCE; 
    iowrite32(ch0cfg, data->base + MCSPI_CHCONF0);

    msleep(MS_DELAY); // delay 2s before sending read data


    // low CS 
    ch0cfg = ioread32(data->base + MCSPI_CHCONF0);
    ch0cfg |= MCSPI_CHCONF_FORCE; 
    iowrite32(ch0cfg, data->base + MCSPI_CHCONF0);

    ch0cfg = ioread32(data->base + MCSPI_CHCONF0);
    ch0cfg |= 1 << 14; // DMA write request
    iowrite32(ch0cfg, data->base + MCSPI_CHCONF0);

    reinit_completion(&data->tx_completion);
    tx_desc = dmaengine_prep_slave_single(data->tx_chan, dma_addr + 1, count - 1,
                                            DMA_MEM_TO_DEV, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
    if (!tx_desc) {
        dev_err(data->dev, "Failed to prepare TX DMA descriptor\n");
        goto free_buf;
    }

    tx_desc->callback = spi0_tx_dma_callback;
    tx_desc->callback_param = data;

    //dev_info(data->dev, "Submitting TX DMA descriptor\n");
    dmaengine_submit(tx_desc);

    //dev_info(data->dev, "Issuing TX DMA pending\n");
    dma_async_issue_pending(data->tx_chan);

    //udelay(50); // wait 50 us to let DMA transfer is done
    iowrite32(((char*)data->tx_buf)[0], data->base + MCSPI_TX0); // write the number of bytes defined in TX_TRIGGER to trigger DMA request (here TX_TRIGGER is 1, so write 1 byte manually)
    udelay(50); // wait 50 us to let DMA transfer is done

    ret = wait_for_completion_timeout(&data->tx_completion, msecs_to_jiffies(1000));
    if (ret == 0) {
        dev_err(data->dev, "DMA TX timeout\n");
        dmaengine_terminate_sync(data->tx_chan);
        goto free_buf;
    }

    udelay(500); // wait 50 us to let DMA transfer is done
    //msleep(2000);

    // high CS
    ch0cfg = ioread32(data->base + MCSPI_CHCONF0);
    ch0cfg &=~ MCSPI_CHCONF_FORCE; 
    iowrite32(ch0cfg, data->base + MCSPI_CHCONF0);

free_buf:
    dma_unmap_single(data->dev, dma_addr, count, DMA_TO_DEVICE);

    // high CS
    ch0cfg = ioread32(data->base + MCSPI_CHCONF0);
    ch0cfg &=~ MCSPI_CHCONF_FORCE; 
    iowrite32(ch0cfg, data->base + MCSPI_CHCONF0);
}

static ssize_t spi_device_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    struct spi_device_data *data = filp->private_data;
    char *kbuf;
    kbuf = kmalloc(count, GFP_KERNEL);
    if (!kbuf)
        return -ENOMEM;

    data->jump_irq = 0;
    data->jump = 0;
    data->DMA_IRQ = 0;

    if (copy_from_user(kbuf, buf, count)) {
        kfree(kbuf);
        return -EFAULT;
    }
    data->tx_buf = kbuf;
    data->count = count;

#if (DMA_USED_TX)
    Dma_write(data);
#else
    Non_dma_write(data);
#endif

    // usleep_range(1000, 3000);
#if (PRINT_DEBUG_TX)
    dev_info(data->dev, "Done writing %zu bytes: %*ph\n", count, (int)count, data->tx_buf);
#endif
    return count;
}

static ssize_t spi_device_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
    struct spi_device_data *data = filp->private_data;

    //iowrite32(1, data->base + MCSPI_CHCTRL0);
    dev_info(data->dev, "Jump IRQ = %d, jump w = %d, coun = %d\n", data->jump_irq, data->jump, data->count);

    if (data->DMA_IRQ) dev_info(data->dev, "TX DMA callback called\n");

    data->jump_irq = 0;
    data->jump = 0;
    data->DMA_IRQ = 0;

    return 0;
}

static const struct file_operations spi_device_fops = {
    .owner = THIS_MODULE,
    .open = spi_device_open,
    .write = spi_device_write,
    .read = spi_device_read,
};

void spi_hw_init(struct spi_device_data* data){
    u32 chconf;
    // wait until software resets
    iowrite32(1<<1, data->base + MCSPI_SYSCONFIG);
    while(!(ioread32(data->base + MCSPI_SYSSTATUS)));

    // Configure SPI controller (single-channel mode)
    iowrite32(MCSPI_MODULCTRL_SINGLE, data->base + MCSPI_MODULCTRL);

    chconf = ioread32(data->base + MCSPI_CHCONF0);
    // Configure CHCONF0 for SPI0
    chconf |= (8 - 1) << 7; // 8-bit word length
    chconf &= ~MCSPI_CHCONF_POL; // SPI Mode 0
    chconf &= ~MCSPI_CHCONF_PHA;

    chconf &= ~MCSPI_CHCONF_IS; // D0 as input (MISO)
    chconf |= MCSPI_CHCONF_DPE0; 

    chconf &= ~MCSPI_CHCONF_DPE1; // D1 as output (MOSI)

    // MCSPI_CHCONF_EPOL -> 0

    chconf &= ~(1<<29); // Clock granularity of power of 2
    /* set clock divisor */
    chconf |= 6 << 2; // Divide by 64 (2^6 = 64) -> clk =  48 000 000 / 64 = 750 000 Hz

    chconf |= MCSPI_CHCONF_EPOL; // low EN - EPOL = 1, high EN - EPOL = 0

    chconf |= 2 << 12; // transmit-mode only

    chconf |= FIFO_USED << 27; // The buffer is used to transmit data.

    iowrite32(chconf, data->base + MCSPI_CHCONF0);


    iowrite32(TX_TRIGGER, data->base + MCSPI_XFERLEVEL); // interrupt at least 3 bytes

    // in DMA, no need interrupt
    // Enable TX0 empty interrupt
    //iowrite32(MCSPI_CHSTAT_TX0_EMPTY, data->base + MCSPI_IRQENABLE);

    // Enable the SPI channel
    iowrite32(1, data->base + MCSPI_CHCTRL0);

    dev_info(data->dev, "ch0ststus = 0x%x\n", ioread32(data->base + MCSPI_CHSTAT0));
}
#if (DMA_USED_TX)
static int spi0_configure_dma(struct spi_device_data *data)
{
    struct dma_slave_config tx_conf = {0};
    int ret;

    if (data->tx_chan) {
        tx_conf.direction = DMA_MEM_TO_DEV;
        tx_conf.dst_addr = (dma_addr_t)(SPI0_BASE + MCSPI_TX0);
        tx_conf.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE; /* SPI0 uses 8-bit transfers */
        tx_conf.dst_maxburst = 1; /* Single byte per transfer */
        ret = dmaengine_slave_config(data->tx_chan, &tx_conf);
        if (ret) {
            dev_err(data->dev, "Failed to configure TX DMA channel: %d\n", ret);
            return ret;
        }
        dev_info(data->dev, "TX DMA channel configured\n");
    }

    return 0;
}
#endif
static int spi_device_probe(struct platform_device *pdev)
{
    struct spi_device_data *data;
    int ret;
#if (DMA_USED_TX)
    struct clk *edma_clk;
#endif

    dev_info(&pdev->dev, "SPI device probed successfully with compatible 'spi0-based'\n");

    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    platform_set_drvdata(pdev, data);

    data->count = 0;

    // Map SPI controller registers
    // if (of_address_to_resource(pdev->dev.of_node, 0, &res)) {
    //     dev_err(&pdev->dev, "Failed to get resource\n");
    //     return -EINVAL;
    // }
    // data->base = devm_ioremap_resource(&pdev->dev, &res);
    // if (IS_ERR(data->base)) {
    //     dev_err(&pdev->dev, "Failed to map resource\n");
    //     return PTR_ERR(data->base);
    // }
    data->base = ioremap(SPI0_BASE, 0x10000);
    data->dev = &pdev->dev;
    data->jump = 0;
    data->wait_TX = 0;
    data->DMA_IRQ = 0;

    /* Clock setup (assuming this part is unchanged) */
    data->clk = devm_clk_get(&pdev->dev, "fck-spi0");
    if (IS_ERR(data->clk)) {
        dev_err(&pdev->dev, "Failed to get clock: %ld\n", PTR_ERR(data->clk));
        return PTR_ERR(data->clk);
    }
    ret = clk_prepare_enable(data->clk);
    if (ret) {
        dev_err(&pdev->dev, "Failed to enable clock: %d\n", ret);
        return ret;
    }
    dev_info(&pdev->dev, "SPI0 clock rate: %lu Hz\n", clk_get_rate(data->clk));

    spi_hw_init(data);

    // ========== Request IRQ (hwirq 65 maps to swirq 19 on AM33xx) ==========
    data->irq = platform_get_irq(pdev, 0);
    if (data->irq < 0) {
        dev_err(&pdev->dev, "Failed Formatted: Unable to get IRQ: %d\n", data->irq);
        return data->irq;
    }
    ret = devm_request_irq(&pdev->dev, data->irq, irqHandler, 0, "SPI0", data);
    if (ret < 0) {
        dev_err(&pdev->dev, "Unable to request IRQ %d: %d\n", data->irq, ret);
        return ret;
    }
#if (DMA_USED_TX)
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
    data->tx_chan = dma_request_chan(&pdev->dev, "tx1");
    if (IS_ERR(data->tx_chan)) {
        dev_info(&pdev->dev, "No TX DMA channel, falling back to non-DMA mode: %ld\n",
                 PTR_ERR(data->tx_chan));
        data->tx_chan = NULL;
        return -1;
    }

    /* COnfig parameters for DMA */
    data->use_dma = (data->tx_chan);
    if (data->use_dma){
        dev_info(&pdev->dev, "Using DMA for UART transfers\n");
        ret = spi0_configure_dma(data);
        if (ret) {
            dev_info(&pdev->dev, "Failed to configure DMA, falling back to non-DMA mode\n");
            if (data->tx_chan)
                dma_release_channel(data->tx_chan);
            data->tx_chan = NULL;
            data->use_dma = false;
        }
        else {
            dev_info(&pdev->dev, "Succeed to configure DMA\n");
        }
    }
    else
        dev_info(&pdev->dev, "Using interrupt-driven transfers\n");

    init_completion(&data->tx_completion);
#endif
    // ========== Create character device ==========
    ret = alloc_chrdev_region(&data->dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to allocate chrdev region: %d\n", ret);
        return ret;
    }

    cdev_init(&data->cdev, &spi_device_fops);
    data->cdev.owner = THIS_MODULE;
    ret = cdev_add(&data->cdev, data->dev_num, 1);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to add cdev: %d\n", ret);
        //unregister_chrdev_region(&data->dev_num, 1);
        iounmap(data->base);
        return ret;
    }

    data->class = class_create(THIS_MODULE, "spi0_class");
    if (IS_ERR(data->class)) {
        dev_err(&pdev->dev, "Failed to create class: %ld\n", PTR_ERR(data->class));
        cdev_del(&data->cdev);
        //unregister_chrdev_region(&data->dev_num, 1);
        iounmap(data->base);
        return PTR_ERR(data->class);
    }

    data->dev = device_create(data->class, &pdev->dev, data->dev_num, NULL, "spi0");
    if (IS_ERR(data->dev)) {
        dev_err(&pdev->dev, "Failed to create device: %ld\n", PTR_ERR(data->dev));
        class_destroy(data->class);
        cdev_del(&data->cdev);
        //unregister_chrdev_region(&data->dev_num, 1);
        iounmap(data->base);
        return PTR_ERR(data->dev);
    }

    dev_info(&pdev->dev, "Created /dev/%s\n", DEVICE_NAME);

    return 0;
}

static int spi_device_remove(struct platform_device *pdev)
{
    struct spi_device_data *data = platform_get_drvdata(pdev);

    if (data->tx_chan)
        dma_release_channel(data->tx_chan);

    if (data->dev)
        device_destroy(data->class, data->dev_num);
    if (data->class)
        class_destroy(data->class);

    cdev_del(&data->cdev);
    unregister_chrdev_region(data->dev_num, 1);
    dev_info(&pdev->dev, "SPI device removed\n");
    return 0;
}

static const struct of_device_id spi_device_of_match[] = {
    { .compatible = "spi0-based" },
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
MODULE_DESCRIPTION("Custom SPI Device Driver for BeagleBone Black SPI0");
