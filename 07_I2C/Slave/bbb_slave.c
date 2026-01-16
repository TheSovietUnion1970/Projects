#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include "main.h"

// Register base addresses
// #define i2c1_BASE       0x4819c000
#define I2C1_BASE       0x4802A000
#define GPIO_BASE       0x44e10000

// i2c1 registers (offsets)
#define I2C_SYSC        0x10  // System configuration
#define I2C_SYSS        0x90  // System status
#define I2C_CON         0xA4  // Control
#define I2C_PSC         0xB0  // Prescaler
#define I2C_SCLL        0xB4  // SCL low time
#define I2C_SCLH        0xB8  // SCL high time
#define I2C_SA          0xAC  // Slave address
#define I2C_OA          0xA8  // Own address
#define I2C_CNT         0x98  // Data count
#define I2C_DATA        0x9C  // Data
#define I2C_IRQENABLE_SET 0x2C  // Enable interrupts
#define I2C_IRQENABLE_CLR 0x30  // Enable interrupts
#define I2C_IRQSTATUS_RAW 0x24  // Interrupt raw status
#define I2C_IRQSTATUS   0x28  // Interrupt status
#define I2C_BUF         0x94  // Buffer
#define I2C_DMARXENABLE_SET 0x38

#define I2C_IRQSTATUS_RAW_XRDY BIT(4)
#define I2C_IRQSTATUS_RAW_BB BIT(12)
#define I2C_IRQSTATUS_RAW_RRDY BIT(3)
#define I2C_IRQSTATUS_RAW_ARDY BIT(2)
#define I2C_IRQSTATUS_RAW_NACK BIT(1)

#define I2C_BUF_RXTRSH 8 // [13:8]
#define I2C_BUF_RXFIFO_CLR BIT(14)

#define XRDY_IE BIT(4)
#define RRDY_IE BIT(3)
#define AAS_IE BIT(9)

#define DRIVER_NAME "i2c1_device_driver"
#define DEVICE_NAME "i2c1"

struct i2c_device_data {
    struct i2c_adapter *adap;

    dev_t dev_num;
    struct cdev cdev;
    struct class *class;
    struct device *dev;

    void __iomem *base;  // Mapped base address of i2c1 registers
    void __iomem *base_gpio;
    struct clk *clk;

    int irq;
    int count;
    int data_count;

    u8 rx[256];
    int byte;
    bool isGetByte;
    u8* kbuf;
    dma_addr_t dma_dst_addr;

    /* DMA-related fields */
    struct dma_chan *rx_chan;
    struct completion rx_completion;
    bool use_dma; /* Flag to indicate if DMA is used */

    struct work_struct re_request_work;
    bool is_scheduled;
};

void i2c1_slave_init(struct i2c_device_data *data);
#if (DMA_USED)
static void i2c1_rx_dma_callback(void *data)
{
    struct i2c_device_data *i2c1 = data;
    complete(&i2c1->rx_completion);
    dev_info(i2c1->dev, "RX DMA callback called\n");

    if (i2c1->kbuf){
        if (i2c1->kbuf[i2c1->byte - 1] == '\n'){
            i2c1->kbuf[i2c1->byte - 1] = '\0';
            i2c1->data_count = 0;
            i2c1->count = 0; // avoid too many interrupts if data is read
            i2c1->byte = -1;
            dev_info(i2c1->dev, "[DMA] Rx: %s\n", i2c1->kbuf);
        }
    }
    else {
        printk("Error: printing data using DMA\n");
    }
}

void DMA_probe(struct i2c_device_data* data){
    /* Allocate DMA-coherent buffer */
    data->kbuf = dma_alloc_coherent(data->dev, data->byte, &data->dma_dst_addr, GFP_KERNEL);
    if (!data->kbuf) {
        dev_err(data->dev, "Failed to allocate DMA-coherent buffer\n");
        return;
    }
}

void Dma_read(struct i2c_device_data* data, u8 size){
    struct dma_async_tx_descriptor *rx_desc;
    int ret;
    int i;

    memset(data->kbuf, 0x40, data->byte);

    reinit_completion(&data->rx_completion);
    rx_desc = dmaengine_prep_slave_single(data->rx_chan, data->dma_dst_addr + RX_TRIGGER + 1, size - RX_TRIGGER - 1,
                                            DMA_DEV_TO_MEM, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
    if (!rx_desc) {
        dev_err(data->dev, "Failed to prepare RX DMA descriptor\n");

        dma_free_coherent(data->dev, data->byte, data->kbuf, data->dma_dst_addr);
        data->kbuf = NULL;
    }

    rx_desc->callback = i2c1_rx_dma_callback;
    rx_desc->callback_param = data;

    dev_info(data->dev, "Submitting RX DMA descriptor\n");
    dmaengine_submit(rx_desc);

    dev_info(data->dev, "Issuing RX DMA pending\n");
    dma_async_issue_pending(data->rx_chan);

    data->count = 0;
    for (i = 0; i < RX_TRIGGER + 1; i++){
        ((char*)data->kbuf)[data->count++] = ioread32(data->base + I2C_DATA);
    }

    // due to every 2ms, msg is sent, so the timeout should be 2ms
    ret = wait_for_completion_timeout(&data->rx_completion, msecs_to_jiffies(MS_TIMEOUT));
    if (ret == 0) {
        dev_err(data->dev, "DMA RX timeout\n");
        dmaengine_terminate_sync(data->rx_chan);

        dma_free_coherent(data->dev, data->byte, data->kbuf, data->dma_dst_addr);
        data->kbuf = NULL;
    }
}

static int i2c1_configure_dma(struct i2c_device_data *data)
{
    struct dma_slave_config rx_conf = {0};
    int ret;

    if (data->rx_chan) {
        rx_conf.direction = DMA_DEV_TO_MEM;
        rx_conf.src_addr = (dma_addr_t)(I2C1_BASE + I2C_DATA);
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

static void re_request_irq_work(struct work_struct *work)
{
    struct i2c_device_data *data = container_of(work, struct i2c_device_data, re_request_work);
    Dma_read(data, data->byte);
    i2c1_slave_init(data); // reinit slave
    data->isGetByte = false;
}
#endif
static irqreturn_t irqHandler(int irq, void *d)
{
    struct i2c_device_data *data = d;
    u32 irqsts;
#if (!DMA_USED)
    int i = 0;
#endif

    irqsts = ioread32(data->base + I2C_IRQSTATUS);

    if (data->data_count == 0){
        memset(data->rx, 0, 256);
    }

    if ((irqsts & RRDY_IE) == RRDY_IE) {
#if (DMA_USED)
        if (!data->isGetByte) {
            data->byte = ioread32(data->base + I2C_DATA); // Read bytes
            data->isGetByte = true;
        }
        else {
            if (data->byte > 0){
                iowrite32(DMA_USED, data->base + I2C_DMARXENABLE_SET); // Receive data DMA enabled

                schedule_work(&data->re_request_work);

                iowrite32(0, data->base + I2C_IRQENABLE_SET); 
                iowrite32(0xFF, data->base + I2C_IRQENABLE_CLR);            
            }
            else {
                printk("Byte must be > 0\n");
            }
        }
#else
        for (i = 0; i < RX_TRIGGER + 1; i++){
            data->rx[data->data_count++] = ioread32(data->base + I2C_DATA); // Read data
        }
#endif
        // Clear the XRDY interrupt
        iowrite32(RRDY_IE, data->base + I2C_IRQSTATUS);
    }

    if ((irqsts & AAS_IE) == AAS_IE) {

        // used when an address slave is choosen by master
        // Clear the AAS interrupt
        iowrite32(AAS_IE, data->base + I2C_IRQSTATUS);
    }
#if (!DMA_USED)
    if (data->rx[data->data_count - 1] == '\n'){
        data->rx[data->data_count - 1] = '\0';
        data->data_count = 0;
        data->count = 0; // avoid too many interrupts if data is read

        dev_info(data->dev, "[CPU] Rx: %s\n", data->rx);
    }
#endif
    data->count++;
    if (data->count > 100){
        printk("Too many interrupts, 0x%x\n", ioread32(data->base + I2C_IRQSTATUS));
        iowrite32(0, data->base + I2C_IRQENABLE_SET); 
        iowrite32(0xFF, data->base + I2C_IRQENABLE_CLR); 
        data->count = 0;
    }

    return IRQ_HANDLED;
}

void init_clk2(struct i2c_device_data *data, u32 fclk_rate, u32 internal_speed, u32 speed)
{
    u32 scll = 0, sclh = 0;
    u8 psc = 0;

    /* Compute prescaler divisor */
    psc = fclk_rate / internal_speed;
    psc = psc - 1;
    iowrite32(psc, data->base + I2C_PSC); // Prescaler: 48 MHz / (4+1) = 9.6 MHz

    scll = internal_speed / (speed*2) - 7;
    sclh = internal_speed / (speed*2) - 5;
    iowrite32(scll, data->base + I2C_SCLL); // SCL low time
    iowrite32(sclh, data->base + I2C_SCLH); // SCL high time
#if (PRINT_DEBUG)
    dev_info(data->dev, "[init_clk2] pdc = %d, scll = %d. sclh = %d\n", psc, scll, sclh);
#endif
}

void GPIO_init(struct i2c_device_data *data)
{
    // iowrite32(0x32, data->base_gpio + 0x954); // i2c1_scl
    // iowrite32(0x32, data->base_gpio + 0x950); // i2c1_sda

    iowrite32(0x32, data->base_gpio + 0x95c); // i2c1_scl
    iowrite32(0x32, data->base_gpio + 0x958); // i2c1_sda
}

// Initialize i2c1 as slave
void i2c1_slave_init(struct i2c_device_data *data) {
    // enable_i2c1_clock(data);
    u32 i2c_con = 0;

    i2c_con = ioread32(data->base + I2C_CON);
    i2c_con |= (0u << 15); // [15] disable i2c module before reset
    iowrite32(i2c_con, data->base + I2C_CON);
#if (PRINT_DEBUG)
    dev_info(data->dev, "[i2c1_slave_init] Begin reset\n");
#endif
    iowrite32(0x2, data->base + I2C_SYSC); // Set soft reset

    i2c_con = ioread32(data->base + I2C_CON);
    i2c_con |= (1u << 15); // [15] enable i2c module before reset
    iowrite32(i2c_con, data->base + I2C_CON);
#if (PRINT_DEBUG)
    dev_info(data->dev, "[i2c1_slave_init] Wait to reset\n");
#endif
    while (!(ioread32(data->base + I2C_SYSS)&(1u)));  // Wait for reset complete
#if (PRINT_DEBUG)
    dev_info(data->dev, "[i2c1_slave_init] Done reset\n");
#endif
    iowrite32(0x0, data->base + I2C_SYSC); // Clear reset - normal mode

    i2c_con = ioread32(data->base + I2C_CON);
    i2c_con &= ~(1u << 15); // disable i2c module
    iowrite32(i2c_con, data->base + I2C_CON);

    iowrite32(SLAVE_ADDRESS, data->base + I2C_OA); // Set own address of slave (0x40)
    iowrite32(SLAVE_ADDRESS, data->base + I2C_SA); 

    init_clk2(data, 48000000, 12000000, 100000);

    i2c_con = ioread32(data->base + I2C_CON);
    i2c_con |= (1u << 15)|(0u << 10)|(0u << 9); // [15] enable i2c module, [MST:10]: slave mode, [TRX:9]:  MST = 0, TRX = x, Operating Mode = Slave receiver
    iowrite32(i2c_con, data->base + I2C_CON);

    iowrite32((RX_TRIGGER << I2C_BUF_RXTRSH) | I2C_BUF_RXFIFO_CLR, data->base + I2C_BUF); // clear RX FIFO, RX threshold is 1 byte
    iowrite32(RRDY_IE | AAS_IE, data->base + I2C_IRQENABLE_SET); // Receive data ready interrupt enabled
}

static const struct file_operations i2c_device_fops = {
    .owner = THIS_MODULE,
};

static int i2c1_probe(struct platform_device *pdev)
{
    struct i2c_device_data *data;
#if (DMA_USED)
    struct clk *edma_clk;
#endif
    int ret;

    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    platform_set_drvdata(pdev, data);

    data->base_gpio = ioremap(GPIO_BASE, 0x1000);
    data->base = ioremap(I2C1_BASE, 0x1000);
    data->dev = &pdev->dev;
    data->is_scheduled = 0;
    data->byte = -1;
    data->byte = 50;
    data->isGetByte = false;
    data->data_count = 0;

    /* Clock setup (assuming this part is unchanged) */
    data->clk = devm_clk_get(&pdev->dev, "fck-i2c1");
    if (IS_ERR(data->clk)) {
        dev_err(&pdev->dev, "Failed to get clock: %ld\n", PTR_ERR(data->clk));
        return PTR_ERR(data->clk);
    }
    ret = clk_prepare_enable(data->clk);
    if (ret) {
        dev_err(&pdev->dev, "Failed to enable clock: %d\n", ret);
        return ret;
    }
    dev_info(&pdev->dev, "i2c1 clock rate: %lu Hz\n", clk_get_rate(data->clk));

    GPIO_init(data);
    // Initialize hardware
    i2c1_slave_init(data);

    // ========== Request IRQ (hwirq 30 maps to swirq x on AM33xx) ==========
    data->irq = platform_get_irq(pdev, 0);
    if (data->irq < 0) {
        dev_err(&pdev->dev, "Failed Formatted: Unable to get IRQ: %d\n", data->irq);
        return data->irq;
    }
    ret = devm_request_irq(&pdev->dev, data->irq, irqHandler, 0, "i2c1", data);
    if (ret < 0) {
        dev_err(&pdev->dev, "Unable to request IRQ %d: %d\n", data->irq, ret);
        return ret;
    }

    dev_info(&pdev->dev, "IRQ num = %d\n", data->irq);

#if (DMA_USED)
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

    DMA_probe(data);

    /* COnfig parameters for DMA */
    data->use_dma = (data->rx_chan);
    if (data->use_dma){
        dev_info(&pdev->dev, "Using DMA for I2C1 transfers\n");
        ret = i2c1_configure_dma(data);
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

    // ============ Create character device
    ret = alloc_chrdev_region(&data->dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to allocate chrdev region: %d\n", ret);
        return ret;
    }

    cdev_init(&data->cdev, &i2c_device_fops);
    data->cdev.owner = THIS_MODULE;
    ret = cdev_add(&data->cdev, data->dev_num, 1);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to add cdev: %d\n", ret);
        //unregister_chrdev_region(&data->dev_num, 1);
        iounmap(data->base);
        return ret;
    }

    data->class = class_create(THIS_MODULE, "i2c1_class");
    if (IS_ERR(data->class)) {
        dev_err(&pdev->dev, "Failed to create class: %ld\n", PTR_ERR(data->class));
        cdev_del(&data->cdev);
        //unregister_chrdev_region(&data->dev_num, 1);
        iounmap(data->base);
        return PTR_ERR(data->class);
    }

    data->dev = device_create(data->class, &pdev->dev, data->dev_num, NULL, "i2c1");
    if (IS_ERR(data->dev)) {
        dev_err(&pdev->dev, "Failed to create device: %ld\n", PTR_ERR(data->dev));
        class_destroy(data->class);
        cdev_del(&data->cdev);
        //unregister_chrdev_region(&data->dev_num, 1);
        iounmap(data->base);
        return PTR_ERR(data->dev);
    }

    dev_info(&pdev->dev, "Created /dev/%s\n", DEVICE_NAME);

#if (DMA_USED)
    INIT_WORK(&data->re_request_work, re_request_irq_work);
#endif
    return 0;
}

static int i2c1_remove(struct platform_device *pdev)
{
    struct i2c_device_data *data = platform_get_drvdata(pdev);
#if (DMA_USED)
    // Cancel any pending workqueue tasks
    cancel_work_sync(&data->re_request_work);
#endif

    if (data->rx_chan)
        dma_release_channel(data->rx_chan);

    if (data->dev)
        device_destroy(data->class, data->dev_num);
    if (data->class)
        class_destroy(data->class);
    cdev_del(&data->cdev);

    return 0;
}

static const struct of_device_id i2c_device_of_match[] = {
    { .compatible = "i2c1-based" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, i2c_device_of_match);

static struct platform_driver i2c_device_driver = {
    .probe = i2c1_probe,
    .remove = i2c1_remove,
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = i2c_device_of_match,
    },
};

module_platform_driver(i2c_device_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vinh");
MODULE_DESCRIPTION("Custom I2C Device Driver for BeagleBone Black I2C1");