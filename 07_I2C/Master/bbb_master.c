#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include "main.h"

// Register base addresses
#define I2C1_BASE       0x4802A000
#define GPIO_BASE       0x44e10000
#define CM_PER_BASE     0x44E00000

// I2C1 registers (offsets)
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
#define I2C_IRQSTATUS_RAW 0x24  // Interrupt raw status
#define I2C_IRQENABLE_CLR 0x30  // Enable interrupts
#define I2C_IRQSTATUS   0x28  // Interrupt status
#define I2C_BUF         0x94  // Buffer
#define I2C_DMATXENABLE_SET 0x3C

#define I2C_IRQSTATUS_RAW_XRDY BIT(4)
#define I2C_IRQSTATUS_RAW_BB BIT(12)
#define I2C_IRQSTATUS_RAW_ARDY BIT(2)
#define I2C_IRQSTATUS_RAW_NACK BIT(1)

#define XRDY_IE BIT(4)
#define XDR_IE BIT(14)

#define I2C_BUF_TXTRSH 0 // [5:0]
#define I2C_BUF_TXFIFO_CLR BIT(6)

#define MS_DELAY 2000

#define DRIVER_NAME "i2c1_device_driver"
#define DEVICE_NAME "i2c1"

struct i2c_device_data {

    dev_t dev_num;
    struct cdev cdev;
    struct class *class;
    struct device *dev;

    void __iomem *base;  // Mapped base address of I2c1 registers
    void __iomem *base_gpio;
    struct clk *clk;

    int irq;
    int count;

    u8 *tx;
    int tx_count;
    bool is_ended;

    /* DMA-related fields */
    struct dma_chan *tx_chan;
    struct completion tx_completion;
    bool use_dma; /* Flag to indicate if DMA is used */
};

static void i2c1_tx_dma_callback(void *data)
{
    struct i2c_device_data *i2c1 = data;
    complete(&i2c1->tx_completion);
    dev_info(i2c1->dev, "TX DMA callback called\n");
}

static irqreturn_t irqHandler(int irq, void *d)
{
    struct i2c_device_data *data = d;
    u32 irqsts;

    irqsts = ioread32(data->base + I2C_IRQSTATUS);
#if (PRINT_DEBUG)
    printk("[Handler] irqsts = 0x%x\n", irqsts);
#endif
    if ((irqsts & XRDY_IE) == XRDY_IE) {

        if (data->tx[data->tx_count] == '\n'){
            data->tx[data->tx_count] = END_STRING;
            data->is_ended = 1;
        }

        iowrite32(data->tx[data->tx_count++], data->base + I2C_DATA); // Write data
    
        // Clear the XRDY interrupt
        iowrite32(XRDY_IE, data->base + I2C_IRQSTATUS);

        //printk("cnt = %d\n", data->tx_count-1);
    }

    // draining feature to make sure the remaining free TX_TRIGGER bytes sent
    if ((irqsts & XDR_IE) == XDR_IE) {

        if (data->tx[data->tx_count] == '\n'){
            data->tx[data->tx_count] = END_STRING;
            data->is_ended = 1;
        }

        iowrite32(data->tx[data->tx_count++], data->base + I2C_DATA); // Write data
    
        // Clear the XRDY interrupt
        iowrite32(XDR_IE, data->base + I2C_IRQSTATUS);

        //printk("XDR_IE = %d\n", data->tx_count-1);
    }

    // end of transmit
    if (data->tx[data->tx_count-1] == END_STRING){
        data->tx_count = 0;
        data->count = 0;
    }

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
 
    dev_info(data->dev, "pdc = %d, scll = %d. sclh = %d\n", psc, scll, sclh);
}

// Initialize I2C1 as master
void i2c1_master_init(struct i2c_device_data *data) {
    // enable_i2c1_clock(data);
    u32 i2c_con = 0;

    i2c_con = ioread32(data->base + I2C_CON);
    i2c_con &= ~(1u << 15); // [15] disable i2c module before reset
    iowrite32(i2c_con, data->base + I2C_CON);

    dev_info(data->dev, "Begin reset\n");
    iowrite32(0x2, data->base + I2C_SYSC); // Set soft reset

    i2c_con = ioread32(data->base + I2C_CON);
    i2c_con |= (1u << 15); // [15] enable i2c module before reset
    iowrite32(i2c_con, data->base + I2C_CON);

    dev_info(data->dev, "Wait to reset ...\n");
    while (!(ioread32(data->base + I2C_SYSS)&(1u)));  // Wait for reset complete
    iowrite32(0x0, data->base + I2C_SYSC); // Clear reset - normal mode
    dev_info(data->dev, "Done reset\n");



    i2c_con = ioread32(data->base + I2C_CON);
    i2c_con &= ~(1u << 15); // disable i2c module
    iowrite32(i2c_con, data->base + I2C_CON);

    //init_clk1(data, 48000000, 400000);
    init_clk2(data, 48000000, 12000000, 100000);

    iowrite32(0x30, data->base + I2C_OA);

    i2c_con = ioread32(data->base + I2C_CON);
    i2c_con |= (1u << 15)|(1u << 10)|(1u << 9); // [15] enable i2c module, [MST:10]: master mode, [TRX:9]: MST = 1, TRX = 1, Operating Modes = Master transmitter
    iowrite32(i2c_con, data->base + I2C_CON);

    iowrite32((TX_TRIGGER << I2C_BUF_TXTRSH) | I2C_BUF_TXFIFO_CLR, data->base + I2C_BUF); // clear TX FIFO, TX threshold is 1 byte

#if (!DMA_USED)
    iowrite32(XRDY_IE | XDR_IE, data->base + I2C_IRQENABLE_SET); // TX data ready interrupt enabled
#else
    iowrite32(DMA_USED, data->base + I2C_DMATXENABLE_SET); // Transmit data DMA enabled
#endif
}

static int i2c1_master_open(struct inode *inode, struct file *file)
{
    struct i2c_device_data *data = container_of(inode->i_cdev, struct i2c_device_data, cdev);
    file->private_data = data;
    return 0;
}

// Write data to slave
int i2c1_write(struct i2c_device_data *data, uint16_t slave_addr, uint8_t *tx, uint32_t len) {
    u32 i2c_sts_raw = 0, i2c_con = 0;
    unsigned long timeout;

    timeout = jiffies + msecs_to_jiffies(2000);
    while ((ioread32(data->base + I2C_IRQSTATUS_RAW)&I2C_IRQSTATUS_RAW_BB) == I2C_IRQSTATUS_RAW_BB)  // Wait for bus to be free
    {
        if (time_after(jiffies, timeout)) {
            dev_err(data->dev, "Timeout BB, irqsts_raw = 0x%x\n", ioread32(data->base + I2C_IRQSTATUS_RAW));

            i2c_con = ioread32(data->base + I2C_CON);
            i2c_con &=~ 0x1; 
            i2c_con |= 0x2; // Stop condition
            iowrite32(i2c_con, data->base + I2C_CON);

            return -ETIMEDOUT;
        }
        cpu_relax();
    }

    iowrite32(slave_addr, data->base + I2C_SA); // Set slave address
    iowrite32(len, data->base + I2C_CNT); // Number of bytes to write

    i2c_con = ioread32(data->base + I2C_CON);
    i2c_con &=~ 0x2;
    i2c_con |= 0x1; // Start condition
    iowrite32(i2c_con, data->base + I2C_CON);

    // wait ACK from slave
    while ((ioread32(data->base + I2C_IRQSTATUS_RAW)&(I2C_IRQSTATUS_RAW_NACK)) == I2C_IRQSTATUS_RAW_NACK)  // Wait for ACK (addr)
    {
        if (time_after(jiffies, timeout)) {
            dev_err(data->dev, "Timeout wait ACK1, irqsts_raw = 0x%x\n", ioread32(data->base + I2C_IRQSTATUS_RAW));

            i2c_con = ioread32(data->base + I2C_CON);
            i2c_con &=~ 0x1; 
            i2c_con |= 0x2; // Stop condition
            iowrite32(i2c_con, data->base + I2C_CON);

            return -ETIMEDOUT;
        }
        cpu_relax();
    }

    // data is sent in handler
    timeout = jiffies + msecs_to_jiffies(2000);
    while(!(data->is_ended))
    {
        if (time_after(jiffies, timeout)) {
            dev_err(data->dev, "Timeout data->is_ended\n");

            i2c_con = ioread32(data->base + I2C_CON);
            i2c_con &=~ 0x1; 
            i2c_con |= 0x2; // Stop condition
            iowrite32(i2c_con, data->base + I2C_CON);

            return -ETIMEDOUT;
        }
        cpu_relax();
    }

    timeout = jiffies + msecs_to_jiffies(2000);
    while ((ioread32(data->base + I2C_IRQSTATUS_RAW)&(I2C_IRQSTATUS_RAW_ARDY)) != I2C_IRQSTATUS_RAW_ARDY)  // Wait for ARDY (Access ready)
    {
        if (time_after(jiffies, timeout)) {
            dev_err(data->dev, "Timeout ARDY, irqsts_raw = 0x%x\n", ioread32(data->base + I2C_IRQSTATUS_RAW));

            i2c_con = ioread32(data->base + I2C_CON);
            i2c_con &=~ 0x1; 
            i2c_con |= 0x2; // Stop condition
            iowrite32(i2c_con, data->base + I2C_CON);

            return -ETIMEDOUT;
        }
        cpu_relax();
    }

    i2c_con = ioread32(data->base + I2C_CON);
    i2c_con &=~ 0x1; 
    i2c_con |= 0x2; // Stop condition
    iowrite32(i2c_con, data->base + I2C_CON);

    i2c_sts_raw = ioread32(data->base + I2C_IRQSTATUS_RAW);
    i2c_sts_raw |= I2C_IRQSTATUS_RAW_ARDY; // Clear ARDY
    iowrite32(i2c_sts_raw, data->base + I2C_IRQSTATUS_RAW);

    if ((ioread32(data->base + I2C_IRQSTATUS_RAW)&(I2C_IRQSTATUS_RAW_NACK)) == I2C_IRQSTATUS_RAW_NACK){

        // i2c_sts_raw = ioread32(data->base + I2C_IRQSTATUS_RAW);
        // i2c_sts_raw |= I2C_IRQSTATUS_RAW_NACK; // Clear NACK
        // iowrite32(i2c_sts_raw, data->base + I2C_IRQSTATUS_RAW);

        return -1;  // Error
    }

    // i2c_sts_raw = ioread32(data->base + I2C_IRQSTATUS_RAW);
    // i2c_sts_raw |= I2C_IRQSTATUS_RAW_NACK; // Clear NACK
    // iowrite32(i2c_sts_raw, data->base + I2C_IRQSTATUS_RAW);
    return 0;  // Success
}

void Dma_write(struct i2c_device_data *data, uint16_t slave_addr, uint8_t *tx, uint32_t len){
    u32 i2c_sts_raw = 0, i2c_con = 0;
    unsigned long timeout;
    char *kbuf;
    dma_addr_t dma_src_addr;
    struct dma_async_tx_descriptor *tx_desc;
    int ret;


    timeout = jiffies + msecs_to_jiffies(2000);
    while ((ioread32(data->base + I2C_IRQSTATUS_RAW)&I2C_IRQSTATUS_RAW_BB) == I2C_IRQSTATUS_RAW_BB)  // Wait for bus to be free
    {
        if (time_after(jiffies, timeout)) {
            dev_err(data->dev, "Timeout BB, irqsts_raw = 0x%x, cnt = %d\n", ioread32(data->base + I2C_IRQSTATUS_RAW), ioread32(data->base + I2C_CNT));

            i2c_con = ioread32(data->base + I2C_CON);
            i2c_con &=~ 0x1; 
            i2c_con |= 0x2; // Stop condition
            iowrite32(i2c_con, data->base + I2C_CON);

            return;
        }
        cpu_relax();
    }

    iowrite32(slave_addr, data->base + I2C_SA); // Set slave address
    iowrite32(len, data->base + I2C_CNT); // Number of bytes to write

    i2c_con = ioread32(data->base + I2C_CON);
    i2c_con &=~ 0x2;
    i2c_con |= 0x1; // Start condition
    iowrite32(i2c_con, data->base + I2C_CON);

    // wait ACK from slave
    while ((ioread32(data->base + I2C_IRQSTATUS_RAW)&(I2C_IRQSTATUS_RAW_NACK)) == I2C_IRQSTATUS_RAW_NACK)  // Wait for ACK (addr)
    {
        if (time_after(jiffies, timeout)) {
            dev_err(data->dev, "Timeout wait ACK1, irqsts_raw = 0x%x\n", ioread32(data->base + I2C_IRQSTATUS_RAW));

            i2c_con = ioread32(data->base + I2C_CON);
            i2c_con &=~ 0x1; 
            i2c_con |= 0x2; // Stop condition
            iowrite32(i2c_con, data->base + I2C_CON);

            return;
        }
        cpu_relax();
    }

    // ================================= DMA WRITE ===============================
    /* Allocate DMA-coherent buffer */
    kbuf = dma_alloc_coherent(data->dev, len, &dma_src_addr, GFP_KERNEL);
    if (!kbuf) {
        dev_err(data->dev, "Failed to allocate DMA-coherent buffer\n");
        return;
    }
    
    memcpy(kbuf, tx, len);

    reinit_completion(&data->tx_completion);
    tx_desc = dmaengine_prep_slave_single(data->tx_chan, dma_src_addr + 1, len - 1,
                                            DMA_MEM_TO_DEV, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
    if (!tx_desc) {
        dev_err(data->dev, "Failed to prepare TX DMA descriptor\n");
        goto free_buf;
    }

    tx_desc->callback = i2c1_tx_dma_callback;
    tx_desc->callback_param = data;

    dev_info(data->dev, "Submitting TX DMA descriptor\n");
    dmaengine_submit(tx_desc);

    dev_info(data->dev, "Issuing TX DMA pending\n");
    dma_async_issue_pending(data->tx_chan);

    iowrite32(kbuf[0], data->base + I2C_DATA); // send the first byte manually to activate DMA

    ret = wait_for_completion_timeout(&data->tx_completion, msecs_to_jiffies(MS_DELAY));
    if (ret == 0) {
        dev_err(data->dev, "DMA TX timeout\n");
        printk("cnt left = %d\n", ioread32(data->base + I2C_CNT));
        dmaengine_terminate_sync(data->tx_chan);
        goto free_buf;
    }
    // ================================= END DMA WRITE ===============================

    // end of transfer and wait ARDY is set
    timeout = jiffies + msecs_to_jiffies(2000);
    while ((ioread32(data->base + I2C_IRQSTATUS_RAW)&(I2C_IRQSTATUS_RAW_ARDY)) != I2C_IRQSTATUS_RAW_ARDY)  // Wait for ARDY (Access ready)
    {
        if (time_after(jiffies, timeout)) {
            dev_err(data->dev, "Timeout ARDY, irqsts_raw = 0x%x\n", ioread32(data->base + I2C_IRQSTATUS_RAW));

            i2c_con = ioread32(data->base + I2C_CON);
            i2c_con &=~ 0x1; 
            i2c_con |= 0x2; // Stop condition
            iowrite32(i2c_con, data->base + I2C_CON);

            goto free_buf;
        }
        cpu_relax();
    }

    i2c_con = ioread32(data->base + I2C_CON);
    i2c_con &=~ 0x1; 
    i2c_con |= 0x2; // Stop condition
    iowrite32(i2c_con, data->base + I2C_CON);

    i2c_sts_raw = ioread32(data->base + I2C_IRQSTATUS_RAW);
    i2c_sts_raw |= I2C_IRQSTATUS_RAW_ARDY; // Clear ARDY
    iowrite32(i2c_sts_raw, data->base + I2C_IRQSTATUS_RAW);

free_buf:
    dma_free_coherent(data->dev, len, kbuf, dma_src_addr);

    return;
} 

static ssize_t i2c1_master_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    struct i2c_device_data *data = filp->private_data;
    u8 *tx_buf;
    u32 i2c_con = 0;
#if (!DMA_USED)
    int ret = 0;
#endif

    // Allocate buffer for TX data
    tx_buf = kmalloc(count, GFP_KERNEL);
    if (!tx_buf)
        return -ENOMEM;

    if (copy_from_user(tx_buf, buf, count)) {
        kfree(tx_buf);
        return -EFAULT;
    }

    dev_info(data->dev, "Writing %zu bytes: %*ph\n", count, (int)count, tx_buf);

    data->tx = tx_buf;

    i2c_con = ioread32(data->base + I2C_CON);
    i2c_con |= (1u << 15)|(1u << 10)|(1u << 9); // [15] enable i2c module, [MST:10]: master mode, [TRX:9]: MST = 1, TRX = 1, Operating Modes = Master transmitter
    iowrite32(i2c_con, data->base + I2C_CON);

#if (!DMA_USED)
    ret = i2c1_write(data, SLAVE_ADDRESS, tx_buf, count);
    if (ret == -1){
        dev_info(data->dev, "No ACK\n");
    }
    else if (ret == 0) {
        dev_info(data->dev, "Passed + ACK\n");
    }
    else {
        dev_info(data->dev, "Status error\n");
    }
#else
    tx_buf[count - 1] = END_STRING;
    iowrite32((TX_TRIGGER << I2C_BUF_TXTRSH) | I2C_BUF_TXFIFO_CLR, data->base + I2C_BUF); // clear TX FIFO, TX threshold is 1 byte
    Dma_write(data, SLAVE_ADDRESS, tx_buf, count);
#endif

    kfree(tx_buf);
    return count;
}

static const struct file_operations i2c_device_fops = {
    .owner = THIS_MODULE,
    .open = i2c1_master_open,
    .write = i2c1_master_write,
};

void GPIO_init(struct i2c_device_data *data)
{
    iowrite32(0x32, data->base_gpio + 0x95c); // i2c1_scl
    iowrite32(0x32, data->base_gpio + 0x958); // i2c1_sda
}

static int i2c1_configure_dma(struct i2c_device_data *data)
{
    struct dma_slave_config tx_conf = {0};
    int ret;

    if (data->tx_chan) {
        tx_conf.direction = DMA_MEM_TO_DEV;
        tx_conf.dst_addr = (dma_addr_t)(I2C1_BASE + I2C_DATA);
        tx_conf.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE; /* UART uses 8-bit transfers */
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

static int i2c1_probe(struct platform_device *pdev)
{
    struct i2c_device_data *data;
    int ret;
    struct clk *edma_clk;

    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    platform_set_drvdata(pdev, data);

    data->base = ioremap(I2C1_BASE, 0x1000);
    data->base_gpio = ioremap(GPIO_BASE, 0x1000);
    data->dev = &pdev->dev;
    data->is_ended = 0;

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
    dev_info(&pdev->dev, "I2c1 clock rate: %lu Hz\n", clk_get_rate(data->clk));

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

    GPIO_init(data);
    // Initialize hardware
    i2c1_master_init(data);

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
    }
    /* COnfig parameters for DMA */
    data->use_dma = (data->tx_chan);
    if (data->use_dma){
        dev_info(&pdev->dev, "Using DMA for I2C1 transfers\n");
        ret = i2c1_configure_dma(data);
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

    // =================== Create character device
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

    return 0;
}

static int i2c1_remove(struct platform_device *pdev)
{
    struct i2c_device_data *data = platform_get_drvdata(pdev);
    
    if (data->tx_chan)
        dma_release_channel(data->tx_chan);

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
MODULE_DESCRIPTION("Custom I2C Device Driver for BeagleBone Black I2C");
