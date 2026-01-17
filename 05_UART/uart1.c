#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/ctype.h> /* Add this for isprint */
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include "main.h"

/* AM335x UART register offsets */
#define UART_THR 0x00 /* Transmit Holding Register */
#define UART_RHR 0x00 /* Receive Holding Register */
#define UART_LSR 0x14 /* Line Status Register */
#define UART_IER 0x04 /* Interrupt Enable Register */
#define UART_FCR 0x08 /* FIFO Control Register */
#define UART_LCR 0x0C /* Line Control Register */
#define UART_MCR 0x10 /* Modem Control Register */
#define UART_DLL 0x00 /* Divisor Latch Low */
#define UART_DLH 0x04 /* Divisor Latch High */
#define UART_SYSC 0x54 /* System Configuration Register */
#define UART_SYSS 0x58 /* System Status Register */
#define UART_MDR1 0x20 /* Mode Definition Register 1 */
#define UART_IIR 0x08 /* Interrupt Identification Register */

#define UART_SCR 0x40 /* SCR */

/* Line Status Register bits */
#define UART_LSR_TXFIFOE (1 << 5) /* Transmit FIFO empty */
#define UART_LSR_DR      (1 << 0) /* Data Ready (Receive FIFO has data) */

/* Interrupt Enable Register bits */
#define UART_IER_RHR_IT  (1 << 0) /* Received Data Available */

/* Interrupt Enable Register bits */
#define UART_IIR_IT_PENDING (1 << 0) /* Interrupt pending (0 = pending) */
#define UART_IIR_CTO_IT  (0x6 << 1) /* Character Timeout (priority 2) */
#define UART_IIR_RLS_IT  (0x3 << 1) /* Receiver Line Status (priority 1) */
#define UART_IIR_RHR_IT  (0x2 << 1) /* Received Data Available (priority 2) */
#define UART_IIR_THR_IT  (0x1 << 1) /* Transmitter Holding Register Empty (priority 3) */
#define UART_IIR_MSI     (0x0 << 1) /* Modem Status (priority 4) */

#define MAX_NUM_INTERUPTS 50
#define UART_BUFFER_SIZE 256

#define EDMA_BASE 0x49000000
#define EDMA_DCHMAP 0x0100

/* Device structure */
struct bbb_uart {
    void __iomem *base;     /* UART registers */
    void __iomem *sysc;     /* System control register */
    void __iomem *syss;     /* System status register */
    dev_t devno;
    struct cdev cdev;
    struct device *dev;
    struct clk *clk;
    struct class *class;    /* Class for device */
    int irq;
    unsigned int count;
    unsigned long irqFlags;
    unsigned int Tx_Rx_count;
    char RX_buffer[UART_BUFFER_SIZE];
};

static irqreturn_t irqHandler(int irq, void *d)
{
    struct bbb_uart *uart = d;
    uart->count++;

    // Read val in UART_RHR to clear IIR from 0xcc to 0xc1
    //printk("int iir -> '%x'\n", ioread32(uart->base + UART_IIR));
    uart->RX_buffer[uart->Tx_Rx_count++] = ioread32(uart->base + UART_RHR);

    if (uart->RX_buffer[uart->Tx_Rx_count - 1] == '\n'){
        uart->RX_buffer[uart->Tx_Rx_count - 1] = 0;
        printk("[CPU] RX data[%d]: '%s'\n", uart->Tx_Rx_count, uart->RX_buffer);

        uart->Tx_Rx_count = 0;
        uart->count = 0;
    }


    if (uart->count > MAX_NUM_INTERUPTS){
        printk("Too many interrupts - IIR=0x%x, LSR=0x%x\n", ioread32(uart->base + UART_IIR), ioread32(uart->base + UART_LSR));
        iowrite32(0x0, uart->base + UART_IER);
        uart->count = 0;
    }

    return IRQ_HANDLED;
}

/* File operations */
static int bbb_uart_open(struct inode *inode, struct file *filp)
{
    struct bbb_uart *uart = container_of(inode->i_cdev, struct bbb_uart, cdev);
    filp->private_data = uart;
    return 0;
}

static int bbb_uart_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static ssize_t bbb_uart_write(struct file *filp, const char __user *buf,
                              size_t count, loff_t *ppos)
{
    struct bbb_uart *uart = filp->private_data;
    char *kbuf;
    int i;

    /* Re-enable RHR IT */
    iowrite32(UART_IER_RHR_IT, uart->base + UART_IER);

    uart->count = 0;
    uart->Tx_Rx_count = 0;

    kbuf = kmalloc(count, GFP_KERNEL);
    if (!kbuf)
        return -ENOMEM;

    if (copy_from_user(kbuf, buf, count)) {
        kfree(kbuf);
        return -EFAULT;
    }
    //printk("TX - count = %d\n", count);

    for (i = 0; i < count; i++) {
        unsigned long timeout = jiffies + msecs_to_jiffies(1000);
        u32 lsr;
        while (1) {
            lsr = ioread32(uart->base + UART_LSR);
            //dev_info(uart->dev, "Write: LSR=0x%x\n", lsr);  // Log LSR value
            if (lsr & UART_LSR_TXFIFOE)
                break;
            if (time_after(jiffies, timeout)) {
                dev_err(uart->dev, "TX timeout, LSR=0x%x\n", lsr);
                kfree(kbuf);
                return -ETIMEDOUT;
            }
            cpu_relax();
        }
        // dev_info(uart->dev, "Before writting, UART_THR = '%c'\n", ioread32(uart->base + UART_THR));  
        iowrite32(kbuf[i], uart->base + UART_THR);
        //dev_info(uart->dev, "After writting, UART_THR = '%c'\n", ioread32(uart->base + UART_THR));  
    }
    printk("[CPU] TX data[%d] - Done\n", count);

    kfree(kbuf);
    return count;
}

static ssize_t bbb_uart_read(struct file *filp, char __user *buf,
                             size_t count, loff_t *ppos)
{
    struct bbb_uart *uart = filp->private_data;
    unsigned int bytes_read = 0;

    // copy_to_user will print the buf in the terminal
    if (copy_to_user(buf, uart->RX_buffer, uart->Tx_Rx_count)) {
        return -EFAULT;
    }

    /* Save how many bytes we are returning */
    bytes_read = uart->Tx_Rx_count;

    /* Reset buffer and counters */
    memset(uart->RX_buffer, 0, sizeof(uart->RX_buffer));
    uart->Tx_Rx_count = 0;

    // return correct bytes_read to avoid the next automatic read operation
    return bytes_read;
}

static const struct file_operations bbb_uart_fops = {
    .owner = THIS_MODULE,
    .open = bbb_uart_open,
    .release = bbb_uart_release,
    .write = bbb_uart_write,
    .read = bbb_uart_read,
};

static void bbb_uart_init_hw(struct bbb_uart *uart)
{
    u32 val;

    /* Reset the UART via SYSC register */
    iowrite32(0x2, uart->sysc);  // Example: soft reset
#if (PRINT_DEBUG)
    printk("Before while, val = %du\n", ioread32(uart->syss));
#endif
    do {
        val = ioread32(uart->syss);
    } while (!(val & 0x1));  // Wait for reset completion
#if (PRINT_DEBUG)
    printk("After while\n");
    dev_info(uart->dev, "Soft reset completed\n");
#endif

    /* 2. Disable UART */
    iowrite32(0x7, uart->base + UART_MDR1);  /* Disable UART */

    /* 3. Configure baud rate (e.g., 115200 with 48MHz clock) */
    iowrite32(0xBF, uart->base + UART_LCR);  /* Access DLL/DLH */
    iowrite32(26 & 0xFF, uart->base + UART_DLL);  /* 115200 baud */
    iowrite32(0, uart->base + UART_DLH);
    iowrite32(0x03, uart->base + UART_LCR);  /* 8N1 */

    /* 4. Enable and configure FIFOs */
    iowrite32(0x07, uart->base + UART_FCR);  /* Enable FIFO, clear TX/RX */

    /* DMA */
    iowrite32(0x07, uart->base + UART_SCR);

    /* 5. Enable UART */
    iowrite32(0x0, uart->base + UART_MDR1);  /* UART 16x mode */

    usleep_range(1000, 2000);  /* 1ms */

    /* 6. Enable loopback */
    iowrite32(0x00, uart->base + UART_MCR);  /* Not Set loopback */

    /* Enable RHR interrupts */
    iowrite32(UART_IER_RHR_IT, uart->base + UART_IER);
}

static int bbb_uart_probe(struct platform_device *pdev)
{
    struct bbb_uart *uart;
    struct resource *res;
    int ret;
#if (PRINT_DEBUG)
    dev_info(&pdev->dev, "Probing device: %s, Node: %s\n", 
             pdev->name, pdev->dev.of_node->full_name);
#endif
    uart = devm_kzalloc(&pdev->dev, sizeof(*uart), GFP_KERNEL);
    if (!uart)
        return -ENOMEM;

    uart->count = 0;
    uart->dev = &pdev->dev;

    /* Get the single memory resource */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev, "No memory resource\n");
        return -ENODEV;
    }
#if (PRINT_DEBUG)
    dev_info(&pdev->dev, "Memory resource start: 0x%x, end: 0x%x, flags: 0x%lx\n",
             res->start, res->end, res->flags);
#endif
    /* Map the entire region */
    uart->base = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(uart->base)) {
        dev_err(&pdev->dev, "Failed to map memory resource: %ld\n", 
                PTR_ERR(uart->base));
        return PTR_ERR(uart->base);
    }


    uart->irq = platform_get_irq(pdev, 0);
	if (uart->irq < 0) {
		dev_err(&pdev->dev, "%s: unable to get IRQ\n", __func__);
		return uart->irq;
	}

    ret = devm_request_irq(&pdev->dev, uart->irq, irqHandler, 0, "bbb-uart1", uart);
    if (ret < 0) 
    {
        dev_err(&pdev->dev, "%s: unable to request IRQ %d (%d)\n", __func__, uart->irq, ret);
        return ret;
    }



    /* Set offsets for control registers */
    uart->sysc = uart->base + 0x54;  // SYSC register at offset 0x54
    uart->syss = uart->base + 0x58;  // SYSS register at offset 0x58

    /* Clock setup (assuming this part is unchanged) */
    uart->clk = devm_clk_get(&pdev->dev, "fck_uart1");
    if (IS_ERR(uart->clk)) {
        dev_err(&pdev->dev, "Failed to get clock: %ld\n", PTR_ERR(uart->clk));
        return PTR_ERR(uart->clk);
    }
    ret = clk_prepare_enable(uart->clk);
    if (ret) {
        dev_err(&pdev->dev, "Failed to enable clock: %d\n", ret);
        return ret;
    }
    dev_info(&pdev->dev, "Uart clock rate: %lu Hz\n", clk_get_rate(uart->clk));

    /* Remaining initialization (e.g., PM, chrdev) */
    pm_runtime_enable(&pdev->dev);
    ret = pm_runtime_get_sync(&pdev->dev);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to enable PM: %d\n", ret);
        clk_disable_unprepare(uart->clk);
        return ret;
    }

    /* ===================== Create a charecter device ======================== */
    ret = alloc_chrdev_region(&uart->devno, 0, 1, "bbb_uart1");
    if (ret) {
        dev_err(&pdev->dev, "Failed to allocate chrdev: %d\n", ret);
        pm_runtime_put_sync(&pdev->dev);
        clk_disable_unprepare(uart->clk);
        return ret;
    }
#if (PRINT_DEBUG)
    dev_info(&pdev->dev, "Allocated chrdev major: %d\n", MAJOR(uart->devno));
#endif
    cdev_init(&uart->cdev, &bbb_uart_fops);
    uart->cdev.owner = THIS_MODULE;
    ret = cdev_add(&uart->cdev, uart->devno, 1);
    if (ret) {
        dev_err(&pdev->dev, "Failed to add cdev: %d\n", ret);
        unregister_chrdev_region(uart->devno, 1);
        pm_runtime_put_sync(&pdev->dev);
        clk_disable_unprepare(uart->clk);
        return ret;
    }
    dev_info(&pdev->dev, "Added cdev successfully\n");

    /* Create device class */
    uart->class = class_create(THIS_MODULE, "bbb_uart1_class");
    if (IS_ERR(uart->class)) {
        dev_err(&pdev->dev, "Failed to create class: %ld\n", PTR_ERR(uart->class));
        cdev_del(&uart->cdev);
        unregister_chrdev_region(uart->devno, 1);
        pm_runtime_put_sync(&pdev->dev);
        clk_disable_unprepare(uart->clk);
        return PTR_ERR(uart->class);
    }

    /* Create device node /dev/uart1 */
    uart->dev = device_create(uart->class, &pdev->dev, uart->devno, NULL, "uart1");
    if (IS_ERR(uart->dev)) {
        dev_err(&pdev->dev, "Failed to create device: %ld\n", PTR_ERR(uart->dev));
        class_destroy(uart->class);
        cdev_del(&uart->cdev);
        unregister_chrdev_region(uart->devno, 1);
        pm_runtime_put_sync(&pdev->dev);
        clk_disable_unprepare(uart->clk);
        return PTR_ERR(uart->dev);
    }
    dev_info(&pdev->dev, "Created device /dev/uart1\n");


    bbb_uart_init_hw(uart);  // Ensure this uses uart->sysc and uart->syss correctly
    dev_info(&pdev->dev, "Done - bbb_uart_init_hw\n");

    uart->dev = &pdev->dev;
    platform_set_drvdata(pdev, uart);

    dev_info(&pdev->dev, "Uart1 initialized\n");
#if (PRINT_DEBUG)
    dev_info(uart->dev, "LCR=0x%x, FCR=0x%x, MDR1=0x%x, MCR=0x%x, DLL = 0x%x\n",
            ioread32(uart->base + UART_LCR),
            ioread32(uart->base + UART_FCR),
            ioread32(uart->base + UART_MDR1),
            ioread32(uart->base + UART_MCR),
            ioread32(uart->base + UART_DLL));
#endif
    return 0;
}
/* Remove function */
static int bbb_uart_remove(struct platform_device *pdev)
{
    struct bbb_uart *uart = platform_get_drvdata(pdev);

    device_destroy(uart->class, uart->devno);
    class_destroy(uart->class);

    cdev_del(&uart->cdev);
    unregister_chrdev_region(uart->devno, 1);
    pm_runtime_put_sync(&pdev->dev);
    pm_runtime_disable(&pdev->dev);
    dev_info(&pdev->dev, "Uart1 removed\n");
    return 0;
}

/* Device tree match table */
static const struct of_device_id bbb_uart_of_match[] = {
    { .compatible = "uart1-based" },
    { }
};
MODULE_DEVICE_TABLE(of, bbb_uart_of_match);

/* Platform driver */
static struct platform_driver bbb_uart1 = {
    .probe = bbb_uart_probe,
    .remove = bbb_uart_remove,
    .driver = {
        .name = "bbb_uart1",
        .of_match_table = bbb_uart_of_match,
    },
};
module_platform_driver(bbb_uart1);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vinh");
MODULE_DESCRIPTION("BeagleBone Black Uart1 Driver");