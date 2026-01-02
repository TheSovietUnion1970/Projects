#include <linux/module.h>
#include <linux/fs.h> // alloc_chrdev_region
#include <linux/pci.h> // ioremap
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/pm_runtime.h>
#include "mdio.h"
#include "cpsw.h"
#include "cpdma.h"
#include "debug.h"

#define DRIVER_NAME "ether0_driver"
#define DEVICE_NAME "ether0"
#define DEVICE_CLASS "ether0_class"

static int ether_open(struct inode *inode, struct file *file){
    struct ether_device_data *data = container_of(inode->i_cdev, struct ether_device_data, cdev);
    file->private_data = data;
    return 0;
}
static ssize_t ether_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    struct ether_device_data *data = filp->private_data;

    //ret = ether_GetDesc_Transfer(data);
    schedule_work(&data->re_request_work);

    return count;
}
static const struct file_operations ether_device_fops = {
    .owner = THIS_MODULE,
    .open = ether_open,
    .write = ether_write,
};

static int ether_probe(struct platform_device *pdev)
{
    struct ether_device_data *data;
    int ret;

    phys_addr_t cpsw_phys;

    //dev_info(&pdev->dev, "Probed\n");
    printk("probed\n");

    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    platform_set_drvdata(pdev, data);
    data->dev = &pdev->dev;
    dev_set_drvdata(data->dev, data);


    data->base_ctrmod = ioremap(CTRMOD_BASE, 0x1000);
    data->base_clk = ioremap(CLK_BASE, 0x1000);

    data->base_cpsw = ioremap(CPSW_BASE, 0x100);
    cpsw_phys = virt_to_phys(data->base_cpsw);

    data->base_ale = ioremap(ALE_BASE, 0x200);
    data->base_cpsw_sl = ioremap(CPSW_SL_BASE, 0x200);
    data->base_port0 = ioremap(PORT0_BASE, 0x100);
    data->base_port1 = ioremap(PORT1_BASE, 0x100);
    data->base_port2 = ioremap(PORT2_BASE, 0x100);
    data->base_wr = ioremap(CPSW_WR_BASE, 0x100);

    data->base_cpdma = ioremap(CPDMA_BASE, 0x100);

    data->base_txhdp = ioremap(TXHDP_BASE, 0x100);
    data->base_rxhdp = data->base_txhdp + 0x20;
    data->base_txcp = data->base_txhdp + 0x40;
    data->base_rxcp = data->base_txhdp + 0x60;

    data->base_mdio = ioremap(MDIO_BASE, 0x1000);

    // data->desc_dma_tx = ioremap(CPPIRAM_BASE, CPSW_BD_RAM_SIZE);


    /* ===== Clock setup (assuming this part is unchanged) */

    // ========== Request IRQ ==========
    data->rx_thresh_irq = platform_get_irq(pdev, 0);
    if (data->rx_thresh_irq < 0) {
        dev_err(&pdev->dev, "Failed Formatted: Unable to get IRQ: %d\n", data->rx_thresh_irq);
        return data->rx_thresh_irq;
    }
    ret = devm_request_irq(&pdev->dev, data->rx_thresh_irq, rx_thresh_handler, 0, "rx_thresh_handler", data);
    if (ret < 0) {
        printk("Fail request rx_thresh_handler\n");
        return -1;
    }

    data->rx_irq = platform_get_irq(pdev, 1);
    if (data->rx_irq < 0) {
        dev_err(&pdev->dev, "Failed Formatted: Unable to get IRQ: %d\n", data->rx_irq);
        return data->rx_irq;
    }
    ret = devm_request_irq(&pdev->dev, data->rx_irq, rx_handler, 0, "rx_handler", data);
    if (ret < 0) {
        printk("Fail request rx_handler\n");
        return -1;
    }

    data->tx_irq = platform_get_irq(pdev, 2);
    if (data->tx_irq < 0) {
        dev_err(&pdev->dev, "Failed Formatted: Unable to get IRQ: %d\n", data->tx_irq);
        return data->tx_irq;
    }
    ret = devm_request_irq(&pdev->dev, data->tx_irq, tx_handler, 0, "tx_handler", data);
    if (ret < 0) {
        printk("Fail request tx_handler\n");
        return -1;
    }

    data->misc_irq = platform_get_irq(pdev, 3);
    if (data->misc_irq < 0) {
        dev_err(&pdev->dev, "Failed Formatted: Unable to get IRQ: %d\n", data->misc_irq);
        return data->misc_irq;
    }
    ret = devm_request_irq(&pdev->dev, data->misc_irq, misc_handler, 0, "misc_handler", data);
    if (ret < 0) {
        printk("Fail request misc_handler\n");
        return -1;
    }

    // ===== Create character device
    ret = alloc_chrdev_region(&data->dev_num, 0, 1, DRIVER_NAME);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to allocate chrdev region: %d\n", ret);
        return ret;
    }

    cdev_init(&data->cdev, &ether_device_fops);
    data->cdev.owner = THIS_MODULE;
    ret = cdev_add(&data->cdev, data->dev_num, 1);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to add cdev: %d\n", ret);
        // iounmap(data->base_etherss);
        // iounmap(data->base_etherctl);
        // iounmap(data->base_etherphy);
        // iounmap(data->base_ethercore);
        return ret;
    }

    data->class = class_create(THIS_MODULE, DEVICE_CLASS);
    if (IS_ERR(data->class)) {
        dev_err(&pdev->dev, "Failed to create class: %ld\n", PTR_ERR(data->class));
        cdev_del(&data->cdev);
        // iounmap(data->base_etherss);
        // iounmap(data->base_etherctl);
        // iounmap(data->base_etherphy);
        // iounmap(data->base_ethercore);
        return PTR_ERR(data->class);
    }

    data->chardev = device_create(data->class, &pdev->dev, data->dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(data->chardev)) {
        dev_err(&pdev->dev, "Failed to create device: %ld\n", PTR_ERR(data->chardev));
        class_destroy(data->class);
        cdev_del(&data->cdev);
        // iounmap(data->base_etherss);
        // iounmap(data->base_etherctl);
        // iounmap(data->base_etherphy);
        // iounmap(data->base_ethercore);
        return PTR_ERR(data->chardev);
    }

    dev_info(&pdev->dev, "Created /dev/%s, cpsw_phys = 0x%x\n", DEVICE_NAME, cpsw_phys);

    data->tx_dma_channel = TX_DMA_CHANNEL;
    data->rx_dma_channel = RX_DMA_CHANNEL;

    gmii_sel_init(data);
    ret = clock_init(data);
    if (ret == 0){
        ret = ether_mdio_init(data);   
        if (ret < 0) printk("Fail ether_mdio_init\n");
    }
    if (ret == 0){
        ret = cpsw_init(data); 
        if (ret < 0) printk("Fail cpsw_init\n");
    }


    return 0;
}

static int ether_remove(struct platform_device *pdev)
{
    struct ether_device_data *data = platform_get_drvdata(pdev);

    printk(KERN_INFO "ether_remove called\n");

    cpsw_remove(data);

    if (data->dev) {
        device_destroy(data->class, data->dev_num);
        data->dev = NULL;
    }

    if (data->class) {
        class_destroy(data->class);
        data->class = NULL;
    }

    cdev_del(&data->cdev);
    unregister_chrdev_region(data->dev_num, 1);

    /* unmap */
    iounmap(data->base_ctrmod);
    iounmap(data->base_clk);
    iounmap(data->base_cpsw);
    iounmap(data->base_ale);
    iounmap(data->base_cpsw_sl);
    iounmap(data->base_mdio);
    iounmap(data->base_port0);
    iounmap(data->base_port1);
    iounmap(data->base_port2);

    iounmap(data->base_cpdma);
    iounmap(data->base_txhdp);


    return 0;
}
static const struct of_device_id ether_device_of_match[] = {
    { .compatible = "ether-based" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ether_device_of_match);

static struct platform_driver ether_device_driver = {
    .probe = ether_probe,
    .remove = ether_remove,
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = ether_device_of_match,
    },
};

module_platform_driver(ether_device_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Soviet");
MODULE_DESCRIPTION("Custom ether Device Driver for BeagleBone Black ether");
