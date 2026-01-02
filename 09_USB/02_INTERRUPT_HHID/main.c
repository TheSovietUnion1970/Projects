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
#include "usb1.h"
#include "main.h"

#define DRIVER_NAME "usb1_driver"
#define DEVICE_NAME "usb1"

static void re_request_irq_work(struct work_struct *work){
    struct usb_device_data *data = container_of(work, struct usb_device_data, re_request_work);
    int ret;

    ret = USB1_IN_Phase_Interrupt(data, 1, 0x1);

}

static int usb1_open(struct inode *inode, struct file *file){
    struct usb_device_data *data = container_of(inode->i_cdev, struct usb_device_data, cdev);
    file->private_data = data;
    return 0;
}
static ssize_t usb1_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    struct usb_device_data *data = filp->private_data;

    //ret = USB1_GetDesc_Transfer(data);
    schedule_work(&data->re_request_work);

    return count;
}
static const struct file_operations usb_device_fops = {
    .owner = THIS_MODULE,
    .open = usb1_open,
    .write = usb1_write,
};

static int usb1_probe(struct platform_device *pdev)
{
    struct usb_device_data *data;
    int ret;

    //dev_info(&pdev->dev, "Probed\n");
    printk("probed\n");

    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    platform_set_drvdata(pdev, data);

    data->dev = &pdev->dev;
    data->base_usbss = ioremap(BASE_USBSS, 0x1000);
    data->base_usb1ctl = ioremap(BASE_USB1CTL, 0x200);
    data->base_usb1phy = ioremap(BASE_USB1PHY, 0x100);
    data->base_usb1core = ioremap(BASE_USB1CORE, 0x400);
#if (!INDEX_USED)
    data->base_usb1ep0 = ioremap(USB1EP0_base, 0x10); // 16 bytes
#else
    data->base_usb1ep0 = ioremap(BASE_USB1CORE + 0x10, 0x10); // 16 bytes
#endif
    data->base_con_usb1ctrl1 = ioremap(CONTROL_MODULE, 0x1000);

    /* ===== Clock setup (assuming this part is unchanged) */
    data->clk = devm_clk_get(&pdev->dev, "fck-usb1");
    if (IS_ERR(data->clk)) {
        dev_err(&pdev->dev, "Failed to get clock: %ld\n", PTR_ERR(data->clk));
        return PTR_ERR(data->clk);
    }
    ret = clk_prepare_enable(data->clk);
    if (ret) {
        dev_err(&pdev->dev, "Failed to enable clock: %d\n", ret);
        return ret;
    }
    dev_info(&pdev->dev, "can0 clock rate: %lu Hz\n", clk_get_rate(data->clk));

    // ========== Request IRQ (hwirq 19 maps to swirq x on AM33xx) ==========
    data->irq1 = platform_get_irq(pdev, 0);
    if (data->irq1 < 0) {
        dev_err(&pdev->dev, "Failed Formatted: Unable to get IRQ: %d\n", data->irq1);
        return data->irq1;
    }

    //ret = request_irq(data->irq1, USB1_handler, IRQF_SHARED, "usb1", data);
    ret = devm_request_irq(&pdev->dev, data->irq1, USB1_handler, 0, "usb1", data);
    if (ret < 0) {
        dev_err(&pdev->dev, "Unable to request IRQ %d: %d\n", data->irq1, ret);
        return ret;
    }
    dev_info(&pdev->dev, "IRQ1 num = %d\n", data->irq1);

    // ===== Create character device
    ret = alloc_chrdev_region(&data->dev_num, 0, 1, DRIVER_NAME);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to allocate chrdev region: %d\n", ret);
        return ret;
    }

    cdev_init(&data->cdev, &usb_device_fops);
    data->cdev.owner = THIS_MODULE;
    ret = cdev_add(&data->cdev, data->dev_num, 1);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to add cdev: %d\n", ret);
        //unregister_chrdev_region(&data->dev_num, 1);
        iounmap(data->base_usbss);
        iounmap(data->base_usb1ctl);
        iounmap(data->base_usb1phy);
        iounmap(data->base_usb1core);
        return ret;
    }

    data->class = class_create(THIS_MODULE, "can0_class");
    if (IS_ERR(data->class)) {
        dev_err(&pdev->dev, "Failed to create class: %ld\n", PTR_ERR(data->class));
        cdev_del(&data->cdev);
        //unregister_chrdev_region(&data->dev_num, 1);
        iounmap(data->base_usbss);
        iounmap(data->base_usb1ctl);
        iounmap(data->base_usb1phy);
        iounmap(data->base_usb1core);
        return PTR_ERR(data->class);
    }

    data->dev = device_create(data->class, &pdev->dev, data->dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(data->dev)) {
        dev_err(&pdev->dev, "Failed to create device: %ld\n", PTR_ERR(data->dev));
        class_destroy(data->class);
        cdev_del(&data->cdev);
        //unregister_chrdev_region(&data->dev_num, 1);
        iounmap(data->base_usbss);
        iounmap(data->base_usb1ctl);
        iounmap(data->base_usb1phy);
        iounmap(data->base_usb1core);
        return PTR_ERR(data->dev);
    }

    dev_info(&pdev->dev, "Created /dev/%s\n", DEVICE_NAME);

    // ===== USB1 init
    USB1_reset(data);
    PHY1_init(data);
    USB1_init(data);
    data->isEnd = 0;
    data->RX_index = 0;

    //musb_init_controller_V(data);

    INIT_WORK(&data->re_request_work, re_request_irq_work);
    atomic_set(&data->should_stop, 0); // Initialize to 0 (false)

    msleep(2000);
    /* Reset */
    USB1_Reset_Speed(data);

    /* Getting descriptor */
    ret = USB1_GetDesc_Transfer(data);
    if (ret < 0) {
        printk("Fail USB1_GetDesc_Transfer\n");
    }
    else {
#if (PRINT_DATA)
        USB1_Print_DeviceDescriptor(data);
        USB1_Print_DeviceDescriptor2(data);
        USB1_Print_DeviceDescriptorIf0(data);
#endif
    }

    schedule_work(&data->re_request_work);

    return 0;
}

static int usb1_remove(struct platform_device *pdev)
{
    struct usb_device_data *data = platform_get_drvdata(pdev);

    atomic_set(&data->should_stop, 1); // Set to 1 (true)
    smp_mb(); // Memory barrier to ensure should_stop is visible

    cancel_work_sync(&data->re_request_work);

    USB1_exit(data);


    // NEW: Disable clock (manual, since enable was manual)
    clk_disable_unprepare(data->clk);

    iounmap(data->base_usbss);
    iounmap(data->base_usb1ctl);
    iounmap(data->base_usb1phy);
    iounmap(data->base_usb1core);
    iounmap(data->base_usb1ep0);
    iounmap(data->base_con_usb1ctrl1);

    if (data->dev)
        device_destroy(data->class, data->dev_num);
    if (data->class)
        class_destroy(data->class);
    cdev_del(&data->cdev);

    return 0;
}

static const struct of_device_id usb_device_of_match[] = {
    { .compatible = "usb1-based" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, usb_device_of_match);

static struct platform_driver usb_device_driver = {
    .probe = usb1_probe,
    .remove = usb1_remove,
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = usb_device_of_match,
    },
};

module_platform_driver(usb_device_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Soviet");
MODULE_DESCRIPTION("Custom USB1 Device Driver for BeagleBone Black USB1");
