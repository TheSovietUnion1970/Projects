#include "can.h"
#include "main.h"

static irqreturn_t irqHandler(int irq, void *d){
    struct can_device_data *data = d;
    u32 canctl, Int0ID, can_mctl, can_arb;
    u32 es = 0, dlc, id;
    u32 intpnd_x;
#if(!DMA_USED)
    u32 raw_data1 = 0, raw_data2 = 0;
    u8 i = 0;
#endif

    if (!data) {
        pr_err("NULL data in irqHandler\n");
        return IRQ_NONE;
    }

    Int0ID = ioread32(data->base + CAN_INT);
    es = ioread32(data->base + CAN_ES);
    intpnd_x = ioread32(data->base + CAN_INTPND_X);


#if(!DMA_USED)
    if (Int0ID == 0x8000){ // Error and status interuupt
        if ((es&(1u << 8)) == 1u << 8){
            dev_err(data->dev, "Parity error, es = 0x%x\n", es);
        }
        else if ((es&(1u << 4)) == 1u << 4){
            Reading_received_msg(data, intpnd_x);

            raw_data1 = ioread32(data->base + CAN_IF2DATA);
            raw_data2 = ioread32(data->base + CAN_IF2DATB);
            can_mctl = ioread32(data->base + CAN_IF2MCTL);
            can_arb = ioread32(data->base + CAN_IF2ARB);
            
            dlc = can_mctl&0xF;
            id = can_mctl&CAN_IFxARB_ID;

            printk("Received data[%d]:\n", dlc);
            for (i = 0; i < dlc; i++){
                if (i < 4) {
                    printk("'0x%x'\n", (raw_data1&(0xFF << i*8)) >> i*8);
                }
                else {
                    printk("'0x%x'\n", (raw_data2&(0xFF << i*8)) >> i*8);
                }
            }
            //printk("0x8000 - raw1 = 0x%x, raw2 = 0x%x, dlc = 0x%x, id = 0x%x\n", raw_data1, raw_data2, dlc, id);
        }
        else if ((es&(1u << 3)) == 1u << 3){
            Clear_TxRqst_mctl(data); // clear TxRqst
            Set_MsgNum_cmd(data, intpnd_x); // write updated cleared TxRqst to msg RAM
            printk("IRQ TX\n");
            data->count_many = 0;
        }
    }
    else { // msg object interrupt
        if ((es&(1u << 8)) == 1u << 8){
            dev_err(data->dev, "Parity error, es = 0x%x\n", es);
        }
        else if ((es&(1u << 3)) == 1u << 3){
            Clear_TxRqst_mctl(data); // clear TxRqst
            Set_MsgNum_cmd(data, Int0ID); // write updated cleared TxRqst to msg RAM
            printk("IRQ TX\n");
            data->count_many = 0;
        }
        else if ((es&(1u << 4)) == 1u << 4){
            Reading_received_msg(data, Int0ID);

            raw_data1 = ioread32(data->base + CAN_IF2DATA);
            raw_data2 = ioread32(data->base + CAN_IF2DATB);
            can_mctl = ioread32(data->base + CAN_IF2MCTL);
            can_arb = ioread32(data->base + CAN_IF2ARB);

            id = (can_arb >> 18)&0x7FF;
            dlc = can_mctl&0xF;
            
            printk("Received data[%d]:\n", dlc);
            for (i = 0; i < dlc; i++){
                if (i < 4) {
                    printk("'0x%x'\n", (raw_data1&(0xFF << i*8)) >> i*8);
                }
                else {
                    printk("'0x%x'\n", (raw_data2&(0xFF << (i-4)*8)) >> (i-4)*8);
                }
            }
            data->count_many = 0;
        }
    }
#else
    if (Int0ID == 0x8000){ // Error and status interuupt
        if ((es&(1u << 8)) == 1u << 8){
            dev_err(data->dev, "Parity error, es = 0x%x\n", es);
        }
        else if ((es&(1u << 4)) == 1u << 4){
            Reading_received_msg(data, intpnd_x);
            
            dlc = can_mctl&0xF;
            id = can_mctl&CAN_IFxARB_ID;
        }
        else if ((es&(1u << 3)) == 1u << 3){
            Clear_TxRqst_mctl(data); // clear TxRqst
            Set_MsgNum_cmd(data, intpnd_x); // write updated cleared TxRqst to msg RAM
            printk("IRQ TX\n");
            data->count_many = 0;
        }
    }
    else { // msg object interrupt
        if ((es&(1u << 8)) == 1u << 8){
            dev_err(data->dev, "Parity error, es = 0x%x\n", es);
        }
        else if ((es&(1u << 4)) == 1u << 4){
            Reading_received_msg(data, Int0ID);
            can_arb = ioread32(data->base + CAN_IF2ARB);

            dlc = can_mctl&0xF;
            id = (can_arb >> 18)&0x7FF;
        }
        else if ((es&(1u << 3)) == 1u << 3){
            Clear_TxRqst_mctl(data); // clear TxRqst
            Set_MsgNum_cmd(data, Int0ID); // write updated cleared TxRqst to msg RAM
            printk("IRQ TX\n");
            data->count_many = 0;
        }

        //Disable_all_INT(data);
        //schedule_work(&data->re_request_work);
    }

    #endif
    Clear_rx_flag(data);

    // Clear_MsgNum_cmd(data);

    data->count_many++;
    if (data->count_many > 100){
        printk("Too many interrupts\n");
        data->count_many = 0;

        canctl = ioread32(data->base + CAN_CTL);
        canctl &=~ CAN_CTL_IE0; // disable DCAN0INT
        iowrite32(canctl, data->base + CAN_CTL);
    }


    return IRQ_HANDLED;
}

static void request_work(struct work_struct *work)
{
    struct can_device_data *data = container_of(work, struct can_device_data, re_request_work);
    u32 txx, tx12, mux12;
    u8 i = 0;

    data->dma_buffer_tx[0] = 1;
    data->dma_buffer_tx[1] = 2;
    data->dma_buffer_tx[2] = 3;
    data->dma_buffer_tx[3] = 4;
    
    while (!atomic_read(&data->should_stop)){
        // Config message object
        can0_transmit_obj_Data_Frames_tx(data, ID_sent, data->byte_num_tx, DMA_OBJ_MSG_NUM_TX, 1);

        txx = ioread32(data->base + CAN_TXRQ_X);
        tx12 = ioread32(data->base + CAN_TXRQ12);
        mux12 = ioread32(data->base + CAN_INTMUX12);
    
        //printk("WORK -> txrq = 0x%x, can_mctl = 0x%x\n", ioread32(data->base + CAN_TXRQ12), ioread32(data->base + CAN_IF1MCTL));
        printk("Sent data[%d]:\n", data->byte_num_tx);
        for (i = 0; i < data->byte_num_tx; i++){
            printk("0x%x\n", data->dma_buffer_tx[i]);
        }

        if (data->dma_buffer_tx[0] == 0xFF) data->dma_buffer_tx[0] = 0;
        if (data->dma_buffer_tx[1] == 0xFF) data->dma_buffer_tx[1] = 0;
        if (data->dma_buffer_tx[2] == 0xFF) data->dma_buffer_tx[2] = 0;
        if (data->dma_buffer_tx[3] == 0xFF) data->dma_buffer_tx[3] = 0;

        data->dma_buffer_tx[0]++;
        data->dma_buffer_tx[1]++;
        data->dma_buffer_tx[2]++;
        data->dma_buffer_tx[3]++;
 

        msleep(WORK_DELAY);
    }
}

static const struct file_operations can_device_fops = {
    .owner = THIS_MODULE,
};

static int can0_probe(struct platform_device *pdev){
    struct can_device_data *data;
    int ret;

    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    platform_set_drvdata(pdev, data);

    data->base = ioremap(CAN0_BASE, 0x1000);
    data->base_gpio = ioremap(GPIO_BASE, 0x1000);
    data->base_edma = ioremap(EDMA_BASE, 0x8000);
    data->dev = &pdev->dev;
    data->increment = 0;
    data->count_many = 0;

    /* ===== Clock setup (assuming this part is unchanged) */
    data->clk = devm_clk_get(&pdev->dev, "fck-can0");
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

    // ========== Request IRQ (hwirq 30 maps to swirq x on AM33xx) ==========
    data->irq = platform_get_irq(pdev, 0);
    if (data->irq < 0) {
        dev_err(&pdev->dev, "Failed Formatted: Unable to get IRQ: %d\n", data->irq);
        return data->irq;
    }
    ret = devm_request_irq(&pdev->dev, data->irq, irqHandler, 0, "can0", data);
    if (ret < 0) {
        dev_err(&pdev->dev, "Unable to request IRQ %d: %d\n", data->irq, ret);
        return ret;
    }
    dev_info(&pdev->dev, "IRQ num = %d\n", data->irq);

    // ====== Can init
    GPIO_init(data);
    can0_init_tx(data);
    can0_init_rx(data);
    can0_INT_init(data);
#if(DMA_USED)
    can0_DMA_init(data); // for both IF1 and IF2
    DMA_probe(data);
#endif

    // ===== Create character device
    ret = alloc_chrdev_region(&data->dev_num, 0, 1, DRIVER_NAME);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to allocate chrdev region: %d\n", ret);
        return ret;
    }

    // ====== Create device character
    cdev_init(&data->cdev, &can_device_fops);
    data->cdev.owner = THIS_MODULE;
    ret = cdev_add(&data->cdev, data->dev_num, 1);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to add cdev: %d\n", ret);
        //unregister_chrdev_region(&data->dev_num, 1);
        iounmap(data->base);
        return ret;
    }

    data->class = class_create(THIS_MODULE, "can0_class");
    if (IS_ERR(data->class)) {
        dev_err(&pdev->dev, "Failed to create class: %ld\n", PTR_ERR(data->class));
        cdev_del(&data->cdev);
        //unregister_chrdev_region(&data->dev_num, 1);
        iounmap(data->base);
        return PTR_ERR(data->class);
    }

    data->dev = device_create(data->class, &pdev->dev, data->dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(data->dev)) {
        dev_err(&pdev->dev, "Failed to create device: %ld\n", PTR_ERR(data->dev));
        class_destroy(data->class);
        cdev_del(&data->cdev);
        //unregister_chrdev_region(&data->dev_num, 1);
        iounmap(data->base);
        return PTR_ERR(data->dev);
    }

    dev_info(&pdev->dev, "Created /dev/%s\n", DEVICE_NAME);
    
    /* out of init mode */
    can0_enter_mode(data);

    /* Create a work */
    INIT_WORK(&data->re_request_work, request_work);
#if (TEST_TX_DATA)
    schedule_work(&data->re_request_work);
#endif

    return 0;
}

static int can0_remove(struct platform_device *pdev){
    struct can_device_data *data = platform_get_drvdata(pdev);

    dev_info(data->dev, "Removed\n");

    /* For tx */
    atomic_set(&data->should_stop, 1); // Set to 1 (true)
    smp_mb(); // Memory barrier to ensure should_stop is visible
    cancel_work_sync(&data->re_request_work);

#if(DMA_USED)
    if (data->if1_chan){
        dmaengine_terminate_sync(data->if1_chan);
        dma_release_channel(data->if1_chan);
    }

    if (data->if2_chan){
        dma_free_coherent(data->dev, MAX_BUFFER_LEN, data->dma_buffer_rx, data->dma_buffer_phys);
        dmaengine_terminate_sync(data->if2_chan);
        dma_release_channel(data->if2_chan);
    }
#endif

    /* CAN is done */
    can0_enter_done(data);

    //clk_disable_unprepare(data->clk);

    if (data->dev)
        device_destroy(data->class, data->dev_num);
    if (data->class)
        class_destroy(data->class);
    cdev_del(&data->cdev);

    devm_kfree(&pdev->dev, data);

    return 0;
}

static const struct of_device_id can_device_of_match[] = {
    { .compatible = "can0-based" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, can_device_of_match);

static struct platform_driver can_device_driver = {
    .probe = can0_probe,
    .remove = can0_remove,
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = can_device_of_match,
    },
};

module_platform_driver(can_device_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Soviet");
MODULE_DESCRIPTION("Custom CAN Device Driver for BeagleBone Black can0");
