#include "can.h"
#include "main.h"

static int wait_register_update(struct can_device_data *data, u16 offset, u16 bit_offset, u8 bit_val, u16 delay_ms, u8* name_register){
    unsigned long timeout;
    timeout = jiffies + msecs_to_jiffies(delay_ms);

    while ((ioread32(data->base + offset)&(1u << bit_offset)) != (bit_val << bit_offset))  // Wait register updated
    {
        if (time_after(jiffies, timeout)) {
            dev_err(data->dev, "Timeout %s\n", name_register);
            return -ETIMEDOUT;
        }
        cpu_relax();
    } 

    return 0;
}

static void can0_bit_timing(struct can_device_data *data, u32 can_clk, u32 bitrate){
    u32 val = 0;

    u8 SJW = 1;
    u8 TSeg1 = 4;
    u8 TSeg2 = 3;

    /* CAN_CLK is 24MHz */
    u16 BRP = can_clk/(bitrate*(SJW + TSeg1 + TSeg2)); // 5

    BRP = BRP - 1;
    SJW = SJW - 1;
    TSeg1 = TSeg1 - 1;
    TSeg2 = TSeg2 - 1;

    val = ((BRP)&0x3F) | ((SJW << 6)&0xC0) | ((TSeg1 << 8)&0xF00) | ((TSeg2 << 12)&0x7000);

    iowrite32(val, data->base + CAN_BTR);

}

#if(DMA_USED)
static void can0_DMAactive_IF1(struct can_device_data *data){
    u32 can_cmd = 0;

    can_cmd = ioread32(data->base + CAN_IF1CMD);
    can_cmd |= 1u << 14; // DMAactive
    iowrite32(can_cmd, data->base + CAN_IF1CMD);
    
    wait_register_update(data, CAN_IF1CMD, CAN_IFxCMD_Busy_OFFSET, 0, MS_DELAY, "Busy bit");
}
void print_DMA_reg(struct can_device_data *data){
    u32 param_addr = 0x4820;

    printk(" ===== >> DMA reg ========\n");
    printk("DMA_EESRH = 0x%x\n", ioread32(data->base_edma + DMA_IERH));
    printk("DMA_IPRH = 0x%x\n", ioread32(data->base_edma + DMA_IPRH));

    printk("opt = 0x%x\n", ioread32(data->base_edma + param_addr + 0));
    printk("src = 0x%x\n", ioread32(data->base_edma + param_addr + 0x4));
    printk(" ===== >> DMA end ========\n");
}
#endif /* DMA_USED */

void can0_transmit_obj_Data_Frames_tx(struct can_device_data *data, u8 can_id, u8 size, u8 msg_num, u8 msg_handler){
    u32 if1cmd = 0, if1mctl = 0, if1arb = 0;
#if(!DMA_USED)
    u32 if1data = 0;
#endif

    // Transfer a complete message structure into a message object.
    iowrite32(0, data->base + CAN_IF1ARB); // reset arb registers
    if (msg_handler == 1) if1arb = (1u << 31)&(CAN_IFxARB_MsgVal); // The message object is to be used by the message handler.
    else if1arb &=~ (1u << 31)&(CAN_IFxARB_MsgVal);
    if1arb &=~ (1U << 30); // CAN_IFxARB_Xtd: no Extended Identifier
    if1arb |= (1U << 29); // CAN_IFxARB_Dir: transmit
    if1arb |= (can_id << 18)&(CAN_IFxARB_ID); // 11-bit ID for [28:18]
    iowrite32(if1arb, data->base + CAN_IF1ARB); 

#if(!DMA_USED)
    // Transfer the data bytes of a message into a message object 
    if1data |= data->dma_buffer_tx[0]&(CAN_IFxDATA_0);
    if1data |= (data->dma_buffer_tx[1] << 8)&(CAN_IFxDATA_1);
    if1data |= (data->dma_buffer_tx[2] << 16)&(CAN_IFxDATA_2);
    if1data |= (data->dma_buffer_tx[3] << 24)&(CAN_IFxDATA_3);
    iowrite32(if1data, data->base + CAN_IF1DATA); 
#else
    // /* For using DMA, data by DMA is triggered after this*/
    can0_DMAactive_IF1(data);
#endif

    // config msg control,  set TxRqst 
    if1mctl &=~ (1u << 12); // CAN_IFxMCTL_UMask: mask ignored
    if1mctl |= (1u << 7); // CAN_IFxMCTL_EoB: a single msg obj
    if1mctl |= (1u << 15); // CAN_IFxMCTL_NewDat: 
    if1mctl |= (1u << 8); // :CAN_IFxMCTL_TxRqst message object is waiting for a transmission
    if1mctl |= (size); // DLC = size byte
    if1mctl |= (1u << 11); // TxIE
    iowrite32(if1mctl, data->base + CAN_IF1MCTL); 

 
    // ===== config cmd as the last config
    if1cmd = ioread32(data->base + CAN_IF1CMD);

    if1cmd = (1u << 21)&(CAN_IFxCMD_Arb); // ****** Access arbitration bits -> to update the next time
    if1cmd &=~ (1u << 22); // no use mask

    if1cmd |= (1u << 23); // CAN_IFxCMD_WR_RD: Write
    if1cmd &=~ (1u << 18); // TxRqst_NewDat will by handled by CAN_IFxMCTL_TxRqst or CAN_IFxMCTL_NewDat in CAN_IFxMCTL

    if1cmd |= (1u << 20); // access control bits: msg control bits is transfered FROM IF1 register set TO message object by message number (Bits [7:0]).
    if1cmd |= (1u << 17); // use DATA_A: The data bytes 0-3 will be   transfered FROM IF1 register set TO message object by message number (Bits [7:0]).

    if1cmd |= msg_num << 0; // CAN_IFxCMD_msgnum = 1, trigger transfer

    iowrite32(if1cmd, data->base + CAN_IF1CMD);

    wait_register_update(data, CAN_IF1CMD, CAN_IFxCMD_Busy_OFFSET, 0, MS_DELAY, "Busy bit");
    //data->count_many = 0;

}

void Set_MsgNum_cmd(struct can_device_data *data, u8 msgnum){
    u32 cmd;

    cmd = ioread32(data->base + CAN_IF1CMD);
    cmd |= msgnum&(0xFF);
    if (msgnum == 0) cmd &=~ (0xFF);
    iowrite32(cmd, data->base + CAN_IF1CMD);

    wait_register_update(data, CAN_IF1CMD, CAN_IFxCMD_Busy_OFFSET, 0, MS_DELAY, "Busy bit");
};

void Clear_TxRqst_mctl(struct can_device_data *data){
    u32 mctl;

    mctl = ioread32(data->base + CAN_IF1MCTL);
    mctl &=~ (1u << 8);
    iowrite32(mctl, data->base + CAN_IF1MCTL);
}

void can0_transmit_obj_Data_Frames_rx_msk(struct can_device_data *data, u8 can_id, u8 msg_num, u8 msg_handler){
    u32 IF2cmd = 0, IF2msk = 0, IF2mctl = 0, IF2arb = 0;

    // Transfer msgVal to be used by msg handler.
    iowrite32(0, data->base + CAN_IF2ARB); // reset arb registers
    if (msg_handler == 1) IF2arb = (1u << 31)&(CAN_IFxARB_MsgVal); // The message object is to be used by the message handler.
    else IF2arb &=~ (1u << 31)&(CAN_IFxARB_MsgVal);
    IF2arb |= 0x100 << 18;
    IF2arb &=~ (1u << 29); // read
    iowrite32(IF2arb, data->base + CAN_IF2ARB); 

    // ID: accept any ID comming from any node
    IF2msk &=~ (1u << 31); // Mask Extended Identifier or acceptance filtering.
    IF2msk |= (1u << 30); // the message direction bit (Dir) is used for acceptance filtering
    IF2msk |= (0x700 << 18); // Match all ID bits, no wildcards
    iowrite32(IF2msk, data->base + CAN_IF2MSK); 

    // config msg control: set RxIE, use mask
    IF2mctl |= (1u << 12); // CAN_IFxMCTL_UMask: mask used (Msk[28:0], MXtd, and MDir)
    IF2mctl |= (1u << 7); // CAN_IFxMCTL_EoB: a single msg obj
    IF2mctl &=~ (1u << 15); // CAN_IFxMCTL_NewDat: msg_hler/CPU write no new data
    IF2mctl &=~ (1u << 8); // :CAN_IFxMCTL_TxRqst message object is not waiting for a transmission (not remote frame)
    //IF2mctl |= (size); // DLC = size byte
    IF2mctl |= (1u << 10); // RxIE
    IF2mctl &=~ (1u << 11); // TxIE
    iowrite32(IF2mctl, data->base + CAN_IF2MCTL); 

 
    // ===== config cmd as the last config: -> msg obj for init received obj (msg control bits, DATA_A, DATA_B)
    IF2cmd = ioread32(data->base + CAN_IF2CMD);

    IF2cmd = (1u << 21)&(CAN_IFxCMD_Arb); // ****** Access arbitration bits -> to update the next time
    IF2cmd |= (1u << 22); // no use mask

    IF2cmd |= (1u << 23); // CAN_IFxCMD_WR_RD: Write
    IF2cmd &=~ (1u << 18); // TxRqst_NewDat will by handled by CAN_IFxMCTL_TxRqst or CAN_IFxMCTL_NewDat in CAN_IFxMCTL

    IF2cmd |= (1u << 20); // access control bits: msg control bits is transfered FROM IF2 register set TO message object by message number (Bits [7:0]).
    IF2cmd |= (1u << 17); // use DATA_A: The data bytes 0-3 will be   transfered FROM IF2 register set TO message object by message number (Bits [7:0]).
    IF2cmd |= (1u << 16); // use DATA_B: The data bytes 0-3 will be   transfered FROM IF2 register set TO message object by message number (Bits [7:0]).

    IF2cmd |= msg_num << 0; // CAN_IFxCMD_msgnum = 1, trigger transfer

    iowrite32(IF2cmd, data->base + CAN_IF2CMD);

    // wait the hanler send msg object from IF2 registers to msg RAM
    wait_register_update(data, CAN_IF2CMD, CAN_IFxCMD_Busy_OFFSET, 0, MS_DELAY, "Busy bit");

}

static void Reset_msg_obj_tx(struct can_device_data *data, u8 rd_wr, u8 msg_num, u8 msg_handler){
    u32 if1cmd = 0, if1arb = 0;

    iowrite32(0x00, data->base + CAN_IF1MCTL); // Reset mctl
    iowrite32(0x00, data->base + CAN_IF1MSK); // Reset msk

    iowrite32(0, data->base + CAN_IF1ARB); // reset arb registers
    if (msg_handler == 1) if1arb = (1u << 31)&(CAN_IFxARB_MsgVal); // The message object is to be used by the message handler.
    else if1arb &=~ (1u << 31);
    iowrite32(if1arb, data->base + CAN_IF1ARB); 

    // cmd: w/r, control, msgnum, arb
    if1cmd |= (1u << 21); // Access arbitration bits
    if1cmd |= (1u << 22); // Access mask bits
    if1cmd |= (1u << 20); // access control bits
    if (rd_wr == 1) if1cmd |= 1u << 23; // W
    else if1cmd &=~ (1u << 23); // R
    if1cmd |= msg_num&(0xFF);
    iowrite32(if1cmd, data->base + CAN_IF1CMD);

    // wait the hanler send msg object from IF1 registers to msg RAM
    wait_register_update(data, CAN_IF1CMD, CAN_IFxCMD_Busy_OFFSET, 0, MS_DELAY, "Busy bit");
}

static void Reset_msg_obj_rx(struct can_device_data *data, u8 rd_wr, u8 msg_num, u8 msg_handler){
    u32 IF2cmd = 0, IF2arb = 0;

    iowrite32(0x00, data->base + CAN_IF2MCTL); // Reset mctl
    iowrite32(0x00, data->base + CAN_IF2MSK); // Reset msk

    iowrite32(0, data->base + CAN_IF2ARB); // reset arb registers
    if (msg_handler == 1) IF2arb = (1u << 31)&(CAN_IFxARB_MsgVal); // The message object is to be used by the message handler.
    else IF2arb &=~ (1u << 31);
    iowrite32(IF2arb, data->base + CAN_IF2ARB); 

    // cmd: w/r, control, msgnum, arb
    IF2cmd |= (1u << 21); // Access arbitration bits
    IF2cmd |= (1u << 22); // Access mask bits
    IF2cmd |= (1u << 20); // access control bits
    if (rd_wr == 1) IF2cmd |= 1u << 23; // W
    else IF2cmd &=~ (1u << 23); // R
    IF2cmd |= msg_num&(0xFF);
    iowrite32(IF2cmd, data->base + CAN_IF2CMD);

    // wait the hanler send msg object from IF2 registers to msg RAM
    wait_register_update(data, CAN_IF2CMD, CAN_IFxCMD_Busy_OFFSET, 0, MS_DELAY, "Busy bit");
}

void Reading_received_msg(struct can_device_data *data, u8 msg_num){
    u32 IF2cmd = 0;


    IF2cmd = 0x7F << 16; // [23]: WR_RD  = 0 (Read)
    IF2cmd |= 1u << 14; // DMAactive
    IF2cmd |= msg_num&(0xFF);
    iowrite32(IF2cmd, data->base + CAN_IF2CMD);

    // can0_DMAactive_IF2(data);
    //printk("cmd[reading] = 0x%x\n", ioread32(data->base + CAN_IF2CMD));

    // wait the hanler send msg object from IF2 registers to msg RAM
    wait_register_update(data, CAN_IF2CMD, CAN_IFxCMD_Busy_OFFSET, 0, MS_DELAY, "Busy bit");
    data->count_many = 0;
}

void Clear_rx_flag(struct can_device_data* data){
    u32 can_cmd = 0;

    can_cmd = ioread32(data->base + CAN_IF2CMD);
    can_cmd |= (1u << 19); // ClrIntPnd
    can_cmd &=~ (0xFF);

    // write ClrIntPnd to msg RAM to erase RX interrupt flag
    iowrite32(can_cmd, data->base + CAN_IF2CMD);
    wait_register_update(data, CAN_IF2CMD, CAN_IFxCMD_Busy_OFFSET, 0, MS_DELAY, "Busy bit");
}

/* Init funcs */
void can0_init_tx(struct can_device_data *data) {
    u32 can_ctl = 0;
    int i = 0;

    can_ctl = CAN_CTL_INIT | CAN_CTL_CCE ; // CAN_CTL_DAR is used to disable auto retransmission
    iowrite32(can_ctl, data->base + CAN_CTL); // enter init mode, access to registers
    wait_register_update(data, CAN_CTL, CAN_CTL_INIT_OFFSET, 1, MS_DELAY, "INIT mode");

    // Bit timing values into BTR, output bitrate is 500KHz
    can0_bit_timing(data, clk_get_rate(data->clk), 500000);

    // In init mode, configure received obj for all slots (0-64)
    for (i = 0; i < 64; i ++){
        Reset_msg_obj_tx(data, WR, i+1, 0); // reset msg obj including msgVal
    }

    // clear init, CCE
    can_ctl &=~ (CAN_CTL_INIT | CAN_CTL_CCE);
    iowrite32(can_ctl, data->base + CAN_CTL); // enter init mode, access to registers
    wait_register_update(data, CAN_CTL, CAN_CTL_INIT_OFFSET, 0, MS_DELAY, "Normal mode");
    //printk("ctl[0] = 0x%x\n", ioread32(data->base + CAN_CTL));

    // if1mctl = ioread32(data->base + CAN_IF1MCTL);
    // if1mctl &=~ (1u << 8); // :CAN_IFxMCTL_TxRqst message object is not waiting for a transmission
    // iowrite32(if1mctl, data->base + CAN_IF1MCTL); 
    // iowrite32(if1cmd, data->base + CAN_IF1CMD); // Apply the update
}

void can0_init_rx(struct can_device_data *data) {
    u32 can_ctl = 0, IF2cmd = 0, IF2mctl = 0;
    int i = 0;

    can_ctl = CAN_CTL_INIT | CAN_CTL_CCE;
    iowrite32(can_ctl, data->base + CAN_CTL); // enter init mode, access to registers
    wait_register_update(data, CAN_CTL, CAN_CTL_INIT_OFFSET, 1, MS_DELAY, "INIT mode");

    // Bit timing values into BTR, output bitrate is 500KHz
    can0_bit_timing(data, clk_get_rate(data->clk), 500000);

    // In init mode, configure received obj for all slots (0-64)
    for (i = 0; i < 64; i ++){
        // Reset_msg_obj_tx(data, WR, i+1, 0);
        Reset_msg_obj_rx(data, WR, i+1, 0); // reset msg obj including msgVal (transmit obj, invalid obj msg)
    }

    can0_transmit_obj_Data_Frames_rx_msk(data, ID_received, DMA_OBJ_MSG_NUM_RX, 1); // config received msg obj

    // clear init, CCE
    can_ctl &=~ (CAN_CTL_INIT | CAN_CTL_CCE);
    iowrite32(can_ctl, data->base + CAN_CTL); // enter init mode, access to registers
    wait_register_update(data, CAN_CTL, CAN_CTL_INIT_OFFSET, 0, MS_DELAY, "Normal mode");
    //printk("ctl[0] = 0x%x\n", ioread32(data->base + CAN_CTL));

    IF2mctl = ioread32(data->base + CAN_IF2MCTL);
    IF2mctl &=~ (1u << 8); // :CAN_IFxMCTL_TxRqst message object is not waiting for a transmission
    iowrite32(IF2mctl, data->base + CAN_IF2MCTL); 
    iowrite32(IF2cmd, data->base + CAN_IF2CMD); // Apply the update
}

void GPIO_init(struct can_device_data *data){
    /* P9_19: can0_rx - AM335X_PIN_UART1_RTSN 0x97c */
    iowrite32(0x32, data->base_gpio + 0x97c);

    /* P9_20: can0_tx - AM335X_PIN_UART1_CTSN 0x978 */
    iowrite32(0x12, data->base_gpio + 0x978);
}

void can0_INT_init(struct can_device_data *data){
    u32 canctl = 0;

    canctl = ioread32(data->base + CAN_CTL);
    canctl |= CAN_CTL_IE0; // DCan0INT
    canctl |= CAN_CTL_IE1; // DCAN1INT
    //canctl |= CAN_CTL_SIE; // used for see status if no INT for msg object is raised
    canctl |= CAN_CTL_EIE; 
    iowrite32(canctl, data->base + CAN_CTL);

}

/* DMA func part */
#if(DMA_USED)
void can0_DMA_init(struct can_device_data *data){
    u32 can_ctl = 0;

    can_ctl = ioread32(data->base + CAN_CTL);
    can_ctl |= 1u << 18; // DE1 [IF1]
    can_ctl |= 1u << 19; // DE2 [IF2]
    iowrite32(can_ctl, data->base + CAN_CTL);

    data->dma_channel_tx = 40; // CAN0_IF1 uses DMA channel 40
    data->dma_channel_rx = 41; // CAN0_IF2 uses DMA channel 41

    data->dma_buffer_rx = dma_alloc_coherent(data->dev, MAX_BUFFER_LEN, &data->dma_buffer_phys, GFP_KERNEL | GFP_DMA);
    if (!data->dma_buffer_rx) {
        dev_err(data->dev, "Failed to allocate DMA buffer\n");
        return;
    }
}

void dma_start(struct can_device_data* data, int ch){
    /* Clear any flags */
    iowrite32(1 << (ch - 31), data->base_edma + DMA_ECRH);  
    iowrite32(1 << (ch - 31), data->base_edma + DMA_EMCRH);  
    iowrite32(1 << (ch - 31), data->base_edma + DMA_SECRH); 

    //dev_info(data->dev, "Issuing IF2 DMA pending 1\n");
    iowrite32(1 << (ch - 31), data->base_edma + DMA_EESRH);  /* EESR , HW-TRIGGER*/
#if (PRINT_DEBUG)
    dev_info(data->dev, "Issuing IFx DMA pending\n");
#endif
}

static void can0_if1_dma_callback(void *data)
{
    struct can_device_data *can0 = data;
    int ret;

    //dev_info(can0->dev, "***TX DMA callback called***\n");
    printk("TX DMA callback\n");

    ret = dma_param_set_tx(data, can0->dma_channel_tx, can0->byte_num_rx, (dma_addr_t)&can0->dma_buffer_tx[0]);
    if (ret == -1) return;

    dma_start(can0, can0->dma_channel_tx);
}

static void can0_if2_dma_callback(void *data)
{
    struct can_device_data *can0 = data;
    int i;
    int ret;

    //dev_info(can0->dev, "RX DMA callback called\n");
    printk("RX DMA callback\n");

    printk("Received data[%d]:\n", can0->byte_num_rx);
    for (i = 0; i < can0->byte_num_rx; i++){
        printk("'0x%x' ", ((char*)can0->dma_buffer_rx)[i]);
    }
    printk("\n");
    can0->is_DMAtriggered = 1;

    /* Set this only once time */
    ret = dma_param_set_rx(data, can0->dma_channel_rx, can0->byte_num_rx, (dma_addr_t)can0->dma_buffer_phys);
    if (ret == -1) {
        printk("Fail dma_param_set");
        return;
    }
    else {
        //print_DMA_reg(can0);
    }

    memset(can0->dma_buffer_rx, 0x40, MAX_BUFFER_LEN);
    // pending DMA for next DMA transfer
    dma_start(can0, can0->dma_channel_rx);
}

int dma_param_set_tx(struct can_device_data* data, int ch, u8 byte_num, dma_addr_t dma_src_addr){
    struct dma_async_tx_descriptor *if1_desc;
    u32 param_addr, param_num, opt = 0;
    u32 AB_Cnt = 0, xxxBIDX = 0;

    //int ret; 
    reinit_completion(&data->if1_completion);
    if1_desc = dmaengine_prep_slave_single(data->if1_chan, dma_src_addr, 1,
                                            DMA_MEM_TO_DEV, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
    if (!if1_desc) {
        dev_err(data->dev, "Failed to prepare if1 DMA descriptor\n");
        return -1;
    }

    if1_desc->callback = can0_if1_dma_callback;
    if1_desc->callback_param = data;
#if (PRINT_DEBUG)
    dev_info(data->dev, "Submitting if1 DMA descriptor\n");
#endif
    dmaengine_submit(if1_desc);

    /* Control */
    param_num = (ioread32(data->base_edma + EDMA_DCHMAP_OFFSET + ch*4) >> 5)&0x1FF; // incremented by 4 bytes
#if (PRINT_DEBUG)
    printk("[DMA] - param_num = %d\n", param_num);
#endif

    param_addr = 0x4000 + param_num*0x20; /* 0x4000 + param_num*0x20 (incremented by 32 bytes) */
#if (PRINT_DEBUG)
    printk("[DMA] - param_addr = 0x%x\n", param_addr);
#endif

    //dev_info(data->dev, "Issuing if1 DMA pending\n");
    dma_async_issue_pending(data->if1_chan);

    opt &=~ ((1u << 0) | (1u << 1)); // constant address mode (SAM, DAM)
    opt &=~ (1u << 2); // A-synchronized. Each event triggers the transfer of a single array of ACNT bytes
    opt |= (1u << 3); // Set is static. The PaRAM set is not updated or linked after a TR is submitted
    opt |= (ch << 12); // set channel
    opt |= (1u << 20); // Transfer complete interrupt
    opt |= (1u << 23); // Intermediate transfer complete chaining
    iowrite32(opt, data->base_edma + param_addr + 0x0);  /** OPT */

    // CAN0_BASE + CAN_IF1DATA
    iowrite32(dma_src_addr, data->base_edma + param_addr + 0x4);  /** SRC */

    AB_Cnt = 1<<16 | byte_num; // send byte_num bytes at once
    iowrite32(AB_Cnt, data->base_edma + param_addr + 0x8);  /** ACNT/BCNT */

    iowrite32(CAN0_BASE + CAN_IF1DATA, data->base_edma + param_addr + 0xC);  /** DST */

    xxxBIDX &=~ ((1u >> 16) | 1u); // no incrementing addr as constant addr
    iowrite32(xxxBIDX, data->base_edma + param_addr + 0x10);  /** xxxBIDX */

    iowrite32(0xFFFF, data->base_edma + param_addr + 0x14);  /* LINK=0xFFFF */

    iowrite32(0x0, data->base_edma + param_addr + 0x18);  /* xxxCIDX */
    iowrite32(0x1, data->base_edma + param_addr + 0x1C);  /* CCNT */

    // // due to every 2ms, msg is sent, so the timeout should be 2ms
    // ret = wait_for_completion_timeout(&data->if1_completion, msecs_to_jiffies(5000));
    // if (ret == 0) {
    //     dev_err(data->dev, "DMA if1 timeout\n");
    //     dmaengine_terminate_sync(data->if1_chan);
    //     //goto free_buf;
    // }

    return 0;
}

int dma_param_set_rx(struct can_device_data* data, int ch, u8 byte_num, dma_addr_t dma_dst_addr){
    struct dma_async_tx_descriptor *if2_desc;
    u32 param_addr, param_num, opt = 0;
    u32 AB_Cnt = 0, xxxBIDX = 0;

    reinit_completion(&data->if2_completion);
    if2_desc = dmaengine_prep_slave_single(data->if2_chan, dma_dst_addr, 1,
                                            DMA_DEV_TO_MEM, DMA_CTRL_ACK | DMA_PREP_INTERRUPT); // [DIFF]
    if (!if2_desc) {
        dev_err(data->dev, "Failed to prepare IFx DMA descriptor\n");
        return -1;
    }

    if2_desc->callback = can0_if2_dma_callback;
    if2_desc->callback_param = data;
#if (PRINT_DEBUG)
    dev_info(data->dev, "Submitting IFx DMA descriptor, ch = %d\n", ch);
#endif
    dmaengine_submit(if2_desc);

    /* Control */
    param_num = (ioread32(data->base_edma + EDMA_DCHMAP_OFFSET + ch*4) >> 5)&0x1FF; // incremented by 4 bytes
#if (PRINT_DEBUG)
    printk("[DMA] - param_num = %d\n", param_num);
#endif

    param_addr = 0x4000 + param_num*0x20; /* 0x4000 + param_num*0x20 (incremented by 32 bytes) */
#if (PRINT_DEBUG)
    printk("[DMA] - param_addr = 0x%x\n", param_addr);
#endif

    // dev_info(data->dev, "Issuing IF2 DMA pending\n");
    dma_async_issue_pending(data->if2_chan);

    opt &=~ ((1u << 0) | (1u << 1)); // constant address mode (SAM, DAM)
    opt &=~ (1u << 2); // A-synchronized. Each event triggers the transfer of a single array of ACNT bytes
    opt |= (1u << 3); // Set is static. The PaRAM set is not updated or linked after a TR is submitted
    opt |= (ch << 12); // set channel
    opt |= (1u << 20); // Transfer complete interrupt
    opt |= (1u << 23); // Intermediate transfer complete chaining
    iowrite32(opt, data->base_edma + param_addr + 0x0);  /** OPT */

    // CAN0_BASE + CAN_IF2DATA
    iowrite32(CAN0_BASE + CAN_IF2DATA, data->base_edma + param_addr + 0x4);  /** SRC */

    AB_Cnt = 1<<16 | byte_num; // send byte_num bytes at once
    iowrite32(AB_Cnt, data->base_edma + param_addr + 0x8);  /** ACNT/BCNT */

    iowrite32(dma_dst_addr, data->base_edma + param_addr + 0xC);  /** DST */

    xxxBIDX &=~ ((1u >> 16) | 1u); // no incrementing addr as constant addr
    iowrite32(xxxBIDX, data->base_edma + param_addr + 0x10);  /** xxxBIDX */

    iowrite32(0xFFFF, data->base_edma + param_addr + 0x14);  /* LINK=0xFFFF */

    iowrite32(0x0, data->base_edma + param_addr + 0x18);  /* xxxCIDX */
    iowrite32(0x1, data->base_edma + param_addr + 0x1C);  /* CCNT */

    return 0;
}

void Dma_write(struct can_device_data* data){
    int ret;

    memset(data->dma_buffer_tx, 0xFF, MAX_BUFFER_LEN);

    /* Set this only once time */
    ret = dma_param_set_tx(data, data->dma_channel_tx, data->byte_num_tx, (dma_addr_t)&data->dma_buffer_tx[0]);
    if (ret == -1) return;

    /* Start for the first time */
    dma_start(data, data->dma_channel_tx);

}

void Dma_read(struct can_device_data* data){
    int ret;

    memset(data->dma_buffer_rx, 0xFF, MAX_BUFFER_LEN);

    /* Set this only once time */
    ret = dma_param_set_rx(data, data->dma_channel_rx, data->byte_num_rx, (dma_addr_t)data->dma_buffer_phys);
    if (ret == -1) {
        printk("Fail dma_param_set");
        return;
    }
    else {
        //print_DMA_reg(data);
    }

    /* Start for the first time */
    dma_start(data, data->dma_channel_rx);

}

static int can0_configure_dma_tx(struct can_device_data *data)
{
    struct dma_slave_config if1_conf = {0};
    int ret;

    if (data->if1_chan) {
        if1_conf.direction = DMA_MEM_TO_DEV;
        if1_conf.dst_addr = (dma_addr_t)(CAN0_BASE + CAN_IF1DATA); // will include CAN_IF1DATA and CAN_IF1DATB
        if1_conf.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE; /* UART uses 8-bit transfers */
        if1_conf.dst_maxburst = 1; /* Single byte per transfer */
        ret = dmaengine_slave_config(data->if1_chan, &if1_conf);
        if (ret) {
            dev_err(data->dev, "Failed to configure if1 DMA channel: %d\n", ret);
            return ret;
        }
        dev_info(data->dev, "if1 DMA channel configured\n");
    }

    return 0;
}

static int can0_configure_dma_rx(struct can_device_data *data)
{
    struct dma_slave_config IF2_conf = {0};
    int ret;

    if (data->if2_chan) {
        IF2_conf.direction = DMA_DEV_TO_MEM; // [DIFF]
        IF2_conf.src_addr = (dma_addr_t)(CAN0_BASE + CAN_IF2DATA); // will include CAN_IF2DATA and CAN_IF2DATB
        IF2_conf.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE; /* UART uses 8-bit transfers */
        IF2_conf.src_maxburst = 1; /* Single byte per transfer */
        ret = dmaengine_slave_config(data->if2_chan, &IF2_conf);
        if (ret) {
            dev_err(data->dev, "Failed to configure IF2 DMA channel: %d\n", ret);
            return ret;
        }
        dev_info(data->dev, "if2 DMA channel configured\n");
    }
    return 0;
}

void DMA_probe(struct can_device_data *data){
    int ret;

    data->edma_clk = devm_clk_get(data->dev, "fck-dma");
    if (IS_ERR(data->edma_clk)) {
        dev_err(data->dev, "Failed to get eDMA clock: %ld\n", PTR_ERR(data->edma_clk));
        return;
    }
    ret = clk_prepare_enable(data->edma_clk);
    if (ret) {
        dev_err(data->dev, "Failed to enable eDMA clock: %d\n", ret);
        return;
    }
    dev_info(data->dev, "Edma clock rate: %lu Hz\n", clk_get_rate(data->edma_clk));

    /* Request DMA channels */
    data->if1_chan = dma_request_chan(data->dev, "if1_dma");
    if (IS_ERR(data->if1_chan)) {
        dev_info(data->dev, "No TX DMA channel, falling back to non-DMA mode: %ld\n",
                 PTR_ERR(data->if1_chan));
        data->if1_chan = NULL;
    }
    data->if2_chan = dma_request_chan(data->dev, "if2_dma");
    if (IS_ERR(data->if2_chan)) {
        dev_info(data->dev, "No RX DMA channel, falling back to non-DMA mode: %ld\n",
                 PTR_ERR(data->if2_chan));
        data->if2_chan = NULL;
    }

    /* COnfig parameters for DMA */
    /* For tx */
    data->use_dma = (data->if1_chan);
    if (data->use_dma){
#if (PRINT_DEBUG)
        dev_info(data->dev, "Using DMA for CAN0 transfers for TX\n");
#endif
        ret = can0_configure_dma_tx(data);
        if (ret) {
#if (PRINT_DEBUG)
            dev_info(data->dev, "Failed to configure DMA, falling back to non-DMA mode\n");
#endif
            if (data->if1_chan)
                dma_release_channel(data->if1_chan);
            data->if1_chan = NULL;
            data->use_dma = false;
        }
        else {
            dev_info(data->dev, "Succeed to configure DMA for TX\n");
        }
    }
    else
        dev_info(data->dev, "Using interrupt-driven transfers\n");
    
    /* For rx */
    data->use_dma = 0;
    data->use_dma = (data->if2_chan);
    if (data->use_dma){
#if (PRINT_DEBUG)
        dev_info(data->dev, "Using DMA for CAN0 transfers for RX\n");
#endif
        ret = can0_configure_dma_rx(data);
        if (ret) {
            dev_info(data->dev, "Failed to configure DMA, falling back to non-DMA mode\n");
            if (data->if2_chan)
                dma_release_channel(data->if2_chan);
            data->if2_chan = NULL;
            data->use_dma = false;
        }
        else {
            dev_info(data->dev, "Succeed to configure DMA for RX\n");
        }
    }
    else
        dev_info(data->dev, "Using interrupt-driven transfers\n");

    init_completion(&data->if1_completion);
    init_completion(&data->if2_completion);
}

#endif

void can0_enter_mode(struct can_device_data *data){
    wait_register_update(data, CAN_CTL, CAN_CTL_INIT_OFFSET, 0, MS_DELAY, "Normal mode 1");

    data->byte_num_tx = 4;
#if (DMA_USED)
    Dma_write(data);

    data->byte_num_rx = 8;
    Dma_read(data);
#endif
}

void can0_enter_done(struct can_device_data *data){
    u32 can_ctl;

    can_ctl = CAN_CTL_INIT | CAN_CTL_CCE | CAN_CTL_SWR ; // CAN_CTL_DAR is used to disable auto retransmission
    iowrite32(can_ctl, data->base + CAN_CTL); // enter init mode, access to registers
    wait_register_update(data, CAN_CTL, CAN_CTL_INIT_OFFSET, 1, MS_DELAY, "INIT mode");
    wait_register_update(data, CAN_CTL, CAN_CTL_SWR_OFFSET, 0, MS_DELAY, "Reset mode");
}