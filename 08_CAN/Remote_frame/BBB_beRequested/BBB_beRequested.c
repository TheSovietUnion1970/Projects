#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/completion.h>
#include "main.h"

#define DRIVER_NAME "can0_driver"
#define DEVICE_NAME "can0"

#define MS_DELAY 2000

#define CAN0_BASE       0x481cc000
#define GPIO_BASE       0x44e10000
#define EDMA_BASE       0x49000000

#define EDMA_DCHMAP_OFFSET 0x0100
#define DMA_EER 0x1020
#define DMA_EERH 0x1024
#define DMA_ER 0x1000
#define DMA_ERH 0x1004
#define DMA_ECR 0x1008
#define DMA_ECRH 0x100C
#define DMA_EMCR 0x308
#define DMA_EMCRH 0x30C
#define DMA_SECR 0x1040
#define DMA_SECRH 0x1044
#define DMA_EESR 0x1030
#define DMA_EESRH 0x1034
#define DMA_IPR 0x1068
#define DMA_IPRH 0x106C
#define DMA_ESR 0x1010
#define DMA_ESRH 0x1014
#define DMA_IESR 0x1060
#define DMA_IESRH 0x1064
#define DMA_IER 0x1050
#define DMA_IERH 0x1054

#define CAN_CTL 0x00
#define CAN_ES 0x04
#define CAN_BTR 0x0C
#define CAN_INT 0x10
#define CAN_IF1CMD 0x100
#define CAN_IF1MSK 0x104
#define CAN_IF1ARB 0x108
#define CAN_IF1MCTL 0x10C
#define CAN_IF1DATA 0x110
#define CAN_INTPND_X 0xAC
#define CAN_INTPND12 0xB0
#define CAN_MSGVAL12 0xC4
#define CAN_MSGVAL34 0xC8
#define CAN_MSGVAL56 0xCC
#define CAN_MSGVAL78 0xD0
#define CAN_NWDAT_X 0x98
#define CAN_NWDAT12 0x9C
#define CAN_TXRQ_X 0x84
#define CAN_TXRQ12 0x88
#define CAN_INTMUX12 0xD8

#define CAN_CTL_INIT BIT(0)
#define CAN_CTL_INIT_OFFSET 0
#define CAN_CTL_CCE BIT(6)
#define CAN_CTL_CCE_OFFSET 6
#define CAN_CTL_DAR BIT(5)
#define CAN_CTL_IE1 BIT(17)
#define CAN_CTL_IE0 BIT(1)
#define CAN_CTL_SIE BIT(2)
#define CAN_CTL_EIE BIT(3)

#define CAN_IFxCMD_msgnum 0xff // [7:0]
#define CAN_IFxCMD_Busy BIT(15)
#define CAN_IFxCMD_Busy_OFFSET 15
#define CAN_IFxCMD_WR_RD BIT(23)

// #define CAN_IFxMSK_Msk 0x1fffffff // [28:0]
// #define CAN_IFxMSK_MDir BIT(30)
// #define CAN_IFxMSK_MXtd BIT(31)
#define CAN_IFxCMD_Arb BIT(21)

#define CAN_IFxARB_MsgVal BIT(31)
#define CAN_IFxARB_Dir BIT(29)
#define CAN_IFxARB_Xtd BIT(30)
#define CAN_IFxARB_ID 0x1fffffff // [28:0]

#define CAN_IFxMCTL_UMask BIT(12)
#define CAN_IFxMCTL_EoB BIT(7)
#define CAN_IFxMCTL_NewDat BIT(15)
#define CAN_IFxMCTL_TxRqst BIT(8)
#define CAN_IFxMCTL_DLC 0xf // [3:0]
#define CAN_IFxMCTL_IntPnd BIT(13)
#define CAN_CTL_SWR BIT(15)
#define CAN_CTL_SWR_OFFSET 15

#define CAN_IFxDATA_0 0xff // [7:0]
#define CAN_IFxDATA_1 0xff00 // [15:8]
#define CAN_IFxDATA_2 0xff0000 // [23:16]
#define CAN_IFxDATA_3 0xff000000 // [31:24]

#define RD 0
#define WR 1

u8 TX[4] = {0x1F, 0x1F, 0x1F, 0x1F};
u8 TX_print[4] = {0};
u8 size = sizeof(TX);
#define DMA_REG 1

struct can_device_data {

    dev_t dev_num;
    struct cdev cdev;
    struct class *class;
    struct device *dev;

    void __iomem *base;  // Mapped base address of can0 registers
    void __iomem *base_gpio;
    void __iomem *base_edma;
    struct clk *clk;

    u8 increment;
    struct work_struct re_request_work;
    bool is_scheduled;
    atomic_t should_stop; // Use atomic_t instead of bool

    int irq;
    u8 count_many;
    u32 tmpBuffer[2]; // for datA and datB

    /* DMA-related fields */
    struct dma_chan *if1_chan;
    struct completion if1_completion;
    bool use_dma; /* Flag to indicate if DMA is used */
    struct clk *edma_clk;
    int dma_channel;
    u8 dma_buffer[256];
    u8 byte_num;
    u8 isDMAInit;
    u8 DMA_triggered;
    u8 DMA_callback;

    spinlock_t mylock;

    u32 TX2;
};

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

void Dma_write(struct can_device_data* data);
void can0_DMAactive_IF1(struct can_device_data *data, u8 msg_num);
int dma_param_set(struct can_device_data* data, int ch, u8 byte_num, dma_addr_t dma_src_addr);

void can0_bit_timing(struct can_device_data *data, u32 can_clk, u32 bitrate){
    u32 val = 0;

    u8 SJW = 1;
    u8 TSeg1 = 4;
    u8 TSeg2 = 3;

    /* CAN_CLK is 24MHz */
    u16 BRP = can_clk/(bitrate*(SJW + TSeg1 + TSeg2)); // 5
    printk("BRP = 0x%x\n", BRP);

    BRP = BRP - 1;
    SJW = SJW - 1;
    TSeg1 = TSeg1 - 1;
    TSeg2 = TSeg2 - 1;

    val = ((BRP)&0x3F) | ((SJW << 6)&0xC0) | ((TSeg1 << 8)&0xF00) | ((TSeg2 << 12)&0x7000);

    iowrite32(val, data->base + CAN_BTR);

}

void can0_receive_obj_Remote_Frames_write(struct can_device_data *data, u8 msg_num){
    u32 if1cmd = 0;
#if (!DMA_USED)
    u32 if1data = 0, if1mctl = 0;
#endif

    if1cmd = ioread32(data->base + CAN_IF1CMD);
    if1cmd &=~ (1u << 23); // CAN_IFxCMD_WR_RD: Read
    if1cmd |= msg_num&(0xFF);
    iowrite32(if1cmd, data->base + CAN_IF1CMD);
    wait_register_update(data, CAN_IF1CMD, CAN_IFxCMD_Busy_OFFSET, 0, MS_DELAY, "Busy bit");

#if (!DMA_USED)
    // Transfer the data bytes of a message into a message object 
    if1data = TX[0]&(CAN_IFxDATA_0);
    if1data |= (TX[1] << 8)&(CAN_IFxDATA_1);
    if1data |= (TX[2] << 16)&(CAN_IFxDATA_2);
    if1data |= (TX[3] << 24)&(CAN_IFxDATA_3);
    iowrite32(if1data, data->base + CAN_IF1DATA);

    // config msg control
    if1mctl = ioread32(data->base + CAN_IF1MCTL);
    if1mctl &=~ (1u << 8); // :CAN_IFxMCTL_TxRqst message object is set by rmtEn at reception /* Remote frame only */
    iowrite32(if1mctl, data->base + CAN_IF1MCTL); 

    if1cmd = ioread32(data->base + CAN_IF1CMD);
    if1cmd |= (1u << 23); // CAN_IFxCMD_WR_RD: Write
    if1cmd |= msg_num&(0xFF);
    iowrite32(if1cmd, data->base + CAN_IF1CMD);
    printk("-> CAN_IF1CMD <-\n");

    //printk("KK\n");
    wait_register_update(data, CAN_IF1CMD, CAN_IFxCMD_Busy_OFFSET, 0, MS_DELAY, "Busy bit");
#else
    Dma_write(data); // enable HW DMA on DMA side
    can0_DMAactive_IF1(data, DMA_OBJ_MSG_NUM); // enable DMA on CAN side
    //if (data->DMA_callback) while(!data->DMA_triggered);
    data->DMA_triggered = 0;
#endif
}

void can0_receive_obj_Remote_Frames_init(struct can_device_data *data, u16 can_id, u8 msg_num, u8 msg_handler, u8* TX){
    u32 if1cmd = 0, if1mctl = 0, if1arb = 0, if1msk = 0;
    u32 if1data = 0;

    // if1cmd = ioread32(data->base + CAN_IF1CMD);
    // if1cmd &=~ (1u << 23); // CAN_IFxCMD_WR_RD: Read
    // if1cmd |= msg_num&(0xFF);
    // iowrite32(if1cmd, data->base + CAN_IF1CMD);
    // wait_register_update(data, CAN_IF1CMD, CAN_IFxCMD_Busy_OFFSET, 0, MS_DELAY, "Busy bit");

    // ID: accept any ID comming from any node
    if1msk &=~ (1u << 31); // Mask Extended Identifier or acceptance filtering.
    if1msk &=~ (1u << 30); // the message direction bit (Dir) is used for acceptance filtering
    if1msk &=~ (0x7FF << 18); // Match all ID bits, no wildcards
    iowrite32(if1msk, data->base + CAN_IF1MSK); 

    // Transfer a complete message structure into a message object.
    iowrite32(0, data->base + CAN_IF1ARB); // reset arb registers
    if (msg_handler == 1) if1arb = (1u << 31)&(CAN_IFxARB_MsgVal); // The message object is to be used by the message handler.
    else if1arb &=~ (1u << 31)&(CAN_IFxARB_MsgVal);
    if1arb &=~ (1U << 30); // CAN_IFxARB_Xtd: no Extended Identifier
    if1arb |= (1U << 29); // CAN_IFxARB_Dir: transmit
    if1arb |= (can_id << 18)&(CAN_IFxARB_ID); // 11-bit ID for [28:18]
    iowrite32(if1arb, data->base + CAN_IF1ARB); 

    // Transfer the data bytes of a message into a message object 
    if1data = TX[0]&(CAN_IFxDATA_0);
    if1data |= (TX[1] << 8)&(CAN_IFxDATA_1);
    if1data |= (TX[2] << 16)&(CAN_IFxDATA_2);
    if1data |= (TX[3] << 24)&(CAN_IFxDATA_3);
    iowrite32(if1data, data->base + CAN_IF1DATA); 

    // TX[0] = 0;
    // TX[1] = 1;
    // TX[2] = 2;
    // TX[3] = 3;

    // config msg control,  set TxRqst 
    if1mctl = (1u << 12); // CAN_IFxMCTL_UMask: mask used (Msk[28:0], MXtd, and MDir)
    if1mctl |= (1u << 7); // CAN_IFxMCTL_EoB: a single msg obj
    if1mctl &=~ (1u << 15); // CAN_IFxMCTL_NewDat: 

    if1mctl |= (1u << 9); //  RmtEn: At the reception of a remote frame, TxRqst is set. /* Remote frame only */
    if1mctl &=~ (1u << 8); // :CAN_IFxMCTL_TxRqst message object is set by rmtEn at reception /* Remote frame only */

    if1mctl |= (size); // DLC = size byte
    if1mctl |= (1u << 10); // RxIE /* Remote frame only */
    iowrite32(if1mctl, data->base + CAN_IF1MCTL); 

 
    // ===== config cmd as the last config
    if1cmd = ioread32(data->base + CAN_IF1CMD);

    if1cmd = (1u << 21)&(CAN_IFxCMD_Arb); // ****** Access arbitration bits -> to update the next time
    if1cmd |= (1u << 22); // use mask

    if1cmd |= (1u << 23); // CAN_IFxCMD_WR_RD: Write
    if1cmd &=~ (1u << 18); // TxRqst_NewDat will by handled by CAN_IFxMCTL_TxRqst or CAN_IFxMCTL_NewDat in CAN_IFxMCTL

    if1cmd |= (1u << 20); // access control bits: msg control bits is transfered FROM IF1 register set TO message object by message number (Bits [7:0]).
    if1cmd |= (1u << 17); // use DATA_A: The data bytes 0-3 will be   transfered FROM IF1 register set TO message object by message number (Bits [7:0]).

    if1cmd |= msg_num << 0; // CAN_IFxCMD_msgnum = 1, trigger transfer

    iowrite32(if1cmd, data->base + CAN_IF1CMD);

    wait_register_update(data, CAN_IF1CMD, CAN_IFxCMD_Busy_OFFSET, 0, MS_DELAY, "Busy bit");
    //data->count_many = 0;

}

void Reset_msg_obj(struct can_device_data *data, u8 rd_wr, u8 msg_num, u8 msg_handler){
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

/* DMA func part */
#if(DMA_USED)
void can0_DMA_init(struct can_device_data *data){
    u32 can_ctl = 0;

    can_ctl = ioread32(data->base + CAN_CTL);
    can_ctl |= 1u << 18; // DE1
    iowrite32(can_ctl, data->base + CAN_CTL);

    data->dma_channel = 40; // CAN0_IF1 uses DMA channel 40
}

void can0_DMAactive_IF1(struct can_device_data *data, u8 msg_num){
    u32 can_cmd = 0;

    can_cmd = ioread32(data->base + CAN_IF1CMD);
    can_cmd |= 1u << 23; // W
    can_cmd |= 1u << 14; // DMAactive
    can_cmd |= msg_num&(0xFF);
    iowrite32(can_cmd, data->base + CAN_IF1CMD);
    
    wait_register_update(data, CAN_IF1CMD, CAN_IFxCMD_Busy_OFFSET, 0, MS_DELAY, "Busy bit");
}

void can0_DMAdeactive_IF1(struct can_device_data *data){
    u32 can_cmd = 0;

    can_cmd = ioread32(data->base + CAN_IF1CMD);
    can_cmd |= 1u << 23; // W
    can_cmd &=~ (1u << 14); // DMAactive
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

void dma_start(struct can_device_data* data, int ch){
    /* Clear any flags */
    iowrite32(1 << (ch - 31), data->base_edma + DMA_ECRH);  
    iowrite32(1 << (ch - 31), data->base_edma + DMA_EMCRH);  
    iowrite32(1 << (ch - 31), data->base_edma + DMA_SECRH); 
#if (PRINT_DEBUG)
    dev_info(data->dev, "Issuing if1 DMA ch %d pending\n", ch);
#endif
    iowrite32(1 << (ch - 31), data->base_edma + DMA_EESRH);  /* EESR , HW-TRIGGER*/
}

static void can0_if1_dma_callback(void *d)
{
    struct can_device_data *data = d;
    u32 if1mctl = 0, if1cmd = 0;
    unsigned long flags;

    spin_lock_irqsave(&data->mylock, flags);

    printk("TX DMA callback\n");
    data->DMA_callback = 1;
    data->DMA_triggered = 1;

#if (PRINT_DEBUG)
    printk("[DMA_callbk]******can0_if1_dma_callback -> '0x%x'*********\n", ioread32(data->base + CAN_IF1DATA));
    printk("[DMA_callbk] data->tmpBuffer[0] = 0x%x\n", data->tmpBuffer[0]);
#endif

    // config msg control
    if1mctl = ioread32(data->base + CAN_IF1MCTL);
    //if1mctl |= (1u << 9); // RmtEn
    if1mctl &=~ (1u << 8); // :CAN_IFxMCTL_TxRqst message object is set by rmtEn at reception /* Remote frame only */
    iowrite32(if1mctl, data->base + CAN_IF1MCTL); 

    if1cmd = ioread32(data->base + CAN_IF1CMD);
    if1cmd |= (1u << 23); // CAN_IFxCMD_WR_RD: Write
    if1cmd |= DMA_OBJ_MSG_NUM&(0xFF);
    iowrite32(if1cmd, data->base + CAN_IF1CMD);
#if (PRINT_DEBUG)
    printk("-> CAN_IF1CMD <-\n");
    printk("=================================\n");
#endif
    wait_register_update(data, CAN_IF1CMD, CAN_IFxCMD_Busy_OFFSET, 0, MS_DELAY, "Busy bit");

    spin_unlock_irqrestore(&data->mylock, flags);
}

int dma_param_set(struct can_device_data* data, int ch, u8 byte_num, dma_addr_t dma_src_addr){
    struct dma_async_tx_descriptor *if1_desc;
    u32 param_addr, param_num, opt = 0;
    u32 AB_Cnt = 0, xxxBIDX = 0;
#if(!DMA_REG)
    reinit_completion(&data->if1_completion);
    if1_desc = dmaengine_prep_slave_single(data->if1_chan, dma_dst_addr, 1,
                                            DMA_DEV_TO_MEM, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
    if (!if1_desc) {
        dev_err(data->dev, "Failed to prepare if1 DMA descriptor\n");
        goto free_buf;
    }

    if1_desc->callback = can0_if1_dma_callback;
    if1_desc->callback_param = data;

    dev_info(data->dev, "Submitting if1 DMA descriptor\n");
    dmaengine_submit(if1_desc);

    

    dev_info(data->dev, "Issuing if1 DMA pending\n");
    dma_async_issue_pending(data->if1_chan);

    print_DMA_reg(data);

    //((char*)kbuf)[0] = ioread32(data->base + CAN_if1DATA);

    // due to every 2ms, msg is sent, so the timeout should be 2ms
    ret = wait_for_completion_timeout(&data->if1_completion, msecs_to_jiffies(5000));
    if (ret == 0) {
        dev_err(data->dev, "DMA if1 timeout\n");
        dmaengine_terminate_sync(data->if1_chan);
        goto free_buf;
    }

    goto free_buf;
#else
    //dma_addr_t phys = dma_map_single(data->dev, dma_src_addr, 4, DMA_TO_DEVICE);
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

    return 0;
#endif
}

void Dma_write(struct can_device_data* data){
    int ret;
#if (PRINT_DEBUG)
    printk("[Dma_write] Before: data->tmpBuffer[0] = 0x%x\n", data->tmpBuffer[0]);
#endif
    /* Set this only once time */
    ret = dma_param_set(data, data->dma_channel, data->byte_num, (dma_addr_t)&data->tmpBuffer[0]);
    //ret = dma_param_set(data, data->dma_channel, data->byte_num, (u32*)&data->TX2);
    if (ret == -1) return;

    /* Start for the first time */
    dma_start(data, data->dma_channel);

}

static int can0_configure_dma(struct can_device_data *data)
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

    /* COnfig parameters for DMA */
    data->use_dma = (data->if1_chan);
    if (data->use_dma){
        dev_info(data->dev, "Using DMA for CAN0 transfers\n");
        ret = can0_configure_dma(data);
        if (ret) {
            dev_info(data->dev, "Failed to configure DMA, falling back to non-DMA mode\n");
            if (data->if1_chan)
                dma_release_channel(data->if1_chan);
            data->if1_chan = NULL;
            data->use_dma = false;
        }
        else {
            dev_info(data->dev, "Succeed to configure DMA\n");
        }
    }
    else
        dev_info(data->dev, "Using interrupt-driven transfers\n");

    init_completion(&data->if1_completion);
}

#endif

static irqreturn_t irqHandler(int irq, void *d){
    struct can_device_data *data = d;
    u32 canctl, Int0ID, can_mctl, can_cmd, can_arb;
    u32 es = 0, intpnd_x = 0, intpnd12 = 0, msgval12 = 0, msgval34 = 0, msgval56 = 0, msgval78 = 0;
    u32 datx, dat12;
    u32 txx, tx12, mux12;
    u16 id = 0;
    int i = 0;
    unsigned long flags;
    u32 if1data = 0, if1mctl, if1cmd;

    if (!data) {
        pr_err("NULL data in irqHandler\n");
        return IRQ_NONE;
    }

    spin_lock_irqsave(&data->mylock, flags);

    intpnd12 = ioread32(data->base + CAN_INTPND12);
    intpnd_x = ioread32(data->base + CAN_INTPND_X);
    Int0ID = ioread32(data->base + CAN_INT);
    can_mctl = ioread32(data->base + CAN_IF1MCTL);
    can_cmd = ioread32(data->base + CAN_IF1CMD);
    can_arb = ioread32(data->base + CAN_IF1ARB);

    msgval12 = ioread32(data->base + CAN_MSGVAL12);
    msgval34 = ioread32(data->base + CAN_MSGVAL34);
    msgval56 = ioread32(data->base + CAN_MSGVAL56);
    msgval78 = ioread32(data->base + CAN_MSGVAL78);

    datx = ioread32(data->base + CAN_NWDAT_X);
    dat12 = ioread32(data->base + CAN_NWDAT12);
    txx = ioread32(data->base + CAN_TXRQ_X);
    tx12 = ioread32(data->base + CAN_TXRQ12);
    mux12 = ioread32(data->base + CAN_INTMUX12);

    id = (can_arb >> 18)&0x7FF;

    es = ioread32(data->base + CAN_ES);
#if (PRINT_DEBUG)
    printk("[IRQ_hdler] es = 0x%x, msgval12 = 0x%x\n", es, msgval12);
#endif
    if (Int0ID != 0x8000){ 
        /* Not applicable for Remote frame */
    } 
    else { /* Int0ID = 0x8000 */
#if (PRINT_DEBUG)
        printk("es = 0x%x, mctl = 0x%x, intpnd_x = 0x%x, id = 0x%x, pd12 = 0x%x\n", es, can_mctl, intpnd_x, id, intpnd12);
#endif
        if ((es&(1u << 4u)) == 1u << 4u){ // RxOk -> update Tx value
            printk("[IRQ_hdler] RX_IRQ - Get remote frame\n");

            // DMA INT is triggered after TxOk, so this new data by DMA will be used for the next remote frame request 
            //data->tmpBuffer[0] = (TX[3] << 24) | (TX[2] << 16) | (TX[1] << 8) | (TX[0] << 0);
#if (!DMA_USED)
            printk("Sent data[%d] corresponding to remote frame\n", SIZE_REQUESTED);
            for (i = 0; i < SIZE_REQUESTED; i++){
                printk("'0x%x'\n", TX[i]);
            }

#endif
        }
        if ((es&(1u << 3u)) == 1u << 3u){
#if (DMA_USED)
            //can0_DMAdeactive_IF1(data);

            if (TX[0] == 0xFF) TX[0] = 0;
            if (TX[1] == 0xFF) TX[1] = 0;
            if (TX[2] == 0xFF) TX[2] = 0;
            if (TX[3] == 0xFF) TX[3] = 0;

            /* Print TX_print as TX now is the updated value for the next operation
            as DMA needs this updated value from TX to IF1 */
            printk("[DMA] Sent data[%d] corresponding to remote frame\n", SIZE_REQUESTED);
            for (i = 0; i < SIZE_REQUESTED; i++){
                printk("'0x%x'\n", TX_print[i]);
            }

            memcpy(TX_print, TX, SIZE_REQUESTED);

            TX[0]++;
            TX[1]++;
            TX[2]++;
            TX[3]++;
            data->tmpBuffer[0] = (TX[3] << 24) | (TX[2] << 16) | (TX[1] << 8) | (TX[0] << 0);
#if (PRINT_DEBUG)
            printk("[IRQ_hdler] data->tmpBuffer[0] = 0x%x\n", data->tmpBuffer[0]);
            printk("[IRQ_hdler] handler -> '0x%x'\n", ioread32(data->base + CAN_IF1DATA));
#endif
            can0_receive_obj_Remote_Frames_write(data, DMA_OBJ_MSG_NUM);
            data->count_many = 0;           
#else
            /* Update val for the next time */
            TX[0]++;
            TX[1]++;
            TX[2]++;
            TX[3]++;

            // Transfer the data bytes of a message into a message object 
            if1data = TX[0]&(CAN_IFxDATA_0);
            if1data |= (TX[1] << 8)&(CAN_IFxDATA_1);
            if1data |= (TX[2] << 16)&(CAN_IFxDATA_2);
            if1data |= (TX[3] << 24)&(CAN_IFxDATA_3);
            iowrite32(if1data, data->base + CAN_IF1DATA);

            // config msg control
            if1mctl = ioread32(data->base + CAN_IF1MCTL);
            //if1mctl |= (1u << 9); // RmtEn
            if1mctl &=~ (1u << 8); // :CAN_IFxMCTL_TxRqst message object is set by rmtEn at reception /* Remote frame only */
            iowrite32(if1mctl, data->base + CAN_IF1MCTL); 

            if1cmd = ioread32(data->base + CAN_IF1CMD);
            if1cmd |= (1u << 23); // CAN_IFxCMD_WR_RD: Write
            if1cmd |= DMA_OBJ_MSG_NUM&(0xFF);
            iowrite32(if1cmd, data->base + CAN_IF1CMD);
#endif

        }
    }


    data->count_many++;
    if (data->count_many > 100){
        printk("Too many interrupts\n");
        data->count_many = 0;

        canctl = ioread32(data->base + CAN_CTL);
        canctl &=~ CAN_CTL_IE0; // disable DCAN0INT
        iowrite32(canctl, data->base + CAN_CTL);
    }

    spin_unlock_irqrestore(&data->mylock, flags);


    return IRQ_HANDLED;
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
    canctl |= CAN_CTL_SIE; // used for see status if no INT for msg object is raised
    canctl |= CAN_CTL_EIE; 
    iowrite32(canctl, data->base + CAN_CTL);

}

void can0_init(struct can_device_data *data) {
    u32 can_ctl = 0;
    int i = 0;

    can_ctl = CAN_CTL_INIT | CAN_CTL_CCE ; // CAN_CTL_DAR is used to disable auto retransmission
    iowrite32(can_ctl, data->base + CAN_CTL); // enter init mode, access to registers
    //while((ioread32(data->base + CAN_CTL)&CAN_CTL_INIT) == CAN_CTL_INIT); // wait init = 1;
    wait_register_update(data, CAN_CTL, CAN_CTL_INIT_OFFSET, 1, MS_DELAY, "INIT mode");
    //wait_register_update(data, CAN_CTL, 15, 0, MS_DELAY, "Reset state");

    // Bit timing values into BTR, output bitrate is 500KHz
    can0_bit_timing(data, clk_get_rate(data->clk), 500000);

    // // In init mode, configure received obj for all slots (0-128)
    for (i = 0; i < 64; i ++){
        Reset_msg_obj(data, WR, i+1, 0); // reset msg obj including msgVal
    }
    // init transmit obj with remote frame configuration (no use DMA)
    can0_receive_obj_Remote_Frames_init(data, ID_CAN_TARGET, DMA_OBJ_MSG_NUM, 1, &TX[0]);

}

static int can0_open(struct inode *inode, struct file *file){
    struct can_device_data *data = container_of(inode->i_cdev, struct can_device_data, cdev);
    file->private_data = data;
    return 0;
}

static ssize_t can0_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    //struct can_device_data *data = filp->private_data;
    return count;
}

static const struct file_operations can_device_fops = {
    .owner = THIS_MODULE,
    .open = can0_open,
    .write = can0_write,
};

static int can0_probe(struct platform_device *pdev)
{
    struct can_device_data *data;
    int ret;
    u32 can_ctl;

    data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    platform_set_drvdata(pdev, data);

    data->base = ioremap(CAN0_BASE, 0x1000);
    data->base_gpio = ioremap(GPIO_BASE, 0x1000);
    data->base_edma = ioremap(EDMA_BASE, 0x8000);

    data->dev = &pdev->dev;
    data->increment = 0;
    data->is_scheduled = 0;
    atomic_set(&data->should_stop, 0); // Initialize to 0 (false)

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

    // ===== Create character device
    ret = alloc_chrdev_region(&data->dev_num, 0, 1, DRIVER_NAME);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to allocate chrdev region: %d\n", ret);
        return ret;
    }

    // ====== Can init
    GPIO_init(data);
    can0_init(data);
    can0_INT_init(data);
#if(DMA_USED)
    can0_DMA_init(data);

    /* If using DMA, we need to update Tx for the next operation tx
       as we set initial TX not by DMA, then we need to update TX here
       -> when interrupt (rx - remote frame) is triggered. DMA will trigger the transfer
       of this updated TX below */
    memcpy(TX_print, TX, SIZE_REQUESTED);
    TX[0]++;
    TX[1]++;
    TX[2]++;
    TX[3]++;
    data->tmpBuffer[0] = (TX[3] << 24) | (TX[2] << 16) | (TX[1] << 8) | (TX[0] << 0);
#endif    

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

    spin_lock_init(&data->mylock);

    /* =============== DMA ================= */

#if(DMA_USED)
    DMA_probe(data);
    data->byte_num = 4;
#endif

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

    /* Release CAN from init */
    can_ctl = ioread32(data->base + CAN_CTL);
    // clear init, CCE
    can_ctl &=~ (CAN_CTL_INIT | CAN_CTL_CCE);
    iowrite32(can_ctl, data->base + CAN_CTL); // enter init mode, access to registers
    //while((ioread32(data->base + CAN_CTL)&CAN_CTL_INIT) != CAN_CTL_INIT); // wait init = 0;
    wait_register_update(data, CAN_CTL, CAN_CTL_INIT_OFFSET, 0, MS_DELAY, "Normal mode");

    return 0;
}

static int can0_remove(struct platform_device *pdev)
{
    struct can_device_data *data = platform_get_drvdata(pdev);
    u32 can_ctl;

    dev_info(data->dev, "Removed\n");

    //Reset_msg_obj(data, WR, DMA_OBJ_MSG_NUM, 0);
    iowrite32(0, data->base + CAN_CTL);

    atomic_set(&data->should_stop, 1); // Set to 1 (true)
    smp_mb(); // Memory barrier to ensure should_stop is visible

#if(DMA_USED)
    if (data->if1_chan){
        dmaengine_terminate_sync(data->if1_chan);
        dma_release_channel(data->if1_chan);
    }
#endif

    can_ctl = CAN_CTL_INIT | CAN_CTL_CCE | CAN_CTL_SWR ; // CAN_CTL_DAR is used to disable auto retransmission
    iowrite32(can_ctl, data->base + CAN_CTL); // enter init mode, access to registers
    wait_register_update(data, CAN_CTL, CAN_CTL_INIT_OFFSET, 1, MS_DELAY, "INIT mode");
    wait_register_update(data, CAN_CTL, CAN_CTL_SWR_OFFSET, 0, MS_DELAY, "Reset mode");

    clk_disable_unprepare(data->clk);
    
    if (data->dev)
        device_destroy(data->class, data->dev_num);
    if (data->class)
        class_destroy(data->class);
    cdev_del(&data->cdev);

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
MODULE_DESCRIPTION("Custom CAN Device Driver for BeagleBone Black CAN0");

