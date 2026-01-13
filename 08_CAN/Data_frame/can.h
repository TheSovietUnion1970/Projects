#ifndef CAN01_H
#define CAN01_H

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
#include "can.h"
#include "main.h"

#define DRIVER_NAME "can0_driver"
#define DEVICE_NAME "can0"

#define MAX_BUFFER_LEN 8

#define RD 0
#define WR 1

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

/* IF1 */
#define CAN_IF1CMD 0x100
#define CAN_IF1MSK 0x104
#define CAN_IF1ARB 0x108
#define CAN_IF1MCTL 0x10C
#define CAN_IF1DATA 0x110
#define CAN_IF1DATB 0x114

/* IF2 */
#define CAN_IF2CMD 0x120
#define CAN_IF2MSK 0x124
#define CAN_IF2ARB 0x128
#define CAN_IF2MCTL 0x12C
#define CAN_IF2DATA 0x130
#define CAN_IF2DATB 0x134

#define CAN_INTPND_X 0xAC
#define CAN_MSGVAL12 0xC4
#define CAN_MSGVAL34 0xC8
#define CAN_MSGVAL56 0xCC
#define CAN_MSGVAL78 0xD0
#define CAN_NWDAT_X 0x98
#define CAN_NWDAT12 0x9C
#define CAN_TXRQ_X 0x84
#define CAN_TXRQ12 0x88
#define CAN_INTPND12 0xB0
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
#define CAN_CTL_SWR BIT(15)
#define CAN_CTL_SWR_OFFSET 15

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
#define CAN_IFxMCTL_IntPnd_OFFSET 13

#define CAN_IFxDATA_0 0xff // [7:0]
#define CAN_IFxDATA_1 0xff00 // [15:8]
#define CAN_IFxDATA_2 0xff0000 // [23:16]
#define CAN_IFxDATA_3 0xff000000 // [31:24]

struct can_device_data {

    dev_t dev_num;
    struct cdev cdev;
    struct class *class;
    struct device *dev;

    void __iomem *base;  // Mapped base address of can0 registers
    void __iomem *base_gpio;
    void __iomem *base_edma;     /* edma registers */
    struct clk *clk;

    struct work_struct re_request_work;
    bool is_scheduled;
    atomic_t should_stop; // Use atomic_t instead of bool

    int irq;

    u8 increment;
    u32 count_many;

    /* DMA-related fields */
    struct dma_chan *if1_chan;
    struct completion if1_completion;
    struct dma_chan *if2_chan;
    struct completion if2_completion;
    bool use_dma; /* Flag to indicate if DMA is used */
    struct clk *edma_clk;
    int dma_channel_tx;
    int dma_channel_rx;
    u8 dma_buffer_tx[MAX_BUFFER_LEN];
    u8* dma_buffer_rx; // use dma_alloc_coherent
    dma_addr_t dma_buffer_phys;
    u8 byte_num_tx;
    u8 byte_num_rx;
    bool is_DMAtriggered;
};

/* functions */
int dma_param_set_tx(struct can_device_data* data, int ch, u8 byte_num, dma_addr_t dma_src_addr);
int dma_param_set_rx(struct can_device_data* data, int ch, u8 byte_num, dma_addr_t dma_dst_addr);

void Dma_read(struct can_device_data* data);
void Clear_rx_flag(struct can_device_data* data);
void Reading_received_msg(struct can_device_data *data, u8 msg_num);
void can0_transmit_obj_Data_Frames_tx(struct can_device_data *data, u8 can_id, u8 size, u8 msg_num, u8 msg_handler);
void Set_MsgNum_cmd(struct can_device_data *data, u8 msgnum);
void Clear_TxRqst_mctl(struct can_device_data *data);

/* Init funcs */
void can0_init_tx(struct can_device_data *data);
void can0_init_rx(struct can_device_data *data);
void GPIO_init(struct can_device_data *data);
void can0_INT_init(struct can_device_data *data);
#if(DMA_USED)
void DMA_probe(struct can_device_data *data);
void can0_DMA_init(struct can_device_data *data);
#endif /* DMA_USED */
void can0_enter_mode(struct can_device_data *data);
void can0_enter_done(struct can_device_data *data);
#endif