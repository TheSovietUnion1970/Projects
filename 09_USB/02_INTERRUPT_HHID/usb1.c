#include <linux/module.h>
#include <linux/init.h>
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
#include <linux/delay.h>
#include "usb1.h"
#include "main.h"

/* ================ Const data packet =======================*/
const u8 GetDesc_pkt[8] = {
    0x80,       // bmRequestType: Device-to-host, Standard, Device
    0x06,       // bRequest: USB_REQ_GET_DESCRIPTOR
    0x00, 0x01, // wValue: Descriptor Index = 0 (LOW), Type = DEVICE (1) (HIGH)
    0x00, 0x00, // wIndex: 0
    0x12, 0x00  // wLength: 18 bytes (Device descriptor length)
};

const u8 GetDesc_pkt2[8] = {
    0x80,       // bmRequestType: Device-to-host, Standard, Device
    0x06,       // bRequest: USB_REQ_GET_DESCRIPTOR
    0x00, 0x02, // wValue: Descriptor Index = 0 (LOW), Type = DEVICE (1) (HIGH)
    0x00, 0x00, // wIndex: 0
    0x09, 0x00  // wLength: 18 bytes (Device descriptor length)
};

const u8 GetDescAll_pkt[8] = {
    0x80,       // bmRequestType: Device-to-host, Standard, 
    0x06,       // bRequest: USB_REQ_GET_DESCRIPTOR
    0x00, 0x02, // wValue: Index=0 (LOW), Type=CONFIGURATION (2) (HIGH) → 0x0200
    0x00, 0x00, // wIndex: 
    0x22, 0x00  // wLength: 34 bytes 
};

const u8 SetAddr_pkt[8] = {
    0x00,       // bmRequestType: Host-to-device, Standard, Device
    0x05,       // bRequest: USB_REQ_SET_ADDRESS
    0x01, 0x00, // wValue: Device address: 1
    0x00, 0x00, // wIndex: 0
    0x00, 0x00  // wLength: 0
};

const u8 GetConf_pkt[8] = {
    0x80,       // bmRequestType: Device-to-host, Standard, Device
    0x08,       // bRequest: USB_REQ_GET_CONFIGURATION
    0x00, 0x01, // wValue: Descriptor Index = 0 (LOW), Type = DEVICE (1) (HIGH)
    0x00, 0x00, // wIndex: 0
    0x01, 0x00  // wLength: 9 bytes (Configuration descriptor length)
};

const u8 SetConf_pkt[8] = {
    0x00,       // bmRequestType: Host-to-Device, Standard, Device
    0x09,       // bRequest: USB_REQ_SET_CONFIGURATION
    0x01, 0x00, // wValue: Configuration value 1 (LOW), 0 (HIGH) → 0x0001
    0x00, 0x00, // wIndex: 0
    0x00, 0x00  // wLength: 0 (no data phase)
};

/* ================== Tmp variables ================= */
u16 Tx1_flag = 0, Rx1_flag = 0;
u8 ret;
u16 count;

/* MAP: indexed register
USBCORE1 0x47401C00:
@1C00-1C0F: faddr(1 byte), power(1), ..., index(1), testmode(1).
@1C10-1C1F: EPx control + status register.
@1C20-....: EP0_FIFO_entry(4)-...
 */

/* ================== Utils for control transfer ===================== */
void setIndex(struct usb_device_data *data, u8 epnum){
    iowrite8(epnum, data->base_usb1core + MUSB_INDEX); // @1C0E
}
void setFifo(struct usb_device_data *data){
    u8 babblectl = 0;

    // TX
    iowrite8(3, data->base_usb1core + MUSB_TXFIFOSZ); // sz = 3 -> fifo size = 2^(sz+3) = 64 bytes for TX FIFO0
    iowrite16(0x00, data->base_usb1core + MUSB_TXFIFOADD); 

    // RX
    iowrite8(3, data->base_usb1core + MUSB_RXFIFOSZ); // sz = 3 -> fifo size = 2^(sz+3) = 64 bytes for RX FIFO0
    iowrite16(0x00, data->base_usb1core + MUSB_RXFIFOADD); 

    // fifo type0 (8-bit)
    iowrite8(0x80, data->base_usb1core + 0x10 + MUSB_TYPE0);

    babblectl = ioread8(data->base_usb1core + MUSB_BABBLE_CTL);
    if (babblectl&MUSB_BABBLE_RCV_DISABLE){
		babblectl |= MUSB_BABBLE_SW_SESSION_CTRL;
        iowrite8(babblectl, data->base_usb1core + MUSB_BABBLE_CTL);
	}
}
u32 fifo_offset(u8 epnum)
{
	return 0x20 + (epnum * 4); // @1C20
}

void CpyMem(u8* dst, u8* src, u32 len){
    u32 i = 0;
    for (i = 0; i < len; i++){
        dst[i] = src[i];
    }
}

int wait_register_update(struct usb_device_data *data, void __iomem *mem, u16 reg_offset, u16 bit_offset, u8 bit_val, u16 delay_ms, u8* name_register){
    unsigned long timeout;
    timeout = jiffies + msecs_to_jiffies(delay_ms);

    while ((ioread32(mem + reg_offset)&(1u << bit_offset)) != (bit_val << bit_offset))  // Wait register updated
    {
        if (time_after(jiffies, timeout)) {
            dev_err(data->dev, "Timeout %s\n", name_register);
            return -ETIMEDOUT;
        }
        cpu_relax();
    } 

    return 0;
}

int wait_val_update(struct usb_device_data *data, u16* var, u16 val, u16 delay_ms, u8* name_val){
    unsigned long timeout;
    timeout = jiffies + msecs_to_jiffies(delay_ms);

    while (*var != val)  // Wait val updated
    {
        if (time_after(jiffies, timeout)) {
            dev_err(data->dev, "Timeout %s\n", name_val);
            return -ETIMEDOUT;
        }
        cpu_relax();
    } 

    return 0;
}

void USB1_SetToken(struct usb_device_data *data, const u8* TokenSet){
    data->InsReq.bRequestType = TokenSet[0];
    data->InsReq.bRequest = TokenSet[1];
    data->InsReq.wValue = (TokenSet[3] << 8 | TokenSet[2]);
    data->InsReq.wIndex = (TokenSet[5] << 8 | TokenSet[4]);
    data->InsReq.wLength = (TokenSet[7] << 8 | TokenSet[6]);
}

void USB1_ClrToken(struct usb_device_data *data){
    u16 i = 0;

    u8* p = (u8*)(&data->InsReq);
    u16 s = sizeof(data->InsReq);

    for (i = 0; i < s; i++){
        p[i] = 0;
    }
}

void USB1_ApplyToken(struct usb_device_data *data, u8 epnum){
    u32 FIFO0_offset = fifo_offset(epnum);
    u32 val[2]; // entry to FIFO has size of 8 bytes

    val[0] = (data->InsReq.wValue << 16) | (data->InsReq.bRequest << 8) | (data->InsReq.bRequestType);
    val[1] = (data->InsReq.wLength << 16) | (data->InsReq.wIndex);

#if (PRINT_DEBUG)
    printk("val[0] = 0x%x, val[1] = 0x%x\n", val[0], val[1]);
#endif
    iowrite32(val[0], data->base_usb1core + FIFO0_offset);
    iowrite32(val[1], data->base_usb1core + FIFO0_offset);

}

u32 USB1_ReadFIFO(struct usb_device_data *data, u8 epnum){
    u32 FIFO0_offset = fifo_offset(epnum);
    return ioread32(data->base_usb1core + FIFO0_offset);
}

void USB1_WriteDataFIFO(struct usb_device_data *data, u8 epnum, u32 dataX){
    u32 FIFO0_offset = fifo_offset(epnum);
    iowrite32(dataX, data->base_usb1core + FIFO0_offset);
}

/* ================== Handler =================*/
void stop_reset(struct usb_device_data *data){
    u32 power = 0;

    power = ioread32(data->base_usb1core + MUSB_POWER);
    // release reset
    power &=~ MUSB_POWER_RESET;
    iowrite8(power, data->base_usb1core + MUSB_POWER);
}
void reset(struct usb_device_data *data){
    u32 power = 0;

    power = ioread32(data->base_usb1core + MUSB_POWER);
    // release reset
    power |= MUSB_POWER_RESET;
    iowrite8(power, data->base_usb1core + MUSB_POWER);
}

irqreturn_t USB1_handler(int irq, void *d){
    struct usb_device_data *data = d;
    u32 irqst0, irqst1;
    u16 int_tx, int_rx;

    u16 host_csr0 = 0;
    int ret = 0, i = 0;
    u32 tmp[2] = {0}; // holp up to 7 bytes
    u8 dataX[7] = {0};


    irqst0 = ioread32(data->base_usb1ctl + USB1CTL_IRQST0);
    irqst1 = ioread32(data->base_usb1ctl + USB1CTL_IRQST1);
    int_tx = irqst0&0xFF;
    int_rx = (irqst0 >> 16)&0xFE;

    if (irqst0)
        iowrite32(irqst0, data->base_usb1ctl + USB1CTL_IRQST0);
    if (irqst1)
        iowrite32(irqst1, data->base_usb1ctl + USB1CTL_IRQST1);


    if (irqst0){
        if (int_tx){
            Tx1_flag = 1;
#if (PRINT_DEBUG)
            printk("ISR -> TX[0x%x]\n", int_tx);
#endif
        }
        if (int_rx){
            Rx1_flag = 1;
            //printk("ISR -> RX[0x%x]\n", int_rx);

            // // ************* trigger

            // wait for Endpoint 0 interrupt (Data packet)
            ret = wait_val_update(data, &Rx1_flag, 1, 2000, "IN Bulk: Data0/1");
            if (ret < 0) return -1;
            Rx1_flag = 0;

            // wait MUSB_RXCSR_RXPKTRDY
            ret = wait_register_update(data, data->base_usb1core, 0x10 + MUSB_RXCSR, 0, 1, 2000, "MUSB_RXCSR_RXPKTRDY");
            if (ret < 0) return -1;

            //msleep(2000);
            
            count = ioread16(data->base_usb1core + 0x10 + MUSB_RXCOUNT)&0xFFFF;

            host_csr0 = ioread16(data->base_usb1core + 0x10 + MUSB_RXCSR)&0xFF;

            //printk("host_csr0 = 0x%x, count = 0x%x\n", host_csr0, count);
            // Check error
            if (host_csr0&MUSB_RXCSR_H_RXSTALL) {
                dev_info(data->dev, "RXSTALL - DATA\n");
                return -1;
            }
            if (host_csr0&MUSB_RXCSR_H_ERROR) {
                dev_info(data->dev, "ERROR\n"); // the controller has tried to send the required IN token three times without getting any response
                return -1;
            }
            if (host_csr0&MUSB_CSR0_H_NAKTIMEOUT) {
                dev_info(data->dev, "NAK_TIMEOUT\n"); // .... consider later
                return -1;
            } 
            if (host_csr0&MUSB_RXCSR_FIFOFULL) {
                //dev_info(data->dev, "MUSB_RXCSR_FIFOFULL\n"); // .... consider later
                //return -1;
            } 
            if (host_csr0&MUSB_RXCSR_RXPKTRDY){
                //printk("Reading IN BULK with ACKed!\n");

                for (i = 0; i < (count/4); i++){
                    tmp[i] = USB1_ReadFIFO(data, 1);
                }
                if (count%4){
                    tmp[i] = USB1_ReadFIFO(data, 1);
                }
                //printk("i = %d, tmp[0] = 0x%x, tmp[1] = 0x%x, count = %d\n", i, tmp[0], tmp[1], count);

                for (i = 0; i < count; i++){
                    dataX[data->RX_index++] = (tmp[i/4] >> (i%4)*8)&0xFF;
                }

                // fetch data correctly
                data->report_id = dataX[0];
                data->button = dataX[1];
                data->dx = (int8_t)dataX[2];
                data->dy = (int8_t)dataX[3];
                data->dz = (int8_t)dataX[4];

                // clear RXPKTRDY
                host_csr0 = ioread32(data->base_usb1core + 0x10 + MUSB_RXCSR);
                host_csr0 &=~ MUSB_RXCSR_RXPKTRDY; 
                iowrite32(host_csr0, data->base_usb1core + 0x10 + MUSB_RXCSR);
            } 

            printk("report_id = 0x%x, button = 0x%x, dx = %d, dy = %d, dz = %d\n", data->report_id, data->button, data->dx, data->dy, data->dz);

            // ************* trigger for the next time
            iowrite16(MUSB_RXCSR_H_REQPKT, data->base_usb1core + 0x10 + MUSB_RXCSR);
            data->RX_index = 0;

        }
    }
    if (irqst1){
        if ((irqst1)&(1u << 7)){
            printk("< VBUS valid threshold>\n");
        } 
        if ((irqst1)&(1u << 2)){
            printk("Babble detected\n");
        } 
        if ((irqst1)&(1u << 4)){
            printk("Device connected\n");
        }   
        if ((irqst1)&(1u << 5)){
            printk("Device disconnected\n");
        }     
        if ((irqst1)&(1u << 8)){
            printk("DRVVBUS level change\n");
        }   
        if ((irqst1)&(1u << 6)){
            printk("SRF ...\n");
        }    
    }

    //data->count_many++;
    if (data->count_many > 100){
        printk("Too many interrupts\n");
        data->count_many = 0;

        USB1_reset(data);
    }

    return IRQ_HANDLED;
}

/* ================ Init funcs ============= */
void USB1_reset(struct usb_device_data *data){
    iowrite32((1u << 0) | (1u << 5), data->base_usb1ctl + USB1CTL_CTRL); // soft reset + isolation
    wait_register_update(data, data->base_usb1ctl, USB1CTL_CTRL, 0, 0, 2000, "RESET"); // wait reset
    wait_register_update(data, data->base_usb1ctl, USB1CTL_CTRL, 5, 0, 2000, "RESET ISOLATION"); // wait reset

}

int USB1_init(struct usb_device_data *data){
    u32 usbcore_pwr = 0, usbcore_testmode = 0;
    u8 power = 0;
    power &= 0xf0;

    // disable testmode
    usbcore_testmode = 0x00;
    iowrite32(usbcore_testmode, data->base_usb1core + MUSB_TESTMODE);

    // init high speed
    usbcore_pwr = MUSB_POWER_ISOUPDATE;
    usbcore_pwr &=~ (MUSB_POWER_HSENAB); /* full speed */
    iowrite32(usbcore_pwr, data->base_usb1core + MUSB_POWER);

    // host mode by sw
    iowrite32(1u << 7, data->base_usb1ctl + USB1CTL_MODE); // host mode by sw

    // test mode
    //iowrite8(MUSB_TEST_FORCE_HOST, data->base_usb1core + MUSB_TESTMODE);

    // enable all interrupts after session
    iowrite32(0xFFFEFFFF, data->base_usb1ctl + USB1CTL_IRQEN0);
    iowrite32(0x1F7, data->base_usb1ctl + USB1CTL_IRQEN1);

    iowrite16(0xFFFF, data->base_usb1core + MUSB_INTRTXE); // enable TX ep0 and 15 eps
    iowrite16(0xFFFE, data->base_usb1core + MUSB_INTRRXE); // enable RX 15 eps
    iowrite8(0xF7, data->base_usb1core + MUSB_INTRUSBE); // 

    // session
    iowrite8(MUSB_DEVCTL_SESSION, data->base_usb1core + MUSB_DEVCTL); // When the USB controller go into session, it will assume the role of a host
    ret = wait_register_update(data, data->base_usb1core, MUSB_DEVCTL, 0, MUSB_DEVCTL_SESSION, 2000, "DEVCTL_SESSION"); // wait DEVCTL_SESSION is set to 1
    if (ret < 0) return -1;

    return 0;
}

void PHY1_init(struct usb_device_data *data){
    u32 usb1_ctrl = 0;

    // return 0 if not clocked
    usb1_ctrl = ioread32(data->base_usb1ctl + USB1CTL_REV);
    if (!usb1_ctrl){
        printk("REV = 0 -> error\n");
        return;
    }

    // usb1 ctrl
    usb1_ctrl = ioread32(data->base_con_usb1ctrl1 + USB_CTRL1);
    usb1_ctrl &= ~(USBPHY_CM_PWRDN | USBPHY_OTG_PWRDN | USBPHY_OTGVDET_EN); // power: normal mode, no Vbus detect as host mode
    usb1_ctrl |= USBPHY_OTGSESSEND_EN;

    iowrite32(usb1_ctrl, data->base_con_usb1ctrl1 + USB_CTRL1);

    msleep(1); // Give the PHY ~1ms to complete the power up operation.
}

void USB1_exit(struct usb_device_data *data){
    iowrite32(0x1ff, data->base_usb1ctl + USB1CTL_COREINT_CLR);
    iowrite32(0xfffeffff, data->base_usb1ctl + USB1CTL_EPINT_CLR);
	
    /* disable interrupts */
    iowrite8(0, data->base_usb1core + MUSB_INTRUSBE);
    iowrite16(0, data->base_usb1core + MUSB_INTRTXE);
    iowrite16(0, data->base_usb1core + MUSB_INTRRXE);

    /*  flush pending interrupts */
    iowrite8(0xff, data->base_usb1core + MUSB_INTRUSB);
    iowrite16(0xffff, data->base_usb1core + MUSB_INTRTX);
    iowrite16(0xffff, data->base_usb1core + MUSB_INTRRX);

    iowrite8(0, data->base_usb1core + MUSB_DEVCTL);
}

/* ================== API for Control Transfer ===================== */
/* First, reset */
void USB1_Reset_Speed(struct usb_device_data *data){
    u8 devctl = 0;

    reset(data);
    msleep(5);
    stop_reset(data);

    devctl = ioread8(data->base_usb1core + MUSB_DEVCTL);
    if (devctl&MUSB_DEVCTL_FSDEV) printk("Full speed\n");
    else if (devctl&MUSB_DEVCTL_LSDEV) printk("Low speed\n");
    else printk("Undefined speed\n");

    // set ptr
    data->DeviceDescriptorPtr = (u8*)(&data->InsDeviceDescriptor);
}

/* Second -> getting  */
int USB1_SETUP_Phase_GetDesc(struct usb_device_data *data, const u8* pkt, u16 addr, u8* string){
    u16 host_csr0 = 0;
    int ret = 0;

    setIndex(data, 0);
    setFifo(data);

    // flush FIFO if needed (maybe 5 times)
    iowrite16(MUSB_CSR0_FLUSHFIFO, data->base_usb1core + 0x10 + MUSB_TXCSR);

    USB1_ClrToken(data);
    USB1_SetToken(data, pkt);
    USB1_ApplyToken(data, 0); // Load the 8 bytes of the required Device request command into the Endpoint 0 FIFO

    iowrite16(addr, data->base_usb1core + MUSB_TXFUNCADDR);

    host_csr0 = ioread16(data->base_usb1core + 0x10 + MUSB_CSR0);
    host_csr0 |= MUSB_CSR0_H_SETUPPKT | MUSB_CSR0_TXPKTRDY; // Set SETUPPKT and TXPKTRDY 
    iowrite16(host_csr0, data->base_usb1core + 0x10 + MUSB_CSR0);

    ret = wait_val_update(data, &Tx1_flag, 1, 2000, "SETUP: Token + Data0/1");
    if (ret < 0) return -1;
    Tx1_flag = 0;

    host_csr0 = ioread16(data->base_usb1core + 0x10 + MUSB_CSR0)&0xFF;
    // Check error for sending SETUP stage
    if (host_csr0 == MUSB_CSR0_H_RXSTALL) {
        dev_info(data->dev, "RXSTALL for %s\n", string);
        return -1;
    }
    if (host_csr0 == MUSB_CSR0_H_ERROR) {
        dev_info(data->dev, "ERROR for %s\n", string); // send additional 2 times
        return -1;
    }
    if (host_csr0 == MUSB_CSR0_H_NAKTIMEOUT) {
        dev_info(data->dev, "NAK_TIMEOUT for %s\n", string); // .... consider later
        return -1;
    } 

    // set length for getting descriptor
    data->InsReq.leftLength = data->InsReq.wLength; // 

    // if USB_REQ_GET_DESCRIPTOR -> update ptr to fill device descriptor
    if (data->InsReq.bRequest == USB_REQ_GET_DESCRIPTOR) {
        // get the whole descriptor, update ptr at the start + 0x12
        if (data->InsReq.wLength == 0x3e) {
            // as it starts sends data from Device descriptor (bLength2 = 0x9)
            data->DeviceDescriptorPtr = (u8*)(&data->InsDeviceDescriptor) + 0x12; // reset ptr
        }
    }
    return 0;
}

int USB1_IN_Phase_GetDesc(struct usb_device_data *data){
    u16 host_csr0 = 0;
    int ret = 0;
    u32 tmp[2];

    iowrite16(MUSB_CSR0_H_REQPKT, data->base_usb1core + 0x10 + MUSB_CSR0);

    // wait for Endpoint 0 interrupt (Data packet)
    ret = wait_val_update(data, &Tx1_flag, 1, 2000, "IN: Data0/1");
    if (ret < 0) return -1;
    Tx1_flag = 0;
    
    count = ioread16(data->base_usb1core + 0x10 + MUSB_COUNT0)&0xFFFF;
    // printk("ret = 0x%x, count = 0x%x\n", ret, count);

    host_csr0 = ioread16(data->base_usb1core + 0x10 + MUSB_CSR0)&0xFF;
    // Check error
    if (host_csr0 == MUSB_CSR0_H_RXSTALL) {
        dev_info(data->dev, "RXSTALL - DATA\n");
        return -1;
    }
    else if (host_csr0 == MUSB_CSR0_H_ERROR) {
        dev_info(data->dev, "ERROR\n"); // the controller has tried to send the required IN token three times without getting any response
        return -1;
    }
    else if (host_csr0 == MUSB_CSR0_H_NAKTIMEOUT) {
        dev_info(data->dev, "NAK_TIMEOUT\n"); // .... consider later
        return -1;
    } 
    else if (host_csr0 == MUSB_CSR0_RXPKTRDY) {
        //printk("Reading IN with ACKed!\n");
        tmp[0] = USB1_ReadFIFO(data, 0);
        tmp[1] = USB1_ReadFIFO(data, 0);

        if (data->oldAddr != ioread16(data->base_usb1core + MUSB_TXFUNCADDR)) data->isAddrChanged = 1;
        // if
        if (data->isAddrChanged)
        {
            //printk("CHANGEDDDDD ADDR, addr 3 = 0x%x\n", &data->InsDeviceDescriptor.bLength3);
            data->DeviceDescriptorPtr = (u8*)(&data->InsDeviceDescriptor);

            data->oldAddr = ioread16(data->base_usb1core + MUSB_TXFUNCADDR);
            data->isAddrChanged = 0;

        }

        if (data->InsReq.wLength == 0x22) {
            data->DeviceDescriptorPtr = (u8*)(&data->InsDeviceDescriptor) + 0x12;

            data->InsReq.wLength = 0;
        }
        

        if (count == 8) {
            CpyMem(data->DeviceDescriptorPtr, (u8*)&tmp[0], 4);
            data->DeviceDescriptorPtr+=4;
            CpyMem(data->DeviceDescriptorPtr, (u8*)&tmp[1], 4);
            data->DeviceDescriptorPtr+=4;
        }
        else if (count <= 4){
            CpyMem(data->DeviceDescriptorPtr, (u8*)&tmp[0], count);
            data->DeviceDescriptorPtr+=(count);
        }
        else {
            CpyMem(data->DeviceDescriptorPtr, (u8*)&tmp[0], 4);
            data->DeviceDescriptorPtr+=4;
            CpyMem(data->DeviceDescriptorPtr, (u8*)&tmp[1], count - 4);
            data->DeviceDescriptorPtr+=(count - 4);
        }

        //printk("0x%x 0x%x, count = %d, data->DeviceDescriptorPtr = 0x%x\n", tmp[0], tmp[1], count, data->DeviceDescriptorPtr);
        //printk("Addr -> 0x%x, 0x%x\n", &data->InsDeviceDescriptor.bLength1, &data->InsDeviceDescriptor.bLength2);


        // clear RXPKTRDY
        host_csr0 = ioread32(data->base_usb1core + 0x10 + MUSB_CSR0);
        host_csr0 &=~ MUSB_CSR0_RXPKTRDY; 
        iowrite32(host_csr0, data->base_usb1core + 0x10 + MUSB_CSR0);
    } 
      
    // handle Length
    data->InsReq.maxLengthEntryFIFO = count; // usually 8 bytes for endpoint 0
    if (data->InsReq.leftLength < 8){
        data->InsReq.leftLength = 0;
    } 
    else {
        data->InsReq.leftLength = data->InsReq.leftLength - data->InsReq.maxLengthEntryFIFO;
    }
    return 0;
}

int USB1_STATUS_Phase_GetDesc(struct usb_device_data *data){
    u16 host_csr0 = 0;

    host_csr0 = ioread16(data->base_usb1core + 0x10 + MUSB_CSR0);
    host_csr0 |= MUSB_CSR0_H_STATUSPKT | MUSB_CSR0_TXPKTRDY; // Set STATUSPKT and TXPKTRDY 
    iowrite16(host_csr0, data->base_usb1core + 0x10 + MUSB_CSR0);

    ret = wait_val_update(data, &Tx1_flag, 1, 2000, "SETUP: Token + Data0/1");
    if (ret < 0) return -1;
    Tx1_flag = 0;

    host_csr0 = ioread16(data->base_usb1core + 0x10 + MUSB_CSR0)&0xFF;
    // Check error for sending SETUP stage
    if (host_csr0 == MUSB_CSR0_H_RXSTALL) {
        dev_info(data->dev, "RXSTALL\n");
        return -1;
    }
    if (host_csr0 == MUSB_CSR0_H_ERROR) {
        dev_info(data->dev, "ERROR\n"); // send additional 2 times
        return -1;
    }
    if (host_csr0 == MUSB_CSR0_H_NAKTIMEOUT) {
        dev_info(data->dev, "NAK_TIMEOUT\n"); // .... consider later
        return -1;
    } 
    return 0;
}

/* Third -> setting */
int USB1_SETUP_Phase_SetAddr(struct usb_device_data *data, const u8* pkt, u16 addr, u8* string){
    u16 host_csr0 = 0;
    int ret = 0;

    setIndex(data, 0);
    setFifo(data);

    // flush FIFO if needed (maybe 5 times)
    iowrite16(MUSB_CSR0_FLUSHFIFO, data->base_usb1core + 0x10 + MUSB_TXCSR);

    USB1_ClrToken(data);
    USB1_SetToken(data, pkt);
    USB1_ApplyToken(data, 0); // Load the 8 bytes of the required Device request command into the Endpoint 0 FIFO

    iowrite16(addr, data->base_usb1core + MUSB_TXFUNCADDR);
    iowrite16(addr, data->base_usb1core + MUSB_RXFUNCADDR);

    host_csr0 = ioread16(data->base_usb1core + 0x10 + MUSB_CSR0);
    host_csr0 |= MUSB_CSR0_H_SETUPPKT | MUSB_CSR0_TXPKTRDY; // Set SETUPPKT and TXPKTRDY 
    iowrite16(host_csr0, data->base_usb1core + 0x10 + MUSB_CSR0);

    ret = wait_val_update(data, &Tx1_flag, 1, 2000, "SETUP: Token + Data0/1 vvv");
    if (ret < 0) return -1;
    Tx1_flag = 0;

    host_csr0 = ioread16(data->base_usb1core + 0x10 + MUSB_CSR0)&0xFF;
    // Check error for sending SETUP stage
    if (host_csr0 == MUSB_CSR0_H_RXSTALL) {
        dev_info(data->dev, "RXSTALL for %s\n", string);
        return -1;
    }
    if (host_csr0 == MUSB_CSR0_H_ERROR) {
        dev_info(data->dev, "ERROR for %s\n", string); // send additional 2 times
        return -1;
    }
    if (host_csr0 == MUSB_CSR0_H_NAKTIMEOUT) {
        dev_info(data->dev, "NAK_TIMEOUT for %s\n", string); // .... consider later
        return -1;
    } 

    // set length for getting descriptor
    data->InsReq.leftLength = data->InsReq.wLength; // 0 bytes
    //data->DeviceDescriptorPtr = (u8*)(&data->InsDeviceDescriptor);

    return 0;
}

int USB1_OUT_Phase_GetDesc(struct usb_device_data *data, const u8* dataX, u32 len){
    u16 host_csr0 = 0;
    int ret = 0, i = 0;
    u32 tmp[2] = {0};

    for (i = 0; i < len; i++){
        if (i < 4) tmp[0] |= (dataX[i] << (i*8));
        else tmp[1] |= (dataX[i] << ((i - 4)*8));
    }
    USB1_WriteDataFIFO(data, 0, tmp[0]);
    USB1_WriteDataFIFO(data, 0, tmp[1]);
#if (PRINT_DEBUG)
    printk("0x%x  0x%x\n", tmp[0], tmp[1]);
#endif
    iowrite16(MUSB_CSR0_TXPKTRDY, data->base_usb1core + 0x10 + MUSB_CSR0);

    // wait for Endpoint 0 interrupt (Data packet)
    ret = wait_val_update(data, &Tx1_flag, 1, 2000, "IN: Data0/1");
    if (ret < 0) return -1;
    Tx1_flag = 0;
    
    count = ioread16(data->base_usb1core + 0x10 + MUSB_COUNT0)&0xFFFF;
    // printk("ret = 0x%x, count = 0x%x\n", ret, count);

    host_csr0 = ioread16(data->base_usb1core + 0x10 + MUSB_CSR0)&0xFF;
    // Check error
    if (host_csr0 == MUSB_CSR0_H_RXSTALL) {
        dev_info(data->dev, "RXSTALL - DATA\n");
        return -1;
    }
    else if (host_csr0 == MUSB_CSR0_H_ERROR) {
        dev_info(data->dev, "ERROR\n"); // the controller has tried to send the required IN token three times without getting any response
        return -1;
    }
    else if (host_csr0 == MUSB_CSR0_H_NAKTIMEOUT) {
        dev_info(data->dev, "NAK_TIMEOUT\n"); // .... consider later
        return -1;
    } 
     
    return 0;
}

int USB1_STATUS_Phase_SetAddr(struct usb_device_data *data){
    u16 host_csr0 = 0;
    int ret = 0;

    host_csr0 = ioread16(data->base_usb1core + 0x10 + MUSB_CSR0);
    host_csr0 = MUSB_CSR0_H_STATUSPKT | MUSB_CSR0_H_REQPKT; // Set STATUSPKT and TXPKTRDY 
    iowrite16(host_csr0, data->base_usb1core + 0x10 + MUSB_CSR0);

    ret = wait_val_update(data, &Tx1_flag, 1, 2000, "SETUP: Token + Data0/1 yyy");
    Tx1_flag = 0;
    if (ret < 0) {
        return -1;
    }

    host_csr0 = ioread16(data->base_usb1core + 0x10 + MUSB_CSR0)&0xFF;
    //printk("ret = %x\n", host_csr0);
    // Check error for sending SETUP stage
    if (host_csr0 == MUSB_CSR0_H_RXSTALL) {
        dev_info(data->dev, "RXSTALL\n");
        return -1;
    }
    if (host_csr0 == MUSB_CSR0_H_ERROR) {
        dev_info(data->dev, "ERROR\n"); // send additional 2 times
        return -1;
    }
    if (host_csr0 == MUSB_CSR0_H_NAKTIMEOUT) {
        dev_info(data->dev, "NAK_TIMEOUT\n"); // .... consider later
        return -1;
    } 
    else if (host_csr0&MUSB_CSR0_RXPKTRDY) {
        //dev_info(data->dev, "RXPKTRDY - read FIFO\n"); 
#if (PRINT_DEBUG)
        printk("Sending STATUS with ACKed!\n");
#endif
        // clear RXPKTRDY
        host_csr0 = ioread32(data->base_usb1core + 0x10 + MUSB_CSR0);
        host_csr0 &=~ MUSB_CSR0_RXPKTRDY; 
        iowrite32(host_csr0, data->base_usb1core + 0x10 + MUSB_CSR0);
    } 
    return 0;
}

/* Read + Write */
int USB1_READ_Transaction(struct usb_device_data *data, const u8* pkt, u16 addr, u8* string){

    int ret = 0;
    ret = USB1_SETUP_Phase_GetDesc(data, pkt, addr, string);

    if (ret == 0){
        do {
            ret = USB1_IN_Phase_GetDesc(data);
        } while(data->InsReq.leftLength && (ret >= 0));
    }

    if (ret == 0){
        ret = USB1_STATUS_Phase_GetDesc(data);
    }

    return ret;
}

int USB1_WRITE_Transaction(struct usb_device_data *data, const u8* pkt, u16 addr, const u8* dataX, u32 len, u8* string){

    int ret = 0;
    ret = USB1_SETUP_Phase_SetAddr(data, pkt, addr, string);

    if (ret == 0){
        if (len) {
            ret = USB1_OUT_Phase_GetDesc(data, dataX, len);
        }
    }   

    if (ret == 0){
        ret = USB1_STATUS_Phase_SetAddr(data);
    }

    return ret;
}

/* Main */
int USB1_GetDesc_Transfer(struct usb_device_data *data){
    int ret;

    /* Getting descriptor with default address 0 */
    ret = USB1_READ_Transaction(data, GetDesc_pkt, 0x0, "GetDesc_pkt");

    /* Setting address 0x1 with default address 0 */
    if (ret == 0){
        ret = USB1_WRITE_Transaction(data, SetAddr_pkt, 0x0, NULL, 0, "SetAddr_pkt");
    }

    /* Getting descriptor with new address 0x1 */
    if (ret == 0){
        ret = USB1_READ_Transaction(data, GetDesc_pkt, 0x1, "GetDesc_pkt");
    }

    /* Getting descriptor with new address 0x1 */
    if (ret == 0){
        ret = USB1_READ_Transaction(data, GetDesc_pkt2, 0x1, "GetDesc_pkt2");
    }

    /* Getting all-byte device descriptor with new address 0x1 */
    if (ret == 0){
        ret = USB1_READ_Transaction(data, GetDescAll_pkt, 0x1, "GetDescAll_pkt");
    }

    /* Setting configuration with new address 0x1 */
    if (ret == 0){
        ret = USB1_WRITE_Transaction(data, SetConf_pkt, 0x1, NULL, 0, "SetConf_pkt");
    }

    /* Getting configuration with new address 0x1, data recieved should be 0x1 */
    if (ret == 0){
        ret = USB1_READ_Transaction(data, GetConf_pkt, 0x1, "GetConf_pkt");
    }

    return ret;
}

/* Print result */
void USB1_Print_DeviceDescriptor(struct usb_device_data *data){
    printk("# ----------------------------------- #\n");
    printk("# bLength1 = 0x%x\n", data->InsDeviceDescriptor.bLength1);
    printk("# bDescriptorType1 = 0x%x\n", data->InsDeviceDescriptor.bDescriptorType1);
    printk("# bcdUSB = 0x%x\n", data->InsDeviceDescriptor.bcdUSB);
    printk("# bDeviceClass = 0x%x\n", data->InsDeviceDescriptor.bDeviceClass);
    printk("# bDeviceSubClass = 0x%x\n", data->InsDeviceDescriptor.bDeviceSubClass);
    printk("# bDeviceProtocol = 0x%x\n", data->InsDeviceDescriptor.bDeviceProtocol);
    printk("# bMaxPacketSize = 0x%x\n", data->InsDeviceDescriptor.bMaxPacketSize);
    printk("# idVendor = 0x%x\n", data->InsDeviceDescriptor.idVendor);
    printk("# idProduct = 0x%x\n", data->InsDeviceDescriptor.idProduct);
    printk("# bcdDevice = 0x%x\n", data->InsDeviceDescriptor.bcdDevice);
    printk("# iManufacturer = 0x%x\n", data->InsDeviceDescriptor.iManufacturer);
    printk("# iProduct = 0x%x\n", data->InsDeviceDescriptor.iProduct);
    printk("# iSerialNumber = 0x%x\n", data->InsDeviceDescriptor.iSerialNumber);
    printk("# bNumConfigurations = 0x%x\n", data->InsDeviceDescriptor.bNumConfigurations);
    printk("# ----------------------------------- #\n");
}

void USB1_Print_DeviceDescriptor2(struct usb_device_data *data){
    printk("# --------Device descriptor----------- #\n");
    printk("# bLength2 = 0x%x\n", data->InsDeviceDescriptor.bLength2);
    printk("# bDescriptorType2 = 0x%x\n", data->InsDeviceDescriptor.bDescriptorType2);
    printk("# wTotalLenght = 0x%x\n", data->InsDeviceDescriptor.wTotalLenght);
    printk("# bNumInterfaces = 0x%x\n", data->InsDeviceDescriptor.bNumInterfaces);
    printk("# bConfigurationValue = 0x%x\n", data->InsDeviceDescriptor.bConfigurationValue);
    printk("# iConfiguration = 0x%x\n", data->InsDeviceDescriptor.iConfiguration);
    printk("# bmAttributes = 0x%x\n", data->InsDeviceDescriptor.bmAttributes);
    printk("# bMaxPower = 0x%x\n", data->InsDeviceDescriptor.bMaxPower);
    printk("# ----------------------------------- #\n");
}

void USB1_Print_DeviceDescriptorIf0(struct usb_device_data *data){
    printk("# --------Device descriptor----------- #\n");
    printk("# bLength3 = 0x%x\n", data->InsDeviceDescriptor.bLength3);
    printk("# bDescriptorType3 = 0x%x\n", data->InsDeviceDescriptor.bDescriptorType3);
    printk("# bInterfaceNumber = 0x%x\n", data->InsDeviceDescriptor.bInterfaceNumber);
    printk("# bAlternateSetting = 0x%x\n", data->InsDeviceDescriptor.bAlternateSetting);
    printk("# bNumEndpoints = 0x%x\n", data->InsDeviceDescriptor.bNumEndpoints);
    printk("# bInterfaceClass = 0x%x\n", data->InsDeviceDescriptor.bInterfaceClass);
    printk("# bInterfaceSubClass = 0x%x\n", data->InsDeviceDescriptor.bInterfaceSubClass);
    printk("# bInterfaceProtocol = 0x%x\n", data->InsDeviceDescriptor.bInterfaceProtocol);
    printk("# iInterface = 0x%x\n", data->InsDeviceDescriptor.iInterface);


    printk("\n");
    printk("# bLength4 = 0x%x\n", data->InsDeviceDescriptor.bLength4);
    printk("# bDescriptorType4 = 0x%x\n", data->InsDeviceDescriptor.bDescriptorType4);
    printk("# bcdHID = 0x%x\n", data->InsDeviceDescriptor.bcdHID);
    printk("# bCountryCode = 0x%x\n", data->InsDeviceDescriptor.bCountryCode);
    printk("# bNumDescriptors = 0x%x\n", data->InsDeviceDescriptor.bNumDescriptors);
    printk("# bDescriptorType5 = 0x%x\n", data->InsDeviceDescriptor.bDescriptorType5);
    printk("# wDescriptorLength = 0x%x\n", data->InsDeviceDescriptor.wDescriptorLength);

    printk("\n");
    printk("# bLength5 = 0x%x\n", data->InsDeviceDescriptor.bLength5);
    printk("# bDescriptorType5 = 0x%x\n", data->InsDeviceDescriptor.bDescriptorType5);
    printk("# bEndpointAddress = 0x%x\n", data->InsDeviceDescriptor.bEndpointAddress);
    printk("# bmAttributes5 = 0x%x\n", data->InsDeviceDescriptor.bmAttributes5);
    printk("# wMaxPacketSize = 0x%x\n", data->InsDeviceDescriptor.wMaxPacketSize);
    printk("# bInterval = 0x%x\n", data->InsDeviceDescriptor.bInterval);
    printk("# ----------------------------------- #\n");
}
/* ================== Utils for interrupt transfer ===================== */

/* RX */
void USB1_Interrupt_SetAddrRx(struct usb_device_data *data, u8 addr, u8 epnum){
    iowrite16(addr, data->base_usb1core + MUSB_RXFUNCADDR + (0x08 * epnum));
}
void USB1_Interrupt_SetTypeRx(struct usb_device_data *data, u8 epnum, u8 speed){
    u8 RxType = 0;

    RxType = (speed << 6)&MUSB_TYPE_SPEED;
    RxType |= (0x3 << 4)&MUSB_TYPE_PROTO; // Interrupt type
    RxType |= (epnum << 0)&MUSB_TYPE_REMOTE_END;

    iowrite8(RxType, data->base_usb1core + 0x10 + MUSB_RXTYPE);
}
void USB1_Interrupt_SetRxMaxp(struct usb_device_data *data, u16 maxp){
    iowrite16(maxp, data->base_usb1core + 0x10 + MUSB_RXMAXP);
}
void USB1_Interrupt_SetRxInterval(struct usb_device_data *data, u8 rxInterval){
    iowrite8(rxInterval, data->base_usb1core + 0x10 + MUSB_RXINTERVAL);
}


/* ================== API for Interrupt Transfer ===================== */
int USB1_IN_Phase_Interrupt(struct usb_device_data *data, u8 epnum, u8 addr){
    u16 rxcsr = 0;

    setIndex(data, epnum); 
    USB1_Interrupt_SetAddrRx(data, addr, epnum);
    USB1_Interrupt_SetTypeRx(data, epnum, 0x2); // 0x2 = full speed
    USB1_Interrupt_SetRxMaxp(data, 0x08); // max packet is 7 bytes
    USB1_Interrupt_SetRxInterval(data, 0x2); // as bInterval = 0x2

    // RX
    iowrite8(3, data->base_usb1core + MUSB_RXFIFOSZ); // sz = 3 -> fifo size = 2^(sz+3) = 64 bytes for RX FIFO0
    iowrite16(0x00, data->base_usb1core + MUSB_RXFIFOADD); 

    // trigger
    rxcsr = MUSB_RXCSR_H_REQPKT;
    iowrite16(rxcsr, data->base_usb1core + 0x10 + MUSB_RXCSR);

    data->RX_index = 0;

    return 0;
}

