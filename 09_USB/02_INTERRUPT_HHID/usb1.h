#ifndef U1_H
#define U1_H

#include <linux/module.h>
#include <linux/fs.h> // alloc_chrdev_region
#include <linux/pci.h> // ioremap
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/delay.h>

/* CONTROL */
#define INDEX_USED 1

/* ============= CONTROL MODULE ===================== */
#define CONTROL_MODULE 0x44e10000
#define USB_CTRL1 0x628
#define USB_WKUP 0x648

#define USBPHY_CM_PWRDN		(1 << 0)
#define USBPHY_OTG_PWRDN	(1 << 1)
#define USBPHY_CHGDET_DIS	(1 << 2)
#define USBPHY_CHGDET_RSTRT	(1 << 3)
#define USBPHY_SRCONDM		(1 << 4)
#define USBPHY_SINKONDP		(1 << 5)
#define USBPHY_CHGISINK_EN	(1 << 6)
#define USBPHY_CHGVSRC_EN	(1 << 7)
#define USBPHY_DMPULLUP		(1 << 8)
#define USBPHY_DPPULLUP		(1 << 9)
#define USBPHY_CDET_EXTCTL	(1 << 10)
#define USBPHY_GPIO_MODE	(1 << 12)
#define USBPHY_DPOPBUFCTL	(1 << 13)
#define USBPHY_DMOPBUFCTL	(1 << 14)
#define USBPHY_DPINPUT		(1 << 15)
#define USBPHY_DMINPUT		(1 << 16)
#define USBPHY_DPGPIO_PD	(1 << 17)
#define USBPHY_DMGPIO_PD	(1 << 18)
#define USBPHY_OTGVDET_EN	(1 << 19)
#define USBPHY_OTGSESSEND_EN	(1 << 20)
#define USBPHY_DATA_POLARITY	(1 << 23)

/* ============= USBSS ===================== */
#define BASE_USBSS 0x47400000
#define USBSS_IRQSTAT 0x28

/* USB1 CTL */
#define BASE_USB1CTL 0x47401800

#define USB1CTL_REV 0x00
#define USB1CTL_IRQST0 0x30
#define USB1CTL_IRQST1 0x34
#define USB1CTL_IRQEN0 0x38
#define USB1CTL_IRQEN1 0x3c
#define USB1CTL_IRQCLR0 0x40
#define USB1CTL_IRQCLR1 0x44
#define USB1CTL_CTRL 0x14
#define USB1CTL_EPINT_CLR 0x40
#define USB1CTL_COREINT_CLR 0x44
#define USB1CTL_UTMI 0xe0
#define USB1CTL_MODE 0xe8

/* USB1 PHY */
#define BASE_USB1PHY 0x47401b00

/* ============= USB1 CORE ================= */
#define BASE_USB1CORE 0x47401c00

/* POWER */
#define MUSB_POWER_ISOUPDATE	0x80
#define MUSB_POWER_SOFTCONN	0x40
#define MUSB_POWER_HSENAB	0x20
#define MUSB_POWER_HSMODE	0x10
#define MUSB_POWER_RESET	0x08
#define MUSB_POWER_RESUME	0x04
#define MUSB_POWER_SUSPENDM	0x02
#define MUSB_POWER_ENSUSPEND	0x01

/* DEVCTL */
#define MUSB_DEVCTL_BDEVICE	0x80
#define MUSB_DEVCTL_FSDEV	0x40
#define MUSB_DEVCTL_LSDEV	0x20
#define MUSB_DEVCTL_VBUS	0x18
#define MUSB_DEVCTL_VBUS_SHIFT	3
#define MUSB_DEVCTL_HM		0x04
#define MUSB_DEVCTL_HR		0x02
#define MUSB_DEVCTL_SESSION	0x01

/* BABBLE_CTL */
#define MUSB_BABBLE_FORCE_TXIDLE	0x80
#define MUSB_BABBLE_SW_SESSION_CTRL	0x40
#define MUSB_BABBLE_STUCK_J		0x20
#define MUSB_BABBLE_RCV_DISABLE		0x04

/* TESTMODE */
#define MUSB_TEST_FORCE_HOST	0x80
#define MUSB_TEST_FIFO_ACCESS	0x40
#define MUSB_TEST_FORCE_FS	0x20
#define MUSB_TEST_FORCE_HS	0x10
#define MUSB_TEST_PACKET	0x08
#define MUSB_TEST_K		0x04
#define MUSB_TEST_J		0x02
#define MUSB_TEST_SE0_NAK	0x01

/*
 * Common USB registers (USB1_CORE)
 */

#define MUSB_FADDR		0x00	/* 8-bit */
#define MUSB_POWER		0x01	/* 8-bit */

#define MUSB_INTRTX		0x02	/* 16-bit */
#define MUSB_INTRRX		0x04
#define MUSB_INTRTXE		0x06
#define MUSB_INTRRXE		0x08
#define MUSB_INTRUSB		0x0A	/* 8 bit */
#define MUSB_INTRUSBE		0x0B	/* 8 bit */
#define MUSB_FRAME		0x0C
#define MUSB_INDEX		0x0E	/* 8 bit */
#define MUSB_TESTMODE		0x0F	/* 8 bit */

#define MUSB_DEVCTL		0x60	/* 8 bit */
#define MUSB_BABBLE_CTL		0x61	/* 8 bit */
#define MUSB_TXFIFOSZ		0x62	/* 8-bit (see masks) */
#define MUSB_RXFIFOSZ		0x63	/* 8-bit (see masks) */
#define MUSB_TXFIFOADD		0x64	/* 16-bit offset shifted right 3 */
#define MUSB_RXFIFOADD		0x66	/* 16-bit offset shifted right 3 */


/* ============ ENDPOINT registers =============== */
#define USB1EP0_base    0x47401D00 // if non_index is used

/* Offsets to endpoint registers */
#define MUSB_TXMAXP		0x00
#define MUSB_TXCSR		0x02
#define MUSB_CSR0		MUSB_TXCSR	/* Re-used for EP0 */
#define MUSB_RXMAXP		0x04
#define MUSB_RXCSR		0x06
#define MUSB_RXCOUNT		0x08
#define MUSB_COUNT0		MUSB_RXCOUNT	/* Re-used for EP0 */
#define MUSB_TXTYPE		0x0A
#define MUSB_TYPE0		MUSB_TXTYPE	/* Re-used for EP0 */
#define MUSB_TXINTERVAL		0x0B
#define MUSB_NAKLIMIT0		MUSB_TXINTERVAL	/* Re-used for EP0 */
#define MUSB_RXTYPE		0x0C
#define MUSB_RXINTERVAL		0x0D
#define MUSB_FIFOSIZE		0x0F
#define MUSB_CONFIGDATA		MUSB_FIFOSIZE	/* Re-used for EP0 */

/* CSR0 */
#define MUSB_CSR0_FLUSHFIFO	0x0100
#define MUSB_CSR0_TXPKTRDY	0x0002
#define MUSB_CSR0_RXPKTRDY	0x0001
/* CSR0 in Host mode */
#define MUSB_CSR0_H_DIS_PING		0x0800
#define MUSB_CSR0_H_WR_DATATOGGLE	0x0400	/* Set to allow setting: */
#define MUSB_CSR0_H_DATATOGGLE		0x0200	/* Data toggle control */
#define MUSB_CSR0_H_NAKTIMEOUT		0x0080
#define MUSB_CSR0_H_STATUSPKT		0x0040
#define MUSB_CSR0_H_REQPKT		0x0020
#define MUSB_CSR0_H_ERROR		0x0010
#define MUSB_CSR0_H_SETUPPKT		0x0008
#define MUSB_CSR0_H_RXSTALL		0x0004

/* CONFIGDATA */
#define MUSB_CONFIGDATA_MPRXE		0x80	/* Auto bulk pkt combining */
#define MUSB_CONFIGDATA_MPTXE		0x40	/* Auto bulk pkt splitting */
#define MUSB_CONFIGDATA_BIGENDIAN	0x20
#define MUSB_CONFIGDATA_HBRXE		0x10	/* HB-ISO for RX */
#define MUSB_CONFIGDATA_HBTXE		0x08	/* HB-ISO for TX */
#define MUSB_CONFIGDATA_DYNFIFO		0x04	/* Dynamic FIFO sizing */
#define MUSB_CONFIGDATA_SOFTCONE	0x02	/* SoftConnect */
#define MUSB_CONFIGDATA_UTMIDW		0x01	/* Data width 0/1 => 8/16bits */

/* TXCSR in Peripheral and Host mode */
#define MUSB_TXCSR_AUTOSET		0x8000
#define MUSB_TXCSR_DMAENAB		0x1000
#define MUSB_TXCSR_FRCDATATOG		0x0800
#define MUSB_TXCSR_DMAMODE		0x0400
#define MUSB_TXCSR_CLRDATATOG		0x0040
#define MUSB_TXCSR_FLUSHFIFO		0x0008
#define MUSB_TXCSR_FIFONOTEMPTY		0x0002
#define MUSB_TXCSR_TXPKTRDY		0x0001
#define MUSB_TXCSR_MODE		0x2000

/* TXCSR in Host mode */
#define MUSB_TXCSR_H_WR_DATATOGGLE	0x0200
#define MUSB_TXCSR_H_DATATOGGLE		0x0100
#define MUSB_TXCSR_H_NAKTIMEOUT		0x0080
#define MUSB_TXCSR_H_RXSTALL		0x0020
#define MUSB_TXCSR_H_ERROR		0x0004

#define MUSB_TXCSR_H_WZC_BITS	\
	(MUSB_TXCSR_H_NAKTIMEOUT | MUSB_TXCSR_H_RXSTALL \
	| MUSB_TXCSR_H_ERROR | MUSB_TXCSR_FIFONOTEMPTY)

/* RXCSR in Host mode */
#define MUSB_RXCSR_H_AUTOREQ		0x4000
#define MUSB_RXCSR_H_WR_DATATOGGLE	0x0400
#define MUSB_RXCSR_H_DATATOGGLE		0x0200
#define MUSB_RXCSR_H_RXSTALL		0x0040
#define MUSB_RXCSR_H_REQPKT		0x0020
#define MUSB_RXCSR_H_ERROR		0x0004

/* RXCSR in Peripheral and Host mode */
#define MUSB_RXCSR_AUTOCLEAR		0x8000
#define MUSB_RXCSR_DMAENAB		0x2000
#define MUSB_RXCSR_DISNYET		0x1000
#define MUSB_RXCSR_PID_ERR		0x1000
#define MUSB_RXCSR_DMAMODE		0x0800
#define MUSB_RXCSR_INCOMPRX		0x0100
#define MUSB_RXCSR_CLRDATATOG		0x0080
#define MUSB_RXCSR_FLUSHFIFO		0x0010
#define MUSB_RXCSR_DATAERROR		0x0008
#define MUSB_RXCSR_FIFOFULL		0x0002
#define MUSB_RXCSR_RXPKTRDY		0x0001

/* RXCSR in Host mode */
#define MUSB_RXCSR_H_AUTOREQ		0x4000
#define MUSB_RXCSR_H_WR_DATATOGGLE	0x0400
#define MUSB_RXCSR_H_DATATOGGLE		0x0200
#define MUSB_RXCSR_H_RXSTALL		0x0040
#define MUSB_RXCSR_H_REQPKT		0x0020
#define MUSB_RXCSR_H_ERROR		0x0004

/* TxType/RxType */
#define MUSB_TYPE_SPEED		0xc0
#define MUSB_TYPE_SPEED_SHIFT	6
#define MUSB_TYPE_PROTO		0x30	/* Implicitly zero for ep0 */
#define MUSB_TYPE_PROTO_SHIFT	4
#define MUSB_TYPE_REMOTE_END	0xf	/* Implicitly zero for ep0 */


/* ============ Address registers (0x80 + (0x08 * epnum) + offset;) =============== */
/* endpoint 0 */

#define MUSB_TXFUNCADDR		0x80
#define MUSB_TXHUBADDR		0x82
#define MUSB_TXHUBPORT		0x83

#define MUSB_RXFUNCADDR		0x84
#define MUSB_RXHUBADDR		0x86
#define MUSB_RXHUBPORT		0x87

/* =============== TOKEN ============ */
#define USB_REQ_GET_STATUS		0x00
#define USB_REQ_CLEAR_FEATURE		0x01
#define USB_REQ_SET_FEATURE		0x03
#define USB_REQ_SET_ADDRESS		0x05
#define USB_REQ_GET_DESCRIPTOR		0x06
#define USB_REQ_SET_DESCRIPTOR		0x07
#define USB_REQ_GET_CONFIGURATION	0x08 
#define USB_REQ_SET_CONFIGURATION	0x09
#define USB_REQ_GET_INTERFACE		0x0A
#define USB_REQ_SET_INTERFACE		0x0B
#define USB_REQ_SYNCH_FRAME		0x0C
#define USB_REQ_SET_SEL			0x30
#define USB_REQ_SET_ISOCH_DELAY		0x31

/* ================ Main structure ============== */
struct usb_devRequest {
	u8 bRequestType;
	u8 bRequest;
	u16 wValue;
	u16 wIndex;
	u16 wLength;

    u16 maxLengthEntryFIFO;
    u16 actualLength;
    u16 leftLength;
};
struct usb_DeviceDescriptor {
    // 18 bytes
	u8 bLength1;
	u8 bDescriptorType1;
	u16 bcdUSB;
	u8 bDeviceClass;
	u8 bDeviceSubClass;
    u8 bDeviceProtocol;
    u8 bMaxPacketSize;
    u16 idVendor;
    u16 idProduct;
    u16 bcdDevice;
    u8 iManufacturer;
    u8 iProduct;
    u8 iSerialNumber;
    u8 bNumConfigurations;

// Configuration Header (bytes 0-8 / offsets 18-26)
    u8 bLength2;              // 0: 0x09
    u8 bDescriptorType2;      // 1: 0x02 (Configuration)
    u16 wTotalLenght;        // 2-3: 0x003E (62)
    u8 bNumInterfaces;       // 4: 0x02 (2 interfaces: Comm + Data)
    u8 bConfigurationValue;  // 5: 0x01 (config #1)
    u8 iConfiguration;       // 6: 0x00 (no string)
    u8 bmAttributes;         // 7: 0xC0 (self-powered, remote wakeup)
    u8 bMaxPower;            // 8: 0x32 (100mA / 2 = 50mA units)


    
    // Interface Descriptor (bytes 9-17 / offsets 27-35)
    u8 bLength3; // 0x09
    u8 bDescriptorType3; // 0x04 (Interface)
    u8 bInterfaceNumber; // 0x00
    u8 bAlternateSetting; // 0x00
    u8 bNumEndpoints; // 0x01 (one interrupt endpoint)
    u8 bInterfaceClass; // 0x03 (HID)
    u8 bInterfaceSubClass; // 0x01 (Boot Interface)
    u8 bInterfaceProtocol; // 0x02 (Mouse)
    u8 iInterface; // 0x00 (no string)

    // HID Descriptor (bytes 18-26 / offsets 36-44)
    u8 bLength4; // 0x09
    u8 bDescriptorType4; // 0x21 (HID)
    u16 bcdHID; // 0x0111 (HID 1.11)
    u8 bCountryCode; // 0x00 (not localized)
    u8 bNumDescriptors; // 0x01 (one class descriptor)
    u8 bDescriptorType5; // 0x22 (Report)
    u16 wDescriptorLength; // Typically 0x0034 (52 bytes for mouse report descriptor)
    
    // Endpoint Descriptor (bytes 27-33 / offsets 45-51)
    u8 bLength5; // 0x07
    u8 bDescriptorType6; // 0x05 (Endpoint)
    u8 bEndpointAddress; // 0x81 (Endpoint 1 IN)
    u8 bmAttributes5; // 0x03 (Interrupt)
    u16 wMaxPacketSize; // 0x0004 or 0x0008 (4 or 8 bytes for mouse reports)
    u8 bInterval; // 0x0A (10ms polling interval)

} __attribute__((packed));  // Prevents padding; total size now 80 bytes;


struct usb_device_data {

    dev_t dev_num;
    struct cdev cdev;
    struct class *class;
    struct device *dev;

    void __iomem *base_usbss;  
    void __iomem *base_usb1ctl;
    void __iomem *base_usb1phy;
    void __iomem *base_usb1core;
    void __iomem *base_usb1ep0;
    void __iomem *base_con_usb1ctrl1;

    struct clk *clk;
    struct usb_devRequest InsReq;

    struct usb_DeviceDescriptor InsDeviceDescriptor;
    u8* DeviceDescriptorPtr;

    // check addr
    u8 isAddrChanged;
    u16 oldAddr;

    int irq1;
    int irqs;

    u8 report_id;
    u8 button;
    int8_t dx; // right+, left-
    int8_t dy; // down+, up-
    int8_t dz; // scroll up +, scroll down -

    bool isEnd;
    u8 RX_index;

    struct work_struct re_request_work;
    bool is_scheduled;
    atomic_t should_stop; // Use atomic_t instead of bool

    u8 count_many;
};

/* ================== Handler ====================*/
irqreturn_t USB1_handler(int irq, void *d);

/* ================== Utils ===================== */
void setIndex(struct usb_device_data *data, u8 epnum);
u32 fifo_offset(u8 epnum);
int wait_register_update(struct usb_device_data *data, void __iomem *mem, u16 offset, u16 bit_offset, u8 bit_val, u16 delay_ms, u8* name_register);
int wait_val_update(struct usb_device_data *data, u16 *var, u16 val, u16 delay_ms, u8* name_val);
void USB1_SetToken(struct usb_device_data *data, const u8* TokenSet);
void USB1_ClrToken(struct usb_device_data *data);
void USB1_ApplyToken(struct usb_device_data *data, u8 epnum);

/* ================ Init funcs ============= */
void USB1_reset(struct usb_device_data *data);
int USB1_init(struct usb_device_data *data);
void PHY1_init(struct usb_device_data *data);
void USB1_exit(struct usb_device_data *data);

/* ================== API for Control Transfer ===================== */
void USB1_Reset_Speed(struct usb_device_data *data);
// int USB1_SETUP_Phase_GetDesc(struct usb_device_data *data, const u8* pkt, u16 addr);
// int USB1_IN_Phase_GetDesc(struct usb_device_data *data);
// int USB1_STATUS_Phase_GetDesc(struct usb_device_data *data);
int USB1_GetDesc_Transfer(struct usb_device_data *data); /* main */

void USB1_Print_DeviceDescriptor(struct usb_device_data *data);
void USB1_Print_DeviceDescriptor2(struct usb_device_data *data);
void USB1_Print_DeviceDescriptorIf0(struct usb_device_data *data);

/* ================== API for Bulk Transfer ===================== */  
int USB1_IN_Phase_Interrupt(struct usb_device_data *data, u8 epnum, u8 addr);
#endif

