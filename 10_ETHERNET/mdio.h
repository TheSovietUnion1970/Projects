#ifndef E_H
#define E_H

#include <linux/module.h>
#include <linux/fs.h> // alloc_chrdev_region
#include <linux/pci.h> // ioremap
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/workqueue.h>
#include <linux/genalloc.h>
#include <linux/netdevice.h>
// #include "cpdma.h"

#define CPSW_MAX_QUEUES 8

#define PHY_ID0 0
#define MDIO_TIMEOUT		100 /* msecs */

#define BIT_VAL_0 0
#define BIT_VAL_1 1

#define CTRMOD_BASE     0x44e10000   
#define CLK_BASE        0x44e00000

#define CPSW_BASE       0x4a100000   // CPSW subsystem base
#define PORT0_BASE      (CPSW_BASE + 0x100)    // Host port registers
#define PORT1_BASE      (CPSW_BASE + 0x200)    // Slave port 0 (eth0)
#define PORT2_BASE      (CPSW_BASE + 0x300)    // Slave port 1 (eth1)
#define CPDMA_BASE      (CPSW_BASE + 0x800)    // CPDMA
#define STATS_BASE      (CPSW_BASE + 0x900)    // Statistics Registers
#define TXHDP_BASE      (CPSW_BASE + 0xA00)    // TXHDP
#define CPTS_BASE       (CPSW_BASE + 0xC00)    // Common Platform Time Sync
#define ALE_BASE        (CPSW_BASE + 0xD00)    // ALE
#define CPSW_SL_BASE    (CPSW_BASE + 0xD80)    // CPSW_SL
#define MDIO_BASE       (CPSW_BASE + 0x1000)   // MDIO block = 0x4a101000
#define CPSW_WR_BASE    (CPSW_BASE + 0x1200)   // WRAPPER (if you need it)
#define CPPIRAM_BASE    (CPSW_BASE + 0x2000)   // CPPI-RAM

/* MDIO command */
#define USERACCESS_GO		BIT(31)
#define USERACCESS_GO_BIT		31
#define USERACCESS_WRITE	BIT(30)
#define USERACCESS_ACK		BIT(29)
#define USERACCESS_ACK_BIT		29
#define USERACCESS_READ		(0)
#define USERACCESS_DATA		(0xffff)

/* MDIO register + offset */
#define MDIO_MDIOVER 0x00
#define MDIO_MDIOCONTROL 0x04
    #define MDIOC_PREAMBLE			1u << 20
    #define MDIOC_CLKDIV(div)		((div) & 0xff)
    #define MDIOC_ENABLE     		1u << 30
    #define CONTROL_IDLE            1u << 31
#define MDIO_MDIOALIVE 0x08
#define MDIO_MDIOUSERACCESS0 0x80
#define MDIO_MDIOUSERPHYSEL0 0x84
#define MDIO_MDIOUSERACCESS1 0x88
#define MDIO_MDIOUSERPHYSEL1 0x8c

/* Generic MII registers. */
#define MII_BMCR		0x00	/* Basic mode control register */
    #define BMCR_ISOLATE        1u << 10
    #define BMCR_RESET          1u << 15
    #define BMCR_SPEED100       1u << 13
    #define BMCR_ANENABLE       1u << 12
    #define BMCR_ANRESTART      1u << 9
    #define BMCR_FULLDPLX       1u << 8
#define MII_BMSR		0x01	/* Basic mode status register  */
    #define BMSR_LINK_UP        1u << 2
#define MII_PHYSID1		0x02	/* PHYS ID 1                   */
#define MII_PHYSID2		0x03	/* PHYS ID 2                   */
#define MII_ADVERTISE		0x04	/* Advertisement control reg   */
    /* Advertisement control register. */
    #define ADVERTISE_SLCT		0x001f	/* Selector bits               */
    #define ADVERTISE_CSMA		0x0001	/* Only selector supported     */
    #define ADVERTISE_10HALF	0x0020	/* Try for 10mbps half-duplex  */
    #define ADVERTISE_1000XFULL	0x0020	/* Try for 1000BASE-X full-duplex */
    #define ADVERTISE_10FULL	0x0040	/* Try for 10mbps full-duplex  */
    #define ADVERTISE_1000XHALF	0x0040	/* Try for 1000BASE-X half-duplex */
    #define ADVERTISE_100HALF	0x0080	/* Try for 100mbps half-duplex */
    #define ADVERTISE_1000XPAUSE	0x0080	/* Try for 1000BASE-X pause    */
    #define ADVERTISE_100FULL	0x0100	/* Try for 100mbps full-duplex */
    #define ADVERTISE_1000XPSE_ASYM	0x0100	/* Try for 1000BASE-X asym pause */
    #define ADVERTISE_100BASE4	0x0200	/* Try for 100mbps 4k packets  */
    #define ADVERTISE_PAUSE_CAP	0x0400	/* Try for pause               */
    #define ADVERTISE_PAUSE_ASYM	0x0800	/* Try for asymetric pause     */
    #define ADVERTISE_RESV		0x1000	/* Unused...                   */
    #define ADVERTISE_RFAULT	0x2000	/* Say we can detect faults    */
    #define ADVERTISE_LPACK		0x4000	/* Ack link partners response  */
    #define ADVERTISE_NPAGE		0x8000	/* Next page bit               */
    #define ADVERTISE_FULL		(ADVERTISE_100FULL | ADVERTISE_10FULL | \
                    ADVERTISE_CSMA)
    #define ADVERTISE_ALL		(ADVERTISE_10HALF | ADVERTISE_10FULL | \
                    ADVERTISE_100HALF | ADVERTISE_100FULL)

#define MII_LAN83C185_ISF 29 /* Interrupt Source Flags */
#define MII_LAN83C185_IM  30 /* Interrupt Mask */
#define MII_LAN83C185_CTRL_STATUS 17 /* Mode/Status Register */
#define MII_LAN83C185_SPECIAL_MODES 18 /* Special Modes Register */

#define MII_LAN83C185_MODE_MASK      0xE0
#define MII_LAN83C185_MODE_POWERDOWN 0xC0 /* Power Down mode */
#define MII_LAN83C185_MODE_ALL       0xE0 /* All capable mode */

#define MII_LAN83C185_ISF_INT1 (1<<1) /* Auto-Negotiation Page Received */
#define MII_LAN83C185_ISF_INT2 (1<<2) /* Parallel Detection Fault */
#define MII_LAN83C185_ISF_INT3 (1<<3) /* Auto-Negotiation LP Ack */
#define MII_LAN83C185_ISF_INT4 (1<<4) /* Link Down */
#define MII_LAN83C185_ISF_INT5 (1<<5) /* Remote Fault Detected */
#define MII_LAN83C185_ISF_INT6 (1<<6) /* Auto-Negotiation complete */
#define MII_LAN83C185_ISF_INT7 (1<<7) /* ENERGYON */

struct cpdma_desc_pool {
	phys_addr_t		phys;
	dma_addr_t		hw_addr;
	void __iomem		*iomap;		/* ioremap map */
	void			*cpumap;	/* dma_alloc map */
	int			desc_size, mem_size;
	int			num_desc;
	struct device		*dev;
	struct gen_pool		*gen_pool;
};

struct cpdma_desc {
	/* hardware fields */
	u32			hw_next;
	u32			hw_buffer;
	u32			hw_len;
	u32			hw_mode;
	/* software fields */
	void			*sw_token;
	u32			sw_buffer;
	u32			sw_len;    
};

struct ether_device_data {
    /* Essential */
    dev_t dev_num;
    struct cdev cdev;
    struct class *class;
    struct device *dev;
    struct device *chardev;
    struct clk *clk;
    struct clk *clk2;
    struct clk *clk3;
    u32 clk_freq;

    /* Lock */
    spinlock_t lock;

    /* Base address */
    void __iomem *base_ctrmod; 
    void __iomem *base_clk; 

    void __iomem *base_cpsw; 
    void __iomem *base_ale; 
    void __iomem *base_cpsw_sl; 
    void __iomem *base_port0; 
    void __iomem *base_port1; 
    void __iomem *base_port2; 
    void __iomem *base_wr; 

    void __iomem *base_cpdma; 
    void __iomem *base_txhdp; 
    void __iomem *base_rxhdp; 
    void __iomem *base_txcp; 
    void __iomem *base_rxcp; 

    void __iomem *base_mdio; 

    struct cpdma_desc_pool *desc_pool;
    struct cpdma_desc *desc_dma_tx; // for tx only
    struct cpdma_desc *desc_dma_rx;

    struct net_device *ndev;
	struct napi_struct		napi_rx;
	struct napi_struct		napi_tx;

    int tx_dma_channel;
    int rx_dma_channel;

    struct page_pool *pool[CPSW_MAX_QUEUES]; // CPSW_MAX_QUEUES
    struct page *page[CPSW_MAX_QUEUES]; // CPSW_MAX_QUEUES
    
    /* scheduled work */
    struct work_struct re_request_work;
    bool is_scheduled;
    atomic_t should_stop; // Use atomic_t instead of bool

    struct delayed_work phy_work; // check phy state
    struct timer_list timer; // remove old MAC entries

    /* ALE params */
    u16 ale_entries;

    /* "rx_thresh", "rx", "tx", "misc" */
    int rx_thresh_irq;
    int rx_irq;
    int tx_irq;
    int misc_irq;
};

int wait_register_update(struct ether_device_data *data, void __iomem *mem, u16 reg_offset, u16 bit_offset, u8 bit_val, u16 delay_ms, u8* name_register);
int wait_val_update(struct ether_device_data *data, u16* var, u16 val, u16 delay_ms, u8* name_val);

int clock_init(struct ether_device_data *data);
int clock_deinit(struct ether_device_data *data);

void gmii_sel_init(struct ether_device_data *data);
int ether_mdio_init(struct ether_device_data* data);

int mdio_read(struct ether_device_data *data, u32 phy_id, u32 phy_reg, u16* dataX);
int mido_write(struct ether_device_data *data, u32 phy_id, u32 phy_reg, u16 dataX);

void ETHER1_Print_Hex(u8 *data, u16 len, u8 *name);

#endif /* E_H */