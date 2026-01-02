#ifndef CPDMA_H
#define CPDMA_H

#include <linux/dma-mapping.h>
#include<linux/interrupt.h>

#include "mdio.h"

#define TX_DMA_CHANNEL 7 // -> interrupt mask: 7
#define RX_DMA_CHANNEL 32 // -> interrupt mask: 0

#define VLAN_HLEN 4

/* VLAN encap header(4) + MAC addr(12) + VLAN header(4) + Type(2) + Payload max(46) + FCS(4)
 = 72 bytes*/
#define CPSW_MIN_PACKET_SIZE 72

 /* VLAN encap header(4) + MAC addr(12) + VLAN header(4) + Type(2) + Payload max(1500) + FCS(4)
 = 1526 bytes*/
#define CPSW_MAX_PACKET_SIZE 1526

#define CPSW_RX_VLAN_ENCAP_HDR_SIZE 4
#define CPSW_RX_VLAN_ENCAP_HDR_PKT_TYPE_SHIFT	8
#define CPSW_RX_VLAN_ENCAP_HDR_PKT_TYPE_MSK	GENMASK(1, 0)
#define CPSW_RX_VLAN_ENCAP_HDR_VID_SHIFT	16
#define VLAN_VID_MASK		0x0fff /* VLAN Identifier */
enum {
	CPSW_RX_VLAN_ENCAP_HDR_PKT_VLAN_TAG = 0,
	CPSW_RX_VLAN_ENCAP_HDR_PKT_RESERV,
	CPSW_RX_VLAN_ENCAP_HDR_PKT_PRIO_TAG,
	CPSW_RX_VLAN_ENCAP_HDR_PKT_UNTAG,
};

#define chan_linear(chan_num)	((chan_num) & (32 - 1))

#define CPSW_BD_RAM_SIZE		0x2000
#define CPSW_CPDMA_DESCS_POOL_SIZE_DEFAULT 256

/* CPSW_CPDMA */
#define CPDMA_RXTHRESH		0x0c0
#define CPDMA_RXFREE		0x0e0

/* CPDMA_STATERAM */
#define CPDMA_TXHDP		0x00
#define CPDMA_RXHDP		0x20
#define CPDMA_TXCP		0x40
#define CPDMA_RXCP		0x60

/* DMA Registers */
#define CPDMA_TXIDVER		0x00
#define CPDMA_TXCONTROL		0x04
#define CPDMA_TXTEARDOWN	0x08
#define CPDMA_RXIDVER		0x10
#define CPDMA_RXCONTROL		0x14
#define CPDMA_SOFTRESET		0x1c
#define CPDMA_RXTEARDOWN	0x18
#define CPDMA_CONTROL   	0x20
    #define TX_PTYPE    1u << 0
#define CPDMA_STATUS       	0x24
#define CPDMA_RX_BUFF_OFFSET       	0x28
#define CPDMA_TX_PRI0_RATE	0x30
#define CPDMA_TXINTSTATRAW	0x80
#define CPDMA_TXINTSTATMASKED	0x84
#define CPDMA_TXINTMASKSET	0x88
#define CPDMA_TXINTMASKCLEAR	0x8c
#define CPDMA_MACINVECTOR	0x90
#define CPDMA_MACEOIVECTOR	0x94
#define CPDMA_RXINTSTATRAW	0xa0
#define CPDMA_RXINTSTATMASKED	0xa4
#define CPDMA_RXINTMASKSET	0xa8
#define CPDMA_RXINTMASKCLEAR	0xac
#define CPDMA_DMAINTSTATRAW	0xb0
#define CPDMA_DMAINTSTATMASKED	0xb4
#define CPDMA_DMAINTMASKSET	0xb8
#define CPDMA_DMAINTMASKCLEAR	0xbc
#define CPDMA_DMAINT_HOSTERR	BIT(1)

#define CPDMA_EOI_RX_THRESH	0x0
#define CPDMA_EOI_RX		0x1
#define CPDMA_EOI_TX		0x2
#define CPDMA_EOI_MISC		0x3

/* WRAPPER */
#define WR_CONTROL          0x08
#define WR_C0_RX_THRESH_EN  0x10
#define WR_C0_RX_EN         0x14
#define WR_C0_TX_EN         0x18
#define WR_C0_MISC_EN       0x1c

/* Descriptor mode bits */
#define CPDMA_DESC_SOP		BIT(31)
#define CPDMA_DESC_EOP		BIT(30)
#define CPDMA_DESC_OWNER	BIT(29)
#define CPDMA_DESC_EOQ		BIT(28)
#define CPDMA_DESC_TO_PORT_EN	BIT(20)

int cpdma_ctlr_start(struct ether_device_data* data);
int cpdma_ctlr_stop(struct ether_device_data* data);
void cpdma_intr_enable(struct ether_device_data* data);
void cpdma_intr_disable(struct ether_device_data* data);

irqreturn_t rx_thresh_handler(int irq, void *dev_id);
irqreturn_t rx_handler(int irq, void *dev_id);
irqreturn_t tx_handler(int irq, void *dev_id);
irqreturn_t misc_handler(int irq, void *dev_id);

int p_create_ports(struct ether_device_data *data);
int p_register_ports(struct ether_device_data *data);

void p_create_rx_pool(struct ether_device_data *data, int ch);

void cpdma_desc_free(struct cpdma_desc_pool *pool, struct cpdma_desc **desc);

int cpdma_desc_pool_create(struct ether_device_data *data, phys_addr_t desc_mem_phys, u32 bd_ram_size, u32 descs_pool_size);

int cpdma_rx_fill(struct ether_device_data* data);

#endif /* CPDMA_H */