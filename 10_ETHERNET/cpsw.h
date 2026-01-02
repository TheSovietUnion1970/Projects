#ifndef CPSW_H
#define CPSW_H

#include "mdio.h"
#include "ale.h"

#define MASK_BITS(n, s) (((1u << n) - 1) << s)

/* CPSW_BASE */
#define CPSW_CONTROL            0x04
    #define CPSW_VLAN_AWARE     1u << 1
    #define CPSW_RX_VLAN_ENCAP  1u << 2
#define CPSW_SOFT_RESET         0x08
#define CPSW_STAT_PORT_EN       0x0C
#define CPSW_PTYPE              0x10
#define CPSW_FLOW_CONTROL       0x24

/* SLx_BASE */
#define SLx_SOFT_RESET 0x0C

/* ALE_BASE */
#define ALE_IDVER		0x00
#define ALE_STATUS		0x04
#define ALE_CONTROL		0x08
    #define ALE_ENABLE_ALE		1u << 31
    #define ALE_CLEAR_TABLE		1u << 30
    #define AGE_OUT_NOW		    1u << 29
    #define ALE_P0_UNI_FLOOD    1u << 8
    #define ALE_BYPASS_MODE     1u << 4
    #define ALE_VLAN_AWARE      1u << 2
#define ALE_PRESCALE		0x10
#define ALE_AGING_TIMER		0x14
#define ALE_UNKNOWNVLAN		0x18
#define ALE_TABLE_CONTROL	0x20
    #define ALE_TABLE_WRITE     1u << 31
#define ALE_TABLE		0x34
#define ALE_PORTCTL0		0x40
#define ALE_PORTCTL1		0x44
    #define ALE_NO_LEARN            1u << 4
    #define ALE_DROP_UNKNOWN_VLAN   1u << 3
    #define ALE_DROP_UNTAGGED_VLAN  1u << 2

#define ALE_TABLE_SIZE_MULTIPLIER	1024
#define ALE_STATUS_SIZE_MASK		0x1f

/* CPDMA BASE */

/* PORT0 BASE */
#define P0_TX_IN_CTL            0x10
#define P0_PORT_VLAN            0x14
#define P0_TX_PRI_MAP           0x18
#define P0_CPDMA_TX_PRI_MAP     0x1C
#define P0_CPDMA_RX_CH_MAP      0x20

/* PORT1 BASE */
#define P1_MAX_BLKS             0x08
#define P1_PORT_VLAN            0x14
#define P1_TX_PRI_MAP           0x18
#define P1_SA_HI                0x24
#define P1_SA_LO                0x20

/* CSPW_SL */
#define P1_MACCONTROL           0x04
    #define P1_FULLDUPLEX   1u << 0
    #define P1_GMII_EN      1u << 5
    #define P1_IFCTL_A      1u << 15 // speed 100Mbps
#define P1_MACSTATUS            0x08
#define P1_SOFTRESET            0x0C
#define P1_RX_MAXLEN            0x10
#define P1_RX_PRI_MAP           0x24

#define RX_PRIORITY_MAPPING	0x76543210
#define TX_PRIORITY_MAPPING	0x33221100
#define CPDMA_TX_PRIORITY_MAP	0x76543210

#define CPSW_MAX_BLKS_TX		15
#define CPSW_MAX_BLKS_RX		5

#define CPSW_FIFO_DUAL_MAC_MODE		(1 << 16)

/* Macro for calculation */
#define tx_chan_num(chan)	(chan)
#define rx_chan_num(chan)	((chan) + CPDMA_MAX_CHANNELS)

#define mac_hi(mac) (((mac)[0] << 0) | ((mac)[1] << 8) | ((mac)[2] << 16) | ((mac)[3] << 24))
#define mac_lo(mac) (((mac)[4] << 0) | ((mac)[5] << 8))

enum cpsw_ale_port_state {
	ALE_PORT_STATE_DISABLE	= 0x00,
	ALE_PORT_STATE_BLOCK	= 0x01,
	ALE_PORT_STATE_LEARN	= 0x02,
	ALE_PORT_STATE_FORWARD	= 0x03,
};


int cpsw_init(struct ether_device_data *data);
int cpsw_remove(struct ether_device_data *data);

int cpsw_open(struct ether_device_data *data);

#endif /* CPSW_H */