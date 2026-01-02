#ifndef ALE_H
#define ALE_H

#include "mdio.h"
#include "cpsw.h"

/* ALE unicast entry flags - passed into cpsw_ale_add_ucast() */
#define ALE_SECURE			BIT(0)
#define ALE_BLOCKED			BIT(1)
#define ALE_SUPER			BIT(2)
#define ALE_VLAN			BIT(3)

#define ALE_TYPE_FREE			0
#define ALE_TYPE_ADDR			1
#define ALE_TYPE_VLAN			2
#define ALE_TYPE_VLAN_ADDR		3

#define ALE_UCAST_PERSISTANT    0
#define ALE_UCAST_UNTOUCHED		1
#define ALE_UCAST_OUI			2
#define ALE_UCAST_TOUCHED		3

#define ALE_PORT_HOST			BIT(0)
#define ALE_PORT_1			BIT(1)
#define ALE_PORT_2			BIT(2)

#define ALE_MCAST_FWD			0
#define ALE_MCAST_BLOCK_LEARN_FWD	1
#define ALE_MCAST_FWD_LEARN		2
#define ALE_MCAST_FWD_2			3

#define HOST_PORT_NUM		0
#define CPSW_ALE_PORTS_NUM	3
#define CPSW_SLAVE_PORTS_NUM	2
#define SLIVER_SIZE		0x40

/*
ale_entry[0] -> ale[95 - 64]
ale_entry[1] -> ale[63 - 32]
ale_entry[2] -> ale[31 - 0] 
*/

/* ALE table
TBL0[31 - 0]  = TBL[31 - 0]  =>(32 bits) 4 bytes of LOW MAC addr
TBL1[15 - 0]  = TBL[47 - 32] =>(16 bits) 2 bytes of HIGH MAC addr
TBL1[27 - 16] = TBL[59 - 47] =>(12 bits) VLAN id
TBL1[29 - 28] = TBL[61 - 60] =>(2 bits)  Entry type
TBL1[31 - 30] = TBL[63 - 62] =>(2 bits)  Unicast type
TBL2[0]       = TBL[64]      =>(1 bit)   Secure flags
TBL2[1]       = TBL[65]      =>(1 bit)   Blocked/Super flags
TBL2[4 - 2]   = TBL[68 - 66] =>(3 bits)  Port number
*/

#define ENTRY_TYPE_START    60
#define ENTRY_TYPE_BITS     2

#define VLAN_ID_START       48
#define VLAN_ID_BITS        12

#define MAC_ADDR_START      0
#define MAC_ADDR_BYTES      6

#define UCAST_TYPE_START    62
#define UCAST_TYPE_BITS     2

#define SECURE_START        64
#define SECURE_BITS         1

#define BLOCKED_START       65
#define BLOCKED_BITS        1

#define SUPER_START         65
#define SUPER_BITS          1

#define PORT_NUM_START      66
#define PORT_NUM_BITS       3 // (host port 0, slave port 1, slave port 2)

void ale_read(struct ether_device_data *data, u32* ale_entry, u16 idx);
void ale_write(struct ether_device_data *data, u32* ale_entry, u16 idx);

void ale_add_vlan_id(struct ether_device_data *data, u16 vid, u8 port_mask);
void ale_add_ucast(struct ether_device_data *data, u16 vid, u8* addr, u32 flags, u32 port);
void ale_add_mcast(struct ether_device_data *data, u16 vid, u8* addr, u32 flags, u32 port, u32 mcast_state);

#endif /* ALE_H */