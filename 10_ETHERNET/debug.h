#ifndef DEBUG_H
#define DEBUG_H

#include "cpsw.h"
#include "ale.h"
#include "mdio.h"

#define SS_EN 1
#define SS_DIS 0

#define HOST_EN 1
#define HOST_DIS 0

#define WR_EN 1
#define WR_DIS 0

#define SL_EN 1
#define SL_DIS 0

#define ALE_EN 1
#define ALE_DIS 0

#define CPDMA_EN 1
#define CPDMA_DIS 0

#define STSRAM_EN 1
#define STSRAM_DIS 0

#define MDIO_EN 1
#define MDIO_DIS 0

/* 
Common function create + destroy:

dma_desc = cpdma_desc_alloc(data->desc_pool);
cpdma_desc_free(data->desc_pool, dma_desc); 
*/

void Print_register_val_cpsw(struct ether_device_data *data,
                u8 ss_regs,
                u8 host_port_regs,
                u8 wr_regs,
                u8 slaves,
                u8 ale_regs,
                u8 cpdma_regs,
                u8 state_ram,
                u8 mdio_regs);

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
void Print_ale_entry(struct ether_device_data *data, u32 idx_total);

void Print_phy(struct ether_device_data *data);


#endif /* DEBUG_H */