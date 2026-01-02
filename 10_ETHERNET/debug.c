#include "debug.h"

void Print_register_val_cpsw(struct ether_device_data *data,
                u8 ss_regs,
                u8 host_port_regs,
                u8 wr_regs,
                u8 slaves,
                u8 ale_regs,
                u8 cpdma_regs,
                u8 state_ram,
                u8 mdio_regs){
	u16 i = 0;

    if (ss_regs){
        printk("--- [ss regs] ---\n");
        for (i = 0; i < 13; i++){
            printk("# [%xh] = 0x%x\n", i*4, readl_relaxed((u8*)data->base_cpsw + i*4));
        }
        printk("--- [>>>>><<<<<] ---\n");
    }

    if (host_port_regs){
        printk("--- [host_port_regs] ---\n");
        for (i = 0; i < 16; i++){
            printk("# [%xh] = 0x%x\n", i*4, readl_relaxed((u8*)data->base_port0 + i*4));
        }
        for (i = 16; i < 34; i++){
            printk("# [%xh] = 0x%x\n", i*4, readl_relaxed((u8*)data->base_port1 + i*4));
        }
        printk("--- [>>>>><<<<<] ---\n");
    }

    if (wr_regs){
        printk("--- [wr_regs] ---\n");
        for (i = 0; i < 8; i++){
            printk("# [%xh] = 0x%x\n", i*4, readl_relaxed((u8*)data->base_wr + i*4));
        }
        printk("--- [>>>>><<<<<] ---\n");
    }

    if (slaves){
        printk("--- [slaves] ---\n");
        for (i = 0; i < 11; i++){
            printk("# [%xh] = 0x%x\n", i*4, readl_relaxed((u8*)data->base_cpsw_sl + i*4));
        }
        printk("--- [>>>>><<<<<] ---\n");
    }

    if (ale_regs){
        printk("--- [ale_regs] ---\n");
        for (i = 0; i < 19; i++){
            printk("# [%xh] = 0x%x\n", i*4, readl_relaxed((u8*)data->base_ale + i*4));
        }
        printk("--- [>>>>><<<<<] ---\n");
    }

    if (cpdma_regs){
        printk("--- [cpdma_regs] ---\n");
        for (i = 0; i < 64; i++){
            printk("# [%xh] = 0x%x\n", i*4, readl_relaxed((u8*)data->base_cpdma + i*4));
        }
        printk("--- [>>>>><<<<<] ---\n");
    }

    if (state_ram){
        printk("--- [state_ram] ---\n");
        for (i = 0; i < 32; i++){
            printk("# [%xh] = 0x%x\n", i*4, readl_relaxed((u8*)data->base_txhdp + i*4));
        }
        printk("--- [>>>>><<<<<] ---\n");
    }

    if (mdio_regs){
        printk("--- [mdio_regs] ---\n");
        for (i = 0; i < 6; i++){
            printk("# [%xh] = 0x%x\n", i*4, readl_relaxed((u8*)data->base_mdio + i*4));
        }

        for (i = 8; i < 12; i++){
            printk("# [%xh] = 0x%x\n", i*4, readl_relaxed((u8*)data->base_mdio + i*4));
        }

        for (i = 32; i < 36; i++){
            printk("# [%xh] = 0x%x\n", i*4, readl_relaxed((u8*)data->base_mdio + i*4));
        }
        printk("--- [>>>>><<<<<] ---\n");
    }

}

void Print_ale_entry(struct ether_device_data *data, u32 idx_total){
    u32 ale_entry[3];
    u32 i;

    for (i = 0; i < idx_total; i++){
        ale_read(data, ale_entry, i);
        printk("--- [ale idx = %d] ---\n", i);
        printk("# 0x%x 0x%x 0x%x\n", ale_entry[0], ale_entry[1], ale_entry[2]);
        printk("--- [>>>>><<<<<] ---\n");
    }
}

void Print_phy(struct ether_device_data *data){
    u32 i = 0;
    u32 shareddata = 0;

    // mdio_read(struct ether_device_data *data, u32 phy_id, u32 phy_reg, u16* dataX)
    printk("--- [mdio regs] ---\n");
    for (i = 0; i <= 6; i++){
        memset((u8*)&shareddata, 0, 4);
        mdio_read(data, PHY_ID0, i, (u16*)&shareddata);
        printk("# [%d] = 0x%x\n", i, shareddata);
    }

    for (i = 17; i <= 18; i++){
        memset((u8*)&shareddata, 0, 4);
        mdio_read(data, PHY_ID0, i, (u16*)&shareddata);
        printk("# [%d] = 0x%x\n", i, shareddata);
    }

    for (i = 26; i <= 27; i++){
        memset((u8*)&shareddata, 0, 4);
        mdio_read(data, PHY_ID0, i, (u16*)&shareddata);
        printk("# [%d] = 0x%x\n", i, shareddata);
    }

    for (i = 29; i <= 31; i++){
        memset((u8*)&shareddata, 0, 4);
        mdio_read(data, PHY_ID0, i, (u16*)&shareddata);
        printk("# [%d] = 0x%x\n", i, shareddata);
    }
    printk("--- [>>>>><<<<<] ---\n"); 
 
}

/* */