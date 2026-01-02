#include "cpsw.h"
#include "mdio.h"
#include "cpdma.h"
#include "debug.h"
#include <linux/delay.h>
#include <linux/workqueue.h>
#include "eth0.h"

u8 mac_addr[6] = {0x24, 0x76, 0x25, 0xe7, 0x29, 0xf0};
u8 broadcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

int cpsw_ale_version(struct ether_device_data *data){
    u32 ale_id = 0;

    ale_id = ioread32(data->base_ale + ALE_IDVER);
    printk("ALE version: %d.%d\n", (ale_id >> 8)&0xFF, (ale_id)&0xFF);

    return 0;
}

int cpsw_soft_reset(struct ether_device_data *data){
    int ret;

    iowrite32(BIT_VAL_1, data->base_cpsw + CPSW_SOFT_RESET);
    ret = wait_register_update(data, data->base_cpsw, CPSW_SOFT_RESET, 0, BIT_VAL_0, 2000, "CPSW_SOFT_RESET");
    if (ret < 0) return -1;
    return 0;
}

/* cpsw_init_host_port funcs */
void cpsw_ale_start(struct ether_device_data *data)
{
    u32 ale_control = 0;

    /* soft reset the controller and initialize ale */
    ale_control = (ALE_ENABLE_ALE | ALE_CLEAR_TABLE);
    iowrite32(ale_control, data->base_ale + ALE_CONTROL);
}

void cpsw_init_host_port_dual_mac(struct ether_device_data *data){
    u32 ale_control = 0;

    /* host_port_dual_mac */
    iowrite32(CPSW_FIFO_DUAL_MAC_MODE, data->base_port0 + P0_TX_IN_CTL);

    /* unset P0_UNI_FLOOD */
    ale_control = ioread32(data->base_ale + ALE_CONTROL);
    ale_control &=~ (ALE_P0_UNI_FLOOD);
    iowrite32(ale_control, data->base_ale + ALE_CONTROL);

    /* learning make no sense in dual_mac mode in port 1 */
    ale_control = ioread32(data->base_ale + ALE_PORTCTL0);
    ale_control |= (ALE_NO_LEARN);
    iowrite32(ale_control, data->base_ale + ALE_PORTCTL0);

    ale_control = ioread32(data->base_ale + ALE_PORTCTL0);
    ale_control |= (ALE_PORT_STATE_FORWARD);
    iowrite32(ale_control, data->base_ale + ALE_PORTCTL0);  
}

void cpsw_init_host_port(struct ether_device_data *data){
    u32 ale_control = 0;
    u32 cpsw_control = 0;
    int ret;

    /* Soft reset */
    ret = cpsw_soft_reset(data);

    if (ret == 0){
        /* soft reset the controller and initialize ale */
        cpsw_ale_start(data);

#if (VLAN_ENCAP_USED)
        /* switch to vlan unaware mode */
        // Drop packet if VLAN not found
        ale_control = ioread32(data->base_ale + ALE_CONTROL);
        ale_control |= ALE_VLAN_AWARE;
        iowrite32(ale_control, data->base_ale + ALE_CONTROL);

        // Port 0 receive packets (from 3G) are VLAN encapsulated
        cpsw_control = ioread32(data->base_cpsw + CPSW_CONTROL);
        cpsw_control |= CPSW_VLAN_AWARE | CPSW_RX_VLAN_ENCAP;
        iowrite32(cpsw_control, data->base_cpsw + CPSW_CONTROL);
#endif

        // [V2] 
        ale_control = ioread32(data->base_ale + ALE_CONTROL);
        //ale_control |= ALE_BYPASS_MODE;
        iowrite32(ale_control, data->base_ale + ALE_CONTROL);


        /* setup host port priority mapping */
        iowrite32(CPDMA_TX_PRIORITY_MAP, data->base_port0 + P0_CPDMA_TX_PRI_MAP);
        iowrite32(0, data->base_port0 + P0_CPDMA_RX_CH_MAP);

        /* disable priority elevation */
        iowrite32(0, data->base_cpsw + CPSW_PTYPE);

        /* enable statistics collection only on all ports */
        iowrite32(0x7, data->base_cpsw + CPSW_STAT_PORT_EN);

        /* Enable internal fifo flow control */
        iowrite32(0x7, data->base_cpsw + CPSW_FLOW_CONTROL);  

        cpsw_init_host_port_dual_mac(data);
    }
}

/* cpsw_slave_open funcs */
void cpsw_port_add_dual_emac_def_ale_entries(struct ether_device_data *data){
    u32 ale_control = 0;

    iowrite32(VID_USED, data->base_port1 + P1_PORT_VLAN);

    /* VLAN member list for both port host and port 1 */
    ale_add_vlan_id(data, VID_USED, ALE_PORT_HOST | ALE_PORT_1);


    /* [LEARN] ALE_PORT_HOST is set here to indicate that all packets
    received are forwared to host port for processing */
    /* [LEARN] SECURE is set - drop packet if ... */
    /* [LEARN] BLOCK is set - drop packet if a matching dst/src addr -> used for blocking
    devices with desired MAC addr */
    /* [LEARN] HOST_PORT_NUM is set - packet is forwared ONLY to host port */
    ale_add_mcast(data, VID_USED, broadcast, ALE_VLAN, ALE_PORT_HOST, ALE_MCAST_FWD);
    ale_add_ucast(data, VID_USED, mac_addr, ALE_VLAN | ALE_SECURE, HOST_PORT_NUM);


    ale_control = ioread32(data->base_ale + ALE_PORTCTL1);
    ale_control |= (ALE_DROP_UNKNOWN_VLAN);
    iowrite32(ale_control, data->base_ale + ALE_PORTCTL1);  

    /* [LEARNING] dual-mode = port0 <-> port1 only, no switching between port1 and port2
    ALE learning -> essential for layer 2 forwarding in switch mode,
    In switch mode, the program will add/update ALE entries 
    + Hit: Forward to the specific port(s).
    + Miss: Flood to all ports in the VLAN */
    /* learning make no sense in dual_mac mode */
    ale_control = ioread32(data->base_ale + ALE_PORTCTL1);
    ale_control |= (ALE_NO_LEARN);
    iowrite32(ale_control, data->base_ale + ALE_PORTCTL1);  
}

int cpsw_slave_open(struct ether_device_data *data){
    int ret;

    // soft reset
    iowrite32(BIT_VAL_1, data->base_cpsw_sl + P1_SOFTRESET);
    ret = wait_register_update(data, data->base_cpsw_sl, P1_SOFTRESET, 0, BIT_VAL_0, 2000, "CPSW_SL_P1_SOFT_RESET");
    if (ret < 0) return -1;
    if (ret == 0){
        // reset MACCONTROL
        iowrite32(BIT_VAL_0, data->base_cpsw_sl + P1_MACCONTROL);

        /* setup priority mapping */
        iowrite32(RX_PRIORITY_MAPPING, data->base_cpsw_sl + P1_RX_PRI_MAP);
    }
    if (ret == 0){
        // CPSW_VERSION_2:
        iowrite32(TX_PRIORITY_MAPPING, data->base_port1 + P1_TX_PRI_MAP);
		/* Increase RX FIFO size to 5 for supporting fullduplex
		 * flow control mode
		 */
        iowrite32((CPSW_MAX_BLKS_TX << 4) | CPSW_MAX_BLKS_RX, data->base_port1 + P1_MAX_BLKS);
    }
    if (ret == 0){
        /* setup max packet size, and mac address */
        iowrite32(0x5f6, data->base_cpsw_sl + P1_RX_MAXLEN);
        iowrite32(mac_hi(mac_addr), data->base_port1 + P1_SA_HI);
        iowrite32(mac_lo(mac_addr), data->base_port1 + P1_SA_LO);

        cpsw_port_add_dual_emac_def_ale_entries(data);
    }

    return ret;
}

/* phy_init_hw funcs*/
void smsc_phy_config_intr(struct ether_device_data *data, bool enabled){
    if (enabled){
        /* Auto-Negotiation complete */
        mido_write(data, PHY_ID0, MII_LAN83C185_IM, MII_LAN83C185_ISF_INT6);
    }
    else {
        mido_write(data, PHY_ID0, MII_LAN83C185_IM, 0);
    }
}

// it just enables power down -> maybe no need this time
void smsc_phy_config_init(struct ether_device_data *data){
    u16 ctlsts = 0;

    mdio_read(data, PHY_ID0, MII_LAN83C185_CTRL_STATUS, &ctlsts);
}

int phy_init_hw(struct ether_device_data *data){
    int ret = 0;
    u16 mmi_bmcr = 0;

    // set "all capable" mode
    ret = mido_write(data, PHY_ID0, MII_LAN83C185_SPECIAL_MODES, MII_LAN83C185_MODE_ALL);
    if (ret < 0) return -1;

    /* reset the phy */
    ret = mdio_read(data, PHY_ID0, MII_BMCR, &mmi_bmcr);
    mmi_bmcr &=~ BMCR_ISOLATE;
    mmi_bmcr |= BMCR_RESET | BMCR_ANRESTART;
    ret = mido_write(data, PHY_ID0, MII_BMCR, mmi_bmcr);
    if (ret < 0) return -1;

    msleep(10); // wait reset bit is clear
    ret = mdio_read(data, PHY_ID0, MII_BMCR, &mmi_bmcr);
    if (ret < 0) return -1;
    if (mmi_bmcr&BMCR_RESET) {
        printk("Fail to reset phy\n");
        return -1;
    }

    /* BMCR may be reset to defaults */
    // ret = mido_write(data, PHY_ID0, MII_BMCR, BMCR_SPEED100 | BMCR_ANRESTART | BMCR_ANENABLE);
    // if (ret < 0) return -1;
    // ret = mido_write(data, PHY_ID0, MII_BMCR, BMCR_ANRESTART | BMCR_ANENABLE);
    // if (ret < 0) return -1;

    // disbale first
    smsc_phy_config_intr(data, 0);

    // genphy_resume -> MII_BMCR: BMCR_PDOWN

    return 0;
}

/* phy_gmii_sel_mode */
void phy_gmii_sel_mode(struct ether_device_data *data){
    iowrite32(0xE0, data->base_ctrmod + 0x650);
}

/* genphy_config_advert */
int genphy_config_advert(struct ether_device_data *data){
    int ret = 0;
    u16 mii_advert = 0;
    ret = mdio_read(data, PHY_ID0, MII_ADVERTISE, &mii_advert);
    if (ret < 0) return -1;

    mii_advert |= ADVERTISE_ALL | 0x1;
    ret = mido_write(data, PHY_ID0, MII_ADVERTISE, mii_advert);
    if (ret < 0) return -1;

    /* bmsr = phy_read(phydev, MII_BMSR); */

    return ret;
}

void phy_adjust_link(struct ether_device_data *data){
    u32 mac_control = 0, ale_control = 0;

    /* cpsw_sl_ctl_set(slave->mac_sl, mac_control); */
    mac_control = ioread32(data->base_cpsw_sl + P1_MACCONTROL);
    mac_control |= P1_FULLDUPLEX | P1_GMII_EN | P1_IFCTL_A;
    iowrite32(mac_control, data->base_cpsw_sl + P1_MACCONTROL);

    /* enable forwarding for slave port 1 */
    ale_control = ioread32(data->base_ale + ALE_PORTCTL1);
    ale_control |= ALE_PORT_STATE_FORWARD;
    iowrite32(ale_control, data->base_ale + ALE_PORTCTL1);

    /* phy_print_status(phy); */

    /* [V] port-1, ctr-12, val-3 <- cpsw_set_promiscious */

}

/* phy_status_work */
u16 old_state = 0xff;
static void phy_status_work(struct work_struct *work)
{
    u16 mmi_bmsr = 0;
    int ret;
    struct ether_device_data *data = container_of(work, struct ether_device_data, phy_work);
    if (!data) {
        printk("NULL data\n");
        return;
    }

    ret = mdio_read(data, PHY_ID0, MII_BMSR, &mmi_bmsr);
    if (ret < 0) return;

    if (mmi_bmsr&BMSR_LINK_UP){
        // print only when state is changed:
        if ((mmi_bmsr&BMSR_LINK_UP) != old_state){
            printk("Link is up\n");

            phy_adjust_link(data);

            netif_carrier_on(data->ndev);                // ← link up
            netif_tx_wake_all_queues(data->ndev);         // ← ALLOW ndo_start_xmit!
        }
    }
    else {
        // print only when state is changed:
        if ((mmi_bmsr&BMSR_LINK_UP) != old_state){
            printk("Link is down\n");

            netif_carrier_off(data->ndev);                // ← link down
            netif_tx_stop_all_queues(data->ndev);         // ← BLOCK ndo_start_xmit!
        }        
    }
    old_state = mmi_bmsr&BMSR_LINK_UP;

    // re-schedule
    mod_delayed_work(system_power_efficient_wq, &data->phy_work,
                1 * HZ);
}

static void cpsw_ale_timer(struct timer_list *t)
{
	struct ether_device_data *data = from_timer(data, t, timer);
    u32 ale_control = 0;

    ale_control = ioread32(data->base_ale + ALE_CONTROL);
    ale_control |= AGE_OUT_NOW;
    iowrite32(ale_control, data->base_ale + ALE_CONTROL);

    // update expires
    data->timer.expires = jiffies + 10*HZ;
    add_timer(&data->timer);
	
    //printk("AGE_OUT_NOW\n");
}

int cpsw_open(struct ether_device_data *data){
    int ret = 0;
    /* Initialize host and slave ports */
    if (ret == 0){
#if (PRINT_DATA)
        printk("Host port\n");
#endif
        // ale, ss, 
        cpsw_init_host_port(data);
#if (PRINT_DATA)
        printk("cpsw_slave_open\n");
#endif
        cpsw_slave_open(data);

        if (ret == 0){
            /* === phy = of_phy_connect(priv->ndev, slave->data->phy_node, ===*/
#if (PRINT_DATA)
            printk("of_phy_connect\n");
#endif
            phy_init_hw(data);

            /* phy_attached_info(slave->phy); */

            /* phy_start(slave->phy); = set PHY_UP + start PHY machine*/

            /* Configure GMII_SEL register */
            phy_gmii_sel_mode(data);
        }

        if (ret == 0){
            /* err = phy_start_aneg(phydev); -> */
#if(PRINT_DATA)
            printk("genphy_config_advert\n");
#endif
            ret = genphy_config_advert(data);
        }

        if (ret == 0){
            // create a workqueue to check link every 1s
            INIT_DELAYED_WORK(&data->phy_work, phy_status_work);
            mod_delayed_work(system_power_efficient_wq, &data->phy_work,
                1 * HZ);

            // add a timer to remove old MAC entries every 10s
            timer_setup(&data->timer, cpsw_ale_timer, 0);
            data->timer.expires = jiffies + 10*HZ;
            add_timer(&data->timer);
        }
    }

    if (ret == 0){
        // create dma setup for rx buffer
        p_create_rx_pool(data, chan_linear(data->rx_dma_channel));

        napi_enable(&data->napi_tx);
        napi_enable(&data->napi_rx);
    }

    /* Intr */
    if (ret == 0){
        /* cpdma_ctlr_start */
        cpdma_ctlr_start(data);
        cpdma_rx_fill(data);
        cpdma_intr_enable(data);
    }

#if(PRINT_DEBUG)
    Print_register_val_cpsw(data,
        SS_EN, HOST_EN, WR_EN, SL_EN, ALE_EN, CPDMA_EN, STSRAM_EN, MDIO_EN);

    Print_ale_entry(data, 5);
    Print_phy(data);
#endif
    return ret;
}

int cpsw_init(struct ether_device_data *data){
    int ret = 0;

#if(PRINT_DATA)
    ret = cpsw_ale_version(data);
#endif

    /* setup netdevs */
    if (ret == 0){
        ret = cpdma_desc_pool_create(data, CPPIRAM_BASE,
                CPSW_BD_RAM_SIZE, CPSW_CPDMA_DESCS_POOL_SIZE_DEFAULT);

        if (ret == 0){
            p_create_ports(data);
            ret = p_register_ports(data);

            netif_carrier_off(data->ndev);                // ← link down
            netif_tx_stop_all_queues(data->ndev);         // ← BLOCK ndo_start_xmit!
        }
    }

    return ret;
}

int cpsw_remove(struct ether_device_data *data){
    cancel_delayed_work_sync(&data->phy_work);
    del_timer_sync(&data->timer);

    cpdma_ctlr_stop(data);
    cpdma_intr_disable(data);

    /* Free tx/rx desc */
    // check if data->desc_dma_tx/rx is free in tx/rx_mq_poll
    if (data->desc_dma_rx) cpdma_desc_free(data->desc_pool, &data->desc_dma_rx);
    if (data->desc_dma_tx) cpdma_desc_free(data->desc_pool, &data->desc_dma_tx);

    napi_disable(&data->napi_tx);
    napi_disable(&data->napi_rx);

    /* Free page_pool and page  */
    page_pool_recycle_direct(data->pool[chan_linear(data->rx_dma_channel)], data->page[chan_linear(data->rx_dma_channel)]); 
    page_pool_destroy(data->pool[chan_linear(data->rx_dma_channel)]);

    if (data->ndev){
        printk("unregister_netdev is called");
        unregister_netdev(data->ndev);
        data->ndev = NULL;
    }
    return 0;
}