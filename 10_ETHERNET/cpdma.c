#include "cpdma.h"
#include "cpsw.h"
#include "eth0.h"

#include <linux/delay.h>
#include <net/page_pool.h>
#include <net/xdp.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/dma-mapping.h>

u8 macaddr[6] = {0x24, 0x76, 0x25, 0xe7, 0x29, 0xf0};

/*
DMA 64 channels:
0-31 Tx
32-64 Rx

tx idx = 7 => Tx ch = 0 + 7 = 7
rx idx = 0 => Rx ch = 32 + 0 = 32

TxDma[n] <-> TxInt[n], 0 =< n <= 7
RxDma[n + 32] <-> RxInt[n], 0 =< n <= 7

#define __chan_linear(chan_num)	((chan_num) & (CPDMA_MAX_CHANNELS - 1))
*/

int cpdma_ctlr_start(struct ether_device_data* data){
    int ret = 0;
    u8 i = 0;
    u32 cpdma_control = 0;

    iowrite32(1, data->base_cpdma + CPDMA_SOFTRESET);
    ret = wait_register_update(data, data->base_cpdma, CPDMA_SOFTRESET, 0, BIT_VAL_0, 2000, "CPDMA_SOFTRESET");
    if (ret < 0) return -1;

    //8 chan num
    for (i = 0; i < 8; i++){
        iowrite32(0, data->base_txhdp + 4*i);
        iowrite32(0, data->base_rxhdp + 4*i);
        iowrite32(0, data->base_txcp + 4*i);
        iowrite32(0, data->base_rxcp + 4*i);
    }

    iowrite32(0xffffffff, data->base_cpdma + CPDMA_RXINTMASKCLEAR);
    iowrite32(0xffffffff, data->base_cpdma + CPDMA_TXINTMASKCLEAR);

    // enable
    iowrite32(1, data->base_cpdma + CPDMA_TXCONTROL);
    iowrite32(1, data->base_cpdma + CPDMA_RXCONTROL);

    /* ctlr->state = CPDMA_STATE_ACTIVE; */

    /*
    cpdma_chan_set_chan_shaper
    cpdma_chan_on
    -> no need as chan->rate = 0 and CPDMA_STATE_ACTIVE
    */

    cpdma_control = ioread32(data->base_cpdma + CPDMA_CONTROL);
    cpdma_control |= TX_PTYPE; // uses the highest priority 7
    iowrite32(cpdma_control, data->base_cpdma + CPDMA_CONTROL);

    /* Data received at the start */
    iowrite32(0, data->base_cpdma + CPDMA_RX_BUFF_OFFSET);

    return 0;
}

int cpdma_ctlr_stop(struct ether_device_data* data){
    iowrite32(0xffffffff, data->base_cpdma + CPDMA_RXINTMASKCLEAR);
    iowrite32(0xffffffff, data->base_cpdma + CPDMA_TXINTMASKCLEAR);

    // disable
    iowrite32(0, data->base_cpdma + CPDMA_TXCONTROL);
    iowrite32(0, data->base_cpdma + CPDMA_RXCONTROL);

    /* ctlr->state = CPDMA_STATE_IDLE; */

    return 0;
}

void cpdma_intr_enable(struct ether_device_data* data){
    iowrite32(0x05, data->base_wr + WR_CONTROL); /* [TODO] */

    iowrite32(0xffff, data->base_wr + WR_C0_RX_EN);
    iowrite32(0xff, data->base_wr + WR_C0_TX_EN);

    // Int channel 7 TX <-> DMA channel 7 TX
    iowrite32(BIT(chan_linear(data->tx_dma_channel)), data->base_cpdma + CPDMA_TXINTMASKSET);

    // Int channel 0 RX <-> DMA channel 32 RX
    iowrite32(BIT(chan_linear(data->rx_dma_channel)), data->base_cpdma + CPDMA_RXINTMASKSET);
}

void cpdma_intr_disable(struct ether_device_data* data){
    iowrite32(0, data->base_wr + WR_C0_RX_EN);
    iowrite32(0, data->base_wr + WR_C0_TX_EN);

    // Int channel 7 TX <-> DMA channel 7 TX
    iowrite32(BIT(chan_linear(data->tx_dma_channel)), data->base_cpdma + CPDMA_TXINTMASKCLEAR);

    // Int channel 0 RX <-> DMA channel 32 RX
    iowrite32(BIT(chan_linear(data->rx_dma_channel)), data->base_cpdma + CPDMA_RXINTMASKCLEAR);
}

/* IRQ handlers */
irqreturn_t rx_thresh_handler(int irq, void *dev_id){
    struct ether_device_data *data = dev_id;

    printk("rx_thresh_handler\n");

    iowrite32(0, data->base_wr + WR_C0_RX_THRESH_EN);
    return IRQ_HANDLED; 
}

irqreturn_t rx_handler(int irq, void *dev_id){
    struct ether_device_data *data = dev_id;
#if (PRINT_DATA)
    printk("custom_rx_handler\n");
#endif
    iowrite32(0, data->base_wr + WR_C0_RX_EN);
    iowrite32(CPDMA_EOI_RX, data->base_cpdma + CPDMA_MACEOIVECTOR); 

    napi_schedule(&data->napi_rx);

    return IRQ_HANDLED; 
}

irqreturn_t tx_handler(int irq, void *dev_id){
    struct ether_device_data *data = dev_id;

    //printk("tx_handler\n");

    //cpdma_intr_disable(data);

    iowrite32(0, data->base_wr + WR_C0_TX_EN);
    iowrite32(CPDMA_EOI_TX, data->base_cpdma + CPDMA_MACEOIVECTOR); 

    napi_schedule(&data->napi_tx);

    return IRQ_HANDLED; 
}

irqreturn_t misc_handler(int irq, void *dev_id){
    struct ether_device_data *data = dev_id;

     printk("misc_handler\n");

    iowrite32(0, data->base_wr + WR_C0_MISC_EN);
    return IRQ_HANDLED; 
}

// =================== [cpdma_desc]
dma_addr_t desc_phys(struct cpdma_desc_pool *pool, struct cpdma_desc __iomem *desc)
{
	if (!desc){
        return 0;
    }
	return pool->hw_addr + (__force long)desc - (__force long)pool->iomap;
}

struct cpdma_desc __iomem *
desc_from_phys(struct cpdma_desc_pool *pool, dma_addr_t dma)
{
	return dma ? pool->iomap + dma - pool->hw_addr : NULL;
}

int cpdma_desc_pool_create(struct ether_device_data *data, phys_addr_t desc_mem_phys, u32 bd_ram_size, u32 descs_pool_size){
    struct cpdma_desc_pool *desc_pool;
    int ret;

    desc_pool = devm_kzalloc(data->dev, sizeof(*desc_pool), GFP_KERNEL);
    if (!desc_pool) {
        printk("Fail: desc_pool\n");
        data->desc_pool = NULL;
        return -1;
    }
    data->desc_pool = desc_pool;

    desc_pool->hw_addr = desc_mem_phys;

    desc_pool->desc_size = ALIGN(sizeof(struct cpdma_desc), 16); // 16 bytes
    desc_pool->gen_pool = devm_gen_pool_create(data->dev, ilog2(desc_pool->desc_size),
					      -1, "cpdma");

    desc_pool->mem_size = desc_pool->desc_size * descs_pool_size;  
    // desc_pool->iomap = ioremap(desc_mem_phys,
    //                 desc_pool->mem_size);              
    desc_pool->iomap = devm_ioremap(data->dev, desc_mem_phys,
                    desc_pool->mem_size);
    if (!desc_pool->iomap){
        printk("Fail: desc_pool->iomap\n");
        data->desc_pool = NULL;
        return -1;
    }

	ret = gen_pool_add_virt(desc_pool->gen_pool, (unsigned long)desc_pool->iomap,
				desc_mem_phys, desc_pool->mem_size, -1);

    if (ret < 0){
        printk("Fail: gen_pool_add_virt\n");
        return -1;
    }

    //printk("desc_pool->iomap = 0x%x\n", desc_pool->iomap);

    return 0;
}

struct cpdma_desc __iomem *
cpdma_desc_alloc(struct cpdma_desc_pool *pool)
{
	return (struct cpdma_desc __iomem *)
		gen_pool_alloc(pool->gen_pool, pool->desc_size);
}

void cpdma_desc_free(struct cpdma_desc_pool *pool, struct cpdma_desc **desc)
{
	gen_pool_free(pool->gen_pool, (unsigned long)(*desc), pool->desc_size);
    (*desc) = NULL;
}

/* DMA submit */
void cpdma_submit_rx(struct ether_device_data* data, dma_addr_t dma, u8* buf, u32 len, u8 dir, int ch){
    u32 mode;
    struct cpdma_desc __iomem	*dma_desc;
    dma_addr_t desc_dma_phys;

    mode = CPDMA_DESC_OWNER | CPDMA_DESC_SOP | CPDMA_DESC_EOP;
    // must be tx
    if ((dir == 1) ||(dir == 2)) mode |= CPDMA_DESC_TO_PORT_EN | (dir << 16);

    /* sync dma addr */
    dma_sync_single_for_device(data->dev, dma, len, dir);

    /* allocate dma_desc */
    /* desc_pool is alreay allocated in cpsw_init, half for tx (128) and half for rx (128) */
    dma_desc = cpdma_desc_alloc(data->desc_pool);
    dma_desc->hw_len = 0;
    desc_dma_phys = desc_phys(data->desc_pool, dma_desc);
    data->desc_dma_rx = dma_desc;

    // fulfill desc
    iowrite32(0, &dma_desc->hw_next);
    iowrite32(dma, &dma_desc->hw_buffer);
    iowrite32(len, &dma_desc->hw_len);
    iowrite32(mode | len, &dma_desc->hw_mode);

    /* buf here is page */
    iowrite32((u32)buf, &dma_desc->sw_token);
    iowrite32((u32)buf, &dma_desc->sw_buffer);
    iowrite32(len, &dma_desc->sw_len);

    // store desc into rxhdp
    iowrite32((u32)desc_dma_phys, data->base_rxhdp + 4*chan_linear(data->rx_dma_channel)); // at channel rx
}

void cpdma_submit_tx(struct ether_device_data* data, u8* buf, u16 len, u8 dir, int ch){
    dma_addr_t buffer, desc_dma_phys;
    u32 mode;
    int ret;
    unsigned long flags;

    if (!data->dev){
        printk("ERROR\n");
        return;
    }

    spin_lock_irqsave(&data->lock, flags);

    buffer = dma_map_single(data->dev, buf, len, dir);
    ret = dma_mapping_error(data->dev, buffer);
    if (ret) {
        printk("Fail to dma_map_single\n");
        return ;
    }

    mode = CPDMA_DESC_OWNER | CPDMA_DESC_SOP | CPDMA_DESC_EOP;

    // must be tx
    if ((dir == 1) ||(dir == 2)) mode |= CPDMA_DESC_TO_PORT_EN | (dir << 16);

    /* dma desc */
    /* Allocate desc_dma at phys addr */
    if (!data->desc_dma_tx){
        data->desc_dma_tx = cpdma_desc_alloc(data->desc_pool);
        desc_dma_phys = desc_phys(data->desc_pool, data->desc_dma_tx);

        // fulfill desc
        iowrite32(0, &data->desc_dma_tx->hw_next);
        iowrite32(buffer, &data->desc_dma_tx->hw_buffer);
        iowrite32(len, &data->desc_dma_tx->hw_len);
        iowrite32(mode | len, &data->desc_dma_tx->hw_mode);

        iowrite32((u32)buf, &data->desc_dma_tx->sw_token);
        iowrite32((u32)buf, &data->desc_dma_tx->sw_buffer);
        iowrite32(len, &data->desc_dma_tx->sw_len);

#if (PRINT_DATA)
        printk(">>> [TX] Begin transmit the packet\n");
        ETHER1_Print_Hex(buf, len, "txch");
#endif
        iowrite32(desc_dma_phys, data->base_txhdp + 4*ch); // at channel 7

    }
    spin_unlock_irqrestore(&data->lock, flags);
}

/* rx_fill */
int cpdma_rx_fill(struct ether_device_data* data){
    struct page *page;
    dma_addr_t dma;

    page = page_pool_dev_alloc_pages(data->pool[chan_linear(data->rx_dma_channel)]); // for rx only
    if (!page) {
        printk("Error: allocate rx page\n");
        return -1;
    }
    data->page[chan_linear(data->rx_dma_channel)] = page;

    dma = page_pool_get_dma_addr(page);

    // page is assigned to virtual data
    cpdma_submit_rx(data, dma, (u8*)page_address(page), CPSW_MAX_PACKET_SIZE, 0, chan_linear(data->rx_dma_channel));


    return 0;
}

/* Page pool funcs for dma physical addr */
void p_create_rx_pool(struct ether_device_data *data, int ch){
    int pool_size = 128; /* TODO */

    /* cpsw_create_page_pool for rx channel 32 (1) */
    struct page_pool_params pp_params = {};

	pp_params.order = 0;
	pp_params.flags = PP_FLAG_DMA_MAP;
	pp_params.pool_size = pool_size;
	pp_params.nid = NUMA_NO_NODE;
	pp_params.dma_dir = DMA_BIDIRECTIONAL;
	pp_params.dev = data->dev;

    data->pool[ch] = page_pool_create(&pp_params);

    if (IS_ERR(data->pool[ch])){
        printk("cannot create rx page pool\n");
    } 

}

int tx_mq_poll(struct napi_struct *napi_tx, int budget){
    struct ether_device_data *data = container_of(napi_tx, struct ether_device_data, napi_tx);
    u8 ch = 0;
    dma_addr_t desc_dma;
    unsigned long flags;
    u32 token, len, hw_mode;

	spin_lock_irqsave(&data->lock, flags);

    ch = ioread32(data->base_cpdma + CPDMA_TXINTSTATMASKED);

    /*cpdma_desc_free(pool, desc, 1);*/
    if (data->desc_dma_tx){
        /* = __cpdma_chan_process =*/
        /* Get dma addr of desc */
        desc_dma = desc_phys(data->desc_pool, data->desc_dma_tx);
        /* Store desc to complete pointer */
        iowrite32(desc_dma, data->base_txcp + 4*data->tx_dma_channel);
        
        /* __cpdma_chan_free */
        token = ioread32(&data->desc_dma_tx->sw_token);
        len = ioread32(&data->desc_dma_tx->sw_len);
        hw_mode = ioread32(&data->desc_dma_tx->hw_mode);

        dma_unmap_single(data->dev, desc_dma, len, 1);
        if (data->desc_dma_tx) cpdma_desc_free(data->desc_pool, &data->desc_dma_tx);
        //data->desc_dma_tx = NULL;

        //printk("hw_mode end = 0x%x\n", hw_mode);
        /* cpsw_tx_handler */
    }

    /* End of queue and owner bit is clear */
    if ((hw_mode&CPDMA_DESC_EOQ) && (!(hw_mode&CPDMA_DESC_OWNER))){
#if (PRINT_DATA)
        printk("<<< [TX] Done transmitted the last packet\n");
#endif
        /* End of tx_mq_poll */
        napi_complete(napi_tx);
        iowrite32(0xff, data->base_wr + WR_C0_TX_EN);
    }


    spin_unlock_irqrestore(&data->lock, flags);
    return 0;
}

void cpsw_rx_vlan_encap(struct sk_buff *skb)
{
	u32 rx_vlan_encap_hdr;
	u16 vid, pkt_type;

	/* Remove VLAN header encapsulation word */
    rx_vlan_encap_hdr = *((u32 *)skb->data);
	skb_pull(skb, CPSW_RX_VLAN_ENCAP_HDR_SIZE);

    /* to determine untagged or VLAN-tagged pkt */
	pkt_type = (rx_vlan_encap_hdr >>
		    CPSW_RX_VLAN_ENCAP_HDR_PKT_TYPE_SHIFT) &
		    CPSW_RX_VLAN_ENCAP_HDR_PKT_TYPE_MSK;

	/* Ignore unknown & Priority-tagged packets*/
	if (pkt_type == CPSW_RX_VLAN_ENCAP_HDR_PKT_RESERV ||
	    pkt_type == CPSW_RX_VLAN_ENCAP_HDR_PKT_PRIO_TAG)
		return;

	vid = (rx_vlan_encap_hdr >>
	       CPSW_RX_VLAN_ENCAP_HDR_VID_SHIFT) &
	       VLAN_VID_MASK;
	/* Ignore vid 0 and pass packet as is */
	if (!vid){
        printk("VID0 here\n");
        return;
    }
		

	// /* Untag P0 packets if set for vlan */
	// if (!cpsw_ale_get_vlan_p0_untag(cpsw->ale, vid)) {
	// 	prio = (rx_vlan_encap_hdr >>
	// 		CPSW_RX_VLAN_ENCAP_HDR_PRIO_SHIFT) &
	// 		CPSW_RX_VLAN_ENCAP_HDR_PRIO_MSK;

	// 	vtag = (prio << VLAN_PRIO_SHIFT) | vid;
	// 	__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), vtag);
	// }
	// prio = (rx_vlan_encap_hdr >>
	// 	CPSW_RX_VLAN_ENCAP_HDR_PRIO_SHIFT) &
	// 	CPSW_RX_VLAN_ENCAP_HDR_PRIO_MSK;

	// vtag = (prio << VLAN_PRIO_SHIFT) | vid;
	// __vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q), vtag);

	/* strip vlan tag for VLAN-tagged packet */
	if (pkt_type == CPSW_RX_VLAN_ENCAP_HDR_PKT_VLAN_TAG) {
        /* Move MAC addr after 4 bytes of VLAN encap header */
		memmove(skb->data + VLAN_HLEN, skb->data, 2 * ETH_ALEN);
		skb_pull(skb, VLAN_HLEN);
	}
}

static unsigned int cpsw_rxbuf_total_len(unsigned int len)
{
	len += SKB_DATA_ALIGN(sizeof(struct skb_shared_info));

	return SKB_DATA_ALIGN(len);
}

int rx_mq_poll(struct napi_struct *napi_rx, int budget){
    struct ether_device_data *data = container_of(napi_rx, struct ether_device_data, napi_rx);
    struct page *new_page;
    u8* payload;
    dma_addr_t desc_dma, dma_buf;
    u32 len, hw_mode;
    struct sk_buff *skb;
    void *pa;

    //spin_lock_irqsave(&data->lock, flags);

    /* Get data and len */
    if (data->desc_dma_rx->sw_token && data->desc_dma_rx) {
        hw_mode = ioread32(&data->desc_dma_rx->hw_mode);
        len = ioread32(&data->desc_dma_rx->hw_len);
        dma_buf = ioread32(&data->desc_dma_rx->hw_buffer);
        pa = data->desc_dma_rx->sw_token;

        payload = pa;
    }

    /* Get dma addr of desc */
    desc_dma = desc_phys(data->desc_pool, data->desc_dma_rx);
    /* Store desc to complete pointer */
    iowrite32(desc_dma, data->base_rxcp + 4*chan_linear(data->rx_dma_channel));

    dma_sync_single_for_cpu(data->dev, dma_buf, len, 0); // dir = 0

    // ======================

    /* Free the old desc for the next new one */
    if (data->desc_dma_rx) cpdma_desc_free(data->desc_pool, &data->desc_dma_rx);
    
    /* Because we need new_page, so don't use page_pool_recycle_direct and page_pool_destroy here */
    /* Process the next receive */
    new_page = page_pool_dev_alloc_pages(data->pool[chan_linear(data->rx_dma_channel)]); // for rx only
    if (!new_page) {
        printk("Error: allocate rx page\n");
        return -1;
    }
    data->page[chan_linear(data->rx_dma_channel)] = new_page;

    // =======================[build_skb]==================
    // if not using cpsw_rxbuf_total_len -> skb_over_panic
    // just add the len of aligned HDEADROOM and aligned skb_shared_info
	skb = build_skb(pa, cpsw_rxbuf_total_len(CPSW_MAX_PACKET_SIZE));
	if (!skb) {
        printk("DROPPED\n");
        return -1;
	}

	skb_put(skb, len);
	skb->dev = data->ndev;

#if(PRINT_DATA)
    ETHER1_Print_Hex(skb->data, skb->len, "skb->data RXX");
#endif
#if (VLAN_ENCAP_USED)
	cpsw_rx_vlan_encap(skb);
#endif

	skb->protocol = eth_type_trans(skb, data->ndev);

	/* mark skb for recycling */
	skb_mark_for_recycle(skb);
    /* Hands the packet to the core networking stack for processing */
	netif_receive_skb(skb);

    // ======================[cpdma_submit_rx]====================

    dma_buf = page_pool_get_dma_addr(new_page);
    cpdma_submit_rx(data, dma_buf, (u8*)page_address(new_page), CPSW_MAX_PACKET_SIZE, 0, chan_linear(data->rx_dma_channel));

    if ((hw_mode&CPDMA_DESC_EOQ) && (!(hw_mode&CPDMA_DESC_OWNER))){
        /* End of rx_mq_poll */
        napi_complete_done(napi_rx, 1);
        iowrite32(0xff, data->base_wr + WR_C0_RX_EN);
    }

    //spin_unlock_irqrestore(&data->lock, flags);

    return 0;
}

static int cpsw_ndo_open(struct net_device *ndev){

    struct device *dev = ndev->dev.parent;
    struct ether_device_data* data = dev_get_drvdata(dev);
    int ret;

#if(PRINT_DATA)
    printk("cpsw_ndo_open\n");
#endif
    ret = netif_set_real_num_tx_queues(ndev, data->tx_dma_channel);
    if (ret < 0) {
        printk("Fail netif_set_real_num_tx_queues\n");
        return -1;
    }

    if (ret == 0){
        ret = cpsw_open(data);
    }

    return ret;
}

static int cpsw_ndo_stop(struct net_device *ndev){
    return 0;
}


static netdev_tx_t eth_xmit(struct sk_buff *skb, struct net_device *ndev)
{
    struct netdev_queue *txq;
    int q_idx;
    struct device *dev = ndev->dev.parent;
    struct ether_device_data* data = dev_get_drvdata(dev);
    if (!data) {
        printk("error getting data\n");
        return -1;
    }

    // add pading to 72 bytes (minimum for ARP ethernet packet)
	if (skb_put_padto(skb, CPSW_MIN_PACKET_SIZE)) {
        printk("PADDING failed\n");
		return NET_XMIT_DROP;
	}

    /* ndev for queues */
    q_idx = skb_get_queue_mapping(skb);
    if (q_idx >= data->tx_dma_channel){
        q_idx = q_idx % data->tx_dma_channel;
    }
    txq = netdev_get_tx_queue(ndev, q_idx);
    skb_tx_timestamp(skb);

    cpdma_submit_tx(data, skb->data, skb->len, 1, chan_linear(data->tx_dma_channel));

    dev_kfree_skb(skb);
    return NETDEV_TX_OK;
}

static int cpsw_ndo_vlan_rx_add_vid(struct net_device *ndev,
				    __be16 proto, u16 vid){
    printk("cpsw_ndo_vlan_rx_add_vid\n");
    return 0;
}

static int cpsw_ndo_vlan_rx_kill_vid(struct net_device *ndev,
				    __be16 proto, u16 vid){
    printk("cpsw_ndo_vlan_rx_kill_vid\n");
    return 0;
}

static const struct net_device_ops cpsw_netdev_ops = {
	.ndo_open		= cpsw_ndo_open,
	.ndo_stop		= cpsw_ndo_stop,
    .ndo_start_xmit = eth_xmit,

	.ndo_vlan_rx_add_vid	= cpsw_ndo_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid	= cpsw_ndo_vlan_rx_kill_vid,
};

int p_create_ports(struct ether_device_data *data){
    struct ether_device_data *test;
    data->ndev = devm_alloc_etherdev_mqs(data->dev, sizeof(struct ether_device_data),
                        CPSW_MAX_QUEUES,
                        CPSW_MAX_QUEUES);

    test = netdev_priv(data->ndev);

    //printk("0x%x - 0x%x\n", data, test);

    eth_hw_addr_set(data->ndev, macaddr);

    data->ndev->features |= NETIF_F_HW_VLAN_CTAG_FILTER |
                NETIF_F_HW_VLAN_CTAG_RX | NETIF_F_NETNS_LOCAL;

    data->ndev->netdev_ops = &cpsw_netdev_ops;

    SET_NETDEV_DEV(data->ndev, data->dev);

    /* #define CPSW_POLL_WEIGHT	64 */
    netif_napi_add(data->ndev, &data->napi_tx,
                tx_mq_poll,
                64);
    netif_napi_add(data->ndev, &data->napi_rx,
                rx_mq_poll,
                64);

    return 0;
}

int p_register_ports(struct ether_device_data *data){
    int ret;

    if (!data->ndev) {
        return -1;
    }
    ret = register_netdev(data->ndev);
    if (ret) {
        printk("err registering net device\n");
        return -1;
    }

    return 0;
}
