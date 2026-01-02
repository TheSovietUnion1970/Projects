#include <linux/module.h>
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
#include "mdio.h"
#include "eth0.h"

/* Print data */
void ETHER1_Print_Hex(u8 *data, u16 len, u8 *name){
    char line[3 * 8 + 1]; // "XX " * 8 bytes + null terminator = 25 chars
    u16 i;

    if (!data || len == 0)
        return;

    printk("# %s (len=%u bytes):\n", name, len);

    for (i = 0; i < len; i++) {
        int pos = (i % 8) * 3;
        snprintf(&line[pos], sizeof(line) - pos, "%02X ", data[i]);

        // Print every 8 bytes, or at the end of data
        if ((i % 8) == 7 || i == len - 1) {
            printk("  %s\n", line);
            memset(line, 0, sizeof(line));
        }
    }
}

int wait_register_update(struct ether_device_data *data, void __iomem *mem, u16 reg_offset, u16 bit_offset, u8 bit_val, u16 delay_ms, u8* name_register){
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

int wait_val_update(struct ether_device_data *data, u16* var, u16 val, u16 delay_ms, u8* name_val){
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

int clock_init(struct ether_device_data *data){

    iowrite32(0x02, data->base_clk + 0x144);
    iowrite32(0x02, data->base_clk + 0x120);
    iowrite32(0x02, data->base_clk + 0x11c);
    iowrite32(0x02, data->base_clk + 0x14);
    iowrite32(0x02, data->base_clk + 0x12c);
    //printk("0x0C = 0x%x\n", ioread32(data->base_clk + 0x0C));
    //printk("0x11C = 0x%x\n", ioread32(data->base_clk + 0x11C));

    // ret = wait_register_update(data, data->base_clk, 0x14, 18, 0, 2000, "clock_init");
    // if (ret < 0) {
    //     printk("clk = 0x%x\n", ioread32(data->base_clk + 0x14));
    //     return ret;
    // }
    data->clk_freq = 125000000;
    //printk("clk = 0x%x\n", ioread32(data->base_clk + 0x14));
    return 0;
}

int clock_deinit(struct ether_device_data *data){

    iowrite32(0, data->base_clk + 0x144);
    iowrite32(0, data->base_clk + 0x120);
    iowrite32(0, data->base_clk + 0x11c);
    iowrite32(0, data->base_clk + 0x14);
    iowrite32(0, data->base_clk + 0x12c);
    //printk("0x0C = 0x%x\n", ioread32(data->base_clk + 0x0C));
    //printk("0x11C = 0x%x\n", ioread32(data->base_clk + 0x11C));

    // ret = wait_register_update(data, data->base_clk, 0x14, 18, 0, 2000, "clock_init");
    // if (ret < 0) {
    //     printk("clk = 0x%x\n", ioread32(data->base_clk + 0x14));
    //     return ret;
    // }
    data->clk_freq = 0;
    //printk("clk = 0x%x\n", ioread32(data->base_clk + 0x14));
    return 0;
}

void gmii_sel_init(struct ether_device_data *data){
    iowrite32(0xE0, data->base_ctrmod + 0x650);
}

void mdio_enable(struct ether_device_data *data){
    u32 mdioc = 0;
    /* PREAMBLE and CLKDIV bits in the MDIO control register */
    /* bus freq = 1000000 */
    mdioc &=~ MDIOC_PREAMBLE; // 
    mdioc |= MDIOC_ENABLE; // enable mdio module
    mdioc |= MDIOC_CLKDIV((data->clk_freq/1000000) - 1);
    iowrite32(mdioc, data->base_mdio + MDIO_MDIOCONTROL);
#if(PRINT_DATA)
    printk("mdioc = 0x%x, div = 0x%x\n", mdioc, MDIOC_CLKDIV((data->clk_freq/1000000) - 1));
#endif
}

/* wait until hardware is ready for another user access */
int wait_for_user_access(struct ether_device_data *data)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(MDIO_TIMEOUT);
	u32 reg;

	while (time_after(timeout, jiffies)) {
		reg = ioread32(data->base_mdio + MDIO_MDIOUSERACCESS0);
		if ((reg & USERACCESS_GO) == 0)
			return 0;

		reg = ioread32(data->base_mdio + MDIO_MDIOCONTROL);
		if ((reg & CONTROL_IDLE) == 0) {
			usleep_range(100, 200);
			continue;
		}

		/*
		 * An emac soft_reset may have clobbered the mdio controller's
		 * state machine.  We need to reset and retry the current
		 * operation
		 */
		dev_warn(data->dev, "resetting idled controller\n");
		mdio_enable(data);
		return -EAGAIN;
	}

	reg = ioread32(data->base_mdio + MDIO_MDIOUSERACCESS0);
	if ((reg & USERACCESS_GO) == 0)
		return 0;

	dev_err(data->dev, "timed out waiting for user access\n");
	return -ETIMEDOUT;
}

int mdio_read(struct ether_device_data *data, u32 phy_id, u32 phy_reg, u16* dataX){
    u32 reg = 0;
    int ret = 0;

    /* Check idle state */
    reg = ioread32(data->base_mdio + MDIO_MDIOCONTROL);
    if ((reg&CONTROL_IDLE)) {
        printk("Idle state\n");
        return -1;
    }

    /* Clear GO bit */
    iowrite32(0x00, data->base_mdio + MDIO_MDIOUSERACCESS0);

	reg = (USERACCESS_GO | USERACCESS_READ | (phy_reg << 21) |
	       (phy_id << 16));
    iowrite32(reg, data->base_mdio + MDIO_MDIOUSERACCESS0);

    /* wait GO bit is cleared */
    ret = wait_register_update(data, data->base_mdio, MDIO_MDIOUSERACCESS0, USERACCESS_GO_BIT, 0, 2000, "USERACCESS_GO_BIT");
    if (ret < 0){
        printk("reg = 0x%x\n", reg);
        return -1;
    }

    /* wait ACK bit is set */
    ret = wait_register_update(data, data->base_mdio, MDIO_MDIOUSERACCESS0, USERACCESS_ACK_BIT, 1, 2000, "USERACCESS_ACK_BIT");
    if (ret < 0){
        return -1;
    }

    *dataX = (u16)(ioread32(data->base_mdio + MDIO_MDIOUSERACCESS0)&USERACCESS_DATA);
    //printk("data = 0x%x\n", ioread32(data->base_mdio + MDIO_MDIOUSERACCESS0)&USERACCESS_DATA);
    return 0;
}

int mido_write(struct ether_device_data *data, u32 phy_id, u32 phy_reg, u16 dataX){
    u32 reg = 0;
    int ret = 0;

    /* Check idle state */
    reg = ioread32(data->base_mdio + MDIO_MDIOCONTROL);
    if ((reg&CONTROL_IDLE)) {
        printk("Idle state\n");
        return -1;
    }

    /* Clear GO bit */
    iowrite32(0x00, data->base_mdio + MDIO_MDIOUSERACCESS0);

	reg = (USERACCESS_GO | USERACCESS_WRITE | (phy_reg << 21) |
	       (phy_id << 16) | ((dataX)&USERACCESS_DATA));
    iowrite32(reg, data->base_mdio + MDIO_MDIOUSERACCESS0);

    /* wait GO bit is cleared */
    ret = wait_register_update(data, data->base_mdio, MDIO_MDIOUSERACCESS0, USERACCESS_GO_BIT, 0, 2000, "USERACCESS_GO_BIT");
    if (ret < 0){
        printk("Failed writting to MDIO, reg = 0x%x\n", reg);
        return -1;
    }

    return 0;
}

int ether_mdio_init(struct ether_device_data* data){
    u32 mdioalive = 0, mdiover = 0;
    u16 shareddata = 0;
    int ret;

#if(PRINT_DATA)
    printk("ether_mdio_init is called\n");
#endif
    /* wait for scan logic to settle */
    msleep(1000);

    // iowrite32(0x3, data->base_clk + 0x14);
    //printk("clk = 0x%x\n", ioread32(data->base_clk + 0x14));

    /* Read version */
    mdiover = ioread32(data->base_mdio + MDIO_MDIOVER);
    if (mdiover){
        //printk("Revision: %d.%d\n", (mdiover >> 8)&0xFF, (mdiover)&0xFF);
    }

    /* get phy mask from the alive register */
    mdioalive = ioread32(data->base_mdio + MDIO_MDIOALIVE);
    if (mdioalive){
        //printk("detected phy mask %x\n", mdioalive);
    }

    mdio_enable(data);

    /* Read register 2 */
    ret = mdio_read(data, PHY_ID0, MII_PHYSID1, &shareddata);
    if (ret < 0){
        return -1;
    }
    else {
        //printk("PHY ID Number: 0x%x\n", shareddata%0xFFFF);
    }

    /* Read register 3 */
    ret = mdio_read(data, PHY_ID0, MII_PHYSID2, &shareddata);
    if (ret < 0){
        return -1;
    }
    else {
        // printk("PHY ID Number: 0x%x\n", shareddata%0xFC00);
        // printk("Model Number: %d\n", shareddata%0x3F0);
        // printk("Revision Number: %d\n", shareddata%0xF);
    }

    /* Disable interrupts */
    ret = mido_write(data, PHY_ID0, MII_LAN83C185_IM, 0x00);
    if (ret < 0){
        return -1;
    }

    /* Read ISF */
    ret = mdio_read(data, PHY_ID0, MII_LAN83C185_ISF, &shareddata);
    if (ret < 0){
        return -1;
    }
    else {
        //printk("MII_LAN83C185_ISF: 0x%x\n", shareddata%0xFE);
    }

    /* genphy_read_abilities */
    ret = mdio_read(data, PHY_ID0, MII_BMSR, &shareddata);
    if (ret < 0){
        return -1;
    }
    else {
        //printk("genphy_read_abilities: 0x%x\n", shareddata);
    }


    /* TODO */
    // ret = mido_write(data, PHY_ID0, MII_BMCR, 0x3000);
    // if (ret < 0){
    //     return -1;
    // }
    
    return 0;
}