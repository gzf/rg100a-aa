/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
 * Copyright (C) 2008 Florian Fainelli <florian@openwrt.org>
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
//#include <linux/ssb/ssb.h>
//#include <linux/gpio_buttons.h>
#include <linux/input.h>
#include <asm/addrspace.h>
#include <bcm63xx_board.h>
#include <bcm63xx_cpu.h>
#include <bcm63xx_regs.h>
#include <bcm63xx_io.h>
//#include <bcm63xx_dev_pci.h>
#include <bcm63xx_dev_enet.h>
//#include <bcm63xx_dev_dsp.h>
//#include <bcm63xx_dev_pcmcia.h>
//#include <bcm63xx_dev_usb_ohci.h>
//#include <bcm63xx_dev_usb_ehci.h>
//#include <bcm63xx_dev_usb_udc.h>
#include <board_bcm963xx.h>

#define PFX	"board_bcm963xx: "

static struct bcm963xx_nvram nvram;
static unsigned int mac_addr_used;
static struct board_info board;

/*
 * known 6358 boards
 */
#ifdef CONFIG_BCM63XX_CPU_6358
static struct board_info __initdata board_96358vw = {
	.name				= "96358VW",
	.expected_cpu_id		= 0x6358,

	.has_enet0			= 1,
	.has_enet1			= 1,

	.enet0 = {
		.has_phy		= 1,
		.use_internal_phy	= 1,
	},

	.enet1 = {
		.force_speed_100	= 1,
		.force_duplex_full	= 1,
	},
};

static struct board_info __initdata board_96358vw2 = {
	.name				= "96358VW2",
	.expected_cpu_id		= 0x6358,

	.has_enet0			= 1,
	.has_enet1			= 1,

	.enet0 = {
		.has_phy		= 1,
		.use_internal_phy	= 1,
	},

	.enet1 = {
		.force_speed_100	= 1,
		.force_duplex_full	= 1,
	},
};

#endif

/*
 * all boards
 */
static const struct board_info __initdata *bcm963xx_boards[] = {
#ifdef CONFIG_BCM63XX_CPU_6358
	&board_96358vw,
	&board_96358vw2,
#endif
};

/*
 * early init callback, read nvram data from flash and checksum it
 */
void __init board_prom_init(void)
{
	unsigned int check_len, i;
	u8 *boot_addr, *cfe, *p;
	char cfe_version[32];
	u32 val;

	/* read base address of boot chip select (0)
	 * 6345 does not have MPI but boots from standard
	 * MIPS Flash address */
	if (BCMCPU_IS_6345())
		val = 0x1fc00000;
	else {
		val = bcm_mpi_readl(MPI_CSBASE_REG(0));
		val &= MPI_CSBASE_BASE_MASK;
	}
	boot_addr = (u8 *)KSEG1ADDR(val);

	/* dump cfe version */
	cfe = boot_addr + BCM963XX_CFE_VERSION_OFFSET;
	if (!memcmp(cfe, "cfe-v", 5))
		snprintf(cfe_version, sizeof(cfe_version), "%u.%u.%u-%u.%u",
			 cfe[5], cfe[6], cfe[7], cfe[8], cfe[9]);
	else
		strcpy(cfe_version, "unknown");
	printk(KERN_INFO PFX "CFE version: %s\n", cfe_version);

	/* extract nvram data */
	memcpy(&nvram, boot_addr + BCM963XX_NVRAM_OFFSET, sizeof(nvram));

	/* check checksum before using data */
	if (nvram.version <= 4)
		check_len = offsetof(struct bcm963xx_nvram, checksum_old);
	else
		check_len = sizeof(nvram);
	val = 0;
	p = (u8 *)&nvram;
	while (check_len--)
		val += *p;
	if (val) {
		printk(KERN_ERR PFX "invalid nvram checksum\n");
		return;
	}

	/* find board by name */
	for (i = 0; i < ARRAY_SIZE(bcm963xx_boards); i++) {
		if (strncmp(nvram.name, bcm963xx_boards[i]->name,
			    sizeof(nvram.name)))
			continue;
		/* copy, board desc array is marked initdata */
		memcpy(&board, bcm963xx_boards[i], sizeof(board));
		break;
	}

	/* bail out if board is not found, will complain later */
	if (!board.name[0]) {
		char name[17];
		memcpy(name, nvram.name, 16);
		name[16] = 0;
		printk(KERN_ERR PFX "unknown bcm963xx board: %s\n",
		       name);
		return;
	}

	/* setup pin multiplexing depending on board enabled device,
	 * this has to be done this early since PCI init is done
	 * inside arch_initcall */
    /*
	val = 0;

#ifdef CONFIG_PCI
	if (board.has_pci) {
		bcm63xx_pci_enabled = 1;
		if (BCMCPU_IS_6348())
			val |= GPIO_MODE_6348_G2_PCI;
	}
#endif

	if (board.has_pccard) {
		if (BCMCPU_IS_6348())
			val |= GPIO_MODE_6348_G1_MII_PCCARD;
	}

	if (board.has_enet0 && !board.enet0.use_internal_phy) {
		if (BCMCPU_IS_6348())
			val |= GPIO_MODE_6348_G3_EXT_MII |
				GPIO_MODE_6348_G0_EXT_MII;
	}

	if (board.has_enet1 && !board.enet1.use_internal_phy) {
		if (BCMCPU_IS_6348())
			val |= GPIO_MODE_6348_G3_EXT_MII |
				GPIO_MODE_6348_G0_EXT_MII;
	}

	bcm_gpio_writel(val, GPIO_MODE_REG);
    */
}

/*
 * second stage init callback, good time to panic if we couldn't
 * identify on which board we're running since early printk is working
 */
void __init board_setup(void)
{
	if (!board.name[0])
		panic("unable to detect bcm963xx board");
	printk(KERN_INFO PFX "board name: %s\n", board.name);

	/* make sure we're running on expected cpu */
	if (bcm63xx_get_cpu_id() != board.expected_cpu_id)
		panic("unexpected CPU for bcm963xx board");
}

/*
 * return board name for /proc/cpuinfo
 */
const char *board_get_name(void)
{
	return board.name;
}

/*
 * register & return a new board mac address
 */
static int board_get_mac_address(u8 *mac)
{
	u8 *p;
	int count;

	if (mac_addr_used >= nvram.mac_addr_count) {
		printk(KERN_ERR PFX "not enough mac address\n");
		return -ENODEV;
	}

	memcpy(mac, nvram.mac_addr_base, ETH_ALEN);
	p = mac + ETH_ALEN - 1;
	count = mac_addr_used;

	while (count--) {
		do {
			(*p)++;
			if (*p != 0)
				break;
			p--;
		} while (p != mac);
	}

	if (p == mac) {
		printk(KERN_ERR PFX "unable to fetch mac address\n");
		return -ENODEV;
	}

	mac_addr_used++;
	return 0;
}

/*
 * third stage init callback, register all board devices.
 */
int __init board_register_devices(void)
{
	if (board.has_enet0 &&
	    !board_get_mac_address(board.enet0.mac_addr))
		bcm63xx_enet_register(0, &board.enet0);

	if (board.has_enet1 &&
	    !board_get_mac_address(board.enet1.mac_addr))
		bcm63xx_enet_register(1, &board.enet1);

	return 0;
}

