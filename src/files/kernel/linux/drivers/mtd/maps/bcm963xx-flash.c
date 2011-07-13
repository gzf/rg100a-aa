/*
 * Copyright (C) 2006-2008  Florian Fainelli <florian@openwrt.org>
 * 			    Mike Albon <malbon@openwrt.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mtd/map.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>

#include <bcm_tag.h>
#include <asm/io.h>

#define BUSWIDTH 2                     /* Buswidth */
#define EXTENDED_SIZE 0xBFC00000       /* Extended flash address */

#define FLASH_VIRT_START   0xBE000000
#define FLASH_PHYS_START   0x1E000000
//#define FLASH_SIZE       0x01000000       // 16M
#define DEFAULT_ALIGN      0x200000         // align to 2M

#define PFX KBUILD_MODNAME ": "

#define rounddown(x, y) ((x) - ((x) % (y)))

extern int parse_redboot_partitions(struct mtd_info *master, struct mtd_partition **pparts, unsigned long fis_origin);
static struct mtd_partition *parsed_parts;

static struct mtd_info *bcm963xx_mtd_info;

static struct map_info bcm963xx_map = {
       .name		= "bcm963xx",
       .bankwidth	= BUSWIDTH,
};


static int parse_cfe_partitions( struct mtd_info *master, struct mtd_partition **pparts)
{
	const int nrparts = 6; /* CFE,NVRAM, kernel, rootfs, rootfs_data and global LINUX. */
	struct bcm_tag *buf;
	struct mtd_partition *parts;
	int ret;
	size_t retlen;
	//unsigned int rootfsaddr, kerneladdr, spareaddr, imagestart;
	//unsigned int rootfslen, kernellen, sparelen, totallen;
	//int namelen = 0;
	int i;
	char *boardid;
    char *tagversion;
    unsigned int imgstart, offset, len;
	int align, min_offset;


	/* Allocate memory for buffer */
	buf = vmalloc(sizeof(struct bcm_tag));
	if (!buf)
		return -ENOMEM;

	/* Get the tag */
	ret = master->read(master,master->erasesize,sizeof(struct bcm_tag), &retlen, (void *)buf);
	if (retlen != sizeof(struct bcm_tag)){
		vfree(buf);
		return -EIO;
	}

	tagversion = &(buf->tagVersion[0]);
	boardid = &(buf->boardid[0]);
	printk(KERN_INFO PFX "CFE boot tag found with version %s and board type %s\n",tagversion, boardid);

	/* Ask kernel for more memory */
	//parts = kzalloc(sizeof(*parts) * nrparts + 10 * nrparts, GFP_KERNEL);
	parts = kzalloc(sizeof(struct mtd_partition) * nrparts, GFP_KERNEL);
	if (!parts) {
		vfree(buf);
		return -ENOMEM;
	};

	/* Start building partition list */
	sscanf(buf->flashImageStart, "%u", &imgstart);

	parts[0].name = "CFE";
	sscanf(buf->cfeLength, "%u", &len);
	parts[0].offset = 0;
	parts[0].size = rounddown(imgstart - EXTENDED_SIZE, master->erasesize);
	parts[0].mask_flags = MTD_WRITEABLE;

    parts[1].name = "kernel";
	sscanf(buf->kernelAddress, "%u", &offset);
	sscanf(buf->kernelLength, "%u", &len);
    parts[1].offset = offset - EXTENDED_SIZE;
    parts[1].size = len;
	parts[1].mask_flags = MTD_WRITEABLE;

    parts[2].name = "rootfs";
	sscanf(buf->rootLength, "%u", &len);
    parts[2].offset = imgstart == offset ? parts[1].offset + parts[1].size : imgstart - EXTENDED_SIZE;
    parts[2].size = len;
	parts[2].mask_flags = MTD_WRITEABLE;

	parts[3].name = "nvram";
	parts[3].offset = master->size - master->erasesize;
	parts[3].size = master->erasesize;

	/* Global partition "linux" to make easy firmware upgrade */
	parts[4].name = "linux";
	parts[4].offset = parts[0].size;
	parts[4].size = master->size - parts[0].size - parts[3].size;

    // Spare partition rootfs_data
	//sscanf(buf->totalLength, "%u", &len);
	//offset = roundup(len, master->erasesize) + master->erasesize;
	parts[5].name = "rootfs_data";
#ifdef CONFIG_ROOTFS_DATA_ALIGN
    align = CONFIG_ROOTFS_DATA_ALIGN;
#else
    align = DEFAULT_ALIGN;
#endif
    min_offset = parts[1].offset > parts[2].offset ? 
        parts[1].offset + parts[1].size :
        parts[2].offset + parts[2].size;
    parts[5].offset = roundup(roundup(min_offset, align), master->erasesize);
	parts[5].size = master->size - parts[5].offset - parts[3].size;

	for (i = 0; i < nrparts; i++)
		printk(KERN_INFO PFX "Partition %d is %s offset %lx and length %lx\n", i, parts[i].name, (long unsigned int)(parts[i].offset), (long unsigned int)(parts[i].size));

 	//printk(KERN_INFO PFX "Spare partition is %x offset and length %x\n", offset, len);
	*pparts = parts;
	vfree(buf);

	return nrparts;
};

/*
static int bcm963xx_detect_cfe(struct mtd_info *master)
{
	int idoffset = 0x4e0;
	static char idstring[8] = "CFE1CFE1";
	char buf[9];
	int ret;
	size_t retlen;

	ret = master->read(master, idoffset, 8, &retlen, (void *)buf);
	buf[retlen] = 0;
	printk(KERN_INFO PFX "Read Signature value of %s\n", buf);

	return strncmp(idstring, buf, 8);
}
*/
static int bcm963xx_probe(struct platform_device *pdev)
{
	int err = 0;
	int parsed_nr_parts = 0;
//	char *part_type;
	struct resource *r;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	bcm963xx_map.phys = r->start;
	bcm963xx_map.size = (r->end - r->start) + 1;
	bcm963xx_map.virt = ioremap(r->start, r->end - r->start + 1);

	if (!bcm963xx_map.virt) {
		printk(KERN_ERR PFX "Failed to ioremap\n");
		return -EIO;
	}
	printk(KERN_INFO PFX "0x%08lx at 0x%08x\n", bcm963xx_map.size, bcm963xx_map.phys);

	simple_map_init(&bcm963xx_map);

	bcm963xx_mtd_info = do_map_probe("cfi_probe", &bcm963xx_map);
	if (!bcm963xx_mtd_info) {
		printk(KERN_ERR PFX "Failed to probe using CFI\n");
		err = -EIO;
		goto err_probe;
	}

	bcm963xx_mtd_info->owner = THIS_MODULE;

	/* This is mutually exclusive */
/*
//	if (bcm963xx_detect_cfe(bcm963xx_mtd_info) == 0) {
	if (1) {
		printk(KERN_INFO PFX "CFE bootloader detected\n");
		if (parsed_nr_parts == 0) {
			int ret = parse_cfe_partitions(bcm963xx_mtd_info, &parsed_parts);
			if (ret > 0) {
				part_type = "CFE";
				parsed_nr_parts = ret;
			}
		}
	} else {
		printk(KERN_INFO PFX "assuming RedBoot bootloader\n");
		if (bcm963xx_mtd_info->size > 0x00400000) {
			printk(KERN_INFO PFX "Support for extended flash memory size : 0x%x ; ONLY 64MBIT SUPPORT\n", bcm963xx_mtd_info->size);
			bcm963xx_map.virt = (u32)(EXTENDED_SIZE);
		}

#ifdef CONFIG_MTD_REDBOOT_PARTS
		if (parsed_nr_parts == 0) {
			int ret = parse_redboot_partitions(bcm963xx_mtd_info, &parsed_parts, 0);
			if (ret > 0) {
				part_type = "RedBoot";
				parsed_nr_parts = ret;
			}
		}
#endif
	}
*/

	parsed_nr_parts = parse_cfe_partitions(bcm963xx_mtd_info, &parsed_parts);
	return add_mtd_partitions(bcm963xx_mtd_info, parsed_parts, parsed_nr_parts);

err_probe:
	iounmap(bcm963xx_map.virt);
	return err;
}

static int bcm963xx_remove(struct platform_device *pdev)
{
	if (bcm963xx_mtd_info) {
		del_mtd_partitions(bcm963xx_mtd_info);
		map_destroy(bcm963xx_mtd_info);
	}

	if (bcm963xx_map.virt) {
		iounmap(bcm963xx_map.virt);
		bcm963xx_map.virt = 0;
	}

	return 0;
}

static struct platform_driver bcm63xx_mtd_dev = {
	.probe	= bcm963xx_probe,
	.remove = bcm963xx_remove,
	.driver = {
		.name	= "bcm963xx-flash",
		.owner	= THIS_MODULE,
	},
};

static struct resource mtd_resources[] = {
    {
        .start      = FLASH_PHYS_START,
        .end        = 0x1FFFFFFF,
//      .end        = FLASH_PHYS_START + FLASH_SIZE - 1,
        .flags      = IORESOURCE_MEM
    }
};

static struct platform_device mtd_dev = {
    .name           = "bcm963xx-flash",
    .resource       = mtd_resources,
    .num_resources  = ARRAY_SIZE(mtd_resources),
};


static int __init bcm963xx_mtd_init(void)
{
    int err;

	err = platform_driver_register(&bcm63xx_mtd_dev);
	if (err == 0)
		platform_device_register(&mtd_dev);

    return err;
}

static void __exit bcm963xx_mtd_exit(void)
{
	platform_device_unregister(&mtd_dev);
	platform_driver_unregister(&bcm63xx_mtd_dev);
}

module_init(bcm963xx_mtd_init);
module_exit(bcm963xx_mtd_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Broadcom BCM63xx MTD partition parser/mapping for CFE and RedBoot");
MODULE_AUTHOR("Florian Fainelli <florian@openwrt.org>");
MODULE_AUTHOR("Mike Albon <malbon@openwrt.org>");
