#include <linux/module.h>

#include <bcm_map_part.h>
#include <board.h>
#include "flash_api.h"
#include "boardparms.h"

void cfi_flash_sched_init(void);

static FLASH_ADDR_INFO fInfo;
NVRAM_DATA bootNvramData;

void kerSysEarlyFlashInit( void )
{
    flash_init();

    fInfo.flash_nvram_length = NVRAM_LENGTH;
    fInfo.flash_nvram_start_blk = 0;  // always the first block
    fInfo.flash_nvram_number_blk = 1; // always fits in the first block
    fInfo.flash_nvram_blk_offset = NVRAM_DATA_OFFSET;

    fInfo.flash_rootfs_start_offset = flash_get_sector_size(0);
    if( fInfo.flash_rootfs_start_offset < FLASH_LENGTH_BOOT_ROM )
        fInfo.flash_rootfs_start_offset = FLASH_LENGTH_BOOT_ROM;
 
    // Read the flash contents into NVRAM buffer
    flash_read_buf (fInfo.flash_nvram_start_blk, fInfo.flash_nvram_blk_offset,
        (unsigned char *)&bootNvramData, sizeof (NVRAM_DATA)) ;

    printk("boardid: %s\n", bootNvramData.szBoardId);
}

static int __init m29_init(void) {
    cfi_flash_sched_init();
    kerSysEarlyFlashInit();
    return 0;
}

static void __exit m29_cleanup(void) {
}

module_init(m29_init);
module_exit(m29_cleanup);

MODULE_DESCRIPTION("m29 flash driver testing");
MODULE_VERSION("0.0.1");
MODULE_AUTHOR("Zhifeng Gu <guzhifeng1979@hotmail.com>");
MODULE_LICENSE("GPL");
