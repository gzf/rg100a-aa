#ifndef _BTN_HOTPLUG_
#define _BTN_HOTPLUG_

#include <linux/init.h>

void __init ext_intr_init(void);
void __exit ext_intr_cleanup(void);

#endif
