#ifndef _BTN_HOTPLUG_
#define _BTN_HOTPLUG_

#include <linux/init.h>
#include <net/sock.h>

#define NUM_OF_EXTIRQ   4
#define DRV_NAME	    "bcm96358vw2_btn"

struct bh_event {
	char			*name;
	char			*action;
	unsigned long	seen;
    
	struct sk_buff		*skb;
	struct work_struct	work;
};

#endif
