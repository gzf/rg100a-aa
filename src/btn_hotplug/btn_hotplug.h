#ifndef _BTN_HOTPLUG_
#define _BTN_HOTPLUG_

#include <linux/init.h>
#include <net/sock.h>

#define NUM_OF_EXTIRQ   6

struct bh_event {
	char			*name;
	char			*action;
	unsigned long	seen;
    
	struct sk_buff		*skb;
	struct work_struct	work;
};

extern struct sock *uevent_sock;
extern u64 uevent_next_seqnum(void);


#endif
