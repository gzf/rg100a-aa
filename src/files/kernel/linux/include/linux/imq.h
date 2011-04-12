#ifndef _IMQ_H
#define _IMQ_H

#include <linux/skbuff.h>

#define IMQ_MAX_DEVS   16

#define IMQ_F_IFMASK   0x7f
#define IMQ_F_ENQUEUE  0x80

struct skb_cb_imq {
	struct nf_info		*nf_info;
	unsigned char		imq_flags;
};

static inline struct skb_cb_imq* skb_imq(struct sk_buff *skb)
{
    return (struct skb_cb_imq*) &(skb->cb[sizeof(skb->cb) - sizeof(struct skb_cb_imq)]);
}

#endif /* _IMQ_H */
