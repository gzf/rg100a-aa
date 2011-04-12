#ifndef _BACKPORT_H_
#define _BACKPORT_H_

#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/skbuff.h>
#include <net/netfilter/nf_conntrack.h>
#include <net/netfilter/nf_conntrack_core.h>
#include <net/netfilter/nf_conntrack_tuple.h>

int nf_ct_get_tuplepr(const struct sk_buff *skb, unsigned int nhoff,
                  u_int16_t l3num, struct nf_conntrack_tuple *tuple);

static inline unsigned char *skb_network_header(const struct sk_buff *skb)
{
    return skb->nh.raw;
}

static inline struct ipv6hdr *ipv6_hdr(const struct sk_buff *skb)
{
    return (struct ipv6hdr *)skb_network_header(skb);
}

static inline struct iphdr *ip_hdr(const struct sk_buff *skb)
{
    return (struct iphdr *)skb_network_header(skb);
}

static inline int skb_network_offset(const struct sk_buff *skb)
{
    return skb_network_header(skb) - skb->data;
}


#endif
