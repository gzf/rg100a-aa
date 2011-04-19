#include "backport.h"

int 
nf_ct_get_tuplepr(const struct sk_buff *skb, unsigned int nhoff,
                  u_int16_t l3num, struct nf_conntrack_tuple *tuple)
{
        struct nf_conntrack_l3proto *l3proto;
        struct nf_conntrack_l4proto *l4proto;
        unsigned int protoff;
        u_int8_t protonum;
        int ret;

        rcu_read_lock();

        l3proto = __nf_ct_l3proto_find(l3num);
        ret = l3proto->prepare((struct sk_buff**) &skb, nhoff, &protoff, &protonum);
        if (ret != NF_ACCEPT) {
                rcu_read_unlock();
                return false;
        }

        l4proto = __nf_ct_l4proto_find(l3num, protonum);

        ret = nf_ct_get_tuple(skb, nhoff, protoff, l3num, protonum, tuple,
                              l3proto, l4proto);

        rcu_read_unlock();
        return ret;
}
