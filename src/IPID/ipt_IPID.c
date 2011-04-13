/* IPID modification target for IP tables
 * This software is distributed under the terms of GNU GPL
 */

#if defined(MODVERSIONS)
#include <linux/modversions.h>
#endif
#include <linux/module.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <net/checksum.h>
#include <linux/random.h>

#include <linux/netfilter/x_tables.h>
#include "ipt_IPID.h"

MODULE_AUTHOR("Ssffzz1 <ssffzz1@126.com>");
MODULE_DESCRIPTION("IP tables IPID modification module");
MODULE_LICENSE("GPL");

u_int16_t get_id(const char name []){
	u_int16_t i=0;
	u_int16_t o=0;
	while(name[i] != '\0'){
		o ^= name[i];
		i++;
	}
	return(o);
}


#if LINUX_VERSION_CODE <KERNEL_VERSION(2,6,0)

static unsigned int ipt_ipid_target(struct sk_buff **pskb, unsigned int hooknum,
		const struct net_device *in, const struct net_device *out,
		const void *targinfo, void *userinfo)
{
	struct iphdr *iph = (*pskb)->nh.iph;
	const struct ipt_IPID_info *info = targinfo;
	static u_int16_t new_ipid[255];
	u_int16_t id=0;

	id = get_id(out->name);
	switch(info->mode){
		case IP_IPID_PACE:
		new_ipid[id] += info->ipid;
		break;
		case IP_IPID_CHAOTIC:
		default:
		get_random_bytes(&(new_ipid[id]),sizoef(new_ipid[i]));
	}			
	iph->id = htons(new_ipid[id]);
	iph->check = 0;
	iph->check = ip_fast_csum((char *)iph,iph->ihl);

	return IPT_CONTINUE;
}

static int ipt_ipid_checkentry(const char *tablename,
		const struct ipt_entry *e,
		void *targinfo,
		unsigned int targinfosize,
		unsigned int hook_mask)
{

	if (targinfosize != IPT_ALIGN(sizeof(struct ipt_IPID_info))) {
		printk(KERN_WARNING "IPID: targinfosize %u != %Zu\n",
				targinfosize,
				IPT_ALIGN(sizeof(struct ipt_IPID_info)));
		return 0;	
	}	

	if (strcmp(tablename, "mangle")) {
		printk(KERN_WARNING "IPID: can only be called from \"mangle\" table, not \"%s\"\n", tablename);
		return 0;
	}

	return 1;
}

static struct ipt_target ipt_IPID = { { NULL, NULL }, "IPID", 
	ipt_ipid_target, ipt_ipid_checkentry, NULL, THIS_MODULE };

static int __init init(void)
{
	return ipt_register_target(&ipt_IPID);
}

static void __exit fini(void)
{
	ipt_unregister_target(&ipt_IPID);
}

module_init(init);
module_exit(fini);

#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)

static unsigned int 
ipt_ipid_target(struct sk_buff **pskb, const struct net_device *in, 
		const struct net_device *out, unsigned int hooknum,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,17))
		const struct xt_target *target,
#endif 
		const void *targinfo)
{
	struct iphdr *iph;
	const struct ipt_IPID_info *info = targinfo;
	static u_int16_t new_ipid[255];
	u_int16_t id;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,17))
	if(!skb_make_writable(pskb,(*pskb)->len))
		return NF_DROP;
#else
	if (!skb_ip_make_writable(pskb, (*pskb)->len))
		return NF_DROP;
#endif
	iph = (*pskb)->nh.iph;
	
	id=get_id(out->name);		 
	switch(info->mode){
		case IP_IPID_PACE:
		new_ipid[id] += info->ipid;
		break;
		case IP_IPID_CHAOTIC:
		default:
		get_random_bytes(&(new_ipid[id]),sizeof(new_ipid[id]));
	}
	iph->id = htons(new_ipid[id]);
	iph->check = 0;
	iph->check = ip_fast_csum((char *)iph,iph->ihl);
	
	return XT_CONTINUE;
}

static int ipt_ipid_checkentry(const char *tablename,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,17))
		const void *e,
		const struct xt_target *target,
#else
		const struct ipt_entry *e,
#endif
		void *targinfo,
		unsigned int hook_mask)
{
	//struct ipt_IPID_info *info = targinfo;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,17))
	return 1;
#else
	if (targinfosize != IPT_ALIGN(sizeof(struct ipt_IPID_info))) {
		printk(KERN_WARNING "IPID: targinfosize %u != %Zu\n",
				targinfosize,
				IPT_ALIGN(sizeof(struct ipt_IPID_info)));
		return 0;	
	}	

	if (strcmp(tablename, "mangle")) {
		printk(KERN_WARNING "IPID: can only be called from \"mangle\" table, not \"%s\"\n", tablename);
		return 0;
	}

	return 1;
#endif
}

static struct xt_target ipt_IPID = { 
	.name = "IPID",
    .family = AF_INET,
	.target = ipt_ipid_target, 
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,17))
	.targetsize = sizeof(struct ipt_IPID_info),
	.table = "mangle",
#endif
	.checkentry = ipt_ipid_checkentry, 
	.me = THIS_MODULE, 
};

static int __init init(void)
{
	return xt_register_target(&ipt_IPID);
}

static void __exit fini(void)
{
	xt_unregister_target(&ipt_IPID);
}

module_init(init);
module_exit(fini);

#endif
