/* IPID modification module for IP tables */

#ifndef _IPT_IPID_H
#define _IPT_IPID_H

//#define	IP_IPID_PACE  0
//#define IP_IPID_CHAOTIC 1
enum {
	IP_IPID_PACE = 0,
	IP_IPID_CHAOTIC
}IP_IPID;

struct ipt_IPID_info {
	u_int8_t	mode;
	u_int16_t	ipid;
};


#endif
