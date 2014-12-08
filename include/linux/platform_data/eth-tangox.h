#ifndef __PLATFORM_DATA_ETH_TANGOX_H
#define __PLATFORM_DATA_ETH_TANGOX_H

#include <linux/if_ether.h>

struct tangox_enet_pdata {
	unsigned char mac_addr[ETH_ALEN];
	int gigabit;
};

#endif /* __PLATFORM_DATA_ETH_TANGOX_H */
