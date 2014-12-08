#ifndef __PLATFORM_DATA_SATA_TANGOX_PHY_H
#define __PLATFORM_DATA_SATA_TANGOX_PHY_H

struct tangox_sata_phy_pdata {
	int clk_sel;
	int clk_ref;
	int tx_erc;
	int tx_ssc;
	int rx_ssc0;
	int rx_ssc1;
};

#endif /* __PLATFORM_DATA_SATA_TANGOX_PHY_H */
