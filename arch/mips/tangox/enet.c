#include <linux/init.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <asm/io.h>

#include "memmap.h"
#include "setup.h"

static int tangox_phy_init(struct phy_device *phydev)
{
	int err;

	err = phy_write(phydev, 31, 1);
	if (!err)
		err = phy_write(phydev, 28, 0x5000);
	phy_write(phydev, 31, 0);

	return err;
}

static void __init tangox_enet_set_bw(void __iomem *reg)
{
	u32 val = readl(reg);
	val = (val & ~0xff) | 0x3f;
	writel(val, reg);
}

static int __init tangox_enet_setup(void)
{
	void __iomem *ctl = ioremap(SYS_BASE + 0x130, 8);
	int i;

	for (i = 0; i < 2; i++)
		if (tangox_ethernet_enabled(i))
			tangox_enet_set_bw(ctl + i * 4);

	iounmap(ctl);

	if (of_machine_is_compatible("syabas,pch-c200")) {
		pr_info("PCH C-200 Ethernet PHY fixup enabled\n");
		phy_register_fixup_for_uid(0x00070420, 0xfffffff0,
					   tangox_phy_init);
	}

	return 0;
}
device_initcall(tangox_enet_setup);
