
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 
 *********************************************************************/

/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/config.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/io.h>
#ifdef CONFIG_TANGO2
#include <asm/tango2/tango2_pci.h>
#elif defined(CONFIG_TANGO3)
#include <asm/tango3/tango3_pci.h>
#endif

/*
 * remapped address to access config space from kernel
 */
void __iomem *tangox_pci_config_base;

/*
 * list of devices for which we allow configuration access
 */
int enabled_devices[PCIEM86XX_IDSEL_MAX];

/*
 * find cfg address to use for given bus/device/Address
 */
#define CFG_ADDR(bus,devfn,where)	\
	(tangox_pci_config_base + (((bus) << 16) + ((devfn) << 8) + (where)))

/*
 * read/write callbacks for pci configuration memory access
 */
int tangox_pcibios_read(struct pci_bus *bus, unsigned int devfn,
			int where, int size, u32 *val)
{
	void __iomem *addr = CFG_ADDR(bus->number, devfn, where);
	int ret, slot;

	slot = PCI_SLOT(devfn);

	/*
	 * check that it is ok to use this slot
	 */
	if (!enabled_devices[slot] || slot >= PCIEM86XX_IDSEL_MAX)
		return PCIBIOS_DEVICE_NOT_FOUND;

	/*
	 * reject silly sizes
	 */
	if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	switch (size) {
	case 1:
		ret = tangox_cfg_read8(addr, val);
		break;

	case 2:
		ret = tangox_cfg_read16(addr, val);
		break;

	default:
		ret = tangox_cfg_read32(addr, val);
		break;
	}

	return ret;
}

int tangox_pcibios_write(struct pci_bus *bus, unsigned int devfn,
			 int where, int size, u32 val)
{
	void __iomem *addr = CFG_ADDR(bus->number, devfn, where);
	int ret, slot;

	slot = PCI_SLOT(devfn);

	/*
	 * check that it is ok to use this slot
	 */
	if (!enabled_devices[slot] || slot >= PCIEM86XX_IDSEL_MAX)
		return PCIBIOS_DEVICE_NOT_FOUND;

	/*
	 * reject silly sizes
	 */
	if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	switch (size) {
	case 1:
		ret = tangox_cfg_write8(addr, &val);
		break;

	case 2:
		ret = tangox_cfg_write16(addr, &val);
		break;

	default:
		ret = tangox_cfg_write32(addr, &val);
		break;
	}

	return ret;
}

struct pci_ops tangox_pci_ops = {
	.read = tangox_pcibios_read,
	.write = tangox_pcibios_write,
};
