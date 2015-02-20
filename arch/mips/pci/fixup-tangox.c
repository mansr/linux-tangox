
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

#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>

#ifdef CONFIG_TANGO2
#include <asm/tango2/hardware.h>
#include <asm/tango2/tango2_pci.h>
#elif defined(CONFIG_TANGO3)
#include <asm/tango3/hardware.h>
#include <asm/tango3/tango3_pci.h>
#endif

extern int tangox_pcidev_irq_map(int pci_idsel, int int_num);

int pcibios_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	/* return xenv config */
	return tangox_pcidev_irq_map(slot, pin);
}

int pcibios_plat_dev_init(struct pci_dev *dev)
{
	return 0;
}

/*
 * final fixup for our pci bus, called after all resource allocation.
 */
static void tangox_fixup(struct pci_dev *dev)
{
	dev->dev.platform_data = 0;
}

DECLARE_PCI_FIXUP_FINAL(PCI_ANY_ID, PCI_ANY_ID, tangox_fixup);

