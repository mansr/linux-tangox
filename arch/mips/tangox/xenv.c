
/*********************************************************************
 Copyright (C) 2001-2008
 Sigma Designs, Inc.

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#include <asm/bootinfo.h>
#include <asm/unaligned.h>
#include <asm/io.h>

#include "memmap.h"
#include "lram.h"
#include "setup.h"
#include "xenvkeys.h"

#define XENV_HDR_SIZE 36

#define MAX_XENV_SIZE   16384
#define MAX_LR_XENV2_RO 628
#define MAX_LR_XENV2_RW 628

#define ENABLED_BIT(cfg, sh) \
	(IS_ENABLED(CONFIG_TANGOX_XENV_DEF_##cfg) << sh##_SHIFT)

static u32 enabled_devices =
	ENABLED_BIT(PCI_ID1,	PCI1)		|
	ENABLED_BIT(PCI_ID2,	PCI2)		|
	ENABLED_BIT(PCI_ID3,	PCI3)		|
	ENABLED_BIT(PCI_ID4,	PCI4)		|
	ENABLED_BIT(ENET,	ETHERNET)	|
	ENABLED_BIT(FIP,	FIP)		|
	ENABLED_BIT(I2CM,	I2CM)		|
	ENABLED_BIT(I2CS,	I2CS)		|
	ENABLED_BIT(IR,		IR)		|
	ENABLED_BIT(PCIHOST,	PCIHOST)	|
	ENABLED_BIT(USB,	USB)		|
	ENABLED_BIT(SATA,	SATA);

static u32 sata_channel_cfg;
static u8 mac_address[2][6];

#ifdef CONFIG_TANGOX_XENV_READ

static u32 mac0_lo, mac0_hi;
static u32 mac1_lo, mac1_hi;

struct xenv_key {
	const char *name;
	void *dest;
	size_t size;
	unsigned elems;
};

#define XENV_KEY(n, d) { .name = n, .dest = &d, .size = sizeof(d) }

#define XENV_KEY_ARR(n, d) { .name = n "%n", .dest = &d[0], \
			     .size = sizeof(d[0]), .elems = ARRAY_SIZE(d) }

static const struct xenv_key xenv_keys[] __initconst = {
	XENV_KEY(XENV_KEY_ENABLED_DEVICES, enabled_devices),
	XENV_KEY(XENV_KEY_SATA_CHANNEL_CFG, sata_channel_cfg),

	{ 0 }
};

static const struct xenv_key xenv2_keys[] __initconst = {
	XENV_KEY(XENV_LRRW_ETH_MACL, mac0_lo),
	XENV_KEY(XENV_LRRW_ETH_MACH, mac0_hi),

	XENV_KEY(XENV_LRRW_ETH1_MACL, mac1_lo),
	XENV_KEY(XENV_LRRW_ETH1_MACH, mac1_hi),

	{ 0 }
};

static void xenv_copy(const struct xenv_key *xk, int offset,
		      const char *rname, const void *rdata, u32 rsize)
{
	void *dest;
	u32 size;

	size = min(rsize, xk->size);
	if (size < rsize)
		pr_warn("xenv data for '%s' truncated from %d to %d\n",
			rname, rsize, xk->size);

	if (size < xk->size)
		memset(xk->dest, 0, xk->size);

	dest = (char *)xk->dest + offset;

	pr_debug("xenv key '%s' size %d @ %p -> %p\n",
		 rname, size, rdata, dest);

	memcpy(dest, rdata, size);
}

static int __init xenv_match_arr(const struct xenv_key *xk, const char *rname,
				 const void *rdata, u32 rsize)
{
	int idx, len = 0;

	if (sscanf(rname, xk->name, &idx, &len) != 1)
		return -1;

	if (len != strlen(rname))
		return -1;

	if (idx >= xk->elems) {
		pr_warn("xenv index out of bounds for '%s'\n", rname);
		return -1;
	}

	xenv_copy(xk, idx * xk->size, rname, rdata, rsize);

	return 0;
}

static int __init xenv_match(const struct xenv_key *xk, const char *rname,
			     const void *rdata, u32 rsize)
{
	if (xk->elems)
		return xenv_match_arr(xk, rname, rdata, rsize);

	if (strcmp(rname, xk->name))
		return -1;

	xenv_copy(xk, 0, rname, rdata, rsize);

	return 0;
}

static int __init xenv_parse(const char *xenv, u32 max_size,
			     const struct xenv_key *xk)
{
	u32 size = get_unaligned_le32(xenv);

	if (size <= XENV_HDR_SIZE || size > max_size) {
		pr_err("XENV: invalid size %d\n", size);
		return -1;
	}

	xenv += XENV_HDR_SIZE;
	size -= XENV_HDR_SIZE;

	pr_debug("XENV: parsing %d bytes at %p\n", size, xenv);

	while (size > 2) {
		int rlen, nlen, dlen;
		const void *data;
		int i;

		rlen = get_unaligned_be16(xenv) & 0xfff;

		pr_debug("  record length %d\n", rlen);

		if (rlen > size) {
			pr_err("XENV: record too large: %d\n", rlen);
			break;
		}

		if (rlen < 3) {
			pr_err("XENV: invalid record length %d\n", rlen);
			break;
		}

		xenv += 2;
		size -= 2;
		rlen -= 2;

		nlen = strnlen(xenv, rlen);

		pr_debug("  record name length %d\n", nlen);

		if (nlen < 1) {
			pr_err("XENV: zero-length recorn name\n");
			goto next;
		}

		if (nlen >= rlen) {
			pr_err("XENV: unterminated record name %*s\n",
			       nlen, xenv);
			goto next;
		}

		pr_debug("  record name %s\n", xenv);

		data = xenv + nlen + 1;
		dlen = rlen - nlen - 1;

		for (i = 0; xk[i].name; i++) {
			if (!xenv_match(&xk[i], xenv, data, dlen))
				break;
		}

	next:
		xenv += rlen;
		size -= rlen;
	}

	if (size > 0)
		pr_warn("XENV: %d bytes trailing junk\n", size);

	return 0;
}

static unsigned char xenv_buf[MAX_XENV_SIZE] __initdata;

static void * __init xenv_get(unsigned long addr)
{
	unsigned long xenv_addr;
	unsigned long xenv_size;
	void __iomem *xenv_base;
	void __iomem *xenv_loc;
	void *xenv = NULL;

	xenv_loc = ioremap(addr, 4);
	xenv_addr = readl(xenv_loc);
	iounmap(xenv_loc);

	if (!xenv_addr)
		return NULL;

	xenv_base = ioremap(xenv_addr, MAX_XENV_SIZE);
	if (!xenv_base)
		return NULL;

	xenv_size = readl(xenv_base);
	if (xenv_size > MAX_XENV_SIZE)
		goto end;

	memcpy_fromio(xenv_buf, xenv_base, xenv_size);
	xenv = xenv_buf;

end:
	iounmap(xenv_base);

	return xenv;
}

static int __init xenv_read_content(void)
{
	const void *xenv, *xenv2;
	uint32_t mac_lo, mac_hi;

	xenv = xenv_get(CPU_BASE + LR_ZBOOTXENV_LOCATION);
	if (!xenv)
		return -1;

	xenv2 = ioremap(CPU_BASE + LR_XENV2_RW, MAX_LR_XENV2_RW);

	xenv_parse(xenv, MAX_XENV_SIZE, xenv_keys);
	xenv_parse(xenv2, MAX_LR_XENV2_RW, xenv2_keys);

	if (mac0_lo && mac0_hi) {
		mac_hi = cpu_to_be32(mac0_hi);
		mac_lo = cpu_to_be32(mac0_lo);
		memcpy(mac_address[0], (u8 *)&mac_hi + 2, 2);
		memcpy(mac_address[0] + 2, &mac_lo, 4);
	}

	if (mac1_lo && mac1_hi) {
		mac_hi = cpu_to_be32(mac1_hi);
		mac_lo = cpu_to_be32(mac1_lo);
		memcpy(mac_address[1], (u8 *)&mac_hi + 2, 2);
		memcpy(mac_address[1] + 2, &mac_lo, 4);
	}

	return 0;
}

struct xenv_device {
	int bit;
	const char *name;
};

static const struct xenv_device xenv_devices[] __initconst = {
	{ ISAIDE_SHIFT,		"ISA/IDE"	},
	{ BMIDE_SHIFT,		"BMIDE"		},
	{ SATA_SHIFT,		"SATA"		},
	{ ETHERNET_SHIFT,	"Ethernet0"	},
	{ ETHERNET1_SHIFT,	"Ethernet1"	},
	{ USB_SHIFT,		"USB"		},
	{ IR_SHIFT,		"IR"		},
	{ FIP_SHIFT,		"FIP"		},
	{ I2CM_SHIFT,		"I2CM"		},
	{ I2CS_SHIFT,		"I2CS"		},
	{ SDIO_SHIFT,		"SDIO0"		},
	{ SDIO1_SHIFT,		"SDIO1"		},
	{ PCIHOST_SHIFT,	"PCI"		},
	{ PCI1_SHIFT,		"PCI1"		},
	{ PCI2_SHIFT,		"PCI2"		},
	{ PCI3_SHIFT,		"PCI3"		},
	{ PCI4_SHIFT,		"PCI4"		},
	{ PCI5_SHIFT,		"PCI5"		},
	{ PCI6_SHIFT,		"PCI6"		},
	{ SCARD_SHIFT,		"SCARD0"	},
	{ SCARD1_SHIFT,		"SCARD1"	},
	{ GNET_SHIFT,		"GNET"		},
};

static void __init tangox_device_info(void)
{
	int i;

	pr_info("XENV enabled devices:\n");
	pr_info(" ");

	for (i = 0; i < ARRAY_SIZE(xenv_devices); i++)
		if (enabled_devices & (1 << xenv_devices[i].bit))
			pr_cont(" %s", xenv_devices[i].name);

	pr_cont("\n");
}
#endif

int __init xenv_config(void)
{
#ifdef CONFIG_TANGOX_XENV_READ
	if (xenv_read_content()) {
		pr_err("Failed to load XENV params\n");
		return -1;
	}

	tangox_device_info();
#endif
	return 0;
}

int tangox_ethernet_enabled(unsigned int i)
{
	switch (i) {
	case 0: return (enabled_devices >> ETHERNET_SHIFT) & 1;
	case 1: return (enabled_devices >> ETHERNET1_SHIFT) & 1;
	}

	return 0;
}

unsigned char *tangox_ethernet_getmac(unsigned int i)
{
	if (i > 1)
		return NULL;

	return mac_address[i];
}

unsigned int tangox_sata_cfg(void)
{
	return sata_channel_cfg;
}
