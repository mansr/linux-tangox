
/*
 * Converting XENV partition information to psudo HDD-like partitions.
 */

#ifndef CONFIG_TANGO3
#error Not supported platform.
#endif

/* Only applicable to Tango3 */
#include <asm/io.h>
#include <asm/tango3/tango3_gbus.h>
#include <asm/tango3/hardware.h>
#include "check.h"

#include "xenv_part.h"

/* XENV query function prototypes */
extern int tangox_flash_get_info(int cs, unsigned int *size, unsigned int *part_count);
extern int tangox_flash_get_parts(int cs, unsigned int offset[], unsigned int size[]);

int xenv_partition(struct parsed_partitions *state, struct block_device *bdev)
{
	struct xenv_disk_marker *sdisk = bdev->bd_disk->private_data;
	unsigned int cs, cs_part, cs_size;
	unsigned int offsets[SDND_MINORS], sizes[SDND_MINORS];
	int i;

	if (sdisk == NULL)
		goto out;
	else if (sdisk->magic != SDND_MAGIC)
		goto out;
	else if ((cs = sdisk->cs) >= SDND_MAX_CS)
		goto out;

	tangox_flash_get_info(cs, &cs_size, &cs_part);

	if ((cs_part == 0) || (cs_size == 0)) 
		goto out;
	else if (cs_part > (SDND_MINORS - 2))
		cs_part = SDND_MINORS - 2;

	tangox_flash_get_parts(cs, &offsets[0], &sizes[0]);
	state->limit = SDND_MINORS - 1;

	for (i = 0; i < cs_part; i++) 
		put_partition(state, i + 1, offsets[i] / KERNEL_SECTOR_SIZE, sizes[i] / KERNEL_SECTOR_SIZE);

	return 1;

out:
	return 0;
}

