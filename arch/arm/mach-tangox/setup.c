#include <asm/mach/arch.h>
#include <asm/hardware/cache-l2x0.h>

#include "smc.h"

#define TANGOX_L2C_AUX_CTRL	(L2C_AUX_CTRL_SHARED_OVERRIDE	|	\
				 L310_AUX_CTRL_NS_LOCKDOWN	|	\
				 L310_AUX_CTRL_NS_INT_CTRL	|	\
				 L310_AUX_CTRL_DATA_PREFETCH	|	\
				 L310_AUX_CTRL_INSTR_PREFETCH)

static void tangox_l2c_write_sec(unsigned long val, unsigned reg)
{
	unsigned smc_op;

	switch (reg) {
	case L2X0_CTRL:
		smc_op = 0x102;
		break;

	case L2X0_AUX_CTRL:
		smc_op = 0x109;
		break;

	case L2X0_DEBUG_CTRL:
		smc_op = 0x100;
		break;

	case L310_PREFETCH_CTRL:
		smc_op = 0x113;
		break;

	default:
		WARN_ONCE(1, "L2C310: ignoring write to reg 0x%x\n", reg);
		return;
	}

	tangox_smc1(smc_op, val);
}

static const char *tangox_dt_compat[] = {
	"sigma,smp8734-soc",
	"sigma,smp8756-soc",
	"sigma,smp8758-soc",
	"sigma,smp8759-soc",
	NULL
};

DT_MACHINE_START(TANGOX_87XX, "Sigma Designs SMP87xx")
	.dt_compat	= tangox_dt_compat,
	.l2c_write_sec	= tangox_l2c_write_sec,
	.l2c_aux_val	= TANGOX_L2C_AUX_CTRL,
	.l2c_aux_mask	= 0xc3bffffe,
MACHINE_END
