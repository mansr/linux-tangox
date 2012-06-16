/*********************************************************************
 Copyright (C) 2001-2011
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#ifndef _ASSEMBLER_

struct cpulaunch {
    unsigned long	pc;
    unsigned long	gp;
    unsigned long	sp;
    unsigned long	a0;
    unsigned long	_pad[3]; /* pad to cache line size to avoid thrashing */
    unsigned long	flags;
};

#else

#define LOG2CPULAUNCH	5
#define	LAUNCH_PC	0
#define	LAUNCH_GP	4
#define	LAUNCH_SP	8
#define	LAUNCH_A0	12
#define	LAUNCH_FLAGS	28

#endif

#define LAUNCH_FREADY	1
#define LAUNCH_FGO	2
#define LAUNCH_FGONE	4

#define CPULAUNCH	(REG_BASE_cpu_block + LR_CPULAUNCH)
#define NCPULAUNCH	4

/* Polling period in count cycles for secondary CPU's */
#define LAUNCHPERIOD	10000
