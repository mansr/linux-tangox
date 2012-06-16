#include <asm/asm.h>
#include <asm/asm-offsets.h>
#include <asm/regdef.h>

.set noat
.set noreorder
	
/* size_t mips_strnlen(int *p_src, int loopCnt); */
LEAF(mips_strnlen)

	move	a2, $0

	li	v0, 0x80808080 
	li	v1, 0xfefefeff # nice trick, google on it
	
	pref	0, 0x0(a0)	# bring the first line of src
	pref	0, 0x20(a0)	# bring the first line of src
	pref	0, 0x40(a0)	# bring the first line of src
1:
	pref	0, 0x60(a0)	# bring the next lines of src

	lw 	t0, 0x0(a0)
	addu	t0, v1
	and	t0, v0
	addu	a2, t0

	lw 	t1, 0x4(a0)
	addu	t1, v1
	and	t1, v0
	addu	a2, t1

	lw 	t2, 0x8(a0)
	addu	t2, v1
	and	t2, v0
	addu	a2, t2

	lw 	t3, 0xc(a0)
	addu	t3, v1
	and	t3, v0
	addu	a2, t3

	lw 	t0, 0x10(a0)
	addu	t0, v1
	and	t0, v0
	addu	a2, t0

	lw 	t1, 0x14(a0)
	addu	t1, v1
	and	t1, v0
	addu	a2, t1

	lw 	t2, 0x18(a0)
	addu	t2, v1
	and	t2, v0
	addu	a2, t2

	lw 	t3, 0x1c(a0)
	addu	t3, v1
	and	t3, v0
	addu	a2, t3

	bnez	a2, 2f
	addu	a0, 0x20

	bnez	a1, 1b
	subu    a1, 1

2:	
	jr	ra
	addu	v0, a1, 1
END(mips_strnlen)
