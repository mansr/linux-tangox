#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/bitops.h>

#include <asm/bcache.h>
#include <asm/bootinfo.h>
#include <asm/cache.h>
#include <asm/cacheops.h>
#include <asm/cpu.h>
#include <asm/cpu-features.h>
#include <asm/io.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/r4kcache.h>
#include <asm/mmu_context.h>
#include <asm/war.h>
#include <asm/cacheflush.h> /* for run_uncached() */

// those are left un-documented and have many bizarre requirements. read the ASM. loopCnt is cache lines + 2. 
void mips_memcopy_std(int *p_src, int *p_dst, int loopCnt);
void mips_memcopy_std_ua(int *p_uasrc, int *p_dst, int loopCnt);
void mips_memset(int v,int *p_dst, int loopCnt);
size_t mips_memcmp(int *p_src, int *p_dst, int loopCnt);
size_t mips_memcmp_ua(int *p_uasrc, int *p_dst, int loopCnt);
size_t mips_strnlen(int *p_src, int loopCnt); // return non-zero means found
size_t mips_strnlen_c(int *p_src,int v, int loopCnt); // return non-zero means found
void mips_hhblend(int *p_src0,int *p_src1, int *p_dst, int loopCnt);

size_t __strnlen_user_nocheck_asm_alt(const char *s, size_t maxlen)
{
	char *src=(void *)s;
	int n=maxlen;
	int lines;

	// word align src

	while ((n>0)&&((int)src&3)) {
		if (*src==0)
			return maxlen-n;
		n--;
		src++;
	}
	
	// do as many lines as possible

	lines=n>>5;
	if (lines>0) {
		int l;
		
		l=lines-mips_strnlen((int *)src,lines-1);

		src+=l<<5; 
		n-=l<<5;
	}
	
	// finish quietly

	while (n>0) {
		if (*src==0)
			return maxlen-n;
		n--;
		src++;
	}
	
	return maxlen;
}
