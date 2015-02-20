
/*********************************************************************
 Copyright (C) 2001-2008
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#include "setup.h"
#include "xenv.h"
#include "sha.h"

#if defined(CONFIG_TANGO3)
#include <asm/tango3/rmdefs.h>
#endif

#if 1
# define DPRINTK(fmt, args...)	printk(KERN_DEBUG "xenv: " fmt, ## args)
#else
# define DPRINTK(fmt, args...)
#endif

#if defined(CONFIG_TANGO2)
#define XENV_DIGEST_SIZE   SHA1_DIGEST_SIZE
void sha1_full(u8 *digest, const u8 *src, u32 len);
#elif defined(CONFIG_TANGO3)
#define XENV_DIGEST_SIZE   SHA256_DIGEST_SIZE
void sha256_full(u8 *digest, const u8 *src, u32 len);
#endif

#define XENV_HDR_SIZE      (XENV_DIGEST_SIZE + 4)
#define REC_SIZE(x)	   ((((u16)x[0] & 0xff) << 8) | ((u16)x[1] & 0xff))
#define REC_ATTR(x)	   ((x[0] & 0xff) >> 4)

#define xos_strlen(x)	   strlen(x)
#define xos_strcmp(x,y)	   strcmp(x,y)
#define xos_memcpy(x,y,z)  memcpy(x,y,z)

void __init xenv_digest_full(u8 *digest, const u8 *src, u32 len)
{
#if defined(CONFIG_TANGO2)
	sha1_full(digest, src, len);
#elif defined(CONFIG_TANGO3)
	sha256_full(digest, src, len);
#endif
}

#if 0
static void dump_digest(const u8 *digest)
{
	int i;
	printk("Dump digest\n");
	for (i = 0; i < XENV_DIGEST_SIZE; i++, digest++)
		printk("%02x ", *digest);
	printk("\n");
}
#endif

/*
 * check for valid XENV at given address
 */
int __init xenv_isvalid(u32 *base, u32 maxsize)
{
	u32 env_size = base[0];
	u32 hash[XENV_DIGEST_SIZE/4];

	if ((XENV_HDR_SIZE <= env_size) && (env_size <= maxsize)) {
		memset(hash, 0, sizeof (hash));
		xenv_digest_full((u8 *)hash, (const u8 *)(base + (XENV_HDR_SIZE / 4)), env_size - XENV_HDR_SIZE);
		if (memcmp((const u8 *)(base + 1), hash, XENV_DIGEST_SIZE) != 0) {
			DPRINTK("corrupted\n");
			return -1;
		}
		/* valid xenv ! */
		return env_size;
	}
	DPRINTK("runaway %d\n", env_size);
	return -1;
}

int __init xenv_foreach(u32 *base, u32 size,
			void (*cb)(char *recordname, void *data, u32 datasize))
{
	int i;

	/* jump over first header */
	i = XENV_HDR_SIZE;

	/* loop on each record name */
	while (i < size) {
		u16 rec_size;
		char *p, *recordname;
		void *data;
		u32 key_len, data_len;

		p = (char *)base + i;
		rec_size = REC_SIZE(p);
		recordname = p + 2;
		key_len = strlen(recordname);
		data = recordname + key_len + 1;
		data_len = rec_size - 2 - key_len - 1;

		cb(recordname, data, data_len);
		i += rec_size;
	}

	return -1;
}

#ifdef CONFIG_TANGO3
/* Use this to set xenv to lrrw */
static int __init xenv_lookup(RMuint32 *base,RMuint32 size,RMascii *recordname)
{
	RMascii *p;
	int i;
	int env_size;
	
	env_size=xenv_isvalid((u32 *)base,size);
	
	if (env_size<0) 
		return -2;
	
	// RMDBGLOG((LOCALDBG, "[%s]\n",recordname));
	
	p=(RMascii *)base;
	i=XENV_HDR_SIZE; 			// jump over header
	
	while(i<env_size){
		RMuint16 rec_size=((p[i]&0xf)<<8) + (((RMuint16)p[i+1])&0xff);

		if (!xos_strcmp(recordname, p+i+2)) 
			return i;
		
		i+=rec_size;
	}
	
	return -1;
}

/* Use this to get xenv to lrrw/lrro */
int __init xenv_get(u32 *base, u32 size, char *recordname, void *dst, u32 *datasize)
{
	RMascii *p = (RMascii *)base;
	RMuint32 data_len;
	RMuint32 key_len;
	RMuint32 env_size;
	
	int i;	
	
	env_size=base[0];
	i = xenv_lookup((RMuint32 *)base, size, recordname);
	if(i==-2) 
		return -2 /* RM_ERROR */;
	if(i==-1) 
		return -1 /* RM_NOT_FOUND */;

	// else we found the record
	key_len=xos_strlen(recordname);
	data_len=((p[i] & 0xf)<<8) + (((RMuint32)p[i+1])&0xff);
	data_len-=2+key_len+1;
	
	if(data_len>*datasize) {
		*datasize=data_len;

		// RMDBGLOG((ENABLE,"cannot store result\n"));
		return -3 /* RM_INSUFFICIENT_SIZE */;
	}
	
	*datasize=data_len;
	xos_memcpy(dst, p+i+2+key_len+1, data_len);

	// RMDBGLOG((LOCALDBG, "found [%s], length %d\n",recordname,*datasize));

	return 0 /* RM_OK */;
}

int __init xenv_set(u32 *base, u32 size, char *recordname, void *src, u8 attr, u32 datasize)
{
	RMascii *p;
	RMuint32 env_size;
	RMuint32 rec_attr;
	RMuint32 rec_size;
	RMuint32 key_len;
	int i;

	// RMDBGLOG((LOCALDBG, "[%s], length %d\n", recordname,datasize));
	
	p=(RMascii *)base;
	i = xenv_lookup((RMuint32 *)base, size, recordname);
	if(i==-2) 
		return -1 /* RM_ERROR */;
	env_size=base[0];

	if(i>=0) {
		// RMDBGLOG((LOCALDBG, "deleting record\n"));
		
		rec_attr=p[i]>>4;
		rec_size=((p[i]&0xf)<<8) + (((RMuint32)p[i+1])&0xff);
			
#if 0
		if ((rec_attr==XENV_ATTR_RO)&&src) {
			// RMDBGLOG((ENABLE,"wanna change ro record\n"));
			return RM_INVALIDMODE;
		}
		
		if (rec_attr==XENV_ATTR_OTP) {
			// RMDBGLOG((ENABLE,"wanna change/clear otp record\n"));
			return RM_INVALIDMODE;
		}
#endif

		// delete the record. Supposes memcpy is implemented increasing.
		xos_memcpy(p+i, p+i+rec_size, env_size-(i+rec_size));
		env_size-=rec_size;
	}
		
	// add the record at the end if needed.
	if(src) {
		i=env_size;
		key_len=xos_strlen(recordname);
		rec_size=2+key_len+1+datasize;

		if((i+rec_size)>=size)
			return -3 /* RM_INSUFFICIENT_SIZE */;
		
		p[i]=((attr&0xf)<<4) | ((rec_size>>8)&0xf);
		p[i+1]=rec_size&0xff;
		
		xos_memcpy(p+i+2, recordname, key_len+1);
		xos_memcpy(p+i+2+key_len+1, src, datasize);
		
		env_size+=rec_size;
	}
	
	base[0]=env_size;
	//full_sha256(base+1,(const RMuint8 *)(base+9),env_size-36,0);
	xenv_digest_full((u8 *)(base+1), (const u8 *)(base + (XENV_HDR_SIZE / 4)), env_size - XENV_HDR_SIZE);

	return 0 /* RM_OK */;
}
#endif

#ifdef CONFIG_TANGOX_XENV_DUMP
void __init xenv_dump(u32 *base, u32 size)
{
	int i;
	u32 records = 0;

	printk("@%p\n", base);

	/* jump over first header */
	i = XENV_HDR_SIZE;

	while (i < size){
		u8 rec_attr;
		u16 rec_size;
		char *p, *recordname, *x;
		u32 key_len;

		p = (char *)base + i;
		rec_attr = REC_ATTR(p);
		rec_size = REC_SIZE(p);
		recordname = p + 2;
		key_len = strlen(recordname);

		printk("(0x%02x) [%s] =", rec_attr, recordname);
		for (x = recordname + key_len + 1;
		     x < recordname + rec_size - 2; x++)
			printk(" %02x", (u8)*x);
		printk(" .\n");

		records++;
		i += rec_size;
	}

	printk("%d records, %d bytes\n\n", records, size);
}
#endif
