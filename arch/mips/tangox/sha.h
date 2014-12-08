
/*********************************************************************
 Copyright (C) 2001-2008
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

#ifndef __SHA_H__
#define __SHA_H__

#ifdef CONFIG_CRYPTO_SHA1
#define SHA1_DIGEST_SIZE        20

struct sha1_ctx {
	u64 count;
        u32 state[5];
        u8 buffer[64];
};
#endif

#ifdef CONFIG_CRYPTO_SHA256
#define SHA256_DIGEST_SIZE      32

struct sha256_ctx {
	u32 count[2];
        u32 state[8];
        u8 buf[128];
};
#endif

#endif

