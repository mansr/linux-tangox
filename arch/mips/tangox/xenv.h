
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/

/**
  @file   xenv.h
  @brief

  The prototypes below act on a clear area respecting the xenv format.

  (De)ciphering the  area, as well as committing  the changes (writing
  the area to flash) are trivial add-ons on top of this API.

  The underlying  implementation is not optimized for  speed (read and
  write in  O(nrecords)). There  is no fragmentation  (data completely
  rearranged at each write).

  It is  not advised to repeatedly  act on flash stored  data, for the
  device does  not support  unlimited read/write operations  (refer to
  spec)

  Power  loss when  committing the  changes cause  loss of  the stored
  data.  This  can be avoided  by storing the  same data twice  to two
  different sectors.

  --------------------------------------------------------------------------
  Specification   of   a   secure   storage   on   serial   flash   of
  reboot-persistent data (xenv format)

  We  describe a  way  to  concatenate (at  most  4KByte area  (12bits
  limit)) variable  length records identified  by a string,  the `key'
  (working much like Windows registry)

  Page, seen as a byte array, is

  0          4                   24                 env_size         4KB
  | env_size | SHA-1 of following | rec0 | rec1 | .. | recn | xx xx .. |

  The SHA-1 extent is env_size-24.

  Description of a rec (bytes):

  |4bits   12bits (2bytes)   | variable, NULL terminated | variable         |
  attr     total record size   record name (string)        record value

  attr =
   XENV_ATTR_RW
   XENV_ATTR_RO can be written once only but can be deleted
   XENV_ATTR_OTP this record can be written once only and cannot be deleted
  --------------------------------------------------------------------------

  @author Emmanuel Michon
  @date   2005-05-17
*/

#ifndef __XENV_H__
#define __XENV_H__

#include "setup.h"

#define MAX_XENV_SIZE   16384

/**
   Check for compliance with xenv format

   May be corrupted by:
   - forgot to format
   - power loss during sflash write
   - intrusion

   @param base
   @param size
   @return -ReturnValue-: env_size>=0 if valid, -1 if not.
*/
int xenv_isvalid(u32 *base, u32 maxsize);

int xenv_foreach(u32 *base, u32 size,
		 void (*cb)(char *recordname, void *data, u32 datasize));

int xenv_get(u32 *base, u32 size, char *recordname, void *dst, u32 *datasize);
int xenv_set(u32 *base, u32 size, char *recordname, void *src, u8 attr, u32 datasize);

void xenv_dump(u32 *base, u32 size);

#ifdef CONFIG_TANGO3
/*
 * XENV sizes LRRO/LRRW
 */
#define MAX_LR_XENV2_RO 768
#define MAX_LR_XENV2_RW 768
#endif

#endif // __XENV_H__
