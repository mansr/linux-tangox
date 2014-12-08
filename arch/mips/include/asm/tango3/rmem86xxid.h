/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
/**
  @file   rmem86xxid.h
  @brief  

  long description

  @author Emmanuel Michon
  @date   2004-09-22
*/

#ifndef __RMEM86XXID_H__
#define __RMEM86XXID_H__

/*
  the main chip ids 
  
  tango3 is for asic development (should be tango\infty)

  Usually, users do not set by hand, but thru `rmcflags' helper
*/
#define EM86XX_CHIPID_MAMBO      1000
#define EM86XX_CHIPID_MAMBOLIGHT 2000
#define EM86XX_CHIPID_TANGO      3000
#define EM86XX_CHIPID_TANGOLIGHT 4000
#define EM86XX_CHIPID_TANGO15    4500
#define EM86XX_CHIPID_TANGO2     5000
#define EM86XX_CHIPID_TANGO3    10000

#if (EM86XX_CHIP==EM86XX_CHIPID_MAMBO)
#define S_EM86XX_CHIPID "mambo"
#elif (EM86XX_CHIP==EM86XX_CHIPID_MAMBOLIGHT)
#define S_EM86XX_CHIPID "mambolight"
#elif (EM86XX_CHIP==EM86XX_CHIPID_TANGOLIGHT)
#define S_EM86XX_CHIPID "tangolight"
#elif (EM86XX_CHIP==EM86XX_CHIPID_TANGO15)
#define S_EM86XX_CHIPID "tango15"
#elif (EM86XX_CHIP==EM86XX_CHIPID_TANGO2)
#define S_EM86XX_CHIPID "tango2"
#elif (EM86XX_CHIP==EM86XX_CHIPID_TANGO3)
#define S_EM86XX_CHIPID "tango3"
#else
#error EM86XX_CHIP is not set
#endif

/* 
  revisions...
  
  Referring to whatever is written at the surface of the BGA,
  not the PCI revid / subid / etc. This detail is important for some chips
  are ambiguous software wise.
  
  1: ES1
  2: ES2
  3: ES3
  4: ES4 
  5: ES5 
  6: ES6 
  65: revA
  66: revB
  67: revC
  
  No ID, but numbers. For a 8630ES2 for instance: build with
  RMCFLAGS="... -DEM86XX_CHIP=EM86XX_CHIPID_TANGO2 -DEM86XX_REVISION=2 ..."

  --------------------------------------------------------------------------
  package writing          ES1  ES2  ES3  ES4     ES5     ES6  ES7  ES8  ES9 revA revB revC

  EM86XX_REVISION            1    2    3    4       5       6    7    8    9  'A'  'B'  'C'

  8600 `mambo' series                                                           1    2    3

  8620 `tangolight' series                                                    (a)  (b) 0x82
  8622 `tango15' series   0x81                (d)0x81                   0x82
                                                                           ^this is the PCI revID

  863x `tango2' series (c)0x81 0x81 0x81 0x82 (e)0x82 (f)0x83 0x84 0x85 0x86                

  (a) don't remember
  (b) no such chip
  (c) 8630: FibbedES1 ES1 ES2 ES3 are the same chip
  (d) 8622: ES1 and revA cannot be distinguished from revID. Software test impossible in practice
  (e) 8630: ES4 and ES5 cannot be distinguished from revID. Software test with 0x6c900 bit12
  (f) 8634: ES6 and RevA have the same revID (just different bonding option)
      8634: ES7 and RevB have the same revID (just different bonding option)
      8634: ES9 and RevC have the same revID (just different bonding option)
  --------------------------------------------------------------------------

  Usually, users do not set by hand, but thru `rmcflags' helper
*/
#ifndef EM86XX_REVISION
#error EM86XX_REVISION is not set
#endif

#if (EM86XX_CHIP==EM86XX_CHIPID_TANGO2) && (EM86XX_REVISION=='A') 
#error inconsistent: 863x revA is actually -DWITH_PROD=1 -DEM86XX_REVISION=6
#endif

#if (EM86XX_CHIP==EM86XX_CHIPID_TANGO2) && (EM86XX_REVISION=='B') 
#error inconsistent: 863x revB is actually -DWITH_PROD=1 -DEM86XX_REVISION=7
#endif

#if (EM86XX_CHIP==EM86XX_CHIPID_TANGO2) && (EM86XX_REVISION=='C') 
#error inconsistent: 863x revC is actually -DWITH_PROD=1 -DEM86XX_REVISION=9
#endif

#if (EM86XX_REVISION==1)
#define S_EM86XX_REVISION "ES1"
#elif (EM86XX_REVISION==2)
#define S_EM86XX_REVISION "ES2"
#elif (EM86XX_REVISION==3)
#define S_EM86XX_REVISION "ES3"
#elif (EM86XX_REVISION==4)
#define S_EM86XX_REVISION "ES4"
#elif (EM86XX_REVISION==5)
#define S_EM86XX_REVISION "ES5"
#elif (EM86XX_REVISION==6)
#if (EM86XX_CHIP==EM86XX_CHIPID_TANGO2) && (defined(WITH_PROD) || defined(WITH_FACSPROD))
#define S_EM86XX_REVISION "revA"
#else
#define S_EM86XX_REVISION "ES6"
#endif
#elif (EM86XX_REVISION==7)
#if (EM86XX_CHIP==EM86XX_CHIPID_TANGO2) && (defined(WITH_PROD) || defined(WITH_FACSPROD))
#define S_EM86XX_REVISION "revB"
#else
#define S_EM86XX_REVISION "ES7"
#endif
#elif (EM86XX_REVISION==8)
#define S_EM86XX_REVISION "ES8"
#elif (EM86XX_REVISION==9)
#if (EM86XX_CHIP==EM86XX_CHIPID_TANGO2) && (defined(WITH_PROD) || defined(WITH_FACSPROD))
#define S_EM86XX_REVISION "revC"
#else
#define S_EM86XX_REVISION "ES9"
#endif

#elif (EM86XX_REVISION=='A')
#define S_EM86XX_REVISION "revA"
#elif (EM86XX_REVISION=='B')
#define S_EM86XX_REVISION "revB"
#elif (EM86XX_REVISION=='C')
#define S_EM86XX_REVISION "revC"
#else
#error complete revision strings
#endif

/* the compilation modes */
#define EM86XX_MODEID_WITHHOST   1000
#define EM86XX_MODEID_STANDALONE 2000

/* the dsps */
#define EM86XX_ENGINEID_MPEG0 10
#define EM86XX_ENGINEID_MPEG1 11
#define EM86XX_ENGINEID_AUDIO0 20
#define EM86XX_ENGINEID_AUDIO1 21
#define EM86XX_ENGINEID_DEMUX 30

/* user does not have to set an engine id. This makes sense for mu only */
#ifdef EM86XX_ENGINE
#if (EM86XX_ENGINE==EM86XX_ENGINEID_MPEG0)
#define SENG "mpeg0"
#elif (EM86XX_ENGINE==EM86XX_ENGINEID_MPEG1)
#define SENG "mpeg1"
#elif (EM86XX_ENGINE==EM86XX_ENGINEID_AUDIO0)
#define SENG "audio0"
#elif (EM86XX_ENGINE==EM86XX_ENGINEID_AUDIO1)
#define SENG "audio1"
#elif (EM86XX_ENGINE==EM86XX_ENGINEID_DEMUX)
#define SENG "demux"
#else
#endif // end of engine dependent stuff
#endif

#if (EM86XX_CHIP==EM86XX_CHIPID_TANGO2)
/*
#if (defined(WITH_PROD) || defined(WITH_FACSPROD)) && !defined WITH_XLOADED_UCODE
#error inconsistent flag combination.
#endif

#if (defined(WITH_PROD) || defined(WITH_FACSPROD)) && !defined WITH_IRQHANDLER_BOOTLOADER
#error inconsistent flag combination.
#endif
*/
#ifdef WITH_UCODE_BOOTLOADER
#error inconsistent flag combination. You probably want WITH_XLOADED_UCODE
#endif

#endif

/* the microcode debug mode */

#define EM86XX_DEBUG_CHIP	1000
#define EM86XX_DEBUG_SIMU	2000

#endif // __RMEM86XXID_H__
