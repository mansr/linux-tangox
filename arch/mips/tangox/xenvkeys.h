
/*********************************************************************
 Copyright (C) 2001-2007
 Sigma Designs, Inc. 

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 2 as
 published by the Free Software Foundation.
 *********************************************************************/
/* 
 * The keys defined in XENV, more can be added.
 */

#ifndef __XENV_KEYS_H__
#define __XENV_KEYS_H__

#define XENV_KEY_BOARD_ID           "a.board_id"
#define XENV_KEY_CHIP_REV           "a.chip_rev"

#define XENV_KEY_PREMUX             "a.premux"
#define XENV_KEY_AVCLK_MUX          "a.avclk_mux"
#define XENV_KEY_HOSTCLK_MUX        "a.hostclk_mux"
#define XENV_KEY_IRQ_RISE_EDGE_LO   "a.irq_rise_edge_lo"
#define XENV_KEY_IRQ_FALL_EDGE_LO   "a.irq_fall_edge_lo"
#define XENV_KEY_GPIO_IRQ_MAP       "a.gpio_irq_map"

#define XENV_KEY_DEF_BAUDRATE       "a.baudrate"
#define XENV_KEY_CONSOLE_UART_PORT  "a.uart_console_port"
#define XENV_KEY_UART_USED_PORTS    "a.uart_used_ports"

#define XENV_KEY_PB_CS_CONFIG       "a.pb_cs_config"
#ifdef CONFIG_TANGO3
#define XENV_KEY_PB_CS_CONFIG1      "a.pb_cs_config1"
#define XENV_KEY_PB_CS_CTRL         "a.pb_cs_ctrl"
#endif
#define XENV_KEY_DEF_TIMING         "a.pb_def_timing"
#define XENV_KEY_PB_TIMING0         "a.pb_timing0"
#define XENV_KEY_PB_USE_TIMING0     "a.pb_use_timing0"
#define XENV_KEY_PB_TIMING1         "a.pb_timing1"
#define XENV_KEY_PB_USE_TIMING1     "a.pb_use_timing1"
#define XENV_KEY_PB_TIMING2         "a.pb_timing2"
#define XENV_KEY_PB_USE_TIMING2     "a.pb_use_timing2"
#define XENV_KEY_PB_TIMING3         "a.pb_timing3"
#define XENV_KEY_PB_USE_TIMING3     "a.pb_use_timing3"
#define XENV_KEY_PB_TIMING4         "a.pb_timing4"
#define XENV_KEY_PB_USE_TIMING4     "a.pb_use_timing4"
#define XENV_KEY_PB_TIMING5         "a.pb_timing5"
#define XENV_KEY_PB_USE_TIMING5     "a.pb_use_timing5"

#define XENV_KEY_IRQ_RISE_EDGE_HI   "a.irq_rise_edge_hi"
#define XENV_KEY_IRQ_FALL_EDGE_HI   "a.irq_fall_edge_hi"

#define XENV_KEY_ENABLED_DEVICES    "a.enable_devices"

#define XENV_KEY_ETH_MAC            "a.eth_mac"
#ifdef CONFIG_TANGO3
#define XENV_KEY_ETH1_MAC           "a.eth1_mac"
#endif

#define XENV_KEY_SCARD_OFF          "a.scard_off_pin"
#define XENV_KEY_SCARD_5V           "a.scard_5v_pin"
#define XENV_KEY_SCARD_CMD          "a.scard_cmd_pin"

#ifdef CONFIG_TANGO3
#define XENV_KEY_SCARD1_OFF         "a.scard1_off_pin"
#define XENV_KEY_SCARD1_5V          "a.scard1_5v_pin"
#define XENV_KEY_SCARD1_CMD         "a.scard1_cmd_pin"
#endif

#define XENV_KEY_ISAIDE_IRQ_ROUTE   "a.isaide_irq_route"
#define XENV_KEY_ISAIDE_TIMING_SLOT "a.isaide_timing_slot"

#define XENV_KEY_GPIO_DIR           "a.gpio_dir"
#define XENV_KEY_GPIO_DATA          "a.gpio_data"

#define XENV_KEY_LINUX_CMD          "a.linux_cmd"

#define XENV_KEY_Z_BOOT_DEF         "z.default_boot"

#define XENV_KEY_Z_PROD_TEST        "z.dt"
#define XENV_KEY_Z_PROD_LOOPS       "z.prod_loops"
#define XENV_KEY_Z_PROD_FTEST       "z.prod_ftest"
#define XENV_KEY_Z_PROD_FSTART      "z.prod_fstart"
#define XENV_KEY_Z_PROD_FEND        "z.prod_fend"
#define XENV_KEY_Z_PROD_FSTEP       "z.prod_fstep"
#define XENV_KEY_Z_PROD_RWIN        "z.prod_rwindow"
#define XENV_KEY_Z_PROD_WWIN        "z.prod_wwindow"
#define XENV_KEY_Z_PROD_CL          "z.prod_cl"
#define XENV_KEY_Z_PROD_NBLOCKS     "z.prod_nblocks"

#define XENV_KEY_YAMON_ENV          "y.env"
#define XENV_KEY_YAMON_IPADDR       "y.ipaddr"
#define XENV_KEY_YAMON_SUBNET       "y.subnetmask"
#define XENV_KEY_YAMON_GATEWAY      "y.gateway"
#define XENV_KEY_YAMON_START        "y.start"
#define XENV_KEY_YAMON_STARTDELAY   "y.startdelay"

#define XENV_KEYS_PCI_IRQ_ROUTE     "a.pcidev%d_irq_route"
#define XENV_KEYS_CD_FREQUENCY      "a.cd%d_freq"
#define XENV_KEYS_CD_DIV            "a.cd%d_div"
#define XENV_KEYS_UART_GPIO_MODE    "a.uart%d_gpio_mode"
#define XENV_KEYS_UART_GPIO_DIR     "a.uart%d_gpio_dir"
#define XENV_KEYS_UART_GPIO_DATA    "a.uart%d_gpio_data"
#define XENV_KEYS_UART_BAUDRATE     "a.uart%d_baudrate"
#define XENV_KEYS_GPIO_PULSE        "a.gpio%d_pulse"

#define XENV_KEYS_Z_BOOT_LOCATION   "z.boot%d"

#define XENV_KEYS_CS_SIZE           "l.cs%d_size"
#define XENV_KEYS_CS_PARTS          "l.cs%d_parts"

#define XENV_KEYS_CS_PART_SIZE      "l.cs%d_part%d_size"
#define XENV_KEYS_CS_PART_OFFSET    "l.cs%d_part%d_offset" 

#ifdef CONFIG_TANGO3
#define XENV_KEY_SATA_CHANNEL_CFG   "a.sata_channel_cfg"
#endif

#ifdef CONFIG_TANGO2
#define XENV_KEY_PS_PLL3            "a.ps.pll3"
#define XENV_KEY_PS_PLL2            "a.ps.pll2"
#define XENV_KEY_PS_PLL1            "a.ps.pll1"
#define XENV_KEY_PS_PLL0            "a.ps.pll0"
#define XENV_KEY_PS_MUX             "a.ps.mux"
#define XENV_KEY_PS_D0CFG           "a.ps.d0cfg"
#define XENV_KEY_PS_D1CFG           "a.ps.d1cfg"
#define XENV_KEY_PS_D0DELAY         "a.ps.d0delay"
#define XENV_KEY_PS_D1DELAY         "a.ps.d1delay"

#define XENV_KEY_STANDBY_PLL3       "a.standby.pll3"
#define XENV_KEY_STANDBY_PLL2       "a.standby.pll2"
#define XENV_KEY_STANDBY_PLL1       "a.standby.pll1"
#define XENV_KEY_STANDBY_PLL0       "a.standby.pll0"
#define XENV_KEY_STANDBY_MUX        "a.standby.mux"
#define XENV_KEY_STANDBY_D0CFG      "a.standby.d0cfg"
#define XENV_KEY_STANDBY_D1CFG      "a.standby.d1cfg"
#define XENV_KEY_STANDBY_D0DELAY    "a.standby.d0delay"
#define XENV_KEY_STANDBY_D1DELAY    "a.standby.d1delay"
#endif

#ifdef CONFIG_TANGO3
#define XENV_KEY_PS_PLL2            "a.ps.pll2"
#define XENV_KEY_PS_PLL0            "a.ps.pll0"
#define XENV_KEY_PS_MT3_HS          "a.ps.mt3_hs"

#define XENV_KEY_STANDBY_PLL2       "a.standby.pll2"
#define XENV_KEY_STANDBY_PLL0       "a.standby.pll0"
#define XENV_KEY_STANDBY_MT3_HS     "a.standby.mt3_hs"
#endif

#define XENV_KEY_STANDBY_GPIODIR    "a.standby.gpio_dir"
#define XENV_KEY_STANDBY_GPIODATA   "a.standby.gpio_data"

#ifndef CONFIG_SIGBLOCK_SUPPORT
#define ISAIDE_SHIFT		0
#define BMIDE_SHIFT		1
#define PCIHOST_SHIFT		2
#define ETHERNET_SHIFT		3
#define IR_SHIFT		4
#define FIP_SHIFT		5	
#define I2CM_SHIFT		6
#define I2CS_SHIFT		7
#define SDIO_SHIFT		8
#define USB_SHIFT		9
#define PCI1_SHIFT		10
#define PCI2_SHIFT		11
#define PCI3_SHIFT		12
#define PCI4_SHIFT		13
#define PCI5_SHIFT		14
#define PCI6_SHIFT		15
#define SATA_SHIFT		16
#define SCARD_SHIFT		17
#define GNET_SHIFT		18
#ifdef CONFIG_TANGO3
#define SCARD1_SHIFT		19
#define ETHERNET1_SHIFT		20
#endif
#endif

/* Only used internally for LR_XENV2_RW and LR_XENV2_RO. Will be prefixed with z. when read off pfla */
#ifdef CONFIG_TANGO3
/* lrrw */
#define XENV_LRRW_ETH_MACL	     "lrrw.maclo"
#define XENV_LRRW_ETH_MACH	     "lrrw.machi"
#define XENV_LRRW_ETH1_MACL	     "lrrw.mac1lo"
#define XENV_LRRW_ETH1_MACH	     "lrrw.mac1hi"
#define XENV_LRRW_KERNEL_END	     "lrrw.kend"

#define XENV_LRRW_RUAMM0_GA          "lrrw.ruamm0_ga"
#define XENV_LRRW_RUAMM1_GA          "lrrw.ruamm1_ga"

#define XENV_LRRW_0_UZDATA_OFFSET    "lrrw.0.uzdata_offset"
#define XENV_LRRW_0_ZDATA_OFFSET     "lrrw.0.zdata_offset"
#define XENV_LRRW_0_DSP_OFFSET       "lrrw.0.dsp_offset"
#define XENV_LRRW_0_XPU_OFFSET       "lrrw.0.xpu_offset"
#define XENV_LRRW_1_UZDATA_OFFSET    "lrrw.1.uzdata_offset"
#define XENV_LRRW_1_ZDATA_OFFSET     "lrrw.1.zdata_offset"
#define XENV_LRRW_1_DSP_OFFSET       "lrrw.1.dsp_offset"
#define XENV_LRRW_1_XPU_OFFSET       "lrrw.1.xpu_offset"

#define XENV_LRRW_CHANNEL_INDEX_GA   "lrrw.channel_index_ga"
#define XENV_LRRW_CHANNEL_INDEX_SIZE "lrrw.channel_index_size"
#define XENV_LRRW_IOS_GA             "lrrw.ios_ga"
#define XENV_LRRW_IOS_SIZE           "lrrw.ios_size"
#define XENV_LRRW_XOS_PUBLIC_GA      "lrrw.xos_public_ga"
#define XENV_LRRW_XOS_PUBLIC_SIZE    "lrrw.xos_public_size"
#define XENV_LRRW_IHAPI_GA           "lrrw.ihapi_ga"
#define XENV_LRRW_MM_VERSION         "lrrw.mm_version"

/* For handshaking and setup purpose */
#define XENV_HS_RUAMM0_OFFSET	     "a.ruamm0_offset"
#define XENV_HS_RUAMM1_OFFSET	     "a.ruamm1_offset"
#define XENV_HS_XOS_PUBLIC_SIZE      "a.xos_public_size"
#define XENV_HS_IOS_SIZE	     "a.ios_size"

/* lrro */
#define XENV_LRRO_LOCKED	     "lrro.locked"

#endif

#endif

