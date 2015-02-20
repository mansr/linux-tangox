/******************************************************/
/* This file is generated automatically, DO NOT EDIT! */
/******************************************************/
/*
 * ../emhwlib_hal/include/tango2/emhwlib_registers_tango2.h
 *
 * Copyright (C) 2001-2007 Sigma Designs, Inc. 
 * 
 *
 */
 
/**
  @file ../emhwlib_hal/include/tango2/emhwlib_registers_tango2.h
  @brief emhwlib generated file
   
  @author Jacques Mahe, Christian Wolff, Julien Soulier, Emmanuel Michon
  @ingroup hwlproperties
*/

#ifndef __EMHWLIB_REGISTERS_TANGO2_H__
#define __EMHWLIB_REGISTERS_TANGO2_H__

/* SystemBlock registers */
#define REG_BASE_system_block 0x00010000 /* width RMuint32 */
#define SYS_clkgen0_pll 0x0000 /* width RMuint32 */
#define SYS_clkgen0_div 0x0004 /* width RMuint32 */
#define SYS_clkgen1_pll 0x0008 /* width RMuint32 */
#define SYS_clkgen1_div 0x000C /* width RMuint32 */
#define SYS_clkgen2_pll 0x0010 /* width RMuint32 */
#define SYS_clkgen2_div 0x0014 /* width RMuint32 */
#define SYS_clkgen3_pll 0x0018 /* width RMuint32 */
#define SYS_clkgen3_div 0x001C /* width RMuint32 */
#define SYS_avclk_mux 0x0038 /* width RMuint32 */
#define SYS_sysclk_mux 0x003C /* width RMuint32 */
#define SYS_clk_cnt 0x0040 /* width RMuint32 */
#define SYS_xtal_in_cnt 0x0048 /* width RMuint32 */
#define DRAM_vbus_w0_cfg 0x0300 /* width RMuint32 */
#define DRAM_vbus_w1_cfg 0x0304 /* width RMuint32 */
#define DRAM_vbus_w2_cfg 0x0308 /* width RMuint32 */
#define DRAM_vbus_w3_cfg 0x030c /* width RMuint32 */
#define DRAM_vbus_r0_cfg 0x0340 /* width RMuint32 */
#define DRAM_vbus_r1_cfg 0x0344 /* width RMuint32 */
#define DRAM_vbus_r2_cfg 0x0348 /* width RMuint32 */
#define DRAM_vbus_r3_cfg 0x034c /* width RMuint32 */
#define DRAM_vbus_r4_cfg 0x0350 /* width RMuint32 */
#define DRAM_vbus_r5_cfg 0x0354 /* width RMuint32 */
#define DRAM_vbus_r6_cfg 0x0358 /* width RMuint32 */
#define DRAM_vbus_r7_cfg 0x035c /* width RMuint32 */
#define DRAM_vbus_r8_cfg 0x0360 /* width RMuint32 */
#define DRAM_vbus_r9_cfg 0x0364 /* width RMuint32 */
#define DRAM_vbus_r10_cfg 0x0368 /* width RMuint32 */
#define DRAM_vbus_r11_cfg 0x036c /* width RMuint32 */
#define DRAM_mbus_w0_cfg 0x0200 /* width RMuint32 */
#define DRAM_mbus_w1_cfg 0x0204 /* width RMuint32 */
#define DRAM_mbus_w2_cfg 0x0208 /* width RMuint32 */
#define DRAM_mbus_w3_cfg 0x020c /* width RMuint32 */
#define DRAM_mbus_w4_cfg 0x0210 /* width RMuint32 */
#define DRAM_mbus_w5_cfg 0x0214 /* width RMuint32 */
#define DRAM_mbus_w6_cfg 0x0218 /* width RMuint32 */
#define DRAM_mbus_w7_cfg 0x021c /* width RMuint32 */
#define DRAM_mbus_w8_cfg 0x0220 /* width RMuint32 */
#define DRAM_mbus_w9_cfg 0x0224 /* width RMuint32 */
#define DRAM_mbus_w10_cfg 0x0228 /* width RMuint32 */
#define DRAM_mbus_r0_cfg 0x0240 /* width RMuint32 */
#define DRAM_mbus_r1_cfg 0x0244 /* width RMuint32 */
#define DRAM_mbus_r2_cfg 0x0248 /* width RMuint32 */
#define DRAM_mbus_r3_cfg 0x024c /* width RMuint32 */
#define DRAM_mbus_r4_cfg 0x0250 /* width RMuint32 */
#define DRAM_mbus_r5_cfg 0x0254 /* width RMuint32 */
#define DRAM_mbus_r6_cfg 0x0258 /* width RMuint32 */
#define DRAM_mbus_r7_cfg 0x025c /* width RMuint32 */
#define DRAM_mbus_r8_cfg 0x0260 /* width RMuint32 */
#define DRAM_mbus_r9_cfg 0x0264 /* width RMuint32 */
#define DRAM_mbus_r10_cfg 0x0268 /* width RMuint32 */
#define SYS_hostclk_mux 0x0030 /* width RMuint32 */
#define SYS_sysclk_premux 0x0034 /* width RMuint32 */
#define SYS_rnd_cnt 0x0044 /* width RMuint32 */
#define SYS_cnt_cfg 0x004c /* width RMuint32 */
#define SYS_cfg_cnt0 0x0050 /* width RMuint32 */
#define SYS_cfg_cnt1 0x0054 /* width RMuint32 */
#define SYS_cfg_cnt2 0x0058 /* width RMuint32 */
#define SYS_cfg_cnt3 0x005c /* width RMuint32 */
#define SYS_cfg_cnt4 0x0060 /* width RMuint32 */
#define SYS_cleandiv0_div 0x0080 /* width RMuint32 */
#define SYS_cleandiv1_div 0x0088 /* width RMuint32 */
#define SYS_cleandiv2_div 0x0090 /* width RMuint32 */
#define SYS_cleandiv3_div 0x0098 /* width RMuint32 */
#define SYS_cleandiv4_div 0x00a0 /* width RMuint32 */
#define SYS_cleandiv5_div 0x00a8 /* width RMuint32 */
#define SYS_cleandiv6_div 0x00b0 /* width RMuint32 */
#define SYS_cleandiv7_div 0x00b8 /* width RMuint32 */
#define SYS_cleandiv8_div 0x00c0 /* width RMuint32 */
#define SYS_cleandiv9_div 0x00c8 /* width RMuint32 */
#define SYS_cleandiv10_div 0x00d0 /* width RMuint32 */
#define MARB_mid01_cfg 0x0200 /* width RMuint32 */
#define MARB_mid21_cfg 0x0204 /* width RMuint32 */
#define MARB_mid02_cfg 0x0208 /* width RMuint32 */
#define MARB_mid22_cfg 0x020c /* width RMuint32 */
#define MARB_mid04_cfg 0x0210 /* width RMuint32 */
#define MARB_mid24_cfg 0x0214 /* width RMuint32 */
#define MARB_mid25_cfg 0x0218 /* width RMuint32 */
#define MARB_mid08_cfg 0x021c /* width RMuint32 */
#define MARB_mid28_cfg 0x0220 /* width RMuint32 */
#define MARB_mid29_cfg 0x0224 /* width RMuint32 */
#define MARB_mid0C_cfg 0x0228 /* width RMuint32 */
#define MARB_mid2C_cfg 0x022c /* width RMuint32 */
#define MARB_mid10_cfg 0x0230 /* width RMuint32 */
#define MARB_mid30_cfg 0x0234 /* width RMuint32 */
#define MARB_mid31_cfg 0x0238 /* width RMuint32 */
#define MARB_mid12_cfg 0x023c /* width RMuint32 */
#define MARB_mid32_cfg 0x0240 /* width RMuint32 */
#define VARB_mid01_cfg 0x0300 /* width RMuint32 */
#define VARB_mid02_cfg 0x0304 /* width RMuint32 */
#define VARB_mid21_cfg 0x0308 /* width RMuint32 */
#define VARB_mid22_cfg 0x030c /* width RMuint32 */
#define VARB_mid23_cfg 0x0310 /* width RMuint32 */
#define VARB_mid24_cfg 0x0314 /* width RMuint32 */
#define VARB_mid25_cfg 0x0318 /* width RMuint32 */
#define VARB_mid26_cfg 0x031c /* width RMuint32 */
#define VARB_mid27_cfg 0x0320 /* width RMuint32 */
#define VARB_mid28_cfg 0x0324 /* width RMuint32 */
#define VARB_mid29_cfg 0x0328 /* width RMuint32 */
#define VARB_mid2A_cfg 0x032c /* width RMuint32 */
#define VARB_mid10_cfg 0x0330 /* width RMuint32 */
#define VARB_mid30_cfg 0x0334 /* width RMuint32 */
#define VARB_mid31_cfg 0x0338 /* width RMuint32 */
#define VARB_mid03_cfg 0x033c /* width RMuint32 */
#define IARB_mid01_cfg 0x0400 /* width RMuint32 */
#define IARB_mid02_cfg 0x0404 /* width RMuint32 */
#define SYS_gpio_dir 0x0500 /* width RMuint32 */
#define SYS_gpio_data 0x0504 /* width RMuint32 */
#define SYS_gpio_int 0x0508 /* width RMuint32 */
#define SYS_gpio15_pwm 0x0510 /* width RMuint32 */
#define SYS_gpio14_pwm 0x0514 /* width RMuint32 */
#define REG_BASE_dram_controller_0 0x00030000 /* width RMuint32 */
#define REG_BASE_dram_controller_1 0x00040000 /* width RMuint32 */
#define MEM_BASE_dram_controller_0_alias 0x10000000 /* width RMuint32 */
#define MEM_BASE_dram_controller_0 0x10000000 /* width RMuint32 */
#define MEM_BASE_dram_controller_1_alias 0x20000000 /* width RMuint32 */
#define MEM_BASE_dram_controller_1 0x20000000 /* width RMuint32 */
#define DRAM_dunit_cfg 0x0000 /* width RMuint32 */
#define DRAM_dunit_delay0_ctrl 0x0004 /* width RMuint32 */
#define DRAM_dunit_delay1_ctrl 0x0008 /* width RMuint32 */
#define DRAM_dunit_auto_delay 0x000c /* width RMuint32 */
#define DRAM_dunit_fall_delay0 0x0010 /* width RMuint32 */
#define DRAM_dunit_fall_delay1 0x0014 /* width RMuint32 */
#define DRAM_dunit_bw_lobound 0x0018 /* width RMuint32 */
#define DRAM_dunit_bw_hibound 0x001c /* width RMuint32 */
#define DRAM_dunit_bw_probe_cfg 0x0020 /* width RMuint32 */
#define DRAM_dunit_bw_probe_cnt 0x0024 /* width RMuint32 */
#define DRAM_dunit_bw_cntall 0x0028 /* width RMuint32 */
#define DRAM_dunit_calibration_delay 0x0030 /* width RMuint32 */
#define DRAM_dunit_calibration_rise_err 0x0034 /* width RMuint32 */
#define DRAM_dunit_calibration_fall_err 0x0038 /* width RMuint32 */
#define DRAM_dunit_calibration_page 0x0088 /* width RMuint32 */
#define DRAM_dunit_flush_buffer 0x0104 /* width RMuint32 */
#define REG_BASE_host_interface 0x00020000 /* width RMuint32 */
#define MEM_BASE_host_interface 0x40000000 /* width RMuint32 */
#define IDE_data 0x0000 /* width RMuint32 */
#define IDE_error 0x0004 /* width RMuint32 */
#define IDE_count 0x0008 /* width RMuint32 */
#define IDE_start_sector 0x000c /* width RMuint32 */
#define IDE_cylinder_lo 0x0010 /* width RMuint32 */
#define IDE_cylinder_hi 0x0014 /* width RMuint32 */
#define IDE_head_device 0x0018 /* width RMuint32 */
#define IDE_cmd_stat 0x001c /* width RMuint32 */
#define IDE_irq_stat 0x0218 /* width RMuint32 */
#define IDE_cmd_stat__ 0x021c /* width RMuint32 */
#define PB_timing0 0x0800 /* width RMuint32 */
#define PB_timing1 0x0804 /* width RMuint32 */
#define PB_timing2 0x0808 /* width RMuint32 */
#define PB_timing3 0x080c /* width RMuint32 */
#define PB_timing4 0x0810 /* width RMuint32 */
#define PB_timing5 0x0814 /* width RMuint32 */
#define PB_default_timing 0x0818 /* width RMuint32 */
#define PB_use_timing0 0x081c /* width RMuint32 */
#define PB_use_timing1 0x0820 /* width RMuint32 */
#define PB_use_timing2 0x0824 /* width RMuint32 */
#define PB_use_timing3 0x0828 /* width RMuint32 */
#define PB_use_timing4 0x082c /* width RMuint32 */
#define PB_use_timing5 0x0830 /* width RMuint32 */
#define PB_CS_config 0x0834 /* width RMuint32 */
#define PB_automode_start_address 0x0840 /* width RMuint32 */
#define PB_automode_control 0x0844 /* width RMuint32 */
#define EMHWLIB_IS_HOST 0xe000 /* width RMuint32 */
#define HOST_REG1 0xfed0 /* width RMuint32 */
#define HOST_REG2 0xfed4 /* width RMuint32 */
#define READ_ADDRESS 0xfec0 /* width RMuint32 */
#define READ_COUNTER 0xfec4 /* width RMuint32 */
#define READ_ENABLE 0xfec8 /* width RMuint32 */
#define REV_ORDER 0xfecc /* width RMuint32 */
#define WRITE_ADDRESS 0xfed8 /* width RMuint32 */
#define WRITE_COUNTER 0xfedc /* width RMuint32 */
#define WRITE_ENABLE 0xfee0 /* width RMuint32 */
#define BURST 0xfee4 /* width RMuint32 */
#define PCI_TIMEOUT 0x8000 /* width RMuint32 */
#define PCI_TIMEOUT_STATUS 0x8004 /* width RMuint32 */
#define PCI_TIMER 0x8008 /* width RMuint32 */
#define PCI_TIMER_TEST 0x800c /* width RMuint32 */
#define PCI_WAKEUP 0x8010 /* width RMuint32 */
#define PCI_REGION_0_BASE 0x9000 /* width RMuint32 */
#define PCI_REGION_1_BASE 0x9004 /* width RMuint32 */
#define PCI_REGION_2_BASE 0x9008 /* width RMuint32 */
#define PCI_REGION_3_BASE 0x900c /* width RMuint32 */
#define PCI_REGION_4_BASE 0x9010 /* width RMuint32 */
#define PCI_REGION_5_BASE 0x9014 /* width RMuint32 */
#define PCI_REGION_6_BASE 0x9018 /* width RMuint32 */
#define PCI_REGION_7_BASE 0x901c /* width RMuint32 */
#define PCI_irq_status 0x9020 /* width RMuint32 */
#define PCI_irq_set 0x9024 /* width RMuint32 */
#define PCI_irq_clear 0x9028 /* width RMuint32 */
#define SBOX_FIFO_RESET 0x90a0 /* width RMuint32 */
#define SBOX_ROUTE 0x90a8 /* width RMuint32 */
#define output_SBOX_MBUS_W0 0x9080 /* width RMuint32 */
#define output_SBOX_MBUS_W1 0x9084 /* width RMuint32 */
#define output_SBOX_PCI_MASTER 0x9088 /* width RMuint32 */
#define output_SBOX_PCI_SLAVE 0x908c /* width RMuint32 */
#define output_SBOX_CIPHER 0x9090 /* width RMuint32 */
#define output_SBOX_IDE_ISA 0x9094 /* width RMuint32 */
#define output_SBOX_IDE_DVD 0x9098 /* width RMuint32 */
#define input_keep_SBOX 0 /* width RMuint32 */
#define input_MBUS_R0_SBOX 1 /* width RMuint32 */
#define input_MBUS_R1_SBOX 2 /* width RMuint32 */
#define input_PCI_MASTER_SBOX 3 /* width RMuint32 */
#define input_PCI_SLAVE_SBOX 4 /* width RMuint32 */
#define input_CIPHER_SBOX 5 /* width RMuint32 */
#define input_IDE_DVD_SBOX 6 /* width RMuint32 */
#define input_IDE_ISA_SBOX 7 /* width RMuint32 */
#define input_SFLA_SBOX 8 /* width RMuint32 */
#define input_unconnected_SBOX 0xf /* width RMuint32 */
#define host_mutex0 0x9040 /* width RMuint32 */
#define host_mutex1 0x9044 /* width RMuint32 */
#define host_mutex2 0x9048 /* width RMuint32 */
#define host_mutex3 0x904c /* width RMuint32 */
#define host_mutex4 0x9050 /* width RMuint32 */
#define host_mutex5 0x9054 /* width RMuint32 */
#define host_mutex6 0x9058 /* width RMuint32 */
#define host_mutex7 0x905c /* width RMuint32 */
#define host_mutex8 0x9060 /* width RMuint32 */
#define host_mutex9 0x9064 /* width RMuint32 */
#define host_mutex10 0x9068 /* width RMuint32 */
#define host_mutex11 0x906c /* width RMuint32 */
#define host_mutex12 0x9070 /* width RMuint32 */
#define host_mutex13 0x9074 /* width RMuint32 */
#define host_mutex14 0x9078 /* width RMuint32 */
#define host_mutex15 0x907c /* width RMuint32 */
#define PCI_host_reg5 0xfe94 /* width RMuint32 */
#define PCI_chip_is_host 0xfe90 /* width RMuint32 */
#define IDECTRL_idesrc 0x20d0 /* width RMuint32 */
#define IDECTRL_pri_drv1udmatim1 0x20e0 /* width RMuint32 */
#define IDECTRL_pri_drv1udmatim2 0x20f0 /* width RMuint32 */
#define IDECTRL_pri_idectl 0x2100 /* width RMuint32 */
#define IDECTRL_pri_drv0tim 0x2110 /* width RMuint32 */
#define IDECTRL_pri_drv1tim 0x2120 /* width RMuint32 */
#define IDECTRL_idemisc 0x2130 /* width RMuint32 */
#define IDECTRL_idestatus 0x2140 /* width RMuint32 */
#define IDECTRL_udmactl 0x2150 /* width RMuint32 */
#define IDECTRL_pri_drv0udmatim1 0x2160 /* width RMuint32 */
#define IDECTRL_pri_drv0udmatim2 0x2170 /* width RMuint32 */
#define IDECTRL_pref_st 0x2310 /* width RMuint32 */
#define IDECTRL_pri_ctrlblock 0x2398 /* width RMuint32 */
#define IDECTRL_pri_cmdblock 0x23c0 /* width RMuint32 */
#define IDECTRL_bmic 0x2400 /* width RMuint32 */
#define IDECTRL_bmis 0x2410 /* width RMuint32 */
#define IDECTRL_bmidtp 0x2420 /* width RMuint32 */
#define IDECTRL_ide_dmaptr 0x2780 /* width RMuint32 */
#define IDECTRL_ide_dmalen 0x2790 /* width RMuint32 */
#define IDECTRL_pio_prefetch_data 0x27c0 /* width RMuint32 */
#define MEM_BASE_pfla 0x40000000 /* width RMuint32 */
#define PB_CS0_OFFSET 0x00000000 /* width RMuint32 */
#define PB_CS1_OFFSET 0x04000000 /* width RMuint32 */
#define PB_CS2_OFFSET 0x08000000 /* width RMuint32 */
#define PB_CS3_OFFSET 0x0c000000 /* width RMuint32 */
#define ETH_gpio_dir1 0x7100 /* width RMuint32 */
#define ETH_gpio_data1 0x7104 /* width RMuint32 */
#define ETH_gpio_mask1 0x7108 /* width RMuint32 */
#define ETH_gpio_dir2 0x710c /* width RMuint32 */
#define ETH_gpio_data2 0x7110 /* width RMuint32 */
#define PCI_host_reg1 0xfed0 /* width RMuint32 */
#define PCI_host_reg2 0xfed4 /* width RMuint32 */
#define PCI_host_reg3 0xfe80 /* width RMuint32 */
#define PCI_host_reg4 0xfe84 /* width RMuint32 */
#define PCI_pcictrl_reg1 0xfe88 /* width RMuint32 */
#define PCI_pcictrl_reg2 0xfe8c /* width RMuint32 */
#define PCI_pcictrl_reg3 0xfefc /* width RMuint32 */
#define PCI_REG0 0xfee8 /* width RMuint32 */
#define PCI_REG1 0xfeec /* width RMuint32 */
#define PCI_REG2 0xfef0 /* width RMuint32 */
#define PCI_REG3 0xfef4 /* width RMuint32 */
#define PCI_CONFIG 0xfef8 /* width RMuint32 */
#define MIF_W0_ADD 0xb000 /* width RMuint32 */
#define MIF_W0_CNT 0xb004 /* width RMuint32 */
#define MIF_W0_SKIP 0xb008 /* width RMuint32 */
#define MIF_W0_CMD 0xb00c /* width RMuint32 */
#define MIF_W1_ADD 0xb040 /* width RMuint32 */
#define MIF_W1_CNT 0xb044 /* width RMuint32 */
#define MIF_W1_SKIP 0xb048 /* width RMuint32 */
#define MIF_W1_CMD 0xb04c /* width RMuint32 */
#define MIF_R0_ADD 0xb080 /* width RMuint32 */
#define MIF_R0_CNT 0xb084 /* width RMuint32 */
#define MIF_R0_SKIP 0xb088 /* width RMuint32 */
#define MIF_R0_CMD 0xb08c /* width RMuint32 */
#define MIF_R1_ADD 0xb0c0 /* width RMuint32 */
#define MIF_R1_CNT 0xb0c4 /* width RMuint32 */
#define MIF_R1_SKIP 0xb0c8 /* width RMuint32 */
#define MIF_R1_CMD 0xb0cc /* width RMuint32 */
#define MBUS_IDLE 0 /* width RMuint32 */
#define MBUS_LINEAR 1 /* width RMuint32 */
#define MBUS_DOUBLE 2 /* width RMuint32 */
#define MBUS_RECTANGLE 3 /* width RMuint32 */
#define MBUS_VOID 4 /* width RMuint32 */
#define MBUS_LINEAR_VOID 5 /* width RMuint32 */
#define MBUS_DOUBLE_VOID 6 /* width RMuint32 */
#define MBUS_RECTANGLE_VOID 7 /* width RMuint32 */
#define MBUS_TILED 8 /* width RMuint32 */
#define GBUS_MUTEX_XPU 0x14 /* width RMuint32 */
#define GBUS_MUTEX_PT110 0x16 /* width RMuint32 */
#define GBUS_MUTEX_TDMX 0x19 /* width RMuint32 */
#define GBUS_MUTEX_AUDIO_0 0x1b /* width RMuint32 */
#define GBUS_MUTEX_AUDIO_1 0x1c /* width RMuint32 */
#define GBUS_MUTEX_MPEG_0 0x1d /* width RMuint32 */
#define GBUS_MUTEX_MPEG_1 0x1e /* width RMuint32 */
#define GBUS_MUTEX_HOST 0x1f /* width RMuint32 */
#define GBUS_MUTEX_LOCAL 0x10 /* width RMuint32 */
/* SystemBlock registers done */

/* CPUBlock registers */
#define REG_BASE_cpu_block 0x00060000 /* width RMuint32 */
#define CPU_time0_load 0xc500 /* width RMuint32 */
#define CPU_time0_value 0xc504 /* width RMuint32 */
#define CPU_time0_ctrl 0xc508 /* width RMuint32 */
#define CPU_time0_clr 0xc50c /* width RMuint32 */
#define CPU_time1_load 0xc600 /* width RMuint32 */
#define CPU_time1_value 0xc604 /* width RMuint32 */
#define CPU_time1_ctrl 0xc608 /* width RMuint32 */
#define CPU_time1_clr 0xc60c /* width RMuint32 */
#define CPU_rtc_data 0xc800 /* width RMuint32 */
#define CPU_rtc_match 0xc804 /* width RMuint32 */
#define CPU_rtc_stat 0xc808 /* width RMuint32 */
#define CPU_rtc_load 0xc80c /* width RMuint32 */
#define CPU_rtc_ctrl 0xc810 /* width RMuint32 */
#define CPU_irq_status 0xe000 /* width RMuint32 */
#define CPU_irq_rawstat 0xe004 /* width RMuint32 */
#define CPU_irq_enableset 0xe008 /* width RMuint32 */
#define CPU_irq_enableclr 0xe00c /* width RMuint32 */
#define CPU_irq_softset 0xe010 /* width RMuint32 */
#define CPU_irq_softclr 0xe014 /* width RMuint32 */
#define CPU_fiq_status 0xe100 /* width RMuint32 */
#define CPU_fiq_rawstat 0xe104 /* width RMuint32 */
#define CPU_fiq_enableset 0xe108 /* width RMuint32 */
#define CPU_fiq_enableclr 0xe10c /* width RMuint32 */
#define CPU_fiq_softset 0xe110 /* width RMuint32 */
#define CPU_fiq_softclr 0xe114 /* width RMuint32 */
#define CPU_edge_status 0xe200 /* width RMuint32 */
#define CPU_edge_rawstat 0xe204 /* width RMuint32 */
#define CPU_edge_config_rise 0xe208 /* width RMuint32 */
#define CPU_edge_config_fall 0xe20c /* width RMuint32 */
#define CPU_SOFT_INT 0x00000001 /* width RMuint32 */
#define CPU_UART0_INT 0x00000002 /* width RMuint32 */
#define CPU_UART1_INT 0x00000004 /* width RMuint32 */
#define CPU_TIMER0_INT 0x00000020 /* width RMuint32 */
#define CPU_TIMER1_INT 0x00000040 /* width RMuint32 */
#define CPU_HOST_MBUS_W0_INT 0x00000200 /* width RMuint32 */
#define CPU_HOST_MBUS_W1_INT 0x00000400 /* width RMuint32 */
#define CPU_HOST_MBUS_R0_INT 0x00000800 /* width RMuint32 */
#define CPU_HOST_MBUS_R1_INT 0x00001000 /* width RMuint32 */
#define CPU_PCI_INTA 0x00002000 /* width RMuint32 */
#define CPU_PCI_INTB 0x00004000 /* width RMuint32 */
#define CPU_PCI_INTC 0x00008000 /* width RMuint32 */
#define CPU_PCI_INTD 0x00010000 /* width RMuint32 */
#define CPU_PCI_FAULT_INT 0x00100000 /* width RMuint32 */
#define CPU_INFRARED_INT 0x00200000 /* width RMuint32 */
#define CPU_SFLA_INT 0x00000010 /* width RMuint32 */
#define CPU_DVD_INT 0x00000080 /* width RMuint32 */
#define CPU_ETH_INT 0x00000100 /* width RMuint32 */
#define CPU_DMAIDE_INT 0x00020000 /* width RMuint32 */
#define CPU_IDE_INT 0x00040000 /* width RMuint32 */
#define CPU_FRONTPANEL_INT 0x00080000 /* width RMuint32 */
#define CPU_I2C_INT 0x00400000 /* width RMuint32 */
#define CPU_GFX_ACCEL_INT 0x00800000 /* width RMuint32 */
#define CPU_VSYNC0_INT 0x01000000 /* width RMuint32 */
#define CPU_VSYNC1_INT 0x02000000 /* width RMuint32 */
#define CPU_VSYNC2_INT 0x04000000 /* width RMuint32 */
#define CPU_VSYNC3_INT 0x08000000 /* width RMuint32 */
#define CPU_VSYNC4_INT 0x10000000 /* width RMuint32 */
#define CPU_VSYNC4BKEND_INT 0x20000000 /* width RMuint32 */
#define CPU_VSYNC5_INT 0x40000000 /* width RMuint32 */
#define CPU_VSYNC5BKEND_INT 0x80000000 /* width RMuint32 */
#define CPU_SMARTCARD_HI_INT 0x00000001 /* width RMuint32 */
#define CPU_HDMI_HI_INT 0x00000002 /* width RMuint32 */
#define CPU_HDMI_I2C_HI_INT 0x00000004 /* width RMuint32 */
#define CPU_VBUS_W0_HI_INT 0x00000008 /* width RMuint32 */
#define CPU_VBUS_W3_HI_INT 0x00000010 /* width RMuint32 */
#define CPU_ETH_PHY_HI_INT 0x00000020 /* width RMuint32 */
#define CPU_ETH_MAC_HI_INT 0x00000040 /* width RMuint32 */
#define CPU_USB_OHCI_MAC_HI_INT 0x00000080 /* width RMuint32 */
#define CPU_USB_EHCI_MAC_HI_INT 0x00000100 /* width RMuint32 */
#define LOG2_CPU_SOFT_INT 0 /* width RMuint32 */
#define LOG2_CPU_UART0_INT 1 /* width RMuint32 */
#define LOG2_CPU_UART1_INT 2 /* width RMuint32 */
#define LOG2_CPU_TIMER0_INT 5 /* width RMuint32 */
#define LOG2_CPU_TIMER1_INT 6 /* width RMuint32 */
#define LOG2_CPU_DVD_INT 7 /* width RMuint32 */
#define LOG2_CPU_RTC_INT 8 /* width RMuint32 */
#define LOG2_CPU_HOST_MBUS_W0_INT 9 /* width RMuint32 */
#define LOG2_CPU_HOST_MBUS_W1_INT 10 /* width RMuint32 */
#define LOG2_CPU_HOST_MBUS_R0_INT 11 /* width RMuint32 */
#define LOG2_CPU_HOST_MBUS_R1_INT 12 /* width RMuint32 */
#define LOG2_CPU_PCI_INTA 13 /* width RMuint32 */
#define LOG2_CPU_PCI_INTB 14 /* width RMuint32 */
#define LOG2_CPU_PCI_INTC 15 /* width RMuint32 */
#define LOG2_CPU_PCI_INTD 16 /* width RMuint32 */
#define LOG2_CPU_DMAIDE_INT 17 /* width RMuint32 */
#define LOG2_CPU_IDE_INT 18 /* width RMuint32 */
#define LOG2_CPU_FRONTPANEL_INT 19 /* width RMuint32 */
#define LOG2_CPU_PCI_FAULT_INT 20 /* width RMuint32 */
#define LOG2_CPU_INFRARED_INT 21 /* width RMuint32 */
#define LOG2_CPU_I2C_INT 22 /* width RMuint32 */
#define LOG2_CPU_GFX_ACCEL_INT 23 /* width RMuint32 */
#define LOG2_CPU_VSYNC0_INT 24 /* width RMuint32 */
#define LOG2_CPU_VSYNC1_INT 25 /* width RMuint32 */
#define LOG2_CPU_VSYNC2_INT 26 /* width RMuint32 */
#define LOG2_CPU_VSYNC3_INT 27 /* width RMuint32 */
#define LOG2_CPU_VSYNC4_INT 28 /* width RMuint32 */
#define LOG2_CPU_VSYNC4BKEND_INT 29 /* width RMuint32 */
#define LOG2_CPU_VSYNC5_INT 30 /* width RMuint32 */
#define LOG2_CPU_VSYNC5BKEND_INT 31 /* width RMuint32 */
#define LOG2_CPU_SMARTCARD_INT 32 /* width RMuint32 */
#define LOG2_CPU_HDMI_INT 33 /* width RMuint32 */
#define LOG2_CPU_HDMI_I2C_INT 34 /* width RMuint32 */
#define LOG2_CPU_VBUS_W0_INT 35 /* width RMuint32 */
#define LOG2_CPU_VBUS_W3_INT 36 /* width RMuint32 */
#define LOG2_CPU_ETH_PHY_INT 37 /* width RMuint32 */
#define LOG2_CPU_ETH_MAC_INT 38 /* width RMuint32 */
#define LOG2_CPU_USB_OHCI_INT 39 /* width RMuint32 */
#define LOG2_CPU_USB_EHCI_INT 40 /* width RMuint32 */
#define CPU_edge_status_hi 0xe220 /* width RMuint32 */
#define CPU_edge_rawstat_hi 0xe224 /* width RMuint32 */
#define CPU_edge_config_rise_hi 0xe228 /* width RMuint32 */
#define CPU_edge_config_fall_hi 0xe22c /* width RMuint32 */
#define CPU_irq_status_hi 0xe018 /* width RMuint32 */
#define CPU_irq_rawstat_hi 0xe01c /* width RMuint32 */
#define CPU_irq_enableset_hi 0xe020 /* width RMuint32 */
#define CPU_irq_enableclr_hi 0xe024 /* width RMuint32 */
#define CPU_fiq_status_hi 0xe118 /* width RMuint32 */
#define CPU_fiq_rawstat_hi 0xe11c /* width RMuint32 */
#define CPU_fiq_enableset_hi 0xe120 /* width RMuint32 */
#define CPU_fiq_enableclr_hi 0xe124 /* width RMuint32 */
#define CPU_iiq_status 0xe300 /* width RMuint32 */
#define CPU_iiq_rawstat 0xe304 /* width RMuint32 */
#define CPU_iiq_enableset 0xe308 /* width RMuint32 */
#define CPU_iiq_enableclr 0xe30c /* width RMuint32 */
#define CPU_iiq_softset 0xe310 /* width RMuint32 */
#define CPU_iiq_softclr 0xe314 /* width RMuint32 */
#define CPU_iiq_status_hi 0xe318 /* width RMuint32 */
#define CPU_iiq_rawstat_hi 0xe31c /* width RMuint32 */
#define CPU_iiq_enableset_hi 0xe320 /* width RMuint32 */
#define CPU_iiq_enableclr_hi 0xe324 /* width RMuint32 */
#define CPU_UART_GPIOMODE 0x38 /* width RMuint32 */
#define CPU_UART_GPIODIR 0x30 /* width RMuint32 */
#define CPU_UART_GPIODATA 0x34 /* width RMuint32 */
#define CPU_edge_config_rise_set 0xe210 /* width RMuint32 */
#define CPU_edge_config_rise_clr 0xe214 /* width RMuint32 */
#define CPU_edge_config_fall_set 0xe218 /* width RMuint32 */
#define CPU_edge_config_fall_clr 0xe21c /* width RMuint32 */
#define CPU_edge_config_rise_set_hi 0xe230 /* width RMuint32 */
#define CPU_edge_config_rise_clr_hi 0xe234 /* width RMuint32 */
#define CPU_edge_config_fall_set_hi 0xe238 /* width RMuint32 */
#define CPU_edge_config_fall_clr_hi 0xe23c /* width RMuint32 */
#define CPU_pm_select_0 0xc900 /* width RMuint32 */
#define CPU_pm_counter_0 0xc904 /* width RMuint32 */
#define CPU_pm_select_1 0xc908 /* width RMuint32 */
#define CPU_pm_counter_1 0xc90c /* width RMuint32 */
#define CPU_remap 0xf000 /* width RMuint32 */
#define CPU_remap1 0xf004 /* width RMuint32 */
#define CPU_remap2 0xf008 /* width RMuint32 */
#define CPU_remap3 0xf00c /* width RMuint32 */
#define CPU_remap4 0xf010 /* width RMuint32 */
#define CPU_remap_address 0x1fc00000 /* width RMuint32 */
#define CPU_remap1_address 0 /* width RMuint32 */
#define CPU_remap2_address 0x04000000 /* width RMuint32 */
#define CPU_remap3_address 0x08000000 /* width RMuint32 */
#define CPU_remap4_address 0x0c000000 /* width RMuint32 */
#define REG_BASE_irq_handler_block 0xe0000 /* width RMuint32 */
#define G2L_BIST_BUSY 0xffe0 /* width RMuint32 */
#define G2L_BIST_PASS 0xffe4 /* width RMuint32 */
#define G2L_BIST_MASK 0xffe8 /* width RMuint32 */
#define G2L_RESET_CONTROL 0xfffc /* width RMuint32 */
#define CPU_UART0_base 0xc100 /* width RMuint32 */
#define CPU_UART1_base 0xc200 /* width RMuint32 */
#define CPU_UART_RBR 0x00 /* width RMuint32 */
#define CPU_UART_THR 0x04 /* width RMuint32 */
#define CPU_UART_IER 0x08 /* width RMuint32 */
#define CPU_UART_IIR 0x0c /* width RMuint32 */
#define CPU_UART_FCR 0x10 /* width RMuint32 */
#define CPU_UART_LCR 0x14 /* width RMuint32 */
#define CPU_UART_MCR 0x18 /* width RMuint32 */
#define CPU_UART_LSR 0x1c /* width RMuint32 */
#define CPU_UART_MSR 0x20 /* width RMuint32 */
#define CPU_UART_SCR 0x24 /* width RMuint32 */
#define CPU_UART_CLKDIV 0x28 /* width RMuint32 */
#define CPU_UART_CLKSEL 0x2c /* width RMuint32 */
/* CPUBlock registers done */

/* XPUBlock registers */
#define REG_BASE_xpu_block 0x000e0000 /* width RMuint32 */
/* XPUBlock registers done */

/* IPUBlock registers */
#define REG_BASE_ipu_block 0x000f0000 /* width RMuint32 */
/* IPUBlock registers done */

/* DisplayBlock registers */
#define REG_BASE_display_block 0x00070000 /* width RMuint32 */
#define PMEM_BASE_display_block 0x00300000 /* width RMuint32 */
#define VBUS_IDLE 0x0 /* width RMuint32 */
#define VBUS_LINEAR 0x1 /* width RMuint32 */
#define VBUS_DOUBLE 0x2 /* width RMuint32 */
#define VBUS_RECTANGLE 0x3 /* width RMuint32 */
#define VBUS_DOUBLE_FIELD 0x4 /* width RMuint32 */
#define VBUS_DOUBLE_RECTANGLE 0x5 /* width RMuint32 */
#define VBUS_8BYTE_COLUMN 0x6 /* width RMuint32 */
#define VBUS_VOID 0x8 /* width RMuint32 */
#define VBUS_LINEAR_VOID 0x9 /* width RMuint32 */
#define VBUS_DOUBLE_VOID 0xa /* width RMuint32 */
#define VBUS_RECTANGLE_VOID 0xb /* width RMuint32 */
#define VBUS_DOUBLE_FIELD_VOID 0xc /* width RMuint32 */
#define VBUS_DOUBLE_RECTANGLE_VOID 0xd /* width RMuint32 */
#define VBUS_8BYTE_COLUMN_VOID 0xe /* width RMuint32 */
/* DisplayBlock registers done */

/* DemuxEngine registers */
#define REG_BASE_demux_engine 0x000A0000 /* width RMuint32 */
#define MEM_BASE_demux_engine 0x00140000 /* width RMuint32 */
#define PMEM_BASE_demux_engine 0x00140000 /* width RMuint32 */
#define DMEM_BASE_demux_engine 0x00150000 /* width RMuint32 */
#define REG_BASE_demux_engine_0 0x000A0000 /* width RMuint32 */
#define MEM_BASE_demux_engine_0 0x00140000 /* width RMuint32 */
#define PMEM_BASE_demux_engine_0 0x00140000 /* width RMuint32 */
#define DMEM_BASE_demux_engine_0 0x00150000 /* width RMuint32 */
#define REG_BASE_demux_engine_1 0x000b0000 /* width RMuint32 */
#define MEM_BASE_demux_engine_1 0x00160000 /* width RMuint32 */
#define PMEM_BASE_demux_engine_1 0x00160000 /* width RMuint32 */
#define DMEM_BASE_demux_engine_1 0x00170000 /* width RMuint32 */
#define TDMX_gpio_data 0x2e0c /* width RMuint32 */
#define TDMX_gpio_dir 0x2e0d /* width RMuint32 */
/* DemuxEngine registers done */

/* MpegEngine registers */
#define REG_BASE_mpeg_engine_0 0x00080000 /* width RMuint32 */
#define MEM_BASE_mpeg_engine_0 0x00100000 /* width RMuint32 */
#define PMEM_BASE_mpeg_engine_0 0x00100000 /* width RMuint32 */
#define DMEM_BASE_mpeg_engine_0 0x00110000 /* width RMuint32 */
#define REG_BASE_mpeg_engine_1 0x00090000 /* width RMuint32 */
#define MEM_BASE_mpeg_engine_1 0x00120000 /* width RMuint32 */
#define PMEM_BASE_mpeg_engine_1 0x00120000 /* width RMuint32 */
#define DMEM_BASE_mpeg_engine_1 0x00130000 /* width RMuint32 */
#define RBUS_offset 0x4000 /* width RMuint32 */
/* MpegEngine registers done */

/* AudioEngine registers */
#define REG_BASE_audio_engine_0 0x000c0000 /* width RMuint32 */
#define MEM_BASE_audio_engine_0 0x00180000 /* width RMuint32 */
#define PMEM_BASE_audio_engine_0 0x00180000 /* width RMuint32 */
#define DMEM_BASE_audio_engine_0 0x00190000 /* width RMuint32 */
#define REG_BASE_audio_engine_1 0x000d0000 /* width RMuint32 */
#define MEM_BASE_audio_engine_1 0x001a0000 /* width RMuint32 */
#define PMEM_BASE_audio_engine_1 0x001a0000 /* width RMuint32 */
#define DMEM_BASE_audio_engine_1 0x001b0000 /* width RMuint32 */
#define audio_mutex0 0x3e90 /* width RMuint32 */
#define audio_mutex1 0x3e91 /* width RMuint32 */
#define audio_mutex2 0x3e92 /* width RMuint32 */
#define audio_mutex3 0x3e93 /* width RMuint32 */
#define audio_mutex4 0x3e94 /* width RMuint32 */
#define audio_mutex5 0x3e95 /* width RMuint32 */
#define audio_mutex6 0x3e96 /* width RMuint32 */
#define audio_mutex7 0x3e97 /* width RMuint32 */
/* AudioEngine registers done */

/* AudioDecoder registers */
/* AudioDecoder registers done */

/* AudioCapture registers */
/* AudioCapture registers done */

/* VoipCodec registers */
/* VoipCodec registers done */

/* CRCDecoder registers */
/* CRCDecoder registers done */

/* XCRCDecoder registers */
/* XCRCDecoder registers done */

/* StreamCapture registers */
/* StreamCapture registers done */

/* RawDataTransfer registers */
/* RawDataTransfer registers done */

/* I2C registers */
#define I2C_MASTER_CONFIG 0x80 /* width RMuint32 */
#define I2C_MASTER_CLK_DIV 0x84 /* width RMuint32 */
#define I2C_MASTER_DEV_ADDR 0x88 /* width RMuint32 */
#define I2C_MASTER_ADDR 0x8c /* width RMuint32 */
#define I2C_MASTER_DATA_OUT 0x90 /* width RMuint32 */
#define I2C_MASTER_DATA_IN 0x94 /* width RMuint32 */
#define I2C_MASTER_STATUS 0x98 /* width RMuint32 */
#define I2C_MASTER_STARTXFER 0x9c /* width RMuint32 */
#define I2C_MASTER_BYTE_CNT 0xa0 /* width RMuint32 */
#define I2C_MASTER_INTEN 0xa4 /* width RMuint32 */
#define I2C_MASTER_INT 0xa8 /* width RMuint32 */
#define I2C_SLAVE_ADDR_REG 0xC0 /* width RMuint32 */
#define I2C_SLAVE_DATAOUT 0xC4 /* width RMuint32 */
#define I2C_SLAVE_DATAIN 0xC8 /* width RMuint32 */
#define I2C_SLAVE_STATUS 0xCC /* width RMuint32 */
#define I2C_SLAVE_INTEN 0xD0 /* width RMuint32 */
#define I2C_SLAVE_INT 0xD4 /* width RMuint32 */
#define I2C_SLAVE_BUS_HOLD 0xD8 /* width RMuint32 */
/* I2C registers done */

/* MM registers */
/* MM registers done */

/* SpuDecoder registers */
/* SpuDecoder registers done */

/* PictureTransform registers */
/* PictureTransform registers done */

/* ClosedCaptionDecoder registers */
/* ClosedCaptionDecoder registers done */

/* RTC registers */
/* RTC registers done */

/* Cipher registers */
/* Cipher registers done */

/* STC registers */
/* STC registers done */

/* PLL registers */
/* PLL registers done */

/* DemuxCipher registers */
/* DemuxCipher registers done */

/* DemuxTask registers */
/* DemuxTask registers done */

/* DemuxOutput registers */
/* DemuxOutput registers done */

/* CCFifo registers */
/* CCFifo registers done */

/* Sha1Sum registers */
/* Sha1Sum registers done */

/* XTask registers */
/* XTask registers done */

/* TTXFifo registers */
/* TTXFifo registers done */

/* VCXO registers */
/* VCXO registers done */

/* PPF registers */
/* PPF registers done */

#endif /* __EMHWLIB_REGISTERS_TANGO2_H__ */

/* End of generated file ../emhwlib_hal/include/tango2/emhwlib_registers_tango2.h */
