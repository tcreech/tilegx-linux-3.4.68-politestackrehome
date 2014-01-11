/*
 * Copyright 2013 Tilera Corporation. All Rights Reserved.
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation, version 2.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 *   NON INFRINGEMENT.  See the GNU General Public License for
 *   more details.
 */

#ifndef __ARCH_RSH_H__
#define __ARCH_RSH_H__

#include <arch/abi.h>
#include <arch/rsh_def.h>

#ifndef __ASSEMBLER__




/*
 * MMIO Address Space.
 * The MMIO physical address space for the rshim is described below.  This is
 * a general description of the MMIO space as opposed to a register
 * description
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * This field of the address provides an offset into the channel whose
     * register space is being accessed.
     */
    uint_reg_t offset       : 16;
    /*
     * Protection level.  Setting to 0 or 1 allows access to all registers.
     * Setting to 2 denies access to registers at level 2.  Setting to 3
     * denies access to registers at levels 1 and 2.
     */
    uint_reg_t prot         : 2;
    /* Reserved. */
    uint_reg_t __reserved_0 : 18;
    /*
     * This field of the address selects the channel (device).  Channel-0 is
     * the common rshim registers.  Channels 1-7 are used to access the
     * rshim-attached devices.
     */
    uint_reg_t channel      : 4;
    /* Reserved. */
    uint_reg_t __reserved_1 : 24;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 24;
    uint_reg_t channel      : 4;
    uint_reg_t __reserved_0 : 18;
    uint_reg_t prot         : 2;
    uint_reg_t offset       : 16;
#endif
  };

  uint_reg_t word;
} RSH_MMIO_ADDRESS_SPACE_t;


/* Boot Control. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Indicates source for ROM boot on next software reset.  Initialized
     * based on strapping pins on hard_rst_l.  This register is not reset on
     * software reset.
     */
    uint_reg_t boot_mode  : 2;
    /* Reserved. */
    uint_reg_t __reserved : 62;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 62;
    uint_reg_t boot_mode  : 2;
#endif
  };

  uint_reg_t word;
} RSH_BOOT_CONTROL_t;


/*
 * Breadcrumb0.
 * Scratchpad register that resets only on hard_rst_l (power on reset).
 * Typically used by software to leave information for the reboot software on
 * software reset.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Breadcrumb0.
     * Scratchpad register that resets only on hard_rst_l (power on reset).
     * Typically used by software to leave information for the reboot
     * software on software reset.
     */
    uint_reg_t breadcrumb0 : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t breadcrumb0 : 64;
#endif
  };

  uint_reg_t word;
} RSH_BREADCRUMB0_t;


/*
 * Breadcrumb1.
 * Scratchpad register that resets only on hard_rst_l (power on reset).
 * Typically used by software to leave information for the reboot software on
 * software reset.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Breadcrumb1.
     * Scratchpad register that resets only on hard_rst_l (power on reset).
     * Typically used by software to leave information for the reboot
     * software on software reset.
     */
    uint_reg_t breadcrumb1 : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t breadcrumb1 : 64;
#endif
  };

  uint_reg_t word;
} RSH_BREADCRUMB1_t;


/*
 * Byte Access Register Address.
 * Register address used for read/write accesses triggered by
 * BYTE_ACC_WDAT/RDAT.  The BYTE_ACC_CTL.BYTE_PTR is used and auto
 * incremented when writing or reading this register.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Reserved. */
    uint_reg_t __reserved_0 : 3;
    /*
     * This field selects the 8-byte register to be accessed within the
     * channel specified by CHANNEL.
     */
    uint_reg_t regnum       : 13;
    /*
     * This field of the address selects the channel (device).  Channel-0 is
     * the common rshim registers.  Channels 1-7 are used to access the
     * rshim-attached devices.
     */
    uint_reg_t channel      : 4;
    /* Reserved. */
    uint_reg_t __reserved_1 : 44;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 44;
    uint_reg_t channel      : 4;
    uint_reg_t regnum       : 13;
    uint_reg_t __reserved_0 : 3;
#endif
  };

  uint_reg_t word;
} RSH_BYTE_ACC_ADDR_t;


/*
 * Byte Access Control.
 * Provides 8-byte access to rshim registers using 1B, 2B, or 4B accesses.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Byte-index into the 8-byte wdat/rdat/addr register for access.
     * Increments automatically on access to the BYTE_ACC_WDAT,
     * BYTE_ACC_RDAT, or BYTE_ACC_ADDR registers.
     */
    uint_reg_t byte_ptr   : 3;
    /*
     * Size of access.  This controls how much the BYTE_PTR increments by on
     * each BYTE_ACC_WDAT/RDAT access.
     */
    uint_reg_t size       : 2;
    /*
     * When asserted, an access is still pending.  Attempting to access the
     * WDAT/RDAT/ADDR registers when an access is pending has unpredictable
     * results.
     */
    uint_reg_t pending    : 1;
    /*
     * When written with a 1, a read to BYTE_ACC_ADDR is triggered and the
     * resulting data is placed into BYTE_ACC_RDAT.
     */
    uint_reg_t read_trig  : 1;
    /* Reserved. */
    uint_reg_t __reserved : 57;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 57;
    uint_reg_t read_trig  : 1;
    uint_reg_t pending    : 1;
    uint_reg_t size       : 2;
    uint_reg_t byte_ptr   : 3;
#endif
  };

  uint_reg_t word;
} RSH_BYTE_ACC_CTL_t;


/*
 * Byte Access Read Data.
 * 8 bytes of read data captured when READ_TRIGGER is written with a 1.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Each time this register is read, the BYTE_ACC_CTL.BYTE_PTR is
     * incremented based on BYTE_ACC_CTL.SIZE.  The bytes returned will be
     * shifted into the least significant BYTE_ACC_CTL.SIZE bytes based on
     * the current value of BYTE_ACC_CTL.BYTE_PTR.
     */
    uint_reg_t dat : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t dat : 64;
#endif
  };

  uint_reg_t word;
} RSH_BYTE_ACC_RDAT_t;


/*
 * Byte Access Write Data.
 * 8 bytes of write data written when BYTE_ACC_CTL.BYTE_PTR wraps back to
 * zero.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * The 8-bytes in this register are updated based on
     * BYTE_ACC_CTL.BYTE_PTR and SIZE.  Each time this register is written,
     * the BYTE_ACC_CTL.BYTE_PTR is incremented based on BYTE_ACC_CTL.SIZE.
     * When the most significant byte of this register is written, the
     * register at address BYTE_ACC_ADDR is written using the data in this
     * register.
     */
    uint_reg_t dat : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t dat : 64;
#endif
  };

  uint_reg_t word;
} RSH_BYTE_ACC_WDAT_t;


/*
 * Clock Control.
 * Provides control over the core PLL
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * When written with a 1, the PLL will be configured with the settings in
     * PLL_RANGE/Q/N/M.  When written with a zero, the PLL will be bypassed.
     */
    uint_reg_t ena          : 1;
    /*
     * This value should be set based on the post divider frequency,
     * REF/(PLL_N+1).
     */
    uint_reg_t pll_range    : 3;
    /*
     * Output divider.  The VCO clock is divided by 2^PLL_Q to create the
     * final output clock.  This should be set such that 2133
     * MHz<(output_frequency * 2^PLL_Q)<4266 MHz.  The maximum supported
     * value of PLL_Q is 6 (divide by 64).
     */
    uint_reg_t pll_q        : 3;
    /*
     * Reference divider value.  Input refclk is divided by (PLL_N+1).  The
     * post-divided clock must be in the range of 14 MHz to 200 MHz.  The low
     * 5 bits of this field are used.  The MSB is reserved.  The minimum
     * value of PLL_N that keeps the post-divided clock within the legal
     * range will provide the best jitter performance.
     */
    uint_reg_t pll_n        : 6;
    /*
     * Feedback divider value.  The resulting clock frequency is calculated
     * as ((REF / (PLL_N+1)) * 2 * (PLL_M + 1)) / (2^Q).  The VCO frequency
     * is (REF / (PLL_N+1)) * 2 * (PLL_M + 1)) and must be in the range of
     * 2133 MHz to 4266 MHz.
     *
     * For example, to create a 1 GHz clock from a 125 MHz refclk, Q would be
     * set to 2 so that the VCO would be 1 GHz*2^2 = 4000 MHz.
     * 2*(PLL_M+1)/(PLL_N+1) = 32.  PLL_N=0 and PLL_M=15 would keep the post
     * divide frequency within the legal range since 125/(0+1) is 125.
     * PLL_RANGE would be set to R104_166.
     */
    uint_reg_t pll_m        : 8;
    /* Reserved. */
    uint_reg_t __reserved_0 : 10;
    /*
     * Indicates that PLL has been enabled and clock is now running off of
     * the PLL output.
     */
    uint_reg_t clock_ready  : 1;
    /* Reserved. */
    uint_reg_t __reserved_1 : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 32;
    uint_reg_t clock_ready  : 1;
    uint_reg_t __reserved_0 : 10;
    uint_reg_t pll_m        : 8;
    uint_reg_t pll_n        : 6;
    uint_reg_t pll_q        : 3;
    uint_reg_t pll_range    : 3;
    uint_reg_t ena          : 1;
#endif
  };

  uint_reg_t word;
} RSH_CLOCK_CONTROL_t;


/* Clock Count. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * When 1, the counter is running.  Cleared by HW once count is complete.
     *  When written with a 1, the count sequence is restarted.  Counter runs
     * automatically after reset.  Software must poll until this bit is zero,
     * then read the CLOCK_COUNT register again to get the final COUNT value.
     */
    uint_reg_t run        : 1;
    /*
     * Indicates the number of core clocks that were counted during 1000
     * device clock periods.  Result is accurate to within +/-1 core clock
     * periods.
     */
    uint_reg_t count      : 15;
    /* Reserved. */
    uint_reg_t __reserved : 48;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 48;
    uint_reg_t count      : 15;
    uint_reg_t run        : 1;
#endif
  };

  uint_reg_t word;
} RSH_CLOCK_COUNT_t;


/*
 * Device Protection.
 * Controls device access to rshim services.  When a bit is one, the
 * associated device will be disabled.  Reads and writes from the device will
 * acknowledged but ignored.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Each bit corresponds to a device.  The device mapping is:
     *
     * 0 : UART0.
     * 1 : UART1.
     * 2 : I2CM0.
     * 3 : I2CM1.
     * 4 : I2CM2.
     * 5 : I2CS.
     * 6 : SPI.
     */
    uint_reg_t disable_device : 7;
    /* Reserved. */
    uint_reg_t __reserved_0   : 25;
    /*
     * Each bit corresponds to a remote command interface.  Multiple bits may
     * be set.  The interface mapping is:
     *
     * 0 : JTAG.
     * 1 : TRIO0.
     * 2 : USB.
     * 3 : TRIO1.
     * 4 : RSVD4.
     * 5 : RSVD5.
     * 6 : RSVD6.
     * 7 : RSVD7.
     */
    uint_reg_t disable_rci    : 8;
    /* Reserved. */
    uint_reg_t __reserved_1   : 24;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1   : 24;
    uint_reg_t disable_rci    : 8;
    uint_reg_t __reserved_0   : 25;
    uint_reg_t disable_device : 7;
#endif
  };

  uint_reg_t word;
} RSH_DEVICE_PROTECTION_t;


/*
 * Device Control.
 * This register provides general device control.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Reserved. */
    uint_reg_t __reserved_0    : 2;
    /*
     * When 1, packets sent on the RDN will be routed x-first.  When 0,
     * packets will be routed y-first.  This setting must match the setting
     * in the Tiles.  Devices may have additional interfaces with customized
     * route-order settings used in addition to or instead of this field.
     */
    uint_reg_t rdn_route_order : 1;
    /*
     * When 1, packets sent on the SDN will be routed x-first.  When 0,
     * packets will be routed y-first.  This setting must match the setting
     * in the Tiles.  Devices may have additional interfaces with customized
     * route-order settings used in addition to or instead of this field.
     */
    uint_reg_t sdn_route_order : 1;
    /* Reserved. */
    uint_reg_t __reserved_1    : 60;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1    : 60;
    uint_reg_t sdn_route_order : 1;
    uint_reg_t rdn_route_order : 1;
    uint_reg_t __reserved_0    : 2;
#endif
  };

  uint_reg_t word;
} RSH_DEV_CTL_t;


/*
 * Device Info.
 * This register provides general information about the device attached to
 * this port and channel.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Encoded device Type - 33 to indicate rshim */
    uint_reg_t type         : 12;
    /* Reserved. */
    uint_reg_t __reserved_0 : 4;
    /* Device revision ID. */
    uint_reg_t device_rev   : 8;
    /* Register format architectural revision. */
    uint_reg_t register_rev : 4;
    /* Reserved. */
    uint_reg_t __reserved_1 : 4;
    /* Instance ID for multi-instantiated devices. */
    uint_reg_t instance     : 4;
    /* Reserved. */
    uint_reg_t __reserved_2 : 28;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_2 : 28;
    uint_reg_t instance     : 4;
    uint_reg_t __reserved_1 : 4;
    uint_reg_t register_rev : 4;
    uint_reg_t device_rev   : 8;
    uint_reg_t __reserved_0 : 4;
    uint_reg_t type         : 12;
#endif
  };

  uint_reg_t word;
} RSH_DEV_INFO_t;


/* Diag Broadcast. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Asserted by hardware when the associated broadcast wire is asserted
     * (Bit[0] = BCST0, Bit[1] = BCST1, Bit[2] = BCST2, Bit[3] = BCST3).
     * Cleared by software writing a 1 to this register.   This field
     * provides the same functionality as the associated interrupt status bit.
     */
    uint_reg_t diag_bcst_in      : 4;
    /*
     * When written with a 1, the associated diag broadcast wire will be
     * asserted for a single cycle going to the neighboring tile.
     */
    uint_reg_t diag_bcst_out     : 4;
    /*
     * Each bit corresponds to one of the rshim's down-counters (0=DCNT0,
     * 1=DCNT1, 2=DCNT2).  When asserted, the down counter's interrupt pulse
     * will be sent onto the associated diag broadcast network (BCST0 =
     * DCNT0,  BCST1 = DCNT1,  BCST2 = DCNT2).
     */
    uint_reg_t diag_dcnt_drv_ena : 3;
    /*
     * When one of the DIAG_DCNT_DRV_ENA bits is set, this controls how the
     * associated network is driven.  1 indicates level, 0 indicates pulse.
     * When set to level, the DIAG_BCST_OUT_LVL bit will be asserted when the
     * associated DIAG_DCNT_DRV_ENA event occurs.  In DRV_LVL one mode,
     * software can clear a bit that's been asserted by the down counter by
     * writing a zero do the DIAG_BCST_OUT_LVL field.
     */
    uint_reg_t diag_dcnt_drv_lvl : 1;
    /*
     * When written with a 1, the associated diag broadcast wire will be
     * asserted.  When written with a zero, the associated diag broadcast
     * wire will be deasserted.  These bits may also be set by hardware when
     * DIAG_DCNT_DRV_LVL is 1 and the associated down counter interrupt
     * occurs with the corresponding DIAG_DCNT_DRV_ENA bit set.
     */
    uint_reg_t diag_bcst_out_lvl : 4;
    /* Reserved. */
    uint_reg_t __reserved        : 48;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved        : 48;
    uint_reg_t diag_bcst_out_lvl : 4;
    uint_reg_t diag_dcnt_drv_lvl : 1;
    uint_reg_t diag_dcnt_drv_ena : 3;
    uint_reg_t diag_bcst_out     : 4;
    uint_reg_t diag_bcst_in      : 4;
#endif
  };

  uint_reg_t word;
} RSH_DIAG_BROADCAST_t;


/* Down Count Control. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * When 0, the current value is frozen and no interrupts will be signaled.
     */
    uint_reg_t ena          : 1;
    /* Clock mode for the down counter. */
    uint_reg_t mode         : 2;
    /*
     * Pin select for external clocking.  Can source 1 of 3 pins.  This
     * setting only matters if MODE is nonzero.  The minimum pulse width
     * supported on these pins is 10 ns (50 MHz with a 50% duty cycle).
     */
    uint_reg_t pinsel       : 2;
    /* Reserved. */
    uint_reg_t __reserved_0 : 7;
    /*
     * Clock divisor.  Number of additional ticks of clock source to trigger
     * a down count.  Setting to 0 causes the down counter to decrement every
     * cycle.  Setting to 1 causes the down counter to decrement every other
     * cycle.  To convert a 10 MHz source to a 100 KHz down count, for
     * example, the value would be set to 99(d).
     */
    uint_reg_t div          : 20;
    /* Reserved. */
    uint_reg_t __reserved_1 : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 32;
    uint_reg_t div          : 20;
    uint_reg_t __reserved_0 : 7;
    uint_reg_t pinsel       : 2;
    uint_reg_t mode         : 2;
    uint_reg_t ena          : 1;
#endif
  };

  uint_reg_t word;
} RSH_DOWN_COUNT_CONTROL_t;


/* Down Count Control. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * When 0, the current value is frozen and no interrupts will be signaled.
     */
    uint_reg_t ena          : 1;
    /* Clock mode for the down counter. */
    uint_reg_t mode         : 2;
    /*
     * Pin select for external clocking.  Can source 1 of 3 pins.  This
     * setting only matters if MODE is nonzero.  The minimum pulse width
     * supported on these pins is 10 ns (50 MHz with a 50% duty cycle).
     */
    uint_reg_t pinsel       : 2;
    /* Reserved. */
    uint_reg_t __reserved_0 : 7;
    /*
     * Clock divisor.  Number of additional ticks of clock source to trigger
     * a down count.  Setting to 0 causes the down counter to decrement every
     * cycle.  Setting to 1 causes the down counter to decrement every other
     * cycle.  To convert a 10 MHz source to a 100 KHz down count, for
     * example, the value would be set to 99(d).
     */
    uint_reg_t div          : 20;
    /* Reserved. */
    uint_reg_t __reserved_1 : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 32;
    uint_reg_t div          : 20;
    uint_reg_t __reserved_0 : 7;
    uint_reg_t pinsel       : 2;
    uint_reg_t mode         : 2;
    uint_reg_t ena          : 1;
#endif
  };

  uint_reg_t word;
} RSH_DOWN_COUNT_CONTROL_1_t;


/* Down Count Control. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * When 0, the current value is frozen and no interrupts will be signaled.
     */
    uint_reg_t ena          : 1;
    /* Clock mode for the down counter. */
    uint_reg_t mode         : 2;
    /*
     * Pin select for external clocking.  Can source 1 of 3 pins.  This
     * setting only matters if MODE is nonzero.  The minimum pulse width
     * supported on these pins is 10 ns (50 MHz with a 50% duty cycle).
     */
    uint_reg_t pinsel       : 2;
    /* Reserved. */
    uint_reg_t __reserved_0 : 7;
    /*
     * Clock divisor.  Number of additional ticks of clock source to trigger
     * a down count.  Setting to 0 causes the down counter to decrement every
     * cycle.  Setting to 1 causes the down counter to decrement every other
     * cycle.  To convert a 10 MHz source to a 100 KHz down count, for
     * example, the value would be set to 99(d).
     */
    uint_reg_t div          : 20;
    /* Reserved. */
    uint_reg_t __reserved_1 : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 32;
    uint_reg_t div          : 20;
    uint_reg_t __reserved_0 : 7;
    uint_reg_t pinsel       : 2;
    uint_reg_t mode         : 2;
    uint_reg_t ena          : 1;
#endif
  };

  uint_reg_t word;
} RSH_DOWN_COUNT_CONTROL_2_t;


/* Down Count Refresh Value. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * When COUNT reaches 0, this value is loaded at the next time interval
     * (defined in the control register).
     */
    uint_reg_t refresh    : 48;
    /* Reserved. */
    uint_reg_t __reserved : 16;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 16;
    uint_reg_t refresh    : 48;
#endif
  };

  uint_reg_t word;
} RSH_DOWN_COUNT_REFRESH_VALUE_t;


/* Down Count Refresh Value. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * When COUNT reaches 0, this value is loaded at the next time interval
     * (defined in the control register).
     */
    uint_reg_t refresh    : 48;
    /* Reserved. */
    uint_reg_t __reserved : 16;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 16;
    uint_reg_t refresh    : 48;
#endif
  };

  uint_reg_t word;
} RSH_DOWN_COUNT_REFRESH_VALUE_1_t;


/* Down Count Refresh Value. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * When COUNT reaches 0, this value is loaded at the next time interval
     * (defined in the control register).
     */
    uint_reg_t refresh    : 48;
    /* Reserved. */
    uint_reg_t __reserved : 16;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 16;
    uint_reg_t refresh    : 48;
#endif
  };

  uint_reg_t word;
} RSH_DOWN_COUNT_REFRESH_VALUE_2_t;


/* Down Count Current Value. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Current value for the down counter.  Writable by SW.  Decremented by
     * HW based on setup in control register.  When counter is 0 and timer
     * tick occurs, an interrupt is signaled.
     */
    uint_reg_t count      : 48;
    /* Reserved. */
    uint_reg_t __reserved : 16;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 16;
    uint_reg_t count      : 48;
#endif
  };

  uint_reg_t word;
} RSH_DOWN_COUNT_VALUE_t;


/* Down Count Current Value. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Current value for the down counter.  Writable by SW.  Decremented by
     * HW based on setup in control register.  When counter is 0 and timer
     * tick occurs, an interrupt is signaled.
     */
    uint_reg_t count      : 48;
    /* Reserved. */
    uint_reg_t __reserved : 16;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 16;
    uint_reg_t count      : 48;
#endif
  };

  uint_reg_t word;
} RSH_DOWN_COUNT_VALUE_1_t;


/* Down Count Current Value. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Current value for the down counter.  Writable by SW.  Decremented by
     * HW based on setup in control register.  When counter is 0 and timer
     * tick occurs, an interrupt is signaled.
     */
    uint_reg_t count      : 48;
    /* Reserved. */
    uint_reg_t __reserved : 16;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 16;
    uint_reg_t count      : 48;
#endif
  };

  uint_reg_t word;
} RSH_DOWN_COUNT_VALUE_2_t;


/*
 * EFUSE Control.
 * Provides access to the on-chip eFuse structure
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* 8-byte address into the fuse array. */
    uint_reg_t index        : 5;
    /* Reserved. */
    uint_reg_t __reserved_0 : 3;
    /*
     * When 1, the fuse read is still pending and the data in EFUSE_DATA is
     * not valid.  Software must poll this bit until clear whenever
     * EFUSE_CTL.INDEX is changed.
     */
    uint_reg_t read_pend    : 1;
    /* Reserved. */
    uint_reg_t __reserved_1 : 55;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 55;
    uint_reg_t read_pend    : 1;
    uint_reg_t __reserved_0 : 3;
    uint_reg_t index        : 5;
#endif
  };

  uint_reg_t word;
} RSH_EFUSE_CTL_t;


/*
 * EFUSE Control Data word0.
 * EFUSE_DATA contents when EFUSE_CTL.INDEX is set to the first efuse control
 * (after IO_DISABLE words).
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Contents of the Tile-Disable fuses for column-0. */
    uint_reg_t col0_dis   : 10;
    /* Contents of the Tile-Disable fuses for column-1. */
    uint_reg_t col1_dis   : 10;
    /* Contents of the Tile-Disable fuses for column-2. */
    uint_reg_t col2_dis   : 10;
    /* Contents of the Tile-Disable fuses for column-3. */
    uint_reg_t col3_dis   : 10;
    /* Contents of the Tile-Disable fuses for column-4. */
    uint_reg_t col4_dis   : 10;
    /* Contents of the Tile-Disable fuses for column-5. */
    uint_reg_t col5_dis   : 10;
    /* Reserved. */
    uint_reg_t __reserved : 4;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 4;
    uint_reg_t col5_dis   : 10;
    uint_reg_t col4_dis   : 10;
    uint_reg_t col3_dis   : 10;
    uint_reg_t col2_dis   : 10;
    uint_reg_t col1_dis   : 10;
    uint_reg_t col0_dis   : 10;
#endif
  };

  uint_reg_t word;
} RSH_EFUSE_CTL_DATA_0_t;


/*
 * EFUSE Control Data word1.
 * EFUSE_DATA contents when EFUSE_CTL.INDEX is set to the second efuse
 * control (after IO_DISABLE words).
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Contents of the Tile-Disable fuses for column-6. */
    uint_reg_t col6_dis          : 10;
    /* Contents of the Tile-Disable fuses for column-7. */
    uint_reg_t col7_dis          : 10;
    /* Contents of the Tile-Disable fuses for column-8. */
    uint_reg_t col8_dis          : 10;
    /* Contents of the Tile-Disable fuses for column-9. */
    uint_reg_t col9_dis          : 10;
    /* Provides an adjustment of the reset timer. */
    uint_reg_t reset_adj         : 16;
    /* Cache SKU Configuration */
    uint_reg_t cache_sku         : 4;
    /* This bit is not used (RESERVED). */
    uint_reg_t cache_sku_disable : 1;
    /*
     * Fuse info valid.  Bit[61] must be zero and Bit[62] must be one to
     * indicate valid efuse data.
     */
    uint_reg_t info_vld          : 2;
    /* FUSEs are Write-Protected */
    uint_reg_t fuse_protect      : 1;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t fuse_protect      : 1;
    uint_reg_t info_vld          : 2;
    uint_reg_t cache_sku_disable : 1;
    uint_reg_t cache_sku         : 4;
    uint_reg_t reset_adj         : 16;
    uint_reg_t col9_dis          : 10;
    uint_reg_t col8_dis          : 10;
    uint_reg_t col7_dis          : 10;
    uint_reg_t col6_dis          : 10;
#endif
  };

  uint_reg_t word;
} RSH_EFUSE_CTL_DATA_1_t;


/*
 * EFUSE Data.
 * Data corresponding to EFUSE_CTL.INDEX.  Valid only when
 * EFUSE_CTL.READ_PEND is zero.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EFUSE Data.
     * Data corresponding to EFUSE_CTL.INDEX.  Valid only when
     * EFUSE_CTL.READ_PEND is zero.
     */
    uint_reg_t efuse_data : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t efuse_data : 64;
#endif
  };

  uint_reg_t word;
} RSH_EFUSE_DATA_t;


/*
 * Error Status.
 * Indicators for various fatal and non-fatal RSH error conditions
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Illegal opcode received on MMIO interface */
    uint_reg_t mmio_ill_opc : 1;
    /* Reserved. */
    uint_reg_t __reserved   : 63;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved   : 63;
    uint_reg_t mmio_ill_opc : 1;
#endif
  };

  uint_reg_t word;
} RSH_ERROR_STATUS_t;


/*
 * Fabric Connections.
 * Indicates devices connected at the edges of the Tile fabric.  A set bit
 * indicates that an IO device will respond to MMIO accesses at the
 * associated location.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Bit[0] represents the device located to the North of Tile(0,0).
     * Bit[1] represents the device located to the North of Tile(1,0).  etc.
     */
    uint_reg_t north : 16;
    /*
     * Bit[0] represents the device located to the East of Tile(x,0).  Bit[1]
     * represents the device located to the East of Tile(x,1). etc.
     */
    uint_reg_t east  : 16;
    /*
     * Bit[0] represents the device located to the South of Tile(0,x).
     * Bit[1] represents the device located to the South of Tile(1,x). etc.
     */
    uint_reg_t south : 16;
    /*
     * Bit[0] represents the device located to the West of Tile(0,0).  Bit[1]
     * represents the device located to the West of Tile(0,1). etc.
     */
    uint_reg_t west  : 16;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t west  : 16;
    uint_reg_t south : 16;
    uint_reg_t east  : 16;
    uint_reg_t north : 16;
#endif
  };

  uint_reg_t word;
} RSH_FABRIC_CONN_t;


/*
 * Fabric Dimensions.
 * Indicates the size of the Tile Fabric and the location of the rshim.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Indicates the number of Tiles in the Y dimension. */
    uint_reg_t dim_y      : 4;
    /* Indicates the number of Tiles in the X dimension. */
    uint_reg_t dim_x      : 4;
    /*
     * Indicates the location of rshim on the mesh.  If this register returns
     * 0x00, the rshim location is 1,-1 (or 0x1f as an encoded mesh
     * coordinate).
     */
    uint_reg_t rshim_loc  : 8;
    /* Reserved. */
    uint_reg_t __reserved : 48;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 48;
    uint_reg_t rshim_loc  : 8;
    uint_reg_t dim_x      : 4;
    uint_reg_t dim_y      : 4;
#endif
  };

  uint_reg_t word;
} RSH_FABRIC_DIM_t;


/*
 * IO Disable-0.
 * Device specific feature disable bits.  These may be set (hardcoded) in
 * hardware or set by software.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Each bit corresponds to an SGMII port.  When set, the port is disabled.
     */
    uint_reg_t mpipe_sgmii_3_0_disable   : 4;
    /* When set, the associated XAUI port is disabled. */
    uint_reg_t mpipe_xaui0_disable       : 1;
    /*
     * Each bit corresponds to an SGMII port.  When set, the port is disabled.
     */
    uint_reg_t mpipe_sgmii_7_4_disable   : 4;
    /* When set, the associated XAUI port is disabled. */
    uint_reg_t mpipe_xaui1_disable       : 1;
    /*
     * Each bit corresponds to an SGMII port.  When set, the port is disabled.
     */
    uint_reg_t mpipe_sgmii_11_8_disable  : 4;
    /* When set, the associated XAUI port is disabled. */
    uint_reg_t mpipe_xaui2_disable       : 1;
    /*
     * Each bit corresponds to an SGMII port.  When set, the port is disabled.
     */
    uint_reg_t mpipe_sgmii_15_12_disable : 4;
    /* When set, the associated XAUI port is disabled. */
    uint_reg_t mpipe_xaui3_disable       : 1;
    /*
     * Each bit corresponds to a classification processor.  When set, the
     * processor is disabled.
     */
    uint_reg_t mpipe_cls_disable         : 10;
    /* When set, the entire MPIPE is disabled. */
    uint_reg_t mpipe_device_disable      : 1;
    /* Configures width/availability of the PCIe port */
    uint_reg_t trio_pcie_port0_config    : 2;
    /* Configures width/availability of the PCIe port */
    uint_reg_t trio_pcie_port1_config    : 2;
    /* Configures width/availability of the PCIe port */
    uint_reg_t trio_pcie_port2_config    : 2;
    /*
     * Each bit corresponds to a StreamIO port.  When set, the associated
     * port may not be used for StreamIO.
     */
    uint_reg_t trio_stream_disable       : 3;
    /* When set, the entire TRIO interface is disabled. */
    uint_reg_t trio_device_disable       : 1;
    /* When set, the associated DDR3 interface is disabled. */
    uint_reg_t ddr3_disable              : 2;
    /* When set, USB Host0 is disabled. */
    uint_reg_t usb_host0_disable         : 1;
    /* When set, USB Host1 is disabled. */
    uint_reg_t usb_host1_disable         : 1;
    /* When set, the USB endpoint is disabled. */
    uint_reg_t usb_ep_disable            : 1;
    /* When set, the entire USB interface is disabled (all ports disabled). */
    uint_reg_t usb_interface_disable     : 1;
    /* When set, the MemCopy engine is disabled. */
    uint_reg_t crypto0_mem_cpy_disable   : 1;
    /* When set, the SNOW and KASUMI crypto engines are disabled. */
    uint_reg_t crypto0_wireless_disable  : 1;
    /*
     * When set, the associated set of packet crypto engines (DES/AES/SHA/MD5
     * etc.) is disabled.
     */
    uint_reg_t crypto0_pkt_disable       : 2;
    /* When set, the public key engine is disabled. */
    uint_reg_t crypto0_pubkey_disable    : 1;
    /* When set, the entire crypto interface is disabled for all engines. */
    uint_reg_t crypto0_interface_disable : 1;
    /* When set, the MemCopy engine is disabled. */
    uint_reg_t crypto1_mem_cpy_disable   : 1;
    /* When set, the SNOW and KASUMI crypto engines are disabled. */
    uint_reg_t crypto1_wireless_disable  : 1;
    /*
     * When set, the associated set of packet crypto engines (DES/AES/SHA/MD5
     * etc.) is disabled.
     */
    uint_reg_t crypto1_pkt_disable       : 2;
    /* When set, the public key engine is disabled. */
    uint_reg_t crypto1_pubkey_disable    : 1;
    /* When set, the entire crypto interface is disabled for all engines. */
    uint_reg_t crypto1_interface_disable : 1;
    /* Reserved. */
    uint_reg_t reserved_io               : 5;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t reserved_io               : 5;
    uint_reg_t crypto1_interface_disable : 1;
    uint_reg_t crypto1_pubkey_disable    : 1;
    uint_reg_t crypto1_pkt_disable       : 2;
    uint_reg_t crypto1_wireless_disable  : 1;
    uint_reg_t crypto1_mem_cpy_disable   : 1;
    uint_reg_t crypto0_interface_disable : 1;
    uint_reg_t crypto0_pubkey_disable    : 1;
    uint_reg_t crypto0_pkt_disable       : 2;
    uint_reg_t crypto0_wireless_disable  : 1;
    uint_reg_t crypto0_mem_cpy_disable   : 1;
    uint_reg_t usb_interface_disable     : 1;
    uint_reg_t usb_ep_disable            : 1;
    uint_reg_t usb_host1_disable         : 1;
    uint_reg_t usb_host0_disable         : 1;
    uint_reg_t ddr3_disable              : 2;
    uint_reg_t trio_device_disable       : 1;
    uint_reg_t trio_stream_disable       : 3;
    uint_reg_t trio_pcie_port2_config    : 2;
    uint_reg_t trio_pcie_port1_config    : 2;
    uint_reg_t trio_pcie_port0_config    : 2;
    uint_reg_t mpipe_device_disable      : 1;
    uint_reg_t mpipe_cls_disable         : 10;
    uint_reg_t mpipe_xaui3_disable       : 1;
    uint_reg_t mpipe_sgmii_15_12_disable : 4;
    uint_reg_t mpipe_xaui2_disable       : 1;
    uint_reg_t mpipe_sgmii_11_8_disable  : 4;
    uint_reg_t mpipe_xaui1_disable       : 1;
    uint_reg_t mpipe_sgmii_7_4_disable   : 4;
    uint_reg_t mpipe_xaui0_disable       : 1;
    uint_reg_t mpipe_sgmii_3_0_disable   : 4;
#endif
  };

  uint_reg_t word;
} RSH_GX36_IO_DISABLE0_t;


/*
 * IO Disable-1.
 * Device specific feature disable bits.  These may be set (hardcoded) in
 * hardware or set by software.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* When set, the associated compression engine is disabled. */
    uint_reg_t comp0_disable           : 1;
    /* When set, the associated decompression engine is disabled. */
    uint_reg_t decomp0_disable         : 1;
    /* When set, the associated MEM_CPY (DMA) interface is disabled. */
    uint_reg_t mem_cpy0_disable        : 1;
    /*
     * When set, the entire compression/decompression/MemCPY interface is
     * disabled.
     */
    uint_reg_t comp_interface0_disable : 1;
    /* When set, the associated compression engine is disabled. */
    uint_reg_t comp1_disable           : 1;
    /* When set, the associated decompression engine is disabled. */
    uint_reg_t decomp1_disable         : 1;
    /* When set, the associated MEM_CPY (DMA) interface is disabled. */
    uint_reg_t mem_cpy1_disable        : 1;
    /*
     * When set, the entire compression/decompression/MemCPY interface is
     * disabled.
     */
    uint_reg_t comp_interface1_disable : 1;
    /* Reserved. */
    uint_reg_t reserved_io             : 28;
    /* Reserved. */
    uint_reg_t __reserved              : 28;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved              : 28;
    uint_reg_t reserved_io             : 28;
    uint_reg_t comp_interface1_disable : 1;
    uint_reg_t mem_cpy1_disable        : 1;
    uint_reg_t decomp1_disable         : 1;
    uint_reg_t comp1_disable           : 1;
    uint_reg_t comp_interface0_disable : 1;
    uint_reg_t mem_cpy0_disable        : 1;
    uint_reg_t decomp0_disable         : 1;
    uint_reg_t comp0_disable           : 1;
#endif
  };

  uint_reg_t word;
} RSH_GX36_IO_DISABLE1_t;


/*
 * IO Reset.
 * Each bit corresponds to a device or function within a device.  When
 * written with a 1, the associated device or function will be reset.
 * Software is responsible for any quiesce procedures required for the
 * associated device prior to reset.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    uint_reg_t mpipe_pclk  : 1;
    uint_reg_t mpipe_kclk  : 1;
    uint_reg_t mpipe_quad0 : 1;
    uint_reg_t mpipe_quad1 : 1;
    uint_reg_t mpipe_quad2 : 1;
    uint_reg_t mpipe_quad3 : 1;
    uint_reg_t trio        : 1;
    uint_reg_t pcie0       : 1;
    uint_reg_t pcie1       : 1;
    uint_reg_t pcie2       : 1;
    uint_reg_t usb         : 1;
    uint_reg_t msh0        : 1;
    uint_reg_t msh1        : 1;
    uint_reg_t gpio        : 1;
    uint_reg_t compression : 1;
    uint_reg_t crypto      : 1;
    uint_reg_t msh2        : 1;
    uint_reg_t msh3        : 1;
    /* Reserved. */
    uint_reg_t __reserved  : 46;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved  : 46;
    uint_reg_t msh3        : 1;
    uint_reg_t msh2        : 1;
    uint_reg_t crypto      : 1;
    uint_reg_t compression : 1;
    uint_reg_t gpio        : 1;
    uint_reg_t msh1        : 1;
    uint_reg_t msh0        : 1;
    uint_reg_t usb         : 1;
    uint_reg_t pcie2       : 1;
    uint_reg_t pcie1       : 1;
    uint_reg_t pcie0       : 1;
    uint_reg_t trio        : 1;
    uint_reg_t mpipe_quad3 : 1;
    uint_reg_t mpipe_quad2 : 1;
    uint_reg_t mpipe_quad1 : 1;
    uint_reg_t mpipe_quad0 : 1;
    uint_reg_t mpipe_kclk  : 1;
    uint_reg_t mpipe_pclk  : 1;
#endif
  };

  uint_reg_t word;
} RSH_GX36_IO_RESET_t;


/*
 * GPIO Mode Control.
 * Device specific mode control for GPIO pins.  The bit location in this
 * register corresponds to the GPIO pin being used for the associated
 * function.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Reserved. */
    uint_reg_t __reserved_0         : 17;
    /*
     * Enable LED outputs on the associated GPIO pins.  The MSB is LED[1]
     * (typically "link") and the LSB is LED[0] (typically "activity").
     */
    uint_reg_t mpipe0_xaui0_led_ena : 2;
    /* Reserved. */
    uint_reg_t __reserved_1         : 2;
    /*
     * Enable LED outputs on the associated GPIO pins.  The MSB is LED[1]
     * (typically "link") and the LSB is LED[0] (typically "activity").
     */
    uint_reg_t mpipe0_xaui1_led_ena : 2;
    /* Reserved. */
    uint_reg_t __reserved_2         : 2;
    /*
     * Enable LED outputs on the associated GPIO pins.  The MSB is LED[1]
     * (typically "link") and the LSB is LED[0] (typically "activity").
     */
    uint_reg_t mpipe0_xaui2_led_ena : 2;
    /* Reserved. */
    uint_reg_t __reserved_3         : 2;
    /*
     * Enable LED outputs on the associated GPIO pins.  The MSB is LED[1]
     * (typically "link") and the LSB is LED[0] (typically "activity").
     */
    uint_reg_t mpipe0_xaui3_led_ena : 2;
    /* Reserved. */
    uint_reg_t __reserved_4         : 2;
    /*
     * Enable LED outputs on the associated GPIO pins.  The MSB is LED[1]
     * (typically "link") and the LSB is LED[0] (typically "activity").
     */
    uint_reg_t mpipe1_xaui0_led_ena : 2;
    /* Reserved. */
    uint_reg_t __reserved_5         : 2;
    /*
     * Enable LED outputs on the associated GPIO pins.  The MSB is LED[1]
     * (typically "link") and the LSB is LED[0] (typically "activity").
     */
    uint_reg_t mpipe1_xaui1_led_ena : 2;
    /* Reserved. */
    uint_reg_t __reserved_6         : 2;
    /*
     * Enable LED outputs on the associated GPIO pins.  The MSB is LED[1]
     * (typically "link") and the LSB is LED[0] (typically "activity").
     */
    uint_reg_t mpipe1_xaui2_led_ena : 2;
    /* Reserved. */
    uint_reg_t __reserved_7         : 2;
    /*
     * Enable LED outputs on the associated GPIO pins.  The MSB is LED[1]
     * (typically "link") and the LSB is LED[0] (typically "activity").
     */
    uint_reg_t mpipe1_xaui3_led_ena : 2;
    /* Reserved. */
    uint_reg_t __reserved_8         : 17;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_8         : 17;
    uint_reg_t mpipe1_xaui3_led_ena : 2;
    uint_reg_t __reserved_7         : 2;
    uint_reg_t mpipe1_xaui2_led_ena : 2;
    uint_reg_t __reserved_6         : 2;
    uint_reg_t mpipe1_xaui1_led_ena : 2;
    uint_reg_t __reserved_5         : 2;
    uint_reg_t mpipe1_xaui0_led_ena : 2;
    uint_reg_t __reserved_4         : 2;
    uint_reg_t mpipe0_xaui3_led_ena : 2;
    uint_reg_t __reserved_3         : 2;
    uint_reg_t mpipe0_xaui2_led_ena : 2;
    uint_reg_t __reserved_2         : 2;
    uint_reg_t mpipe0_xaui1_led_ena : 2;
    uint_reg_t __reserved_1         : 2;
    uint_reg_t mpipe0_xaui0_led_ena : 2;
    uint_reg_t __reserved_0         : 17;
#endif
  };

  uint_reg_t word;
} RSH_GX72_GPIO_MODE_t;


/*
 * IO Disable-0.
 * Device specific feature disable bits.  These may be set (hardcoded) in
 * hardware or set by software.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Each bit corresponds to an SGMII port.  When set, the port is disabled.
     */
    uint_reg_t mpipe0_sgmii_3_0_disable   : 4;
    /* When set, the associated XAUI port is disabled. */
    uint_reg_t mpipe0_xaui0_disable       : 1;
    /*
     * Each bit corresponds to an SGMII port.  When set, the port is disabled.
     */
    uint_reg_t mpipe0_sgmii_7_4_disable   : 4;
    /* When set, the associated XAUI port is disabled. */
    uint_reg_t mpipe0_xaui1_disable       : 1;
    /*
     * Each bit corresponds to an SGMII port.  When set, the port is disabled.
     */
    uint_reg_t mpipe0_sgmii_11_8_disable  : 4;
    /* When set, the associated XAUI port is disabled. */
    uint_reg_t mpipe0_xaui2_disable       : 1;
    /*
     * Each bit corresponds to an SGMII port.  When set, the port is disabled.
     */
    uint_reg_t mpipe0_sgmii_15_12_disable : 4;
    /* When set, the associated XAUI port is disabled. */
    uint_reg_t mpipe0_xaui3_disable       : 1;
    /*
     * Each bit corresponds to a classification processor.  When set, the
     * processor is disabled.
     */
    uint_reg_t mpipe0_cls_disable         : 10;
    /* When set, the entire MPIPE is disabled. */
    uint_reg_t mpipe0_device_disable      : 1;
    /* Configures width/availability of the PCIe port */
    uint_reg_t trio0_pcie_port0_config    : 2;
    /* Configures width/availability of the PCIe port */
    uint_reg_t trio0_pcie_port1_config    : 2;
    /* Configures width/availability of the PCIe port */
    uint_reg_t trio0_pcie_port2_config    : 2;
    /*
     * Each bit corresponds to a StreamIO port.  When set, the associated
     * port may not be used for StreamIO.
     */
    uint_reg_t trio0_stream_disable       : 3;
    /* When set, the entire TRIO interface is disabled. */
    uint_reg_t trio0_device_disable       : 1;
    /* When set, the associated DDR3 interface is disabled. */
    uint_reg_t ddr3_disable               : 4;
    /* When set, USB Host0 is disabled. */
    uint_reg_t usb_host0_disable          : 1;
    /* When set, USB Host1 is disabled. */
    uint_reg_t usb_host1_disable          : 1;
    /* When set, the USB endpoint is disabled. */
    uint_reg_t usb_ep_disable             : 1;
    /* When set, the entire USB interface is disabled (all ports disabled). */
    uint_reg_t usb_interface_disable      : 1;
    /* When set, the MemCopy engine is disabled. */
    uint_reg_t crypto0_mem_cpy_disable    : 1;
    /* When set, the SNOW and KASUMI crypto engines are disabled. */
    uint_reg_t crypto0_wireless_disable   : 1;
    /*
     * When set, the associated set of packet crypto engines (DES/AES/SHA/MD5
     * etc.) is disabled.
     */
    uint_reg_t crypto0_pkt_disable        : 2;
    /* When set, the public key engine is disabled. */
    uint_reg_t crypto0_pubkey_disable     : 1;
    /* When set, the entire crypto interface is disabled for all engines. */
    uint_reg_t crypto0_interface_disable  : 1;
    /* When set, the MemCopy engine is disabled. */
    uint_reg_t crypto1_mem_cpy_disable    : 1;
    /* When set, the SNOW and KASUMI crypto engines are disabled. */
    uint_reg_t crypto1_wireless_disable   : 1;
    /*
     * When set, the associated set of packet crypto engines (DES/AES/SHA/MD5
     * etc.) is disabled.
     */
    uint_reg_t crypto1_pkt_disable        : 2;
    /* When set, the public key engine is disabled. */
    uint_reg_t crypto1_pubkey_disable     : 1;
    /* When set, the entire crypto interface is disabled for all engines. */
    uint_reg_t crypto1_interface_disable  : 1;
    /* Reserved. */
    uint_reg_t __reserved                 : 3;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved                 : 3;
    uint_reg_t crypto1_interface_disable  : 1;
    uint_reg_t crypto1_pubkey_disable     : 1;
    uint_reg_t crypto1_pkt_disable        : 2;
    uint_reg_t crypto1_wireless_disable   : 1;
    uint_reg_t crypto1_mem_cpy_disable    : 1;
    uint_reg_t crypto0_interface_disable  : 1;
    uint_reg_t crypto0_pubkey_disable     : 1;
    uint_reg_t crypto0_pkt_disable        : 2;
    uint_reg_t crypto0_wireless_disable   : 1;
    uint_reg_t crypto0_mem_cpy_disable    : 1;
    uint_reg_t usb_interface_disable      : 1;
    uint_reg_t usb_ep_disable             : 1;
    uint_reg_t usb_host1_disable          : 1;
    uint_reg_t usb_host0_disable          : 1;
    uint_reg_t ddr3_disable               : 4;
    uint_reg_t trio0_device_disable       : 1;
    uint_reg_t trio0_stream_disable       : 3;
    uint_reg_t trio0_pcie_port2_config    : 2;
    uint_reg_t trio0_pcie_port1_config    : 2;
    uint_reg_t trio0_pcie_port0_config    : 2;
    uint_reg_t mpipe0_device_disable      : 1;
    uint_reg_t mpipe0_cls_disable         : 10;
    uint_reg_t mpipe0_xaui3_disable       : 1;
    uint_reg_t mpipe0_sgmii_15_12_disable : 4;
    uint_reg_t mpipe0_xaui2_disable       : 1;
    uint_reg_t mpipe0_sgmii_11_8_disable  : 4;
    uint_reg_t mpipe0_xaui1_disable       : 1;
    uint_reg_t mpipe0_sgmii_7_4_disable   : 4;
    uint_reg_t mpipe0_xaui0_disable       : 1;
    uint_reg_t mpipe0_sgmii_3_0_disable   : 4;
#endif
  };

  uint_reg_t word;
} RSH_GX72_IO_DISABLE0_t;


/*
 * IO Disable-1.
 * Device specific feature disable bits.  These may be set (hardcoded) in
 * hardware or set by software.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Reserved. */
    uint_reg_t __reserved_0            : 8;
    /* Configures width/availability of the PCIe port */
    uint_reg_t trio1_pcie_port0_config : 2;
    /* Configures width/availability of the PCIe port */
    uint_reg_t trio1_pcie_port1_config : 2;
    /* Configures width/availability of the PCIe port */
    uint_reg_t trio1_pcie_port2_config : 2;
    /*
     * Each bit corresponds to a StreamIO port.  When set, the associated
     * port may not be used for StreamIO.
     */
    uint_reg_t trio1_stream_disable    : 3;
    /* When set, the entire TRIO interface is disabled. */
    uint_reg_t trio1_device_disable    : 1;
    /* Reserved. */
    uint_reg_t __reserved_1            : 46;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1            : 46;
    uint_reg_t trio1_device_disable    : 1;
    uint_reg_t trio1_stream_disable    : 3;
    uint_reg_t trio1_pcie_port2_config : 2;
    uint_reg_t trio1_pcie_port1_config : 2;
    uint_reg_t trio1_pcie_port0_config : 2;
    uint_reg_t __reserved_0            : 8;
#endif
  };

  uint_reg_t word;
} RSH_GX72_IO_DISABLE1_t;


/*
 * IO Disable-2.
 * Device specific feature disable bits.  These may be set (hardcoded) in
 * hardware or set by software.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Each bit corresponds to an SGMII port.  When set, the port is disabled.
     */
    uint_reg_t mpipe1_sgmii_3_0_disable   : 4;
    /* When set, the associated XAUI port is disabled. */
    uint_reg_t mpipe1_xaui0_disable       : 1;
    /*
     * Each bit corresponds to an SGMII port.  When set, the port is disabled.
     */
    uint_reg_t mpipe1_sgmii_7_4_disable   : 4;
    /* When set, the associated XAUI port is disabled. */
    uint_reg_t mpipe1_xaui1_disable       : 1;
    /*
     * Each bit corresponds to an SGMII port.  When set, the port is disabled.
     */
    uint_reg_t mpipe1_sgmii_11_8_disable  : 4;
    /* When set, the associated XAUI port is disabled. */
    uint_reg_t mpipe1_xaui2_disable       : 1;
    /*
     * Each bit corresponds to an SGMII port.  When set, the port is disabled.
     */
    uint_reg_t mpipe1_sgmii_15_12_disable : 4;
    /* When set, the associated XAUI port is disabled. */
    uint_reg_t mpipe1_xaui3_disable       : 1;
    /*
     * Each bit corresponds to a classification processor.  When set, the
     * processor is disabled.
     */
    uint_reg_t mpipe1_cls_disable         : 10;
    /* When set, the entire MPIPE is disabled. */
    uint_reg_t mpipe1_device_disable      : 1;
    /* Reserved. */
    uint_reg_t __reserved                 : 33;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved                 : 33;
    uint_reg_t mpipe1_device_disable      : 1;
    uint_reg_t mpipe1_cls_disable         : 10;
    uint_reg_t mpipe1_xaui3_disable       : 1;
    uint_reg_t mpipe1_sgmii_15_12_disable : 4;
    uint_reg_t mpipe1_xaui2_disable       : 1;
    uint_reg_t mpipe1_sgmii_11_8_disable  : 4;
    uint_reg_t mpipe1_xaui1_disable       : 1;
    uint_reg_t mpipe1_sgmii_7_4_disable   : 4;
    uint_reg_t mpipe1_xaui0_disable       : 1;
    uint_reg_t mpipe1_sgmii_3_0_disable   : 4;
#endif
  };

  uint_reg_t word;
} RSH_GX72_IO_DISABLE2_t;


/*
 * IO Reset.
 * Each bit corresponds to a device or function within a device.  When
 * written with a 1, the associated device or function will be reset.
 * Software is responsible for any quiesce procedures required for the
 * associated device prior to reset.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    uint_reg_t mpipe0_pclk  : 1;
    uint_reg_t mpipe0_kclk  : 1;
    /* Reserved. */
    uint_reg_t __reserved_0 : 1;
    uint_reg_t mpipe0_quad0 : 1;
    uint_reg_t mpipe0_quad1 : 1;
    uint_reg_t mpipe0_quad2 : 1;
    uint_reg_t mpipe0_quad3 : 1;
    uint_reg_t mpipe1_pclk  : 1;
    uint_reg_t mpipe1_kclk  : 1;
    /* Reserved. */
    uint_reg_t __reserved_1 : 1;
    uint_reg_t mpipe1_quad0 : 1;
    uint_reg_t mpipe1_quad1 : 1;
    uint_reg_t mpipe1_quad2 : 1;
    uint_reg_t mpipe1_quad3 : 1;
    uint_reg_t trio0        : 1;
    uint_reg_t pcie0_0      : 1;
    uint_reg_t pcie0_1      : 1;
    uint_reg_t pcie0_2      : 1;
    uint_reg_t trio1        : 1;
    uint_reg_t pcie1_0      : 1;
    uint_reg_t pcie1_1      : 1;
    uint_reg_t pcie1_2      : 1;
    uint_reg_t usb          : 1;
    uint_reg_t msh0         : 1;
    uint_reg_t msh1         : 1;
    uint_reg_t msh2         : 1;
    uint_reg_t msh3         : 1;
    uint_reg_t gpio         : 1;
    uint_reg_t crypto       : 1;
    /* Reserved. */
    uint_reg_t __reserved_2 : 35;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_2 : 35;
    uint_reg_t crypto       : 1;
    uint_reg_t gpio         : 1;
    uint_reg_t msh3         : 1;
    uint_reg_t msh2         : 1;
    uint_reg_t msh1         : 1;
    uint_reg_t msh0         : 1;
    uint_reg_t usb          : 1;
    uint_reg_t pcie1_2      : 1;
    uint_reg_t pcie1_1      : 1;
    uint_reg_t pcie1_0      : 1;
    uint_reg_t trio1        : 1;
    uint_reg_t pcie0_2      : 1;
    uint_reg_t pcie0_1      : 1;
    uint_reg_t pcie0_0      : 1;
    uint_reg_t trio0        : 1;
    uint_reg_t mpipe1_quad3 : 1;
    uint_reg_t mpipe1_quad2 : 1;
    uint_reg_t mpipe1_quad1 : 1;
    uint_reg_t mpipe1_quad0 : 1;
    uint_reg_t __reserved_1 : 1;
    uint_reg_t mpipe1_kclk  : 1;
    uint_reg_t mpipe1_pclk  : 1;
    uint_reg_t mpipe0_quad3 : 1;
    uint_reg_t mpipe0_quad2 : 1;
    uint_reg_t mpipe0_quad1 : 1;
    uint_reg_t mpipe0_quad0 : 1;
    uint_reg_t __reserved_0 : 1;
    uint_reg_t mpipe0_kclk  : 1;
    uint_reg_t mpipe0_pclk  : 1;
#endif
  };

  uint_reg_t word;
} RSH_GX72_IO_RESET_t;


/*
 * IO Disable-0.
 * Device specific feature disable bits.  These may be set (hardcoded) in
 * hardware or set by software.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Each bit corresponds to an SGMII port.  When set, the port is disabled.
     */
    uint_reg_t mpipe_sgmii_3_0_disable   : 4;
    /* When set, the associated XAUI port is disabled. */
    uint_reg_t mpipe_xaui0_disable       : 1;
    /*
     * Each bit corresponds to an SGMII port.  When set, the port is disabled.
     */
    uint_reg_t mpipe_sgmii_7_4_disable   : 4;
    /* When set, the associated XAUI port is disabled. */
    uint_reg_t mpipe_xaui1_disable       : 1;
    /*
     * Each bit corresponds to an SGMII port.  When set, the port is disabled.
     */
    uint_reg_t mpipe_sgmii_11_8_disable  : 4;
    /* When set, the associated XAUI port is disabled. */
    uint_reg_t mpipe_xaui2_disable       : 1;
    /*
     * Each bit corresponds to an SGMII port.  When set, the port is disabled.
     */
    uint_reg_t mpipe_sgmii_15_12_disable : 4;
    /* When set, the associated XAUI port is disabled. */
    uint_reg_t mpipe_xaui3_disable       : 1;
    /*
     * Each bit corresponds to a classification processor.  When set, the
     * processor is disabled.
     */
    uint_reg_t mpipe_cls_disable         : 10;
    /* When set, the entire MPIPE is disabled. */
    uint_reg_t mpipe_device_disable      : 1;
    /* Configures width/availability of the PCIe port */
    uint_reg_t trio_pcie_port0_config    : 2;
    /* Configures width/availability of the PCIe port */
    uint_reg_t trio_pcie_port1_config    : 2;
    /* Configures width/availability of the PCIe port */
    uint_reg_t trio_pcie_port2_config    : 2;
    /*
     * Each bit corresponds to a StreamIO port.  When set, the associated
     * port may not be used for StreamIO.
     */
    uint_reg_t trio_stream_disable       : 3;
    /* When set, the entire TRIO interface is disabled. */
    uint_reg_t trio_device_disable       : 1;
    /* When set, the associated DDR3 interface is disabled. */
    uint_reg_t ddr3_disable              : 2;
    /* When set, USB Host0 is disabled. */
    uint_reg_t usb_host0_disable         : 1;
    /* When set, USB Host1 is disabled. */
    uint_reg_t usb_host1_disable         : 1;
    /* When set, the USB endpoint is disabled. */
    uint_reg_t usb_ep_disable            : 1;
    /* When set, the entire USB interface is disabled (all ports disabled). */
    uint_reg_t usb_interface_disable     : 1;
    /* When set, the MemCopy engine is disabled. */
    uint_reg_t crypto0_mem_cpy_disable   : 1;
    /* When set, the SNOW and KASUMI crypto engines are disabled. */
    uint_reg_t crypto0_wireless_disable  : 1;
    /*
     * When set, the associated set of packet crypto engines (DES/AES/SHA/MD5
     * etc.) is disabled.
     */
    uint_reg_t crypto0_pkt_disable       : 2;
    /* When set, the public key engine is disabled. */
    uint_reg_t crypto0_pubkey_disable    : 1;
    /* When set, the entire crypto interface is disabled for all engines. */
    uint_reg_t crypto0_interface_disable : 1;
    /* When set, the MemCopy engine is disabled. */
    uint_reg_t crypto1_mem_cpy_disable   : 1;
    /* When set, the SNOW and KASUMI crypto engines are disabled. */
    uint_reg_t crypto1_wireless_disable  : 1;
    /*
     * When set, the associated set of packet crypto engines (DES/AES/SHA/MD5
     * etc.) is disabled.
     */
    uint_reg_t crypto1_pkt_disable       : 2;
    /* When set, the public key engine is disabled. */
    uint_reg_t crypto1_pubkey_disable    : 1;
    /* When set, the entire crypto interface is disabled for all engines. */
    uint_reg_t crypto1_interface_disable : 1;
    /* Reserved. */
    uint_reg_t reserved_io               : 5;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t reserved_io               : 5;
    uint_reg_t crypto1_interface_disable : 1;
    uint_reg_t crypto1_pubkey_disable    : 1;
    uint_reg_t crypto1_pkt_disable       : 2;
    uint_reg_t crypto1_wireless_disable  : 1;
    uint_reg_t crypto1_mem_cpy_disable   : 1;
    uint_reg_t crypto0_interface_disable : 1;
    uint_reg_t crypto0_pubkey_disable    : 1;
    uint_reg_t crypto0_pkt_disable       : 2;
    uint_reg_t crypto0_wireless_disable  : 1;
    uint_reg_t crypto0_mem_cpy_disable   : 1;
    uint_reg_t usb_interface_disable     : 1;
    uint_reg_t usb_ep_disable            : 1;
    uint_reg_t usb_host1_disable         : 1;
    uint_reg_t usb_host0_disable         : 1;
    uint_reg_t ddr3_disable              : 2;
    uint_reg_t trio_device_disable       : 1;
    uint_reg_t trio_stream_disable       : 3;
    uint_reg_t trio_pcie_port2_config    : 2;
    uint_reg_t trio_pcie_port1_config    : 2;
    uint_reg_t trio_pcie_port0_config    : 2;
    uint_reg_t mpipe_device_disable      : 1;
    uint_reg_t mpipe_cls_disable         : 10;
    uint_reg_t mpipe_xaui3_disable       : 1;
    uint_reg_t mpipe_sgmii_15_12_disable : 4;
    uint_reg_t mpipe_xaui2_disable       : 1;
    uint_reg_t mpipe_sgmii_11_8_disable  : 4;
    uint_reg_t mpipe_xaui1_disable       : 1;
    uint_reg_t mpipe_sgmii_7_4_disable   : 4;
    uint_reg_t mpipe_xaui0_disable       : 1;
    uint_reg_t mpipe_sgmii_3_0_disable   : 4;
#endif
  };

  uint_reg_t word;
} RSH_GX9_IO_DISABLE0_t;


/*
 * IO Disable-1.
 * Device specific feature disable bits.  These may be set (hardcoded) in
 * hardware or set by software.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* When set, the associated compression engine is disabled. */
    uint_reg_t comp0_disable           : 1;
    /* When set, the associated decompression engine is disabled. */
    uint_reg_t decomp0_disable         : 1;
    /* When set, the associated MEM_CPY (DMA) interface is disabled. */
    uint_reg_t mem_cpy0_disable        : 1;
    /*
     * When set, the entire compression/decompression/MemCPY interface is
     * disabled.
     */
    uint_reg_t comp_interface0_disable : 1;
    /* When set, the associated compression engine is disabled. */
    uint_reg_t comp1_disable           : 1;
    /* When set, the associated decompression engine is disabled. */
    uint_reg_t decomp1_disable         : 1;
    /* When set, the associated MEM_CPY (DMA) interface is disabled. */
    uint_reg_t mem_cpy1_disable        : 1;
    /*
     * When set, the entire compression/decompression/MemCPY interface is
     * disabled.
     */
    uint_reg_t comp_interface1_disable : 1;
    /* Reserved. */
    uint_reg_t reserved_io             : 28;
    /* Reserved. */
    uint_reg_t __reserved              : 28;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved              : 28;
    uint_reg_t reserved_io             : 28;
    uint_reg_t comp_interface1_disable : 1;
    uint_reg_t mem_cpy1_disable        : 1;
    uint_reg_t decomp1_disable         : 1;
    uint_reg_t comp1_disable           : 1;
    uint_reg_t comp_interface0_disable : 1;
    uint_reg_t mem_cpy0_disable        : 1;
    uint_reg_t decomp0_disable         : 1;
    uint_reg_t comp0_disable           : 1;
#endif
  };

  uint_reg_t word;
} RSH_GX9_IO_DISABLE1_t;


/*
 * MMIO HFH Table Init Control.
 * Initialization control for the hash-for-home tables.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Index into the HFH table.  Increments automatically on write or read
     * to HFH_INIT_DAT.
     */
    uint_reg_t idx        : 7;
    /* Reserved. */
    uint_reg_t __reserved : 57;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 57;
    uint_reg_t idx        : 7;
#endif
  };

  uint_reg_t word;
} RSH_HFH_INIT_CTL_t;


/*
 * HFH Table Data.
 * Read/Write data for hash-for-home table
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Fraction field of HFH table.  Determines what portion of the address
     * space maps to TileA vs TileB
     */
    uint_reg_t fract      : 7;
    /*
     * Tile that is selected when the result of the hash_f function is less
     * than FRACT
     */
    uint_reg_t tileb      : 8;
    /*
     * Tile that is selected when the result of the hash_f function is
     * greater than or equal to FRACT
     */
    uint_reg_t tilea      : 8;
    /* Reserved. */
    uint_reg_t __reserved : 41;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 41;
    uint_reg_t tilea      : 8;
    uint_reg_t tileb      : 8;
    uint_reg_t fract      : 7;
#endif
  };

  uint_reg_t word;
} RSH_HFH_INIT_DAT_t;


/*
 * Bindings for interrupt vectors.
 * This register provides read/write access to all of the interrupt bindings
 * for the rshim.  The VEC_SEL field is used to select the vector being
 * configured and the BIND_SEL selects the interrupt within the vector.  To
 * read a binding, first write the VEC_SEL and BIND_SEL fields along with a 1
 * in the NW field.  Then read back the register.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Enable the interrupt.  When 0, the interrupt won't be dispatched,
     * however the STATUS bit will continue to be updated.
     */
    uint_reg_t enable       : 1;
    /*
     * When 1, interrupt will be dispatched each time it occurs.  When 0, the
     * interrupt is only sent if the status bit is clear.
     */
    uint_reg_t mode         : 1;
    /* Tile targeted for this interrupt in {x[3:0],y[3:0]} format. */
    uint_reg_t tileid       : 8;
    /* Interrupt number to be delivered to Tile */
    uint_reg_t int_num      : 2;
    /* Event number to be delivered to Tile */
    uint_reg_t evt_num      : 5;
    /* Reserved. */
    uint_reg_t __reserved_0 : 15;
    /* Selects device whose bindings are to be accessed. */
    uint_reg_t dev_sel      : 4;
    /* Reserved. */
    uint_reg_t __reserved_1 : 4;
    /*
     * Selects binding within the device selected by VEC_SEL.  For all
     * devices except device-0, this field is ignored.
     */
    uint_reg_t bind_sel     : 5;
    /* Reserved. */
    uint_reg_t __reserved_2 : 3;
    /*
     * When written with a 1, the interrupt binding data will not be
     * modified.  Set this when writing the DEV_SEL and BIND_SEL fields in
     * preparation for a read.
     */
    uint_reg_t nw           : 1;
    /* Reserved. */
    uint_reg_t __reserved_3 : 15;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_3 : 15;
    uint_reg_t nw           : 1;
    uint_reg_t __reserved_2 : 3;
    uint_reg_t bind_sel     : 5;
    uint_reg_t __reserved_1 : 4;
    uint_reg_t dev_sel      : 4;
    uint_reg_t __reserved_0 : 15;
    uint_reg_t evt_num      : 5;
    uint_reg_t int_num      : 2;
    uint_reg_t tileid       : 8;
    uint_reg_t mode         : 1;
    uint_reg_t enable       : 1;
#endif
  };

  uint_reg_t word;
} RSH_INT_BIND_t;


/*
 * Interrupt vector-0, read-to-clear.
 * Interrupt status vector with read-to-clear functionality.  Provides access
 * to the same status bits that are visible in INT_VEC0_W1TC.  Reading this
 * register clears all of the associated interrupts.
 *
 * This vector contains the following interrupts:
 *
 * 0: SWINT0
 * 1: SWINT1
 * 2: SWINT2
 * 3: SWINT3
 * 4: DEV_PROT
 * 5: MMIO_ERR
 * 6: DCNT0
 * 7: DCNT1
 * 8: DCNT2
 * 9: TM_HTT_LWM
 * 10: TM_HTT_HWM
 * 11: TM_TTH_LWM
 * 12: TM_TTH_HWM
 * 13: TM_HTT_RERR
 * 14: TM_HTT_WERR
 * 15: TM_TTH_RERR
 * 16: TM_TTH_WERR
 * 17: DIAG_BCST0
 * 18: DIAG_BCST1
 * 19: DIAG_BCST2
 * 20: DIAG_BCST3
 * 21: CFG_PROT_VIOL
 * 22: PWR_ALARM
 * 23: PWR_HIGH
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Software Interrupt */
    uint_reg_t swint0        : 1;
    /* Software Interrupt */
    uint_reg_t swint1        : 1;
    /* Software Interrupt */
    uint_reg_t swint2        : 1;
    /* Software Interrupt */
    uint_reg_t swint3        : 1;
    /*
     * Device Protection Error.  Device or remote command interface attempted
     * access but associated protection bit was set in DEVICE_PROTECTION.
     */
    uint_reg_t dev_prot      : 1;
    /*
     * An MMIO request encountered an error.  Error info captured in
     * MMIO_ERROR_INFO
     */
    uint_reg_t mmio_err      : 1;
    /* Down Counter 0 */
    uint_reg_t dcnt0         : 1;
    /* Down Counter 1 */
    uint_reg_t dcnt1         : 1;
    /* Down Counter 2 */
    uint_reg_t dcnt2         : 1;
    /* TileMonitor HostToTile Low Water Mark */
    uint_reg_t tm_htt_lwm    : 1;
    /* TileMonitor HostToTile High Water Mark */
    uint_reg_t tm_htt_hwm    : 1;
    /* TileMonitor TileToHost Low Water Mark */
    uint_reg_t tm_tth_lwm    : 1;
    /* TileMonitor TileToHost High Water Mark */
    uint_reg_t tm_tth_hwm    : 1;
    /* TileMonitor HostToTile Read-Empty Error */
    uint_reg_t tm_htt_rerr   : 1;
    /* TileMonitor HostToTile Write-Full Error */
    uint_reg_t tm_htt_werr   : 1;
    /* TileMonitor TileToHost Read-Empty Error */
    uint_reg_t tm_tth_rerr   : 1;
    /* TileMonitor TileToHost Write-Full Error */
    uint_reg_t tm_tth_werr   : 1;
    /* Diag Broadcast-0 Asserted */
    uint_reg_t diag_bcst0    : 1;
    /* Diag Broadcast-1 Asserted */
    uint_reg_t diag_bcst1    : 1;
    /* Diag Broadcast-2 Asserted */
    uint_reg_t diag_bcst2    : 1;
    /* Diag Broadcast-3 Asserted */
    uint_reg_t diag_bcst3    : 1;
    /* A config access attempted to exceed its allowed protection level. */
    uint_reg_t cfg_prot_viol : 1;
    /* Reserved. */
    uint_reg_t __reserved    : 42;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved    : 42;
    uint_reg_t cfg_prot_viol : 1;
    uint_reg_t diag_bcst3    : 1;
    uint_reg_t diag_bcst2    : 1;
    uint_reg_t diag_bcst1    : 1;
    uint_reg_t diag_bcst0    : 1;
    uint_reg_t tm_tth_werr   : 1;
    uint_reg_t tm_tth_rerr   : 1;
    uint_reg_t tm_htt_werr   : 1;
    uint_reg_t tm_htt_rerr   : 1;
    uint_reg_t tm_tth_hwm    : 1;
    uint_reg_t tm_tth_lwm    : 1;
    uint_reg_t tm_htt_hwm    : 1;
    uint_reg_t tm_htt_lwm    : 1;
    uint_reg_t dcnt2         : 1;
    uint_reg_t dcnt1         : 1;
    uint_reg_t dcnt0         : 1;
    uint_reg_t mmio_err      : 1;
    uint_reg_t dev_prot      : 1;
    uint_reg_t swint3        : 1;
    uint_reg_t swint2        : 1;
    uint_reg_t swint1        : 1;
    uint_reg_t swint0        : 1;
#endif
  };

  uint_reg_t word;
} RSH_INT_VEC0_RTC_t;


/*
 * Interrupt vector-0, write-one-to-clear.
 * Interrupt status vector with write-one-to-clear functionality.  Provides
 * access to the same status bits that are visible in INT_VEC0_RTC.  Writing
 * a 1 clears the status bit.
 *
 * This vector contains the following interrupts:
 *
 * 0: SWINT0
 * 1: SWINT1
 * 2: SWINT2
 * 3: SWINT3
 * 4: DEV_PROT
 * 5: MMIO_ERR
 * 6: DCNT0
 * 7: DCNT1
 * 8: DCNT2
 * 9: TM_HTT_LWM
 * 10: TM_HTT_HWM
 * 11: TM_TTH_LWM
 * 12: TM_TTH_HWM
 * 13: TM_HTT_RERR
 * 14: TM_HTT_WERR
 * 15: TM_TTH_RERR
 * 16: TM_TTH_WERR
 * 17: DIAG_BCST0
 * 18: DIAG_BCST1
 * 19: DIAG_BCST2
 * 20: DIAG_BCST3
 * 21: CFG_PROT_VIOL
 * 22: PWR_ALARM
 * 23: PWR_HIGH
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Software Interrupt */
    uint_reg_t swint0        : 1;
    /* Software Interrupt */
    uint_reg_t swint1        : 1;
    /* Software Interrupt */
    uint_reg_t swint2        : 1;
    /* Software Interrupt */
    uint_reg_t swint3        : 1;
    /*
     * Device Protection Error.  Device or remote command interface attempted
     * access but associated protection bit was set in DEVICE_PROTECTION.
     */
    uint_reg_t dev_prot      : 1;
    /*
     * An MMIO request encountered an error.  Error info captured in
     * MMIO_ERROR_INFO
     */
    uint_reg_t mmio_err      : 1;
    /* Down Counter 0 */
    uint_reg_t dcnt0         : 1;
    /* Down Counter 1 */
    uint_reg_t dcnt1         : 1;
    /* Down Counter 2 */
    uint_reg_t dcnt2         : 1;
    /* TileMonitor HostToTile Low Water Mark */
    uint_reg_t tm_htt_lwm    : 1;
    /* TileMonitor HostToTile High Water Mark */
    uint_reg_t tm_htt_hwm    : 1;
    /* TileMonitor TileToHost Low Water Mark */
    uint_reg_t tm_tth_lwm    : 1;
    /* TileMonitor TileToHost High Water Mark */
    uint_reg_t tm_tth_hwm    : 1;
    /* TileMonitor HostToTile Read-Empty Error */
    uint_reg_t tm_htt_rerr   : 1;
    /* TileMonitor HostToTile Write-Full Error */
    uint_reg_t tm_htt_werr   : 1;
    /* TileMonitor TileToHost Read-Empty Error */
    uint_reg_t tm_tth_rerr   : 1;
    /* TileMonitor TileToHost Write-Full Error */
    uint_reg_t tm_tth_werr   : 1;
    /* Diag Broadcast-0 Asserted */
    uint_reg_t diag_bcst0    : 1;
    /* Diag Broadcast-1 Asserted */
    uint_reg_t diag_bcst1    : 1;
    /* Diag Broadcast-2 Asserted */
    uint_reg_t diag_bcst2    : 1;
    /* Diag Broadcast-3 Asserted */
    uint_reg_t diag_bcst3    : 1;
    /* A config access attempted to exceed its allowed protection level. */
    uint_reg_t cfg_prot_viol : 1;
    /* Reserved. */
    uint_reg_t __reserved    : 42;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved    : 42;
    uint_reg_t cfg_prot_viol : 1;
    uint_reg_t diag_bcst3    : 1;
    uint_reg_t diag_bcst2    : 1;
    uint_reg_t diag_bcst1    : 1;
    uint_reg_t diag_bcst0    : 1;
    uint_reg_t tm_tth_werr   : 1;
    uint_reg_t tm_tth_rerr   : 1;
    uint_reg_t tm_htt_werr   : 1;
    uint_reg_t tm_htt_rerr   : 1;
    uint_reg_t tm_tth_hwm    : 1;
    uint_reg_t tm_tth_lwm    : 1;
    uint_reg_t tm_htt_hwm    : 1;
    uint_reg_t tm_htt_lwm    : 1;
    uint_reg_t dcnt2         : 1;
    uint_reg_t dcnt1         : 1;
    uint_reg_t dcnt0         : 1;
    uint_reg_t mmio_err      : 1;
    uint_reg_t dev_prot      : 1;
    uint_reg_t swint3        : 1;
    uint_reg_t swint2        : 1;
    uint_reg_t swint1        : 1;
    uint_reg_t swint0        : 1;
#endif
  };

  uint_reg_t word;
} RSH_INT_VEC0_W1TC_t;


/*
 * IO Reset.
 * Each bit corresponds to a device or function within a device.  When
 * written with a 1, the associated device or function will be reset.
 * Software is responsible for any quiesce procedures required for the
 * associated device prior to reset.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Bits in this vector are chip specific (see the definitions for
     * GX**_IO_RESET).
     */
    uint_reg_t val : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t val : 64;
#endif
  };

  uint_reg_t word;
} RSH_IO_RESET_t;


/*
 * IPI Location.
 * Indicates the location(s) of IPI controllers in the fabric. When set to
 * 0xff, there is no associated IPI controller.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Y-Location of IPI Controller 0. */
    uint_reg_t y0 : 4;
    /* X-Location of IPI Controller 0. */
    uint_reg_t x0 : 4;
    /* Y-Location of IPI Controller 1. */
    uint_reg_t y1 : 4;
    /* X-Location of IPI Controller 1. */
    uint_reg_t x1 : 4;
    /* Y-Location of IPI Controller 2. */
    uint_reg_t y2 : 4;
    /* X-Location of IPI Controller 2. */
    uint_reg_t x2 : 4;
    /* Y-Location of IPI Controller 3. */
    uint_reg_t y3 : 4;
    /* X-Location of IPI Controller 3. */
    uint_reg_t x3 : 4;
    /* Y-Location of IPI Controller 4. */
    uint_reg_t y4 : 4;
    /* X-Location of IPI Controller 4. */
    uint_reg_t x4 : 4;
    /* Y-Location of IPI Controller 5. */
    uint_reg_t y5 : 4;
    /* X-Location of IPI Controller 5. */
    uint_reg_t x5 : 4;
    /* Y-Location of IPI Controller 6. */
    uint_reg_t y6 : 4;
    /* X-Location of IPI Controller 6. */
    uint_reg_t x6 : 4;
    /* Y-Location of IPI Controller 7. */
    uint_reg_t y7 : 4;
    /* X-Location of IPI Controller 7. */
    uint_reg_t x7 : 4;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t x7 : 4;
    uint_reg_t y7 : 4;
    uint_reg_t x6 : 4;
    uint_reg_t y6 : 4;
    uint_reg_t x5 : 4;
    uint_reg_t y5 : 4;
    uint_reg_t x4 : 4;
    uint_reg_t y4 : 4;
    uint_reg_t x3 : 4;
    uint_reg_t y3 : 4;
    uint_reg_t x2 : 4;
    uint_reg_t y2 : 4;
    uint_reg_t x1 : 4;
    uint_reg_t y1 : 4;
    uint_reg_t x0 : 4;
    uint_reg_t y0 : 4;
#endif
  };

  uint_reg_t word;
} RSH_IPI_LOC_t;


/*
 * JTAG Control.
 * JTAG Control.  When this register is written, a JTAG transaction will be
 * initiated as long as JTAG_ENA is asserted.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Shift count for reads/writes.  Decremented by hardware as data is
     * shifted into or out of JTAG interface.  Valid values are 0-64.
     */
    uint_reg_t jtag_shift_cnt : 7;
    /*
     * Determines JTAG operation to be performed.  All JTAG operations must
     * end with END_SHIFT which causes the JTAG data register to be updated
     * with the shifted-in data.  The END_SHIFT can contain 0 or more bits.
     * Transactions may contain zero or more START_SHIFT/CONTINUE_SHIFT
     * operations (both are treated the same).
     */
    uint_reg_t jtag_cmd       : 2;
    /* Reserved. */
    uint_reg_t __reserved     : 55;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved     : 55;
    uint_reg_t jtag_cmd       : 2;
    uint_reg_t jtag_shift_cnt : 7;
#endif
  };

  uint_reg_t word;
} RSH_JTAG_CONTROL_t;


/*
 * JTAG Data.
 * JTAG shift Data.  SW provides up to 64 bits of data to be shifted into
 * JTAG controller (bits are shifted out of LSBs). Hardware shifts data into
 * the MSBs.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * JTAG Data.
     * JTAG shift Data.  SW provides up to 64 bits of data to be shifted into
     * JTAG controller (bits are shifted out of LSBs). Hardware shifts data
     * into the MSBs.
     */
    uint_reg_t jtag_data : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t jtag_data : 64;
#endif
  };

  uint_reg_t word;
} RSH_JTAG_DATA_t;


/* JTAG Setup. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Reflects current state of the rjc/trst pins.  When zero, the rshim
     * will not be able to control the JTAG interface.
     */
    uint_reg_t jtag_ena     : 1;
    /* Clock rate to use for rshim-based JTAG interface. */
    uint_reg_t jtag_rate    : 2;
    /*
     * Use decoded instruction.  When 0, the low 9 bits of the JTAG_INST
     * field are decoded.  When 1, the entire JTAG_INST field is used as the
     * JTAG_IR.
     */
    uint_reg_t dec_inst     : 1;
    /* Reserved. */
    uint_reg_t __reserved_0 : 12;
    /* Selects the custom JTAG instruction number to be accessed. */
    uint_reg_t jtag_inst    : 28;
    /* Reserved. */
    uint_reg_t __reserved_1 : 20;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 20;
    uint_reg_t jtag_inst    : 28;
    uint_reg_t __reserved_0 : 12;
    uint_reg_t dec_inst     : 1;
    uint_reg_t jtag_rate    : 2;
    uint_reg_t jtag_ena     : 1;
#endif
  };

  uint_reg_t word;
} RSH_JTAG_SETUP_t;


/*
 * Memory Info.
 * This register provides information about memory setup required for this
 * device.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Each bit represents a request (QDN) port on the tile fabric.  Bit[15]
     * represents the QDN port used for device configuration and is always
     * set for devices implemented on channel-0.  Bit[14] is the first port
     * clockwise from the port used to access configuration space.  Bit[13]
     * is the second port in the clockwise direction.  Bit[16] is the first
     * port counter-clockwise from the port used to access configuration
     * space.  When a bit is set, the device has a QDN port at the associated
     * location.  For devices using a nonzero channel, this register may
     * return all zeros.
     */
    uint_reg_t req_ports    : 32;
    /*
     * Number of hash-for-home tables that must be configured for this
     * channel.
     */
    uint_reg_t num_hfh_tbl  : 4;
    /* Reserved. */
    uint_reg_t __reserved_0 : 4;
    /*
     * Number of ASIDS supported if this device has an IO TLB (otherwise this
     * field is zero).
     */
    uint_reg_t num_asids    : 8;
    /* Number of IO TLB entries per ASID. */
    uint_reg_t num_tlb_ent  : 8;
    /* Reserved. */
    uint_reg_t __reserved_1 : 8;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 8;
    uint_reg_t num_tlb_ent  : 8;
    uint_reg_t num_asids    : 8;
    uint_reg_t __reserved_0 : 4;
    uint_reg_t num_hfh_tbl  : 4;
    uint_reg_t req_ports    : 32;
#endif
  };

  uint_reg_t word;
} RSH_MEM_INFO_t;


/*
 * MMIO Error Information.
 * Provides diagnostics information when an MMIO error occurs.  Captured
 * whenever the MMIO_ERR interrupt condition occurs (typically due to size
 * error).
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Source Tile in {x[3:0],y[3:0]} format. */
    uint_reg_t src        : 8;
    /* Request size.  0=1B, 1=2B, 2=4B, 3=8B, 4=16B, 5=32B, 6=48B, 7=64B. */
    uint_reg_t size       : 4;
    /* Full PA from request. */
    uint_reg_t pa         : 42;
    /*
     * Opcode of request.  MMIO supports only MMIO_READ (0x0e) and MMIO_WRITE
     * (0x0f).  All others are reserved and will only occur on a
     * misconfigured TLB.
     */
    uint_reg_t opc        : 5;
    /* Reserved. */
    uint_reg_t __reserved : 5;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 5;
    uint_reg_t opc        : 5;
    uint_reg_t pa         : 42;
    uint_reg_t size       : 4;
    uint_reg_t src        : 8;
#endif
  };

  uint_reg_t word;
} RSH_MMIO_ERROR_INFO_t;


/*
 * MMIO Info.
 * This register provides information about how the physical address is
 * interpreted by the IO device.  The PA is divided into
 * {CHANNEL,SVC_DOM,IGNORED,REGION,OFFSET}.  The values in this register
 * define the size of each of these fields.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Number of bits of channel specifier for all channels.   The MSBs of
     * the physical addrress are interpreted as {channel, service-domain}
     */
    uint_reg_t ch_width      : 4;
    /*
     * Each configuration channel is mapped to a device having its own config
     * registers.
     */
    uint_reg_t num_ch        : 15;
    /*
     * Number of bits of service-domain specifier for this channel.   The
     * MSBs of the physical addrress are interpreted as {channel,
     * service-domain}
     */
    uint_reg_t svc_dom_width : 3;
    /* Number of service domains associated with this channel. */
    uint_reg_t num_svc_dom   : 7;
    /*
     * Size of the OFFSET field for this channel.  The LSBs of the physical
     * address are interpreted as {REGION,OFFSET}
     */
    uint_reg_t offset_width  : 6;
    /*
     * Size of the REGION field for this channel.  The LSBs of the physical
     * address are interpreted as {REGION,OFFSET}
     */
    uint_reg_t region_width  : 6;
    /* Reserved. */
    uint_reg_t __reserved    : 23;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved    : 23;
    uint_reg_t region_width  : 6;
    uint_reg_t offset_width  : 6;
    uint_reg_t num_svc_dom   : 7;
    uint_reg_t svc_dom_width : 3;
    uint_reg_t num_ch        : 15;
    uint_reg_t ch_width      : 4;
#endif
  };

  uint_reg_t word;
} RSH_MMIO_INFO_t;


/*
 * Packet Generator Completion Data.
 * Any data received on the RDN will be placed into this data buffer.  The
 * first COMP_DATA register contains the RDN header (but no data bytes).  The
 * remaining 8 data registers contain any data from the RDN packet.Any data
 * received on the RDN will be placed into this data buffer.  The first
 * COMP_DATA register contains the RDN header.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Completion data word. */
    uint_reg_t data : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t data : 64;
#endif
  };

  uint_reg_t word;
} RSH_PG_COMP_DATA_t;


/*
 * Packet Generator Completion Data.
 * Any data received on the RDN will be placed into this data buffer.  The
 * first COMP_DATA register contains the RDN header (but no data bytes).  The
 * remaining 8 data registers contain any data from the RDN packet.Any data
 * received on the RDN will be placed into this data buffer.  The first
 * COMP_DATA register contains the RDN header.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Completion data word. */
    uint_reg_t data : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t data : 64;
#endif
  };

  uint_reg_t word;
} RSH_PG_COMP_DATA_1_t;


/*
 * Packet Generator Completion Data.
 * Any data received on the RDN will be placed into this data buffer.  The
 * first COMP_DATA register contains the RDN header (but no data bytes).  The
 * remaining 8 data registers contain any data from the RDN packet.Any data
 * received on the RDN will be placed into this data buffer.  The first
 * COMP_DATA register contains the RDN header.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Completion data word. */
    uint_reg_t data : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t data : 64;
#endif
  };

  uint_reg_t word;
} RSH_PG_COMP_DATA_2_t;


/*
 * Packet Generator Completion Data.
 * Any data received on the RDN will be placed into this data buffer.  The
 * first COMP_DATA register contains the RDN header (but no data bytes).  The
 * remaining 8 data registers contain any data from the RDN packet.Any data
 * received on the RDN will be placed into this data buffer.  The first
 * COMP_DATA register contains the RDN header.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Completion data word. */
    uint_reg_t data : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t data : 64;
#endif
  };

  uint_reg_t word;
} RSH_PG_COMP_DATA_3_t;


/*
 * Packet Generator Completion Data.
 * Any data received on the RDN will be placed into this data buffer.  The
 * first COMP_DATA register contains the RDN header (but no data bytes).  The
 * remaining 8 data registers contain any data from the RDN packet.Any data
 * received on the RDN will be placed into this data buffer.  The first
 * COMP_DATA register contains the RDN header.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Completion data word. */
    uint_reg_t data : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t data : 64;
#endif
  };

  uint_reg_t word;
} RSH_PG_COMP_DATA_4_t;


/*
 * Packet Generator Completion Data.
 * Any data received on the RDN will be placed into this data buffer.  The
 * first COMP_DATA register contains the RDN header (but no data bytes).  The
 * remaining 8 data registers contain any data from the RDN packet.Any data
 * received on the RDN will be placed into this data buffer.  The first
 * COMP_DATA register contains the RDN header.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Completion data word. */
    uint_reg_t data : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t data : 64;
#endif
  };

  uint_reg_t word;
} RSH_PG_COMP_DATA_5_t;


/*
 * Packet Generator Completion Data.
 * Any data received on the RDN will be placed into this data buffer.  The
 * first COMP_DATA register contains the RDN header (but no data bytes).  The
 * remaining 8 data registers contain any data from the RDN packet.Any data
 * received on the RDN will be placed into this data buffer.  The first
 * COMP_DATA register contains the RDN header.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Completion data word. */
    uint_reg_t data : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t data : 64;
#endif
  };

  uint_reg_t word;
} RSH_PG_COMP_DATA_6_t;


/*
 * Packet Generator Completion Data.
 * Any data received on the RDN will be placed into this data buffer.  The
 * first COMP_DATA register contains the RDN header (but no data bytes).  The
 * remaining 8 data registers contain any data from the RDN packet.Any data
 * received on the RDN will be placed into this data buffer.  The first
 * COMP_DATA register contains the RDN header.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Completion data word. */
    uint_reg_t data : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t data : 64;
#endif
  };

  uint_reg_t word;
} RSH_PG_COMP_DATA_7_t;


/*
 * Packet Generator Completion Data.
 * Any data received on the RDN will be placed into this data buffer.  The
 * first COMP_DATA register contains the RDN header (but no data bytes).  The
 * remaining 8 data registers contain any data from the RDN packet.Any data
 * received on the RDN will be placed into this data buffer.  The first
 * COMP_DATA register contains the RDN header.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Completion data word. */
    uint_reg_t data : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t data : 64;
#endif
  };

  uint_reg_t word;
} RSH_PG_COMP_DATA_8_t;


/*
 * Packet Generator Completion Status.
 * This register contains status information for completions that have been
 * captured from the RDN.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Number of completion packets received.  This counter saturates at
     * 65535 but may be cleared by software.
     */
    uint_reg_t count      : 16;
    /* Number of flits received in the last completion packet on the RDN. */
    uint_reg_t size       : 3;
    /* Reserved. */
    uint_reg_t __reserved : 45;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 45;
    uint_reg_t size       : 3;
    uint_reg_t count      : 16;
#endif
  };

  uint_reg_t word;
} RSH_PG_COMP_STS_t;


/*
 * Packet Generator Control.
 * Configuration of the packet generator
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * When written with a 1, the packet in the PG_DATA buffer will be sent.
     * This bit will read as 1 if there is a send in progress or if there is
     * data in the boot FIFO.  Sending a packet while a send is in progress
     * has unpredictable results.
     */
    uint_reg_t send         : 1;
    /* Selects target network for packet */
    uint_reg_t nw_sel       : 3;
    /*
     * Number of 64-bit words to be sent including the route header.  The
     * maximum legal value is 10.  For the S/RDN, two words will be consumed
     * for each flit that is sent.  Hence for R/SDN, this value MUST be even.
     *  For U/IDN, the first DATA word is treated as the header flit and each
     * subsequent data word will result in two 32-bit flits on the network.
     * So setting this register to 3 will result in the 1st word from the
     * data buffer being sent as the route header and the subsequent two
     * words will result in 4 additional flits on the U/IDN.
     */
    uint_reg_t pkt_words    : 4;
    /*
     * When the DATA_IDX reaches PKT_WORDS in AUTO_SEND mode, the DATA_IDX
     * will be reset to START_IDX.  This allows the first START_IDX flits to
     * remain intact for each packet.  START_IDX must be even for S/RDN
     * packets.
     */
    uint_reg_t start_idx    : 4;
    /*
     * When 1, the packet will automatically send when the data register is
     * written and DATA_IDX is (PKT_WORDS-1).  When AUTO_SEND is 1, the 4kB
     * boot FIFO will be used to stream data into the packet generator.  The
     * AUTO_SEND bit must not be changed unless the SEND bit is clear
     * indicating all previous transactions have completed.
     */
    uint_reg_t auto_send    : 1;
    /*
     * When 1, the packet's route header will be auto-generated based on the
     * low 8 bits of data word 0.  The bits are interpreted as
     * {TILE_X[3:0],TILE_Y[3:0]}.  When 0, the first data word contains the
     * actual route header.
     */
    uint_reg_t auto_rte     : 1;
    /*
     * When 1, packets sent on the SDN will calculate the home tile based on
     * the address field in the packet using the hash-for-home table.  In HFH
     * mode, the MASK field for the lookup comes from bits[3:0] of the first
     * data word.  The OFFSET comes from bits[7:4] of the first data word.
     * HFH will only be applied on SDN packets and only when AUTO_RTE is also
     * 1.
     */
    uint_reg_t hfh          : 1;
    /*
     * When 1, packets use X_FIRST route order.  Only applies when AUTO_RTE
     * is 1.
     */
    uint_reg_t x_first      : 1;
    /* Reserved. */
    uint_reg_t __reserved_0 : 16;
    /*
     * Data word to be read/written.  The maximum allowed value is 10.
     * Increments automatically on each access to PG_DATA. When incremented
     * to PKT_WORDS, DATA_IDX wraps back to START_IDX.
     *
     * Note that R/SDN packets each use two data words per packet flit.  16
     * bits from the upper word are unused on RDN packets.
     */
    uint_reg_t data_idx     : 4;
    /* Reserved. */
    uint_reg_t __reserved_1 : 12;
    /*
     * Count of number of words successfully sent from the packet generator.
     * This is typically used by external devices to flow control the boot
     * stream and prevent blocking between boot writes and debug accesses. In
     * FIFO_MODE, this represents the number of words that have been read
     * (dequeued) from the FIFO.
     */
    uint_reg_t sent_count   : 16;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t sent_count   : 16;
    uint_reg_t __reserved_1 : 12;
    uint_reg_t data_idx     : 4;
    uint_reg_t __reserved_0 : 16;
    uint_reg_t x_first      : 1;
    uint_reg_t hfh          : 1;
    uint_reg_t auto_rte     : 1;
    uint_reg_t auto_send    : 1;
    uint_reg_t start_idx    : 4;
    uint_reg_t pkt_words    : 4;
    uint_reg_t nw_sel       : 3;
    uint_reg_t send         : 1;
#endif
  };

  uint_reg_t word;
} RSH_PG_CTL_t;


/*
 * Packet Generator Data.
 * 10 Data words for packet generator.  The data word to be read or written
 * is indexed by PG_CTL.DATA_IDX.  The DATA_IDX is incremented automatically
 * on each access.  Note that R/SDN packets each use two data words per
 * packet flit. If either the PG_CTL.AUTO_SEND or the PG_CTL.FIFO_MODE is 1,
 * writes to PG_DATA go into the boot FIFO.  Similarly, if the
 * PG_CTL.FIFO_MODE is 1, reads to PG_DATA pop one word from the FIFO. In
 * this case, if the FIFO is empty, the read will return all 1's and the FIFO
 * state will not be affected.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Packet Generator Data.
     * 10 Data words for packet generator.  The data word to be read or
     * written is indexed by PG_CTL.DATA_IDX.  The DATA_IDX is incremented
     * automatically on each access.  Note that R/SDN packets each use two
     * data words per packet flit. If either the PG_CTL.AUTO_SEND or the
     * PG_CTL.FIFO_MODE is 1, writes to PG_DATA go into the boot FIFO.
     * Similarly, if the PG_CTL.FIFO_MODE is 1, reads to PG_DATA pop one word
     * from the FIFO. In this case, if the FIFO is empty, the read will
     * return all 1's and the FIFO state will not be affected.
     */
    uint_reg_t pg_data : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t pg_data : 64;
#endif
  };

  uint_reg_t word;
} RSH_PG_DATA_t;


/*
 * Pseudo-Rand.
 * Provides a pseudo-random 64-bit value in the range of [1..(2**64)-1] using
 * a 64-bit LFSR (taps are 64,63,61,60).
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Pseudo-Rand.
     * Provides a pseudo-random 64-bit value in the range of [1..(2**64)-1]
     * using a 64-bit LFSR (taps are 64,63,61,60).
     */
    uint_reg_t pseudo_rand : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t pseudo_rand : 64;
#endif
  };

  uint_reg_t word;
} RSH_PSEUDO_RAND_t;


/* Pseudo-Rand Control. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * When 1, the RSHIM_P_RAND value advances once each time it is read.
     * When 0, it is free running.
     */
    uint_reg_t p_rand_mode : 1;
    /* Reserved. */
    uint_reg_t __reserved  : 63;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved  : 63;
    uint_reg_t p_rand_mode : 1;
#endif
  };

  uint_reg_t word;
} RSH_PSEUDO_RAND_CONTROL_t;


/* Reset Control. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * When written with the KEY, the chip will be soft reset.  On soft
     * reset, the BREADCRUMB registers will be left intact allowing reboot
     * software to determine why the chip was reset.  Using a KEY reduces the
     * likelihood that a misconfigured boot ROM or other errant write could
     * accidentally reset the chip.
     */
    uint_reg_t reset_chip : 32;
    /* Reserved. */
    uint_reg_t __reserved : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 32;
    uint_reg_t reset_chip : 32;
#endif
  };

  uint_reg_t word;
} RSH_RESET_CONTROL_t;


/*
 * Reset MASK.
 * Each bit corresponds to a device or function within a device.  The bit
 * assignments match the IO_RESET register.   Any devices or functions
 * corresponding to bits that are set will NOT be reset when RESET_CONTROL is
 * used to issue a software reset.   Software must insure that any associated
 * devices have been properly quiesced prior to issuing a RESET_CHIP with any
 * RESET_MASK bits set.  This register is reset by hard_rst_l but not
 * software reset.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Reset MASK.
     * Each bit corresponds to a device or function within a device.  The bit
     * assignments match the IO_RESET register.   Any devices or functions
     * corresponding to bits that are set will NOT be reset when
     * RESET_CONTROL is used to issue a software reset.   Software must
     * insure that any associated devices have been properly quiesced prior
     * to issuing a RESET_CHIP with any RESET_MASK bits set.  This register
     * is reset by hard_rst_l but not software reset.
     */
    uint_reg_t reset_mask : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t reset_mask : 64;
#endif
  };

  uint_reg_t word;
} RSH_RESET_MASK_t;


/*
 * Rev ID.
 * Tile Revision
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Provides architectural revision of the tile. */
    uint_reg_t tile_rev_id : 8;
    /*
     * Chip revision.  The low 4 bits indicate the minor revision; the high 4
     * bits indicate the major revision.
     */
    uint_reg_t chip_rev_id : 8;
    /* Reserved. */
    uint_reg_t __reserved  : 48;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved  : 48;
    uint_reg_t chip_rev_id : 8;
    uint_reg_t tile_rev_id : 8;
#endif
  };

  uint_reg_t word;
} RSH_REV_ID_t;


/* Scratchpad. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Scratchpad. */
    uint_reg_t scratchpad : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t scratchpad : 64;
#endif
  };

  uint_reg_t word;
} RSH_SCRATCHPAD_t;


/*
 * Scratch buffer control.
 * Controls R/W access to scratch buffer.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Index into the 128 word scratch buffer.   Increments automatically on
     * write or read to SCRATCH_BUF_DAT.
     */
    uint_reg_t idx        : 7;
    /* Reserved. */
    uint_reg_t __reserved : 57;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 57;
    uint_reg_t idx        : 7;
#endif
  };

  uint_reg_t word;
} RSH_SCRATCH_BUF_CTL_t;


/*
 * Scratch buffer data.
 * Read/Write data for scratch buffer
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Scratch buffer data.
     * Read/Write data for scratch buffer
     */
    uint_reg_t scratch_buf_dat : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t scratch_buf_dat : 64;
#endif
  };

  uint_reg_t word;
} RSH_SCRATCH_BUF_DAT_t;


/*
 * Semaphore0.
 * Semaphore
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * When read, the current semaphore value is returned and the semaphore
     * is set to 1.  Bit can also be written to 1 or 0.
     */
    uint_reg_t val        : 1;
    /* Reserved. */
    uint_reg_t __reserved : 63;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 63;
    uint_reg_t val        : 1;
#endif
  };

  uint_reg_t word;
} RSH_SEMAPHORE0_t;


/*
 * Semaphore1.
 * Semaphore
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * When read, the current semaphore value is returned and the semaphore
     * is set to 1.  Bit can also be written to 1 or 0.
     */
    uint_reg_t val        : 1;
    /* Reserved. */
    uint_reg_t __reserved : 63;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 63;
    uint_reg_t val        : 1;
#endif
  };

  uint_reg_t word;
} RSH_SEMAPHORE1_t;


/*
 * Software Interrupt.
 * Used to trigger interrupts via register write.  Each bit corresponds to a
 * single SWINT interrupt which will be triggered when written with a 1.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Software Interrupt.
     * Used to trigger interrupts via register write.  Each bit corresponds
     * to a single SWINT interrupt which will be triggered when written with
     * a 1.
     */
    uint_reg_t swint      : 4;
    /* Reserved. */
    uint_reg_t __reserved : 60;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 60;
    uint_reg_t swint      : 4;
#endif
  };

  uint_reg_t word;
} RSH_SWINT_t;


/*
 * Tile Disable.
 * Disable individual Tiles. When set, the associated Tile is disabled.
 * There is one register per column.  Not all devices implement all possible
 * rows and columns.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* When 1, the associated Tile is disabled. */
    uint_reg_t disable    : 10;
    /* Reserved. */
    uint_reg_t __reserved : 54;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 54;
    uint_reg_t disable    : 10;
#endif
  };

  uint_reg_t word;
} RSH_TILE_COL_DISABLE_t;


/*
 * Tile Disable.
 * Disable individual Tiles. When set, the associated Tile is disabled.
 * There is one register per column.  Not all devices implement all possible
 * rows and columns.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* When 1, the associated Tile is disabled. */
    uint_reg_t disable    : 10;
    /* Reserved. */
    uint_reg_t __reserved : 54;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 54;
    uint_reg_t disable    : 10;
#endif
  };

  uint_reg_t word;
} RSH_TILE_COL_DISABLE_1_t;


/*
 * Tile Disable.
 * Disable individual Tiles. When set, the associated Tile is disabled.
 * There is one register per column.  Not all devices implement all possible
 * rows and columns.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* When 1, the associated Tile is disabled. */
    uint_reg_t disable    : 10;
    /* Reserved. */
    uint_reg_t __reserved : 54;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 54;
    uint_reg_t disable    : 10;
#endif
  };

  uint_reg_t word;
} RSH_TILE_COL_DISABLE_2_t;


/*
 * Tile Disable.
 * Disable individual Tiles. When set, the associated Tile is disabled.
 * There is one register per column.  Not all devices implement all possible
 * rows and columns.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* When 1, the associated Tile is disabled. */
    uint_reg_t disable    : 10;
    /* Reserved. */
    uint_reg_t __reserved : 54;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 54;
    uint_reg_t disable    : 10;
#endif
  };

  uint_reg_t word;
} RSH_TILE_COL_DISABLE_3_t;


/*
 * Tile Disable.
 * Disable individual Tiles. When set, the associated Tile is disabled.
 * There is one register per column.  Not all devices implement all possible
 * rows and columns.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* When 1, the associated Tile is disabled. */
    uint_reg_t disable    : 10;
    /* Reserved. */
    uint_reg_t __reserved : 54;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 54;
    uint_reg_t disable    : 10;
#endif
  };

  uint_reg_t word;
} RSH_TILE_COL_DISABLE_4_t;


/*
 * Tile Disable.
 * Disable individual Tiles. When set, the associated Tile is disabled.
 * There is one register per column.  Not all devices implement all possible
 * rows and columns.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* When 1, the associated Tile is disabled. */
    uint_reg_t disable    : 10;
    /* Reserved. */
    uint_reg_t __reserved : 54;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 54;
    uint_reg_t disable    : 10;
#endif
  };

  uint_reg_t word;
} RSH_TILE_COL_DISABLE_5_t;


/*
 * Tile Disable.
 * Disable individual Tiles. When set, the associated Tile is disabled.
 * There is one register per column.  Not all devices implement all possible
 * rows and columns.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* When 1, the associated Tile is disabled. */
    uint_reg_t disable    : 10;
    /* Reserved. */
    uint_reg_t __reserved : 54;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 54;
    uint_reg_t disable    : 10;
#endif
  };

  uint_reg_t word;
} RSH_TILE_COL_DISABLE_6_t;


/*
 * Tile Disable.
 * Disable individual Tiles. When set, the associated Tile is disabled.
 * There is one register per column.  Not all devices implement all possible
 * rows and columns.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* When 1, the associated Tile is disabled. */
    uint_reg_t disable    : 10;
    /* Reserved. */
    uint_reg_t __reserved : 54;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 54;
    uint_reg_t disable    : 10;
#endif
  };

  uint_reg_t word;
} RSH_TILE_COL_DISABLE_7_t;


/*
 * Tile Disable.
 * Disable individual Tiles. When set, the associated Tile is disabled.
 * There is one register per column.  Not all devices implement all possible
 * rows and columns.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* When 1, the associated Tile is disabled. */
    uint_reg_t disable    : 10;
    /* Reserved. */
    uint_reg_t __reserved : 54;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 54;
    uint_reg_t disable    : 10;
#endif
  };

  uint_reg_t word;
} RSH_TILE_COL_DISABLE_8_t;


/*
 * Tile Disable.
 * Disable individual Tiles. When set, the associated Tile is disabled.
 * There is one register per column.  Not all devices implement all possible
 * rows and columns.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* When 1, the associated Tile is disabled. */
    uint_reg_t disable    : 10;
    /* Reserved. */
    uint_reg_t __reserved : 54;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 54;
    uint_reg_t disable    : 10;
#endif
  };

  uint_reg_t word;
} RSH_TILE_COL_DISABLE_9_t;


/*
 * TileMonitor Host to Tile Control.
 * Provides control over FIFO interrupts.  Note that the HWM/LWM interrupts
 * trigger continuously as long as the FIFO is is at or above/below the
 * associated water mark.  Thus these interrupts are typically used in
 * INT_BIND.MODE=0 and should be cleared by SW after the condition has been
 * handled (FIFO filled/drained as appropriate).
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Level at which to trigger the low water mark interrupt. */
    uint_reg_t lwm          : 8;
    /* Level at which to trigger the high water mark interrupt. */
    uint_reg_t hwm          : 8;
    /* Reserved. */
    uint_reg_t __reserved_0 : 16;
    /* Provides size of the HostToTile FIFO in terms of 8-byte entries. */
    uint_reg_t max_entries  : 9;
    /* Reserved. */
    uint_reg_t __reserved_1 : 23;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 23;
    uint_reg_t max_entries  : 9;
    uint_reg_t __reserved_0 : 16;
    uint_reg_t hwm          : 8;
    uint_reg_t lwm          : 8;
#endif
  };

  uint_reg_t word;
} RSH_TM_HOST_TO_TILE_CTL_t;


/*
 * TileMonitor Host to Tile Data.
 * Provides read/write access to the HostToTile FIFO.  When written, the
 * TileMonitor HostToTile FIFO will be written with the associated data and
 * the WPTR will be advanced.  When read, the entry at the head of the FIFO
 * will be returned and the RPTR will be advanced.
 *
 * When empty, reads will have no effect.  Behavior on an empty read is
 * device dependent.  Tile software will receive all 1's on an empty read.
 *
 * When full, writes will be ignored.  Behavior on a full write is device
 * dependent.
 *
 * For Gx9/16/36/72 devices, this FIFO may only be used when PG.AUTO_SEND is
 * zero (packet generator not in boot mode).
 *
 * The FIFO may be drained by reading TM_HOST_TO_TILE_STS.COUNT entries.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * TileMonitor Host to Tile Data.
     * Provides read/write access to the HostToTile FIFO.  When written, the
     * TileMonitor HostToTile FIFO will be written with the associated data
     * and the WPTR will be advanced.  When read, the entry at the head of
     * the FIFO will be returned and the RPTR will be advanced.
     *
     * When empty, reads will have no effect.  Behavior on an empty read is
     * device dependent.  Tile software will receive all 1's on an empty read.
     *
     * When full, writes will be ignored.  Behavior on a full write is device
     * dependent.
     *
     * For Gx9/16/36/72 devices, this FIFO may only be used when PG.AUTO_SEND
     * is zero (packet generator not in boot mode).
     *
     * The FIFO may be drained by reading TM_HOST_TO_TILE_STS.COUNT entries.
     */
    uint_reg_t tm_host_to_tile_data : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t tm_host_to_tile_data : 64;
#endif
  };

  uint_reg_t word;
} RSH_TM_HOST_TO_TILE_DATA_t;


/*
 * TileMonitor Host to Tile Status.
 * Provides status of the HostToTile TileMonitor FIFO.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Current entry count. */
    uint_reg_t count      : 9;
    /* Reserved. */
    uint_reg_t __reserved : 55;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 55;
    uint_reg_t count      : 9;
#endif
  };

  uint_reg_t word;
} RSH_TM_HOST_TO_TILE_STS_t;


/*
 * TileMonitor tile to host Control.
 * Provides control over FIFO interrupts.  Note that the HWM/LWM interrupts
 * trigger continuously as long as the FIFO is is at or above/below the
 * associated water mark.  Thus these interrupts are typically used in
 * INT_BIND.MODE=0 and should be cleared by SW after the condition has been
 * handled (FIFO filled/drained as appropriate).
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Level at which to trigger the low water mark interrupt. */
    uint_reg_t lwm          : 8;
    /* Level at which to trigger the high water mark interrupt. */
    uint_reg_t hwm          : 8;
    /* Reserved. */
    uint_reg_t __reserved_0 : 16;
    /* Provides size of the TileToHost FIFO in terms of 8-byte entries. */
    uint_reg_t max_entries  : 9;
    /* Reserved. */
    uint_reg_t __reserved_1 : 23;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 23;
    uint_reg_t max_entries  : 9;
    uint_reg_t __reserved_0 : 16;
    uint_reg_t hwm          : 8;
    uint_reg_t lwm          : 8;
#endif
  };

  uint_reg_t word;
} RSH_TM_TILE_TO_HOST_CTL_t;


/*
 * TileMonitor tile to host Data.
 * Provides read/write access to the TileToHost FIFO.  When written, the
 * TileMonitor TileToHost FIFO will be written with the associated data and
 * the WPTR will be advanced.  When read, the entry at the head of the FIFO
 * will be returned and the RPTR will be advanced.
 *
 * When empty, reads will have no effect.  Behavior on an empty read is
 * device dependent.  Tile software will receive all 1's on an empty read.
 *
 * When full, writes will be ignored.  Behavior on a full write is device
 * dependent.
 *
 * For Gx9/16/36/72 devices, this FIFO may only be used when PG.AUTO_SEND is
 * zero (packet generator not in boot mode).
 *
 * The FIFO may be drained by reading TM_TILE_TO_HOST_STS.COUNT entries.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * TileMonitor tile to host Data.
     * Provides read/write access to the TileToHost FIFO.  When written, the
     * TileMonitor TileToHost FIFO will be written with the associated data
     * and the WPTR will be advanced.  When read, the entry at the head of
     * the FIFO will be returned and the RPTR will be advanced.
     *
     * When empty, reads will have no effect.  Behavior on an empty read is
     * device dependent.  Tile software will receive all 1's on an empty read.
     *
     * When full, writes will be ignored.  Behavior on a full write is device
     * dependent.
     *
     * For Gx9/16/36/72 devices, this FIFO may only be used when PG.AUTO_SEND
     * is zero (packet generator not in boot mode).
     *
     * The FIFO may be drained by reading TM_TILE_TO_HOST_STS.COUNT entries.
     */
    uint_reg_t tm_tile_to_host_data : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t tm_tile_to_host_data : 64;
#endif
  };

  uint_reg_t word;
} RSH_TM_TILE_TO_HOST_DATA_t;


/*
 * TileMonitor tile to host status.
 * Provides status of the TileToHost TileMonitor FIFO.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Current entry count. */
    uint_reg_t count      : 9;
    /* Reserved. */
    uint_reg_t __reserved : 55;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 55;
    uint_reg_t count      : 9;
#endif
  };

  uint_reg_t word;
} RSH_TM_TILE_TO_HOST_STS_t;


/*
 * Uptime.
 * Provides core_refclk cycle count since last reset (either hard or soft).
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Uptime.
     * Provides core_refclk cycle count since last reset (either hard or
     * soft).
     */
    uint_reg_t uptime     : 56;
    /* Reserved. */
    uint_reg_t __reserved : 8;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 8;
    uint_reg_t uptime     : 56;
#endif
  };

  uint_reg_t word;
} RSH_UPTIME_t;


/*
 * Uptime.
 * Provides core_refclk cycle count since last hard_reset.  This register is
 * not reset on a software reset.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Uptime.
     * Provides core_refclk cycle count since last hard_reset.  This register
     * is not reset on a software reset.
     */
    uint_reg_t uptime_por : 56;
    /* Reserved. */
    uint_reg_t __reserved : 8;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 8;
    uint_reg_t uptime_por : 56;
#endif
  };

  uint_reg_t word;
} RSH_UPTIME_POR_t;


/*
 * VID Core.
 * This register provides the the value driven out on VDD_CORE pins, which
 * drive VID[5:0] of the core voltage regulator.  Note that the VR11 VID code
 * is 8-bits; VID[7:6] should be tied to 01 on the PC board.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Value for VID[5:0] of the core voltage regulator.  The reset value
     * corresponds to 0.9500 V.
     */
    uint_reg_t vid        : 6;
    /* Reserved. */
    uint_reg_t __reserved : 58;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 58;
    uint_reg_t vid        : 6;
#endif
  };

  uint_reg_t word;
} RSH_VID_CORE_t;


/*
 * VID for DDR DRAMs.
 * This register provides the value driven out on VDD_DDR pins, which drive
 * VID[6:3] of the DRAM voltage regulator.  Note that the VR11 VID code is
 * 8-bits; VID[7] should be tied to 0 and VID[2:0] tied to 010, respectively,
 * on the PC board.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Reserved. */
    uint_reg_t __reserved_0 : 3;
    /*
     * Value for VID[6:3] of the DRAM voltage regulator.  The reset value
     * corresponds to 1.2000 V.
     */
    uint_reg_t vid          : 4;
    /* Reserved. */
    uint_reg_t __reserved_1 : 57;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 57;
    uint_reg_t vid          : 4;
    uint_reg_t __reserved_0 : 3;
#endif
  };

  uint_reg_t word;
} RSH_VID_DDR_t;


/* Watchdog Control. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Each bit corresponds to one of the 3 down counters.  When asserted, if
     * the associated down counter triggers an interrupt condition, the
     * RESET_CHIP functionality will be triggered automatically (the
     * interrupt binding doesn't have to be enabled).
     */
    uint_reg_t reset_ena  : 3;
    /* Reserved. */
    uint_reg_t __reserved : 61;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 61;
    uint_reg_t reset_ena  : 3;
#endif
  };

  uint_reg_t word;
} RSH_WATCHDOG_CONTROL_t;



#endif /* !defined(__ASSEMBLER__) */

#endif /* !defined(__ARCH_RSH_H__) */
