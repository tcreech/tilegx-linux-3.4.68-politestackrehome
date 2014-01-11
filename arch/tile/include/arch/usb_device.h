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

#ifndef __ARCH_USB_DEVICE_H__
#define __ARCH_USB_DEVICE_H__

#include <arch/abi.h>
#include <arch/usb_device_def.h>

#ifndef __ASSEMBLER__




/*
 * MMIO Address Space.
 * The MMIO physical address space for the USB is described below.  This is a
 * general description of the MMIO space as opposed to a register description
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * This field of the address provides an offset into the region being
     * accessed.
     */
    uint_reg_t offset       : 18;
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
     * the USB Device (Endpoint). Channel-1 is the USB Host colocated with
     * the endpoint device. Channel-2 is the other USB Host.
     */
    uint_reg_t channel      : 2;
    /* Reserved. */
    uint_reg_t __reserved_1 : 24;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 24;
    uint_reg_t channel      : 2;
    uint_reg_t __reserved_0 : 18;
    uint_reg_t prot         : 2;
    uint_reg_t offset       : 18;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_MMIO_ADDRESS_SPACE_t;


/*
 * Clock Control.
 * Provides control over core PLL
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
} USB_DEVICE_CLOCK_CONTROL_t;


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
} USB_DEVICE_CLOCK_COUNT_t;


/*
 * Clock Divider Control.
 * Provides control over clock dividers
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* MAC 48 MHz clock divider control */
    uint_reg_t clk48_config : 8;
    /* MAC 12 MHz clock divider control */
    uint_reg_t clk12_config : 8;
    /* Reserved. */
    uint_reg_t __reserved   : 48;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved   : 48;
    uint_reg_t clk12_config : 8;
    uint_reg_t clk48_config : 8;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_CLOCK_DIVIDER_CONTROL_t;


/*
 * USB DEBUG Table Init Control.
 * Initialization control for the USB DEBUG tables.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Index into the DEBUG table.  Increments automatically on write or read
     * to DEBUG_INIT_DAT.
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
} USB_DEVICE_DEBUG_INIT_CTL_t;


/*
 * DEBUG Table Data.
 * Read/Write data for USB DEBUG table
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Data for the DEBUG table. */
    uint_reg_t dat : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t dat : 64;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_DEBUG_INIT_DAT_t;


/*
 * USB DES Table Init Control.
 * Initialization control for the USB DES tables.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Index into the DES table.  Increments automatically on write or read
     * to DES_INIT_DAT.
     */
    uint_reg_t idx        : 6;
    /* Reserved. */
    uint_reg_t __reserved : 58;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 58;
    uint_reg_t idx        : 6;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_DES_INIT_CTL_t;


/*
 * DES Table Data.
 * Read/Write data for USB DES table
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Data for the DES table. */
    uint_reg_t dat : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t dat : 64;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_DES_INIT_DAT_t;


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
} USB_DEVICE_DEV_CTL_t;


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
    /* Encoded device Type - 42 to indicate USB Device (Slave) */
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
} USB_DEVICE_DEV_INFO_t;


/*
 * D_CFG_UDC_REG_ADDR.
 * This register configures the device. It is only set during initial
 * configuration or when there is a change in
 * the configuration.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Device speed. This is the expected speed the
     * application programs to the subsystem. The
     * actual speed the subsystem operates
     * depends on the enumeration speed (ENUM
     * SPD) of the Device Status register.
     *  2'b00: HS (PHY clock = 30 or 60 MHz)
     *  2'b01: FS (PHY clock = 30 or 60 MHz)
     *  2'b10: LS (PHY clock = 6 MHz)
     *  2'b11: FS (PHY clock = 48 MHz)
     * Note: The  Subsystem uses only
     * bit 0; bit 1 is a don't care bit (2'bx0 = LS,
     * 2'bx1 = FS).
     */
    uint_reg_t spd              : 2;
    /*
     * Indicates that the device is remote wake up
     * capable.
     */
    uint_reg_t rwkp             : 1;
    /* Indicates that the device is self-powered. */
    uint_reg_t sp               : 1;
    /*
     * Indicates that the device supports Sync
     * Frame.
     */
    uint_reg_t ss               : 1;
    /*
     * PHY interface. Indicates if the UTMI PHY
     * must support an 8-bit or 16-bit interface.
     * Options are:
     *  1'b0: 16-bit
     *  1'b1: 8-bit. This is the default value.
     */
    uint_reg_t pi               : 1;
    /*
     * This bit indicates if the UTMI data bus
     * interface has to support a unidirectional or
     * bidirectional interface. Options are:
     *  1'b0: Unidirectional interface
     *  1'b1: Bidirectional interface
     */
    uint_reg_t dir              : 1;
    /*
     * This bit, together with STATUS Bit 8, provides
     * an option for the  Subsystem to
     * respond to the USB host with a STALL or
     * ACK handshake if the USB host has issued a
     * non-zero-length data packet during the
     * STATUS-OUT stage of a CONTROL transfer.
     */
    uint_reg_t status           : 1;
    /*
     * This bit, together with STATUS Bit 7, provides
     * an option for the  Subsystem to
     * respond to the USB host with a STALL or
     * ACK handshake if the USB host has issued a
     * non-zero-length data packet during the
     * STATUS-OUT stage of a CONTROL transfer.
     */
    uint_reg_t status_1         : 1;
    /*
     * If the application sets this bit, the device
     * detects the phy_rxvalid or phy_rxactive input
     * signal to be continuously asserted for 2 ms,
     * indicating PHY error.
     */
    uint_reg_t phy_error_detect : 1;
    /*
     * These three bits indicate the number of PHY
     * clocks to the  Subsystem's
     * timeout counter. The application uses these
     * bits to increase the timeout value (16 to 18 bit
     * times in full-speed operation), which depends
     * on the PHY's delay in generating line state
     * condition. The default timeout value is 16 bit
     * times.
     */
    uint_reg_t fs_timeout_calib : 3;
    /*
     * These three bits indicate the number of PHY
     * clocks to the  Subsystem's
     * timeout counter. The application uses these
     * bits to increase the timeout value (736 to 848
     * bit times in high-speed operation), which
     * depends on the PHY's delay in generating a
     * line state condition. The default timeout value
     * is 736 bit times.
     * For example, using 30 MHz PHY clock, the
     * timeout time would be 736 + 48 = 784 bit
     * times. The value of 48 bit times comes from
     * the 3 x 30 MHz PHY clock = 3 x [(33.3 ns x
     * 3)/2.08]. Please note the bit times in this
     * example are USB bit times.
     */
    uint_reg_t hs_timeout_calib : 3;
    /*
     * This bit indicates whether the  Subsystem must respond with a STALL or
     * an
     * ACK handshake when the USB host has
     * issued a Clear_Feature (ENDPOINT_HALT)
     * request for Endpoint 0.
     * Options are:
     *  1'b0: ACK
     *  1'b1: STALL
     */
    uint_reg_t halt_status      : 1;
    /*
     * Indicates that the device supports dynamic
     *  register programming.
     * The application can program the
     * registers dynamically whenever it has
     * received an interrupt for either a Set
     * Configuration or a Set Interface request. If
     * this bit is enabled, the  Subsystem
     * returns a NAK handshake during the status
     * IN stage of both the Set Configuration and
     * Set Interface requests until the application
     * has written 1'b1 to the CSR_DONE bit 13 of
     * the Device Control Register.
     */
    uint_reg_t csr_prg          : 1;
    /*
     * Indicates that the device supports Set
     * Descriptor requests.
     * Options are:
     *  1'b0: The  Subsystem returns a
     * STALL handshake to the USB host.
     *  1'b1: The SETUP packet for the Set
     * Descriptor request passes to the
     * application.
     */
    uint_reg_t set_desc         : 1;
    /* This field is reserved, and reads 1'b0: SDR mode. */
    uint_reg_t ddr              : 1;
    /* This field is reserved, and reads 1'b0. */
    uint_reg_t lpm_auto         : 1;
    /* This field is reserved, and reads 1'b0. */
    uint_reg_t lpm_en           : 1;
    /* Reserved. */
    uint_reg_t __reserved       : 42;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved       : 42;
    uint_reg_t lpm_en           : 1;
    uint_reg_t lpm_auto         : 1;
    uint_reg_t ddr              : 1;
    uint_reg_t set_desc         : 1;
    uint_reg_t csr_prg          : 1;
    uint_reg_t halt_status      : 1;
    uint_reg_t hs_timeout_calib : 3;
    uint_reg_t fs_timeout_calib : 3;
    uint_reg_t phy_error_detect : 1;
    uint_reg_t status_1         : 1;
    uint_reg_t status           : 1;
    uint_reg_t dir              : 1;
    uint_reg_t pi               : 1;
    uint_reg_t ss               : 1;
    uint_reg_t sp               : 1;
    uint_reg_t rwkp             : 1;
    uint_reg_t spd              : 2;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_D_CFG_UDC_REG_t;


/*
 * D_CTRL_UDC_REG_ADDR.
 * This register is set at runtime and controls the device after device
 * configuration.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Resuming Signaling on the USB
     * To perform a remote wakeup resume the
     * application sets this bit to 1'b1, then resets it to
     * 1'b0 after 1 ms. The  Subsystem
     * signals the USB host to resume the USB bus.
     * However:
     *  The application must first set RWKP bit 2 in
     * the Device Configuration Register, indicating
     * that the subsystem supports the Remote
     * Wakeup feature).
     *  The host must already have issued a Set
     * Feature request to enable the device Remote
     * Wakeup feature.
     *  However:
     *  The application must first set RWKP bit 2 in
     * the Device Configuration Register, indicating
     * that the Subsystem supports the Remote
     * Wakeup feature.
     *  The host must indicate Device Remote
     * Enable in the LPM transaction.
     */
    uint_reg_t res          : 1;
    /* Reserved. */
    uint_reg_t __reserved_0 : 1;
    /* Receive DMA is enabled. */
    uint_reg_t rde          : 1;
    /* Transmit DMA is enabled. */
    uint_reg_t tde          : 1;
    /*
     * Descriptor Update
     * When this bit is set, the DMA updates the
     * descriptor at the end of each packet processed.
     */
    uint_reg_t du           : 1;
    /*
     * System Endianness Bit
     * A value of 1'b1 indicates a big endian system.
     */
    uint_reg_t be           : 1;
    /*
     * The DMA is in Buffer Fill mode and transfers
     * data into contiguous locations pointed to by the
     * buffer address.
     */
    uint_reg_t bf           : 1;
    /*
     * Threshold Enable
     * When this bit is set, a number of quadlets
     * equivalent to the threshold value is transferred
     * from the RxFIFO to the memory.
     */
    uint_reg_t the          : 1;
    /*
     * Burst Enable
     * When this bit is set, transfers on the AHB are
     * split into bursts.
     */
    uint_reg_t bren         : 1;
    /* This field is reserved, and reads 1'b0: Slave-Only mode. */
    uint_reg_t mode         : 1;
    /*
     * Soft Disconnect
     * The application software uses this bit to signal
     * the  Subsystem to soft-disconnect. When set to 1'b1,
     * this bit causes the device to enter the
     * disconnected state.
     * Note: When the enhancement to set SD bit to 1
     * and UTMI OpMode to Non-driving during poweron-
     * reset is enabled in coreConsultant, and the
     * input signal ss_set_opmode_por_i is asserted
     * HIGH, then the SD default power-on value will
     * be 1'b1 (Disconnect ON). When this
     * enhancement is enabled, the application has to
     * clear this bit when it is ready (after POR) to
     * make the core come out of disconnected state.
     */
    uint_reg_t sd           : 1;
    /*
     * Scale Down
     * This bit reduces the timer values inside the
     *  subsystem when running gate-level
     * simulation only. When this bit is set to 1'b1, timer
     * values are scaled down to reduce simulation
     * time.
     * In Scale-Down mode, the  Subsystem detects a
     * USB reset within the following PHY clock cycles:
     *  60-MHz PHY clock, 8-bit UTMI: 150 PHY
     * clock cycles
     *  30-MHz PHY clock, 16-bit UTMI: 75 PHY
     * clock cycles
     * Reset this bit to 1'b0 for normal operation.
     */
    uint_reg_t scale        : 1;
    /*
     * When the application sets this bit, the
     * Subsystem core returns a NAK handshake to all
     * OUT endpoints.By writing 1'b1 to this bit, the
     * application does not need to write 1'b1 to the
     * SNAK bit 7 of each Endpoint Control register.
     */
    uint_reg_t devnak       : 1;
    /*
     * The application uses this bit to notify the
     * Subsystem that the application has
     * completed programming all required
     * registers, and the Subsystem can acknowledge
     * the current Set Configuration or Set Interface
     * command.
     */
    uint_reg_t csr_done     : 1;
    /* This field is reserved, and reads 1'b0. */
    uint_reg_t srx_flush    : 1;
    /* Reserved. */
    uint_reg_t __reserved_1 : 1;
    /*
     * Burst Length
     * Indicates the length, in 32-bit transfers, of a
     * single burst on the AHB. The subsystem sends
     * number of 32-bit transfers equal to (BRLEN + 1).
     */
    uint_reg_t brlen        : 8;
    /*
     * Threshold Length
     * Indicates the number (THLEN + 1) of 32-bit
     * entries in the RxFIFO before the DMA can start
     * data transfer.
     */
    uint_reg_t thlen        : 8;
    /* Reserved. */
    uint_reg_t __reserved_2 : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_2 : 32;
    uint_reg_t thlen        : 8;
    uint_reg_t brlen        : 8;
    uint_reg_t __reserved_1 : 1;
    uint_reg_t srx_flush    : 1;
    uint_reg_t csr_done     : 1;
    uint_reg_t devnak       : 1;
    uint_reg_t scale        : 1;
    uint_reg_t sd           : 1;
    uint_reg_t mode         : 1;
    uint_reg_t bren         : 1;
    uint_reg_t the          : 1;
    uint_reg_t bf           : 1;
    uint_reg_t be           : 1;
    uint_reg_t du           : 1;
    uint_reg_t tde          : 1;
    uint_reg_t rde          : 1;
    uint_reg_t __reserved_0 : 1;
    uint_reg_t res          : 1;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_D_CTRL_UDC_REG_t;


/*
 * D_INTR_MSK_UDC_REG_ADDR.
 * The device interrupt mask can be set for system-level interrupts using
 * this register. Programming 1b1 in
 * the appropriate bit position in the Interrupt Mask register masks the
 * designated interrupt. Once
 * masked, an interrupt signal does not reach the application, nor does its
 * interrupt bit get set.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Mask equivalent device interrupt bit. (RapidScript/
     * coreConsultant determines the reset value.)
     */
    uint_reg_t mask        : 8;
    /* This field is reserved, and reads 1'b0. */
    uint_reg_t lpm_tkn     : 1;
    /* This field is reserved, and reads 1'b0. */
    uint_reg_t sleep       : 1;
    /* This field is reserved, and reads 1'b0. */
    uint_reg_t early_sleep : 1;
    /* Reserved. */
    uint_reg_t __reserved  : 53;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved  : 53;
    uint_reg_t early_sleep : 1;
    uint_reg_t sleep       : 1;
    uint_reg_t lpm_tkn     : 1;
    uint_reg_t mask        : 8;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_D_INTR_MSK_UDC_REG_t;


/*
 * D_INTR_UDC_REG_ADDR.
 * Device interrupts are set when there are system-level events. Interrupts
 * are used by the application to
 * make system-level decisions. After checking the register, the application
 * must clear the interrupt by
 * writing a 1'b1 to the correct bit.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * The device has received a Set_Configuration
     * command.
     * Note: If the application has not served this
     * interrupt, the subsystem returns a NAK
     * handshake to all transactions except the 8
     * SETUP packet bytes coming from the USB host.
     */
    uint_reg_t sc               : 1;
    /*
     * The device has received a Set_Interface
     * command.
     * Note: If the application has not served this
     * interrupt, the subsystem returns a NAK
     * handshake to all transactions except the 8
     * SETUP packet bytes coming from the USB host.
     */
    uint_reg_t si               : 1;
    /*
     * An idle state is detected on the USB for a
     * duration of 3 milliseconds.
     * This interrupt bit is used for the application
     * firmware to finish its job before the subsystem
     * generates a true suspend (US) interrupt (another
     * 3 milliseconds after the ES interrupt).
     */
    uint_reg_t es               : 1;
    /*
     * A reset is detected on the USB.
     * Note: If the application has not served this
     * interrupt, the  Subsystem returns a NAK
     * handshake for all transactions except the 8
     * SETUP packet bytes from the USB host.
     */
    uint_reg_t ur               : 1;
    /*
     * A suspend state is detected on the USB for a
     * duration of 3 milliseconds, following the 3-
     * millisecond ES interrupt activity due to an idle
     * state.
     * Note: For the  subsystem, there is
     * no Suspend interrupt to the application if the PHY
     * clock is suspended via the suspendm signal
     */
    uint_reg_t us               : 1;
    /* An SOF token is detected on the USB. */
    uint_reg_t sof              : 1;
    /*
     * Speed enumeration is complete.
     * Note: This bit is used only for the  Subsystem.
     */
    uint_reg_t enum_comp        : 1;
    /*
     * A Set/Clear Feature (Remote Wakeup) is
     * received by the core. This bit is set by the core
     * whenever bit 17 of the Device Status Register
     * changes: HIGH to LOW or LOW to HIGH.
     * Note: This bit is valid only when the strap signal
     * input ss_setclr_rmtwkp_ena_i is tied HIGH.
     * When ss_setclr_rmtwkp_ena_i is not tied HIGH,
     * this bit reads 1'b0.
     */
    uint_reg_t rmtwkp_state_int : 1;
    /* This field is reserved, and reads 1'b0. */
    uint_reg_t lpm_tkn          : 1;
    /* This field is reserved, and reads 1'b0. */
    uint_reg_t sleep            : 1;
    /* This field is reserved, and reads 1'b0. */
    uint_reg_t early_sleep      : 1;
    /* Reserved. */
    uint_reg_t __reserved       : 53;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved       : 53;
    uint_reg_t early_sleep      : 1;
    uint_reg_t sleep            : 1;
    uint_reg_t lpm_tkn          : 1;
    uint_reg_t rmtwkp_state_int : 1;
    uint_reg_t enum_comp        : 1;
    uint_reg_t sof              : 1;
    uint_reg_t us               : 1;
    uint_reg_t ur               : 1;
    uint_reg_t es               : 1;
    uint_reg_t si               : 1;
    uint_reg_t sc               : 1;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_D_INTR_UDC_REG_t;


/*
 * D_STS_UDC_REG_ADDR.
 * This register reflects status information needed to service some of the
 * interrupts. This is a read-only
 * register.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * This 4-bit field reflects the configuration set
     * by the SetConfiguration command.
     */
    uint_reg_t cfg          : 4;
    /*
     * This 4-bit field reflects the interface set by the
     * SetInterface command.
     */
    uint_reg_t intf         : 4;
    /*
     * This 4-bit field represents the alternate setting
     * to which the above interface is switched.
     */
    uint_reg_t alt          : 4;
    /*
     * Suspend status. This bit is set as long as a
     * Suspend condition is detected on the USB.
     */
    uint_reg_t susp         : 1;
    /*
     * Enumerated Speed. These bits hold the
     * speed at which the subsystem comes up after
     * the speed enumeration.
     *  If the expected speed (SPD of the Device
     * Configuration register) is high speed and
     * the subsystem connects to a 1.1 host
     * controller, then after Speed Enumeration,
     * these bits indicate that the subsystem is
     * operating in full speed mode (2'b01 for
     * SPD = 2b'00).
     *  If the SPD is high speed and the
     * subsystem connects to a 2.0 host
     * controller, then after Speed Enumeration,
     * these bits indicate that the subsystem is
     * operating in high speed mode (2'b00 for
     * SPD = 2'b00).
     *  If the speed is low speed or full speed and
     * the subsystem connects to either a 1.1 or a
     * 2.0 host controller, then after Speed
     * Enumeration, these bits indicate that the
     * subsystem is operating in low speed mode
     * (2'b10 for SPD = 2'b10) or full speed mode
     * (2'b01 for SPD = 2'b01, and 2'b11 for SPD
     * = 2'b11).
     * Possible options are:
     *  2'b00 HS
     *  2'b01 FS
     *  2'b10 LS
     *  2'b11 FS
     */
    uint_reg_t enum_spd     : 2;
    /*
     * This bit indicates Receive Address FIFO
     * empty status.
     * This bit is set after the DMA transfers data to
     * system memory and there are no new
     * packets received.
     * This bit is cleared after receiving a good
     * packet from the USB.
     *  1'b1 RXFIFO is empty.
     *  1'b0 RXFIFO is not empty.
     */
    uint_reg_t rxfifo_empty : 1;
    /*
     * Either the phy_rxvalid or phy_rxactive input
     * signal is detected to be continuously asserted
     * for 2 ms, indicating PHY error. The  Subsystem goes to the Suspend
     * state
     * as a result.
     * When the application serves the early
     * suspend interrupt (ES bit 2 of the Device
     * Interrupt register) it also must check this bit to
     * determine if the early suspend interrupt was
     * generated due to PHY error detection.
     */
    uint_reg_t phy_error    : 1;
    /*
     * The state of Remote wakeup feature due to
     * Set/Clear Feature (Remotewakeup)
     * command from the host. A value of 1
     * indicates a Set Feature (Remotewakeup) has
     * been received. A value of 0 indicates Clear
     * Feature (Remotewakeup) has been received.
     * Any change to this bit sets an interrupt in
     * bit#7 of Device Interrupt register, if not
     * masked.
     * Note: This bit is valid only when the strap
     * signal input ss_setclr_rmtwkp_ena_i is tied
     * HIGH. When ss_setclr_rmtwkp_ena_i is not
     * tied HIGH, this bit reads 1'b0.
     */
    uint_reg_t rmtwkp_state : 1;
    /*
     * Frame number of the received SOF
     * For high-speed operation:
     *  [31:21]: Millisecond frame number
     *  [20:18]: Microframe number
     * For full-speed operation:
     *  [31:29]: Reserved
     *  [28:18]: Millisecond frame number
     */
    uint_reg_t ts           : 14;
    /* Reserved. */
    uint_reg_t __reserved   : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved   : 32;
    uint_reg_t ts           : 14;
    uint_reg_t rmtwkp_state : 1;
    uint_reg_t phy_error    : 1;
    uint_reg_t rxfifo_empty : 1;
    uint_reg_t enum_spd     : 2;
    uint_reg_t susp         : 1;
    uint_reg_t alt          : 4;
    uint_reg_t intf         : 4;
    uint_reg_t cfg          : 4;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_D_STS_UDC_REG_t;


/*
 * EP0_IN_BUFSIZE_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Buffer size required for this endpoint.
     * The application can program this field to make
     * each endpoint's buffers adaptive, providing
     * flexibility in buffer size when the interface or
     * configuration is changed. This value is in 32-bit
     * words, and indicates the number of 32-bit word
     * entries in the Transmit FIFO.
     * (IN only)
     * Note: This value cannot be changed
     * dynamically during operation.
     */
    uint_reg_t buff_size  : 16;
    /*
     * Initial data PID to be sent for a high-bandwidth
     * isochronous IN transaction.
     * This field is used only in Slave-Only mode.
     *  2'b00: DATA0 PID is sent
     *  2'b01: DATA0 PID is sent
     *  2'b10: DATA1 PID is sent
     *  2'b11: DATA2 PID is sent
     */
    uint_reg_t iso_in_pid : 2;
    /* Reserved. */
    uint_reg_t __reserved : 46;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 46;
    uint_reg_t iso_in_pid : 2;
    uint_reg_t buff_size  : 16;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP0_IN_BUFSIZE_UDC_REG_t;


/*
 * EP0_IN_CTRL_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * STALL Handshake:
     * On successful reception of a SETUP packet
     * (decoded by the application), the subsystem clears
     * both IN and OUT Stall bits, and sets both the IN and
     * OUT NAK bits. The application must check for
     * RxFIFO emptiness before setting the IN and OUT
     * STALL bit.
     * For non-SETUP packets, the subsystem clears
     * either IN or out STALL bits only if a STALL
     * handshake is returned to the USB host, then sets
     * the corresponding NAK bit. The subsystem returns
     * a STALL handshake for the subsequent
     * transactions of the stalled endpoint until the USB
     * host issues a Clear_Feature command to clear it.
     * Once this bit is set, if the subsystem has already
     * returned a STALL handshake to the USB host, the
     * application firmware cannot clear the S bit to stop
     * the subsystem from sending the STALL handshake
     * on the endpoint. The host must instead send one of
     * the following commands to clear the endpoint Halt
     * status:
     *  Clear Feature (Halt)
     *  SetConfiguration
     *  SetInterface.
     * Note: When the configuration option Enable to set
     * the IN and OUT STALL bit (S) when the RxFIFO is
     * not empty is selected and
     * ss_set_stall_nempty_en_i = 1, then the application
     * can set the STALL bit anytime without having to
     * wait for RxFIFO empty. No polling is necessary.
     */
    uint_reg_t s          : 1;
    /*
     * Flush the TxFIFO
     * Reserved for OUT endpoints.
     * The application firmware sets this bit to 1'b1 after it
     * has detected a disconnect/connect on the USB
     * cable, then waits for the IN token endpoint interrupt
     * before resetting this bit to 1'b0. This flushes the
     * stale data out of the TxFIFO. This bit is cleared by
     * the core when TDC (Transmit DMA Complete)
     * occurs on this endpoint.
     */
    uint_reg_t f          : 1;
    /*
     * Configures the endpoint for Snoop mode. In this
     * mode, the subsystem does not check the
     * correctness of OUT packets before transferring
     * them to application memory. Reserved for IN
     * endpoints.
     */
    uint_reg_t sn         : 1;
    /*
     * Poll Demand
     * Poll demand from the application.
     * The application can set this bit after an IN token is
     * received from the endpoint. The application can
     * also set this bit before an IN token is received for
     * the endpoint, if it has the IN transfer data in
     * advance.
     * Note: After sending a zero-length or short packet,
     * the application must wait for the next
     * XFERDONE_TXEMPTY interrupt, before setting up
     * the P bit for the next transfer.
     */
    uint_reg_t p          : 1;
    /*
     * Endpoint Type (2-bit). The possible options are:
     *  2'b00: Control endpoint
     *  2'b01: Isochronous endpoint
     *  2'b10: Bulk endpoint
     *  2'b11: Interrupt endpoint
     */
    uint_reg_t et         : 2;
    /*
     * After a SETUP packet, which is decoded by the
     * application, is received by the core, the core sets
     * the NAK bit for all control IN/OUT endpoints. NAK is
     * also set after a STALL response for the endpoint
     * (the STALL bit is set in Endpoint Control register bit
     * 0).
     *  1: The endpoint responds to the USB host with a
     * NAK handshake.
     *  0: The endpoint responds normally.
     * Note 1: A SETUP packet is sent to the application
     * regardless of whether the NAK bit is set.
     * Note 2: If the NAK for ISOC OUT endpoint is set, then
     * the data out from the host is accepted, provided
     * there is space in the fifo. Without this enhancement
     * enabled, the NAK set on ISOC OUT endpoint
     * rejects the ISOC out data packet.
     */
    uint_reg_t nak        : 1;
    /*
     * Set NAK
     * Used by the application to set the NAK bit (bit 6 of
     * this register). The application must not set the NAK
     * bit for an IN endpoint until it has received an IN
     * token interrupt indicating that the TxFIFO is empty.
     */
    uint_reg_t snak       : 1;
    /*
     * Clear NAK
     * Used by the application to clear the NAK bit (bit 6,
     * below). After the subsystem sets bit 6 (NAK), the
     * application must clear it with a write of 1 to the
     * CNAK bit. (For example, after the application has
     * decoded the SETUP packet and determined it is
     * not an invalid command, the application must set
     * the CNAK bit of the control endpoint to 1'b1 to clear
     * the NAK bit.)
     * The application also must clear the NAK bit
     * whenever the subsystem sets it. (The subsystem
     * sets it due to the application setting the Stall bit.)
     * The application can clear this bit only
     * when the RxFIFO
     * corresponding to the same logical endpoint is
     * empty (Multiple RxFIFO implementation).
     * Note: With the Clear NAK enhancement is enabled  the
     * application can write CNAK any time without
     * waiting for a RxFIFO empty condition. The NAK bit
     * is cleared immediately upon write to CNAK bit. No
     * polling is necessary.
     */
    uint_reg_t cnak       : 1;
    /*
     * Receive Ready
     * If this bit is set by the application, on receiving an
     * OUT packet, the DMA sends the packet to system
     * memory. This bit is deasserted at the end of packet
     * if the Descriptor Update bit is set in the Device
     * Control register. This bit is deasserted at the end of
     * payload if the Descriptor Update bit is deasserted.
     * This bit can be set by the application at any time.
     * The application cannot clear this bit if the DMA is
     * busy transferring the data. If multiple receive FIFO
     * controllers is not implemented, this bit is reserved.
     */
    uint_reg_t rrdy       : 1;
    /*
     * Send NULL packet
     * This bit is available only when the Send NULL
     * option is selected through coreConsultant.
     * This bit provides the application with a mechanism
     * to instruct the  Subsystem to send a NULL
     * (zero-length) packet when no data is available in
     * the particular endpoint's TxFIFO. If this bit is set,
     * when no data is available in the endpoint's TxFIFO,
     * the  Subsystem sends a NULL packet.
     * When this bit is available, you can enable or disable
     * it using the strap signal, ss_send_null_ena_i.
     */
    uint_reg_t send_null  : 1;
    /*
     * Close descriptor channel for this endpoint.
     * This bit applies only to OUT endpoints and is
     * available only when the Close Descriptor Channel
     * option is selected through coreConsultant.
     * The application sets this bit to close the descriptor
     * channel, and the  Subsystem clears this bit
     * after the channel is closed.
     * This bit provides the application with a mechanism
     * to close the descriptors in cases where the USB
     * host does not indicate an end-of-transfer (by
     * issuing a short packet to the USB device). To close
     * the descriptor channel for a particular endpoint, the
     * application sets the CLOSE DESC bit in the
     * Endpoint Control register. When the channel is
     * closed, the  Subsystem clears this bit and
     * generates an interrupt.
     * This bit must be used only for bulk and interrupt
     * OUT endpoints. In addition, before closing the
     * descriptor, software must ensure that the current
     * descriptor reachable by the DMA is active (buffer
     * status is Host Ready). When closed, the descriptor
     * is marked with the Last bit set. When the descriptor
     * is closed, it is assigned one of the following
     * descriptor statuses:
     *  Buffer Fill mode: Accumulated byte count is
     * available in the Rx Bytes field.
     *  Packet-Per-Buffer With Descriptor Update mode:
     * Current reachable descriptor is marked with the
     * Last descriptor, and the Rx Bytes field is set to 0.
     *  Packet-Per-Buffer Without Descriptor Update
     * mode: Current reachable descriptor is marked
     * with the Last descriptor, and the accumulated
     * byte count is available in the Rx Bytes field.
     * When this bit is available, you can enable or disable
     * it using the strap signal, ss_close_desc_ena_i.
     */
    uint_reg_t close_desc : 1;
    /*
     * Receive FIFO Flush for Multiple Receive FIFO.
     *  When the application wants to flush the endpoint
     * receive FIFO, it must first set the SNAK bit in the
     * Endpoint Control register. If the receive DMA is in
     * progress, then the core will finish the current
     * descriptor, terminate the DMA, and flush the data in
     * the endpoint receive FIFO and clear this bit. The
     * application must clear this bit after the EP receive
     * FIFO is empty, by checking the MRXFIFO EMPTY
     * bit in the Endpoint Status register.
     */
    uint_reg_t mrx_flush  : 1;
    /* Reserved. */
    uint_reg_t __reserved : 51;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 51;
    uint_reg_t mrx_flush  : 1;
    uint_reg_t close_desc : 1;
    uint_reg_t send_null  : 1;
    uint_reg_t rrdy       : 1;
    uint_reg_t cnak       : 1;
    uint_reg_t snak       : 1;
    uint_reg_t nak        : 1;
    uint_reg_t et         : 2;
    uint_reg_t p          : 1;
    uint_reg_t sn         : 1;
    uint_reg_t f          : 1;
    uint_reg_t s          : 1;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP0_IN_CTRL_UDC_REG_t;


/*
 * EP0_IN_MPKT_SZ_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Maximum packet size for the endpoint. This is
     * the value in bytes. The maximum packet size
     * must equal the packet size being programmed
     * in the  endpoint register.
     */
    uint_reg_t max_pkt_size : 16;
    /* Reserved. */
    uint_reg_t __reserved   : 48;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved   : 48;
    uint_reg_t max_pkt_size : 16;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP0_IN_MPKT_SZ_REG_t;


/*
 * EP0_IN_STS_UDC_REG.
 * The Endpoint Status register indicates the endpoint status.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Reserved. */
    uint_reg_t __reserved_0     : 4;
    /*
     * An OUT packet has been received by this
     * endpoint. The encoding of these two bits
     * indicates the type of data received.
     * The possible options are:
     *  2'b00: None
     *  2'b01: Received data
     *  2'b10: Received SETUP data (8 bytes)
     *  2'b11: Reserved
     * (The application must write the same values to
     * clear these bits.)
     * Note: In Slave mode, when the Set and Clear
     * Feature enhancement is enabled in
     * coreConsultant, then the application must clear
     * these bits only after clearing the FIFO (in other
     * words, only when the RXFIFO is empty).
     * Clearing these bits while FIFO is not empty
     * causes these bits to remain set until the FIFO is
     * cleared. If by that time the Set/Clear Feature
     * command referring to this endpoint is received,
     * then the Endpoint Status register shows both
     * OUT and RCS/RSS bits, which the application
     * might not be able to handle properly.
     */
    uint_reg_t out_rcv          : 2;
    /*
     * An IN token has been received by this
     * endpoint. After servicing the interrupt, the
     * application must clear this bit. (Reserved for
     * OUT endpoints.)
     */
    uint_reg_t in_rcv           : 1;
    /*
     * Buffer Not Available
     * The subsystem sets this bit when the
     * descriptor's status is either Host Busy or DMA
     * Done to indicate that the descriptor was not
     * ready at the time the DMA tried to access it.
     * After servicing the interrupt, the application
     * must clear this bit.
     */
    uint_reg_t bna              : 1;
    /*
     * Receive Address FIFO Empty Status
     * This bit indicates the empty status of the
     * endpoint receive address FIFO. This bit is set
     * by the core after the DMA transfers data to
     * system memory, and there are no new packets
     * received from the USB. This bit is cleared by
     * the core after receiving a valid packet from the
     * USB.
     *  1'b1: EP RXFIFO is empty
     *  1'b0: EP RXFIFO is not empty
     * Note: This field is reserved in single receive
     * FIFO configurations.
     */
    uint_reg_t mrxfifo_empty    : 1;
    /*
     * Error response on the host bus (AHB) when
     * doing a data transfer, descriptor fetch, or
     * descriptor update for this particular endpoint.
     * After servicing the interrupt, the application
     * must clear this bit.
     */
    uint_reg_t he               : 1;
    /*
     * Transmit DMA Completion
     * Indicates the transmit DMA has completed
     * transferring a descriptor chain's data to the Tx
     * FIFO. After servicing the interrupt, the
     * application must clear this bit.
     */
    uint_reg_t tdc              : 1;
    /*
     * Receive Packet Size
     * Indicates the number of bytes in the current
     * receive packet the RxFIFO is receiving.
     * Because the USB host always sends 8 bytes of
     * SETUP data, these bits do not indicate the
     * receipt of 8 bytes of SETUP data for a SETUP
     * packet. Rather, these bits indicate the
     * configuration status (Configuration number
     * [22:19], Interface number [18:15], and Alternate
     * Setting number [14:11]). This field is used in
     * slave mode only.
     * In DMA mode, the application must check the
     * status from the endpoint data descriptor.
     */
    uint_reg_t rx_pkt_size      : 12;
    /*
     * Isochronous IN transaction for the current
     * microframe is complete.
     * This bit indicates that the isochronous IN
     * transaction for this endpoint is complete. The
     * application can use this information to program
     * the isochronous IN data for the next
     * microframe. This bit is used only in Slave-Only
     * mode.
     */
    uint_reg_t iso_in_done      : 1;
    /*
     * Transmit FIFO Empty detected.
     * This bit indicates that the Transmit FIFO Empty
     * condition is triggered. Application can use this
     * information to load the subsequent data into the
     * Transmit FIFO. The application must clear this
     * bit after writing the data into the Transmit FIFO.
     * Note: This bit is not used for Isochronous
     * endpoints
     */
    uint_reg_t txempty          : 1;
    /*
     * Received Clear Stall indication.
     * This bit indicates that the Clear Feature (EP
     * HALT) command is received for this endpoint.
     * To continue to stall the endpoint, the application
     * must set the S bit Endpoint Control Register.
     * After this, the application clears this RCS bit to
     * acknowledge the reception of clear stall
     * command. Once this bit is cleared, core sends
     * the zero-length packet for the Status IN phase
     * of clear stall command. Strap signal
     * ss_set_clr_stall_ena_i is used to enable/
     * disable this enhancement.
     * Received Clear Stall indication is applicable
     * only for Bulk and Interrupt transactions, and
     * only when the Set and Clear Feature
     * enhancement is enabled.
     */
    uint_reg_t rcs              : 1;
    /*
     * Received Set Stall indication.
     * This bit indicates that the Set Feature (EP
     * HALT) command is received for this endpoint.
     * To stall this endpoint, the application can set
     * the S bit in Endpoint Control Register. After
     * this, the application clears this RSS bit to
     * acknowledge the reception of Set Feature stall
     * command. Once this RSS bit is cleared, core
     * sends the zero-length packet for the Status IN
     * phase of Set Feature command. Strap signal
     * ss_set_clr_stall_ena_i is used to enable/disable this enhancement.
     * Received Set Stall indication is applicable only
     * for Bulk and Interrupt transactions, and only
     * when the Set and Clear Feature
     * enhancement is enabled.
     */
    uint_reg_t rss              : 1;
    /*
     * Transfer Done/Transmit FIFO Empty.
     * This bit indicates that the TXFIFO is empty
     * after the DMA transfer has been completed.
     * The application can use this bit to set the poll
     * bit for the next transfer. The application must
     * first clear this bit after servicing the interrupt.
     */
    uint_reg_t xferdone_txempty : 1;
    /*
     * This bit is valid only for OUT endpoints when
     * CLOSE_DESC feature is enabled. This bit will
     * be set by the core HW when CDC bit in the EP
     * Control register is cleared by the HW
     * generating an interrupt. After servicing the
     * interrupt the application (SW) must clear this
     * bit.
     */
    uint_reg_t cdc_clear        : 1;
    /* Reserved. */
    uint_reg_t __reserved_1     : 35;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1     : 35;
    uint_reg_t cdc_clear        : 1;
    uint_reg_t xferdone_txempty : 1;
    uint_reg_t rss              : 1;
    uint_reg_t rcs              : 1;
    uint_reg_t txempty          : 1;
    uint_reg_t iso_in_done      : 1;
    uint_reg_t rx_pkt_size      : 12;
    uint_reg_t tdc              : 1;
    uint_reg_t he               : 1;
    uint_reg_t mrxfifo_empty    : 1;
    uint_reg_t bna              : 1;
    uint_reg_t in_rcv           : 1;
    uint_reg_t out_rcv          : 2;
    uint_reg_t __reserved_0     : 4;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP0_IN_STS_UDC_REG_t;


/*
 * EP0_OUT_BUFSIZE_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP0_OUT_BUFSIZE_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep0_out_bufsize_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved              : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved              : 32;
    uint_reg_t ep0_out_bufsize_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP0_OUT_BUFSIZE_UDC_REG_t;


/*
 * EP0_OUT_CTRL_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP0_OUT_CTRL_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep0_out_ctrl_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved           : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved           : 32;
    uint_reg_t ep0_out_ctrl_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP0_OUT_CTRL_UDC_REG_t;


/*
 * EP0_OUT_STS_UDC_REG.
 * The Endpoint Status register indicates the endpoint status.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP0_OUT_STS_UDC_REG.
     * The Endpoint Status register indicates the endpoint status.
     */
    uint_reg_t ep0_out_sts_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep0_out_sts_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP0_OUT_STS_UDC_REG_t;


/*
 * EP0_RD_CFRM_UDC_REG_ADDR.
 * Writing to register for zero-length OUT data.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP0_RD_CFRM_UDC_REG_ADDR.
     * Writing to register for zero-length OUT data.
     */
    uint_reg_t ep0_rd_cfrm_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep0_rd_cfrm_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP0_RD_CFRM_UDC_REG_t;


/*
 * EP0_WR_CFRM_UDC_REG_ADDR.
 * Writing to register confirms the IN data into the TxFIFO.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP0_WR_CFRM_UDC_REG_ADDR.
     * Writing to register confirms the IN data into the TxFIFO.
     */
    uint_reg_t ep0_wr_cfrm_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep0_wr_cfrm_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP0_WR_CFRM_UDC_REG_t;


/*
 * EP1_IN_BUFSIZE_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP1_IN_BUFSIZE_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep1_in_bufsize_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved             : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved             : 32;
    uint_reg_t ep1_in_bufsize_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP1_IN_BUFSIZE_UDC_REG_t;


/*
 * EP1_IN_CTRL_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP1_IN_CTRL_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep1_in_ctrl_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep1_in_ctrl_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP1_IN_CTRL_UDC_REG_t;


/*
 * EP1_IN_MPKT_SZ_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP1_IN_MPKT_SZ_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep1_in_mpkt_sz_reg : 16;
    /* Reserved. */
    uint_reg_t __reserved         : 48;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved         : 48;
    uint_reg_t ep1_in_mpkt_sz_reg : 16;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP1_IN_MPKT_SZ_REG_t;


/*
 * EP1_IN_STS_UDC_REG.
 * The Endpoint Status register indicates the endpoint status.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP1_IN_STS_UDC_REG.
     * The Endpoint Status register indicates the endpoint status.
     */
    uint_reg_t ep1_in_sts_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved         : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved         : 32;
    uint_reg_t ep1_in_sts_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP1_IN_STS_UDC_REG_t;


/*
 * EP1_OUT_BUFSIZE_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP1_OUT_BUFSIZE_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep1_out_bufsize_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved              : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved              : 32;
    uint_reg_t ep1_out_bufsize_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP1_OUT_BUFSIZE_UDC_REG_t;


/*
 * EP1_OUT_CTRL_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP1_OUT_CTRL_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep1_out_ctrl_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved           : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved           : 32;
    uint_reg_t ep1_out_ctrl_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP1_OUT_CTRL_UDC_REG_t;


/*
 * EP1_OUT_STS_UDC_REG.
 * The Endpoint Status register indicates the endpoint status.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP1_OUT_STS_UDC_REG.
     * The Endpoint Status register indicates the endpoint status.
     */
    uint_reg_t ep1_out_sts_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep1_out_sts_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP1_OUT_STS_UDC_REG_t;


/*
 * EP1_RD_CFRM_UDC_REG_ADDR.
 * Writing to register for zero-length OUT data.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP1_RD_CFRM_UDC_REG_ADDR.
     * Writing to register for zero-length OUT data.
     */
    uint_reg_t ep1_rd_cfrm_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep1_rd_cfrm_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP1_RD_CFRM_UDC_REG_t;


/*
 * EP1_WR_CFRM_UDC_REG_ADDR.
 * Writing to register confirms the IN data into the TxFIFO.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP1_WR_CFRM_UDC_REG_ADDR.
     * Writing to register confirms the IN data into the TxFIFO.
     */
    uint_reg_t ep1_wr_cfrm_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep1_wr_cfrm_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP1_WR_CFRM_UDC_REG_t;


/*
 * EP2_IN_BUFSIZE_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP2_IN_BUFSIZE_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep2_in_bufsize_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved             : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved             : 32;
    uint_reg_t ep2_in_bufsize_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP2_IN_BUFSIZE_UDC_REG_t;


/*
 * EP2_IN_CTRL_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP2_IN_CTRL_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep2_in_ctrl_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep2_in_ctrl_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP2_IN_CTRL_UDC_REG_t;


/*
 * EP2_IN_MPKT_SZ_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP2_IN_MPKT_SZ_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep2_in_mpkt_sz_reg : 16;
    /* Reserved. */
    uint_reg_t __reserved         : 48;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved         : 48;
    uint_reg_t ep2_in_mpkt_sz_reg : 16;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP2_IN_MPKT_SZ_REG_t;


/*
 * EP2_IN_STS_UDC_REG.
 * The Endpoint Status register indicates the endpoint status.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP2_IN_STS_UDC_REG.
     * The Endpoint Status register indicates the endpoint status.
     */
    uint_reg_t ep2_in_sts_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved         : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved         : 32;
    uint_reg_t ep2_in_sts_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP2_IN_STS_UDC_REG_t;


/*
 * EP2_OUT_BUFSIZE_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP2_OUT_BUFSIZE_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep2_out_bufsize_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved              : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved              : 32;
    uint_reg_t ep2_out_bufsize_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP2_OUT_BUFSIZE_UDC_REG_t;


/*
 * EP2_OUT_CTRL_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP2_OUT_CTRL_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep2_out_ctrl_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved           : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved           : 32;
    uint_reg_t ep2_out_ctrl_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP2_OUT_CTRL_UDC_REG_t;


/*
 * EP2_OUT_STS_UDC_REG.
 * The Endpoint Status register indicates the endpoint status.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP2_OUT_STS_UDC_REG.
     * The Endpoint Status register indicates the endpoint status.
     */
    uint_reg_t ep2_out_sts_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep2_out_sts_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP2_OUT_STS_UDC_REG_t;


/*
 * EP2_RD_CFRM_UDC_REG_ADDR.
 * Writing to register for zero-length OUT data.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP2_RD_CFRM_UDC_REG_ADDR.
     * Writing to register for zero-length OUT data.
     */
    uint_reg_t ep2_rd_cfrm_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep2_rd_cfrm_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP2_RD_CFRM_UDC_REG_t;


/*
 * EP2_WR_CFRM_UDC_REG_ADDR.
 * Writing to register confirms the IN data into the TxFIFO.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP2_WR_CFRM_UDC_REG_ADDR.
     * Writing to register confirms the IN data into the TxFIFO.
     */
    uint_reg_t ep2_wr_cfrm_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep2_wr_cfrm_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP2_WR_CFRM_UDC_REG_t;


/*
 * EP3_IN_BUFSIZE_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP3_IN_BUFSIZE_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep3_in_bufsize_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved             : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved             : 32;
    uint_reg_t ep3_in_bufsize_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP3_IN_BUFSIZE_UDC_REG_t;


/*
 * EP3_IN_CTRL_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP3_IN_CTRL_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep3_in_ctrl_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep3_in_ctrl_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP3_IN_CTRL_UDC_REG_t;


/*
 * EP3_IN_MPKT_SZ_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP3_IN_MPKT_SZ_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep3_in_mpkt_sz_reg : 16;
    /* Reserved. */
    uint_reg_t __reserved         : 48;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved         : 48;
    uint_reg_t ep3_in_mpkt_sz_reg : 16;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP3_IN_MPKT_SZ_REG_t;


/*
 * EP3_IN_STS_UDC_REG.
 * The Endpoint Status register indicates the endpoint status.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP3_IN_STS_UDC_REG.
     * The Endpoint Status register indicates the endpoint status.
     */
    uint_reg_t ep3_in_sts_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved         : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved         : 32;
    uint_reg_t ep3_in_sts_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP3_IN_STS_UDC_REG_t;


/*
 * EP3_OUT_BUFSIZE_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP3_OUT_BUFSIZE_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep3_out_bufsize_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved              : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved              : 32;
    uint_reg_t ep3_out_bufsize_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP3_OUT_BUFSIZE_UDC_REG_t;


/*
 * EP3_OUT_CTRL_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP3_OUT_CTRL_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep3_out_ctrl_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved           : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved           : 32;
    uint_reg_t ep3_out_ctrl_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP3_OUT_CTRL_UDC_REG_t;


/*
 * EP3_OUT_STS_UDC_REG.
 * The Endpoint Status register indicates the endpoint status.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP3_OUT_STS_UDC_REG.
     * The Endpoint Status register indicates the endpoint status.
     */
    uint_reg_t ep3_out_sts_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep3_out_sts_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP3_OUT_STS_UDC_REG_t;


/*
 * EP3_RD_CFRM_UDC_REG_ADDR.
 * Writing to register for zero-length OUT data.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP3_RD_CFRM_UDC_REG_ADDR.
     * Writing to register for zero-length OUT data.
     */
    uint_reg_t ep3_rd_cfrm_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep3_rd_cfrm_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP3_RD_CFRM_UDC_REG_t;


/*
 * EP3_WR_CFRM_UDC_REG_ADDR.
 * Writing to register confirms the IN data into the TxFIFO.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP3_WR_CFRM_UDC_REG_ADDR.
     * Writing to register confirms the IN data into the TxFIFO.
     */
    uint_reg_t ep3_wr_cfrm_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep3_wr_cfrm_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP3_WR_CFRM_UDC_REG_t;


/*
 * EP4_IN_BUFSIZE_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP4_IN_BUFSIZE_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep4_in_bufsize_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved             : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved             : 32;
    uint_reg_t ep4_in_bufsize_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP4_IN_BUFSIZE_UDC_REG_t;


/*
 * EP4_IN_CTRL_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP4_IN_CTRL_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep4_in_ctrl_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep4_in_ctrl_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP4_IN_CTRL_UDC_REG_t;


/*
 * EP4_IN_MPKT_SZ_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP4_IN_MPKT_SZ_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep4_in_mpkt_sz_reg : 16;
    /* Reserved. */
    uint_reg_t __reserved         : 48;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved         : 48;
    uint_reg_t ep4_in_mpkt_sz_reg : 16;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP4_IN_MPKT_SZ_REG_t;


/*
 * EP4_IN_STS_UDC_REG.
 * The Endpoint Status register indicates the endpoint status.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP4_IN_STS_UDC_REG.
     * The Endpoint Status register indicates the endpoint status.
     */
    uint_reg_t ep4_in_sts_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved         : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved         : 32;
    uint_reg_t ep4_in_sts_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP4_IN_STS_UDC_REG_t;


/*
 * EP4_OUT_BUFSIZE_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP4_OUT_BUFSIZE_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep4_out_bufsize_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved              : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved              : 32;
    uint_reg_t ep4_out_bufsize_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP4_OUT_BUFSIZE_UDC_REG_t;


/*
 * EP4_OUT_CTRL_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP4_OUT_CTRL_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep4_out_ctrl_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved           : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved           : 32;
    uint_reg_t ep4_out_ctrl_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP4_OUT_CTRL_UDC_REG_t;


/*
 * EP4_OUT_STS_UDC_REG.
 * The Endpoint Status register indicates the endpoint status.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP4_OUT_STS_UDC_REG.
     * The Endpoint Status register indicates the endpoint status.
     */
    uint_reg_t ep4_out_sts_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep4_out_sts_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP4_OUT_STS_UDC_REG_t;


/*
 * EP4_RD_CFRM_UDC_REG_ADDR.
 * Writing to register for zero-length OUT data.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP4_RD_CFRM_UDC_REG_ADDR.
     * Writing to register for zero-length OUT data.
     */
    uint_reg_t ep4_rd_cfrm_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep4_rd_cfrm_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP4_RD_CFRM_UDC_REG_t;


/*
 * EP4_WR_CFRM_UDC_REG_ADDR.
 * Writing to register confirms the IN data into the TxFIFO.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP4_WR_CFRM_UDC_REG_ADDR.
     * Writing to register confirms the IN data into the TxFIFO.
     */
    uint_reg_t ep4_wr_cfrm_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep4_wr_cfrm_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP4_WR_CFRM_UDC_REG_t;


/*
 * EP5_IN_BUFSIZE_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP5_IN_BUFSIZE_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep5_in_bufsize_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved             : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved             : 32;
    uint_reg_t ep5_in_bufsize_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP5_IN_BUFSIZE_UDC_REG_t;


/*
 * EP5_IN_CTRL_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP5_IN_CTRL_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep5_in_ctrl_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep5_in_ctrl_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP5_IN_CTRL_UDC_REG_t;


/*
 * EP5_IN_MPKT_SZ_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP5_IN_MPKT_SZ_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep5_in_mpkt_sz_reg : 16;
    /* Reserved. */
    uint_reg_t __reserved         : 48;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved         : 48;
    uint_reg_t ep5_in_mpkt_sz_reg : 16;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP5_IN_MPKT_SZ_REG_t;


/*
 * EP5_IN_STS_UDC_REG.
 * The Endpoint Status register indicates the endpoint status.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP5_IN_STS_UDC_REG.
     * The Endpoint Status register indicates the endpoint status.
     */
    uint_reg_t ep5_in_sts_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved         : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved         : 32;
    uint_reg_t ep5_in_sts_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP5_IN_STS_UDC_REG_t;


/*
 * EP5_OUT_BUFSIZE_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP5_OUT_BUFSIZE_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep5_out_bufsize_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved              : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved              : 32;
    uint_reg_t ep5_out_bufsize_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP5_OUT_BUFSIZE_UDC_REG_t;


/*
 * EP5_OUT_CTRL_UDC_REG.
 * This register is used to program the endpoints as required by the
 * application.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP5_OUT_CTRL_UDC_REG.
     * This register is used to program the endpoints as required by the
     * application.
     */
    uint_reg_t ep5_out_ctrl_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved           : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved           : 32;
    uint_reg_t ep5_out_ctrl_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP5_OUT_CTRL_UDC_REG_t;


/*
 * EP5_OUT_STS_UDC_REG.
 * The Endpoint Status register indicates the endpoint status.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP5_OUT_STS_UDC_REG.
     * The Endpoint Status register indicates the endpoint status.
     */
    uint_reg_t ep5_out_sts_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep5_out_sts_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP5_OUT_STS_UDC_REG_t;


/*
 * EP5_RD_CFRM_UDC_REG_ADDR.
 * Writing to register for zero-length OUT data.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP5_RD_CFRM_UDC_REG_ADDR.
     * Writing to register for zero-length OUT data.
     */
    uint_reg_t ep5_rd_cfrm_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep5_rd_cfrm_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP5_RD_CFRM_UDC_REG_t;


/*
 * EP5_WR_CFRM_UDC_REG_ADDR.
 * Writing to register confirms the IN data into the TxFIFO.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * EP5_WR_CFRM_UDC_REG_ADDR.
     * Writing to register confirms the IN data into the TxFIFO.
     */
    uint_reg_t ep5_wr_cfrm_udc_reg : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t ep5_wr_cfrm_udc_reg : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP5_WR_CFRM_UDC_REG_t;


/*
 * EP_INTR_MSK_UDC_REG_ADDR.
 * The Endpoint Interrupt register is used to set endpoint-level interrupts.
 * Since all 16 endpoints can be
 * bidirectional, each endpoint has two interrupt bits (one for each
 * direction). The application needs to
 * clear the interrupt by writing a 1'b1 to the correct bit after checking
 * the register.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Masks interrupts to the IN endpoint equivalent to this
     * value. (RapidScript/coreConsultant determines the
     * reset value.)
     */
    uint_reg_t in_ep_mask  : 16;
    /*
     * Masks interrupts to the OUT endpoint equivalent to
     * this value. (RapidScript/coreConsultant determines
     * the reset value.)
     */
    uint_reg_t out_ep_mask : 16;
    /* Reserved. */
    uint_reg_t __reserved  : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved  : 32;
    uint_reg_t out_ep_mask : 16;
    uint_reg_t in_ep_mask  : 16;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP_INTR_MSK_UDC_REG_t;


/*
 * EP_INTR_UDC_REG_ADDR.
 * The Endpoint Interrupt register is used to set endpoint-level interrupts.
 * Since all 16 endpoints can be
 * bidirectional, each endpoint has two interrupt bits (one for each
 * direction). The application needs to
 * clear the interrupt by writing a 1'b1 to the correct bit after checking
 * the register.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * One bit per IN endpoint, set when there is an event
     * on that endpoint.
     */
    uint_reg_t in_ep      : 16;
    /*
     * One bit per OUT endpoint, set when there is an event
     * on that endpoint.
     */
    uint_reg_t out_ep     : 16;
    /* Reserved. */
    uint_reg_t __reserved : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 32;
    uint_reg_t out_ep     : 16;
    uint_reg_t in_ep      : 16;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_EP_INTR_UDC_REG_t;


/*
 * Error Status.
 * Indicators for various fatal and non-fatal USB error conditions
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Illegal opcode received on MMIO interface */
    uint_reg_t mmio_ill_opc              : 1;
    /* Illegal standalone device status */
    uint_reg_t standalone_device_fsm     : 1;
    /* Illegal standalone device OUT endpoint status */
    uint_reg_t standalone_device_out_fsm : 1;
    /* Illegal standalone device IN endpoint status */
    uint_reg_t standalone_device_in_fsm  : 1;
    /* Reserved. */
    uint_reg_t __reserved                : 60;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved                : 60;
    uint_reg_t standalone_device_in_fsm  : 1;
    uint_reg_t standalone_device_out_fsm : 1;
    uint_reg_t standalone_device_fsm     : 1;
    uint_reg_t mmio_ill_opc              : 1;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_ERROR_STATUS_t;


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
} USB_DEVICE_HFH_INIT_CTL_t;


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
} USB_DEVICE_HFH_INIT_DAT_t;


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
    uint_reg_t __reserved_0 : 16;
    /* Selects vector whose bindings are to be accessed. */
    uint_reg_t vec_sel      : 1;
    /* Reserved. */
    uint_reg_t __reserved_1 : 6;
    /* Selects binding within the vector selected by VEC_SEL. */
    uint_reg_t bind_sel     : 2;
    /* Reserved. */
    uint_reg_t __reserved_2 : 6;
    /*
     * When written with a 1, the interrupt binding data will not be
     * modified.  Set this when writing the VEC_SEL and BIND_SEL fields in
     * preparation for a read.
     */
    uint_reg_t nw           : 1;
    /* Reserved. */
    uint_reg_t __reserved_3 : 15;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_3 : 15;
    uint_reg_t nw           : 1;
    uint_reg_t __reserved_2 : 6;
    uint_reg_t bind_sel     : 2;
    uint_reg_t __reserved_1 : 6;
    uint_reg_t vec_sel      : 1;
    uint_reg_t __reserved_0 : 16;
    uint_reg_t evt_num      : 5;
    uint_reg_t int_num      : 2;
    uint_reg_t tileid       : 8;
    uint_reg_t mode         : 1;
    uint_reg_t enable       : 1;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_INT_BIND_t;


/*
 * Interrupt vector-0, write-one-to-clear.
 * This describes the interrupt status vector that is accessible through
 * INT_VEC0_W1TC and INT_VEC0_RTC.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Reserved. */
    uint_reg_t __reserved_0     : 1;
    /*
     * An MMIO request encountered an error.  Error info captured in
     * USB_DEVICE_CFG_MMIO_ERROR_INFO.
     */
    uint_reg_t mmio_error       : 1;
    /*
     * An MMIO request encountered an error.  Error info captured in
     * USB_DEVICE_CFG_MMIO_ERROR_INFO.
     */
    uint_reg_t mac_access_error : 1;
    /* Reserved. */
    uint_reg_t __reserved_1     : 61;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1     : 61;
    uint_reg_t mac_access_error : 1;
    uint_reg_t mmio_error       : 1;
    uint_reg_t __reserved_0     : 1;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_INT_VEC0_t;


/*
 * Interrupt vector-0, read-to-clear.
 * Interrupt status vector with read-to-clear functionality.  Provides access
 * to the same status bits that are visible in INT_VEC0_W1TC.  Reading this
 * register clears all of the associated interrupts. This vector contains the
 * following interrupts:
 * 0: RSVD0                 - NOT IMPLEMENTED
 *
 * 1: MMIO_ERROR            - An MMIO request encountered an error.  Error
 * info captured in USB_DEVICE_CFG_MMIO_ERROR_INFO.
 *
 * 2: MAC_ACCESS_ERROR      - A MAC access encountered an error. Exception
 * information contained in USB_DEVICE_MAC_ACCESS_ERROR_INFO
 *
 * 3: RSVD3                 - NOT IMPLEMENTED
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Interrupt vector-0, read-to-clear.
     * Interrupt status vector with read-to-clear functionality.  Provides
     * access to the same status bits that are visible in INT_VEC0_W1TC.
     * Reading this register clears all of the associated interrupts. This
     * vector contains the following interrupts:
     * 0: RSVD0                 - NOT IMPLEMENTED
     *
     * 1: MMIO_ERROR            - An MMIO request encountered an error.
     * Error info captured in USB_DEVICE_CFG_MMIO_ERROR_INFO.
     *
     * 2: MAC_ACCESS_ERROR      - A MAC access encountered an error.
     * Exception information contained in USB_DEVICE_MAC_ACCESS_ERROR_INFO
     *
     * 3: RSVD3                 - NOT IMPLEMENTED
     */
    uint_reg_t int_vec0_rtc : 4;
    /* Reserved. */
    uint_reg_t __reserved   : 60;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved   : 60;
    uint_reg_t int_vec0_rtc : 4;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_INT_VEC0_RTC_t;


/*
 * Interrupt vector-0, write-one-to-clear.
 * Interrupt status vector with write-one-to-clear functionality.  Provides
 * access to the same status bits that are visible in INT_VEC0_RTC.  Writing
 * a 1 clears the status bit. This vector contains the following interrupts:
 * 0: RSVD0                 - NOT IMPLEMENTED
 *
 * 1: MMIO_ERROR            - An MMIO request encountered an error.  Error
 * info captured in USB_DEVICE_CFG_MMIO_ERROR_INFO.
 *
 * 2: MAC_ACCESS_ERROR      - A MAC access encountered an error. Exception
 * information contained in USB_DEVICE_MAC_ACCESS_ERROR_INFO
 *
 * 3: RSVD3                 - NOT IMPLEMENTED
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Interrupt vector-0, write-one-to-clear.
     * Interrupt status vector with write-one-to-clear functionality.
     * Provides access to the same status bits that are visible in
     * INT_VEC0_RTC.  Writing a 1 clears the status bit. This vector contains
     * the following interrupts:
     * 0: RSVD0                 - NOT IMPLEMENTED
     *
     * 1: MMIO_ERROR            - An MMIO request encountered an error.
     * Error info captured in USB_DEVICE_CFG_MMIO_ERROR_INFO.
     *
     * 2: MAC_ACCESS_ERROR      - A MAC access encountered an error.
     * Exception information contained in USB_DEVICE_MAC_ACCESS_ERROR_INFO
     *
     * 3: RSVD3                 - NOT IMPLEMENTED
     */
    uint_reg_t int_vec0_w1tc : 4;
    /* Reserved. */
    uint_reg_t __reserved    : 60;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved    : 60;
    uint_reg_t int_vec0_w1tc : 4;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_INT_VEC0_W1TC_t;


/*
 * Interrupt vector-1, write-one-to-clear.
 * This describes the interrupt status vector that is accessible through
 * INT_VEC1_W1TC and INT_VEC1_RTC.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* MAC device or endpoint interrupt. */
    uint_reg_t mac_int    : 1;
    /* Reserved. */
    uint_reg_t __reserved : 63;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 63;
    uint_reg_t mac_int    : 1;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_INT_VEC1_t;


/*
 * Interrupt vector-0, read-to-clear.
 * Interrupt status vector with read-to-clear functionality.  Provides access
 * to the same status bits that are visible in INT_VEC1_W1TC.  Reading this
 * register clears all of the associated interrupts. This vector contains the
 * following interrupts:
 * 0: MAC_INT               - MAC device or endpoint interrupt.
 *
 * 1: RSVD1                 - NOT IMPLEMENTED
 *
 * 2: RSVD2                 - NOT IMPLEMENTED
 *
 * 3: RSVD3                 - NOT IMPLEMENTED
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Interrupt vector-0, read-to-clear.
     * Interrupt status vector with read-to-clear functionality.  Provides
     * access to the same status bits that are visible in INT_VEC1_W1TC.
     * Reading this register clears all of the associated interrupts. This
     * vector contains the following interrupts:
     * 0: MAC_INT               - MAC device or endpoint interrupt.
     *
     * 1: RSVD1                 - NOT IMPLEMENTED
     *
     * 2: RSVD2                 - NOT IMPLEMENTED
     *
     * 3: RSVD3                 - NOT IMPLEMENTED
     */
    uint_reg_t int_vec1_rtc : 4;
    /* Reserved. */
    uint_reg_t __reserved   : 60;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved   : 60;
    uint_reg_t int_vec1_rtc : 4;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_INT_VEC1_RTC_t;


/*
 * Interrupt vector-1, write-one-to-clear.
 * Interrupt status vector with write-one-to-clear functionality.  Provides
 * access to the same status bits that are visible in INT_VEC1_RTC.  Writing
 * a 1 clears the status bit. This vector contains the following interrupts:
 * 0: MAC_INT               - MAC device or endpoint interrupt.
 *
 * 1: RSVD1                 - NOT IMPLEMENTED
 *
 * 2: RSVD2                 - NOT IMPLEMENTED
 *
 * 3: RSVD3                 - NOT IMPLEMENTED
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Interrupt vector-1, write-one-to-clear.
     * Interrupt status vector with write-one-to-clear functionality.
     * Provides access to the same status bits that are visible in
     * INT_VEC1_RTC.  Writing a 1 clears the status bit. This vector contains
     * the following interrupts:
     * 0: MAC_INT               - MAC device or endpoint interrupt.
     *
     * 1: RSVD1                 - NOT IMPLEMENTED
     *
     * 2: RSVD2                 - NOT IMPLEMENTED
     *
     * 3: RSVD3                 - NOT IMPLEMENTED
     */
    uint_reg_t int_vec1_w1tc : 4;
    /* Reserved. */
    uint_reg_t __reserved    : 60;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved    : 60;
    uint_reg_t int_vec1_w1tc : 4;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_INT_VEC1_W1TC_t;


/*
 * MAC Access Error Information.
 * Provides diagnostics information when a MAC response error occurs.
 * Captured whenever the MAC_ACCESS_ERROR interrupt condition occurs which
 * includes size errors, read/write errors, and protection errors.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Full address from request. */
    uint_reg_t addr       : 32;
    /* Request size.  0=1B, 1=2B, 2=4B, 3=8B, 4=16B, 5=32B, 6=48B, 7=64B. */
    uint_reg_t size       : 3;
    /* Reserved. */
    uint_reg_t __reserved : 29;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 29;
    uint_reg_t size       : 3;
    uint_reg_t addr       : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_MAC_ACCESS_ERROR_INFO_t;


/*
 * MAC_APP_STATUS.
 * Suspend Status
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Suspend indication to the application. */
    uint_reg_t ua_suspend_o : 1;
    /* Reserved. */
    uint_reg_t __reserved   : 63;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved   : 63;
    uint_reg_t ua_suspend_o : 1;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_MAC_APP_STATUS_t;


/*
 * MAC_CSR_STATUS.
 * CSR Status
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * This output signal is asserted HIGH whenever the
     * application accesses (read or write) the internal UDC
     * CSR register, address range offset of 500h - 7FCh.
     * When this output signal is asserted HIGH, any write
     * or read request addressed within the UDC CSR
     * address range gets a RETRY response from the
     * AHB slave in the DUT. The retried AHB read or write
     * request to the UDC CSR register subsequent to the
     * de-assertion of this signal gets an OKAY response.
     */
    uint_reg_t udc_csr_busy : 1;
    /* Reserved. */
    uint_reg_t __reserved   : 63;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved   : 63;
    uint_reg_t udc_csr_busy : 1;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_MAC_CSR_STATUS_t;


/*
 * MAC_DEBUG_STATUS.
 * Debug Status
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * This is not an actual port. It is used in the CRV
     * environment for simulations.
     */
    uint_reg_t debug_port_o       : 8;
    /*
     * This is not an actual port. It is used in the CRV
     * environment for simulations.
     */
    uint_reg_t debug_sent_nak_o   : 1;
    /*
     * This is not an actual port. It is used in the CRV
     * environment for simulations.
     */
    uint_reg_t debug_sent_stall_o : 1;
    /*
     * This is not an actual port. It is used in the CRV
     * environment for simulations.
     */
    uint_reg_t debug_pkt_dir_o    : 1;
    /*
     * This is not an actual port. It is used in the CRV
     * environment for simulations.
     */
    uint_reg_t debug_utli_epnum_o : 4;
    /* Reserved. */
    uint_reg_t __reserved         : 49;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved         : 49;
    uint_reg_t debug_utli_epnum_o : 4;
    uint_reg_t debug_pkt_dir_o    : 1;
    uint_reg_t debug_sent_stall_o : 1;
    uint_reg_t debug_sent_nak_o   : 1;
    uint_reg_t debug_port_o       : 8;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_MAC_DEBUG_STATUS_t;


/*
 * MAC_INTERRUPT_STATUS.
 * Interrupt status
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Interrupt signal: device or endpoint event. */
    uint_reg_t ua_int_o   : 1;
    /* Reserved. */
    uint_reg_t __reserved : 63;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 63;
    uint_reg_t ua_int_o   : 1;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_MAC_INTERRUPT_STATUS_t;


/*
 * USB Device MAC Optional Sideband Control.
 * Input for USB Device Optional Sideband Signals
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * When this signal is asserted, the SEND NULL bit (bit
     * 10) in the Endpoint Control register is available.
     */
    uint_reg_t ss_send_null_ena_i       : 1;
    /*
     * When this input signal is asserted HIGH, it sets the
     * opmode (ua_phymode_o) to 2'b01 (non-driving)
     * during power-on-reset. Additionally, it sets the SD
     * (Soft_Disconnect) bit in the Device Control register
     * to 1'b1 (Disconnect ON) during power-on-reset.
     */
    uint_reg_t ss_set_opmode_por_i      : 1;
    /*
     * When this signal is asserted, the CLOSE DESC bit
     * (bit 11) in the Endpoint Control register is available.
     * Note: This signal is used for only DMA mode
     * operation.
     */
    uint_reg_t ss_close_desc_ena_i      : 1;
    /*
     * When this signal is asserted, the UDC Subsystem
     * does not close the descriptor when a short packet is
     * received for an isochronous endpoint.
     * Note: This signal is used for only DMA mode
     * operation.
     */
    uint_reg_t ss_cdc_iso_short_dis_i   : 1;
    /*
     * When this input signal is asserted HIGH, then the
     * application/software can write to the Endpoint
     * Control Register [CNAK] bit any time without
     * waiting for an RxFIFO empty condition. The NAK
     * bit is cleared immediately upon write to CNAK bit.
     * No polling is necessary.
     *  When this input signal is asserted LOW, this
     * enhancement is disabled and the core works as
     * before. It clears the NAK bit only when RxFIFO is
     * empty.
     */
    uint_reg_t ss_clear_nak_nempty_en_i : 1;
    /*
     * When this input signal is asserted HIGH, then the
     * Core can receive data for Isochronous OUT
     * endpoint even when the NAK bit of the
     * Isochronous OUT endpoint is set.
     *  When this input signal is asserted LOW, this
     * enhancement is disabled and the core works as
     * before. It rejects Isochronous OUT if NAK is set on
     * the Isochronous OUT endpoint.
     */
    uint_reg_t ss_dis_iso_out_nak_en_i  : 1;
    /*
     * When this input signal is asserted HIGH, then the
     * application/software can write to the Endpoint
     * Control Register [STALL] bit any time without
     * waiting for an RxFIFO empty condition. The
     * STALL bit is written immediately upon write to that
     * bit. No polling is necessary.
     *  When this input signal is asserted LOW, this
     * enhancement is disabled and the core works as
     * before. It writes to the STALL bit only when
     * RxFIFO is empty.
     */
    uint_reg_t ss_set_stall_nempty_en_i : 1;
    /*
     * When this input is asserted HIGH, the core
     * generates an interrupt on Set/Clear Feature (EP
     * Halt) requests and allows the application to
     * process the commands before completing the
     * status stage command from the USB Host.
     *  When this input is asserted LOW, this
     * enhancement is disabled and the core works as
     * before. No interrupt and application processing
     * occurs on Set/Clear_Feature (EP Halt)
     * commands.
     */
    uint_reg_t ss_set_clr_stall_ena_i   : 1;
    /*
     * When this input signal is asserted HIGH, the
     * application/software can set the SNAK bit even if
     * the Tx FIFO is non-empty.
     *  When this input signal is asserted LOW, this
     * enhancement is disabled and the core works as
     * before. The application has to set SNAK only with
     * Tx FIFO is empty.
     */
    uint_reg_t ss_snak_bit_enh_en_i     : 1;
    /* Reserved. */
    uint_reg_t __reserved               : 55;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved               : 55;
    uint_reg_t ss_snak_bit_enh_en_i     : 1;
    uint_reg_t ss_set_clr_stall_ena_i   : 1;
    uint_reg_t ss_set_stall_nempty_en_i : 1;
    uint_reg_t ss_dis_iso_out_nak_en_i  : 1;
    uint_reg_t ss_clear_nak_nempty_en_i : 1;
    uint_reg_t ss_cdc_iso_short_dis_i   : 1;
    uint_reg_t ss_close_desc_ena_i      : 1;
    uint_reg_t ss_set_opmode_por_i      : 1;
    uint_reg_t ss_send_null_ena_i       : 1;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_MAC_OPTIONAL_SIDEBAND_CONTROL_t;


/*
 * USB Device MAC Sideband Control.
 * Input for USB Device Sideband Signals
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * When this input is asserted HIGH, the core generates
     * an interrupt on Set/Clear Feature
     * (DEVICE_REMOTE_WAKEUP) requests.
     *  When this input is asserted LOW, this enhancement is
     * disabled and the core works as before. No interrupts
     * occur on Set/Clear_Feature
     * (DEVICE_REMOTE_WAKEUP) commands.
     */
    uint_reg_t ss_setclr_rmtwkp_ena_i : 1;
    /* Reserved. */
    uint_reg_t __reserved             : 63;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved             : 63;
    uint_reg_t ss_setclr_rmtwkp_ena_i : 1;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_MAC_SIDEBAND_CONTROL_t;


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
} USB_DEVICE_MEM_INFO_t;


/*
 * MMIO Error Information.
 * Provides diagnostics information when an MMIO error occurs.  Captured
 * whenever the MMIO_ERROR interrupt condition occurs which includes size
 * errors, read/write errors, and protection errors.
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
} USB_DEVICE_MMIO_ERROR_INFO_t;


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
     * Each configuration channel is mapped to a packet device (MAC) having
     * its own config registers.  Mapping TBD
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
} USB_DEVICE_MMIO_INFO_t;


/*
 * USB Device Reset Control.
 * Control USB device reset
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Control USB device PHY reset */
    uint_reg_t phy_reset_mode : 2;
    /* Control USB device MAC reset */
    uint_reg_t mac_reset_mode : 2;
    /* Reserved. */
    uint_reg_t __reserved     : 60;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved     : 60;
    uint_reg_t mac_reset_mode : 2;
    uint_reg_t phy_reset_mode : 2;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_RESET_CONTROL_t;


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
} USB_DEVICE_SCRATCHPAD_t;


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
} USB_DEVICE_SEMAPHORE0_t;


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
} USB_DEVICE_SEMAPHORE1_t;


/*
 * Standalone Device Configuration.
 * USB standalone device configuration
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Set to disable the logic. Note that if the PORT0 is used as the USB
     * host, the device logic is disabled.
     */
    uint_reg_t disable                     : 1;
    /*
     * Set to disable the prefetch function in the TM BULK IN endpoint logic
     */
    uint_reg_t tm_bulk_in_prefetch_disable : 1;
    /* Reserved. */
    uint_reg_t __reserved                  : 62;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved                  : 62;
    uint_reg_t tm_bulk_in_prefetch_disable : 1;
    uint_reg_t disable                     : 1;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_STANDALONE_DEVICE_CONFIG_t;


/*
 * TEST_MODE_UDC_REG.
 * In Test mode, the application can use the AHB to read from a TxFIFO or
 * write to an RxFIFO. Test mode
 * is supported only in the Slave-Only operational mode. In Test mode, only
 * single- DWORD transactions
 * are supported: byte and word transactions on the AHB are not supported. To
 * synchronize the write/
 * read pulses between AHB and VCI clocks, wait states are introduced on the
 * AHB for every data transfer.
 * While in Test mode, all transactions from the USB host, except the SETUP
 * packet, receive a NAK
 * response. When the SETUP packet is received, it is accepted, but the data
 * is not written into the Receive
 * FIFO. After writing the TxFIFO, the data must be confirmed by writing to
 * the offset address (offset
 * 12h01C + [endpoint number x 12h020]) (hexadecimal), which is similar to
 * the normal mode of
 * operation. In non-Test modes, reading from a TxFIFO or writing to an
 * RxFIFO results in an AHB error
 * response. The application must never read an empty TxFIFO or write a full
 * RxFIFO even though an
 * AHB error response is not provided. Writing to the RxFIFO in Test mode and
 * enabling the DMA to transfer
 * the data are not supported.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Test Mode indicator */
    uint_reg_t tstmode    : 1;
    /* Reserved. */
    uint_reg_t __reserved : 63;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 63;
    uint_reg_t tstmode    : 1;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_TEST_MODE_UDC_REG_t;


/*
 * TLB Entry VPN and PFN Data.
 * Read/Write data for the TLB entry's VPN and PFN.  When written, the
 * associated entry's VLD bit will be cleared.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Physical Frame Number */
    uint_reg_t pfn          : 30;
    /* Reserved. */
    uint_reg_t __reserved_0 : 2;
    /* Virtual Page Number */
    uint_reg_t vpn          : 30;
    /* Reserved. */
    uint_reg_t __reserved_1 : 2;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 2;
    uint_reg_t vpn          : 30;
    uint_reg_t __reserved_0 : 2;
    uint_reg_t pfn          : 30;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_TLB_ENTRY_ADDR_t;


/*
 * TLB Entry Attributes.
 * Read/Write data for the TLB entry's ATTR bits.  When written, the TLB
 * entry will be updated.  TLB_ENTRY_ADDR must always be written before this
 * register.  Writing to this register without first writing the
 * TLB_ENTRY_ADDR register causes unpredictable behavior including memory
 * corruption.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Entry valid bit.  Resets to 1 for the first entry in each ASID.
     * Resets to 0 for all other entries.   Clears whenever the associated
     * entry's TLB_ENTRY_ADDR register is written.
     */
    uint_reg_t vld             : 1;
    /* Reserved. */
    uint_reg_t __reserved_0    : 2;
    /*
     * Page size.  Size is 2^(PS+12) so  0=4 kB, 1=8 kB, 2=16 kB ... 30=4096
     * GB.  The max supported page size is 30.
     */
    uint_reg_t ps              : 5;
    /* Reserved. */
    uint_reg_t __reserved_1    : 12;
    /*
     * When 0, physical addresses are hashed to find the home Tile.  When 1,
     * an explicit home is stored in LOC_X,LOC_Y.
     */
    uint_reg_t home_mapping    : 1;
    /* Reserved. */
    uint_reg_t __reserved_2    : 2;
    /*
     * When asserted, only the IO pinned ways in the home cache will be used.
     *  This attribute only applies to writes.
     */
    uint_reg_t pin             : 1;
    /*
     * NonTemporal Hint.  Device services may use this hint as a performance
     * optimization to inform the Tile memory system that the associated data
     * is unlikely to be accessed within a relatively short period of time.
     * Read interfaces may use this hint to invalidate cache data after
     * reading.
     */
    uint_reg_t nt_hint         : 1;
    /* Reserved. */
    uint_reg_t __reserved_3    : 1;
    /*
     * Y-coordinate of home Tile when page is explicitly homed (HOME_MAPPING
     * = 1).  AMT offset when HOME_MAPPING = 0.
     */
    uint_reg_t loc_y_or_offset : 4;
    /* Reserved. */
    uint_reg_t __reserved_4    : 7;
    /*
     * X-coordinate of home Tile when page is explicitly homed (HOME_MAPPING
     * = 1).  AMT mask when HOME_MAPPING = 0.
     */
    uint_reg_t loc_x_or_mask   : 4;
    /* Reserved. */
    uint_reg_t __reserved_5    : 7;
    /*
     * On reads, provides the LRU pointer for the associated ASID.  Ignored
     * on writes.
     */
    uint_reg_t lru             : 4;
    /* Reserved. */
    uint_reg_t __reserved_6    : 12;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_6    : 12;
    uint_reg_t lru             : 4;
    uint_reg_t __reserved_5    : 7;
    uint_reg_t loc_x_or_mask   : 4;
    uint_reg_t __reserved_4    : 7;
    uint_reg_t loc_y_or_offset : 4;
    uint_reg_t __reserved_3    : 1;
    uint_reg_t nt_hint         : 1;
    uint_reg_t pin             : 1;
    uint_reg_t __reserved_2    : 2;
    uint_reg_t home_mapping    : 1;
    uint_reg_t __reserved_1    : 12;
    uint_reg_t ps              : 5;
    uint_reg_t __reserved_0    : 2;
    uint_reg_t vld             : 1;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_TLB_ENTRY_ATTR_t;


/*
 * TLB Table.
 * TLB table.  This table consists of 32 TLB entries.  Each entry is two
 * registers: TLB_ENTRY_ADDR and TLB_ENTRY_ATTR.  This register definition is
 * a description of the address as opposed to the registers themselves.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Reserved. */
    uint_reg_t __reserved_0 : 3;
    /* Selects TLB_ENTRY_ADDR vs TLB_ENTRY_ATTR. */
    uint_reg_t is_attr      : 1;
    /* Selects which TLB entry is accessed. */
    uint_reg_t entry        : 4;
    /* Address space identifier (IOTLB number). */
    uint_reg_t asid         : 1;
    /* Reserved. */
    uint_reg_t __reserved_1 : 55;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 55;
    uint_reg_t asid         : 1;
    uint_reg_t entry        : 4;
    uint_reg_t is_attr      : 1;
    uint_reg_t __reserved_0 : 3;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_TLB_TABLE_t;


/*
 * USB0 Address Extension Register.
 * This register specifies the Virtual Address extension for USB0.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * USB0 Address Extension Register.
     * This register specifies the Virtual Address extension for USB0.
     */
    uint_reg_t tlb_usb0_addr_ext : 32;
    /* Reserved. */
    uint_reg_t __reserved        : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved        : 32;
    uint_reg_t tlb_usb0_addr_ext : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_TLB_USB0_ADDR_EXT_t;


/*
 * TLB USB0 Exception.
 * Captures exception information on USB0 TLB misses.  Software must provide
 * a valid translation to allow forward progress of the transaction.  Other
 * channels may become blocked while the TLB miss is being handled.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Contains the suggested replacement pointer for filling a new TLB entry.
     */
    uint_reg_t lru          : 4;
    /* Reserved. */
    uint_reg_t __reserved_0 : 8;
    /* Contains the virtual address for the last miss. */
    int_reg_t va           : 30;
    /* Reserved. */
    uint_reg_t __reserved_1 : 6;
    /* Contains the ASID for the last miss. */
    uint_reg_t asid         : 1;
    /* Reserved. */
    uint_reg_t __reserved_2 : 15;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_2 : 15;
    uint_reg_t asid         : 1;
    uint_reg_t __reserved_1 : 6;
    int_reg_t va           : 30;
    uint_reg_t __reserved_0 : 8;
    uint_reg_t lru          : 4;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_TLB_USB0_EXC_t;


/*
 * TLB USB0 Exception Response.
 * This register specifies the responses to the HOST0 MAC upon TLB misses (0:
 * Error, 1: Retry).
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * TLB USB0 Exception Response.
     * This register specifies the responses to the HOST0 MAC upon TLB misses
     * (0: Error, 1: Retry).
     */
    uint_reg_t tlb_usb0_exc_resp : 1;
    /* Reserved. */
    uint_reg_t __reserved        : 63;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved        : 63;
    uint_reg_t tlb_usb0_exc_resp : 1;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_TLB_USB0_EXC_RESP_t;


/*
 * TLB USB0 order vector.
 * This register specifies the writes to the indicated TLB entry are required
 * to be global visible before sending acknowledges back to the MAC.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * TLB USB0 order vector.
     * This register specifies the writes to the indicated TLB entry are
     * required to be global visible before sending acknowledges back to the
     * MAC.
     */
    uint_reg_t tlb_usb0_ord : 16;
    /* Reserved. */
    uint_reg_t __reserved   : 48;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved   : 48;
    uint_reg_t tlb_usb0_ord : 16;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_TLB_USB0_ORD_t;


/*
 * USB1 Address Extension Register.
 * This register specifies the Virtual Address extension for USB1.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * USB1 Address Extension Register.
     * This register specifies the Virtual Address extension for USB1.
     */
    uint_reg_t tlb_usb1_addr_ext : 32;
    /* Reserved. */
    uint_reg_t __reserved        : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved        : 32;
    uint_reg_t tlb_usb1_addr_ext : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_TLB_USB1_ADDR_EXT_t;


/*
 * TLB USB1 Exception.
 * Captures exception information on USB1 TLB misses.  Software must provide
 * a valid translation to allow forward progress of the transaction.  Other
 * channels may become blocked while the TLB miss is being handled.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Contains the suggested replacement pointer for filling a new TLB entry.
     */
    uint_reg_t lru          : 4;
    /* Reserved. */
    uint_reg_t __reserved_0 : 8;
    /* Contains the virtual address for the last miss. */
    int_reg_t va           : 30;
    /* Reserved. */
    uint_reg_t __reserved_1 : 6;
    /* Contains the ASID for the last miss. */
    uint_reg_t asid         : 1;
    /* Reserved. */
    uint_reg_t __reserved_2 : 15;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_2 : 15;
    uint_reg_t asid         : 1;
    uint_reg_t __reserved_1 : 6;
    int_reg_t va           : 30;
    uint_reg_t __reserved_0 : 8;
    uint_reg_t lru          : 4;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_TLB_USB1_EXC_t;


/*
 * TLB USB1 Exception Response.
 * This register specifies the responses to the HOST1 MAC upon TLB misses (0:
 * Error, 1: Retry).
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * TLB USB1 Exception Response.
     * This register specifies the responses to the HOST1 MAC upon TLB misses
     * (0: Error, 1: Retry).
     */
    uint_reg_t tlb_usb1_exc_resp : 1;
    /* Reserved. */
    uint_reg_t __reserved        : 63;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved        : 63;
    uint_reg_t tlb_usb1_exc_resp : 1;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_TLB_USB1_EXC_RESP_t;


/*
 * TLB USB1 order vector.
 * This register specifies the writes to the indicated TLB entry are required
 * to be global visible before sending acknowledges back to the MAC.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * TLB USB1 order vector.
     * This register specifies the writes to the indicated TLB entry are
     * required to be global visible before sending acknowledges back to the
     * MAC.
     */
    uint_reg_t tlb_usb1_ord : 16;
    /* Reserved. */
    uint_reg_t __reserved   : 48;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved   : 48;
    uint_reg_t tlb_usb1_ord : 16;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_TLB_USB1_ORD_t;


/*
 * UDC_EP_NE_UDC_REG_0.
 * This register stores endpoint information. A register is dedicated to each
 * implemented endpoint, and
 * each register stores the endpoints maximum packet size, direction, type,
 * configuration number,
 * interface number, and the alternate interface value to which the endpoint
 * is tied.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Endpoint number */
    uint_reg_t ep_num     : 4;
    /*
     * Endpoint direction
     *  0: OUT
     *  1: IN
     */
    uint_reg_t ep_dir     : 1;
    /*
     * Endpoint type
     *  00: Control
     *  01: Isochronous
     *  10: Bulk
     *  11: Interrupt
     */
    uint_reg_t ep_type    : 2;
    /* Configuration number to which this endpoint belongs */
    uint_reg_t config_num : 4;
    /* Interface number to which this endpoint belongs */
    uint_reg_t int_num    : 4;
    /* Alternate setting to which this endpoint belongs */
    uint_reg_t asn        : 4;
    /*
     * Maximum packet size
     * Isochronous IN endpoints may have a maximum packet size of 1024 in
     * High-
     * Speed mode. If this is the case, and if the  Subsystem enumerates in
     * Full-Speed
     * mode, then this field must be reprogrammed to 1023.
     */
    uint_reg_t mpkt       : 11;
    /*
     * Reserved for non-isochronous IN endpoints.
     * If the endpoint is an isochronous IN endpoint, this field indicates
     * the number of
     * isochronous IN commands the application sends in a given microframe.
     * If the
     * application programs 20x0 or 20x1 in this field, the  Subsystem uses
     * DATA0 PID
     * for all transfers from this endpoint. If the application programs 20x2
     * in this field,
     * the  Subsystem uses DATA1 PID for first transfer and DATA0 PID for the
     * second
     * transfer from this endpoint in the given microframe. If the
     * application programs
     * 20x3 in this field, the  Subsystem uses DATA2 PID for first transfer
     * from this
     * endpoint, DATA1 PID for the second transfer, and DATA0 for the third
     * transfer in
     * the given microframe.
     */
    uint_reg_t iso        : 2;
    /* Reserved. */
    uint_reg_t __reserved : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 32;
    uint_reg_t iso        : 2;
    uint_reg_t mpkt       : 11;
    uint_reg_t asn        : 4;
    uint_reg_t int_num    : 4;
    uint_reg_t config_num : 4;
    uint_reg_t ep_type    : 2;
    uint_reg_t ep_dir     : 1;
    uint_reg_t ep_num     : 4;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_UDC_EP_NE_UDC_REG_0_t;


/*
 * UDC_EP_NE_UDC_REG_1.
 * This register stores endpoint information. A register is dedicated to each
 * implemented endpoint, and
 * each register stores the endpoints maximum packet size, direction, type,
 * configuration number,
 * interface number, and the alternate interface value to which the endpoint
 * is tied.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * UDC_EP_NE_UDC_REG_1.
     * This register stores endpoint information. A register is dedicated to
     * each implemented endpoint, and
     * each register stores the endpoints maximum packet size, direction,
     * type, configuration number,
     * interface number, and the alternate interface value to which the
     * endpoint is tied.
     */
    uint_reg_t udc_ep_ne_udc_reg_1 : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t udc_ep_ne_udc_reg_1 : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_UDC_EP_NE_UDC_REG_1_t;


/*
 * UDC_EP_NE_UDC_REG_2.
 * This register stores endpoint information. A register is dedicated to each
 * implemented endpoint, and
 * each register stores the endpoints maximum packet size, direction, type,
 * configuration number,
 * interface number, and the alternate interface value to which the endpoint
 * is tied.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * UDC_EP_NE_UDC_REG_2.
     * This register stores endpoint information. A register is dedicated to
     * each implemented endpoint, and
     * each register stores the endpoints maximum packet size, direction,
     * type, configuration number,
     * interface number, and the alternate interface value to which the
     * endpoint is tied.
     */
    uint_reg_t udc_ep_ne_udc_reg_2 : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t udc_ep_ne_udc_reg_2 : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_UDC_EP_NE_UDC_REG_2_t;


/*
 * UDC_EP_NE_UDC_REG_3.
 * This register stores endpoint information. A register is dedicated to each
 * implemented endpoint, and
 * each register stores the endpoints maximum packet size, direction, type,
 * configuration number,
 * interface number, and the alternate interface value to which the endpoint
 * is tied.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * UDC_EP_NE_UDC_REG_3.
     * This register stores endpoint information. A register is dedicated to
     * each implemented endpoint, and
     * each register stores the endpoints maximum packet size, direction,
     * type, configuration number,
     * interface number, and the alternate interface value to which the
     * endpoint is tied.
     */
    uint_reg_t udc_ep_ne_udc_reg_3 : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t udc_ep_ne_udc_reg_3 : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_UDC_EP_NE_UDC_REG_3_t;


/*
 * UDC_EP_NE_UDC_REG_4.
 * This register stores endpoint information. A register is dedicated to each
 * implemented endpoint, and
 * each register stores the endpoints maximum packet size, direction, type,
 * configuration number,
 * interface number, and the alternate interface value to which the endpoint
 * is tied.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * UDC_EP_NE_UDC_REG_4.
     * This register stores endpoint information. A register is dedicated to
     * each implemented endpoint, and
     * each register stores the endpoints maximum packet size, direction,
     * type, configuration number,
     * interface number, and the alternate interface value to which the
     * endpoint is tied.
     */
    uint_reg_t udc_ep_ne_udc_reg_4 : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t udc_ep_ne_udc_reg_4 : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_UDC_EP_NE_UDC_REG_4_t;


/*
 * UDC_EP_NE_UDC_REG_5.
 * This register stores endpoint information. A register is dedicated to each
 * implemented endpoint, and
 * each register stores the endpoints maximum packet size, direction, type,
 * configuration number,
 * interface number, and the alternate interface value to which the endpoint
 * is tied.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * UDC_EP_NE_UDC_REG_5.
     * This register stores endpoint information. A register is dedicated to
     * each implemented endpoint, and
     * each register stores the endpoints maximum packet size, direction,
     * type, configuration number,
     * interface number, and the alternate interface value to which the
     * endpoint is tied.
     */
    uint_reg_t udc_ep_ne_udc_reg_5 : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t udc_ep_ne_udc_reg_5 : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_UDC_EP_NE_UDC_REG_5_t;


/*
 * UDC_EP_NE_UDC_REG_6.
 * This register stores endpoint information. A register is dedicated to each
 * implemented endpoint, and
 * each register stores the endpoints maximum packet size, direction, type,
 * configuration number,
 * interface number, and the alternate interface value to which the endpoint
 * is tied.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * UDC_EP_NE_UDC_REG_6.
     * This register stores endpoint information. A register is dedicated to
     * each implemented endpoint, and
     * each register stores the endpoints maximum packet size, direction,
     * type, configuration number,
     * interface number, and the alternate interface value to which the
     * endpoint is tied.
     */
    uint_reg_t udc_ep_ne_udc_reg_6 : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t udc_ep_ne_udc_reg_6 : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_UDC_EP_NE_UDC_REG_6_t;


/*
 * UDC_EP_NE_UDC_REG_7.
 * This register stores endpoint information. A register is dedicated to each
 * implemented endpoint, and
 * each register stores the endpoints maximum packet size, direction, type,
 * configuration number,
 * interface number, and the alternate interface value to which the endpoint
 * is tied.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * UDC_EP_NE_UDC_REG_7.
     * This register stores endpoint information. A register is dedicated to
     * each implemented endpoint, and
     * each register stores the endpoints maximum packet size, direction,
     * type, configuration number,
     * interface number, and the alternate interface value to which the
     * endpoint is tied.
     */
    uint_reg_t udc_ep_ne_udc_reg_7 : 32;
    /* Reserved. */
    uint_reg_t __reserved          : 32;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved          : 32;
    uint_reg_t udc_ep_ne_udc_reg_7 : 32;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_UDC_EP_NE_UDC_REG_7_t;


/*
 * UDC_VCON_UDC_REG.
 * ULPI PHY registers can be accessed by firmware through the FW access path.
 * There is one 32-bit
 * register, vcontrol, whose address is just one above the highest endpoint
 * address. When writing to this register, the structure is as shown. When
 * the CPU reads from this register,
 * the lower 8 bits are replaced by the actual register read data.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* (reg write/set/clear data) or (reg read data) */
    uint_reg_t data         : 8;
    /* ext reg address */
    uint_reg_t ext_reg_addr : 8;
    /* 2F: ext reg, other: immediate reg read/write/set/clear */
    uint_reg_t immediate    : 6;
    /* 10: reg write, 11: reg read. Other: reserved */
    uint_reg_t rw           : 2;
    /* Reserved. */
    uint_reg_t __reserved_0 : 1;
    /* new_reg_request, self cleaned */
    uint_reg_t clean        : 1;
    /* Reserved. */
    uint_reg_t __reserved_1 : 38;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 38;
    uint_reg_t clean        : 1;
    uint_reg_t __reserved_0 : 1;
    uint_reg_t rw           : 2;
    uint_reg_t immediate    : 6;
    uint_reg_t ext_reg_addr : 8;
    uint_reg_t data         : 8;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_UDC_VCON_UDC_REG_t;


/*
 * USB0 CLOCK PAD Control.
 * Provides control over USB0 clock PAD
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Drive strength.
     * 0 = tristate
     * 1 = 2 ma
     * 2 = 4 ma
     * 3 = 6 ma
     * 4 = 8 ma
     * 5 = 10 ma
     * 6 = 12 ma
     * 7 = RESERVED
     */
    uint_reg_t strength   : 3;
    /*
     * 1: Enable sustain (keeper), requires about 100 uA to flip the state
     * 0: no keeper
     */
    uint_reg_t sus        : 1;
    /*
     * Slew rate control
     * 00 = slowest
     * 11 = fastest
     */
    uint_reg_t sl         : 2;
    /* 1: enable pull-up resistor */
    uint_reg_t pu         : 1;
    /* 1: enable pull-down resistor */
    uint_reg_t pd         : 1;
    /*
     * Schmitt trigger select
     * 0: no Schmitt trigger
     * 1: Schmitt trigger
     */
    uint_reg_t schm       : 1;
    /* Reserved. */
    uint_reg_t __reserved : 55;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 55;
    uint_reg_t schm       : 1;
    uint_reg_t pd         : 1;
    uint_reg_t pu         : 1;
    uint_reg_t sl         : 2;
    uint_reg_t sus        : 1;
    uint_reg_t strength   : 3;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_USB0_CLK_PAD_CONTROL_t;


/*
 * USB PHY0 Clock Count.
 * Provides relative clock frequency between usb domain and phy0 clock domain.
 */

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
} USB_DEVICE_USB0_CLOCK_COUNT_t;


/*
 * USB0 Control PAD Control.
 * Provides control over USB0 control PADs
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Drive strength.
     * 0 = tristate
     * 1 = 2 ma
     * 2 = 4 ma
     * 3 = 6 ma
     * 4 = 8 ma
     * 5 = 10 ma
     * 6 = 12 ma
     * 7 = RESERVED
     */
    uint_reg_t strength   : 3;
    /*
     * 1: Enable sustain (keeper), requires about 100 uA to flip the state
     * 0: no keeper
     */
    uint_reg_t sus        : 1;
    /*
     * Slew rate control
     * 00 = slowest
     * 11 = fastest
     */
    uint_reg_t sl         : 2;
    /* 1: enable pull-up resistor */
    uint_reg_t pu         : 1;
    /* 1: enable pull-down resistor */
    uint_reg_t pd         : 1;
    /*
     * Schmitt trigger select
     * 0: no Schmitt trigger
     * 1: Schmitt trigger
     */
    uint_reg_t schm       : 1;
    /* Reserved. */
    uint_reg_t __reserved : 55;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 55;
    uint_reg_t schm       : 1;
    uint_reg_t pd         : 1;
    uint_reg_t pu         : 1;
    uint_reg_t sl         : 2;
    uint_reg_t sus        : 1;
    uint_reg_t strength   : 3;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_USB0_CTL_PAD_CONTROL_t;


/*
 * USB0 DATA PAD Control.
 * Provides control over USB0 data PADs
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Drive strength.
     * 0 = tristate
     * 1 = 2 ma
     * 2 = 4 ma
     * 3 = 6 ma
     * 4 = 8 ma
     * 5 = 10 ma
     * 6 = 12 ma
     * 7 = RESERVED
     */
    uint_reg_t strength   : 3;
    /*
     * 1: Enable sustain (keeper), requires about 100 uA to flip the state
     * 0: no keeper
     */
    uint_reg_t sus        : 1;
    /*
     * Slew rate control
     * 00 = slowest
     * 11 = fastest
     */
    uint_reg_t sl         : 2;
    /* 1: enable pull-up resistor */
    uint_reg_t pu         : 1;
    /* 1: enable pull-down resistor */
    uint_reg_t pd         : 1;
    /*
     * Schmitt trigger select
     * 0: no Schmitt trigger
     * 1: Schmitt trigger
     */
    uint_reg_t schm       : 1;
    /* Reserved. */
    uint_reg_t __reserved : 55;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 55;
    uint_reg_t schm       : 1;
    uint_reg_t pd         : 1;
    uint_reg_t pu         : 1;
    uint_reg_t sl         : 2;
    uint_reg_t sus        : 1;
    uint_reg_t strength   : 3;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_USB0_DAT_PAD_CONTROL_t;


/*
 * USB1 CLOCK PAD Control.
 * Provides control over USB1 clock PAD
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Drive strength.
     * 0 = tristate
     * 1 = 2 ma
     * 2 = 4 ma
     * 3 = 6 ma
     * 4 = 8 ma
     * 5 = 10 ma
     * 6 = 12 ma
     * 7 = RESERVED
     */
    uint_reg_t strength   : 3;
    /*
     * 1: Enable sustain (keeper), requires about 100 uA to flip the state
     * 0: no keeper
     */
    uint_reg_t sus        : 1;
    /*
     * Slew rate control
     * 00 = slowest
     * 11 = fastest
     */
    uint_reg_t sl         : 2;
    /* 1: enable pull-up resistor */
    uint_reg_t pu         : 1;
    /* 1: enable pull-down resistor */
    uint_reg_t pd         : 1;
    /*
     * Schmitt trigger select
     * 0: no Schmitt trigger
     * 1: Schmitt trigger
     */
    uint_reg_t schm       : 1;
    /* Reserved. */
    uint_reg_t __reserved : 55;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 55;
    uint_reg_t schm       : 1;
    uint_reg_t pd         : 1;
    uint_reg_t pu         : 1;
    uint_reg_t sl         : 2;
    uint_reg_t sus        : 1;
    uint_reg_t strength   : 3;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_USB1_CLK_PAD_CONTROL_t;


/*
 * USB PHY1 Clock Count.
 * Provides relative clock frequency between usb domain and phy1 clock domain.
 */

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
} USB_DEVICE_USB1_CLOCK_COUNT_t;


/*
 * USB1 Control PAD Control.
 * Provides control over USB1 control PADs
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Drive strength.
     * 0 = tristate
     * 1 = 2 ma
     * 2 = 4 ma
     * 3 = 6 ma
     * 4 = 8 ma
     * 5 = 10 ma
     * 6 = 12 ma
     * 7 = RESERVED
     */
    uint_reg_t strength   : 3;
    /*
     * 1: Enable sustain (keeper), requires about 100 uA to flip the state
     * 0: no keeper
     */
    uint_reg_t sus        : 1;
    /*
     * Slew rate control
     * 00 = slowest
     * 11 = fastest
     */
    uint_reg_t sl         : 2;
    /* 1: enable pull-up resistor */
    uint_reg_t pu         : 1;
    /* 1: enable pull-down resistor */
    uint_reg_t pd         : 1;
    /*
     * Schmitt trigger select
     * 0: no Schmitt trigger
     * 1: Schmitt trigger
     */
    uint_reg_t schm       : 1;
    /* Reserved. */
    uint_reg_t __reserved : 55;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 55;
    uint_reg_t schm       : 1;
    uint_reg_t pd         : 1;
    uint_reg_t pu         : 1;
    uint_reg_t sl         : 2;
    uint_reg_t sus        : 1;
    uint_reg_t strength   : 3;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_USB1_CTL_PAD_CONTROL_t;


/*
 * USB1 DATA PAD Control.
 * Provides control over USB1 data PADs
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Drive strength.
     * 0 = tristate
     * 1 = 2 ma
     * 2 = 4 ma
     * 3 = 6 ma
     * 4 = 8 ma
     * 5 = 10 ma
     * 6 = 12 ma
     * 7 = RESERVED
     */
    uint_reg_t strength   : 3;
    /*
     * 1: Enable sustain (keeper), requires about 100 uA to flip the state
     * 0: no keeper
     */
    uint_reg_t sus        : 1;
    /*
     * Slew rate control
     * 00 = slowest
     * 11 = fastest
     */
    uint_reg_t sl         : 2;
    /* 1: enable pull-up resistor */
    uint_reg_t pu         : 1;
    /* 1: enable pull-down resistor */
    uint_reg_t pd         : 1;
    /*
     * Schmitt trigger select
     * 0: no Schmitt trigger
     * 1: Schmitt trigger
     */
    uint_reg_t schm       : 1;
    /* Reserved. */
    uint_reg_t __reserved : 55;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 55;
    uint_reg_t schm       : 1;
    uint_reg_t pd         : 1;
    uint_reg_t pu         : 1;
    uint_reg_t sl         : 2;
    uint_reg_t sus        : 1;
    uint_reg_t strength   : 3;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_USB1_DAT_PAD_CONTROL_t;


/*
 * USB Port0 Select.
 * USB PORT 0 configuration
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Set to software control the PORT0 usage */
    uint_reg_t strap_pin_disable : 1;
    /*
     * Set if the USB Host channel 1 uses the PORT0, clear if the USB
     * Endpoint channel 0 uses the PORT0. The STRAP_PIN_DISABLE must be set
     * for software modify the port usage.
     */
    uint_reg_t host_enable       : 1;
    /* Reserved. */
    uint_reg_t __reserved        : 62;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved        : 62;
    uint_reg_t host_enable       : 1;
    uint_reg_t strap_pin_disable : 1;
#endif
  };

  uint_reg_t word;
} USB_DEVICE_USB_PORT0_SELECT_t;



#endif /* !defined(__ASSEMBLER__) */

#endif /* !defined(__ARCH_USB_DEVICE_H__) */
