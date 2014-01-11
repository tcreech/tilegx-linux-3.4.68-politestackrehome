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

#ifndef __ARCH_I2CS_H__
#define __ARCH_I2CS_H__

#include <arch/abi.h>
#include <arch/i2cs_def.h>

#ifndef __ASSEMBLER__




/* Ack Control. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * When an external I2C master control writes the I2C slave controller,
     * it is up to I2C slave controller to ack the write data, or nack the
     * write data.
     * 1: always ack no matter whether the I2C slave controller is ready or
     * not.
     *    This mode can be used when an external device does not support
     * nack, or has a different nack definition. If the I2C slave controller
     * is not ready to take the write data, the write request would be
     * corrupted and the I2C slave controller would raise an interrupt
     * (RFIFO_ERR or WACK_ERR).
     *    This mode can be used when an SMBus-like protocol is desirable, for
     * example, ack indicates the slave is present, nack indicates the slave
     * is not present.
     * 0: ack or nack depends on whether the I2C slave controller is ready or
     * not. If the I2C slave controller is not ready, the I2C slave
     * controller will nack.
     */
    uint_reg_t write_ack  : 1;
    /*
     * When an external I2C master control reads the I2C slave controller, it
     * is up to I2C slave controller to ack the read data, or nack the read
     * data.
     * 1: always ack no matter whether the I2C slave controller is ready or
     * not.
     *    This mode can be used when the I2C master does not support clock
     * stretching.
     * 0: ack or nack depends on whether the I2C slave controller is ready or
     * not. If the I2C slave controller is not ready, the I2C slave
     * controller will stretch the clock.
     */
    uint_reg_t read_ack   : 1;
    /* Reserved. */
    uint_reg_t __reserved : 62;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 62;
    uint_reg_t read_ack   : 1;
    uint_reg_t write_ack  : 1;
#endif
  };

  uint_reg_t word;
} I2CS_ACK_CTL_t;


/* Address Phase Control. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Normally the I2C slave controller treats the first two bytes (after
     * the 7-bit slave address + Read/Write_n + Ack) as the address phase
     * within the I2C slave controller. The address phase can be disabled by
     * setting DISABLE bit.
     * 1: disable the address phase.
     * 0: enable the address phase.
     *
     * In the "no-address-phase" mode, an external I2C master device will
     * push all data (note that all payload is treated as data, no address)
     * to the receive FIFO and an RFIFO_WRITE interrupt will be raised when
     * an entry is written to the receive FIFO. It is up to the TILE side
     * software to read from the I2CS_READ_DATA register in a timely fashion.
     * An external I2C master device will read all data from the transmit
     * FIFO and an TFIFO_READ interrupt will be raised when an entry is read
     * from the transmit FIFO. It is up to the TILE side software to write to
     * the I2CS_WRITE_DATA register in a timely fashion.
     */
    uint_reg_t disable    : 1;
    /* Reserved. */
    uint_reg_t __reserved : 63;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 63;
    uint_reg_t disable    : 1;
#endif
  };

  uint_reg_t word;
} I2CS_ADDR_PHASE_t;


/* Baseline Control. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Interface Enable.
     * 1: to enable the interface
     * 0: to reset the interface (such as FIFOs and state machines)
     */
    uint_reg_t enable     : 1;
    /* Reserved. */
    uint_reg_t __reserved : 63;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 63;
    uint_reg_t enable     : 1;
#endif
  };

  uint_reg_t word;
} I2CS_BASELINE_CTL_t;


/* Boot Revision ID. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * I2C Boot Code Revision ID. In general, it is written by external I2C
     * device.
     */
    uint_reg_t rev_id     : 8;
    /* Reserved. */
    uint_reg_t __reserved : 56;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 56;
    uint_reg_t rev_id     : 8;
#endif
  };

  uint_reg_t word;
} I2CS_BOOT_REVISION_ID_t;


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
} I2CS_CLOCK_COUNT_t;


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
} I2CS_DEV_CTL_t;


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
    /* Encoded device Type - 38 to indicate Two-Wire Slave Interface */
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
} I2CS_DEV_INFO_t;


/* Electrical Control. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Drive strength.
     *
     * 0 = tristate
     * 1 = 2 ma (default)
     * 2 = 4 ma
     * 3 = 6 ma
     * 4 = 8 ma
     * 5 = 10 ma
     * 6 = 12 ma
     * 7 = RESERVED
     */
    uint_reg_t elec_strength : 3;
    /*
     * 1: Enable sustain keeper, about 100 uA to flip the state
     * 0: no keeper
     */
    uint_reg_t elec_sustain  : 1;
    /*
     * Slew rate control
     * 00 = slowest
     * 11 = fastest
     */
    uint_reg_t elec_slew     : 2;
    /* 0: No pullup in IO (use external pull-up resister) */
    uint_reg_t elec_pullup   : 1;
    /* 0: No pulldown in IO */
    uint_reg_t elec_pulldown : 1;
    /*
     * Schmitt control.
     * 0: no schmitt
     */
    uint_reg_t elec_schm     : 2;
    /* Reserved. */
    uint_reg_t __reserved    : 54;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved    : 54;
    uint_reg_t elec_schm     : 2;
    uint_reg_t elec_pulldown : 1;
    uint_reg_t elec_pullup   : 1;
    uint_reg_t elec_slew     : 2;
    uint_reg_t elec_sustain  : 1;
    uint_reg_t elec_strength : 3;
#endif
  };

  uint_reg_t word;
} I2CS_ELECTRICAL_CONTROL_t;


/* FIFO Count. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * n: n active entries in the read FIFO (max is 2**3). Each entry has 64
     * bits.
     * 0: no active entry in the receive FIFO (that is empty).
     */
    uint_reg_t rfifo_count  : 4;
    /* Reserved. */
    uint_reg_t __reserved_0 : 12;
    /*
     * n: n active entries in the transmit FIFO (max is 2**3). Each entry has
     * 64 bits.
     * 0: no active entry in the transmit FIFO (that is empty).
     */
    uint_reg_t tfifo_count  : 4;
    /* Reserved. */
    uint_reg_t __reserved_1 : 4;
    /*
     * n: n active entries in the buffer FIFO (max is 2**1). Each entry has
     * 64 bits + controls.
     * 0: no active entry in the buffer FIFO (that is empty).
     */
    uint_reg_t bfifo_count  : 2;
    /* Reserved. */
    uint_reg_t __reserved_2 : 38;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_2 : 38;
    uint_reg_t bfifo_count  : 2;
    uint_reg_t __reserved_1 : 4;
    uint_reg_t tfifo_count  : 4;
    uint_reg_t __reserved_0 : 12;
    uint_reg_t rfifo_count  : 4;
#endif
  };

  uint_reg_t word;
} I2CS_FIFO_COUNT_t;


/* FLAG. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* 1: I2C controller is busy. */
    uint_reg_t busy        : 1;
    /* 1: RFIFO is empty. */
    uint_reg_t rfifo_empty : 1;
    /* 1: TFIFO is empty. */
    uint_reg_t tfifo_empty : 1;
    /* 1: BFIFO is empty. */
    uint_reg_t bfifo_empty : 1;
    /* 1: RFIFO is full. */
    uint_reg_t rfifo_full  : 1;
    /* 1: TFIFO is full. */
    uint_reg_t tfifo_full  : 1;
    /* 1: BFIFO is full. */
    uint_reg_t bfifo_full  : 1;
    /* Reserved. */
    uint_reg_t __reserved  : 57;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved  : 57;
    uint_reg_t bfifo_full  : 1;
    uint_reg_t tfifo_full  : 1;
    uint_reg_t rfifo_full  : 1;
    uint_reg_t bfifo_empty : 1;
    uint_reg_t tfifo_empty : 1;
    uint_reg_t rfifo_empty : 1;
    uint_reg_t busy        : 1;
#endif
  };

  uint_reg_t word;
} I2CS_FLAG_t;


/* Glitch Mask. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * This 6-bit value is used to mask input glitches on SCL and SDA. Any
     * glitches with pulse width (in number of REF_CLK cycles) that is
     * smaller than CYCLE+1 will be removed. For example, with CYCLE=0, any
     * glitches with pulse width of 1 clock cycles or smaller will be
     * removed. Glitches in SCL signal will lock up the I2C controller.
     */
    uint_reg_t cycle      : 6;
    /* Reserved. */
    uint_reg_t __reserved : 58;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 58;
    uint_reg_t cycle      : 6;
#endif
  };

  uint_reg_t word;
} I2CS_GLITCH_MASK_t;


/*
 * Interrupt Vector Mask.
 * Each bit in this register corresponds to a specific interrupt. When set,
 * the associated interrupt will not be dispatched.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* An RFIFO entry was written to the read FIFO. */
    uint_reg_t rfifo_write : 1;
    /* The receive FIFO was read but there was no data in the FIFO. */
    uint_reg_t rdat_err    : 1;
    /* The receive FIFO received write data but the FIFO was full. */
    uint_reg_t rfifo_err   : 1;
    /* A TFIFO entry was popped from the transmit FIFO. */
    uint_reg_t tfifo_read  : 1;
    /* The transmit FIFO was read but there was no data in the FIFO. */
    uint_reg_t tfifo_rerr  : 1;
    /* The transmit FIFO received write data but the FIFO was full. */
    uint_reg_t tfifo_werr  : 1;
    /*
     * A read or write request was received but cannot be fulfilled; for
     * example the BFIFO is full.
     */
    uint_reg_t wack_err    : 1;
    /*
     * Timed out waiting for write data from the external device; for example
     * a first byte was received but a second byte did not.
     */
    uint_reg_t timeout     : 1;
    /* Reserved. */
    uint_reg_t __reserved  : 56;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved  : 56;
    uint_reg_t timeout     : 1;
    uint_reg_t wack_err    : 1;
    uint_reg_t tfifo_werr  : 1;
    uint_reg_t tfifo_rerr  : 1;
    uint_reg_t tfifo_read  : 1;
    uint_reg_t rfifo_err   : 1;
    uint_reg_t rdat_err    : 1;
    uint_reg_t rfifo_write : 1;
#endif
  };

  uint_reg_t word;
} I2CS_INT_VEC_MASK_t;


/*
 * Interrupt Vector, write-one-to-clear.
 * Each bit in this register corresponds to a specific interrupt. Hardware
 * sets the bit when the associated condition has occurred. Writing a 1
 * clears the status bit.
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* A value was written to the receive FIFO */
    uint_reg_t rfifo_write : 1;
    /* The receive FIFO was read but there was no data in the FIFO */
    uint_reg_t rdat_err    : 1;
    /* The receive FIFO received write data but the FIFO was full. */
    uint_reg_t rfifo_err   : 1;
    /* A value was popped from the transmit FIFO */
    uint_reg_t tfifo_read  : 1;
    /* The transmit FIFO was read but there was no data in the FIFO. */
    uint_reg_t tfifo_rerr  : 1;
    /* The transmit FIFO received write data but the FIFO was full. */
    uint_reg_t tfifo_werr  : 1;
    /*
     * A read or write request was received but cannot be fulfilled; for
     * example the BFIFO is full
     */
    uint_reg_t wack_err    : 1;
    /*
     * Timed out waiting for write data from the external device; for example
     * a first byte was received but a second byte did not
     */
    uint_reg_t timeout     : 1;
    /* Reserved. */
    uint_reg_t __reserved  : 56;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved  : 56;
    uint_reg_t timeout     : 1;
    uint_reg_t wack_err    : 1;
    uint_reg_t tfifo_werr  : 1;
    uint_reg_t tfifo_rerr  : 1;
    uint_reg_t tfifo_read  : 1;
    uint_reg_t rfifo_err   : 1;
    uint_reg_t rdat_err    : 1;
    uint_reg_t rfifo_write : 1;
#endif
  };

  uint_reg_t word;
} I2CS_INT_VEC_W1TC_t;


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
} I2CS_MEM_INFO_t;


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
} I2CS_MMIO_INFO_t;


/* I2C Prescaler. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Prescaler to select the I2C frequency. SCL frequency can be calculated
     * by
     * Freq(SCL) = Freq(reference) / (2* PRESCALE + 9).
     *
     * Note:
     * If REF_CLK = 125 MHz, the default I2C frequency = Freq(SCL) = 125M /
     * (2*152 + 9) = 399 KHz.
     */
    uint_reg_t prescaler  : 12;
    /* Reserved. */
    uint_reg_t __reserved : 52;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 52;
    uint_reg_t prescaler  : 12;
#endif
  };

  uint_reg_t word;
} I2CS_PRESCALER_t;


/* RSHIM Device Word Read. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Certain RSHIM device read request may result in a FIFO pop operation.
     * If a device read request pops the FIFO and the device read can not
     * consume all the popped data, then unconsumed data could be lost. In
     * this case, it is desirable for the I2C slave controller not to
     * generate one device read for each byte read request from an external
     * I2C master.
     *
     *
     * 1: The I2C slave controller schedules one RSHIM device read request
     * for each word read request (word-address-aligned) from an external I2C
     * master. Data from a locally stored buffer will be returned for
     * sequential reads from the external I2C master. External I2C master
     * must start a read request from an 8-byte aligned address to avoid
     * unpredictable results.
     *
     * 0: The I2C slave controller schedules one RSHIM device read request
     * for each byte read request from an external I2C master. The external
     * I2C master can start a read request from an unaligned address (no
     * restriction on byte offset). FIFO reads should be avoided in this
     * setting.
     */
    uint_reg_t word_read  : 1;
    /* Reserved. */
    uint_reg_t __reserved : 63;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 63;
    uint_reg_t word_read  : 1;
#endif
  };

  uint_reg_t word;
} I2CS_RDEV_ACCESS_t;


/* RSHIM Device Address High. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Most significant 4-bits of rshim device address. */
    uint_reg_t high       : 4;
    /* Reserved. */
    uint_reg_t __reserved : 60;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 60;
    uint_reg_t high       : 4;
#endif
  };

  uint_reg_t word;
} I2CS_RDEV_ADDR_t;


/* I2C RFIFO Read Data. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Contains the receive data, which is pushed by an external i2c device
     * (via the I2CSS I2C Write Data). When this register is read from an
     * external i2c device, receive data will be returned but the read
     * pointer of RFIFO will not be updated.
     */
    uint_reg_t rdat : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t rdat : 64;
#endif
  };

  uint_reg_t word;
} I2CS_READ_DATA_t;


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
} I2CS_SCRATCHPAD_t;


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
} I2CS_SEMAPHORE0_t;


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
} I2CS_SEMAPHORE1_t;


/*
 * I2C Slave Address.
 * I2C 7-bit Slave Address
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Reserved. */
    uint_reg_t __reserved_0 : 1;
    /*
     * Specifies the top 7 bits I2C slave address to identify this I2C slave
     * controller on the TILE processor.
     */
    uint_reg_t addr         : 7;
    /* Reserved. */
    uint_reg_t __reserved_1 : 56;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 56;
    uint_reg_t addr         : 7;
    uint_reg_t __reserved_0 : 1;
#endif
  };

  uint_reg_t word;
} I2CS_SLAVE_ADDRESS_t;


/* I2C TFIFO Read Data. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Read data. Reading this register from an external i2c device will pop
     * the head of the transmit data FIFO and return that value (up to 8
     * bytes). When this register is read from a TILE, the most recently
     * popped receive data will be returned but the Transmit FIFO will not be
     * popped.
     */
    uint_reg_t rdat : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t rdat : 64;
#endif
  };

  uint_reg_t word;
} I2CS_TFIFO_READ_DATA_t;


/* I2C TFIFO Write Data. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Write data. A write to this register will push the value onto the
     * Transmit FIFO. This is the only RW register that should not be written
     * by external device (no hardware protection). Reading this register
     * will return the last written value.
     */
    uint_reg_t wdat : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t wdat : 64;
#endif
  };

  uint_reg_t word;
} I2CS_TFIFO_WRITE_DATA_t;


/* I2C RFIFO Write Data. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Write only register that can be accessed only by external i2c. Write
     * by the RSHIM will be ignored. Read will always return 0.
     */
    uint_reg_t i2c_wdat : 64;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t i2c_wdat : 64;
#endif
  };

  uint_reg_t word;
} I2CS_WRITE_DATA_t;



#endif /* !defined(__ASSEMBLER__) */

#endif /* !defined(__ARCH_I2CS_H__) */
