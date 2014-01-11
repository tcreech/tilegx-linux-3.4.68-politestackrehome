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

#ifndef __ARCH_UART_H__
#define __ARCH_UART_H__

#include <arch/abi.h>
#include <arch/uart_def.h>

#ifndef __ASSEMBLER__




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
    /*
     * 1: ignore incoming UART receive data. 0: function mode (check incoming
     * UART receive data).
     */
    uint_reg_t freeze     : 1;
    /* Reserved. */
    uint_reg_t __reserved : 62;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 62;
    uint_reg_t freeze     : 1;
    uint_reg_t enable     : 1;
#endif
  };

  uint_reg_t word;
} UART_BASELINE_CTL_t;


/* Boot Revision ID. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* UART Controller Boot Revision ID. */
    uint_reg_t boot_rev_id : 8;
    /* Reserved. */
    uint_reg_t __reserved  : 56;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved  : 56;
    uint_reg_t boot_rev_id : 8;
#endif
  };

  uint_reg_t word;
} UART_BOOT_REVISION_ID_t;


/* Buffer Threshold. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Receive FIFO almost full threshold. */
    uint_reg_t rfifo_afull  : 9;
    /* Reserved. */
    uint_reg_t __reserved_0 : 7;
    /* Transmit FIFO almost empty threshold. */
    uint_reg_t tfifo_aempty : 9;
    /* Reserved. */
    uint_reg_t __reserved_1 : 39;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 39;
    uint_reg_t tfifo_aempty : 9;
    uint_reg_t __reserved_0 : 7;
    uint_reg_t rfifo_afull  : 9;
#endif
  };

  uint_reg_t word;
} UART_BUFFER_THRESHOLD_t;


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
} UART_CLOCK_COUNT_t;


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
} UART_DEV_CTL_t;


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
    /* Encoded device Type - 40 to indicate UART */
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
} UART_DEV_INFO_t;


/* Divisor. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Baud Rate Divisor.  Desired_baud_rate = REF_CLK frequency / (baud *
     * 16).
     *                       Note: REF_CLK is always 125 MHz, the default
     * divisor = 68, baud rate = 125M/(68*16) = 115200 baud.
     */
    uint_reg_t divisor    : 12;
    /* Reserved. */
    uint_reg_t __reserved : 52;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 52;
    uint_reg_t divisor    : 12;
#endif
  };

  uint_reg_t word;
} UART_DIVISOR_t;


/* Electrical Control. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Drive strength.
     *                        0 = tristate
     *                        1 = 2 ma
     *                        2 = 4 ma (default)
     *                        3 = 6 ma
     *                        4 = 8 ma
     *                        5 = 10 ma
     *                        6 = 12 ma
     *                        7 = RESERVED
     */
    uint_reg_t elec_strength : 3;
    /*
     * 1: Enable sustain keeper, about 100 uA to flip the state
     *
     * 0: no keeper
     */
    uint_reg_t elec_sustain  : 1;
    /*
     * Slew rate control.
     * 00 = slowest,
     * 11 = fastest
     */
    uint_reg_t elec_slew     : 2;
    /* 0: No pullup in IO */
    uint_reg_t elec_pullup   : 1;
    /* 0: No pulldown in IO */
    uint_reg_t elec_pulldown : 1;
    /* Schmitt control.  0: no schmitt */
    uint_reg_t elec_schm     : 1;
    /* Reserved. */
    uint_reg_t __reserved    : 55;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved    : 55;
    uint_reg_t elec_schm     : 1;
    uint_reg_t elec_pulldown : 1;
    uint_reg_t elec_pullup   : 1;
    uint_reg_t elec_slew     : 2;
    uint_reg_t elec_sustain  : 1;
    uint_reg_t elec_strength : 3;
#endif
  };

  uint_reg_t word;
} UART_ELECTRICAL_CONTROL_t;


/* FIFO Count. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * n: n active entries in the receive FIFO (max is 2**8). Each entry has
     * 8 bits.
     * 0: no active entry in the receive FIFO (that is empty).
     */
    uint_reg_t rfifo_count  : 9;
    /* Reserved. */
    uint_reg_t __reserved_0 : 7;
    /*
     * n: n active entries in the transmit FIFO (max is 2**8). Each entry has
     * 8 bits.
     * 0: no active entry in the transmit FIFO (that is empty).
     */
    uint_reg_t tfifo_count  : 9;
    /* Reserved. */
    uint_reg_t __reserved_1 : 7;
    /*
     * n: n active entries in the write FIFO (max is 2**2). Each entry has 8
     * bits.
     * 0: no active entry in the write FIFO (that is empty).
     */
    uint_reg_t wfifo_count  : 3;
    /* Reserved. */
    uint_reg_t __reserved_2 : 29;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_2 : 29;
    uint_reg_t wfifo_count  : 3;
    uint_reg_t __reserved_1 : 7;
    uint_reg_t tfifo_count  : 9;
    uint_reg_t __reserved_0 : 7;
    uint_reg_t rfifo_count  : 9;
#endif
  };

  uint_reg_t word;
} UART_FIFO_COUNT_t;


/* FLAG. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Reserved. */
    uint_reg_t __reserved_0 : 1;
    /* 1: receive FIFO is empty */
    uint_reg_t rfifo_empty  : 1;
    /* 1: write FIFO is empty. */
    uint_reg_t wfifo_empty  : 1;
    /* 1: transmit FIFO is empty. */
    uint_reg_t tfifo_empty  : 1;
    /* 1: receive FIFO is full. */
    uint_reg_t rfifo_full   : 1;
    /* 1: write FIFO is full. */
    uint_reg_t wfifo_full   : 1;
    /* 1: transmit FIFO is full. */
    uint_reg_t tfifo_full   : 1;
    /* Reserved. */
    uint_reg_t __reserved_1 : 57;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1 : 57;
    uint_reg_t tfifo_full   : 1;
    uint_reg_t wfifo_full   : 1;
    uint_reg_t rfifo_full   : 1;
    uint_reg_t tfifo_empty  : 1;
    uint_reg_t wfifo_empty  : 1;
    uint_reg_t rfifo_empty  : 1;
    uint_reg_t __reserved_0 : 1;
#endif
  };

  uint_reg_t word;
} UART_FLAG_t;


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
    /* Read data FIFO read and no data available */
    uint_reg_t rdat_err       : 1;
    /* Write FIFO was written but it was full */
    uint_reg_t wdat_err       : 1;
    /* Stop bit not found when current data was received */
    uint_reg_t frame_err      : 1;
    /* Parity error was detected when current data was received */
    uint_reg_t parity_err     : 1;
    /* Data was received but the receive FIFO was full */
    uint_reg_t rfifo_overflow : 1;
    /*
     * An almost full event is reached when data is to be written to the
     * receive FIFO, and the receive FIFO has more than or equal to
     * BUFFER_THRESHOLD.RFIFO_AFULL bytes.
     */
    uint_reg_t rfifo_afull    : 1;
    /* Reserved. */
    uint_reg_t __reserved_0   : 1;
    /* An entry in the transmit FIFO was popped */
    uint_reg_t tfifo_re       : 1;
    /* An entry has been pushed into the receive FIFO */
    uint_reg_t rfifo_we       : 1;
    /* An entry of the write FIFO has been popped */
    uint_reg_t wfifo_re       : 1;
    /* Rshim read receive FIFO in protocol mode */
    uint_reg_t rfifo_err      : 1;
    /*
     * An almost empty event is reached when data is to be read from the
     * transmit FIFO, and the transmit FIFO has less than or equal to
     * BUFFER_THRESHOLD.TFIFO_AEMPTY bytes.
     */
    uint_reg_t tfifo_aempty   : 1;
    /* Reserved. */
    uint_reg_t __reserved_1   : 52;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1   : 52;
    uint_reg_t tfifo_aempty   : 1;
    uint_reg_t rfifo_err      : 1;
    uint_reg_t wfifo_re       : 1;
    uint_reg_t rfifo_we       : 1;
    uint_reg_t tfifo_re       : 1;
    uint_reg_t __reserved_0   : 1;
    uint_reg_t rfifo_afull    : 1;
    uint_reg_t rfifo_overflow : 1;
    uint_reg_t parity_err     : 1;
    uint_reg_t frame_err      : 1;
    uint_reg_t wdat_err       : 1;
    uint_reg_t rdat_err       : 1;
#endif
  };

  uint_reg_t word;
} UART_INTERRUPT_MASK_t;


/*
 * Interrupt vector, write-one-to-clear.
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
    /* Read data FIFO read and no data available */
    uint_reg_t rdat_err       : 1;
    /* Write FIFO was written but it was full */
    uint_reg_t wdat_err       : 1;
    /* Stop bit not found when current data was received */
    uint_reg_t frame_err      : 1;
    /* Parity error was detected when current data was received */
    uint_reg_t parity_err     : 1;
    /* Data was received but the receive FIFO was full */
    uint_reg_t rfifo_overflow : 1;
    /*
     * Data was received and the receive FIFO is now almost full (more than
     * BUFFER_THRESHOLD.RFIFO_AFULL bytes in it)
     */
    uint_reg_t rfifo_afull    : 1;
    /* Reserved. */
    uint_reg_t __reserved_0   : 1;
    /* An entry in the transmit FIFO was popped */
    uint_reg_t tfifo_re       : 1;
    /* An entry has been pushed into the receive FIFO */
    uint_reg_t rfifo_we       : 1;
    /* An entry of the write FIFO has been popped */
    uint_reg_t wfifo_re       : 1;
    /* Rshim read receive FIFO in protocol mode */
    uint_reg_t rfifo_err      : 1;
    /*
     * Data was read from the transmit FIFO and now it is almost empty (less
     * than or equal to BUFFER_THRESHOLD.TFIFO_AEMPTY bytes in it).
     */
    uint_reg_t tfifo_aempty   : 1;
    /* Reserved. */
    uint_reg_t __reserved_1   : 52;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_1   : 52;
    uint_reg_t tfifo_aempty   : 1;
    uint_reg_t rfifo_err      : 1;
    uint_reg_t wfifo_re       : 1;
    uint_reg_t rfifo_we       : 1;
    uint_reg_t tfifo_re       : 1;
    uint_reg_t __reserved_0   : 1;
    uint_reg_t rfifo_afull    : 1;
    uint_reg_t rfifo_overflow : 1;
    uint_reg_t parity_err     : 1;
    uint_reg_t frame_err      : 1;
    uint_reg_t wdat_err       : 1;
    uint_reg_t rdat_err       : 1;
#endif
  };

  uint_reg_t word;
} UART_INTERRUPT_STATUS_t;


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
} UART_MEM_INFO_t;


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
} UART_MMIO_INFO_t;


/*
 * Mode.
 * Protocol Mode. Each UART controller receives a static FORCE_PROTOCOL boot
 * strap input. In protocol mode, MODE.BYPASS can be programmed to 1 so that
 * the FORCE_PROTOCOL boot strap will be ignored.  The strapping pin, BYPASS
 * mode, and UART_MODE determine the behavior of the UART interface as
 * follows:
 * <br><br><table border="1">
 * <tr><th>force_protocol (pin)<th>BYPASS<th>UART_MODE<th>Description</tr>
 * <tr><td>0<td>0<td>1<td>Ingress is in interrupt mode</tr>
 * <tr><td>1<td>0<td>1<td>Ingress is in protocol mode</tr>
 * <tr><td>*<td>1<td>1<td>Ingress is in interrupt mode</tr>
 * <tr><td>*<td>1<td>0<td>Ingress is in protocol mode</tr>
 * <tr><td>*<td>0<td>0<td>Ingress is in protocol mode</tr>
 * </table>
 */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * 0: ingress is in protocol mode
     *                       1: ingress is in interrupt mode
     */
    uint_reg_t uart_mode  : 1;
    /*
     * 0: check FORCE_PROTOCOL boot strap pin.
     *                       1: ignore FORCE_PROTOCOL boot strap pin.
     */
    uint_reg_t bypass     : 1;
    /* Reserved. */
    uint_reg_t __reserved : 62;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 62;
    uint_reg_t bypass     : 1;
    uint_reg_t uart_mode  : 1;
#endif
  };

  uint_reg_t word;
} UART_MODE_t;


/* Receive Data. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Read data. Reading this register from a TILE will pop the head of the
     * receive data FIFO and return that value. When this register is read
     * from external device, receive data will be returned but receive FIFO
     * will not be popped.
     */
    uint_reg_t rdat       : 8;
    /* Reserved. */
    uint_reg_t __reserved : 56;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 56;
    uint_reg_t rdat       : 8;
#endif
  };

  uint_reg_t word;
} UART_RECEIVE_DATA_t;


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
} UART_SCRATCHPAD_t;


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
} UART_SEMAPHORE0_t;


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
} UART_SEMAPHORE1_t;


/* Transmit Data. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /*
     * Write data. A write to this register will push the written value onto
     * the tail of the write FIFO. This is the only RW register that should
     * not be written by external device (no hardware protection). Reading
     * this register will return the last written value.
     */
    uint_reg_t wdat       : 8;
    /* Reserved. */
    uint_reg_t __reserved : 56;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved : 56;
    uint_reg_t wdat       : 8;
#endif
  };

  uint_reg_t word;
} UART_TRANSMIT_DATA_t;


/* Type. */

__extension__
typedef union
{
  struct
  {
#ifndef __BIG_ENDIAN__
    /* Number of stop bits, rx and tx */
    uint_reg_t sbits        : 1;
    /* Reserved. */
    uint_reg_t __reserved_0 : 1;
    /* Data word size, rx and tx */
    uint_reg_t dbits        : 1;
    /* Reserved. */
    uint_reg_t __reserved_1 : 1;
    /* Parity selection, rx and tx */
    uint_reg_t ptype        : 3;
    /* Reserved. */
    uint_reg_t __reserved_2 : 57;
#else   /* __BIG_ENDIAN__ */
    uint_reg_t __reserved_2 : 57;
    uint_reg_t ptype        : 3;
    uint_reg_t __reserved_1 : 1;
    uint_reg_t dbits        : 1;
    uint_reg_t __reserved_0 : 1;
    uint_reg_t sbits        : 1;
#endif
  };

  uint_reg_t word;
} UART_TYPE_t;



#endif /* !defined(__ASSEMBLER__) */

#endif /* !defined(__ARCH_UART_H__) */
