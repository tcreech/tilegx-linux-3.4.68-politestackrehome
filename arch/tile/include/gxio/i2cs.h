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
#ifndef _GXIO_I2CS_H_
#define _GXIO_I2CS_H_

#include <gxio/common.h>

#include <hv/drv_i2cs_intf.h>
#include <hv/iorpc.h>

/*
 *
 * An API for manipulating I2C slave interface.
 */

/*
 *
 * The Rshim allows access to the processor's I2C slave interface.
 */

/* A context object used to manage I2C slave resources. */
typedef struct {

	/* File descriptor for calling up to the hypervisor. */
	int fd;

	/* The VA at which our MMIO registers are mapped. */
	char *mmio_base;

	/* The VA at I2CS_READ_DATA. */
	char *rfifo_base;

	/* The VA at I2CS_TFIFO_WRITE_DATA. */
	char *tfifo_base;

	/* The VA at I2CS_FLAG. */
	char *fifo_flag;

	/* The VA at I2CS_INT_VEC_W1TC. */
	char *int_vec;

} gxio_i2cs_context_t;

/* Request I2C slave interrupts.
 *
 *  Request that interrupts be delivered to a tile when the I2C slave's
 *  Receive FIFO is written, or the Transmit FIFO is pop by an I2C master.
 *
 * @param context Pointer to a properly initialized gxio_i2cs_context_t.
 * @param bind_cpu_x X coordinate of CPU to which interrupt will be delivered.
 * @param bind_cpu_y Y coordinate of CPU to which interrupt will be delivered.
 * @param bind_interrupt IPI interrupt number.
 * @param bind_event Sub-interrupt event bit number; a negative value can
 *  disable the interrupt.
 * @return Zero if all of the requested I2C slave FIFO events were
 *  successfully configured to interrupt.
 */
extern int gxio_gpio_cfg_interrupt(gxio_i2cs_context_t * context,
				   int bind_cpu_x,
				   int bind_cpu_y,
				   int bind_interrupt, int bind_event);

#endif /* _GXIO_I2CS_H_ */
