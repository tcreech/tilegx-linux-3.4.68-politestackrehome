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

/*
 * ** NOTE ** this file is commented using Doxygen standards rather than
 * kernel-doc standards since it is used to build Tilera documentation.
 */

/**
 *
 * Defines the kernel-to-userspace API for performing PCI
 * communication between a host machine and a Tile processor.
 */
#ifndef _ASM_TILE_TILEPCI_H
#define _ASM_TILE_TILEPCI_H

#include <linux/ioctl.h>
#include <linux/types.h>

/* A unique ioctl prefix for the Tile PCI subsystem. */

#define TILEPCI_IOC_TYPE 0xB5


/**
 * @mainpage PCIe User Space Communication API
 *
 * The tilepci driver allows user space applications to send and
 * receive data via a PCIe link that connects a host machine to the
 * TILExpress card.  This document describes the kernel APIs that
 * allow user space applications to perform PCIe communication.  For
 * information on configuring the PCIe drivers, see the System
 * Programmer's Guide.
 *
 * All of the PCIe user space APIs are exported via device files in
 * /dev.  The path to these files depends on the type of card being
 * used.  On the host machine, the device files are located in the
 * /dev/tilepciN/ directory, where N identifies which of potentially
 * many TILExpress cards is being used.  TILExpress-20G cards have two
 * PCIe connections to the host; the second link is accessed via
 * /dev/tilepciN-link1/.  On the Tile side, the PCIe links are
 * accessed via the /dev/hostpci/ and /dev/hostpci-link1/ directories.
 *
 * The PCIe driver stack provides the user with several communication
 * mechanisms, each tuned for different usage models:
 *
 * - @ref char_streams provide a simple socket-like communication model
 * between host and Tile applications.  They provide a familiar API
 * and modest performance.
 *
 * - @ref zero_copy provide high-performance communication via an API
 * that allows the programmer to send commands directly to the PCIe
 * driver's scatter / gather engine.  This interface achieves high
 * performance at the expense of a more complicated programming
 * interface.  The zero copy API can also be used to send buffers
 * between multiple Tile chips connected to the same host.
 *
 * - @ref direct_hv enables user space applications to post zero-copy
 * commands directory to the hypervisor's driver stack without the
 * expense of linux system calls.  This interface achieves maximal
 * performance, but it cannot take advantage of process scheduling
 * features like the linux poll() mechanism.  This API is only
 * recommended for use in applications that need to transfer small
 * data packets across PCI.
 *
 * - @ref chip2chip provides zero-copy command queues that transfer
 * data between multiple Tile chips that are attached to the same host
 * PCIe domain.
 *
 * - @ref csr allows a Tile program to export state registers to the
 * host.  This interface allows the host to access those registers at
 * any time, without having to wait for the Tile side to send it a
 * message.  This communication interface is only intended for very
 * low bandwidth communication, for instance startup configuration or
 * runtime status polling.
 *
 * - @ref barmem allows a Tile program to register huge pages that
 * will be mapped to BAR1 of a specially-configured TilePro device.
 * This interface is often used to allow other PCIe devices (capture
 * cards, FPGAs, etc.) to DMA data to and from Tile Processor memory.
 *
 * - @ref packet_queue allows Tile-side NetIO applications to write
 * small packets into a ring buffer in host memory.  This feature is
 * optional and requires that users specify a PCIe driver with 'epp'
 * support in the hvconfig.
 *
 * The final section of this document, @ref other, provides a brief
 * description of the device files used to perform non-communication
 * tasks like booting, device locking, and collecting driver debug
 * information.
 */

/**
 * @defgroup char_streams Character Streams
 *
 * Character streams provide a simple socket-like communication
 * interface between host and tile side applications.  Each PCIe link
 * provides 16 independent, bidirectional character stream devices.
 * Writing to a stream on the host transmits data that can be read on
 * the tile side, and vice versa.
 *
 * Each character stream is represented by a pair of device files -
 * one on the host machine and one in the Tile Linux filesystem.  The
 * device file is named /dev/tilepci0/N on the host and /dev/hostpci/N
 * on Tile Linux. For example, writing to the special file
 * /dev/tilepci0/3 on the host machine transmits data to
 * /dev/hostpci/3 on the Tile Processor, which any Tile Linux process
 * may then read. Similarly, data written to /dev/hostpci/3 is
 * readable from the host machine's /dev/tilepci0/3 special file.
 *
 * The write() system call is used to send data via a character
 * stream.  The read() system call is used to receive data on the
 * other side.  On success, these functions return the number of
 * bytes sent or received.  If an error occurs, the call will return
 * -1 and set errno to one of the following values:
 *
 * - ENXIO: Connection is down.  This can occur if the TILExpress card
 * is reset while the zero-copy command queue file handle is open.
 *
 * - EAGAIN: The operation would block.  This error is only returned
 * if the file handle has the O_NONBLOCK flag set.
 *
 * - EFAULT: The data buffer is not in accessible address space.
 *
 * - EINTR: The request was interrupted by a signal before any
 * commands were posted.
 *
 * The following charts show the aggregate bandwidth achieved by 15
 * character streams running in parallel on the same PCIe link.  At
 * peak bandwidth, this benchmark utilizes approximately 40% of the
 * host CPU cycles.  For small packets, it can consume up to 90% of
 * the host CPU.  Two measurements are shown, one for the single-tile
 * hypervisor PCIe driver, and one for the three-tile hypervisor PCIe
 * driver.
 *
 * @image html char_stream_bw.png "Achievable Bandwidth with Character Streams"
 * @image latex char_stream_bw.png "Achievable Bandwidth with Character Streams" width=6in
 *
 */

/**
 * @defgroup zero_copy Zero-Copy Command Queues
 *
 * The zero-copy command queue API allows high-bandwidth PCIe
 * applications to transmit and receive buffers without any
 * memory-copy overhead in the driver stack.  Using zero-copy
 * communication allows these applications to achieve maximum PCIe
 * communication bandwidth while minimizing the amount of DRAM
 * bandwidth consumed by the driver stack.
 *
 * Unlike the character stream interface, which provides socket-like
 * bidirectional communication via a device file, zero-copy device
 * files provide a unidirectional transfer mechanism.  Each zero-copy
 * device file provides a 'command queue' abstraction.  Writing to the
 * device file posts a send or receive command and reading from the
 * device file returns a structure indicating the completion status of
 * a previously written command.
 *
 * To include the API declarations for the zero-copy API, do:
 *
 * @code
 * #include <asm/tilepci.h>
 * @endcode
 *
 * The MDE provides separate copies of this header file for compiling
 * Tile-side and host-side applications.  To resolve the include when
 * compiling on the host, add '-I\<mde_path\>/include to your compiler
 * flags.  The Tile-side header (in \<mde_path\>/tile/include) is
 * automatically included by tile-cc.
 *
 * @section open Opening Zero-Copy Device Files
 *
 * Each unidirectional, zero-copy command queue is represented as a
 * device file in the /dev file hierarchy.  On the host (x86) machine,
 * the files are named "/dev/tilepci0/h2t/N" for host-to-tile queues
 * and "/dev/tilepci0/t2h/N" for tile-to-host queues, where N is the
 * queue number.  Similarly, on the endpoint (Tile) processor, the
 * files are named "/dev/hostpci/h2t/N" and "/dev/hostpci/t2h/N".  If
 * a connection can be established, opening one of these files will
 * succeed and the new file descriptor is returned. If an error
 * occurs, the call will return -1 and set errno to one of the
 * following values:
 *
 * - ENOENT: No such file or directory. This can occur if the device
 * file name is wrong.
 *
 * - ENXIO: Connection is down.  This can occur if the TILExpress card
 * is reset while the zero-copy command queue file handle is open.
 *
 * The zero-copy command queue can be opened multiple processes
 * simultaneously. By allowing the device to be opened several times,
 * it provides better support to a cluster of cooperating processes
 * sharing the device. On the other hand, the applications need to do
 * careful synchronization among the processes or threads sharing the
 * device.
 *
 * Whether a device file is used to send or receive is determined by
 * the combination of which side opens the file and which direction
 * that file transfers data.  Thus, opening a host-to-tile file on the
 * host machine returns a file handle that can be used to send data.
 * For example, the following host-side code opens the sending side of
 * host-to-tile queue 0.
 *
 * @code
 * int fd = open("/dev/tilepci0/h2t/0", O_RDWR);
 * assert(fd >= 0);
 * @endcode
 *
 * Similarly, the following code opens the receiving end of the queue
 * on the Tile-side.
 *
 * @code
 * int fd = open("/dev/hostpci/h2t/0", O_RDWR);
 * assert(fd >= 0);
 * @endcode
 *
 * @section mmap Allocating Packet Memory
 *
 * The zero-copy API provides the user with direct access to the PCI
 * subsystem's buffer transfer mechanism.  This mechanism requires
 * that all buffers passed to the device be in contiguous physical
 * memory.  Consequently, the user application needs to guarantee that
 * its buffers do not cross page boundaries that could cause a
 * contiguous range of virtual addresses to map to multiple,
 * discontiguous physical pages.
 *
 * Managing contiguous physical memory is a complicated task, so the
 * tilepci zero-copy API provides several tools to help with the job.
 * First, Tile linux uses a 64kB page size, so the user can use
 * memalign() to guarantee that even large packets do not cross a page
 * boundary.  For example, the following code allocates a buffer on a
 * page boundary, so that packet sizes of up to 64kB can be sent or
 * received without crossing a page boundary.
 *
 * @code
 * // On the Tile side, packet buffers can come from any memory region,
 * // but they must not cross a page boundary.
 * void *buffer = memalign(getpagesize(), PACKET_SIZE);
 * assert(buffer != NULL);
 * @endcode
 *
 * On the other hand, many host machines only support a 4kB page size.
 * The tilepci zero-copy driver enables the use of buffers larger than
 * 4kB by allowing the user to mmap() a region of special memory that
 * is guaranteed to be physically contiguous up to 64kB boundaries.
 * The region must be mapped with the MAP_SHARED flag.  The user can
 * then use this memory just as they would on the Tile side, where any
 * 64kB memory region is contiguous.  Thus, host-side programs can use
 * an mmap() call like the following to allocate PCI packet memory to
 * be used with a particular command queue.
 *
 * @code
 * // On the host side, packet buffers must come from the per-channel
 * // 'fast memory' region.
 * void* buffer = mmap(0, PACKET_SIZE, PROT_READ | PROT_WRITE,
 *                     MAP_SHARED, fd, 0);
 * assert(buffer != MAP_FAILED);
 * @endcode
 *
 * @section transfer Transferring Packets
 *
 * The read() and write() system calls are used to transfer data via a
 * zero-copy command queue.  The write() system call posts a command
 * to the queue and the read() system call retrieves completion
 * information for posted commands.  There is a one-to-one correlation
 * between commands posted via write() and completions returned via
 * read().  In addition, each command queue has a limited number of
 * commands that can be outstanding at any time.  The user must use
 * read() to retrieve completions in order to free up command slots so
 * that more commands can be posted via write().
 *
 * @subsection write Posting Commands with write()
 *
 * Writing a ::tilepci_xfer_req_t structure to a zero-copy command
 * queue posts a send or receive command to that channel.  Writing
 * the request structure to the host end of a host-to-tile queue or
 * the tile end of a tile-to-host queue will cause data to be sent.
 * Similarly, writing this structure to the host end of a tile-to-host
 * queue or the tile end of a host-to-tile queue will cause data to be
 * received.
 *
 * Passing an array of ::tilepci_xfer_req_t structures to write() will
 * attempt to post all of those commands to the command queue.  The
 * device will either post all or none of the requests.  If the
 * command(s) cannot be posted, the write() call will return -1 and
 * errno will be set to indicate the cause of the failure:
 *
 * - EINVAL: The write size was not a multiple of
 * sizeof(tilepci_xfer_req_t), or a buffer specified by one of the
 * requests was not physically contiguous.
 *
 * - ENOBUFS: The number of commands set by the ::TILEPCI_IOC_SET_NCMD
 * ioctl() is too small to ever transmit this many requests.
 *
 * - ENXIO: Connection is down.  This can occur if the TILExpress card
 * is reset while the zero-copy command queue file handle is open.
 *
 * - EAGAIN: The operation would block.  This error is only returned
 * if the file handle has the O_NONBLOCK flag set.
 *
 * - EFAULT: Either the command array passed to write() or the data
 * buffer specified by one of the written commands were not in
 * accessible address space.
 *
 * - EINTR: The request was interrupted by a signal before any
 * commands were posted.
 *
 * The following code demonstrates how to post a send command to a
 * zero-copy command queue.
 *
 * @code
 * // Given that we're the host side of a host-to-tile channel, we can
 * // construct a send command and post it to the PCI subsystem.
 * tilepci_xfer_req_t send_cmd = {
 *   .addr = buffer,
 *   .len = PACKET_SIZE,
 *   .flags = TILEPCI_SEND_EOP,
 *   .cookie = 0x12340000,
 * };
 * int rc;
 * while ((rc = write(fd, &send_cmd, sizeof(send_cmd))) != sizeof(send_cmd))
 *   fprintf(stderr, "write() failed: %s\n", strerror(rc));
 * @endcode
 *
 * The following code posts a receive command that will consume the
 * sent data when it arrives at the other side.  Notice that it is
 * almost identical to the code for posting a send command, except
 * that the file handle is on the other side of the uni-directional
 * communication channel and the 'flags' value is different.
 *
 * @code
 * // Given that we're the tile side of a host-to-tile channel, we can
 * // construct a receive command and post it to the PCI subsystem.
 * tilepci_xfer_req_t recv_cmd = {
 *   .addr = buffer,
 *   .len = PACKET_SIZE,
 *   .flags = TILEPCI_RCV_MUST_EOP,
 *   .cookie = 0,
 * };
 * int rc;
 * while ((rc = write(fd, &recv_cmd, sizeof(recv_cmd))) != sizeof(recv_cmd))
 *   fprintf(stderr, "write() failed: %s\n", strerror(rc));
 * @endcode
 *
 * @subsection read Retrieving Completions with read()
 *
 * Once a send or receive command is posted via the write() system
 * call, the user must retrieve a 'completion structure' to determine
 * that the command has finished.  The ::tilepci_xfer_comp_t structure is
 * filled in by the read() syscall in order to return completion
 * status to the user.  This structure includes information about how
 * much data was transfered and whether a receive operation was the
 * end of a packet or overflowed the buffer.  It also includes a word
 * of metadata that is copied from the sender's
 * ::tilepci_xfer_req_t::cookie field to the receiver's
 * ::tilepci_xfer_comp_t::cookie field.
 *
 * If an array of more than one ::tilepci_xfer_comp_t structures is
 * passed to read(), the driver attempts to fill all of the array
 * entries.  If there are not enough completions to fill the array,
 * the behavior depends on whether the file handle is configured as
 * blocking or non-blocking.  If the file handle is non-blocking, the
 * read() method will return immediately with as many completions as
 * are available, or indicate EAGAIN if none are available.  If the
 * file handle is blocking, the driver will block until it can return
 * at least "high water mark" completions.  By default, this means
 * waiting until at least one completion is available; the "high water
 * mark" can be changed via the ::TILEPCI_IOC_SET_MIN_READ_COMPS ioctl().
 *
 * The read() method may return the following errors:
 *
 * - EINVAL: The read size was not a multiple of sizeof(tilepci_xfer_comp_t).
 *
 * - ENXIO: Connection is down.  This can occur if the TILExpress card
 * is reset while the zero-copy command queue file handle is open.
 *
 * - EAGAIN: The operation would block.  This error is only returned
 * if the file handle has the O_NONBLOCK flag set.
 *
 * - EFAULT: The completion array is not in accessible address space.
 *
 * - EINTR: The request was interrupted by a signal before any
 * commands were posted.
 *
 * The following code shows how to use read() to retrieve a completion
 * from a zero-copy command queue.
 *
 * @code
 * // Read back the completion.
 * tilepci_xfer_comp_t comp;
 * while ((rc = read(fd, &comp, sizeof(comp))) != sizeof(comp))
 *   fprintf(stderr, "read() failed: %s\n", strerror(rc));
 * @endcode
 *
 * Note that the example above assumes that the command queue file
 * descriptor is blocking; a non-blocking file handle would return
 * EAGAIN if the completion was not yet available.
 *
 * @section scatter_gather Scatter / Gather Support
 *
 * The PCIe subsystem's scatter / gather support allows an application
 * to combine multiple send buffers into a single outgoing 'packet' or
 * to split an incoming packet across multiple receive buffers.  The
 * gathering and scattering of buffers is controlled by a set of flag
 * bits in the ::tilepci_xfer_req_t structure that is used to post
 * send and receive commands.
 *
 * A send command may include the following flag to control buffer
 * gathering:
 *
 * - ::TILEPCI_SEND_EOP: Indicates that this fragment is the last in a
 * packet.  The data from this command and any previous commands that
 * did not have the bit set will be combined into a single contiguous
 * packet and copied into the receive buffer(s).
 *
 * A receive command may include either of the following flags to
 * control buffer scattering and to drop unwanted data at the end of a
 * packet:
 *
 * - ::TILEPCI_RCV_MAY_EOP: Indicates that this command may be
 * completed with its receive buffer only partially filled if the send
 * command supplying the data specified ::TILEPCI_SEND_EOP.  This
 * allows the application to skip the rest of this command's buffer
 * and receive the next packet in a fresh receive buffer.
 *
 * - ::TILEPCI_RCV_MUST_EOP: Indicates that this receive buffer must
 * be the last one in a packet.  If the sent packet exceeds the
 * receive buffer size, the sender's data will be discarded until
 * reaching the end of a command with the ::TILEPCI_SEND_EOP set.
 *
 * When a send command specifies ::TILEPCI_SEND_EOP, the driver will
 * transfer the sent data until it is either completely consumed by
 * the receiver or truncated via ::TILEPCI_RCV_MUST_EOP.  If the data
 * is completely consumed and the receiver has not yet specified
 * ::TILEPCI_RCV_MAY_EOP or ::TILEPCI_RCV_MUST_EOP, the driver will
 * complete receive commands with a size of zero until one of those
 * flags is posted.
 *
 * The ::tilepci_xfer_comp_t structure contains several flags that
 * indicate how the end-of-packet information was used to fill a
 * receive command:
 *
 * - ::TILEPCI_CPL_EOP: This flag is set if the receive buffer has
 * been filled with the last piece of data that will come from the
 * current packet.  This condition will occur when either the sender
 * indicates ::TILEPCI_SEND_EOP or the receiver forces end-of-packet
 * via ::TILEPCI_RCV_MUST_EOP.
 *
 * - ::TILEPCI_CPL_OVERFLOW: This flag is set when the transmitted data
 * overflows the receive command's buffer.  If ::TILEPCI_CPL_EOP is
 * set, some amount of data has been dropped.  If ::TILEPCI_CPL_EOP is
 * not set, the data may be continued in this channel's next
 * completion.
 *
 * The scatter / gather control flags can be used to implement a
 * variety of packet transfer patterns, as seen in the following
 * figure.  In the 'truncated receive' model, each incoming packet is
 * truncated to the size of a single received buffer by setting the
 * ::TILEPCI_RCV_MUST_EOP flag.  In the 'variable size receive' model,
 * incoming packets fill as many receive buffers as needed.  By
 * setting the ::TILEPCI_RCV_MAY_EOP flag, the receiver causes the
 * driver to skip to a new receive buffer whenever an end-of-packet
 * arrives.  In the 'fixed size receive' model, every incoming packet
 * consumes the same number of receive buffers.  If the incoming data
 * is relatively short, the later receive buffers are returned empty.
 * If the incoming data is longer than the receive buffers, the data
 * is truncated.
 *
 * @image html packet_framing.png "Example uses of the scatter / gather control bits."
 * @image latex packet_framing.png "Example uses of the scatter / gather control bits." width=6in
 *
 * The scatter / gather mechanism can also be used to build a
 * contiguous ring buffer of incoming data.  In this model, each
 * incoming data fragment is stored starting at the address where the
 * previous fragment finished.  This effect is achieved by sending
 * packets with their flag bits clear, so that end-of-packet is never
 * signaled.  The receiver then posts receive buffers with sequential
 * addresses, and the incoming data fills each receive buffer in
 * turn.  Whenever a receive buffer is filled, its completion is
 * returned to the receiving application and the incoming packet is
 * continued into the next receive buffer.  We do recommend setting
 * the ::TILEPCI_RCV_MAY_EOP flag on the receive buffers; while not
 * strictly required, this allows the sending application to force an
 * immediate receive completion by sending a fragment with
 * ::TILEPCI_SEND_EOP.  This capability can be useful at the end of an
 * application run, when the data stream needs to be terminated but
 * may not have evenly filled a receive buffer.
 *
 * @image html ring_framing.png "Constructing a ring buffer by not sending end-of-packet."
 @image latex ring_framing.png "Constructing a ring buffer by not sending end-of-packet." width=6in
 *
 * @section ncmd Limitations on the Number of Commands
 *
 * The PCIe subsystem provides a limited number of 'command slots'
 * that can be used to post commands to the DMA engine.  Each
 * zero-copy command queue file handle is allocated a fraction of
 * these commands slots.  An application is not allowed to post more
 * than this number of commands to the device without first consuming
 * a completion.
 *
 * By default, the maximum number of commands that can be posted to
 * a queue is 4.
 *
 * The number of commands that can be outstanding on a particular
 * command queue can be read via the ::TILEPCI_IOC_GET_NCMD ioctl() and
 * set via the ::TILEPCI_IOC_SET_NCMD ioctl().  The ioctl() returns
 * 0 on success, or returns -1 on error and sets errno to one of the following:
 *
 * - ENOTTY: Unknown ioctl code; usually results from issuing the code
 * on a file handle that is not a zero-copy command queue.
 *
 * - ENXIO: Connection is down.  This condition can occur if the
 * TILExpress card is reset while the zero-copy command queue file
 * handle is open.
 *
 * - ENOBUFS: Could not allocate enough command slots to complete the
 * request.
 *
 * The following code demonstrates the use of ioctl() to set the
 * number of command slots allocated to a particular command queue:
 *
 * @code
 * int ret = ioctl(zc_fd, TILEPCI_IOC_SET_NCMD, 12);
 * @endcode
 *
 * When opened, each command queue is allocated one command slot.
 * Increasing this number will improve performance; four commands per
 * queue is a good setting for many applications.  Applications that
 * need to transfer small buffers at high line rates may required more
 * than four commands outstanding.  The system as a whole has up to
 * ::TILEPCI_CMD_SLOTS slots available for use by zero-copy command
 * queues.
 *
 * @section zc_reset Command Queue Reset
 *
 * Resetting a command queue causes it to immediately complete any
 * previously posted commands so that the user application can restart
 * from a clean state with no commands outstanding.  One common use of
 * this functionality is in handling critical failures; if an
 * application detects such an error, it can reset its communication
 * channel and start over again from a clean slate.
 *
 * A command queue can be reset in two ways.  First, a command queue
 * is automatically reset whenever the last reference to its file is
 * closed.  As a result, applications should be sure to read()
 * completions for all posted commands before closing, since the
 * automatic reset feature can cause some of those commands to be
 * discarded before data transfer can occur.  Second, an application
 * can force command queue reset by issuing the
 * ::TILEPCI_IOC_START_RESET ioctl() to the command queue's file
 * handle.
 *
 * Once a command queue goes into reset mode, all uncompleted commands
 * (on both sides) will be returned with the ::TILEPCI_CPL_RESET bit
 * set.  Applications should always check completions for the reset
 * bit so that the non-resetting side can detect when a channel is put
 * into reset.  For example, if the process attached to the receiving
 * side of a zero-copy command queue dies due to a segmentation
 * violation, the sender side will discover the dead connection when
 * it posts a send command and has its completion returned immediately
 * with the ::TILEPCI_CPL_RESET bit set.
 *
 * After noticing that a command queue is in reset, an application can
 * emerge from reset in two ways.  First, the application could close
 * all references to the device file (including any mapped memory
 * regions) and then reopen the file.  Second, the application can
 * consume any completions it is interested in (since some outstanding
 * commands may have succeeded before reset took effect) and then
 * issue the ::TILEPCI_IOC_FINISH_RESET ioctl().  The 'finish reset'
 * ioctl() will wait for all outstanding commands to complete and
 * discard their completions.  Once it returns, the application can
 * safely continue with the guarantee that no communication operations
 * are outstanding.
 *
 * @section zc_perf Performance Measurements
 *
 * The following charts show the aggregate bandwidth achieved by 15
 * zero copy command queues running in parallel on a single PCIe link.
 * At peak bandwidth, this benchmark utilizes approximately 10% of the
 * host CPU cycles.  For small packets, it can consume up to 40% of
 * the host CPU.  Two measurements are shown, one for the single-tile
 * hypervisor PCIe driver and one for the three-tile hypervisor PCIe
 * driver.
 *
 * @image html zero_copy_bw.png "Achievable Bandwidth with Zero-Copy Devices"
 * @image latex zero_copy_bw.png "Achievable Bandwidth with Zero-Copy Devices" width=6in
 *
 */

/** Get the number of commands that can be issued to a zero-copy stream. */
#define TILEPCI_IOC_GET_NCMD _IOR(TILEPCI_IOC_TYPE, 0x20, __u32)

/** Set the number of commands that can be issued to a zero-copy stream. */
#define TILEPCI_IOC_SET_NCMD _IOW(TILEPCI_IOC_TYPE, 0x21, __u32)


/**
 * Get the command high-watermark.
 * A blocking read request will return when either the requested number
 * of completions are available, or when the number of available
 * completions is at least the high-water-mark.
 */
#define TILEPCI_IOC_GET_MIN_READ_COMPS _IOR(TILEPCI_IOC_TYPE, 0x22, __u32)

/** Set the command high-watermark.
 * A blocking read request will return when either the requested number
 * of completions are available, or when the number of available
 * completions is at least the high-water-mark.
 */
#define TILEPCI_IOC_SET_MIN_READ_COMPS _IOW(TILEPCI_IOC_TYPE, 0x23, __u32)


/**
 * Put the command queue into reset mode.
 * While in reset mode, all commands issued by both the sender and
 * receiver are completed immediately and returned with the
 * ::TILEPCI_CPL_RESET flag.
 */
#define TILEPCI_IOC_START_RESET _IO(TILEPCI_IOC_TYPE, 0x24)

/**
 * Finish resetting a command queue.
 * This ioctl() will wait for any outstanding posted commands to complete,
 * and it will discard their completions.  Thus, when the ioctl() returns,
 * the user is guaranteed that no commands are outstanding and data transfer
 * can resume as if the command queue had just been opened.
 */
#define TILEPCI_IOC_FINISH_RESET _IO(TILEPCI_IOC_TYPE, 0x25)


/** Flag indicating that a send command is the end of a packet.
 * When processing such a command, the DMA engine consumes receive commands
 * until it finds one with either the TILEPCI_REQ_MAY_EOP or
 * TILEPCI_REQ_MUST_EOP flag set.
 */
#define TILEPCI_SEND_EOP (1 << 0)

/**
 * Flag indicating that a receive command can match a send that has
 * the end-of-packet bit set.
 */
#define TILEPCI_RCV_MAY_EOP (1 << 1)

/**
 * Flag indicating that a receive command must match a send that has
 * the end-of-packet bit set.
 */
#define TILEPCI_RCV_MUST_EOP (1 << 2)

/**
 * For receive completions, this flag is set if the receive command was
 * completed as the end of a packet due to either TILEPCI_SEND_EOP
 * or TILEPCI_RCV_MUST_EOP.
 */
#define TILEPCI_CPL_EOP (1 << 16)

/**
 * For receive completions, this flag is set if the transmitted data
 * overflowed the receive command's buffer.  If TILEPCI_CPL_EOP is
 * set, some amount of data has been dropped.  If TILEPCI_CPL_EOP is
 * not set, the data will be continued in this channel's next
 * completion.
 */
#define TILEPCI_CPL_OVERFLOW (1 << 17)

/**
 * For send and receive completions, this flag is set if the command
 * was returned because the command queue has been reset.
 */
#define TILEPCI_CPL_RESET (1 << 18)


/**
 * A user-controlled word of metadata that is copied from a send
 * command to the receive command that consumes its data.
 */
typedef __u32 tilepci_cookie_t;

/**
 * Writing this structure to a zero-copy file handle issues a command
 * to that PCI channel.  Whether the command is a send or receive
 * command depends on the file handle to which it is written.  For
 * example, a host program writing to /dev/tilepci0/t2h/0 would be
 * issuing a receive command.
 */
typedef struct tilepci_xfer_req_s {
	void              *addr;   /**< Address of data buffer. */
	unsigned int       len;    /**< Length of the buffer at 'addr'. */

	/** For send commands, this flag word can contain
	 * TILEPCI_SEND_EOP.  For receive commands, it can contain
	 * TILEPCI_RCV_MAY_EOP or TILEPCI_RCV_MUST_EOP.*/
	unsigned int       flags;

	tilepci_cookie_t   cookie; /**< A metadata value (send cmds only). */
} tilepci_xfer_req_t;

/** Writes to a zero-copy file handle must be a multiple of this size. */
#define TILEPCI_XFER_REQ_SZ	(sizeof(tilepci_xfer_req_t))

/**
 * Reading from a zero-copy file handle returns one or more of
 * these completion data structures.  Each completion structure
 * indicates that a send or receive command issued to that file handle
 * has completed.  In the case of receive command, the completion
 * structure also contains information on the size of the received
 * data, flag bits indicating packet framing, and the metadata
 * 'cookie' from the last send command that put data into the buffer
 * specified by the completing receive command.
 */
typedef struct tilepci_xfer_comp_s {
	void		  *addr;   /**< Buffer from send or receive request.*/
	unsigned int	   len;    /**< Length of received data. */

	/** For receive completions, this word may contain any of
	 * ::TILEPCI_CPL_EOP and ::TILEPCI_CPL_OVERFLOW. */
	unsigned int	   flags;

	/** Metadata word transmitted from sender to receiver. */
	tilepci_cookie_t   cookie;
} tilepci_xfer_comp_t;

/** Reads from a zero-copy file handle must be a multiple of this size. */
#define TILEPCI_XFER_COMP_SZ	(sizeof(tilepci_xfer_comp_t))

/** The maximum buffer size that can be passed to a zero-copy command queue. */
#define TILEPCI_MAX_XFER_LEN	(64 * 1024)

/**
 * Invoking mmap() on a zero-copy file handle maps memory that is
 * guaranteed to be physically contiguous up to this number of bytes.
 * The buffer passed to a transfer request should never cross a file
 * mapping offset that is a multiple of this number of bytes.
 */
#define TILEPCI_MMAP_GRANULARITY (64 * 1024)

/**
 * The number of command slots available for use with zero-copy
 * command queues.  These command slots can be assigned to specific
 * queues via the TILEPCI_IOC_SET_NCMD ioctl.
 */
#define TILEPCI_CMD_SLOTS (1024 - 128)

/**
 */


/**
 * @defgroup chip2chip Chip-to-Chip Zero Copy
 *
 * The chip-to-chip zero-copy command queue API is very similar to the
 * host-to-tile and tile-to-host interfaces described in @ref
 * zero_copy.  This section describes the communication topology
 * enabled by chip-to-chip transfers and documents the differences
 * between the chip-to-chip API and the generic zero-copy API.
 *
 * @section topology Chip-to-Chip Communication Topology
 *
 * The host driver constructs an all-to-all topology between all Tile
 * PCI links in the system.  TILExpress-64 boards have a single PCI
 * link connected to the host and TILExpress-20G boards have two.
 * Each host-connected PCI shim is assigned an index from zero to
 * "num_tile_endpoints - 1".  Indices are assigned according to the
 * order in which the host machine's BIOS probed the TILExpress
 * boards, and PCIe shim 0 on a TILExpress-20G immediately precedes
 * shim 1 on that same board.
 *
 * Each PCI endpoint driver has an array of chip-to-chip send ports
 * and receive ports.  The host driver connects these ports so that
 * the send or receive port index indicates the endpoint link to which
 * the port is connected.  Thus, send port 2 on link 0 sends data to
 * receive port 0 on link 2, and so forth.  The figure below
 * illustrates a more complicated example, including filenames.
 *
 * @image html all_to_all.png "All-to-All Topology in a Two-Board System"
 * @image latex all_to_all.png "All-to-All Topology in a Two-Board System" width=6in
 *
 * Applications running on the TILExpress boards can determine the
 * host link index for each of their PCI shims by reading the
 * /dev/hostpci(-link1)/info file and looking for the
 * "HOST_LINK_INDEX" field.  Similarly, a host-side application can
 * determine the link index for each board by reading the same field
 * in /dev/tilepciN(-link1)/info.
 *
 * @section open_c2c Opening Zero-Copy Device Files
 *
 * As with host-to-tile and tile-to-host command queues, each
 * unidirectional, zero-copy command queue is represented as a device
 * file in the /dev file hierarchy.  Send ports are named
 * "/dev/hostpci/c2c_send/N" and receive ports are named
 * "/dev/hostpci/c2c_recv/N", where N is the port number on that PCI
 * link.  Opening one of these files will only succeed if the host
 * driver has created a connection between that port and some remote
 * port.  If no connection is available (for example because there is
 * no remote card at the specified index), the open() call will fail
 * with ENXIO.
 *
 * @section mmap_c2c Allocating Packet Memory
 *
 * Allocating packet memory for use with chip-to-chip zero copy
 * command queues is exactly the same as with host-to-tile or
 * tile-to-host command queue.  Since this interface is only available
 * on the tile side, packets can come from any memory region, but they
 * must not span a page boundary.  For more information, see @ref
 * mmap.
 *
 * @section transfer_c2c Transferring Packets
 *
 * The interface for transmitting packets via a chip-to-chip channel
 * is nearly identical to the standard zero-copy command queue API.
 * ::tilepci_xfer_t structures are posted via the write() syscall and
 * ::tilepci_xfer_comp_t structures are returned via read().  For more
 * information, see @ref transfer.
 *
 * Chip-to-chip zero copy does have a few limitations relative to
 * host-to-tile and tile-to-host command queues.  In particular:
 *
 * - The number of outstanding commands on a chip-to-chip command
 * queue cannot exceed ::TILEPCI_MAX_C2C_NCMD.  Attempting to set the
 * ncmd limit to a higher value via the ::TILEPCI_IOC_SET_NCMD ioctl()
 * will fail with EINVAL.
 *
 * - Chip-to-chip command queues do not support scatter / gather.  As
 * a result, the ::TILEPCI_SEND_EOP, ::TILEPCI_RCV_MAY_EOP, and
 * ::TILEPCI_RCV_MUST_EOP flags are ignored.  Instead, the driver
 * treats all send commands as if the ::TILEPCI_SEND_EOP flag was set,
 * and all receive commands as if the ::TILEPCI_RCV_MUST_EOP flag were
 * set.  As a result, each send buffer is copied into exactly one
 * receiver buffer.  Applications should still set the
 * ::TILEPCI_SEND_EOP and ::TILEPCI_RCV_MUST_EOP flags, though, in
 * case future versions of the implementation add scatter / gather
 * support.
 *
 * - Chip-to-chip connections will be reset if the host decides to
 * reboot the Tile processor on the other side of the connection.  In
 * this case, all outstanding requests will be completed with the
 * ::TILEPCI_CPL_LINK_DOWN flag set, indicating that there is no link
 * available.  This flag is distinct from the ::TILEPCI_CPL_RESET flag
 * in that the latter indicates a link is available but the other side
 * has reset the connection and not yet restarted it.  The
 * ::TILEPCI_IOC_FINISH_RESET ioctl() has no effect on connections
 * that are down due to link loss; we suggest that the application
 * deal with this situation by closing the chip-to-chip file handle
 * and polling on open() until the link comes back up.
 *
 * @section c2c_perf Chip-to-Chip Bandwidth Measurements
 *
 * The following chart shows the bandwidth achievable with a single
 * tile issuing chip-to-chip zero-copy commands via the @ref direct_hv
 * API.  These measurements were collected on a TILExpress-20G board
 * transferring packets from shim 0 to shim 1 through the onboard PLX
 * PCIe switch.
 *
 * @image html c2c_direct_hv_bw.png "Chip-to-Chip Bandwidth Using Zero-Copy Direct-to-HV API"
 * @image latex zero_copy_bw.png "Chip-to-Chip Bandwidth Using Zero-Copy Direct-to-HV API" width=6in
 */



/**
 * For send and receive completions, this flag is set if the
 * specified chip-to-chip command queue cannot process transactions
 * because the link is down.  If this flag is set, ::TILEPCI_CPL_RESET
 * will also be set.
 */
#define TILEPCI_CPL_LINK_DOWN (1 << 19)

/**
 * The maximum number of commands that can be posted to a
 * chip-to-chip queue.
 */
#define TILEPCI_MAX_C2C_NCMD 16

/**
 */

/**
 * @defgroup csr Configuration and Status Register Emulation
 *
 * The CSR registers provide a window of memory-mapped I/O space
 * that is visible to both the host and the tile-side application
 * software. The applications can define 32-bit registers that are
 * shared by both the host and the tile. Typically the application
 * can use the region to post status registers or implement 'mode
 * registers' that are set to a particular value according to the
 * current state of the application.
 *
 * While both the host and the tile-side applications can read
 * the CSR registers at any time, a write notification can be
 * sent by one application to the other end after an update to the
 * CSR region is made. Applications can use poll/select system
 * calls to wait for the CSR changes.
 *
 * @section host Host Side API
 *
 * The CSR region is represented as a device file in the /dev file
 * hierarchy on both the host and the tile side. On the host (x86)
 * machine, the file is named "/dev/tilepci0/csr".
 *
 * To access the CSR registers on the host machine, it is necessary
 * to open the device file to obtain the file handle, as shown below.

 * @code
 * int fd = open("/dev/tilepci0/csr", O_RDWR);
 * assert(fd >= 0);
 * @endcode
 *
 * The tilepci driver supports mapping the CSR region directly into
 * the user application address space. The application can map in
 * the CSR region by invoking the mmap system call, as shown below.

 * @code
 * void* buffer = mmap(0, PCIE_CSR_REG_NUM * sizeof(int),
 *                     PROT_READ | PROT_WRITE,
 *                     MAP_SHARED, fd, 0);
 * assert(buffer != MAP_FAILED);
 * @endcode
 *
 * where the constant PCIE_CSR_REG_NUM is the number of CSR registers
 * defined by the applications. Note that the mapped memory size must
 * not be larger than TILE_CSR_MEMORY_PAGE_SIZE.
 *
 * The host application can be alerted of tile-side events by an
 * interrupt sent from the tile to the host. The poll and ioctl system
 * calls can be used together to obtain the tile interrupt conditions,
 * represented by the low 3 bytes of a 32-bit word. The 24-bit field is
 * completely defined by the applications and the tilepci driver just
 * forwards it from the tile application to the host application.
 *
 * @code
 * struct pollfd poll_fd;
 *
 * poll_fd.fd = fd;
 * poll_fd.events = POLLIN;
 *
 * if (!( ret = poll(&poll_fd, 1, 5000))) {
 *   fprintf(stderr, "poll timed out!\n");
 *   exit(1);
 * } else if (ret < 0) {
 *   fprintf(stderr, "poll failed: %s\n", strerror(errno));
 *   exit(1);
 * }
 * if (ioctl(fd, TILEPCI_IOC_GET_TILE_TO_HOST_INTR, &intr) < 0) {
 *   fprintf(stderr, "Host ioctl failed: %s\n", strerror(errno));
 *   exit(1);
 * }
 * printf("Host intr flag 0x%x\n", intr);
 * @endcode
 *
 * where TILEPCI_IOC_GET_TILE_TO_HOST_INTR is the ioctl command to
 * get the tile interrupt conditions, represented in the low 24 bits.
 *
 * @section tile Tile Side API
 *
 * Similar to the host side, the CSR region on the tile side is
 * represented as a device file named "/dev/hostpci/csr".
 *
 * To access the CSR registers on the tile side, it is necessary
 * to open the device file to obtain the file handle, as shown below.

 * @code
 * int csr_fd = open("/dev/hostpci/csr", O_RDWR);
 * assert(csr_fd >= 0);
 * @endcode
 *
 * The tilepci driver supports mapping the CSR region directly into
 * the user application address space. The application can map in
 * the CSR region by invoking the mmap system call, as shown below.

 * @code
 * void* buffer = mmap(0, PCIE_CSR_REG_NUM * sizeof(int),
 *                     PROT_READ | PROT_WRITE,
 *                     MAP_SHARED, csr_fd, 0);
 * assert(buffer != MAP_FAILED);
 * @endcode
 *
 * where the constant PCIE_CSR_REG_NUM is the number of CSR registers
 * defined by the applications. Note that the mapped memory size must
 * not be larger than TILE_CSR_MEMORY_PAGE_SIZE.
 *
 * When the host application writes a value to a CSR register,
 * a write notification is posted to the tile application. This
 * write notification contains the register offset within the CSR
 * range and the write value. The tile application can use the poll
 * system call to wait for the write notification and use the read
 * system call to read it once it arrives, as show below.
 *
 * @code
 * pcie_csr_write_notify_t csr_notify;
 * struct pollfd poll_fd;
 *
 * poll_fd.fd = csr_fd;
 * poll_fd.events = POLLIN;
 *
 * if (!( ret = poll(&poll_fd, 1, 5000))) {
 *   fprintf(stderr, "poll timed out!\n");
 *   exit(1);
 * } else if (ret < 0) {
 *   fprintf(stderr, "poll failed: %s\n", strerror(errno));
 *   exit(1);
 * }
 * if (read(csr_fd, &csr_notify, sizeof(pcie_csr_write_notify_t)) !=
 *                                 sizeof(pcie_csr_write_notify_t)) {
 *   fprintf(stderr, "read failed: %s\n", strerror(errno));
 *   exit(1);
 * }
 * @endcode
 *
 * Once the tile application receives the host write notification,
 * it is the tile application's responsibility to materialize the
 * change to the CSR memory. The pcie driver does not modify the
 * CSR memory. So the tile application is expected to perform the
 * following operation:
 *
 * @code
 * ((uint32_t*)buffer)[csr_notify.offset >> 2] = csr_notify.value;
 * @endcode
 *
 * Whenever the tile application needs to alert the host application
 * of any events, for example when it updates values in the CSR
 * register range, it sets a bitmask of 24 "interrupt-type" bits in
 * the low 3 bytes of an integer and triggers a host interrupt by
 * calling the ioctl system call as shown below.
 *
 * @code
 * int intr = 0x100000;
 * ioctl(csr_fd, TILEPCI_IOC_TILE_TO_HOST_INTR, &intr);
 * @endcode
 *
 * where TILEPCI_IOC_TILE_TO_HOST_INTR is the ioctl command to
 * trigger the host interrupt and send the tile interrupt conditions
 * contained in the low 24 bits of the ioctl argument.
 *
 * @section csr_reboot Chip Reset and the CSR Device File
 *
 * Because the csr device maps the TILE processor's MMIO registers
 * directly into userspace, the host driver will not allow a card to
 * reboot until all csr file handles are unmapped and closed.  Any
 * attempt to open the boot device file will fail and set errno to
 * EBUSY.  This prevents bugs that could result from accessing the
 * MMIO registers while the TILE processor is being rebooted.
 */

/** The mapping page size for the CSR backing memory. */
#define TILE_CSR_MEMORY_PAGE_SIZE	(64 * 1024)

/**
 */

/**
 * @defgroup packet_queue Packet Queue Support
 *
 * The PCIe driver stack provides optional support for NetIO-style
 * packet egress over PCIe.  Packets are sent from Tile userspace via
 * the netio_send_pcie_packet() API.  The driver stack then writes the
 * packets into a ring buffer in host memory.  Each entry in the entry
 * in the ring buffer has the same size (by default, 256 bytes), and
 * each new packet is deposited into the next available ring buffer
 * entry.
 *
 * The user can gain access to the packet_queue interface by opening
 * the 'packet_queue' device file.  The open attempt will fail if the
 * feature is not supported on the specified port or if the Tile
 * processor has not been booted.  For example:
 *
 * @code
 * const char name[64] = "/dev/tilepci0/packet_queue";
 * int epp_fd = open(name, O_RDWR);
 * if (epp_fd < 0)
 * {
 *   fprintf(stderr, "Failed to open '%s': %s\n", name, strerror(errno));
 *   exit(EXIT_FAILURE);
 * }
 * @endcode
 *
 * The user application can configure the entry size and number of
 * entries in the ring buffer via the
 * ::TILEPCI_IOC_SET_PACKET_QUEUE_BUF ioctl().  This ioctl will fail
 * if the parameters are not powers of two, if memory allocaiton
 * failed, or if the application has already invoked mmap() on the
 * packet_queue device.  For example:
 *
 * @code
 * tilepci_packet_queue_info_t buf_info = {
 *   .num_elems = 8192,
 *   .buf_size = 128,
 * };
 *
 * int err = ioctl(epp_fd, TILEPCI_IOC_SET_PACKET_QUEUE_BUF, &buf_info);
 * if (err < 0)
 * {
 *   fprintf(stderr, "Failed TILEPCI_IOC_SET_PACKET_QUEUE_BUF: %s\n",
 *           strerror(errno));
 *   exit(EXIT_FAILURE);
 * }
 * @endcode
 *
 * Once the ring buffer parameters have been set, the host application
 * can use mmap() calls to map the device control registers and the
 * ring buffer into application memory space.  The buffer head and
 * tail pointers are mapped in a ::pcie_packet_queue_indices_reg_t
 * structure at mmap file offset
 * ::TILEPCI_PACKET_QUEUE_INDICES_MMAP_OFFSET.  The ring buffer (which
 * is of variable size) is mapped starting at mmap file offset
 * ::TILEPCI_PACKET_QUEUE_BUF_MMAP_OFFSET.  For example:
 *
 * @code
 * // mmap the EPP register space.
 * pcie_packet_queue_indices_reg_t* epp_regs;
 * epp_regs =(pcie_packet_queue_indices_reg_t*)
 *   mmap(0, sizeof(pcie_packet_queue_indices_reg_t),
 *        PROT_READ | PROT_WRITE,
 *        MAP_SHARED, epp_fd, TILEPCI_PACKET_QUEUE_INDICES_MMAP_OFFSET);
 * assert(epp_regs != MAP_FAILED);
 *
 * //  mmap the receive queue region.
 * void* buffer = mmap(0, host_ring_buffer_size, PROT_READ | PROT_WRITE,
 *                     MAP_SHARED, epp_fd,
 *                     TILEPCI_PACKET_QUEUE_BUF_MMAP_OFFSET);
 * assert(buffer != MAP_FAILED);
 * @endcode
 *
 * Once the control registers and ring buffer are mapped into
 * application address space, the program can watch for changes to
 * pcie_packet_queue_indices_reg_t::write_index to see when packets
 * are posted.  Once a packet is consumed, the application should
 * notify the driver that the ring slot is free again by writing
 * pcie_packet_queue_indices_reg_t::read_index.
 *
 * Because the packet_queue device maps the TILE processor's MMIO
 * registers directly into userspace, the host driver will not allow a
 * card to reboot until all packet_queue file handles are unmapped and
 * closed.  Any attempt to open the boot device file will fail and set
 * errno to EBUSY.  This prevents bugs that could result from
 * accessing the MMIO registers while the TILE processor is being
 * rebooted.
 */


/**
 * Structure to hold the fast PCIe packet queue information, used
 * by application to set the packet queue buffer size via the
 * ::TILEPCI_IOC_SET_PACKET_QUEUE_BUF ioctl().
 */
typedef struct tilepci_packet_queue_info_s {
	__u32 num_elems; /**< Number of elements in the queue. */
	__u32 buf_size;  /**< Size of each element, must be power of 2.*/
} tilepci_packet_queue_info_t;

/** Set the number and size of elements in the recieve ring buffer. */
#define TILEPCI_IOC_SET_PACKET_QUEUE_BUF \
	_IOW(TILEPCI_IOC_TYPE, 0x32, tilepci_packet_queue_info_t*)

/** Offset of the receive ring buffer indices in device mmap space. */
#define TILEPCI_PACKET_QUEUE_INDICES_MMAP_OFFSET 0

/** Offset of the receive ring buffer in device mmap space. */
#define TILEPCI_PACKET_QUEUE_BUF_MMAP_OFFSET 4096

/**
 * Structure to hold the fast pcie packet queue indices registers,
 * used via mmap() the /dev/tilepciN/packet_queue device.
 */
typedef struct pcie_packet_queue_indices_reg_s {
	/** Ring buffer write index. */
	volatile __u32 write_index;
	/** Ring buffer read index. */
	volatile __u32 read_index;
} pcie_packet_queue_indices_reg_t;

/**
 */

/**
 * @defgroup barmem BAR-Mapped Memory
 *
 * Specially-configured TilePro devices can map huge memory pages into
 * the PCIe address space using the PCIe shim's second "base address
 * register" (BAR).  A PCIe BAR defines a region of PCIe address space
 * to which read and write requests can be routed.  If a Tile
 * application registers memory pages to back BAR1, the PCIe subsystem
 * will automatically apply incoming read and write requests to those
 * pages.
 *
 * @section barmem_enable Enabling BAR1
 *
 * By default, the Tile Processor comes out of reset with BAR1
 * disabled.  On TILExpress boards, a special boot block in the
 * onboard SPI ROM can be used to enable BAR1 when the host machine
 * comes out of reset.  The host's BIOS will then probe both BAR0
 * (used for PCIe character streams, zero copy, etc) and BAR1.
 *
 * The following steps will enable the SPI ROM on TILExpressPro boards
 * and program it to enable BAR1 with a specified window size.
 *
 * - Boot the host system and use sbim's --pcie[0,1]-bar-1-size switch
 * to set the size of BAR1 on each PCI link.  sbim requires that you
 * install a bootrom image as well.  This can be either a full bootrom
 * or a "PCI prebooter" that can be used to do an initial boot from
 * ROM but still accept the main bootrom from tile-monitor via a PCI
 * port.  For more information, see the sbim man page.  The following
 * command shows how to enable 128 megabyte BAR1 regions on both links
 * of a TILExpressPro-20G card:
 *
 * @verbatim
 [prompt]$ tile-monitor --pci --mount-tile /boot \
    --run -+- sbim --install-prebooter /boot/pci_preboot.bin \
    --pcie0-bar1-size 128M --pcie1-bar1-size 128M --yes -+- --quit
 @endverbatim
 *
 * - Next, configure the board's DIP switches to boot from SPI ROM.
 * On TILExpressPro-64 boards, this requires opening DIP switch 3 and
 * closing DIP switch 4.  On TILExpressPro-20G boards, this requires
 * closing DIP switch 5.  For more information, see the "Jumpers and
 * DIP Switches" section of UG300: TILExpress-64 Card User's Guide or
 * UG301: TILExpress-20G Card User's Guide.
 *
 * - Reboot the host system.  Run '/sbin/lspci -v' to determine that
 * BAR1 has been enabled.  Look for an entry like the following; the
 * second "Memory at..." line indicates that BAR1 is enabled.
 *
 * @verbatim
 [prompt]$ /sbin/lspci -v
 ...
 09:00.0 Class ffff: Unknown device 1a41:0002 (prog-if ff)
	Subsystem: Unknown device 1862:0001
	Flags: fast devsel, IRQ 217
	Memory at dd800000 (64-bit, non-prefetchable) [size=8M]
	Memory at d0000000 (64-bit, non-prefetchable) [size=128M]
	Capabilities: <available only to root>
 @endverbatim
 *
 *
 * @section barmem_tile Mapping Tile-Side Memory to BAR1
 *
 * The Tile-side Linux PCIe driver provides a special "barmem" device
 * file for mapping huge pages into BAR1.  To map huge pages, open the
 * "/dev/hostpci/barmem" file, fill in a ::tilepci_barmem_map_t to
 * specify which VAs you wish to map, and issue the
 * ::TILEPCI_IOC_BARMEM_MAP ioctl() to perform the mapping.  For
 * example:
 *
 * Once a program has mapped huge pages to DRAM, it can read and write
 * values as it chooses.  However, be aware that incoming PCIe read
 * and write requests modify values in DRAM, so be sure to flush
 * written values to DRAM and invalidate data pulled into a cache by
 * load instructions.
 *
 * @code
 * // Open the 'barmem' device file.
 * int fd = open("/dev/hostpci/barmem", O_RDWR);
 * assert(fd >= 0);
 *
 * // Allocate some huge pages.
 * size_t size = HPAGE_SIZE * 3;
 * alloc_attr_t attr = ALLOC_INIT;
 * alloc_set_huge(&attr);
 * void* mem = alloc_map(&attr, size);
 * assert(mem != 0);
 *
 * // Fill in the tilepci_barmem_map_t structure and ioctl() it.
 * tilepci_barmem_map_t map = {
 *   .va = mem,
 *   .size = size,
 *   .offset = 0,
 * };
 * int err = ioctl(fd, TILEPCI_IOC_BARMEM_MAP, &map);
 * if (err < 0)
 * {
 *   fprintf(stderr, "write() returned %d (%s)\n", errno, strerror(errno));
 *   exit(EXIT_FAILURE);
 * }
 *
 * // Write a string to our BAR-mapped memory, then flush it to DRAM so
 * // that incoming PCIe read requests to BAR1 will find the new value.
 * strcpy(mem, "Hello world\n");
 * tmc_mem_flush(mem, 64);
 * @endcode
 *
 * @section barmem_host Reading and Writing From the Host
 *
 * In general, the BAR1 memory-mapped window is intended for use
 * communicating with other PCIe endpoint devices (video capture
 * cards, FPGAs, etc.).  However, the host-side Linux PCIe driver does
 * provide a mechanism for mapping BAR1 into the VA space of a user
 * program running on the host machine.  Be aware that MMIO load and
 * store operations from the host machine are often very inefficient,
 * so the zero-copy and character stream devices are a
 * higher-performance (and safer!) mechanism for transferring data
 * between the host and TILExpress card.
 *
 * To map BAR1 into host userspace, simply mmap() the
 * "/dev/tilepciN/barmem" file.  This file must be mapped with the
 * MAP_SHARED flag.  For example:
 *
 * @code
 * // Open the device file.
 * int memfd = open("/dev/tilepci0/barmem", O_RDWR);
 * assert(memfd >= 0);

 * // mmap 4kB of the BAR1 memory into our VA space.
 * void* shared_mem = mmap(0, 4 * 1024, PROT_READ | PROT_WRITE,
 *                         MAP_FILE | MAP_SHARED, memfd, 0);
 * if (shared_mem == MAP_FAILED)
 * {
 *   fprintf(stderr, "Failed to map memory: (%s)\n", strerror(errno));
 *   exit(EXIT_FAILURE);
 * }
 * printf("Mapped memory to %p\n", shared_mem);
 * @endcode
 */

/**
 * A structure used to specify a VA range to be mapped to a
 * particular offset within PCIe BAR1.  The va, size, and offset
 * parameters must be huge-page aligned.
 */
typedef struct tilepci_barmem_map_s {
	void *va;         /**< Virtual Address of first huge page. */
	__u32 size;       /**< Size of VA range, in bytes. */
	__u32 offset;     /**< Offset within BAR1, in bytes. */
} tilepci_barmem_map_t;

/**
 * Pass a tilepci_barmem_map_t address to this ioctl on
 * /dev/hostpci/barmem in order to map it into the BAR1 memory mapping
 * window.  Returns 0 on success or non-zero on failure, in which case
 * errno is set to EINVAL if any parameters were not properly aligned
 * or the requested VA range was not composed of huge pages, EFAULT if
 * the specified va range is not accessible to the caller, and
 * EADDRINUSE if the requested offset was already mapped.  May also
 * set EIO for unexpected IO driver errors.
 */
#define TILEPCI_IOC_BARMEM_MAP \
	_IOW(TILEPCI_IOC_TYPE, 0x40, tilepci_barmem_map_t*)


/**
 * Pass a tilepci_barmem_map_t address to this ioctl on
 * /dev/hostpci/barmem in order to unmap it from the BAR1 memory
 * mapping window.  The 'va' pointer is ignored when making unmap
 * requests.  Returns 0 on success or non-zero on failure, in which
 * case errno is set to EINVAL if any parameters were not properly
 * aligned.
 */
#define TILEPCI_IOC_BARMEM_UNMAP \
	_IOW(TILEPCI_IOC_TYPE, 0x41, tilepci_barmem_map_t*)


/**
 */

/**
 * @defgroup other Other Device Files
 *
 * This section provides brief descriptions of the other files that
 * are found in the tilepci /dev hierarchy.
 *
 * The 'boot' file is only available on the host and is located at
 * /dev/tilepciN/boot.  Writing a bootrom to this device file causes
 * the driver to reset the TILExpress board and reboot it using the
 * written file.  The preferred mechanism for booting a board is via
 * 'tile-monitor --pci', but applications can also boot by simply
 * cat'ing a bootrom to the boot file, for example by running 'cat
 * my_file.bootrom > /dev/tilepci0/boot'.  Rebooting a card causes all
 * the character stream and zero-copy connections to that card to be
 * reset.
 *
 * The 'lock' file, available on the host at /dev/tilepciN/lock, can
 * be used to guarantee that only one process can access the boot file
 * at a time.  When a process opens the 'lock' file, it prevents any
 * other process from opening the 'boot' file.  This locking mechanism
 * is particularly helpful for preventing any other process in the
 * system from opening the boot file and resetting the board while the
 * lock holder is still communicating with a program running on the
 * board.
 *
 * Finally, the PCIe driver stack provides a set of files for
 * accessing driver debug information.  These debug files are
 * available at /dev/tilepciN/debug/... on the host and
 * /dev/hostpciN/debug/... on the Tile side.  There are separate files
 * for accessing debug information about character streams, zero-copy
 * command queues, and the hypervisor's dedicated tile driver state.
 * In general, these files are intended for use by driver developers,
 * although they could be occassionally useful to application
 * developers.  The file output is formatted to be easily manipulated
 * with tools like 'grep'.  For example, the command 'cat
 * /dev/tilepci0/debug/char_stream | grep stream2' collects all
 * the debuging information for the character stream device
 * /dev/tilepci0/2.
 */

/**
 * @defgroup host_subset Connecting Tile Root Complex to Tile Endpoint
 *
 * In systems that connect two or more Tile processors via PCIe, one
 * of those Tile processors must serve as a root complex port.  The
 * root complex is responsible for probing the PCIe topology and
 * initializing the other devices.  Tilera provides a
 * tilepci_host_subset driver that can be used to drive Tile endpoints
 * from a Tile root complex port.  This driver is cut-down version of
 * the full x86 host driver, including just enough code to boot Tile
 * endpoints and communicate via memory transactions.
 *
 * The following steps will install the tilepci_host_subset driver:
 *
 * - Use tile-monitor --mount-tile to mount /lib/modules and /usr/sbin.
 * - Run /usr/sbin/tilepci-host-subset-init.
 *
 * This will create /dev/tilepciN directories for each Tile PCIe
 * endpoint connected to the Tile PCIe root complex ports.  Unlike the
 * full host driver, the tilepci_host_subset driver does not attempt
 * to detect TILExpress boards.  As such, each Tile PCIe port in the
 * system gets a separate /dev/tilepciN directory; the second link on
 * a TILEncore board will be /dev/tilepci1, not /dev/tilepci0-link1.
 *
 * Each /dev/tilepciN directory includes the 'boot', 'rshim', and
 * 'barmem' files.  These provide essentially the same interface as in
 * the full x86 host driver, except that the 'barmem' file does not
 * support mmap'ing the remote Tile processor's BAR1 memory mapping.
 * Instead, pread() and pwrite() can be used to perform 1, 2, or 4
 * byte reads and writes to an indicated offset within BAR1.
 */

/** Structure passed to the /dev/tilepciN/barmem
 * ::TILEPCI_IOC_MAP_TO_PCI_BUS ioctl in order to map memory pages
 * onto the PCI bus. */
struct tilepci_bus_map_params {
	unsigned long va;             /**< Page VA to be mapped. */
	unsigned long size;           /**< Size of page. */

	/** If ioctl returns 0, filled with the PCI address. */
	unsigned long long bus_addr;
};

/** ioctl used to map pages to the PCI bus. */
#define TILEPCI_IOC_MAP_TO_PCI_BUS _IOWR(TILEPCI_IOC_TYPE, 0x40,	\
					 struct tilepci_bus_map_params*)


/******************************************************************/
/*               Undocumented, Unstable Interfaces                */
/******************************************************************/


/* Channel reset. */
#define TILEPCI_IOC_CHANNEL_RESET _IOW(TILEPCI_IOC_TYPE, 0x10, __u32)

/* Trigger tile-to-host interrupt. */
#define TILEPCI_IOC_TILE_TO_HOST_INTR _IOW(TILEPCI_IOC_TYPE, 0x11, __u32)

/* Receive tile-to-host interrupt. */
#define TILEPCI_IOC_GET_TILE_TO_HOST_INTR _IOR(TILEPCI_IOC_TYPE, 0x12, __u32)


/* An ioctl that the DV code can use to inform us of a reset via rshim. */
#define TILEPCI_IOC_RSHIM_RESET _IO(TILEPCI_IOC_TYPE, 0x00)

/* Set the size of DMAs to be done by the dedicated tile benchmark. */
#define TILEPCI_IOC_SET_BENCH_SIZE _IOW(TILEPCI_IOC_TYPE, 0x01, __u32)

/* Set the number of DMAs to be done by the dedicated tile benchmark. */
#define TILEPCI_IOC_SET_BENCH_RUNS _IOW(TILEPCI_IOC_TYPE, 0x02, __u32)

/* Run the dedicated tile benchmark. */
#define TILEPCI_IOC_RUN_BENCH _IO(TILEPCI_IOC_TYPE, 0x03)

/* Clear dedicated tile profile counters, if HV compile enabled them. */
#define TILEPCI_IOC_PROFILE_CLEAR _IO(TILEPCI_IOC_TYPE, 0x04)

/* Print dedicated tile profile counters, if HV compile enabled them. */
#define TILEPCI_IOC_PROFILE_PRINT _IO(TILEPCI_IOC_TYPE, 0x05)

/* Run the dedicated tile benchmark against BAR1 on the other link. */
#define TILEPCI_IOC_RUN_BAR1_BENCH _IO(TILEPCI_IOC_TYPE, 0x06)

/** Get the direct-to-HV ZC queue info from the Linux driver. */
#define TILEPCI_IOC_GET_HV_ZC _IOR(TILEPCI_IOC_TYPE, 0x26, __u32)

/** Register the packet memory for direct-to-HV ZC transfer. */
#define TILEPCI_IOC_IOMEM_REGISTER _IOW(TILEPCI_IOC_TYPE, 0x27, __u32)

/** Reserve a channel for use by the direct-to-HV API. */
#define TILEPCI_IOC_CHANNEL_REGISTER _IOW(TILEPCI_IOC_TYPE, 0x28, __u32)


/*
 * Set the number of microseconds that a completion notification could
 * be delayed by the PCIe hardware.
 */
#define TILEPCI_IOC_SET_DELAY_USEC _IOW(TILEPCI_IOC_TYPE, 0x30, __u32)

/*
 * Set the maximum number of completion notifications that could be
 * delayed by the PCIe hardware.
 */
#define TILEPCI_IOC_SET_DELAY_COUNT _IOW(TILEPCI_IOC_TYPE, 0x31, __u32)


/* Define how many of each interface we provide. */

/* The number of character stream devices. */
#define TILEPCI_NUM_CHAR_STREAMS 16

/* The number of host-to-tile zero-copy queues. */
#define TILEPCI_NUM_ZC_H2T_CHAN 64

/* The number of tile-to-host zero-copy queues. */
#define TILEPCI_NUM_ZC_T2H_CHAN 64

/* The number of chip-to-chip zero-copy send queues. */
#define TILEPCI_NUM_C2C_SEND_CHAN 32

/* The number of chip-to-chip zero-copy receive queues. */
#define TILEPCI_NUM_C2C_RECV_CHAN 32

/* The number of DMA read queues. */
#define TILEPCI_NUM_DMA_READ_CHAN 32

/* The number of DMA write queues. */
#define TILEPCI_NUM_DMA_WRITE_CHAN 32


/* Define minor numbers for all the linux device files. */

/* Minor number of first character stream device. */
#define TILEPCI_FIRST_CHAR_STREAM_MINOR 0

/* Minor number of the boot device. */
#define TILEPCI_BOOT_MINOR 16

/* Minor number of the lock device. */
#define TILEPCI_LOCK_MINOR 17

/* Minor number of the ctl device. */
#define TILEPCI_CTL_MINOR 18

/* Minor number of the CSR device. */
#define TILEPCI_CSR_MINOR 19

/* Minor number of character stream debug information. */
#define TILEPCI_CHAR_STREAM_DEBUG_MINOR 20

/* Minor number of zero-copy debug information. */
#define TILEPCI_ZERO_COPY_DEBUG_MINOR 21

/* Minor number of hypervisor debug information. */
#define TILEPCI_HV_DEBUG_MINOR 22

/* Minor number of chip information. */
#define TILEPCI_INFO_MINOR 23

/* Minor number for rshim access on TilePro. */
#define TILEPCI_RSHIM_MINOR 24

/* Minor number for the fast packet queue device. */
#define TILEPCI_PACKET_QUEUE_MINOR 25

/* Minor number for the direct-HV device. */
#define TILEPCI_HV_DIRECT_ZC_MINOR 26

/* Minor number for the memory-mapped BAR device. */
#define TILEPCI_BARMEM_MINOR 27

/* Minor number of first chip-to-chip ZC send queue. */
#define TILEPCI_FIRST_C2C_SEND_MINOR 64

/* Minor number of first chip-to-chip ZC receive queue. */
#define TILEPCI_FIRST_C2C_RECV_MINOR 96

/* Minor number of first host-to-tile ZC command queue. */
#define TILEPCI_FIRST_ZC_H2T_MINOR 128

/* Minor number of first tile-to-host ZC command queue. */
#define TILEPCI_FIRST_ZC_T2H_MINOR 192


/* Mappings between each linux interface type and the HV channel numbers. */

/* First HV channel used for char stream host-to-tile. */
#define TILEPCI_FIRST_CHAR_H2T_CHAN (PCIE_FIRST_HOST_TO_TILE_CHANNEL)

/* First HV channel used for char stream tile-to-host. */
#define TILEPCI_FIRST_CHAR_T2H_CHAN (PCIE_FIRST_TILE_TO_HOST_CHANNEL)

/* First HV channel used for zero-copy host-to-tile. */
#define TILEPCI_FIRST_ZC_H2T_CHAN \
  (TILEPCI_FIRST_CHAR_H2T_CHAN + TILEPCI_NUM_CHAR_STREAMS)

/* First HV channel used for zero-copy tile-to-host. */
#define TILEPCI_FIRST_ZC_T2H_CHAN \
  (TILEPCI_FIRST_CHAR_T2H_CHAN + TILEPCI_NUM_CHAR_STREAMS)

/* First HV channel used for zero-copy chip-to-chip sends. */
#define TILEPCI_FIRST_C2C_SEND_CHAN (PCIE_FIRST_C2C_SEND_CHANNEL)

/* First HV channel used for zero-copy chip-to-chip receives. */
#define TILEPCI_FIRST_C2C_RECV_CHAN (PCIE_FIRST_C2C_RECV_CHANNEL)

/* First HV channel used for DMA reads. */
#define TILEPCI_FIRST_DMA_READ_CHAN (PCIE_FIRST_DMA_READ_CHANNEL)

/* First HV channel used for DMA writes. */
#define TILEPCI_FIRST_DMA_WRITE_CHAN (PCIE_FIRST_DMA_WRITE_CHANNEL)


typedef struct tilepci_iomem_address {
	__u32 va;
	__u32 size;
} tilepci_iomem_address_t;



#endif /* _ASM_TILE_TILEPCI_H */
