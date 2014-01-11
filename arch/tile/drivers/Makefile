#
# Makefile for TILE drivers that don't belong elsewhere in the tree.
# Typically these are drivers that just present a simple char
# file_operations interface.
#
# For other Tilera-specific drivers, see:
#
#  drivers/char/tile-srom.c		SROM character driver
#  drivers/edac/tile_edac.c		EDAC driver
#  drivers/i2c/busses/i2c-tile.c	I2C driver
#  drivers/mtd/devices/tile_srom.c	SROM MTD driver
#  drivers/net/ethernet/tile/		network
#  drivers/rtc/rtc-tile.c		RTC driver
#  drivers/tty/hvc/hvc_tile.c		hypervisor console
#  drivers/usb/host/ehci-tilegx.c	USB EHCI driver glue
#  drivers/usb/host/ohci-tilegx.c	USB OHCI driver glue
#  drivers/watchdog/tile_wdt.c		hypervisor watchdog
#

obj-$(CONFIG_TILE_BME_MEM)	+= bme_mem.o
obj-$(CONFIG_TILE_I2CS)         += i2cs.o
obj-$(CONFIG_TILE_IORPC)	+= iorpc.o
obj-$(CONFIG_TILE_MEMPROF)	+= memprof.o
obj-$(CONFIG_TILE_PKA)		+= pka.o
obj-$(CONFIG_TILE_TMFIFO)	+= tmfifo.o

ifdef CONFIG_TILEGX

obj-$(CONFIG_TILEPCI_ENDP) += gxpci_endp.o tilegxpci_shared_code.o \
  gxpci_endp_nic.o
CFLAGS_gxpci_endp.o = -DTILEPCI_ENDP
CFLAGS_tilegxpci_shared_code.o = -DTILEPCI_ENDP
CFLAGS_gxpci_endp_nic.o = -DTILEPCI_ENDP

obj-$(CONFIG_TILEPCI_STREAMIO) += gxpci_streamio.o

obj-$(CONFIG_TILEPCI_HOST_SUBSET) += \
  gxpci_host_subset.o gxpci_chip_to_chip.o gxpci_host_subset_nic.o

endif
