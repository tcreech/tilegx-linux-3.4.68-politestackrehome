#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#
# This file is included by the global makefile so that you can add your own
# architecture-specific flags and dependencies. Remember to do have actions
# for "archclean" and "archdep" for cleaning up and making dependencies for
# this architecture

# If building with TILERA_ROOT set (i.e. using the Tilera Multicore
# Development Environment) we can set CROSS_COMPILE based on that.
# If we're not cross-compiling, make sure we're on the right architecture.
# Only bother to test for a few common targets, to avoid useless errors.
ifeq ($(CROSS_COMPILE),)
  ifdef TILERA_ROOT
    CROSS_COMPILE := $(TILERA_ROOT)/bin/tile-
  else
    goals := $(if $(MAKECMDGOALS), $(MAKECMDGOALS), all)
    ifneq ($(strip $(filter vmlinux modules all,$(goals))),)
      HOST_ARCH := $(shell uname -m)
      ifneq ($(HOST_ARCH),$(ARCH))
$(error Set TILERA_ROOT or CROSS_COMPILE when building $(ARCH) on $(HOST_ARCH))
      endif
    endif
  endif
else
  TILERA_ROOT ?= $(shell $(CROSS_COMPILE)env sh -c 'echo $$TILERA_ROOT')
endif

# Check that the compiler supports feedback
ifeq ($(call cc-option,-ffeedback-generate),)
  ifdef CONFIG_FEEDBACK_COLLECT
    $(warning Compiler does not support feedback; disabling feedback collection.)
    CONFIG_FEEDBACK_COLLECT=
  endif
  ifneq ($(CONFIG_FEEDBACK_USE),"")
    $(warning Compiler does not support feedback; disabling feedback file.)
    CONFIG_FEEDBACK_USE=""
  endif
endif

ifdef CONFIG_FEEDBACK_COLLECT

# Support feedback collection.
KBUILD_CFLAGS_KERNEL += -ffunction-sections
KBUILD_CFLAGS_KERNEL += -ffeedback-generate
KBUILD_CFLAGS_KERNEL += -fstrip-file-prefix=$(srctree)
KBUILD_AFLAGS_KERNEL += -ffeedback-generate

else

ifneq ($(CONFIG_FEEDBACK_USE),"")
# Support feedback use.
ifneq ($(CROSS_COMPILE),)
  # NOTE: This variable is often "quoted", so we use "subst" not "patsubst",
  # and we must have no space after the first comma.  We use "strip" below
  # to deal with the space/newline after the second comma.
  CONFIG_FEEDBACK_USE := \
    $(subst /usr/lib/feedback/,$(TILERA_ROOT)/tile/usr/lib/feedback/, \
      $(CONFIG_FEEDBACK_USE))
endif
# Only enable hot/cold per-function sections if linker feedback is enabled.
ifdef CONFIG_FEEDBACK_ENABLE_LINKER
KBUILD_CFLAGS_KERNEL += -ffunction-sections
KBUILD_CFLAGS_KERNEL += -freorder-blocks-and-partition
KBUILD_CFLAGS_KERNEL += -ffeedback-use-linker=$(strip $(CONFIG_FEEDBACK_USE))
endif
KBUILD_CFLAGS_KERNEL += -fno-tracer
KBUILD_CFLAGS_KERNEL += -fprofile-correction
KBUILD_CFLAGS_KERNEL += -fcoverage-permissive
KBUILD_CFLAGS_KERNEL += -ffeedback-use-compiler=$(strip $(CONFIG_FEEDBACK_USE))
KBUILD_CFLAGS_KERNEL += -fstrip-file-prefix=$(srctree)
endif

endif # CONFIG_FEEDBACK_COLLECT

# The tile compiler may emit .eh_frame information for backtracing.
# In kernel modules, this causes load failures due to unsupported relocations.
KBUILD_CFLAGS   += -fno-asynchronous-unwind-tables

ifeq ($(CONFIG_WERROR),y)
# We try to keep the builds warning-free.
KBUILD_CFLAGS   += -Werror
endif

LIBGCC_PATH     := \
  $(shell $(CC) $(KBUILD_CFLAGS) $(KCFLAGS) -fno-feedback-generate \
  -print-libgcc-file-name)

# Provide the path to use for "make defconfig".
# We default to the newer TILE-Gx architecture if only "tile" is given.
ifeq ($(ARCH),tile)
        KBUILD_DEFCONFIG := tilegx_defconfig
else
        KBUILD_DEFCONFIG := $(ARCH)_defconfig
endif

# Used as a file extension when useful, e.g. head_$(BITS).o
# Not needed for (e.g.) "$(CC) -m32" since the compiler automatically
# uses the right default anyway.
export BITS
ifeq ($(CONFIG_TILEGX),y)
BITS := 64
else
BITS := 32
endif

head-y		:= arch/tile/kernel/head_$(BITS).o

libs-y		+= arch/tile/lib/
libs-y		+= $(LIBGCC_PATH)

drivers-y	+= arch/tile/drivers/
drivers-y	+= arch/tile/oprofile/

# See arch/tile/Kbuild for content of core part of the kernel
core-y		+= arch/tile/
core-$(CONFIG_KVM) += arch/tile/kvm/

core-$(CONFIG_TILE_GXIO) += arch/tile/gxio/

ifdef TILERA_ROOT
INSTALL_PATH ?= $(TILERA_ROOT)/tile/boot
endif

install:
	install -D -m 755 vmlinux $(INSTALL_PATH)/vmlinux-$(KERNELRELEASE)
	install -D -m 644 .config $(INSTALL_PATH)/config-$(KERNELRELEASE)
	install -D -m 644 System.map $(INSTALL_PATH)/System.map-$(KERNELRELEASE)

define archhelp
	echo '  install         - install kernel into $(INSTALL_PATH)'
endef
