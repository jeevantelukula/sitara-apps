# Concerto is a simple single-pass, non-recursive, full dependency-managed
# Makefile system. This concerto.mak file specifies a target module in the
# Ccncerto build system.


# This is a TI RTOS (SYSBIOS) based application developed specifically for the
# R5F CPU architecture. This must be filtered here so that concerto does not
# attempt any other combinations.
ifeq ($(TARGET_OS),SYSBIOS)
ifeq ($(TARGET_CPU),R5F)

# Begin the concerto module declarations by includng the "PRELUDE"
include $(PRELUDE)

# Define object name (TARGET) and type (TARGET_TYPE)
TARGET      := ex_tirtos_mcu1_0_timesync
TARGETTYPE  := exe

# Provide list of C files by using built-in macro
CSOURCES    := main_timesync_test.c cfg_host_intr.c test_utils.c

# Define application's root directory
APPDIR := $(abspath $(SDIR)/..)

# Add directory to include search path
IDIRS+=$(APPDIR)/include
IDIRS+=$(APPDIR)/ti_osal
IDIRS+=$(APPDIR)/ti_board/include
IDIRS+=$(APPDIR)/../firmware/include
IDIRS+=$(APPDIR)/../firmware/gen
IDIRS+=$(APPDIR)/../driver/include

# Add this for including private board headers
IDIRS+=$(PDK_PATH)/packages/ti/csl
IDIRS+=$(PDK_PATH)/packages/ti/board/src/$(PDK_BOARD)/include

# Define core ID as each core will host an application that provides a unique
# role in the system demo. This is beyond the concerto concept of TARGET_CPU,
# so define this here.
DEFS+=CPU_mcu1_0

# The PDK libraries may also be unique for individual core instances.
PDK_CORE_ID = mcu1_0

# Append to STATIC_LIBS for common demo libraries
# These must also be built using concerto, and concerto will handle the
# dependencies
STATIC_LIBS += ex_timesync_libs_driver

# Append to ADDITIONAL_STATIC_LIBS for external libraries (e.g. PDK)
ADDITIONAL_STATIC_LIBS += ti.osal.aer5f
ADDITIONAL_STATIC_LIBS += ti.csl.init.aer5f
ADDITIONAL_STATIC_LIBS += ti.csl.aer5f
ADDITIONAL_STATIC_LIBS += ti.board.aer5f
ADDITIONAL_STATIC_LIBS += ti.drv.uart.aer5f
ADDITIONAL_STATIC_LIBS += ti.drv.i2c.aer5f
ADDITIONAL_STATIC_LIBS += ti.drv.gpio.aer5f
ADDITIONAL_STATIC_LIBS += ti.drv.spi.aer5f
ADDITIONAL_STATIC_LIBS += ti.drv.pruss.aer5f
ADDITIONAL_STATIC_LIBS += sciclient.aer5f

# Add run-time libraries from toolchain
SYS_STATIC_LIBS += rtsv7R4_T_le_v3D16_eabi

# The following XDC_* files are used to configure SYSBIOS to enable TI RTOS.
# XDC/BIOS configuro need below 3 input arguments to work
#- BUILD File
#- CFG File
#- Linker CMD file
XDC_BLD_FILE = $(APPDIR)/build/config_r5f.bld
XDC_IDIRS    = $(APPDIR)/build/
XDC_CFG_FILE = $(APPDIR)/build/sysbios_r5f.cfg

# The default XDC_PLATFORM is provided by the concerto build target, and based
# on TARGET_PLATFORM and TARGET_CPU
XDC_PLATFORM = $(SITARA_XDC_PLATFORM)

# Set the linker.cmd files that specify linker options along with memory
# placement.
LINKER_CMD_FILES +=  $(APPDIR)/build/linker_r5_sysbios.lds

# End concerto module declarations
include $(FINALE)

endif
endif
