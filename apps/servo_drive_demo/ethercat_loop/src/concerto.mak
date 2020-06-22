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

ifeq ($(TARGET_PLATFORM),AM64X)
SKIPBUILD=1
endif

# Define object name (TARGET) and type (TARGET_TYPE)
TARGET      := app_tirtos_mcu1_0_servo_drive_ethcat
TARGETTYPE  := exe

# Provide list of C files by using built-in macro
CSOURCES    := tiescutils.c $(SITARA_DEMO_SOC)/tiesc_soc.c

# Define application's root directory
APPDIR := $(abspath $(SDIR)/..)

# FIXME
ifeq ($(TARGET_PLATFORM),AM65X)
ADDITIONAL_STATIC_LIBS += ethercat_slave_fwhal_lib_AM65xx_r5f.lib
STATIC_LIBS += app_servo_drive_ethcat_tiboard_idkAM65xx
LDIRS += $(APPDIR)/lib/am65xx/r5f
endif

ifeq ($(TARGET_PLATFORM),AM64X)
ADDITIONAL_STATIC_LIBS += ethercat_slave_fwhal_lib_AM64xx_r5f.lib
STATIC_LIBS += app_servo_drive_ethcat_tiboard_idkAM64xx
LDIRS += $(APPDIR)/lib/am64xx/r5f
endif

# Add directory to include search path
IDIRS+=$(APPDIR)/include
IDIRS+=$(APPDIR)/ti_osal
IDIRS+=$(APPDIR)/beckhoff_ssc
IDIRS+=$(APPDIR)/ti_board/include
IDIRS+=$(APPDIR)/../common/include
IDIRS+=$(APPDIR)/../common/libs/logs/include
IDIRS+=$(APPDIR)/../common/libs/sciclient/include
IDIRS+=$(APPDIR)/../common/libs/ipc_mbx_intr/include/$(SITARA_DEMO_SOC)
IDIRS+=$(APPDIR)/../common/libs/ipc_mbx_intr/include
# Path for files where common info is shared between M4F and other cores.
IDIRS+=$(APPDIR)/../safety_app/include

# Add this for including private board headers
IDIRS+=$(PDK_PATH)/packages/ti/csl
IDIRS+=$(PDK_PATH)/packages/ti/board/src/$(PDK_BOARD)/include

# Define core ID as each core will host an application that provides a unique
# role in the system demo. This is beyond the concerto concept of TARGET_CPU,
# so define this here.
DEFS+=CPU_mcu1_0

# The PDK libraries may also be unique for individual core instances.
PDK_CORE_ID = mcu1_0


# common "config" dependencies
COMMON_CONFIG = $(abspath $(APPDIR)/../common/config/$(SITARA_DEMO_SOC))
IDIRS+=$(COMMON_CONFIG)/mem_map

# Append to STATIC_LIBS for common demo libraries
# These must also be built using concerto, and concerto will handle the
# dependencies
#STATIC_LIBS += app_common_mcu1_0
STATIC_LIBS += app_libs_logs
STATIC_LIBS += app_libs_sciclient
STATIC_LIBS += app_servo_drive_common_ipc_mbx_intr
STATIC_LIBS += app_servo_drive_ethcat_beckhoff_ssc
STATIC_LIBS += app_servo_drive_ethcat_tiboard_common
STATIC_LIBS += app_servo_drive_ethcat_osal
STATIC_LIBS += app_libs_copy_vecs_to_atcm

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
XDC_BLD_FILE = $(APPDIR)/tirtos/config_r5f.bld
XDC_IDIRS    = $(APPDIR)/tirtos/
XDC_CFG_FILE = $(APPDIR)/tirtos/$(SITARA_DEMO_SOC)/mcu1_0.cfg

# The default XDC_PLATFORM is provided by the concerto build target, and based
# on TARGET_PLATFORM and TARGET_CPU
XDC_PLATFORM = $(SITARA_XDC_PLATFORM)

# Set the linker.cmd files that specify linker options along with memory
# placement.
LINKER_CMD_FILES +=  $(COMMON_CONFIG)/mem_map/linker_mem_map.cmd
LINKER_CMD_FILES +=  $(SDIR)/$(SITARA_DEMO_SOC)/linker.cmd

# End concerto module declarations
include $(FINALE)

endif
endif
