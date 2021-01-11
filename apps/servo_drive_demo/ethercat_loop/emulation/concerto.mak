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

ifneq ($(BUILD_DEBUG_TEST_TARGETS),1)
SKIPBUILD=1
endif

# Define object name (TARGET) and type (TARGET_TYPE)
TARGET      := app_tirtos_mcu1_0_ethercat_emulation
TARGETTYPE  := exe

# Provide list of C files by using built-in macro
CSOURCES    := app_pslctrl.c $(SITARA_DEMO_SOC)/app_pslctrl_cfg.c app_pslctrl_mbxipc.c app_pslctrl_timesync.c app_pslctrl_cfg_mcu_intr.c app_pslctrl_esc_sim.c app_pslctrl_save_data.c

# Define application's root directory
APPDIR := $(abspath $(SDIR)/..)
EXAMPLEDIR := $(abspath $(SDIR)/../../../../examples)

# Add directory to include search path
IDIRS+=$(SDIR)/$(SITARA_DEMO_SOC)
IDIRS+=$(APPDIR)/../common/include/$(SITARA_DEMO_SOC)
IDIRS+=$(APPDIR)/../common/include
IDIRS+=$(APPDIR)/../common/libs/logs/include
IDIRS+=$(APPDIR)/../common/libs/sciclient/include
IDIRS+=$(APPDIR)/../common/libs/ipc_mbx_intr/include/$(SITARA_DEMO_SOC)
IDIRS+=$(APPDIR)/../common/libs/ipc_mbx_intr/include
IDIRS+=$(APPDIR)/../common/libs/misc/include/$(SITARA_DEMO_SOC)
IDIRS+=$(APPDIR)/../common/libs/misc/include
IDIRS+=$(APPDIR)/beckhoff_ssc
IDIRS+=$(EXAMPLEDIR)/timesync/driver/include
IDIRS+=$(EXAMPLEDIR)/timesync/driver/firmware

# Add this for including private board headers
IDIRS+=$(PDK_PATH)/packages/ti/csl
IDIRS+=$(PDK_PATH)/packages/ti/board/src/$(PDK_BOARD)/include

# Add directory to library search path
LDIRS+=$(EXAMPLEDIR)/timesync/out/$(TARGET_PLATFORM)/R5F/SYSBIOS/$(TARGET_BUILD)

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
STATIC_LIBS += app_libs_misc
STATIC_LIBS += ex_timesync_libs_driver

# Append to ADDITIONAL_STATIC_LIBS for external libraries (e.g. PDK)
ADDITIONAL_STATIC_LIBS += ti.osal.aer5f
ADDITIONAL_STATIC_LIBS += ti.csl.init.aer5f
ADDITIONAL_STATIC_LIBS += ti.csl.aer5f
ADDITIONAL_STATIC_LIBS += ti.board.aer5f
ADDITIONAL_STATIC_LIBS += ti.drv.uart.aer5f
#ADDITIONAL_STATIC_LIBS += ti.drv.gpio.aer5f
ADDITIONAL_STATIC_LIBS += sciclient.aer5f
ADDITIONAL_STATIC_LIBS += ti.utils.copyvecs.aer5f
ADDITIONAL_STATIC_LIBS += ti.drv.pruss.aer5f
ifeq ($(TARGET_PLATFORM),AM64X)
ADDITIONAL_STATIC_LIBS += mailbox.aer5f
endif

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
