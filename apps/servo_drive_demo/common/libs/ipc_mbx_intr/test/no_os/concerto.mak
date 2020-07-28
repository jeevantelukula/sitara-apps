# Concerto is a simple single-pass, non-recursive, full dependency-managed
# Makefile system. This concerto.mak file specifies a target module in the
# Ccncerto build system.

# This is baremetal (NO_OS) based application developed specifically for the
# R5F CPU architecture. This must be filtered here so that concerto does not
# attempt any other combinations.
ifeq ($(TARGET_OS),NO_OS)
ifeq ($(TARGET_CPU),R5F)

# Begin the concerto module declarations by includng the "PRELUDE"
include $(PRELUDE)

# Define core ID as each core will host an application that provides a unique
# role in the system demo. This is beyond the concerto concept of TARGET_CPU,
# so define this here. The PDK libraries may also be unique for individual core
# instances.
ifeq ($(TARGET_PLATFORM),AM65X)
PDK_CORE_ID = mcu1_1
endif
ifeq ($(TARGET_PLATFORM),AM64X)
PDK_CORE_ID = mcu2_0
endif

# Define object name (TARGET) and type (TARGET_TYPE)
TARGET      := app_no_os_$(PDK_CORE_ID)_mailbox_ipc_test
TARGETTYPE  := exe

# Provide list of C files by using built-in macro
CSOURCES    := $(call all-c-files)
CSOURCES    += ../app_mbx_ipc_test.c

# Define application's root directory
APPDIR := $(abspath $(SDIR)/../../../../..)

# Add directory to include search path
IDIRS+=$(APPDIR)/common/libs/logs/include
IDIRS+=$(APPDIR)/common/libs/sciclient/include
IDIRS+=$(APPDIR)/common/libs/ipc_mbx_intr/include/$(SITARA_DEMO_SOC)
IDIRS+=$(APPDIR)/common/libs/ipc_mbx_intr/include
IDIRS+=$(APPDIR)/common/libs/ipc_mbx_intr/test/$(SITARA_DEMO_SOC)
IDIRS+=$(APPDIR)/common/libs/ipc_mbx_intr/test

# common "config" dependencies
COMMON_CONFIG = $(abspath $(APPDIR)/common/config/$(SITARA_DEMO_SOC))

# Append to STATIC_LIBS for common demo libraries
# These must also be built using concerto, and concerto will handle the
# dependencies
STATIC_LIBS += app_libs_logs
STATIC_LIBS += app_libs_sciclient
STATIC_LIBS += app_servo_drive_common_ipc_mbx_intr
STATIC_LIBS += app_libs_r5f_no_os_mpu_cfg

# Append to ADDITIONAL_STATIC_LIBS for external libraries (e.g. PDK)
ADDITIONAL_STATIC_LIBS += ti.csl.aer5f
ADDITIONAL_STATIC_LIBS += ti.csl.init.aer5f
ADDITIONAL_STATIC_LIBS += ti.osal.aer5f
ADDITIONAL_STATIC_LIBS += sciclient.aer5f
ADDITIONAL_STATIC_LIBS += ti.drv.uart.aer5f
ifeq ($(TARGET_PLATFORM),AM64X)
ADDITIONAL_STATIC_LIBS += mailbox.aer5f
endif

# Add run-time libraries from toolchain
SYS_STATIC_LIBS += rtsv7R4_T_le_v3D16_eabi

# Set the linker.cmd files that specify linker options along with memory
# placement.
LINKER_CMD_FILES +=  $(COMMON_CONFIG)/mem_map/linker_mem_map.cmd
LINKER_CMD_FILES +=  $(APPDIR)/position_speed_loop/src/$(SITARA_DEMO_SOC)/linker.cmd

# End concerto module declarations
include $(FINALE)

endif
endif
