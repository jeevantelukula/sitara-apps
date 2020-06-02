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

# Define object name (TARGET) and type (TARGET_TYPE)
TARGET      := app_no_os_mcu1_1_servo_drive_pscontrol
TARGETTYPE  := exe

# common "config" dependencies
COMMON_CONFIG = $(abspath $(APPDIR)/../common/config/$(SITARA_DEMO_SOC))

# common library dependencies
COMMON_LIB = $(abspath $(APPDIR)/../common/libs)

# Provide list of C files by using built-in macro
CSOURCES    := app_cfg.c app_init.c app_psl_mbxipc.c cfg_icss.c cfg_mcu_intr.c GPIO_board.c main.c multi_axis_master_comms.c multi_axis_master_ctrl.c multi_axis_master_ctrl_user.c position_speed_loop.c

# Define application's root directory
APPDIR := $(abspath $(SDIR)/..)

# Add directory to include search path
IDIRS+=$(APPDIR)/include
IDIRS+=$(COMMON_LIB)
IDIRS+=$(PDK_PATH)/packages/ti/board/src/$(PDK_BOARD)/include
IDIRS+=$(PDK_PATH)/packages/ti/csl
IDIRS+=$(APPDIR)/../common/libs/ipc_mbx_intr/include
IDIRS+=$(APPDIR)/../ethercat_loop/beckhoff_ssc

# Define core ID as each core will host an application that provides a unique
# role in the system demo. This is beyond the concerto concept of TARGET_CPU,
# so define this here. The PDK libraries may also be unique for individual core
# instances.
PDK_CORE_ID = mcu1_1

# Append to STATIC_LIBS for common demo libraries
# These must also be built using concerto, and concerto will handle the
# dependencies
STATIC_LIBS += app_libs_logs
STATIC_LIBS += app_libs_sciclient
STATIC_LIBS += app_libs_misc
STATIC_LIBS += app_servo_drive_common_ipc_mbx_intr

# Append to ADDITIONAL_STATIC_LIBS for external libraries (e.g. PDK)
ADDITIONAL_STATIC_LIBS += ti.csl.aer5f
ADDITIONAL_STATIC_LIBS += ti.csl.init.aer5f
ADDITIONAL_STATIC_LIBS += ti.osal.aer5f
ADDITIONAL_STATIC_LIBS += sciclient.aer5f
ADDITIONAL_STATIC_LIBS += ti.board.aer5f
ADDITIONAL_STATIC_LIBS += ti.drv.uart.aer5f
ADDITIONAL_STATIC_LIBS += ti.drv.gpio.aer5f
ADDITIONAL_STATIC_LIBS += ti.drv.pruss.aer5f

# Add run-time libraries from toolchain
SYS_STATIC_LIBS += rtsv7R4_T_le_v3D16_eabi

# Set the linker.cmd files that specify linker options along with memory
# placement.
LINKER_CMD_FILES +=  $(COMMON_CONFIG)/mem_map/linker_mem_map.cmd
LINKER_CMD_FILES +=  $(SDIR)/linker.cmd

# End concerto module declarations
include $(FINALE)

endif
endif
