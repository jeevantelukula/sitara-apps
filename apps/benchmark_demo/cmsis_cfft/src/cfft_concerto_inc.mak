# Concerto is a simple single-pass, non-recursive, full dependency-managed
# Makefile system. This concerto.mak file specifies a target module in the
# Ccncerto build system.

# This is baremetal (NO_OS)  based application developed specifically for the
# R5F CPU architecture. This must be filtered here so that concerto does not
# attempt any other combinations.
ifeq ($(TARGET_OS),NO_OS)
ifeq ($(TARGET_CPU),R5F)

# Define object name (TARGET) and type (TARGET_TYPE)
TARGET      := app_no_os_mcu$(MCUNUM)_cmsis_cfft
TARGETTYPE  := exe

# Define local compiler options
DEFS+=__COMPILER_BARRIER
DEFS+=BUILD_MCU$(MCUNUM)
DEFS+=BAREMETAL
ifeq ($(BUILD_LINUX_APPS), 1)
DEFS+=ENABLE_IPC_RPMSG_CHAR
endif

# Define application's root directory
APPDIR := $(abspath $(SDIR)/..)
SITARA_APPS_DIR := $(abspath $(SDIR)/../../../..)

# common library dependencies
COMMON_LIB = $(abspath $(SITARA_APPS_DIR)/common/libs)

# Provide list of C files by using built-in macro
CSOURCES    := ./main.c ./cfft.c
CSOURCES    += ../../common/src/r5f_mpu_default_mcu$(MCUNUM).c

# Add directory to include search path
IDIRS+=$(APPDIR)/include
IDIRS+=$(CMSIS_LIB)/CMSIS_5/CMSIS/DSP/Include
IDIRS+=$(CMSIS_LIB)/CMSIS_5/CMSIS/Core/Include
IDIRS+=$(COMMON_LIB)/profile/include
IDIRS+=$(APPDIR)/../common/include/r5f
IDIRS+=$(APPDIR)/../common/include
IDIRS+=$(APPDIR)/../common/libs/benchmark_timer_interrupt/include
IDIRS+=$(APPDIR)/../common/libs/benchmark_timer_interrupt/include/$(SITARA_DEMO_SOC)
IDIRS+=$(APPDIR)/../../common/ipc_rpmsg_lib/include
IDIRS+=$(APPDIR)/../../common/ipc_rpmsg_lib/include/$(SITARA_DEMO_SOC)

# Add directory to library search path
LDIRS+=$(CMSIS_LIB)/lib_prebuild/release
LDIRS+=$(COMMON_LIB)/out/$(TARGET_PLATFORM)/$(TARGET_CPU)/$(TARGET_OS)/$(TARGET_BUILD)

# Define core ID as each core will host an application that provides a unique
# role in the system demo. This is beyond the concerto concept of TARGET_CPU,
# so define this here. The PDK libraries may also be unique for individual core
# instances.
PDK_CORE_ID = mcu$(MCUNUM)

# common "config" dependencies
COMMON_CONFIG = $(abspath $(APPDIR)/../common/config/$(SITARA_DEMO_SOC))

# Append to STATIC_LIBS for common demo libraries
# These must also be built using concerto, and concerto will handle the
# dependencies
STATIC_LIBS += common_libs_profile
STATIC_LIBS += benchmark_timer_interrupt
STATIC_LIBS += ipc_rpmsg_lib_mcu$(MCUNUM)

# Append to ADDITIONAL_STATIC_LIBS for external libraries (e.g. PDK)
ADDITIONAL_STATIC_LIBS += ti.board.aer5f
ADDITIONAL_STATIC_LIBS += ti.csl.aer5f
ADDITIONAL_STATIC_LIBS += ti.csl.init.aer5f
ADDITIONAL_STATIC_LIBS += ti.osal.aer5f
ADDITIONAL_STATIC_LIBS += sciclient.aer5f
ADDITIONAL_STATIC_LIBS += ti.drv.uart.aer5f
ADDITIONAL_STATIC_LIBS += ipc_baremetal.aer5f
ADDITIONAL_STATIC_LIBS += ti_math_Cortex_R5_lspf.lib
ifeq ($(TARGET_PLATFORM),AM64X)
ADDITIONAL_STATIC_LIBS += mailbox.aer5f
endif

# Add run-time libraries from toolchain
SYS_STATIC_LIBS += rtsv7R4_T_le_v3D16_eabi

# Set the linker.cmd files that specify linker options along with memory
# placement.
LINKER_CMD_FILES +=  $(COMMON_CONFIG)/mem_map/linker_mem_map.cmd
LINKER_CMD_FILES +=  $(COMMON_CONFIG)/$(PDK_CORE_ID)/linker.cmd


endif
endif
