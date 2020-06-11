# Concerto is a simple single-pass, non-recursive, full dependency-managed
# Makefile system. This concerto.mak file specifies a target module in the
# Ccncerto build system.

# This is baremetal (NO_OS)  based application developed specifically for the
# R5F CPU architecture. This must be filtered here so that concerto does not
# attempt any other combinations.
ifeq ($(TARGET_OS),NO_OS)
ifeq ($(TARGET_CPU),R5F)

# Begin the concerto module declarations by includng the "PRELUDE"
include $(PRELUDE)

# Define object name (TARGET) and type (TARGET_TYPE)
TARGET      := app_no_os_mcu1_0_cmsis_fir
TARGETTYPE  := exe

# Define local compiler options
DEFS+=__COMPILER_BARRIER

# Define application's root directory
APPDIR := $(abspath $(SDIR)/..)

# Provide list of C files by using built-in macro
CSOURCES    := $(call all-c-files)

# Add directory to include search path
IDIRS+=$(APPDIR)/include
IDIRS+=$(CMSIS_LIB)/CMSIS_5/CMSIS/DSP/Include
IDIRS+=$(CMSIS_LIB)/CMSIS_5/CMSIS/Core/Include
IDIRS+=$(APPDIR)/../common/include/r5f
IDIRS+=$(APPDIR)/../common/include

# Add directory to library search path
LDIRS+=$(CMSIS_LIB)/lib_prebuild/release

# Define core ID as each core will host an application that provides a unique
# role in the system demo. This is beyond the concerto concept of TARGET_CPU,
# so define this here. The PDK libraries may also be unique for individual core
# instances.
PDK_CORE_ID = mcu1_0

# common "config" dependencies
COMMON_CONFIG = $(abspath $(APPDIR)/../common/config/$(SITARA_DEMO_SOC))

# Append to STATIC_LIBS for common demo libraries
# These must also be built using concerto, and concerto will handle the
# dependencies
STATIC_LIBS += profile

# Append to ADDITIONAL_STATIC_LIBS for external libraries (e.g. PDK)
ADDITIONAL_STATIC_LIBS += ti.board.aer5f
ADDITIONAL_STATIC_LIBS += ti.csl.aer5f
ADDITIONAL_STATIC_LIBS += ti.csl.init.aer5f
ADDITIONAL_STATIC_LIBS += ti.osal.aer5f
ADDITIONAL_STATIC_LIBS += sciclient.aer5f
ADDITIONAL_STATIC_LIBS += ti.drv.uart.aer5f
ADDITIONAL_STATIC_LIBS += ti_math_Cortex_R5_lspf.lib

# Add run-time libraries from toolchain
SYS_STATIC_LIBS += rtsv7R4_T_le_v3D16_eabi

# Set the linker.cmd files that specify linker options along with memory
# placement.
LINKER_CMD_FILES +=  $(COMMON_CONFIG)/mem_map/linker_mem_map.cmd
LINKER_CMD_FILES +=  $(SDIR)/linker_$(SITARA_DEMO_SOC).cmd

# End concerto module declarations
include $(FINALE)

endif
endif
