# Concerto is a simple single-pass, non-recursive, full dependency-managed
# Makefile system. This concerto.mak file specifies a target module in the
# Ccncerto build system.


# This is aremetal (NO_OS)  based application developed specifically for the
# M4F CPU architecture. This must be filtered here so that concerto does not
# attempt any other combinations.
ifeq ($(TARGET_OS),NO_OS)
ifeq ($(TARGET_CPU),M4F)

# Begin the concerto module declarations by includng the "PRELUDE"
include $(PRELUDE)

# Define object name (TARGET) and type (TARGET_TYPE)
TARGET      := app_no_os_m4f_0_servo_drive_safety
TARGETTYPE  := exe

# Provide list of C files by using built-in macro
CSOURCES    := $(call all-c-files)


# Define application's root directory
APPDIR := $(abspath $(SDIR)/..)

# Add directory to include search path
IDIRS+=$(APPDIR)/include
IDIRS+=$(APPDIR)/include/priv

# Define core ID as each core will host an application that provides a unique
# role in the system demo. This is beyond the concerto concept of TARGET_CPU,
# so define this here. The PDK libraries may also be unique for individual core
# instances.
PDK_CORE_ID = m4f_0


# common "config" dependencies
COMMON_CONFIG = $(abspath $(APPDIR)/../common/config/$(SITARA_DEMO_SOC))


# Append to STATIC_LIBS for common demo libraries
# These must also be built using concerto, and concerto will handle the
# dependencies
#STATIC_LIBS += app_common_mcu1_0

# Append to ADDITIONAL_STATIC_LIBS for external libraries (e.g. PDK)
ADDITIONAL_STATIC_LIBS += ti.csl.aem4f
ADDITIONAL_STATIC_LIBS += ti.board.aem4f
ADDITIONAL_STATIC_LIBS += ti.csl.init.aem4f
ADDITIONAL_STATIC_LIBS += ti.drv.uart.aem4f
ADDITIONAL_STATIC_LIBS += sciclient.aem4f
ADDITIONAL_STATIC_LIBS += ti.osal.aem4f

# Add run-time libraries from toolchain
SYS_STATIC_LIBS += rtsv7M4_T_le_v4SPD16_eabi

# Set the linker.cmd files that specify linker options along with memory
# placement.
LINKER_CMD_FILES +=  $(COMMON_CONFIG)/mem_map/linker_mem_map.cmd
LINKER_CMD_FILES +=  $(SDIR)/$(SITARA_DEMO_SOC)/linker.cmd


# End concerto module declarations
include $(FINALE)

endif
endif
