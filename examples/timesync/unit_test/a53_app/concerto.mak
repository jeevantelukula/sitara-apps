# Concerto is a simple single-pass, non-recursive, full dependency-managed
# Makefile system. This concerto.mak file specifies a target module in the
# Ccncerto build system.


# This is a TI RTOS (SYSBIOS) based application developed specifically for the
# R5F CPU architecture. This must be filtered here so that concerto does not
# attempt any other combinations.
ifeq ($(TARGET_OS),SYSBIOS)
ifeq ($(TARGET_CPU),A53)

# Begin the concerto module declarations by includng the "PRELUDE"
include $(PRELUDE)

# Define object name (TARGET) and type (TARGET_TYPE)
TARGET      := ex_tirtos_a53_timesync
TARGETTYPE  := exe

# Provide list of C files by using built-in macro
CSOURCES    := main.c

# Define application's root directory
APPDIR := $(abspath $(SDIR)/..)
UNITTEST_PATH := $(APPDIR)

# Append to STATIC_LIBS for common demo libraries
# These must also be built using concerto, and concerto will handle the
# dependencies

# Set the linker.cmd files that specify linker options along with memory
# placement.
LINKER_CMD_FILES +=  $(UNITTEST_PATH)/build/$(SITARA_DEMO_SOC)/a53/linker_a53.lds

DEFS+=CPU_mpu1

SOC_PATH = $(UNITTEST_PATH)/build/$(SITARA_DEMO_SOC)/a53

XDC_BLD_FILE = $(SOC_PATH)/config_$(SITARA_DEMO_SOC)_a53.bld
XDC_IDIRS    = $(SOC_PATH)
XDC_CFG_FILE = $(SOC_PATH)/sysbios_a53.cfg
# The default XDC_PLATFORM is provided by the concerto build target, and based
# on TARGET_PLATFORM and TARGET_CPU
XDC_PLATFORM = $(SITARA_XDC_PLATFORM)

ifeq ($(TARGET_PLATFORM), $(filter $(TARGET_PLATFORM), AM64X))
soc=am64x
DEFS+=SOC_AM64X
endif

LDIRS += $(BIOS_PATH)/packages/gnu/targets/arm/libs/install-native/aarch64-none-elf/lib

SYS_STATIC_LIBS += 
SYS_STATIC_LIBS += stdc++ gcc m c nosys rdimon

ADDITIONAL_STATIC_LIBS += ti.osal.aa53fg
ADDITIONAL_STATIC_LIBS += ti.csl.aa53fg
ADDITIONAL_STATIC_LIBS += ti.csl.init.aa53fg
ADDITIONAL_STATIC_LIBS += ti.board.aa53fg

# End concerto module declarations
include $(FINALE)

endif
endif
