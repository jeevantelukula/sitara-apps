# Concerto is a simple single-pass, non-recursive, full dependency-managed
# Makefile system. This concerto.mak file specifies a target module in the
# Ccncerto build system.

# Filter based on OS so that concerto does not attempt any other
# combinations.
ifeq ($(TARGET_OS), $(filter $(TARGET_OS), SYSBIOS NO_OS))

# Begin the concerto module declarations by includng the "PRELUDE"
include $(PRELUDE)

# Define object name (TARGET) and type (TARGET_TYPE)
TARGET      := app_libs_misc
TARGETTYPE  := library

# Define library root directory
LIBDIR := $(abspath $(SDIR)/..)

# common library dependencies
COMMON_LIB = $(abspath $(LIBDIR)/../)

# Add directory to include search path
IDIRS+=$(LIBDIR)/include/$(SITARA_DEMO_SOC)
IDIRS+=$(LIBDIR)/include
IDIRS+=$(COMMON_LIB)

ifeq ($(TARGET_OS),SYSBIOS)
CSOURCES    := app_cpu_hz.c
endif

ifeq ($(TARGET_CPU),R5F)
CSOURCES += app_r5f_init.c

CSOURCES += $(SITARA_DEMO_SOC)/app_pinmux.c
IDIRS+=$(PDK_PATH)/packages/ti/board/src/$(PDK_BOARD)/include
IDIRS+=$(PDK_PATH)/packages/ti/csl

endif
    
# End concerto module declarations
include $(FINALE)

endif
