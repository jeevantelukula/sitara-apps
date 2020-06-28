# Concerto is a simple single-pass, non-recursive, full dependency-managed
# Makefile system. This concerto.mak file specifies a target module in the
# Ccncerto build system.

# Filter based on OS so that concerto does not attempt any other
# combinations.
ifeq ($(TARGET_OS), $(filter $(TARGET_OS), SYSBIOS NO_OS))

# Begin the concerto module declarations by includng the "PRELUDE"
include $(PRELUDE)

# Define object name (TARGET) and type (TARGET_TYPE)
TARGET      := ex_timesync_libs_driver
TARGETTYPE  := library

# Define library root directory
LIBDIR := $(abspath $(SDIR)/..)

# common library dependencies
COMMON_LIB = $(abspath $(LIBDIR)/../)

# Add directory to include search path
IDIRS+=$(LIBDIR)/include
IDIRS+=$(LIBDIR)/../firmware
IDIRS+=$(COMMON_LIB)

CFLAGS= --define=BUILD_MCU1_0

CSOURCES    := icssgTimesyncDrv.c

# End concerto module declarations
include $(FINALE)

endif
