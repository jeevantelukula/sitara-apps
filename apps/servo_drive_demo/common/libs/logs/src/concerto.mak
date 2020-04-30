# Concerto is a simple single-pass, non-recursive, full dependency-managed
# Makefile system. This concerto.mak file specifies a target module in the
# Ccncerto build system.

# Filter based on OS so that concerto does not attempt any other
# combinations.
ifeq ($(TARGET_OS), $(filter $(TARGET_OS), SYSBIOS LINUX NO_OS))

# Begin the concerto module declarations by includng the "PRELUDE"
include $(PRELUDE)

# Define object name (TARGET) and type (TARGET_TYPE)
TARGET      := app_libs_logs
TARGETTYPE  := library

# Define library root directory
LIBDIR := $(abspath $(SDIR)/..)

# Add directory to include search path
IDIRS+=$(LIBDIR)/include

ifeq ($(TARGET_OS),SYSBIOS)
CSOURCES    := app_log_writer.c app_log_sysbios.c

endif

ifeq ($(TARGET_OS),NO_OS)
CSOURCES    := app_log_writer.c app_log_no_os.c
endif

ifeq ($(TARGET_OS),LINUX)
CSOURCES    := app_log_writer.c app_log_linux.c
endif

# End concerto module declarations
include $(FINALE)

endif
