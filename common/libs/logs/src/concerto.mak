# Concerto is a simple single-pass, non-recursive, full dependency-managed
# Makefile system. This concerto.mak file specifies a target module in the
# Ccncerto build system.

# Begin the concerto module declarations by includng the "PRELUDE"
include $(PRELUDE)

# Define object name (TARGET) and type (TARGET_TYPE)
TARGET      := common_libs_logs
TARGETTYPE  := library

# Define library root directory
LIBDIR := $(abspath $(SDIR)/..)

# Add directory to include search path
IDIRS+=$(LIBDIR)/include

CSOURCES    := app_log_writer.c app_log_no_os.c

# End concerto module declarations
include $(FINALE)
