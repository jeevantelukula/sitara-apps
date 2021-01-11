# Concerto is a simple single-pass, non-recursive, full dependency-managed
# Makefile system. This concerto.mak file specifies a target module in the
# Ccncerto build system.

# Filter based on OS so that concerto does not attempt any other
# combinations.
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)
TARGET      := common_libs_profile
TARGETTYPE  := library

# Define local compiler options
DEFS+=__COMPILER_BARRIER

CSOURCES    := profile.c

include $(FINALE)

endif
