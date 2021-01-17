# Concerto is a simple single-pass, non-recursive, full dependency-managed
# Makefile system. This concerto.mak file specifies a target module in the
# Ccncerto build system.

# Filter based on OS so that concerto does not attempt any other
# combinations.
ifeq ($(TARGET_OS),NO_OS)
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)
TARGET      := ti_r5fmathlib
TARGETTYPE  := library

# Define local compiler options
DEFS+=__COMPILER_BARRIER

CSOURCES    := ti_r5fmath_trig.c
ASSEMBLY    := ti_r5fmath_trig_assy.S

include $(FINALE)

endif
endif
