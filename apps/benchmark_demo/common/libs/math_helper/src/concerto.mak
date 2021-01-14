ifeq ($(TARGET_OS),NO_OS)
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)
TARGET      := math_helper
TARGETTYPE  := library

# Define local compiler options
DEFS+=__COMPILER_BARRIER

# Define application's root directory
APPDIR := $(abspath $(SDIR)/..)

# Add directory to include search path
IDIRS+=$(APPDIR)/include
IDIRS+=$(CMSIS_LIB)/CMSIS_5/CMSIS/DSP/Include
IDIRS+=$(CMSIS_LIB)/CMSIS_5/CMSIS/Core/Include
IDIRS+=$(APPDIR)/../../include/r5f
IDIRS+=$(APPDIR)/../../include

CSOURCES    := math_helper.c

include $(FINALE)

endif
endif
