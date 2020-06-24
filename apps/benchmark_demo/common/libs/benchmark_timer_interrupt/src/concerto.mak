ifeq ($(TARGET_OS),NO_OS)
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)
TARGET      := benchmark_timer_interrupt
TARGETTYPE  := library

# Define application's root directory
APPDIR := $(abspath $(SDIR)/..)

# Add directory to include search path
IDIRS+=$(APPDIR)/include
IDIRS+=$(APPDIR)/include/$(SITARA_DEMO_SOC)
IDIRS+=$(APPDIR)/../../include/r5f
IDIRS+=$(APPDIR)/../../include

CSOURCES    := benchmark_timer_interrupt.c

include $(FINALE)

endif
endif
