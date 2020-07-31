ifeq ($(TARGET_OS),NO_OS)
ifeq ($(TARGET_CPU),R5F)

TARGET      := ipc_rpmsg_lib_mcu$(MCUNUM)
TARGETTYPE  := library

# Define local compiler options
DEFS+=BUILD_MCU$(MCUNUM)
DEFS+=BAREMETAL

# Define application's root directory
APPDIR := $(abspath $(SDIR)/..)

# Add directory to include search path
IDIRS+=$(APPDIR)/include/$(SITARA_DEMO_SOC)
IDIRS+=$(APPDIR)/include

CSOURCES    := ./ipc_rpmsg_baremetal.c ./ipc_trace.c ./ipcapp_baremetal.c

endif
endif
