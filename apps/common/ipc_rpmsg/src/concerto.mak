ifeq ($(TARGET_OS),NO_OS)
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)
TARGET      := ipc_rpmsg
TARGETTYPE  := library

# Define local compiler options
DEFS+=BUILD_MCU1_0
DEFS+=MPU_LINUX_OS
DEFS+=BAREMETAL

# Define application's root directory
APPDIR := $(abspath $(SDIR)/..)

# Add directory to include search path
IDIRS+=$(APPDIR)/include
IDIRS+=$(APPDIR)/../../benchmark_demo/common/include

CSOURCES    := ipc_rpmsg_baremetal.c ipc_trace.c ipcapp_baremetal.c

include $(FINALE)

endif
endif
