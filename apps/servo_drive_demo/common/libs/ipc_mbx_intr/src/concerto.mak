ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)

TARGET      := app_servo_drive_common_ipc_mbx_intr
TARGETTYPE  := library
CSOURCES    := $(SITARA_DEMO_SOC)/app_mailbox_ipc.c
ifeq ($(TARGET_PLATFORM),AM65X)
# MBX configuration in Apps only needed for AM65x MBX IPC
# For AM64x - LLD driver does MBX configuration
CSOURCES    += $(SITARA_DEMO_SOC)/mailbox_config.c
endif

# Define root directory
LIBDIR := $(abspath $(SDIR)/..)

IDIRS+=$(LIBDIR)/include/$(SITARA_DEMO_SOC)
IDIRS+=$(LIBDIR)/include
IDIRS+=$(LIBDIR)/../../../../../common/libs/logs/include
IDIRS+=$(LIBDIR)/src/$(SITARA_DEMO_SOC)

include $(FINALE)

endif

