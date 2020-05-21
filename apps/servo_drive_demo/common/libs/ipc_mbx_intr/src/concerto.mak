ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)

TARGET      := app_servo_drive_common_ipc_mbx_intr
TARGETTYPE  := library
CSOURCES    := 

# Define root directory
LIBDIR := $(abspath $(SDIR)/..)

IDIRS+=$(LIBDIR)/include
IDIRS+=$(LIBDIR)/../logs/include

# This CSL based Mailbox + Interrupt IPC Validated only for AM65xx
ifeq ($(TARGET_PLATFORM),AM65X)
CSOURCES    += app_mailbox_ipc.c mailbox_config_am65x.c
endif

include $(FINALE)

endif

