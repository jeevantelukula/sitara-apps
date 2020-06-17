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
CSOURCES    += $(SITARA_DEMO_SOC)/app_mailbox_ipc.c $(SITARA_DEMO_SOC)/mailbox_config.c

include $(FINALE)

endif

