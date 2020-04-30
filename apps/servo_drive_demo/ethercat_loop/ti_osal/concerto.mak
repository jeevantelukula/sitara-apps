ifeq ($(TARGET_OS),SYSBIOS)
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)

TARGET      := app_servo_drive_ethcat_osal
TARGETTYPE  := library
CSOURCES    := ClockP_tirtos.c EventP_tirtos.c MailboxP_tirtos.c MiscP_tirtos.c
CSOURCES    += OSP_nonos.c OSP_tirtos.c SwiP_tirtos.c

# Define root directory
LIBDIR := $(abspath $(SDIR)/..)

IDIRS+=$(LIBDIR)

ifeq ($(TARGET_PLATFORM),AM65X)
DEFS+=SOC_AM65XX
endif

include $(FINALE)

endif
endif
