ifeq ($(TARGET_OS),SYSBIOS)
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)

TARGET      := app_servo_drive_ethcat_tiboard_common
TARGETTYPE  := library
CSOURCES    := board_rotary_switch.c board_misc.c delay_us.c board_dp83867.c
CSOURCES    += board_dpphy.c

# Define root directory
LIBDIR := $(abspath $(SDIR)/../..)

IDIRS+=$(LIBDIR)/ti_board/include

ifeq ($(TARGET_PLATFORM),AM65X)
DEFS+=SOC_AM65XX
endif

include $(FINALE)

endif
endif
