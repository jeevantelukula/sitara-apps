ifeq ($(TARGET_OS),SYSBIOS)
ifeq ($(TARGET_CPU),R5F)

include $(PRELUDE)

TARGET      := app_servo_drive_ethcat_tiboard_common
TARGETTYPE  := library
ifeq ($(TARGET_PLATFORM),AM65X)
CSOURCES    := board_dp83867.c
endif
ifeq ($(TARGET_PLATFORM),AM64X)
CSOURCES    := board_dp83869.c
endif
CSOURCES    += board_dpphy.c board_misc.c delay_us.c board_rotary_switch.c

# Define root directory
LIBDIR := $(abspath $(SDIR)/../..)

ifeq ($(TARGET_PLATFORM),AM64X)
CFLAGS +=-D=TIESC_EMULATION_PLATFORM
endif

IDIRS+=$(LIBDIR)/ti_board/include

include $(FINALE)

endif
endif
