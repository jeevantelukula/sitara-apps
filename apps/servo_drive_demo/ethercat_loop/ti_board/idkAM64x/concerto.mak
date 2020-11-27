ifeq ($(TARGET_OS),SYSBIOS)
ifeq ($(TARGET_CPU),R5F)
ifeq ($(TARGET_PLATFORM),AM64X)

include $(PRELUDE)

TARGET      := app_servo_drive_ethcat_tiboard_idkAM64x
TARGETTYPE  := library
CSOURCES    := board_gpioLed.c board_i2cLed.c board_phy.c tiescphy_dp83869.c board_eeprom.c

# Define root directory
LIBDIR := $(abspath $(SDIR)/../..)

CFLAGS +=-D=TIESC_EMULATION_PLATFORM

IDIRS+=$(LIBDIR)/include
IDIRS+=$(LIBDIR)/ti_board/include

include $(FINALE)

endif
endif
endif
