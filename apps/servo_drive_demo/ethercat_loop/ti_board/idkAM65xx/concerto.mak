ifeq ($(TARGET_OS),SYSBIOS)
ifeq ($(TARGET_CPU),R5F)
ifeq ($(TARGET_PLATFORM),AM65XX)

include $(PRELUDE)

TARGET      := app_servo_drive_ethcat_tiboard_idkAM65xx
TARGETTYPE  := library
CSOURCES    := board_gpioLed.c board_i2cLed.c board_spi.c board_phy.c tiescphy_dp83867.c

# Define root directory
LIBDIR := $(abspath $(SDIR)/../..)

IDIRS+=$(LIBDIR)/include
IDIRS+=$(LIBDIR)/ti_board/include

ifeq ($(TARGET_PLATFORM),AM65X)
DEFS+=SOC_AM65XX
endif

include $(FINALE)

endif
endif
endif
