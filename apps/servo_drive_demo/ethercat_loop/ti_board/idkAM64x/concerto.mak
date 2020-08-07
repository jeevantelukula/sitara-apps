ifeq ($(TARGET_OS),SYSBIOS)
ifeq ($(TARGET_CPU),R5F)
ifeq ($(TARGET_PLATFORM),AM64X)

include $(PRELUDE)

TARGET      := app_servo_drive_ethcat_tiboard_idkAM64x
TARGETTYPE  := library
CSOURCES    := board_phy.c tiescphy_dp83867.c

# Define root directory
LIBDIR := $(abspath $(SDIR)/../..)

IDIRS+=$(LIBDIR)/include
IDIRS+=$(LIBDIR)/ti_board/include

include $(FINALE)

endif
endif
endif
