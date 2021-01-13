# Get directory that contains this Makefile
DEMO_ROOT := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

# User-supplied paths to dependencies
include $(MAKEENV)

# Boilerplate concerto configuration
CONCERTO_ROOT ?= $(abspath $(DEMO_ROOT)/../../concerto)
BUILD_MULTI_PROJECT := 1
BUILD_TARGET ?= $(CONCERTO_ROOT)/build_targets/processor-sdk/target.mak
BUILD_PLATFORM :=

# Recursively find all concerco.mak in the following directory list.
DIRECTORIES = $(HOST_ROOT)

# Target combos supported in this demo
ifeq ($(TARGET_PLATFORM),AM65X)
TARGET_COMBOS ?= AM65X:SYSBIOS:R5F:2:release:TIARMCGT \
                 AM65X:SYSBIOS:R5F:2:debug:TIARMCGT \
                 AM65X:NO_OS:R5F:2:release:TIARMCGT \
                 AM65X:NO_OS:R5F:2:debug:TIARMCGT
endif
ifeq ($(TARGET_PLATFORM),AM64X)
TARGET_COMBOS ?= AM64X:NO_OS:M4F:2:release:TIARMCGT \
                 AM64X:NO_OS:M4F:2:debug:TIARMCGT \
                 AM64X:SYSBIOS:R5F:2:release:TIARMCGT \
                 AM64X:SYSBIOS:R5F:2:debug:TIARMCGT \
                 AM64X:NO_OS:R5F:2:release:TIARMCGT \
                 AM64X:NO_OS:R5F:2:debug:TIARMCGT
endif

# "Invoke concerto"
include $(CONCERTO_ROOT)/rules.mak

#Build Servo Drive Demo Combined App Image
RTOS_BIN_PATH1=$(HOST_ROOT)/$(BUILD_OUTPUT)
#Dummy parameter should be passed to the script
HLOS_BOOT=

ifeq ($(TARGET_PLATFORM),AM65X)
IMG1=mcu1_0,$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/R5F/SYSBIOS/$(TARGET_BUILD)/app_tirtos_mcu1_0_servo_drive_ethcat.out)
IMG2=mcu1_1,$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/R5F/NO_OS/$(TARGET_BUILD)/app_no_os_mcu1_1_servo_drive_pscontrol.out)
IMG3=
SYSFWVERSION=V0
SYSFWFILE=sysfw_sr2.bin
endif

ifeq ($(TARGET_PLATFORM),AM64X)
IMG1=m4f_0,$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/M4F/NO_OS/$(TARGET_BUILD)/app_no_os_m4f_0_servo_drive_safety.out)
IMG2=mcu1_0,$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/R5F/SYSBIOS/$(TARGET_BUILD)/app_tirtos_mcu1_0_servo_drive_ethcat.out)
IMG3=mcu2_0,$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/R5F/NO_OS/$(TARGET_BUILD)/app_no_os_mcu2_0_servo_drive_pscontrol.out)
SYSFWVERSION=V3
SYSFWFILE=sysfw.bin
endif

app_img_build:
			  $(MAKE) -C $(call PATH_CONV, $(PDK_PATH)/packages/ti/boot/sbl/tools/combined_appimage) IMG1=$(IMG1) IMG2=$(IMG2) IMG3=$(IMG3) BOARD=$(PDK_BOARD) SOC=$(PDK_SOC) HLOS_BOOT=$(HLOS_BOOT)
#Create Servo Drive Demo SD Card Image
			  $(MKDIR) $(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/SDCardImage)
			  $(COPY) "$(call PATH_CONV,$(PDK_PATH)/packages/ti/boot/sbl/tools/combined_appimage/bin/$(PDK_BOARD)/combined.appimage)" "$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/SDCardImage/app)"
			  $(COPY) "$(call PATH_CONV,$(PDK_PATH)/packages/ti/boot/sbl/binary/$(PDK_BOARD)/mmcsd/bin/sbl_mmcsd_img_mcu1_0_release.tiimage)" "$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/SDCardImage/tiboot3.bin)"
			  $(COPY) "$(call PATH_CONV,$(PDK_PATH)/packages/ti/drv/sciclient/soc/$(SYSFWVERSION)/$(SYSFWFILE))" "$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/SDCardImage/sysfw.bin)"
#Create Servo Drive Demo OSPI Image
			  $(MKDIR) $(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/OSPIImage)
			  $(COPY) "$(call PATH_CONV,$(PDK_PATH)/packages/ti/boot/sbl/tools/combined_appimage/bin/$(PDK_BOARD)/combined.appimage)" "$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/OSPIImage/app)"
			  $(COPY) "$(call PATH_CONV,$(PDK_PATH)/packages/ti/boot/sbl/binary/$(PDK_BOARD)/ospi/bin/sbl_ospi_img_mcu1_0_release.tiimage)" "$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/OSPIImage/tiboot3.bin)"
			  $(COPY) "$(call PATH_CONV,$(PDK_PATH)/packages/ti/drv/sciclient/soc/$(SYSFWVERSION)/$(SYSFWFILE))" "$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/OSPIImage/sysfw.bin)"
app_img_build_clean:
			  $(CLEANDIR) $(call PATH_CONV,$(PDK_PATH)/packages/ti/boot/sbl/tools/combined_appimage/bin/)
			  $(CLEANDIR) $(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/SDCardImage)
			  $(CLEANDIR) $(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/OSPIImage)

all:  app_img_build
clean::  app_img_build_clean
.PHONY: app_img_build
