# Get directory that contains this Makefile
EXAMPLE_ROOT := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

# User-supplied paths to dependencies
include $(MAKEENV)

# Boilerplate concerto configuration
CONCERTO_ROOT ?= $(abspath $(EXAMPLE_ROOT)/../../concerto)
BUILD_MULTI_PROJECT := 1
BUILD_TARGET ?= $(CONCERTO_ROOT)/build_targets/processor-sdk/target.mak
BUILD_PLATFORM :=

# Recursively find all concerco.mak in the following directory list.
DIRECTORIES = $(HOST_ROOT)

# Targe combos supported in this demo
ifeq ($(TARGET_PLATFORM),AM64X)
TARGET_COMBOS ?= AM64X:NO_OS:R5F:2:release:TIARMCGT \
                 AM64X:NO_OS:R5F:2:debug:TIARMCGT
endif
# "Invoke concerto"
include $(CONCERTO_ROOT)/rules.mak

#Build Combined App Image
RTOS_BIN_PATH1=$(HOST_ROOT)/$(BUILD_OUTPUT)
#Dummy parameter should be passed to the script
HLOS_BOOT=

ifeq ($(TARGET_PLATFORM),AM64X)
IMG1=mcu1_0,$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/R5F/NO_OS/$(TARGET_BUILD)/app_no_os_mcu1_0_r5f_hello_world.out)
SYSFWVERSION=V3
SYSFWFILE=sysfw.bin
endif

app_img_build:
			  $(MAKE) -C $(call PATH_CONV, $(PDK_PATH)/packages/ti/boot/sbl/tools/combined_appimage) IMG1=$(IMG1) BOARD=$(PDK_BOARD) SOC=$(PDK_SOC) HLOS_BOOT=$(HLOS_BOOT)
#Create SD Card Image
			  $(MKDIR) $(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/SDCardImage)
			  $(COPY) "$(call PATH_CONV,$(PDK_PATH)/packages/ti/boot/sbl/tools/combined_appimage/bin/$(PDK_BOARD)/combined.appimage)" "$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/SDCardImage/app)"
			  $(COPY) "$(call PATH_CONV,$(PDK_PATH)/packages/ti/boot/sbl/binary/$(PDK_BOARD)/mmcsd/bin/sbl_mmcsd_img_mcu1_0_release.tiimage)" "$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/SDCardImage/tiboot3.bin)"
			  $(COPY) "$(call PATH_CONV,$(PDK_PATH)/packages/ti/drv/sciclient/soc/$(SYSFWVERSION)/$(SYSFWFILE))" "$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/SDCardImage/sysfw.bin)"
#Create OSPI Image
			  $(MKDIR) $(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/OSPIImage)
			  $(COPY) "$(call PATH_CONV,$(PDK_PATH)/packages/ti/boot/sbl/tools/combined_appimage/bin/$(PDK_BOARD)/combined.appimage)" "$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/OSPIImage/app)"
			  $(COPY) "$(call PATH_CONV,$(PDK_PATH)/packages/ti/boot/sbl/binary/$(PDK_BOARD)/ospi/bin/sbl_ospi_img_mcu1_0_release.tiimage)" "$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/OSPIImage/tiboot3.bin)"
			  $(COPY) "$(call PATH_CONV,$(PDK_PATH)/packages/ti/drv/sciclient/soc/$(SYSFWVERSION)/$(SYSFWFILE))" "$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/OSPIImage/sysfw.bin)"
app_img_build_clean:
			  $(CLEANDIR) $(call PATH_CONV,$(PDK_PATH)/packages/ti/boot/sbl/tools/combined_appimage/bin/)
			  $(CLEANDIR) $(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/SDCardImage)
			  $(CLEANDIR) $(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/OSPIImage)

all:  app_img_build_clean app_img_build
clean::  app_img_build_clean
.PHONY: app_img_build

