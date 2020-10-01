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

# By default, only build linux userspace apps if we are not on Windows, but also
# allow users to choose on commandline.
BUILD_LINUX_APPS ?= 1
ifeq ($(HOST_OS),Windows_NT)
BUILD_LINUX_APPS = 0
endif

#Build Servo Drive Demo Combined App Image
RTOS_BIN_PATH1=$(HOST_ROOT)/$(BUILD_OUTPUT)
#Dummy parameter should be passed to the script
HLOS_BOOT=

ifeq ($(TARGET_PLATFORM),AM65X)
IMG1=mcu1_0,$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/R5F/SYSBIOS/$(TARGET_BUILD)/app_tirtos_mcu1_0_servo_drive_ethcat.out)
IMG2=mcu1_1,$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/R5F/NO_OS/$(TARGET_BUILD)/app_no_os_mcu1_1_servo_drive_pscontrol.out)
SYSFWVERSION=V0
SYSFWFILE=sysfw_sr2.bin
endif

ifeq ($(TARGET_PLATFORM),AM64X)
IMG1=mcu1_0,$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/R5F/SYSBIOS/$(TARGET_BUILD)/app_tirtos_mcu1_0_servo_drive_ethcat.out)
IMG2=mcu2_0,$(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/R5F/NO_OS/$(TARGET_BUILD)/app_no_os_mcu2_0_servo_drive_pscontrol.out)
SYSFWVERSION=V3
SYSFWFILE=sysfw-zebu.bin
endif

combinedapp_img_build:
			  $(MAKE) -C $(call PATH_CONV, $(PDK_PATH)/packages/ti/boot/sbl/tools/combined_appimage) IMG1=$(IMG1) IMG2=$(IMG2) BOARD=$(PDK_BOARD) SOC=$(PDK_SOC) HLOS_BOOT=$(HLOS_BOOT)
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
combinedapp_img_build_clean:
			  $(CLEANDIR) $(call PATH_CONV,$(PDK_PATH)/packages/ti/boot/sbl/tools/combined_appimage/bin/)
			  $(CLEANDIR) $(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/SDCardImage)
			  $(CLEANDIR) $(call PATH_CONV,$(RTOS_BIN_PATH1)/$(TARGET_PLATFORM)/OSPIImage)

all:  combinedapp_img_build
clean::  combinedapp_img_build_clean

ifeq ($(BUILD_LINUX_APPS),1)
# Add targets for Linux cmake app
all: opc_ua_app
clean:: opc_ua_app_clean
scrub:: opc_ua_app_scrub

OC_UA_APP_BUILDDIR = $(BUILD_OUTPUT)/opc_ua_app

opc_ua_app_builddir:
	$(MKDIR) $(OC_UA_APP_BUILDDIR)

opc_ua_app_config: opc_ua_app_builddir
	. $(LINUX_ENV_SETUP); \
	. $(dir $(LINUX_ENV_SETUP))/sysroots/x86_64-arago-linux/environment-setup.d/cmake.sh; \
	cd $(OC_UA_APP_BUILDDIR); \
	cmake $(HOST_ROOT)/opc_ua_app

opc_ua_app: opc_ua_app_config
	. $(LINUX_ENV_SETUP); \
	. $(dir $(LINUX_ENV_SETUP))/sysroots/x86_64-arago-linux/environment-setup.d/cmake.sh; \
	make -C $(OC_UA_APP_BUILDDIR)

opc_ua_app_clean: opc_ua_app_config
	. $(LINUX_ENV_SETUP); \
	. $(dir $(LINUX_ENV_SETUP))/sysroots/x86_64-arago-linux/environment-setup.d/cmake.sh; \
	make -C $(OC_UA_APP_BUILDDIR) clean

opc_ua_app_scrub:
	$(CLEANDIR) $(OC_UA_APP_BUILDDIR)

endif
