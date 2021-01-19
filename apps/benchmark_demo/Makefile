# Get directory that contains this Makefile
DEMO_ROOT := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

# define the CMSIS library path
CMSIS_LIB ?= $(DEMO_ROOT)/../common/libs/cmsis
RPMSG_DIR = $(DEMO_ROOT)/../common/ipc_rpmsg_lib

# User-supplied paths to dependencies
include $(MAKEENV)

# Boilerplate concerto configuration
CONCERTO_ROOT ?= $(abspath $(DEMO_ROOT)/../../concerto)
BUILD_MULTI_PROJECT := 1
BUILD_TARGET ?= $(CONCERTO_ROOT)/build_targets/processor-sdk/target.mak
BUILD_PLATFORM :=

# By default, only build linux userspace apps if we are not on Windows, but also
# allow users to choose on commandline.
BUILD_LINUX_APPS ?= 1
ifeq ($(HOST_OS),Windows_NT)
BUILD_LINUX_APPS = 0
endif

ifeq ($(TARGET_PLATFORM),AM65X)
# Recursively find all concerco.mak in the following directory list.
DIRECTORIES = $(HOST_ROOT) $(RPMSG_DIR)

# Targe combos supported in this demo
TARGET_COMBOS ?= AM65X:NO_OS:R5F:2:release:TIARMCGT 
endif

ifeq ($(TARGET_PLATFORM),AM64X)
# Recursively find all concerco.mak in the following directory list.
DIRECTORIES = $(HOST_ROOT) $(RPMSG_DIR)

# Targe combos supported in this demo
TARGET_COMBOS ?= AM64X:NO_OS:R5F:4:release:TIARMCGT \
                 AM64X:NO_OS:R5F:4:debug:TIARMCGT
endif

# "Invoke concerto"
include $(CONCERTO_ROOT)/rules.mak

# Build variable to support a RTOS/BM only (non-Linux) demo build
# Linux images will not be available in RTOS build flow
# By default, set for RTOS + Linux build
# Set RTOS_ONLY_BUILD=1 from console for RTOS only build
RTOS_ONLY_BUILD ?= 0

# When RTOS_ONLY_BUILD is set to 0, this build variable is
# used to determine the type of linux boot
# 0: Optimized
# 1: Development
LINUX_BUILD_TYPE ?= 0

ifeq ($(BUILD_LINUX_APPS),1)
all: rpmsg_json_app app_img_build
clean:: rpmsg_json_app_clean app_img_build_clean
.PHONY: rpmsg_json_app app_img_build
RPMSG_JSON_APP_BUILDDIR = webserver_app/linux_app/

rpmsg_json_app:
	. $(LINUX_ENV_SETUP); \
	make -C $(RPMSG_JSON_APP_BUILDDIR)

rpmsg_json_app_clean:
	. $(LINUX_ENV_SETUP); \
	make -C $(RPMSG_JSON_APP_BUILDDIR) clean
else
all: app_img_build
clean:: app_img_build_clean
.PHONY: app_img_build
endif

RTOS_BIN_PATH=$(HOST_ROOT)/$(BUILD_OUTPUT)
IMG1=mcu1_0,$(RTOS_BIN_PATH)/$(TARGET_PLATFORM)/R5F/NO_OS/$(TARGET_BUILD)/app_no_os_mcu1_0_adc_pwm.out
IMG2=mcu1_1,$(RTOS_BIN_PATH)/$(TARGET_PLATFORM)/R5F/NO_OS/$(TARGET_BUILD)/app_no_os_mcu1_1_cmsis_cfft.out
IMG3=mcu2_0,$(RTOS_BIN_PATH)/$(TARGET_PLATFORM)/R5F/NO_OS/$(TARGET_BUILD)/app_no_os_mcu2_0_cmsis_fir.out
IMG4=mcu2_1,$(RTOS_BIN_PATH)/$(TARGET_PLATFORM)/R5F/NO_OS/$(TARGET_BUILD)/app_no_os_mcu2_1_cmsis_foc.out

ifeq ($(RTOS_ONLY_BUILD),0)
HLOS_BIN_PATH=$(PSDK_PATH)/linux/board-support/prebuilt-images
ATF_IMG=mpu1_0,$(HLOS_BIN_PATH)/bl31.bin,0x70020000,0x70020000
OPTEE_IMG=load_only,$(HLOS_BIN_PATH)/bl32.bin,0x9e800000
ifeq ($(TARGET_PLATFORM),AM65X)
KERNEL_IMG=load_only,$(HLOS_BIN_PATH)/Image-am65xx-evm.bin,0x80080000
DTB_IMG=load_only,$(HLOS_BIN_PATH)/k3-am654-base-board.dtb,0x82000000
endif
ifeq ($(TARGET_PLATFORM),AM64X)
KERNEL_IMG=load_only,$(HLOS_BIN_PATH)/Image-am6x-evm.bin,0x80080000
DTB_IMG=load_only,$(HLOS_BIN_PATH)/k3-am64-base-board.dtb,0x82000000
endif
ifeq ($(LINUX_BUILD_TYPE),0)
HLOS_BOOT="optimized"
SPL_IMG=
else
HLOS_BOOT="development"
SPL_TYPE=load_only
SPL_FILE=$(HLOS_BIN_PATH)/u-boot-spl.bin-am6x-evm
SPL_ADDRESS=0x80080000
endif
else
HLOS_BOOT=
ATF_IMG=
OPTEE_IMG=
KERNEL_IMG=
DTB_IMG=
SPL_IMG=
endif

ifeq ($(TARGET_PLATFORM),AM65X)
SYSFWVERSION=V0
SYSFWFILE=sysfw_sr2.bin
endif

ifeq ($(TARGET_PLATFORM),AM64X)
SYSFWVERSION=V3
SYSFWFILE=sysfw.bin
endif

ifeq ($(HOST_OS),Windows_NT)
export GCC_LINUX_ARM_PATH=$(PSDK_PATH)/rtos/gcc-arm-9.2-2019.12-mingw-w64-i686-aarch64-none-elf
endif

app_img_build:
ifeq ($(BUILD_LINUX_APPS),1)
	$(DEMO_ROOT)/../../bin/dtb_boot_args.sh
endif
ifeq ($(RTOS_ONLY_BUILD)$(LINUX_BUILD_TYPE),01)
	$(COPY) "$(call PATH_CONV,$(SPL_FILE))" "$(call PATH_CONV,$(HLOS_BIN_PATH)/u-boot-spl.bin)"
	$(MAKE) -C $(PDK_PATH)/packages/ti/boot/sbl/tools/combined_appimage ATF_IMG=$(ATF_IMG) OPTEE_IMG=$(OPTEE_IMG) KERNEL_IMG=$(KERNEL_IMG) DTB_IMG=$(DTB_IMG) SPL_IMG=$(SPL_TYPE),$(HLOS_BIN_PATH)/u-boot-spl.bin,$(SPL_ADDRESS) HLOS_BOOT=$(HLOS_BOOT) IMG1=$(IMG1) IMG2=$(IMG2) IMG3=$(IMG3) IMG4=$(IMG4) BOARD=$(PDK_BOARD) SOC=$(PDK_SOC)
else
	$(MAKE) -C $(PDK_PATH)/packages/ti/boot/sbl/tools/combined_appimage ATF_IMG=$(ATF_IMG) OPTEE_IMG=$(OPTEE_IMG) KERNEL_IMG=$(KERNEL_IMG) DTB_IMG=$(DTB_IMG) SPL_IMG=$(SPL_IMG) HLOS_BOOT=$(HLOS_BOOT) IMG1=$(IMG1) IMG2=$(IMG2) IMG3=$(IMG3) IMG4=$(IMG4) BOARD=$(PDK_BOARD) SOC=$(PDK_SOC)
endif
#Create Benchmark Demo SD Card Image
	$(MKDIR) $(call PATH_CONV,$(RTOS_BIN_PATH)/$(TARGET_PLATFORM)/SDCardImage)
	$(COPY) "$(call PATH_CONV,$(PDK_PATH)/packages/ti/boot/sbl/tools/combined_appimage/bin/$(PDK_BOARD)/combined.appimage)" "$(call PATH_CONV,$(RTOS_BIN_PATH)/$(TARGET_PLATFORM)/SDCardImage/app)"
	$(COPY) "$(call PATH_CONV,$(PDK_PATH)/packages/ti/boot/sbl/binary/$(PDK_BOARD)/mmcsd/bin/sbl_mmcsd_img_hlos_mcu1_0_release.tiimage)" "$(call PATH_CONV,$(RTOS_BIN_PATH)/$(TARGET_PLATFORM)/SDCardImage/tiboot3.bin)"
	$(COPY) "$(call PATH_CONV,$(PDK_PATH)/packages/ti/drv/sciclient/soc/$(SYSFWVERSION)/$(SYSFWFILE))" "$(call PATH_CONV,$(RTOS_BIN_PATH)/$(TARGET_PLATFORM)/SDCardImage/sysfw.bin)"
ifeq ($(RTOS_ONLY_BUILD)$(LINUX_BUILD_TYPE),01)
	$(COPY) "$(call PATH_CONV,$(HLOS_BIN_PATH)/u-boot-am6x-evm.img)" "$(call PATH_CONV,$(RTOS_BIN_PATH)/$(TARGET_PLATFORM)/SDCardImage/u-boot.img)"
endif
#Create Benchmark Demo OSPI Image
	$(MKDIR) $(call PATH_CONV,$(RTOS_BIN_PATH)/$(TARGET_PLATFORM)/OSPIImage)
	$(COPY) "$(call PATH_CONV,$(PDK_PATH)/packages/ti/boot/sbl/tools/combined_appimage/bin/$(PDK_BOARD)/combined.appimage)" "$(call PATH_CONV,$(RTOS_BIN_PATH)/$(TARGET_PLATFORM)/OSPIImage/app)"
	$(COPY) "$(call PATH_CONV,$(PDK_PATH)/packages/ti/boot/sbl/binary/$(PDK_BOARD)/ospi/bin/sbl_ospi_img_hlos_mcu1_0_release.tiimage)" "$(call PATH_CONV,$(RTOS_BIN_PATH)/$(TARGET_PLATFORM)/OSPIImage/tiboot3.bin)"
	$(COPY) "$(call PATH_CONV,$(PDK_PATH)/packages/ti/drv/sciclient/soc/$(SYSFWVERSION)/$(SYSFWFILE))" "$(call PATH_CONV,$(RTOS_BIN_PATH)/$(TARGET_PLATFORM)/OSPIImage/sysfw.bin)"
ifeq ($(RTOS_ONLY_BUILD)$(LINUX_BUILD_TYPE),01)
	$(COPY) "$(call PATH_CONV,$(HLOS_BIN_PATH)/u-boot-am6x-evm.img)" "$(call PATH_CONV,$(RTOS_BIN_PATH)/$(TARGET_PLATFORM)/OSPIImage/u-boot.img)"
endif
	
app_img_build_clean:
	$(CLEANDIR) $(call PATH_CONV,$(PDK_PATH)/packages/ti/boot/sbl/tools/combined_appimage/bin/)
	$(CLEANDIR) $(call PATH_CONV,$(RTOS_BIN_PATH)/$(TARGET_PLATFORM)/SDCardImage)
	$(CLEANDIR) $(call PATH_CONV,$(RTOS_BIN_PATH)/$(TARGET_PLATFORM)/OSPIImage)
