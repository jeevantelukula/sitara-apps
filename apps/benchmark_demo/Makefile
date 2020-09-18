# Get directory that contains this Makefile
DEMO_ROOT := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

# define the CMSIS library path
CMSIS_LIB ?= $(DEMO_ROOT)/../common/libs/cmsis
RPMSG_DIR = $(DEMO_ROOT)/../common/ipc_rpmsg_lib

RTOS_BIN_PATH=$(PDK_PATH)/../../apps/benchmark_demo/out/$(SITARA_XDC_DEVICE)/R5F/NO_OS/$(TARGET_BUILD)
IMG1=mcu1_0,$(RTOS_BIN_PATH)/app_no_os_mcu1_0_cmsis_cfft.out
IMG2=mcu1_1,$(RTOS_BIN_PATH)/app_no_os_mcu1_1_cmsis_cfft.out
IMG3=
IMG4=
HLOS_BIN_PATH=$(PDK_PATH)/../../linux/prebuilt-images
ATF_IMG=mpu1_0,$(HLOS_BIN_PATH)/bl31.bin,0x70000000,0x70000000
OPTEE_IMG=load_only,$(HLOS_BIN_PATH)/bl32.bin,0x9e800000
KERNEL_IMG=load_only,$(HLOS_BIN_PATH)/Image-am65xx-evm.bin,0x80080000
DTB_IMG=load_only,$(HLOS_BIN_PATH)/k3-am654-base-board.dtb,0x82000000
HLOS_BOOT="optimized"

# User-supplied paths to dependencies
include $(MAKEENV)

# Boilerplate concerto configuration
CONCERTO_ROOT ?= $(abspath $(DEMO_ROOT)/../../concerto)
BUILD_MULTI_PROJECT := 1
BUILD_TARGET ?= $(CONCERTO_ROOT)/build_targets/processor-sdk/target.mak
BUILD_PLATFORM :=

# Recursively find all concerco.mak in the following directory list.
DIRECTORIES = $(HOST_ROOT) $(RPMSG_DIR)

# Targe combos supported in this demo
TARGET_COMBOS ?= AM65X:NO_OS:R5F:2:release:TIARMCGT

# "Invoke concerto"
include $(CONCERTO_ROOT)/rules.mak

# By default, only build linux userspace apps if we are not on Windows, but also
# allow users to choose on commandline.
BUILD_LINUX_APPS ?= 1
ifeq ($(HOST_OS),Windows_NT)
BUILD_LINUX_APPS = 0
endif

ifeq ($(BUILD_LINUX_APPS),1)
all: rpmsg_json_app app_img_build
clean:: rpmsg_json_app_clean app_img_build_clean

RPMSG_JSON_APP_BUILDDIR = webserver_app/linux_app/

rpmsg_json_app:
	. $(LINUX_ENV_SETUP); \
	make -C $(RPMSG_JSON_APP_BUILDDIR)

rpmsg_json_app_clean:
	. $(LINUX_ENV_SETUP); \
	make -C $(RPMSG_JSON_APP_BUILDDIR) clean

app_img_build:
	make -C $(PDK_PATH)/packages/ti/boot/sbl/tools/combined_appimage ATF_IMG=$(ATF_IMG) OPTEE_IMG=$(OPTEE_IMG) KERNEL_IMG=$(KERNEL_IMG) DTB_IMG=$(DTB_IMG) HLOS_BOOT=$(HLOS_BOOT) IMG1=$(IMG1) IMG2=$(IMG2) BOARD=$(PDK_BOARD) SOC=$(PDK_SOC)
	
app_img_build_clean:
	rm -rf $(PDK_PATH)/packages/ti/boot/sbl/tools/combined_appimage/bin/
	
endif