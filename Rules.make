# Provide PATH to install directory of Sitara SDK
PSDK_PATH ?= $(abspath ../)

# Set SoC platform. Supported values: AM64X AM65X
export TARGET_PLATFORM ?= AM64X

# Derive path to RTOS board support package
PRSDK_PATH = $(PSDK_PATH)/rtos

# Derive path to Linux board support pacakge
export PLSDK_PATH = $(PSDK_PATH)/linux

# Assume concerto is in this directory
export CONCERTO_ROOT = $(abspath ./concerto)

# Try using wildcards to make this simpler
# And if there are multiples, use the latest version
find_package_in = $(lastword $(sort $(wildcard $(1)/$(2))))

# Define prefixes for directories that contain versions
TIARMCGT_PREFIX         = ti-cgt-arm_
PDK_PREFIX              = pdk_
GCC_SYSBIOS_ARM_PREFIX  = gcc-arm-
BIOS_PREFIX             = bios_
XDCTOOLS_PREFIX         = xdctools_
ETHERCAT_PREFIX         = pru_icss_ethercat

# Set paths
export TIARMCGT_ROOT           = $(call find_package_in,$(PRSDK_PATH),$(TIARMCGT_PREFIX)*)
export GCC_SYSBIOS_ARM_ROOT    = $(call find_package_in,$(PRSDK_PATH),$(GCC_SYSBIOS_ARM_PREFIX)*)
export BIOS_PATH               = $(call find_package_in,$(PRSDK_PATH),$(BIOS_PREFIX)*)
export XDCTOOLS_PATH           = $(call find_package_in,$(PRSDK_PATH),$(XDCTOOLS_PREFIX)*)
export PDK_PATH                = $(call find_package_in,$(PRSDK_PATH),$(PDK_PREFIX)*)
export LINUX_ENV_SETUP         = $(PLSDK_PATH)/linux-devkit/environment-setup
export ECAT_PATH               = $(call find_package_in,$(PRSDK_PATH),$(ETHERCAT_PREFIX)*)
export PSDK_PATH

ifeq ($(BUILD_DEBUG),1)
$(info TIARMCGT_ROOT         = $(TIARMCGT_ROOT))
$(info GCC_SYSBIOS_ARM_ROOT  = $(GCC_SYSBIOS_ARM_ROOT))
$(info BIOS_PATH             = $(BIOS_PATH))
$(info XDCTOOLS_PATH         = $(XDCTOOLS_PATH))
$(info PDK_PATH              = $(PDK_PATH))
endif
