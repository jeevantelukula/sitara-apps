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

# Targe combos supported in this demo
TARGET_COMBOS ?= AM65X:SYSBIOS:R5F:2:release:TIARMCGT \
                 AM65X:NO_OS:R5F:2:release:TIARMCGT \
                 AM65X:SYSBIOS:A53:1:release:GCC_SYSBIOS_ARM

# "Invoke concerto"
include $(CONCERTO_ROOT)/rules.mak


# By default, only build linux userspace apps if we are not on Windows, but also
# allow users to choose on commandline.
BUILD_LINUX_APPS = 1
ifeq ($(HOST_OS),Windows_NT)
BUILD_LINUX_APPS = 0
endif

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
