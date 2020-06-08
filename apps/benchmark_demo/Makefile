# Get directory that contains this Makefile
DEMO_ROOT := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

# define the CMSIS library path
CMSIS_LIB ?= $(DEMO_ROOT)/../common/libs/cmsis
RPMSG_DIR = $(DEMO_ROOT)/../common/ipc_rpmsg

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
BUILD_LINUX_APPS = 1
ifeq ($(HOST_OS),Windows_NT)
BUILD_LINUX_APPS = 0
endif

ifeq ($(BUILD_LINUX_APPS),1)
all: rpmsg_json_app
clean: rpmsg_json_app_clean

RPMSG_JSON_APP_BUILDDIR = webserver_app/linux_app/

rpmsg_json_app:
	. $(LINUX_ENV_SETUP); \
	make -C $(RPMSG_JSON_APP_BUILDDIR)

rpmsg_json_app_clean:
	. $(LINUX_ENV_SETUP); \
	make -C $(RPMSG_JSON_APP_BUILDDIR) clean

endif