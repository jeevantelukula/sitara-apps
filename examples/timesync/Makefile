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
ifeq ($(TARGET_PLATFORM),AM65X)
TARGET_COMBOS ?= AM65X:SYSBIOS:R5F:2:release:TIARMCGT \
                 AM65X:SYSBIOS:R5F:2:debug:TIARMCGT
endif
ifeq ($(TARGET_PLATFORM),AM64X)
TARGET_COMBOS ?= AM64X:SYSBIOS:R5F:2:release:TIARMCGT \
                 AM64X:SYSBIOS:R5F:2:debug:TIARMCGT \
                 AM64X:NO_OS:M4F:2:release:TIARMCGT \
                 AM64X:NO_OS:M4F:2:debug:TIARMCGT \
                 AM64X:SYSBIOS:A53:2:release:GCC_SYSBIOS_ARM \
                 AM64X:SYSBIOS:A53:2:debug:GCC_SYSBIOS_ARM
endif
# "Invoke concerto"
include $(CONCERTO_ROOT)/rules.mak

