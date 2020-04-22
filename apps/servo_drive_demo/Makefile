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
