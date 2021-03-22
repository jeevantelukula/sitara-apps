# Provide PATH to install directory of Sitara SDK
PSDK_PATH ?= $(abspath ../)

# Set SoC platform. Supported values: AM64X AM65X
export TARGET_PLATFORM ?= AM64X

# Derive path to Linux board support pacakge
export LINUX_ENV_SETUP = $(PSDK_PATH)/linux-devkit/environment-setup
export PSDK_PATH

