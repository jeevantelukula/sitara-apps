#!/bin/bash

# Parse for the DTB File to Modify
DTB_LOAD_ARGS=(${1//,/ })
LINUX_DTB=${DTB_LOAD_ARGS[1]}

# Default boot args to use in the DTB. Update these as needed.
BOOTARGS="console=ttyS2,115200n8 earlycon=ns16550a,mmio32,0x02800000 rw rootwait rootfstype=ext4 root=/dev/mmcblk1p2"

# Modify the DTB
${PLSDK_PATH}/../sitara-apps/bin/dtbmod.sh -d $LINUX_DTB -b "$BOOTARGS" 
