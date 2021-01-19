# DTB File to Modify
LINUX_DTB=${PLSDK_PATH}/board-support/prebuilt-images/k3-am64-base-board.dtb

# Default boot args to use in the DTB. Update these as needed.
BOOTARGS="console=ttyS2,115200n8 earlycon=ns16550a,mmio32,0x02800000 rw rootwait rootfstype=ext4 root=/dev/mmcblk1p2"

# Modify the DTB
${PLSDK_PATH}/../sitara-apps/bin/dtbmod.sh -d $LINUX_DTB -b "$BOOTARGS" 
