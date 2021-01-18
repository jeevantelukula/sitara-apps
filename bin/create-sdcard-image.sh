#!/bin/bash -e

usage() {
    cat << __EOM

Usage: $0 <boot_dir> <rootfs_dir|rootfs_tar> <output.wic>

__EOM
}

notice() {
    echo
    echo "############################################################"
    while [ $# -gt 0 ]
    do
        echo "# $1"
        shift
    done
    echo "############################################################"
}

[ $# -eq 3 ] || { usage; exit 1; }

BOOT=$1
ROOT=$2
IMG_FILE=$3

DEPS="dosfstools mtools uuid-runtime parted pseudo xz-utils"
check_tools() {
    local deps=""
    local ret=0

    which mkfs.ext4 > /dev/null 2>&1 || deps="${deps}e2fsprogs "
    which mkdosfs > /dev/null 2>&1 || deps="${deps}dosfstools "
    which mcopy > /dev/null 2>&1 || deps="${deps}mtools "
    which uuidgen > /dev/null 2>&1 || deps="${deps}uuid-runtime "
    which parted > /dev/null 2>&1 || deps="${deps}parted "
    which pseudo > /dev/null 2>&1 || deps="${deps}pseudo "
    which xz > /dev/null 2>&1 || deps="${deps}xz-utils "

    if [ ! -z "$deps" ]
    then
        cat << __EOM

Missing dependencies! Please run the following command to
install the required packages:

$ sudo apt-get install $deps

__EOM
        ret=1
    fi

    return $ret
}

check_tools_versions() {
    local ret=0

    # Only known version requirement is that mkfs.ext4 must support the "-d" option
    mkfs.ext4 -v 2>&1 | grep -- '\[-d' > /dev/null 2>&1
    if [ $? -ne 0 ]
    then
        cat << __EOM

The "mkfs.ext4" tool version does not meet requirements
(>= mke2fs 1.44.1). Your Ubuntu distro is likely out of
date. Please upgrade to Ubuntu 18.04 or later.

__EOM
        ret=1
    fi

    return $ret
}

notice "Checking tools..."
check_tools || exit 1
check_tools_versions || exit 1

# TEmporary partition files
SCRATCH_DIR=$(mktemp -d /tmp/sitara-sdcard-scratch.XXXXXXXX)
BOOT_PART=$SCRATCH_DIR/boot.vfat
ROOT_PART=$SCRATCH_DIR/root.ext4
ROOT_STAGING=$SCRATCH_DIR/root_staging


#Extra space in Kb
BOOT_EXTRA=102400
ROOT_EXTRA=102400

OVERHEAD_FACTOR_NUM=13
OVERHEAD_FACTOR_DEN=10

MBR_OVERHEAD=1 # sectors
SECTOR_SIZE=512 # bytes
ALIGN=1024 # K Bytes


# Unfortunately, pseudo does not know where it is nor its library...
# Hopefully this holds up for Ubuntu distros at the least
export PSEUDO_BINDIR=/usr/bin
export PSEUDO_LIBDIR=/usr/lib/x86_64-linux-gnu/pseudo
export PSEUDO_PASSWD=$ROOT
export PSEUDO_NOSYMLINKEXP=1
export PSEUDO_LOCALSTATEDIR=$SCRATCH_DIR/pseudo


get_dosfs_size() {
    local dir="$1"
    local extra="$2"
    local size

    size=$(du -bks $BOOT | awk '{print $1}')
    size=$[size + BOOT_EXTRA]

    # align to 32 blocks
    size=$[size * OVERHEAD_FACTOR_NUM / OVERHEAD_FACTOR_DEN]

    # Some dosfstools require some 32 alignment
    size=$[size + 32 - 1 ]
    size=$[size / 32]
    size=$[size * 32]

    echo $size
}

get_dosfs_uuid() {
    local uuid=$(uuidgen)

    # Make uppercase and only take first 8 characters
    uuid=0x$(echo ${uuid^^} | sed -e 's|-.*||')

    echo $uuid
}
# Populate BOOT and ROOT

########################################
# Create boot partition
########################################
notice "Creating boot partition..."
# Calculate BOOT size
boot_size=$(get_dosfs_size "$BOOT" $BOOT_EXTRA)
boot_sectors=$[boot_size * 1024 / SECTOR_SIZE]
boot_uuid=$(get_dosfs_uuid)

# Create bot partition file
mkdosfs -n boot -i $boot_uuid -S $SECTOR_SIZE -C $BOOT_PART $boot_size
mcopy -i $BOOT_PART -s $BOOT/* ::/

# Check contents
#mdir -i $BOOT_PART

# Not sure if this is necessary
chmod 644 $BOOT_PART

get_extfs_size() {
    local dir=$1
    local extra=$2

    size=$(du -ks $dir | awk '{print $1}')
    size=$[size + extra]
    size=$[size * OVERHEAD_FACTOR_NUM / OVERHEAD_FACTOR_DEN]

    echo $size
}

get_extfs_uuid() {
    uuidgen
}

########################################
# Create rootfs partition
########################################
notice "Creating rootfs partition..."

# Create temporary rootfs to get permissions set as root using pseudo
mkdir $ROOT_STAGING
if [ -d $ROOT ]
then
    pseudo cp -r $ROOT/* $ROOT_STAGING/
else
    # Assume a tarball
    pseudo tar -C $ROOT_STAGING -xf $ROOT
fi

# Get rootfs size
root_size=$(get_extfs_size "$ROOT_STAGING" ROOT_EXTRA)
root_sectors=$[root_size * 1024 / SECTOR_SIZE]
root_uuid=$(get_extfs_uuid)

# Create rootfs partition file
truncate -s $[root_size * 1024] $ROOT_PART
pseudo mkfs.ext4 -F -i 8192 $ROOT_PART -L root -U $root_uuid -d $ROOT_STAGING
fsck.ext4 -pvfD $ROOT_PART



########################################
# Calcualte disk image layout
########################################
notice "Calculating disk layout..."
get_align_sectors() {
    local start=$1
    local align=0

    # Calc partition alignment (sectors)
    let "align = $start % ($ALIGN * 1024 / $SECTOR_SIZE)"
    [ $align -eq 0 ] || let "align = ($ALIGN * 1024 / $SECTOR_SIZE) - align"

    echo $align
}

offset=0

# Calc overhead
overhead=$MBR_OVERHEAD
offset=$[offset+overhead]

# Align partition start
align_sectors=$(get_align_sectors $offset)
offset=$[offset+align_sectors]

boot_start=$offset

offset=$[offset+boot_sectors]

# Align partition start
align_sectors=$(get_align_sectors $offset)
offset=$[offset+align_sectors]

root_start=$offset

offset=$[offset+root_sectors]


########################################
# Initialize disk image
########################################
notice "Constructing disk image..."
truncate -s $[offset*SECTOR_SIZE] $IMG_FILE
parted -s $IMG_FILE mklabel msdos

# Set disk identifier (32-bit randowm integer
# (Found this in WIC...)
#seek 0x1B8 $IMG_FILE
#write $(rand 0xFFFFFFFF) $IMG_FILE
dd bs=1 count=4 seek=440 conv=notrunc if=/dev/urandom of=$IMG_FILE

# Configure the boot partition
parted -s $IMG_FILE unit s mkpart primary fat32 $boot_start $[boot_start + boot_sectors - 1]
parted -s $IMG_FILE set 1 boot on

# Configure the root partition
parted -s $IMG_FILE unit s mkpart primary ext2 $root_start $[root_start + root_sectors - 1]

# Install partitions
dd bs=$SECTOR_SIZE seek=$boot_start conv=notrunc if=$BOOT_PART of=$IMG_FILE
dd bs=$SECTOR_SIZE seek=$root_start conv=notrunc if=$ROOT_PART of=$IMG_FILE

# cleanup
rm -rf $SCRATCH

notice "Success"
