#!/bin/bash

function cleanup {
	echo
}

trap cleanup EXIT TERM

INITRD_BASE=0x82400000

if [ -z "$DTC" ]; then
	DTC=dtc
fi

declare -A options_help
EXTRA_HELP=0
usage() {
	if [ x"$*" != x ]; then
		echo "ERROR: $*"
	fi
	if [ $EXTRA_HELP -eq 1 ]; then
	cat <<__END

__END
	fi
	echo -n "Usage: $0 "
	for option in "${!options_help[@]}"
	do
		arg=`echo ${options_help[$option]}|cut -d ':' -f1`
		if [ -n "$arg" ]; then
			arg=" $arg"
		fi
		echo -n "[-$option$arg] "
	done
	echo
	echo -e "\nWhere:"
	for option in "${!options_help[@]}"
	do
		arg=`echo ${options_help[$option]}|cut -d ':' -f1`
		txt=`echo ${options_help[$option]}|cut -d ':' -f2`
		tb="\t\t\t"
		if [ -n "$arg" ]; then
			arg=" $arg"
			tb="\t"
		fi
		echo -e "   -$option$arg:$tb$txt"
	done
	echo
	if [ $EXTRA_HELP -eq 1 ]; then
	cat <<__ENDEX

Examples of usage:-

1. Replace boot args in dtb
$0 -d my.dtb -b "console=ttyS1 init=/bin/sh"

2. Use a Build root rootfs.cpio
$0 -d my.dtb -i buildroot.cpio

3. Replace bootargs AND use a rootfs.cpio
$0 -d my.dtb -i buildroot.cpio -b "console=ttyS1 init=/bin/sh"

__ENDEX
	fi
}
options_help[d]="'device_tree_blob': Device tree blob - also known as .dtb file, if no other options are given, will dump dts"
options_help[i]="'initrd_file': Initrd binary file - also known as rootfs.cpio"
options_help[I]="'initrd_baseaddress': if the initrd needs to be located elsewhere (You may have to change ATF/u-boot defaults to ${INITRD_BASE})"
options_help[b]="'boot_args_string': Boot args you'd like to use(replaces existing one in dtb) - also known as kernel bootargs"
options_help[h]=": Get additional information"

while getopts "d:i:b:hI:" opt
do
	LEGOP=0
	case $opt in
	i)
		INITRD="$OPTARG"
		if [ ! -e "$INITRD" ]; then
			usage "Initrd file '$INITRD' is not a real file."
			exit 1
		fi
	;;
	I)
		INITRD_BASE="$OPTARG"
	;;
	d)
		DTB="$OPTARG"
		if [ ! -e "$DTB" ]; then
			usage "dtb '$DTB' is not a real file."
			exit 1
		fi
	;;
	b)
		CMDLINE="$OPTARG"
		if [ -z "$CMDLINE" ]; then
			usage "Boot args are missing."
			exit 1
		fi
	;;
	h)
		EXTRA_HELP=1
		usage
		exit 0
	;;
	\?)
		usage "Invalid Option '-$OPTARG'"
		exit 1
	;;
	:)
		usage "Option '-$OPTARG' Needs an argument."
		exit 1
	;;
	esac
done

if [ -z "$CMDLINE" -a -z "$INITRD" ]; then
	if [ -n "$DTB" ]; then
		${DTC}  -O dts -I dtb $DTB
		exit $?
	fi
	usage "Additional options needed. Use -h to see the complete help text"
	exit 1
fi

CHOSEN_NODE_FS=""
CHOSEN_NODE_BOOTARGS="";
if [ -n "$INITRD" ]; then
	FILESYSTEM_START=`printf "%d" ${INITRD_BASE}`
	FILESYSTEM_SIZE=`stat -Lc %s ${INITRD} 2>/dev/null || echo 0`
	FILESYSTEM_END=`expr ${FILESYSTEM_START} + ${FILESYSTEM_SIZE}`

	CHOSEN_NODE_FS="/{chosen {
		linux,initrd-start = <${FILESYSTEM_START}>;
		linux,initrd-end = <${FILESYSTEM_END}>;
	}; };"
fi

if [ -n "$CMDLINE" ]; then

	CHOSEN_NODE_BOOTARGS="/{chosen {
		bootargs = \"${CMDLINE}\";
	}; };"
fi

TMP=`mktemp`
(${DTC} -O dts -I dtb $DTB ; echo "${CHOSEN_NODE_FS}"; echo ${CHOSEN_NODE_BOOTARGS}; )  | ${DTC} -O dtb -o $TMP -

mv $TMP $DTB
