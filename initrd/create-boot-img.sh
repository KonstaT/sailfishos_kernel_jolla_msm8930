#!/bin/sh

set -e

OLD_DIR=`pwd`
WORK_DIR=`dirname $0`/
KERNEL=$(find `pwd`/arch/arm/boot/zImage)
BOOTIMG=boot.img
RAMDISK_TGT=rootfs.cpio.gz

# Do NOT remove "root=/dev/mmcblk0p28", it is needed to support factory reset
# to older than 1.0.4.* SbJ images.
BOOTCMD="androidboot.hardware=qcom user_debug=31 ehci-hcd.park=3 maxcpus=2 zram.num_devices=2 root=/dev/mmcblk0p28"
if test -z $1; then
	echo "Error, no modules dir provided"
	exit 1
fi

if [[ "$1" = /* ]]; then
	MODULES_DIR=$1
else
	MODULES_DIR=$OLD_DIR/$1
fi

echo "Modules dir set to: \"$MODULES_DIR\""

if test ! -e $MODULES_DIR/modules.dep; then
	echo "Error, invalid modules directory: \"$1\""
	exit 1
fi

# We need to be in the work dir for local tools to be addable
cd $WORK_DIR

echo "Initializing for ramdisk creation.."

initialize-ramdisk.sh -w ./ -t "/usr/sbin/btrfs sbin/root-mount /sbin/btrfs-mount-repair /usr/bin/yamui res/images/*" -i init

echo "Packing ramdisk.."

moslo-build.sh -w ./ -m $MODULES_DIR -v 1

echo "Creating $BOOTIMG.."

${BINARY_PATH}mkbootimg --kernel $KERNEL --ramdisk ${RAMDISK_TGT} --cmdline "$BOOTCMD" --base 0x80200000 --offset 0x02000000 --pagesize 2048 -o $BOOTIMG

echo "Cleaning up.."
rm $RAMDISK_TGT
rm -rf initfs
rm -rf rootfs
rm -f libraries.txt
cd $OLD_DIR

exit 0
