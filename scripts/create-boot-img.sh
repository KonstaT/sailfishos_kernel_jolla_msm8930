#!/bin/sh

set -e

BOOTIMG=boot.img
RAMDISK_SRC=/boot/hw-ramdisk.gz
RAMDISK_TGT=final-ramdisk.gz
RAMDISKTMP=ramdisk-tmp

# Do NOT remove "root=/dev/mmcblk0p28", it is needed to support factory reset
# to older than 1.0.4.* SbJ images.
BOOTCMD="androidboot.hardware=qcom user_debug=31 ehci-hcd.park=3 maxcpus=2 root=/dev/mmcblk0p28"

clean_up()
{
	echo "Cleaning up.."
	rm $RAMDISK_TGT
	rm -rf $RAMDISKTMP
}

# Create ramdisk
echo "Creating ramdisk.."
if [ -e $RAMDISK_SRC ]; then
	mkdir -p $RAMDISKTMP
	cd $RAMDISKTMP

	echo "Unpacking mer-ramdisk.."
	gzip -cd $RAMDISK_SRC | cpio -imd --quiet

	echo "Copying our root-mount script.."
	cp -f ../scripts/btrfs-mount sbin/root-mount
	cp -f ../scripts/init init

	echo "Packing final ramdisk.."
	find . | cpio --quiet -H newc -o | gzip -9 -n > ../$RAMDISK_TGT

	cd ..
else
	echo "Please install mer-ramdisk to your build target"
	clean_up
	exit 1
fi



echo "Creating $BOOTIMG.."
KERNEL=$(find `pwd`/arch/arm/boot/zImage)

${BINARY_PATH}mkbootimg --kernel $KERNEL --ramdisk ${RAMDISK_TGT} --cmdline "$BOOTCMD" --base 0x80200000 --offset 0x02000000 --pagesize 2048 -o $BOOTIMG

clean_up

exit 0
