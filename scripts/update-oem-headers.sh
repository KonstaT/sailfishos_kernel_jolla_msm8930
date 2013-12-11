#!/bin/sh

set -e

SOURCE_PATH=$1

if [ ! -d $SOURCE_PATH ]; then
  echo "Given source path '$SOURCE_PATH' for oem headers doesn't exist."
  exit 1
fi

cp $SOURCE_PATH/header_comm/* ./arch/arm/mach-msm/include/mach/
cp $SOURCE_PATH/proc_comm/* arch/arm/mach-msm/include/mach/
cp $SOURCE_PATH/qmi/oem_log_def.h ./arch/arm/mach-msm/include/mach/

exit 0
 
