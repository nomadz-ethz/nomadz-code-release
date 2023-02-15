#!/bin/bash

# synchronizes specific changes in local Yocto SDK with image
# this script currently requires root access because it uses mount command

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

set -e
set -o pipefail

if [ "$#" -lt 2 ]; then
    echo "Illegal number of parameters"
    echo "Usage: $0 image.opn yocto-sdk-dir"
    exit 1
fi

# parameters
IMAGE="$1"
YOCTO_SDK="$2"

if [ ! -d $YOCTO_SDK/sysroots/core2-32-sbr-linux/lib ]; then
    echo "Directory passed does not point to valid Yocto-SDK"
    exit 1
fi

# extract filesystem and installer from .opn image
$DIR/extract_opn.sh $IMAGE ./input.ext3

# mount custom image
echo "Synchronizing filesystem with SDK..."
mkdir -p ./mnt
! mountpoint -q ./mnt || umount ./mnt
mount -o rw ./input.ext3 ./mnt

# synchronize libopencv
rsync -a $YOCTO_SDK/sysroots/core2-32-sbr-linux/lib/libopencv* ./mnt/lib/

# generate filesystem with correct UUID and maximum size for Nao's system partition
echo "Generate filesystem..."
mke2fs -F -U 42424242-1120-1120-1120-424242424242 -L "NomadZ-system" -b 4096 -t ext3 -d ./mnt ./output.ext3 999168

echo "Done!"

# generate opn
$DIR/generate_opn.sh ./output.ext3 $IMAGE

# cleaning up
umount ./mnt
rm -rf ./mnt
rm -f ./input.ext3 ./output.ext3
