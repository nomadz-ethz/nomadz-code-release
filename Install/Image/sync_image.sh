#!/bin/bash

# synchronizes local config and binaries with the image
# this script currently requires root access because it uses mount command

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

set -e
set -o pipefail

if [ "$#" -lt 2 ]; then
    echo "Illegal number of parameters"
    echo "Usage: $0 image.opn binary-dir"
    exit 1
fi

# parameters
IMAGE="$1"
BINARY_DIR="$2"

if [ ! -f $BINARY_DIR/nomadz ] || [ ! -f $BINARY_DIR/naobridge ]; then
    echo "Binary directory passed does not contain nomadz and/or naobridge binaries"
    exit 1
fi

# extract filesystem and installer from .opn image
$DIR/extract_opn.sh $IMAGE ./input.ext3

# mount custom image
echo "Synchronizing filesystem with SDK..."
mkdir -p ./mnt
! mountpoint -q ./mnt || umount ./mnt
mount -o rw ./input.ext3 ./mnt

# synchronize binaries
cp $BINARY_DIR/naobridge ./mnt/nao/
cp $BINARY_DIR/nomadz ./mnt/nao/

# synchronize config directory
rsync --del --links --exclude=*.stx --exclude=*.log --exclude=.* --exclude=*.tc --exclude=.svn --exclude=/Images --exclude=/Logs --exclude=/Scenes --exclude=/Keys --exclude=nomadz --exclude=naobridge --chmod=u+rw,go+r,Dugo+x -zrch $DIR/../../Config/. ./mnt/nao/Config

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
