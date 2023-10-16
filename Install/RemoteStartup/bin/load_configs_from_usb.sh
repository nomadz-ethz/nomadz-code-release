#! /bin/bash

MOUNT_POINT=/home/nao/usb_drive

# Make sure the mount point exists
mkdir -p $MOUNT_POINT

# Mount the usb drive
DEVICE=`ls /dev/sd[a-z][0-9]`
mount $DEVICE $MOUNT_POINT
chmod -R 777 $MOUNT_POINT

# Load the field dimensions from the usb drive
python /home/nao/bin/config_field_dimensions.py ${MOUNT_POINT}/field_dimensions.json