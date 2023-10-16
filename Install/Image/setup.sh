#!/bin/bash

# check if already installed
if [ -f /home/nao/robocup.conf ]; then
    echo "Already initialized"
    exit 0
fi

# remount root read-write
mount -o remount,rw /

# give root permissions to Nao user
sed -i 's/nao     ALL=(ALL) ALL/nao     ALL=(ALL) NOPASSWD:ALL/' /etc/sudoers.d/01_sudoers_nao

# set hostname
echo "Nao" > /etc/hostname
hostname -F /etc/hostname

# move nao folders to home
rsync -rch --links --exclude=/etc --exclude=setup.sh /nao/. /home/nao

# enable basic nomadz functionality
mkdir -p /home/nao/.config/systemd/user/default.target.wants/
ln -sf /etc/systemd/user/load-configs-from-usb.service /home/nao/.config/systemd/user/default.target.wants/load-configs-from-usb.service
ln -sf /etc/systemd/user/naobridge.service /home/nao/.config/systemd/user/default.target.wants/naobridge.service
ln -sf /etc/systemd/user/nomadz.service /home/nao/.config/systemd/user/default.target.wants/nomadz.service
ln -sf /etc/systemd/user/network-wired.service /home/nao/.config/systemd/user/default.target.wants/network-wired.service
ln -sf /etc/systemd/user/network-wireless.service /home/nao/.config/systemd/user/default.target.wants/network-wireless.service

# create logs folder
mkdir -p /home/nao/logs

# fix permissions for nao folder
chown -R nao:nao /home/nao

# remove naoqi notifications
rm -rf /media/internal/notification

# remount root read-only
mount -o remount,ro /

# ensure nomadz-setup is not run anymore on reboot
rm /etc/systemd/system/sysinit.target.wants/nomadz-setup.service
rm /data/etc/systemd/system/sysinit.target.wants/nomadz-setup.service
