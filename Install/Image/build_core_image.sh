#!/bin/bash

# creates a core opn image from the base Softbank image
# this script currently requires root access because it uses mount command

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

set -e
set -o pipefail

if [ "$#" -lt 2 ]; then
    echo "Illegal number of parameters"
    echo "Usage: $0 base.opn output.opn"
    exit 1
fi

# parameters
INPUT_IMAGE="$1"
OUTPUT_IMAGE="$2"

# extract filesystem and installer from .opn image
$DIR/extract_opn.sh $INPUT_IMAGE ./nao.ext3

# mount original nao image
echo "Customizing filesystem..."
mkdir -p ./mnt
! mountpoint -q ./mnt || umount ./mnt
mount -o rw ./nao.ext3 ./mnt

# remove large directories that are not needed
rm -rf ./mnt/opt/aldebaran/share/tts ./mnt/opt/aldebaran/share/naoqi

# install the custom public key for auto-login
mkdir -m700 -p ./mnt/nao/.ssh
cat $DIR/../../Config/Keys/id_rsa_nao.pub > ./mnt/nao/.ssh/authorized_keys

# setup ssh config without password auth
cat - <<"EOT" > ./mnt/etc/ssh/sshd_config
PermitRootLogin no
PasswordAuthentication no
PermitEmptyPasswords no
ChallengeResponseAuthentication no
UsePAM no
PrintMotd no
PrintLastLog no
UsePrivilegeSeparation sandbox		# Default for new installations.
ClientAliveInterval 30
UseDNS no
Subsystem	sftp	internal-sftp
EOT

# install core startup files
cp $DIR/../Startup/* ./mnt/etc/systemd/user

# install the remote startup files
cp $DIR/../RemoteStartup/*.service ./mnt/etc/systemd/user

# enable robocup mode
touch ./mnt/nao/robocup.conf

# install core nomadz files
rsync -r $DIR/../Files/. ./mnt/nao
rsync -r $DIR/../Network/Profiles/. ./mnt/nao/Profiles

# install the remote startup scripts
rsync -r $DIR/../RemoteStartup/bin/* ./mnt/nao/bin/

# sync config directory
mkdir -p ./mnt/nao/Config
rsync --del --links --exclude=*.stx --exclude=*.log --exclude=.* --exclude=*.tc --exclude=.svn --exclude=/Images --exclude=/Logs --exclude=/Scenes --exclude=/Keys --exclude=nomadz --exclude=naobridge --chmod=u+rw,go+r,Dugo+x -zrch $DIR/../../Config/. ./mnt/nao/Config

# install setup script and create target to run initial setup
cp $DIR/setup.sh ./mnt/nao/setup.sh
chmod +x ./mnt/nao/setup.sh
cat - <<"EOT" > ./mnt/etc/systemd/system/nomadz-setup.service
[Unit]
Description=Complete custom NomadZ installation
After=data-skeleton.service
Requires=data-skeleton.service
Before=user@1001.service
DefaultDependencies=no

[Service]
Type=oneshot
ExecStart=/bin/bash /nao/setup.sh
ExecStartPost=/bin/systemctl disable nomadz-setup

[Install]
WantedBy=sysinit.target
EOT
ln -s ../nomadz-setup.service ./mnt/etc/systemd/system/sysinit.target.wants/nomadz-setup.service
echo "Done!"

# generate filesystem with correct UUID and maximum size for Nao's system partition
echo "Generate filesystem..."
mke2fs -F -U 42424242-1120-1120-1120-424242424242 -L "NomadZ-system" -b 4096 -t ext3 -d ./mnt ./output.ext3 999168

echo "Done!"

# generate opn
$DIR/generate_opn.sh ./output.ext3 $OUTPUT_IMAGE

# cleaning up
umount ./mnt
rm -rf ./mnt
rm -f ./nao.ext3 ./output.ext3
