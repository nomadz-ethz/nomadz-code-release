#!/bin/bash

# WARNING: first manually
# (1) first install public key in (Config/Keys)
# (2) update for NOPASSWD access for Nao user (change line in /etc/sudoers.d/01_sudoers_nao to `nao     ALL=(ALL) NOPASSWD:ALL`)

if [ $# != 2 ]; then
    echo "Error: give IP as first argument and robot name second"
    exit
fi

KEYFILE=../Config/Keys/id_rsa_nao

REMOTE=$1
HOSTNAME=$2

# create directories
ssh -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@$REMOTE "mkdir -p ~/bin"
ssh -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@$REMOTE "mkdir -p ~/logs"
ssh -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@$REMOTE "mkdir -p ~/Profiles"
ssh -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@$REMOTE "mkdir -p ~/tmp-install"

# create robocup.conf for robocup mode
ssh -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@$REMOTE "touch ~/robocup.conf"

# set hostname
ssh -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@$REMOTE "echo $HOSTNAME | sudo tee /etc/hostname"
ssh -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@$REMOTE "sudo hostname -F /etc/hostname"

# install bin
scp -r -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet ./Files/bin nao@$REMOTE:~/
# install systemd service files
scp -r -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet ./Startup/* nao@$REMOTE:~/tmp-install/
ssh -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@$REMOTE "sudo mv ~/tmp-install/* /etc/systemd/user/"
# enable all services
ssh -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@$REMOTE "systemctl --user daemon-reload"
ssh -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@$REMOTE "systemctl --user enable naobridge"
ssh -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@$REMOTE "systemctl --user enable nomadz"
ssh -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@$REMOTE "systemctl --user enable network-wired"
ssh -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@$REMOTE "systemctl --user enable network-wireless"

# updating config directory
rsync --del --links --exclude=*.stx --exclude=*.log --exclude=.* --exclude=*.tc --exclude=.svn --exclude=/Images --exclude=/Logs --exclude=/Scenes --exclude=/Keys --exclude=nomadz --exclude=naobridge --chmod=u+rw,go+r,Dugo+x -zrche "ssh -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet" ../Config/. nao@$REMOTE:/home/nao/Config

# put profiles
scp -r -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet ../Network/Profiles/* nao@$REMOTE:~/Profiles

ssh -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@$REMOTE "systemctl --user start network-wireless"

echo "Check if wireless works now, afterwards the robot can be restarted!"

# delete temporary install-dir
ssh -i $KEYFILE -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=quiet nao@$REMOTE "rm -rf ~/tmp-install"
