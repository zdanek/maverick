#!/bin/bash

### Script to prepare maverick for release.
# https://github.com/fnoop/maverick/issues/248
# This isn't really a puppet type action, but is hidden down in base module
#  so it's not found and run by accident, as it deletes all user data.
# It must be run as root, otherwise it will fail dismally.

# First stop maverick services
systemctl stop maverick-*

# Clean packages and cache
echo "Cleaning dpkg and apt"
apt-get autoremove --purge -y >/dev/null 2>&1
apt-get clean >/dev/null 2>&1

# Ensure maverick set to stable branch
echo "MAVERICK_BRANCH=stable" >/srv/maverick/data/config/maverick/maverick-branch.conf
chown mav:mav /srv/maverick/data/config/maverick/maverick-branch.conf

echo "Removing logs, config and data"
# Clean as much of /var/log as possible
rm -f /var/log/*.log*
rm -f /var/log/apt/*
rm -f -rf /var/log/installer
rm -f /var/log/faillog /var/log/lastlog
rm -f /var/log/lightdm/*
rm -f /var/log/syslog* /var/log/dmesg
rm -f /var/log/unattended-upgrades/*
rm -f /var/log/btmp* /var/log/wtmp*

# Remove tmp user used to initially install OS
rm -rf /home/tmp
userdel tmp >/dev/null 2>&1

# Remove initial bootstrap maverick from common locations
rm -rf /root/maverick
rm -rf /home/pi/maverick

# Remove any root data
rm -rf /root/.cache /root/.gnupg

# Remove build directories (leave install flags in place)
rm -rf /srv/maverick/var/build/*

# Remove tmp data
rm -rf /var/tmp/* /var/crash/* /var/backup/* 
rm -rf /tmp/*

# Delete puppet client data
rm -rf /var/lib/puppet
# Delete other var/lib data
rm -f /var/lib/dhcp/*
rm -rf /var/lib/dhcpcd5/*

# Clean up maverick user data
# Note: MUST rerun maverick configure again after removing these files, to restore defaults.
# Otherwise odd behaviour will happen.
rm /srv/maverick/.bash_history
rm /srv/maverick/.c9/state.settings
rm -rf /srv/maverick/.c9/metadata/workspace/* /srv/maverick/.c9/metadata/tab* /srv/maverick/.c9/tmp /srv/maverick/.c9/node /srv/maverick/.c9/node_modules /srv/maverick/.c9/lib
rm -rf /srv/maverick/.cache /srv/maverick/.config /srv/maverick/.gconf /srv/maverick/.gnupg /srv/maverick/.ICEauthority /srv/maverick/.local 
rm -rf /srv/maverick/.gitconfig /srv/maverick/.git-credential-cache /srv/maverick/.subversion
rm -rf /srv/maverick/.[Xx]*
rm -rf /srv/maverick/.ros/log/*

# Remove maverick config
find /srv/maverick/data/config -type f -delete
rm /srv/maverick/software/maverick/conf/localconf.json
rm -f /srv/maverick/software/maverick/conf/local-nodes/*.json
# Restore localconf.json
cd /srv/maverick/software/maverick; git checkout conf/localconf.json

# Create generic EFI boot
if [ -e /boot/efi ]; then
    mkdir /boot/efi/EFI/BOOT
    cp /boot/efi/EFI/ubuntu/grubx64.efi /boot/efi/EFI/BOOT/bootx64.efi
fi

echo "Recreating gstreamer cache"
su - -c gst-inspect-1.0 mav >/dev/null 2>&1 # restore gstreamer .cache

echo
echo "Running maverick to regenerate any removed files or config"
maverick configure
systemctl stop maverick-*

# Clean up maverick data
find /srv/maverick/data/logs -type f -delete
find /srv/maverick/data/mavlink -type f -delete
rm -f /srv/maverick/data/vision_landing/*
find /srv/maverick/var/log -path /srv/maverick/var/log/build -prune -o -type f -exec rm -f {} \;
rm -f /srv/maverick/var/log/vision_landing/last.log
rm -rf /srv/maverick/var/log/ros/fc/* /srv/maverick/var/log/ros/sitl/*
rm -f /srv/maverick/var/run/*

echo "Maverick preparation complete, shutting down cleanly"
sudo sync
sudo systemctl poweroff
