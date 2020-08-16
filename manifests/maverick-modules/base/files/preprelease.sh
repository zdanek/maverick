#!/bin/bash

### Script to prepare maverick for release.
# https://github.com/goodrobots/maverick/issues/248
# This isn't really a puppet type action, but is hidden down in base module
#  so it's not found and run by accident, as it deletes all user data.
# It must be run as root, otherwise it will fail dismally.

# First stop maverick services
systemctl stop maverick-* >/dev/null 2>&1

# Clean packages and cache
echo "Cleaning dpkg and apt"
apt-get autoremove --purge -y >/dev/null 2>&1
apt-get clean >/dev/null 2>&1

# Ensure maverick set to stable branch
echo "MAVERICK_BRANCH=stable" >/srv/maverick/config/maverick/maverick-branch.conf
chown mav:mav /srv/maverick/config/maverick/maverick-branch.conf

echo "Removing logs, config and data"
# Clean as much of /var/log as possible
systemctl restart rsyslog
find /var/log -type f -delete
systemctl restart rsyslog
find /var/log -type f -delete

# Remove tmp user used to initially install OS
rm -rf /home/tmp
userdel tmp >/dev/null 2>&1

# Clean up ublinux home directory
if [ -d /home/ubilinux ]; then
    rm -rf /home/ubilinux/maverick
    rm -f /home/ubilinux/.bash_history /home/ubilinux/.Xauthority /home/ubilinux/.xsession-errors
    rm -rf /home/ubilinux/.cache /home/ubilinux/.config /home/ubilinux/.gnupg /home/ubilinux/.local
fi

# Remove initial bootstrap maverick from common locations
rm -rf /root/maverick
rm -rf /home/pi/maverick

# Remove any root data
rm -rf /root/.cache /root/.gnupg

# Remove build directories (leave install flags in place)
rm -rf /srv/maverick/var/build/*

# Remove tmp data
rm -rf /var/tmp/* /var/crash/* /var/backups/* /var/cache/*
rm -rf /tmp/*
# Recreate debconf cache
debconf-show --listdbs

# Delete puppet client data
rm -rf /var/lib/puppet
# Delete other var/lib data
rm -f /var/lib/dhcp/*
rm -rf /var/lib/dhcpcd5/*

# Clean up maverick user data
# Note: MUST rerun maverick configure again after removing these files, to restore defaults.
# Otherwise odd behaviour will happen.
rm -f /srv/maverick/.bash_history
rm /srv/maverick/.c9/state.settings
rm -rf /srv/maverick/.c9/metadata/workspace/* /srv/maverick/.c9/metadata/tab* /srv/maverick/.c9/tmp
rm -rf /srv/maverick/.cache /srv/maverick/.config /srv/maverick/.gconf /srv/maverick/.gnupg /srv/maverick/.ICEauthority /srv/maverick/.local /srv/maverick/.mozilla /srv/maverick/.gnome
rm -rf /srv/maverick/.gitconfig /srv/maverick/.git-credential-cache /srv/maverick/.subversion
rm -rf /srv/maverick/.gazebo /srv/maverick/.ignition /srv/maverick/.nv 
rm -rf /srv/maverick/.node-gyp /srv/maverick/.npm .python_history
rm -rf /srv/maverick/.vscode-server /srv/maverick/.vscode-server-insiders
rm -rf /srv/maverick/.[Xx]*
rm -rf /srv/maverick/.ros/log/* /srv/maverick/.ros/rospack_cache* /srv/maverick/.ros/roscore*.pid
rm -rf /srv/maverick/.ssh
rm -rf /srv/maverick/.pki
rm -rf /srv/maverick/.gitbook /srv/maverick/.bundle /srv/maverick/.gem
rm -f /srv/maverick/.wget-hsts /srv/maverick/.zeroTierOneAuthToken /srv/maverick/.python_history /srv/maverick/.lesshst /srv/maverick/.ICEauthority /srv/maverick/.viminfo
rm -f /srv/maverick/README.txt

# Remove maverick config
find /srv/maverick/config -path /srv/maverick/config/maverick -prune -o -type f -exec rm -f {} \;
rm -f /srv/maverick/config/maverick/localconf.json
rm -f /srv/maverick/config/maverick/local-nodes/*.json

# Clear out root homedirectory
rm -rf /root/.config /root/.gem /root/.npm /root/.subversion
rm -f /root/.bash_history /root/.wget-hsts

# Create generic EFI boot
if [ -e /boot/efi ]; then
    mkdir /boot/efi/EFI/BOOT
    cp /boot/efi/EFI/ubuntu/grubx64.efi /boot/efi/EFI/BOOT/bootx64.efi
fi

rm -f /etc/network/interfaces
rm -f /etc/wpa_supplicant/wpa_supplicant.conf
rm -f /etc/udev/rules.d/10-network-customnames.rules

# Clean up maverick data
find /srv/maverick/data/logs -type f -delete
find /srv/maverick/data/mavlink -type f -delete
find /srv/maverick/data/vision -type f -delete
find /srv/maverick/data/analysis -type f -delete
find /srv/maverick/data/network -type f -delete
rm -rf /srv/maverick/var/lib/influxdb

# Remove large tegra packages, outside of manifests.  This is so users can uninstall/reinstall at will
if [ -f /etc/nv_tegra_release ]; then
    apt purge -y libcudnn8-dev thunderbird
fi
rm -f /README.txt

# Remove ubuntu old snaps crap - seriously..
/srv/maverick/software/maverick/manifests/maverick-modules/base/files/remove-old-snaps.sh
rm -rf /var/lib/snapd/cache/*

echo "Recreating gstreamer cache"
su - -c gst-inspect-1.0 mav >/dev/null 2>&1 # restore gstreamer .cache

echo
echo "Running maverick to regenerate any removed files or config"
maverick configure --env=flight
systemctl stop maverick-* >/dev/null 2>&1

# Final clean up of var data
find /srv/maverick/data/logs -type f -delete
find /srv/maverick/data/mavlink -type f -delete
find /srv/maverick/data/vision -type f -delete
find /srv/maverick/var/log -path /srv/maverick/var/log/build -prune -o -type f -exec rm -f {} \;
rm -f /srv/maverick/var/log/vision_landing/last.log
rm -rf /srv/maverick/var/log/ros/fc/* /srv/maverick/var/log/ros/sitl/*
rm -f /srv/maverick/var/run/*
find /srv/maverick/var/log -type f -delete
find /run/log/journal -type f -delete
systemctl restart systemd-journald
journalctl --rotate
journalctl --vacuum-time=1s

# Re-seed dhclient
$(ip a |grep eth0 >/dev/null 2>&1) && dhclient eth0
$(ip a |grep wlan0 >/dev/null 2>&1) && dhclient wlan0

# Remove raspberry fs expansion marker
rm -f /etc/raspi-expandroot

echo "Maverick preparation complete"
#read -t10 -n1 -r -p 'Press any key in the next ten seconds to cancel shutdown...' key
#echo "Shutting down cleanly"
sudo sync
#sudo systemctl poweroff
echo "Please check the system is prepared as expected for imaging, then shutdown."
echo
