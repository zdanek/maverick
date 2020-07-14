#!/bin/bash

if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root"
    echo
    exit 1
fi

if [ ! -f /etc/raspi-expandroot ]; then
    /usr/bin/raspi-config nonint do_expand_rootfs; echo 'done' > /etc/raspi-expandroot
    echo "Root partition/filesystem has been expanded, rebooting"
    /sbin/reboot
fi
