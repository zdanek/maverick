#!/bin/bash

if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root"
    echo
    exit 1
fi

if [ ! -f /etc/tegra-expandroot ]; then
    /usr/lib/nvidia/resizefs/nvresizefs.sh; echo 'done' > /etc/tegra-expandroot
    echo "Root partition/filesystem has been expanded, rebooting"
    /sbin/reboot
fi
