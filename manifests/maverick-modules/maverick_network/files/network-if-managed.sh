#!/bin/bash

# Set defaults, can be overriden in /srv/maverick/data/config/network/interface-<name>.conf
# COUNTRY=BO
# RATE=54
# CHANNEL=3
# POWER=10

# This script must run as root
if [[ $EUID -ne 0 ]]; then
    echo "This must be run as root"
    exit 1
fi

# Load defaults if they exist
[ ! -r /srv/maverick/data/config/network/interface-$1.conf ] || . /srv/maverick/data/config/network/interface-$1.conf

# Check for required wireless utils
if hash ifconfig; then
    ifconfig=$(which ifconfig)
else
    echo "Error: 'ifconfig' not found"
    exit 0
fi
if hash iw; then
    iw=$(which iw)
else
    echo "Error: 'iw' not found"
    exit 0
fi

# Work out interface driver
driverpath=$(readlink /sys/class/net/$1/device/driver)
if [ -z $driverpath ]; then
    echo "Error determining interface $1 driver info"
    # exit 1
fi
driver=$(basename $driverpath)

### Actions

# Set country regulations
if [ ! -z $COUNTRY ]; then
    $iw reg set BO
fi

# Set legacy bitrates
if [ ! -z $RATE ]; then
    $ifconfig down $1
    $iw dev $1 set bitrates legacy-5 $RATE
    $iw dev $1 set bitrates legacy-2.4 $RATE
    $ifconfig up $1
fi

# Set frequency
if [ ! -z $CHANNEL ]; then
    $iw dev $1 set channel $CHANNEL
fi

# Set tx power
if [ ! -z $POWER ]; then
    $iw dev $1 set txpower fixed $POWER
fi
