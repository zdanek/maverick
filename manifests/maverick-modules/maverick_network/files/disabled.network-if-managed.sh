#!/bin/bash

# This script must run as root
if [[ $EUID -ne 0 ]]; then
    echo "This must be run as root"
    exit 1
fi

if [ ! -f /srv/maverick/config/network/interface-$1.conf ]; then
    echo "Error: config file does not exist"
    exit 0
fi

# Load defaults if they exist
[ ! -r /srv/maverick/config/network/interface-$1.conf ] || . /srv/maverick/config/network/interface-$1.conf

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
else
    driver=$(basename $driverpath)
fi

### Actions
# Set country regulations
if [ ! -z $COUNTRY ]; then
    $iw reg set $COUNTRY >/srv/maverick/var/log/network/interface-$1.log 2>&1
fi

# Set legacy bitrates
if [ ! -z $RATE ]; then
$(
    $ifconfig $1 down
    $iw dev $1 set bitrates legacy-5 $RATE
    $iw dev $1 set bitrates legacy-2.4 $RATE
    $ifconfig $1 up
)
fi

# Set frequency
if [ ! -z $CHANNEL ]; then
    $iw dev $1 set channel $CHANNEL >>/srv/maverick/var/log/network/interface-$1.log 2>&1
fi

# Set tx power
if [ ! -z $POWER ]; then
    $iw dev $1 set txpower fixed $POWER >>/srv/maverick/var/log/network/interface-$1.log 2>&1
fi
