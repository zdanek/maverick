#!/bin/bash

# Set defaults, can be overriden in /srv/maverick/data/config/interface-.conf
RATE=24
CHAN5G=149
CHAN2G=13
POWER=10

# This script must run as root
if [[ $EUID -ne 0 ]]; then
    echo "This must be run as root"
    exit 1
fi

# Load defaults if they exist
[ ! -r /srv/maverick/data/config/network/interface-$2.conf ] || . /srv/maverick/data/config/network/interface-$2.conf

# Check for required wireless utils
if hash ifconfig; then
    ifconfig=$(which ifconfig)
else
    echo "Error: 'ifconfig' not found"
    exit 1
fi
if hash iw; then
    iw=$(which iw)
else
    echo "Error: 'iwconfig' not found"
    exit 1
fi

# Take action
case $1 in
    start)
        driverpath=$(readlink /sys/class/net/$2/device/driver)
        if [ -z $driverpath ]; then
            echo "Error determining interface $2 driver info"
            exit 1
        fi
        driver=$(basename $driverpath)
        
        case $driver in
            brcmfmac_sdio)
                echo "Raspberry onboard wifi chip does not support wifibroadcast"
                exit 1
                ;;
            rt2800usb)
                echo "Starting interface '$2' with chipset rt2800usb, channel $channel"
    			$ifconfig $2 down
    			$iw dev $2 set monitor otherbss fcsfail
    			$ifconfig $2 up
    			$iw reg set BO
    			$iw dev $2 set bitrates legacy-5 $RATE
    			$iw dev $2 set channel $CHAN5G
    			# $iw dev $2 set txpower fixed $POWER
			    ;;
            *)
                echo "This wifi chipset ($driver) has not yet been evaluated with wifibroadcast"
                exit 1
                ;;
        esac
        ;;
    stop)
        ;;
    *)
        echo "Usage: $0 {start|stop} <interface>"
        exit 1
        ;;
esac