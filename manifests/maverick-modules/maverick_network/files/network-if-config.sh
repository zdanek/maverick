#!/bin/bash

# Set defaults, can be overriden in /srv/maverick/config/network/interface-<int>.conf
RATE=24
BAND=2 # 2 (2.4ghz) or 5 (5ghz)
CHANNEL=13 # eg. 149 for 5ghz, see https://en.wikipedia.org/wiki/List_of_WLAN_channels
CHANTYPE="HT40+" # [HT20|HT40+|HT40-]
POWER=10
COUNTRY=BO

if [ -z $2 ]; then
    TYPE="managed"
else
    TYPE=$2
fi

# This script must run as root
if [[ $EUID -ne 0 ]]; then
    echo "This must be run as root"
    exit 1
fi

# Load defaults if they exist
[ ! -r /srv/maverick/config/network/interface-$1.conf ] || . /srv/maverick/config/network/interface-$1.conf

# Check for required wireless utils
if hash ip; then
    ipcmd=$(which ip)
else
    echo "Error: ip not found"
    exit 1
fi
if hash iw; then
    iw=$(which iw)
else
    echo "Error: iw not found"
    exit 1
fi
if hash iwconfig; then
    iwconfig=$(which iwconfig)
else
    echo "Error: iwconfig not found"
    exit 1
fi

# Take action
driverpath=$(readlink /sys/class/net/$1/device/driver)
if [ -z $driverpath ]; then
    echo "Error determining interface $1 driver info"
    exit 1
fi
driver=$(basename $driverpath)

if [[ $TYPE == "monitor" ]]; then
    case $driver in
        brcmfmac_sdio)
            echo "Raspberry onboard wifi chip does not support wifibroadcast"
            exit 1
            ;;
        rt2800usb)
            echo "Starting interface '$1' with chipset rt2800usb, channel $CHANNEL"
		    ;;
        *)
            echo "This wifi chipset ($driver) has not yet been evaluated with wifibroadcast"
            echo "Proceeding, but may fail"
            ;;
    esac
fi

chanset_return=1
safety_counter=0
while [[ $chanset_return -ne 0 ]] && [[ $safety_counter -lt 10 ]]; do
	$ipcmd link set $1 down
    if [[ $TYPE == "monitor" ]]; then
		$iw dev $1 set type monitor
		$iw dev $1 set monitor otherbss fcsfail
	elif [[ $TYPE == "managed" ]]; then
		$iw dev $1 set type managed
	fi
    if [[ $BAND -eq 2 ]]; then
        echo "Setting bitrate band 2.4: ${RATE}"
		$iw dev $1 set bitrates legacy-2.4 $RATE
    elif [[ $BAND -eq 5 ]]; then
        echo "Setting bitrate band 5: ${RATE}"
		$iw dev $1 set bitrates legacy-5 $RATE
    fi        
	$ipcmd link set $1 up
	echo "Setting channel: ${CHANNEL} ${CHANTYPE}"
    $iw dev $1 set channel $CHANNEL $CHANTYPE
    chanset_return=$?
    if [[ $chanset_return -ne 0 ]]; then
        $iwconfig $1 channel $CHANNEL
        chanset_return=$?
    fi
    echo "chanset_return: ${chanset_return}"
	safety_counter=$((safety_counter + 1))
	echo "safety_counter: ${safety_counter}"
done
echo "Setting country: ${COUNTRY}"
$iw reg set $COUNTRY
echo "Setting Power: ${POWER}"
$iw dev $1 set txpower fixed $POWER

# Test that we have an ipv4 address on our interface, if not trigger a dhclient request
sleep 5
ip4a=$($ipcmd -4 a show $1)
if [[ -z $ip4a ]]; then
    echo "No ipv4 address, triggering dhclient request"
    dhclient $1
fi

