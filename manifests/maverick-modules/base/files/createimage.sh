#!/bin/bash

### NOTE: DO NOT RUN THIS UNLESS YOU KNOW EXACTLY WHAT IT DOES.
### IT SHOULD ABSOLUTELY NOT BE RUN INSIDE A RUNNING OS.
### IT MUST BE RUN FROM A LIVE BOOT OS.
### IT MUST BE RUN AS ROOT
###
### IF YOU DO NOT UNDERSTAND WHAT THIS DOES, YOU ARE ALMOST GUARANTEED TO LOSE DATA BY RUNNING IT.

### This script attempts to prepare and compress partitions in preparation for imaging.

if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root"
    echo
    exit 1
fi

# Crude attempt to deal with positional arguments
if [[ $# -lt 2 ]] ; then
    echo "Missing required arguments:"
    echo " - Source Disk Device (eg. mmcblk0)"
    echo " - Destination filepath (eg. /var/tmp/raspberry-os.img)"
    echo
    echo "List of available devices:"
    lsblk
    echo
    exit 1
fi
srcdisk=$1
dstfile=$2

# First, attempt to resize the root partition
echo "Source Disk: ${srcdisk}"
echo "Source Partitions:"
lsblk /dev/${srcdisk}
if [ $? -ne 0 ]; then
    echo "Error, incorrect block device specified."
    echo
    exit 1
fi
read -e -p "Specify partition to shrink: " srcpart
srcpath=${srcdisk}p${srcpart}
echo "Partition data (${srcpath}):" $(lsblk -nOr /dev/${srcpath})

grep $srcpath /proc/mounts >/dev/null
prtcheck=$?
if [ $prtcheck -eq 0 ]; then
    echo
    echo "ERROR: Source partition is currently mounted.  You MUST run this against an unmounted disk/partition"
    echo
    #exit 1
fi

echo "Checking partition:"
#fsck -y /dev/${srcpath}
totalblocks=$(tune2fs -l /dev/$srcpath |grep 'Block count' |awk '{print $3}')
freeblocks=$(tune2fs -l /dev/$srcpath |grep 'Free blocks' |awk '{print $3}')
usedblocks=$(expr ${totalblocks} - ${freeblocks})
echo "Total blocks: ${totalblocks}, Used blocks: ${usedblocks}, Free blocks: ${freeblocks}"
newblocks=$(echo "${usedblocks} + ( ${usedblocks} / 10 )" |bc)
echo "New size of resized partition = Used blocks (${usedblocks}) + 10% = ${newblocks}"
echo "Resizing partition filesystem"
echo "resize2fs /dev/$srcpath ${newblocks}"
startsector=$(fdisk -l /dev/mmcblk0 |grep mmcblk0p2 |awk {'print $2'})

echo "Resizing partition"
echo "parted -s /dev/$srcdisk unit s resizepart $srcpart $newblocks yes"