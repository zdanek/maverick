#!/bin/bash

### NOTE: DO NOT RUN THIS UNLESS YOU KNOW EXACTLY WHAT IT DOES.
### IT SHOULD ABSOLUTELY NOT BE RUN INSIDE A RUNNING OS.
### IT MUST BE RUN FROM A LIVE BOOT OS.
### IT MUST BE RUN AS ROOT
#
### IF YOU DO NOT UNDERSTAND WHAT THIS DOES, YOU ARE ALMOST GUARANTEED TO LOSE DATA BY RUNNING IT.
#
### This script attempts to prepare and compress partitions in preparation for imaging.
#
### Bits of logic liberally lifted from raspi-img script by <kalle@friedrich.xyz> and <abracadabricx>

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
fsck -y /dev/${srcpath}

# Calculate blocks/sectors etc necessary for resizing
totalblocks=$(tune2fs -l /dev/$srcpath |grep 'Block count' |awk '{print $3}')
freeblocks=$(tune2fs -l /dev/$srcpath |grep 'Free blocks' |awk '{print $3}')
blocksize=$(tune2fs -l /dev/$srcpath |grep 'Block size' |awk '{print $3}')
usedblocks=$(expr ${totalblocks} - ${freeblocks})
newblocks=$(echo "${usedblocks} + ( ${usedblocks} / 5 )" |bc)
startsector=$(fdisk -l /dev/${srcdisk} |grep ${srcpath} |awk {'print $2'})
endsector=$(fdisk -l /dev/${srcdisk} |grep ${srcpath} |awk {'print $3'})
sectorsize=$(fdisk -l /dev/${srcdisk} |grep 'Sector size' |awk {'print $4'})
newsectors=$(echo "${newblocks} * ( ${blocksize} / ${sectorsize} )" |bc)
newendsector=$(echo "${startsector} + ${newsectors}" |bc)
echo "Total blocks: ${totalblocks}, Used blocks: ${usedblocks}, Free blocks: ${freeblocks}"
echo "New size of resized filesystem = Used blocks (${usedblocks}) + 5% = ${newblocks}"
echo

# Resize the filesystem first
echo "Resizing partition filesystem"
resize2fs -fp /dev/$srcpath ${newblocks}

echo
echo "Current size of partition: Start=${startsector}s, End=${endsector}s"
echo "New size of resized partition = Start=${startsector}s, End=${newsectors}s"
echo

# Then resize the partition
echo "Resizing partition"
parted /dev/$srcdisk unit s resizepart $srcpart $newendsector yes
echo "parted /dev/$srcdisk unit s resizepart $srcpart $newendsector yes"

# Finally, backup the disk to an image file
echo
echo "Creating image file ${dstfile}.xz from resized disk ${srcdisk}"
# dd if=/dev/${srcdisk} of=${dstfile} bs=${sectorsize} count=${newendsector} conv=noerror,sync status=progress
dd if=/dev/${srcdisk} bs=${sectorsize} iflag=fullblock count="${newendsector}" | nice -n 10 xz -6 --verbose --threads=0 > ${dstfile}.xz
echo
echo "Syncing kernel buffers"
sync
echo
echo "Done!"
echo