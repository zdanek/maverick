#!/bin/bash

### NOTE: DO NOT RUN THIS UNLESS YOU KNOW EXACTLY WHAT IT DOES.
### IT MUST BE RUN AS ROOT
#
### IF YOU DO NOT UNDERSTAND WHAT THIS DOES, YOU ARE ALMOST GUARANTEED TO LOSE DATA BY RUNNING IT.
#
### This script attempts to prepare and compress partitions in preparation for imaging.
#
### Bits of logic liberally lifted from create-img.sh script by Badr BADRI © pythops

if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root"
    echo
    exit 1
fi

# Crude attempt to deal with positional arguments
if [[ $# -lt 2 ]] ; then
    echo "Missing required arguments:"
    echo " - Source Nano Image (eg. sdb)"
    echo " - Destination Build Directory (eg. /var/tmp/nano.creation)"
    echo
    echo "List of available devices:"
    lsblk |grep disk
    echo
    exit 1
fi
srcdisk=$1
dstdir=$2

mkdir -p $dstdir

# Download L4T
BSP=https://developer.nvidia.com/embedded/r32-2-3_Release_v1.0/t210ref_release_aarch64/Tegra210_Linux_R32.2.3_aarch64.tbz2
if [ ! "$(ls -A $dstdir)" ]; then
        printf "\e[32mDownload L4T...       "
        wget -qO- $BSP | tar -jxpf - -C $dstdir
	    rm $dstdir/Linux_for_Tegra/rootfs/README.txt
        printf "[OK]\n"
fi

mkdir /var/tmp/nano.srcdisk
mount /dev/${srcdisk}1 /var/tmp/nano.srcdisk

rsync -avz /var/tmp/nano.srcdisk/* $dstdir/Linux_for_Tegra/rootfs

printf "Create image... "
rootfs_size=$(du -hsBM $dstdir/Linux_for_Tegra/rootfs | awk '{print $1}')
rootfs_size=$(echo $((${rootfs_size%?} + 512))"M")
./create-jetson-nano-sd-card-image.sh -o maverick-nano.img -s $rootfs_size -r 200
printf "OK\n"

printf "\e[32mImage created successfully\n"
printf "Image location: $stdir/Linux_for_Tegra/maverick-nano.img\n"