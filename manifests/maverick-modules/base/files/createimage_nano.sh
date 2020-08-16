#!/bin/bash

### NOTE: DO NOT RUN THIS UNLESS YOU KNOW EXACTLY WHAT IT DOES.
### IT MUST BE RUN AS ROOT
#
### IF YOU DO NOT UNDERSTAND WHAT THIS DOES, YOU ARE ALMOST GUARANTEED TO LOSE DATA BY RUNNING IT.
#
### This script attempts to prepare and compress partitions in preparation for imaging.
### NB: This script is specific to JetPack 4.4 for the Jetson Nano
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

if [ ! -d $dstdir ]; then
    mkdir -p $dstdir
fi

## Download L4T
# BSP for Jetpack 4.2/4.3
# BSP=https://developer.nvidia.com/embedded/L4T/r32_Release_v4.2/t210ref_release_aarch64/Tegra210_Linux_R32.4.2_aarch64.tbz2
# BSP for Jetpack 4.4
BSP=https://developer.nvidia.com/embedded/L4T/r32_Release_v4.3/t210ref_release_aarch64/Tegra210_Linux_R32.4.3_aarch64.tbz2
if [ ! "$(ls -A $dstdir)" ]; then
        printf "\e[32mDownload L4T...       "
        wget -qO- $BSP | tar -jxpf - -C $dstdir
	    rm $dstdir/Linux_for_Tegra/rootfs/README.txt
        printf "[OK]\n"
fi

if [ ! -d /var/tmp/nano.srcdisk ]; then
    mkdir /var/tmp/nano.srcdisk
fi

mount /dev/${srcdisk}1 /var/tmp/nano.srcdisk

rsync -avz /var/tmp/nano.srcdisk/* $dstdir/Linux_for_Tegra/rootfs
cd $dstdir/Linux_for_Tegra
printf "Create image... "
rootfs_size=$(du -hsBM $dstdir/Linux_for_Tegra/rootfs | awk '{print $1}')
rootfs_size=$(echo $((${rootfs_size%?} + 512))"M")
#tools/jetson-disk-image-creator.sh -o $dstdir/maverick-nano.img -s $rootfs_size -r 200
tools/jetson-disk-image-creator.sh -o $dstdir/maverick-nano.img -b jetson-nano -r 300
umount /var/tmp/nano.srcdisk
printf "OK\n"

if [ -f $dstdir/maverick-nano.img ]; then
    bzip2 $dstdir/maverick-nano.img
    printf "\e[32mImage created successfully\n"
    printf "Image location: $dstdir/maverick-nano.img.bz2\n"
else
    printf "Error creating image\n"
fi