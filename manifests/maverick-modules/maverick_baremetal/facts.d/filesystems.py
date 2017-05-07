#!/usr/bin/env python
import os, re, sys, subprocess

# Gather partition data
data = {'partitions': {}, 'filesystems': {}, 'disks': []}
f = open('/proc/partitions', 'r')
for line in f:
    partfrags = line.split()
    if partfrags and partfrags[0] != 'major' and len(partfrags) == 4:
        data['partitions'][partfrags[3]] = partfrags[2]
f.close()

# Gather filesystem data
fsdata = subprocess.check_output(["/bin/df"])
for line in fsdata.split('\n'):
    fsfrags = line.split()
    if fsfrags and fsfrags[0] != 'Filesystem' and len(fsfrags) == 6:
        data['filesystems'][fsfrags[5]] = { "size": fsfrags[1], "partition": fsfrags[0] }

# Gatther disk data
diskdata = subprocess.check_output(["/bin/lsblk","-dn"])
for line in diskdata.split('\n'):
    diskfrags = line.split()
    if diskfrags: data['disks'].append(diskfrags[0])

# Work out root filesystem expanded status
try:
    rootpart = data['filesystems']['/']['partition'].replace("/dev/","")
    rootsize = float(data['filesystems']['/']['size'])
    partsize = float(data['partitions'][rootpart])
    disksize = None
    data['rootpart_partiiton'] = rootpart
    for disk in data['disks']:
        if disk in rootpart:
            data['rootpart_device'] = disk
            disksize = float(data['partitions'][disk])
    part_ratio = (rootsize / partsize) * 100
    disk_ratio = (partsize / disksize) * 100
    data['rootpart_expanded'] = True if disk_ratio > 95 else False
    data['rootfs_expanded'] = True if part_ratio > 95 else False
except:
    data['rootpart_expanded'] = False
    data['rootpart_device'] = False
    data['rootpart_partition'] = False
    data['rootfs_expanded'] = False
    
# Finally, print the data out in the format expected of a fact provider
print "rootpart_expanded="+str(data['rootpart_expanded'])
print "rootpart_device="+str(data['rootpart_device'])
print "rootpart_partiiton="+str(data['rootpart_partiiton'])
print "rootfs_expanded="+str(data['rootfs_expanded'])
