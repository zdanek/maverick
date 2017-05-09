#!/usr/bin/env python
import os, re, sys, subprocess

# Gather partition data
data = {'partitions': {}, 'blockdevices': {}, 'filesystems': {}, 'disks': []}

f = open('/proc/partitions', 'r')
for line in f:
    partfrags = line.split()
    if partfrags and partfrags[0] != 'major' and len(partfrags) == 4:
        data['partitions'][partfrags[3]] = partfrags[2]
f.close()

fsdata = subprocess.check_output(["/bin/lsblk", "-nl"])
for line in fsdata.split('\n'):
    partfrags = line.split()
    if partfrags and len(partfrags) == 7 and partfrags[5] == "part":
        data['blockdevices'][partfrags[6]] = partfrags[0]

# Gather filesystem data
fsdata = subprocess.check_output(["/bin/df"])
for line in fsdata.split('\n'):
    fsfrags = line.split()
    if fsfrags and fsfrags[0] != 'Filesystem' and len(fsfrags) == 6:
        try:
            data['filesystems'][fsfrags[5]] = { "size": fsfrags[1], "partition": data['blockdevices'][fsfrags[5]] }
        except:
            data['filesystems'][fsfrags[5]] = { "size": fsfrags[1], "partition": fsfrags[0] }

# Gatther disk data
diskdata = subprocess.check_output(["/bin/lsblk","-dn"])
for line in diskdata.split('\n'):
    diskfrags = line.split()
    if diskfrags: data['disks'].append(diskfrags[0])

# Work out root filesystem expanded status
try:
    rootpart = data['blockdevices']['/']
    rootsize = float(data['filesystems']['/']['size'])
    partsize = float(data['partitions'][rootpart])
    disksize = None
    data['rootpart_partition'] = rootpart
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
print "rootpart_partition="+str(data['rootpart_partition'])
try:
    print "rootpart_partno="+str(data['rootpart_partition'][-1])
except:
    print "rootpart_partno="
print "rootfs_expanded="+str(data['rootfs_expanded'])
