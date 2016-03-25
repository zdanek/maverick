#!/usr/bin/env python
# This fact extracts as much hardware information out of an odroid as possble

import re, sys, subprocess

# Define main data container
data = {'present': 'no'}
f = open('/proc/cpuinfo', 'r')
for line in f:
    r = re.search('^(.*)\s+:\s+(.*)', line)
    if not r or not r.groups(0) or not r.groups(0)[0] or not r.groups(0)[1]: continue
    (key,val) = r.groups(0)[0],r.groups(0)[1]
    if key == "Hardware": 
        data['model'] = val
        if re.search('ODROID', val):
            data['present'] = 'yes'
    elif key == "Revision": 
        data['revision'] = val
    elif key == "Serial": 
        data['serial'] = val
f.close()

# Obtain the SD card size from proc
f = open('/proc/partitions', 'r')
for line in f:
    if re.search("mmcblk0$", line):
        data['sdsize'] = int(line.split()[2]) / 1024
f.close()

# Finally, print the data out in the format expected of a fact provider
if data:
    for key,val in data.items():
        print "odroid_%s=%s" % (key, val)
