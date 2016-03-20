#!/usr/bin/env python
# This fact extracts as much hardware information out of a beaglebone as possble

import re, sys, subprocess, commands
from xml.dom.minidom import parse, parseString

# Define main data container
data = {}

f = open('/sys/firmware/devicetree/base/model', 'r')
for line in f:
	if re.search('BeagleBone Black', line):
		data['present'] = 'yes'
		data['model'] = 'BeagleBone Black'
	else:
		print "beagle_present=no"
		sys.exit(1)

try:
	lshw_xml = commands.getoutput('/usr/bin/lshw -xml')
	lshw_dom = parseString(lshw_xml)
	nodes = lshw_dom = lshw_dom.childNodes
	for node in nodes:
		continue
		#print node
		#print '----'
except:
	pass

# Obtain the SD card size from proc
f = open('/proc/partitions', 'r')
for line in f:
    if re.search("mmcblk0$", line):
        data['disksize'] = int(line.split()[2]) / 1024
f.close()

# Finally, print the data out in the format expected of a fact provider
if data:
    for key,val in data.items():
        print "beagle_%s=%s" % (key, val)
	

