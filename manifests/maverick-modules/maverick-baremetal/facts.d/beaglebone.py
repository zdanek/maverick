#!/usr/bin/env python
# This fact extracts as much hardware information out of a beaglebone as possble

import re, sys, subprocess, commands
import os.path
from xml.dom.minidom import parse, parseString

# Check if model path available, if not exit
if not os.path.isfile('/sys/firmware/devicetree/base/model'):
	print "beagle_present=no"
	sys.exit(1)

# Define main data container
data = {'emmcbooted': 'no', 'sdcard_present': 'no'}

f = open('/sys/firmware/devicetree/base/model', 'r')
for line in f:
	if re.search('BeagleBone Black', line):
		data['present'] = 'yes'
		data['model'] = 'BeagleBone Black'
	else:
		print "beagle_present=no"
		sys.exit(1)

"""
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
"""

# Obtain the SD card size from proc
f = open('/proc/partitions', 'r')
for line in f:
    if re.search("mmcblk0$", line):
        data['rootdisksize'] = int(line.split()[2]) / 1024
    if re.search("mmcblk0boot0", line):
    	data['emmcbooted'] = 'yes'
    	data['emmc_size'] = data['rootdisksize']
    if re.search("mmcblk1boot0", line):
    	data['emmcbooted'] = 'no'
    	data['sdcard_present'] = 'yes'
    	data['sdcard_size'] = data['rootdisksize']
    try:
    	if data['emmcbooted'] == 'yes' and re.search("mmcblk1$"):
    		data['sdcard_present'] = 'yes'
    		data['sdcard_size'] = int(line.split()[2]) / 1024
    	if data['emmcbooted'] == 'no' and re.search("mmcblk1$"):
    		data['emmc_size'] = int(line.split()[2]) / 1024
    except:
    	pass

f.close()

# Finally, print the data out in the format expected of a fact provider
if data:
    for key,val in data.items():
        print "beagle_%s=%s" % (key, val)
	

