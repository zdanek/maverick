#!/usr/bin/env python
# This fact extracts as much hardware information out of a beaglebone as possble

import re, sys, subprocess, commands
import os.path
from xml.dom.minidom import parse, parseString

class Beagle(object):
    def __init__(self):
        self.data = {'present': 'no', 'emmcbooted': 'no', 'sdcard_present': 'no'}
        
    def cpudata(self):
        f = open('/sys/firmware/devicetree/base/model', 'r')
        for line in f:
            if re.search('BeagleBone Black', line):
                self.data['present'] = 'yes'
                self.data['model'] = 'BeagleBone Black'

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

    def storagedata(self):
        # Obtain the SD card size from proc
        f = open('/proc/partitions', 'r')
        for line in f:
            if re.search("mmcblk0$", line):
                self.data['rootdisksize'] = int(line.split()[2]) / 1024
            if re.search("mmcblk0p1$", line):
                partsize = int(line.split()[2]) / 1024
                try:
                    partpct = (self.data['rootdisksize'] / partsize) * 100
                    if partpct > 98:
                        self.data['rootdiskexpanded'] = 'yes'
                    else:
                        self.data['rootdiskexpanded'] = 'no'
                except:
                    pass
            if re.search("mmcblk0boot0", line):
                self.data['emmcbooted'] = 'yes'
                self.data['emmc_size'] = self.data['rootdisksize']
            if re.search("mmcblk1boot0", line):
                self.data['emmcbooted'] = 'no'
                self.data['sdcard_present'] = 'yes'
                self.data['sdcard_size'] = self.data['rootdisksize']
            try:
                if self.data['emmcbooted'] == 'yes' and re.search("mmcblk1$", line):
                    self.data['sdcard_present'] = 'yes'
                    self.data['sdcard_size'] = int(line.split()[2]) / 1024
                if self.data['emmcbooted'] == 'no' and re.search("mmcblk1$", line):
                    self.data['emmc_size'] = int(line.split()[2]) / 1024
            except:
                pass
        f.close()

#If we're being called as a command, instantiate and report
if __name__ == '__main__':
    # Check if model path available, if not exit
    if not os.path.isfile('/sys/firmware/devicetree/base/model'):
        print "beagle_present=no"
        sys.exit(1)
    
    beagle = Beagle()
    beagle.cpudata()
    if beagle.data['present'] == 'no':
        print "beagle_present=no"
        sys.exit(1)
        
    beagle.storagedata()
    # Finally, print the data out in the format expected of a fact provider
    if beagle.data:
        for key,val in beagle.data.items():
            print "beagle_%s=%s" % (key, val)
