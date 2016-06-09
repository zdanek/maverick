#!/usr/bin/env python
# This fact extracts as much hardware information out of an odroid as possible

import re, sys, subprocess

class Odroid(object):
    def __init__(self):
        self.data = {'present': 'no'}
        
    def cpudata(self):
        # Define main data container
        f = open('/proc/cpuinfo', 'r')
        for line in f:
            r = re.search('^(.*)\s+:\s+(.*)', line)
            if not r or not r.groups(0) or not r.groups(0)[0] or not r.groups(0)[1]: continue
            (key,val) = r.groups(0)[0],r.groups(0)[1]
            if key == "Hardware": 
                self.data['model'] = val
                if re.search('ODROID', val):
                    self.data['present'] = 'yes'
            elif key == "Revision": 
                self.data['revision'] = val
            elif key == "Serial": 
                self.data['serial'] = val
        f.close()

    def storagedata(self):
        # Obtain the SD card size from proc
        f = open('/proc/partitions', 'r')
        for line in f:
            if re.search("mmcblk0$", line):
                self.data['sdsize'] = int(line.split()[2]) / 1024
        f.close()


#If we're being called as a command, instantiate and report
if __name__ == '__main__':
    odroid = Odroid()
    odroid.cpudata()
    if odroid.data['present'] == "no":
        print "odroid_present=no"
        sys.exit(1)
    odroid.storagedata()
    # Finally, print the data out in the format expected of a fact provider
    if odroid.data:
        for key,val in odroid.data.items():
            print "odroid_%s=%s" % (key, val)
