#!/usr/bin/env python3
# This fact extracts as much hardware information out of a joule as possible

import os, re, sys, subprocess

class Joule(object):
    def __init__(self):
        self.data = {'present': 'no'}
        
    def cpudata(self):
        count = 0
        # Define main data container
        f = open('/proc/cpuinfo', 'r')
        for line in f:
            r = re.search('^(.*)\s+:\s+(.*)', line)
            if not r or not r.groups(0) or not r.groups(0)[0] or not r.groups(0)[1]: continue
            (key,val) = r.groups(0)[0],r.groups(0)[1]
            if key == "model name" and (re.search("T5700", val) or re.search("T5500", val)): 
                if re.search("T5700", val):
                    self.data['model'] = "Intel Joule 570x"
                elif re.search("T5500", val):
                    self.data['model'] = "Intel Joule 550x"
                self.data['present'] = 'yes'
            elif key == "processor":
                count += 1
        self.data['cpucores'] = count
        f.close()
        f = open('/proc/meminfo', 'r')
        for line in f:
            r = re.search('^(.*):\s+(.*)', line)
            if not r or not r.groups(0) or not r.groups(0)[0] or not r.groups(0)[1]: continue
            (key,val) = r.groups(0)[0],r.groups(0)[1]
            if key == "MemTotal": 
                self.data['memory'] = val
            elif key == "SwapTotal":
                self.data['swap'] = val
        f.close()

    def storagedata(self):
        # Obtain the SD card size from proc
        f = open('/proc/partitions', 'r')
        for line in f:
            if re.search("mmcblk[01]$", line):
                self.data['sdsize'] = int(line.split()[2]) / 1024
        f.close()
        
    def runall(self):
        self.cpudata()
        self.storagedata()

#If we're being called as a command, instantiate and report
if __name__ == '__main__':
    joule = Joule()
    joule.cpudata()
    if joule.data['present'] == "no":
        print("joule_present=no")
        sys.exit(1)
    joule.storagedata()

    # Finally, print the data out in the format expected of a fact provider
    if joule.data:
        for key,val in joule.data.items():
            print("joule_%s=%s" % (key, val))
