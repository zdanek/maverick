#!/usr/bin/env python
# This fact extracts as much hardware information out of an odroid as possible

import os, re, sys, subprocess

class Odroid(object):
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
            if key == "Hardware": 
                self.data['model'] = val
                if re.search('ODROID', val) or re.search('EXYNOS', val):
                    self.data['present'] = 'yes'
            elif key == "Revision": 
                self.data['revision'] = val
            elif key == "Serial": 
                self.data['serial'] = val
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

    def kernelbuild(self):
        try:
            if os.path.isdir('/srv/maverick/var/build/linux'):
                self.data['kernel4x_dir'] = "yes"
            else:
                self.data['kernel4x_dir'] = "no"
            with open ("/srv/maverick/var/build/linux/.kernelrelease", "r") as kernelrelease:
                kr=kernelrelease.readlines()
            if kr:
                self.data['kernel4x_release'] = kr[0].rstrip()
        except:
            self.data['kernel4x_dir'] = "no"
            self.data['kernel4x_release'] = "no"

    def runall(self):
        self.cpudata()
        self.storagedata()
        self.kernelbuild()

#If we're being called as a command, instantiate and report
if __name__ == '__main__':
    odroid = Odroid()
    odroid.cpudata()
    if odroid.data['present'] == "no":
        print "odroid_present=no"
        sys.exit(1)
    odroid.storagedata()
    odroid.kernelbuild()
    
    # Finally, print the data out in the format expected of a fact provider
    if odroid.data:
        for key,val in odroid.data.items():
            print "odroid_%s=%s" % (key, val)
