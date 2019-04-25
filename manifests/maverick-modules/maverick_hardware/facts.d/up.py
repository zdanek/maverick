#!/usr/bin/env python3
# This fact extracts as much hardware information out of an Up board as possible

import os, re, sys, subprocess

class Up(object):
    def __init__(self):
        self.data = {'present': 'no'}
        
    def boarddata(self):
        count = 0
        
        try:
            with open('/sys/devices/virtual/dmi/id/board_vendor', 'r') as f:
                vendor = f.readline().strip()
                if vendor == "AAEON":
                    self.data['vendor'] = 'Aaeon'
                    self.data['present'] = 'yes'
                else:
                    self.data['vendor'] = None
        except:
            pass
        
        try:
            with open('/sys/devices/virtual/dmi/id/board_name', 'r') as f:
                board = f.readline().strip()
                if board == "UP-CHCR1":
                    self.data['model'] = 'Up Core'
                elif board == "UP-APL01":
                    self.data['model'] = 'Up Squared'
                elif board == "UP-CHT01":
                    self.data['model'] = 'Up'
                else:
                    self.data['model'] = None
        except:
            pass
        
        try:
            with open('/sys/devices/virtual/dmi/id/bios_version', 'r') as f:
                self.data['bios_version'] = f.readline().strip()
            with open('/sys/devices/virtual/dmi/id/bios_date', 'r') as f:
                self.data['bios_date'] = f.readline().strip()
        except:
            pass
        
        # Define main data container
        f = open('/proc/cpuinfo', 'r')
        for line in f:
            r = re.search('^(.*)\s+:\s+(.*)', line)
            if not r or not r.groups(0) or not r.groups(0)[0] or not r.groups(0)[1]: continue
            (key,val) = r.groups(0)[0],r.groups(0)[1]
            if key == "processor":
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
        self.boarddata()
        self.storagedata()

#If we're being called as a command, instantiate and report
if __name__ == '__main__':
    up = Up()
    up.boarddata()
    if up.data['present'] == "no":
        print("up_present=no")
        sys.exit(1)
    up.storagedata()

    # Finally, print the data out in the format expected of a fact provider
    if up.data:
        for key,val in up.data.items():
            print("up_%s=%s" % (key, val))
