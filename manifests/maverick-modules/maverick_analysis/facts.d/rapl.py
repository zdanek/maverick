#!/usr/bin/env python
# This fact extracts as much hardware information out of an Up board as possible

import os, re, sys, subprocess

class RAPL(object):
    def __init__(self):
        self.data = {'present': 'no'}
        
    def rapldata(self):
        count = 0
        
        try:
            with open('/sys/class/powercap/intel-rapl/intel-rapl:0/enabled', 'r') as f:
                self.data['enabled'] = f.readline().strip()
            with open('/sys/class/powercap/intel-rapl/intel-rapl:0/energy_uj', 'r') as f:
                self.data['energyuj'] = f.readline().strip()
            if int(self.data['enabled']) == 1 and int(self.data['energuj']) > 1:
                self.data['present'] = "yes"
        except:
            pass
        
    def runall(self):
        self.rapldata()

#If we're being called as a command, instantiate and report
if __name__ == '__main__':
    rapl = RAPL()
    rapl.rapldata()

    # Finally, print the data out in the format expected of a fact provider
    if rapl.data:
        for key,val in rapl.data.items():
            print "rapl_%s=%s" % (key, val)
