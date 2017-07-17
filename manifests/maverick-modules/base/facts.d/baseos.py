#!/usr/bin/env python
# This fact extracts as much OS information as possible

import os, re, sys, subprocess

class Baseos(object):
    def __init__(self):
        self.data = {'baseos_present': 'yes'}
        
    def facter(self):
        # Define main data container
        f = subprocess.check_output(['facter'])
        data = f.split("\n")
        for line in data:
            r = re.search('^(.*)\s+=>\s+(.*)', line)
            if not r or not r.groups(0) or not r.groups(0)[0] or not r.groups(0)[1]: continue
            (key,val) = r.groups(0)[0],r.groups(0)[1]
            if key == "timezone": 
                self.data['timezone'] = val
            elif key == "operatingsystem" and not self.data['osname']: 
                self.data['osname'] = val
            elif key == "lsbdistid":
                self.data['osname'] = val
            elif key == "operatingsystemrelease": 
                self.data['osrelease'] = val
            elif key == "kernelrelease":
                self.data['kernel'] = val
            elif key == "ipaddress":
                self.data['ipaddress'] = val
            elif key == "fqdn":
                self.data['fqdn'] = val
            elif key == "architecture":
                self.data['arch'] = val
            elif key == "is_virtual":
                self.data['is_virtual'] = val
            elif key == "virtual":
                self.data['virtual'] = val
            elif key == "processorcount":
                self.data['processorcount'] = val
            elif key == "memorysize":
                self.data['memorysize'] = val

    def runall(self):
        self.facter()

#If we're being called as a command, instantiate and report
if __name__ == '__main__':
    baseos = Baseos()
    # Don't run facter as a fact (obviously)
    # Finally, print the data out in the format expected of a fact provider
    if baseos.data:
        for key,val in baseos.data.items():
            print "baseos_%s=%s" % (key, val)
