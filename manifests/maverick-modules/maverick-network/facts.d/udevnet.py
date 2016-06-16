#!/usr/bin/env python
# This fact extracts udev info for network interfaces

import os, re, sys, subprocess

class Udevnet(object):
    def __init__(self):
        self.data = {}
        self._data = {}
        
    def udevnet(self):
        # Parse udev database dump, extract network interfaces and add data into dicts keyed by int_field
        f = subprocess.check_output(['/sbin/udevadm', 'info', '-e'])
        data = f.split("\n")
        counter = 0
        for line in data:
            if re.search('^P:', line):
                r = re.search('^P:\s(.*)', line)
                counter += 1
                self._data[counter] = {}
            elif re.search('^E:', line):
                r = re.search('^E:\s(.*)=(.*)', line)
                if not r or not r.groups(0) or not r.groups(0)[0] or not r.groups(0)[1]: continue
                (key,val) = r.groups(0)[0],r.groups(0)[1]
                self._data[counter][key] = r.groups(0)[1]
        for index,_dat in self._data.items():
            if type(_dat) is dict and _dat['SUBSYSTEM'] == "net":
                for fx,_field in _dat.items():
                    self.data[_dat['INTERFACE']+'_'+str(fx).lower()] = _dat[fx]
    def runall(self):
        self.udevnet()

#If we're being called as a command, instantiate and report
if __name__ == '__main__':
    udev = Udevnet()
    udev.udevnet()
    # Don't run facter as a fact (obviously)
    # Finally, print the data out in the format expected of a fact provider
    if udev.data:
        for key,val in sorted(udev.data.items()):
           print "udevnet_%s=%s" % (key, val)
