#!/usr/bin/env python
# This fact extracts as much hardware information out of a raspberry as possble
# Some ideas taken from: https://www.raspberrypi.org/forums/viewtopic.php?t=56906&p=429560

import re, sys, subprocess

class Raspberry(object):
    def __init__(self):
        self.data = {'present': 'no'}
        
    def cpudata(self):
        # Interrogate cpuinfo as the main source of hardware info for the raspberry
        revdata = {
            'Beta': ['Q1 2012', 'B (Beta)', '?', 256, 'Beta Board'],
            '0002':	['Q1 2012', 'B', '1.0', 256, ''],
            '0003': ['Q3 2012', 'B (ECN0001)', '1.0', 256, 'Fuses mod and D14 removed'],
            '0004':	['Q3 2012', 'B', '2.0', 256, '(Mfg by Sony)'],
            '0005': ['Q4 2012', 'B', '2.0', 256, '(Mfg by Qisda)'],
            '0006':	['Q4 2012',	'B', '2.0', 256, '(Mfg by Egoman)'],
            '0007':	['Q1 2013', 'A', '2.0', 256, '(Mfg by Egoman)'],
            '0008': ['Q1 2013', 'A', '2.0', 256, '(Mfg by Sony)'],
            '0009': ['Q1 2013', 'A', '2.0', 256, '(Mfg by Qisda)'],
            '000d': ['Q4 2012', 'B', '2.0', 512, '(Mfg by Egoman)'],
            '000e': ['Q4 2012', 'B', '2.0', 512, '(Mfg by Sony)'],
            '000f': ['Q4 2012', 'B', '2.0', 512, '(Mfg by Qisda)'],
            '0010': ['Q3 2014', 'B+', '1.0', 512, '(Mfg by Sony)'],
            '0011': ['Q2 2014', 'Compute Module', '1.0', 512, '(Mfg by Sony)'],
            '0012': ['Q4 2014', 'A+', '1.0', 256, '(Mfg by Sony)'],
            '0013': ['Q1 2015', 'B+', '1.2', 512, '?'],
            'a01041': ['Q1 2015', '2 Model B', '1.1', 1024, '(Mfg by Sony)'],
            'a21041': ['Q1 2015', '2 Model B', '1.1', 1024, '(Mfg by Embest, China)'],
            '900092': ['Q4 2015', 'Zero', '1.2', 512, '(Mfg by Sony)'],
            'a02082': ['Q1 2016', '3 Model B', '1.2', 1024, '(Mfg by Sony)']
        }
        count = 0
        f = open('/proc/cpuinfo', 'r')
        for line in f:
            r = re.search('^(.*)\s+:\s+(.*)', line)
            if not r or not r.groups(0) or not r.groups(0)[0] or not r.groups(0)[1]: continue
            (key,val) = r.groups(0)[0],r.groups(0)[1]
            if key == "Hardware": 
                self.data['cpu'] = val
            elif key == "processor":
                count += 1
            elif key == "Revision": 
                # If revision is prefixed with 1000 it means the board has been overvolted
                if re.match('1000', val):
                    rr = re.search('^1000(.*)',val)
                    self.data['overvolted'] = True
                    self.data['revision'] = rr.groups(0)[0]
                else:
                    self.data['overvolted'] = False
                    self.data['revision'] = val
                # Now work out the actual raspberry model from the revision
                try:
                    self.data['model'] = revdata[self.data['revision']][1]
                    self.data['reldate'] = revdata[self.data['revision']][0]
                    self.data['pcbrev'] = revdata[self.data['revision']][2]
                    self.data['physram'] = revdata[self.data['revision']][3]
                    self.data['notes'] = revdata[self.data['revision']][4]
                    self.data['present'] = "yes"
                except:
                    # Raspberry hardware not recognised, exit without returning any facts
                    self.data['raspberry_present'] = "no"
            elif key == "Serial\t": 
                self.data['serial'] = val
        f.close()
        self.data['cpucores'] = count
        
    def gpudata(self):
        try:
            # Interrogate the raspberry gpu for more info
            self.data['memcpu'] = subprocess.check_output(["/opt/vc/bin/vcgencmd", "get_mem arm"]).split("=")[1].rstrip()[:-1]
            self.data['memgpu'] = subprocess.check_output(["/opt/vc/bin/vcgencmd", "get_mem gpu"]).split("=")[1].rstrip()[:-1]
            self.data['mpg2codec'] = subprocess.check_output(["/opt/vc/bin/vcgencmd", "codec_enabled MPG2"]).split("=")[1].rstrip()
            self.data['vc1codec'] = subprocess.check_output(["/opt/vc/bin/vcgencmd", "codec_enabled WVC1"]).split("=")[1].rstrip()
            self.data['cpufreq'] = subprocess.check_output(["/opt/vc/bin/vcgencmd", "get_config arm_freq"]).split("=")[1].rstrip()
            self.data['ramfreq'] = subprocess.check_output(["/opt/vc/bin/vcgencmd", "get_config sdram_freq"]).split("=")[1].rstrip()
            self.data['l2cache'] = subprocess.check_output(["/opt/vc/bin/vcgencmd", "get_config disable_l2cache"]).split("=")[1].rstrip()
            fwdata = subprocess.check_output(["/opt/vc/bin/vcgencmd", "version"])
            fwdated = False
            for dat in fwdata.split("\n"):
                if not fwdated:
                    self.data['fwdate'] = dat
                    fwdated = True
                if re.match('version', dat):
                    self.data['fwversion'] = dat.split('version ')[1]
        except:
            pass

    def storagedata(self):
        # Obtain the SD card size from proc
        f = open('/proc/partitions', 'r')
        for line in f:
            if re.search("mmcblk0$", line):
                self.data['sdsize'] = int(line.split()[2]) / 1024
        f.close()

    def runall(self):
        self.cpudata()
        self.gpudata()
        self.storagedata()

#If we're being called as a command, instantiate and report
if __name__ == '__main__':
    raspberry = Raspberry()
    raspberry.cpudata()
    if raspberry.data['present'] == "no":
        print "raspberry_present=no"
        sys.exit(1)
    raspberry.gpudata()
    raspberry.storagedata()
    # Finally, print the data out in the format expected of a fact provider
    if raspberry.data:
        for key,val in raspberry.data.items():
            print "raspberry_%s=%s" % (key, val)

