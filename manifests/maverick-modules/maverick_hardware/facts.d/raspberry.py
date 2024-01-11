#!/usr/bin/env python3
# This fact extracts as much hardware information out of a raspberry as possble
# Some ideas taken from: https://www.raspberrypi.org/forums/viewtopic.php?t=56906&p=429560

import re, sys, subprocess

class Raspberry(object):
    def __init__(self):
        self.data = {'present': 'no'}

    def cpudata(self):
        # Interrogate cpuinfo as the main source of hardware info for the raspberry
        # Table info is here: http://elinux.org/RPi_HardwareHistory
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
            '0014': ['Q2 2014', 'Compute Module', '1.0', 512, '(Mfg by Embest)'],
            '0015': ['?', 'A+', '1.1', 512, '(Mfg by Embest)'],
            'a01040': ['Unknown', '2 Model B', '1.0', 1024, '(Mfg by Sony)'],
            'a01041': ['Q1 2015', '2 Model B', '1.1', 1024, '(Mfg by Sony)'],
            'a21041': ['Q1 2015', '2 Model B', '1.1', 1024, '(Mfg by Embest)'],
            'a22042': ['Q3 2016', '2 Model B (with BCM2837)', '1.2', 1024, '(Mfg by Embest)'],
            '900021': ['Q3 2016', 'A+', '1.1', 512, '(Mfg by Sony)'],
            '900032': ['Q2 2016?', 'B+', '1.2', 512, '(Mfg by Sony)'],
            '900092': ['Q4 2015', 'Zero', '1.2', 512, '(Mfg by Sony)'],
            '900093': ['Q4 2016', 'Zero', '1.3', 512, '(Mfg by Sony)'],
            '920093': ['Q4 2016?', 'Zero', '1.3', 512, '(Mfg by Embest)'],
            '1900093': ['Q4 2016?', 'Zero', '1.3', 512, '(Mfg by Embest)'],
            '9000c1': ['Q1 2017', 'Zero W', '1.1', 512, '(Mfg by Sony)'],
            '19000c1': ['Q1 2017', 'Zero W', '1.1', 512, '(Mfg by Sony)'],
            'a02082': ['Q1 2016', '3 Model B', '1.2', 1024, '(Mfg by Sony)'],
            'a020a0': ['Q1 2017', 'Compute Module 3 (and CM3 Lite)', '1.0', 1024, '(Mfg by Sony)'],
            'a22082': ['Q1 2016', '3 Model B', '1.2', 1024, '(Mfg by Embest)'],
            'a32082': ['Q4 2016', '3 Model B', '1.2', 1024, '(Mfg by Sony Japan)'],
            'a020d3': ['Q1 2018', '3 Model B+', '1.3', 1024, '(Mfg by Sony)'],
            '2a020d3': ['Q1 2018', '3 Model B+', '1.3', 1024, '(Mfg by Sony)'],
            '9020e0': ['Q4 2018', '3 Model A+', '1.0', 512,	'(Mfg by Sony)'],
            'a03111': ['Q2 2019', '4 Model B', '1.1', 1024,	'(Mfg by Sony)'],
            'b03111': ['Q2 2019', '4 Model B', '1.1', 2048,	'(Mfg by Sony)'],
            'b03112': ['Q2 2019', '4 Model B', '1.2', 2048, '(Mfg by Sony)'],
            'b03114': ['Q2 2020', '4 Model B', '1.4', 2048, '(Mfg by Sony)'],
            'c03111': ['Q2 2019', '4 Model B', '1.1', 4096,	'(Mfg by Sony)'],
            'c03112': ['Q2 2019', '4 Model B', '1.2', 4096, '(Mfg by Sony)'],
            'c03114': ['Q2 2020', '4 Model B', '1.4', 4096, '(Mfg by Sony)'],
            'd03114': ['Q2 2020', '4 Model B', '1.4', 8192, '(Mfg by Sony)'],
            '902120': ['Q1 2021', 'Zero 2 W', '1.6', 512, '']
        }
        count = 0
        f = open('/proc/cpuinfo', 'r')
        for line in f:
            r = re.search('^(.*)\s+:\s+(.*)', line)
            if not r or not r.groups(0) or not r.groups(0)[0] or not r.groups(0)[1]: continue
            (key,val) = r.groups(0)[0],r.groups(0)[1]
            if key == "Hardware":
                self.data['cpu'] = val
                if re.search('^BCM', line):
                    self.data['raspberry_present'] = "yes"
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
                    # self.data['raspberry_present'] = "no"
                    pass
            elif key == "Serial\t":
                self.data['serial'] = val
        f.close()
        self.data['cpucores'] = count

    def gpudata(self):
        try:
            # Interrogate the raspberry gpu for more info
            self.data['memcpu'] = subprocess.getoutput("vcgencmd get_mem arm").split("=")[1].rstrip()[:-1]
            self.data['memgpu'] = subprocess.getoutput("vcgencmd get_mem gpu").split("=")[1].rstrip()[:-1]
            self.data['mpg2codec'] = subprocess.getoutput("vcgencmd codec_enabled MPG2").split("=")[1].rstrip()
            self.data['vc1codec'] = subprocess.getoutput("vcgencmd codec_enabled WVC1").split("=")[1].rstrip()
            self.data['cpufreq'] = subprocess.getoutput("vcgencmd get_config arm_freq").split("=")[1].rstrip()
            self.data['ramfreq'] = subprocess.getoutput("vcgencmd get_config sdram_freq").split("=")[1].rstrip()
            self.data['l2cache'] = subprocess.getoutput("vcgencmd get_config disable_l2cache").split("=")[1].rstrip()
            fwdata = subprocess.getoutput("vcgencmd version")
            fwdated = False
            for dat in fwdata.split("\n"):
                if not fwdated:
                    self.data['fwdate'] = dat
                    fwdated = True
                if re.match('version', dat):
                    self.data['fwversion'] = dat.split('version ')[1]
        except Exception as e:
            print("Error: {}".format(repr(e)))

    def storagedata(self):
        # Obtain the SD card size from proc
        f = open('/proc/partitions', 'r')
        self.data['sdsize'] = None
        for line in f:
            if re.search("mmcblk0$", line):
                self.data['sdsize'] = int(line.split()[2]) / 1024
        f.close()

    def runall(self):
        self.cpudata()
        if self.data['present'] == "yes":
            self.gpudata()
            self.storagedata()

#If we're being called as a command, instantiate and report
if __name__ == '__main__':
    raspberry = Raspberry()
    raspberry.cpudata()
    if raspberry.data['present'] == "no":
        print("raspberry_present=no")
        sys.exit(1)
    raspberry.gpudata()
    raspberry.storagedata()
    # Finally, print the data out in the format expected of a fact provider
    if raspberry.data:
        for key,val in raspberry.data.items():
            print("raspberry_%s=%s" % (key, val))

