#!/usr/bin/python

import sys
# Insert facts directory into module path.  Not very clean, but they have to live there for puppet, so for reuse sake we import it here
sys.path.insert(0, '/srv/maverick/software/maverick/manifests/maverick-modules/maverick-baremetal/facts.d')

ctrls = {}
ctrls['reset'] = '[0m'
ctrls['bold'] = '[1m'
ctrls['bg_black'] = '[40m'
ctrls['bg_red'] = '[41m'
ctrls['bg_blue'] = '[44m'
ctrls['fg_grey'] = '[37m'

from raspberry import Raspberry
raspberry = Raspberry()
raspberry.runall()

from beagle import Beagle
beagle = Beagle()
beagle.runall()

from odroid import Odroid
odroid = Odroid()
odroid.runall()

from camera import Camera
camera = Camera()
camera.runall()

def twocols(col1, col2):
    return ctrls['fg_grey']+ col1+':' +ctrls['reset']+ctrls['bold']+ col2 +ctrls['reset']
    
print ctrls['reset']
print ctrls['bg_blue']+ctrls['fg_grey']+ctrls['bold']+ '** Maverick UAV Companion Computer **' +ctrls['reset']

if raspberry.data['present'] == 'yes':
    print twocols('Vendor', '\t\t\tRaspberry Pi')
    print twocols('Model', '\t\t\t' +raspberry.data['model'])
    print twocols('Physical RAM', '\t\t' +str(raspberry.data['physram']))
    print twocols('C/G Mem Split', '\t\t' +str(raspberry.data['memcpu'])+' / '+str(raspberry.data['memgpu']))
    print twocols('Firmware', '\t\t' +raspberry.data['fwdate'])
    print twocols('CPU Freq', '\t\t' +str(raspberry.data['cpufreq']) +' Mhz')
    print twocols('SD Card Size', '\t\t' +str(raspberry.data['sdsize']) +' Mb')

elif odroid.data['present'] == 'yes':
    vendor = 'odroid'

elif beagle.data['present'] == 'yes':
    vendor = 'beagle'

else:
    print ctrls['bg_red']+ctrls['bold']+ctrls['fg_grey']+ 'Hardware platform not recognised' +ctrls['reset']
    sys.exit(0)
