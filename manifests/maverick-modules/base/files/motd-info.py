#!/usr/bin/python

import os, sys
# Insert facts directory into module path.  Not very clean, but they have to live there for puppet, so for reuse sake we import it here
sys.path.insert(0, '/srv/maverick/software/maverick/manifests/maverick-modules/maverick-baremetal/facts.d')
sys.path.insert(0, '/srv/maverick/software/maverick/manifests/maverick-modules/base/facts.d')
sys.dont_write_bytecode = True # This is to prevent .pyc files in facts.d directory

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

from baseos import Baseos
baseos = Baseos()
baseos.runall()

def twocols(col1, col2):
    return ctrls['fg_grey']+ col1+':' +ctrls['reset']+ctrls['bold']+ col2 +ctrls['reset']
    
print ctrls['reset']
print ctrls['bg_blue']+ctrls['fg_grey']+ctrls['bold']+ '** Maverick UAV Companion Computer **' +ctrls['reset']
print

if raspberry.data['present'] == 'yes':
    print twocols('Vendor', '\t\t\tRaspberry Pi')
    print twocols('Model', '\t\t\t' +raspberry.data['model'])
    print twocols('Physical RAM', '\t\t' +str(raspberry.data['physram']))
    print twocols('C/G Mem Split', '\t\t' +str(raspberry.data['memcpu'])+' / '+str(raspberry.data['memgpu']))
    print twocols('Firmware', '\t\t' +raspberry.data['fwdate'])
    print twocols('CPU Cores', '\t\t' +str(raspberry.data['cpucores']))
    print twocols('CPU Freq', '\t\t' +str(raspberry.data['cpufreq']) +' Mhz')
    print twocols('SD Card Size', '\t\t' +str(raspberry.data['sdsize']) +' Mb')

elif odroid.data['present'] == 'yes':
    print twocols('Vendor', '\t\t\tHardkernel')
    print twocols('Model', '\t\t\t' +odroid.data['model'])
    print twocols('CPU Cores', '\t\t' +str(odroid.data['cpucores']))
    print twocols('Memory', '\t\t\t' +odroid.data['memory'])
    print twocols('Swap', '\t\t\t' +odroid.data['swap'])
    print twocols('SD Card Size', '\t\t' +str(odroid.data['sdsize']) +' Mb')

elif beagle.data['present'] == 'yes':
    print twocols('Vendor', '\t\t\tBeaglebone')
    print twocols('Model', '\t\t\t' +beagle.data['model'])

else:
    print ctrls['bg_red']+ctrls['bold']+ctrls['fg_grey']+ 'Hardware platform not recognised' +ctrls['reset']
    sys.exit(0)

print " -------------------- "

print twocols('OS', '\t\t\t' +baseos.data['osname'])
print twocols('OS Version', '\t\t' +baseos.data['osrelease'])
print twocols('Architecture', '\t\t' +baseos.data['arch'])
print twocols('Kernel', '\t\t\t' +baseos.data['kernel'])
print twocols('Timezone', '\t\t' +baseos.data['timezone'])
print twocols('FQDN', '\t\t\t' +baseos.data['fqdn'])
print twocols('IP Address', '\t\t' +baseos.data['ipaddress'])
