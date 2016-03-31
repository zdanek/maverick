#!/usr/bin/env python

import os.path
print "ardupilotfw_test=yes"
if os.path.isfile("/srv/maverick/code/dronekit-sitl/sitl-fw/ArduCopter/ArduCopter.elf"):
    print "ardupilotfw_arducopter=yes"
else:
    print "ardupilotfw_arducopter=no"

if os.path.isfile("/srv/maverick/code/dronekit-sitl/sitl-fw/ArduPlane/ArduPlane.elf"):
    print "ardupilotfw_arduplane=yes"
else:
    print "ardupilotfw_arduplane=no"

if os.path.isfile("/srv/maverick/code/dronekit-sitl/sitl-fw/APMrover2/APMrover2.elf"):
    print "ardupilotfw_apmrover2=yes"
else:
    print "ardupilotfw_apmrover2=no"

if os.path.isfile("/srv/maverick/code/dronekit-sitl/sitl-fw/AntennaTracker/AntennaTracker.elf"):
    print "ardupilotfw_antennatracker=yes"
else:
    print "ardupilotfw_antennatracker=no"
