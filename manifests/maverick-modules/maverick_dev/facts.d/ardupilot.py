#!/usr/bin/env python

# This fact finds the paths of compiled ardupilot firmwares
# otherwise compile attempts happen every run, which is unnecessary and very slow
import os,re

print "ardupilotfw_test=yes"
if os.path.isfile("/srv/maverick/code/ardupilot/ArduCopter/ArduCopter.elf"):
    print "ardupilotfw_arducopter=yes"
else:
    print "ardupilotfw_arducopter=no"

if os.path.isfile("/srv/maverick/code/ardupilot/ArduPlane/ArduPlane.elf"):
    print "ardupilotfw_arduplane=yes"
else:
    print "ardupilotfw_arduplane=no"

if os.path.isfile("/srv/maverick/code/ardupilot/APMrover2/APMrover2.elf"):
    print "ardupilotfw_apmrover2=yes"
else:
    print "ardupilotfw_apmrover2=no"

if os.path.isfile("/srv/maverick/code/ardupilot/AntennaTracker/AntennaTracker.elf"):
    print "ardupilotfw_antennatracker=yes"
else:
    print "ardupilotfw_antennatracker=no"


# Define main data container
waffiles = []
for root, dirs, files in os.walk("/srv/maverick/code/ardupilot/build"):
    for file in files:
        dirs = root.split("/")
        trpath = "/".join(dirs[-2::])
        file = os.path.join(trpath, file)
        if re.search("bin/", file):
            waffiles.append(file)

# Finally, print the data out in the format expected of a fact provider
if waffiles:
    print "waffiles="+str(",".join(waffiles))
else:
    print "waffiles=false"