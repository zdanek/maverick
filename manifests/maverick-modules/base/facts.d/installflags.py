#!/usr/bin/env python

import fnmatch
import os
import re

data = {
    "install_flag_gstreamer": False,
    "install_flag_opencv": False,
    "install_flag_ros": False
}

try:
    for file in os.listdir('/srv/maverick/var/build'):
        if fnmatch.fnmatch(file, '.install_flag*'):
            data[re.sub("^\.", "", file)] = True
except:
    pass

for flag,value in data.iteritems():
    print flag+"="+str(value)