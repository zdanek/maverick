#!/usr/bin/env python

import fnmatch
import os
import re

installflags = []

try:
    for file in os.listdir('/srv/maverick/var/build'):
        if fnmatch.fnmatch(file, '.install_flag*'):
            installflags.append(re.sub("^\.", "", file))
except:
    pass

if installflags:
    print "installflags="+str(",".join(installflags))
else:
    print "installflags=false"
