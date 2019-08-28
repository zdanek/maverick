#!/usr/bin/env python3
# This fact detects which timesync mechanism is active

import os, re

data = {'ntp': 'no', 'chronyd': 'no', 'timesyncd': 'no'}

pids = [pid for pid in os.listdir('/proc') if pid.isdigit()]
for pid in pids:
    try:
        with open(os.path.join('/proc', pid, 'cmdline'), 'r') as f:
            pidcmd = f.read()
            if re.search('systemd-timesyncd', pidcmd):
                data['timesyncd'] = 'yes'
            if re.search('ntpd', pidcmd):
                data['ntpd'] = 'yes'
            if re.search('chronyd', pidcmd):
                data['chronyd'] = 'yes'
    except IOError: # proc has already terminated
        continue

# Finally, print(the data out in the format expected of a fact provider
for key,val in data.items():
    print("timesync_%s=%s" % (key, val))
	

