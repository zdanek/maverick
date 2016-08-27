#!/usr/bin/env python
# This fact attempts to detect what network/connection manager systems are active

import os, re

data = {'connmand': 'no', 'wpa_supplicant': 'no', 'networkmanager': 'no'}

pids = [pid for pid in os.listdir('/proc') if pid.isdigit()]
for pid in pids:
    try:
        pidcmd = open(os.path.join('/proc', pid, 'cmdline'), 'rb').read()
        if re.search('wpa_supplicant', pidcmd):
        	data['wpa_supplicant'] = 'yes'
        if re.search('connmand', pidcmd):
        	data['connmand'] = 'yes'
        if re.search('NetworkManager', pidcmd):
            data['networkmanager'] = 'yes'
    except IOError: # proc has already terminated
        continue

# Finally, print the data out in the format expected of a fact provider
for key,val in data.items():
    print "netman_%s=%s" % (key, val)
	

