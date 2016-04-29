#!/usr/bin/env python
# This fact extracts as much hardware information out of video devices as possible

import os, re, sys, subprocess, glob

# Define main data container
data = {"ocam": "no", "raspicam": "no"}

os.chdir("/dev")
devices = []
for file in glob.glob("video*"):
    devices.append(file)

for device in devices:
    if not os.path.exists('/dev/'+device):
        data[device+"_present"] = "no"
        continue
    else:
        data[device+"_present"] = "yes"
    devdata = subprocess.check_output(["/usr/bin/v4l2-ctl", "-D", "-d", "/dev/"+device])
    for line in devdata.split("\n"):
        r = re.search('^(.*)\s+:\s+(.*)', line)
        if not r or not r.groups(0) or not r.groups(0)[0] or not r.groups(0)[1]: continue
        (key,val) = r.groups(0)[0],r.groups(0)[1]
        if re.search("Driver name", key): 
            data[device+'_driver'] = val
        elif re.search("Card type", key): 
            data[device+'_type'] = val
            if re.search('oCam', val):
                data['ocam'] = "yes"
            elif re.search('raspberry', val):
                data['raspicam'] = "yes"

# Finally, print the data out in the format expected of a fact provider
if data:
    for key,val in sorted(data.items()):
        print "camera_%s=%s" % (key, val)
