#!/usr/bin/env python
# This fact extracts as much hardware information out of video devices as possible

import os, re, sys, subprocess, glob

class Camera(object):
    def __init__(self):
        self.data = {"ocam": "no", "picam": "no"}
        
    def camhw(self):
        os.chdir("/dev")
        devices = []
        for file in glob.glob("video*"):
            devices.append(file)
        for device in devices:
            if not os.path.exists('/dev/'+device):
                self.data[device+"_present"] = "no"
                continue
            else:
                self.data[device+"_present"] = "yes"
            try:
                devdata = subprocess.check_output(["/usr/bin/v4l2-ctl", "-D", "-d", "/dev/"+device])
                for line in devdata.split("\n"):
                    r = re.search('^(.*)\s+:\s+(.*)', line)
                    if not r or not r.groups(0) or not r.groups(0)[0] or not r.groups(0)[1]: continue
                    (key,val) = r.groups(0)[0],r.groups(0)[1]
                    if re.search("Driver name", key): 
                        self.data[device+'_driver'] = val
                        if re.search("bm2835 mmal", val):
                            self.data['picam'] = 'yes'
                    elif re.search("Card type", key): 
                        self.data[device+'_type'] = val
                        if re.search('oCam', val):
                            self.data['ocam'] = "yes"
            except:
                pass
    def runall(self):
        self.camhw()

#If we're being called as a command, instantiate and report
if __name__ == '__main__':
    camera = Camera()
    camera.camhw()
    
    # Finally, print the data out in the format expected of a fact provider
    if camera.data:
        for key,val in sorted(camera.data.items()):
            print "camera_%s=%s" % (key, val)
