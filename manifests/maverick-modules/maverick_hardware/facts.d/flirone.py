#!/usr/bin/env python
# This fact detects if flir one is attached

import os, re, sys, subprocess, glob

class FlirOne(object):
    def __init__(self):
        self.data = {"present": "no", "model": None, "modelid": None}
        
    def detect(self):
        try:
            devdata = subprocess.check_output(["/usr/bin/lsusb", "-d", "09cb:"])
            for line in devdata.split("\n"):
                linefrags = line.split()
                for frag in linefrags:
                    if "09cb:" in frag:
                        self.data["present"] = "yes"
                        modelfrags = frag.split(":")
                        try:
                            self.data["modelid"] = modelfrags[1]
                            if modelfrags[1] == "1996":
                                self.data["model"] = "Flir One (Gen2)"
                        except:
                            pass
        except:
            pass

    def runall(self):
        self.detect()

#If we're being called as a command, instantiate and report
if __name__ == '__main__':
    flirone = FlirOne()
    flirone.detect()
    
    # Finally, print the data out in the format expected of a fact provider
    if flirone.data:
        for key,val in sorted(flirone.data.items()):
            print "flirone_%s=%s" % (key, val)
