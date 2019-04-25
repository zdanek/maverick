#!/usr/bin/env python3
# This fact detects if seek thermal is attached

import os, re, sys, subprocess, glob

class Seek(object):
    def __init__(self):
        self.data = {"present": "no", "model": None, "modelid": None}
        
    def detect(self):
        try:
            devdata = subprocess.check_output(["/usr/bin/lsusb", "-d", "289d:"])
            for line in devdata.split("\n"):
                linefrags = line.split()
                for frag in linefrags:
                    if "289d:" in frag:
                        self.data["present"] = "yes"
                        modelfrags = frag.split(":")
                        try:
                            self.data["modelid"] = modelfrags[1]
                            if modelfrags[1] == "0010":
                                self.data["model"] = "Compact"
                            if modelfrags[1] == "0011":
                                self.data["model"] = "Compact Pro"
                        except:
                            pass
        except:
            pass

    def runall(self):
        self.detect()

#If we're being called as a command, instantiate and report
if __name__ == '__main__':
    seek = Seek()
    seek.detect()
    
    # Finally, print the data out in the format expected of a fact provider
    if seek.data:
        for key,val in sorted(seek.data.items()):
            print("seekthermal_%s=%s" % (key, val))
