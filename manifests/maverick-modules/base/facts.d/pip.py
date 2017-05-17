#!/usr/bin/env python
import subprocess

pips = ["global", "sitl", "fc"]
pippaths = {"global": "pip", "sitl": "/srv/maverick/.virtualenvs/sitl/bin/pip", "fc": "/srv/maverick/.virtualenvs/fc/bin/pip"}
modules = {}
for vpip in pips:
    modules[vpip] = {}
    
for vpip in pips:
    try:
        pipoutput = subprocess.Popen([pippaths[vpip], "freeze"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        _pips = pipoutput.communicate()[0].split()
        for _pip in _pips:
            [package,version] = _pip.split("==")
            modules[vpip][package.lower()] = version
    except Exception,e:
        pass

for vpip in pips:
    print "python_modules_"+vpip+"="+str(modules[vpip])
