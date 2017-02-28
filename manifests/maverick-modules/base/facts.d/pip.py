#!/usr/bin/env python
import subprocess

modules = {}
for vpip in ["/usr/bin/pip", "/srv/maverick/.virtualenvs/sitl/bin/pip", "/srv/maverick/.virtualenvs/fc/bin/pip"]:
    pipoutput = subprocess.Popen([vpip, "freeze"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    pips = pipoutput.communicate()[0].split()
    for pip in pips:
        [package,version] = pip.split("==")
        modules[package.lower()] = version

print "python_modules="+str(modules)