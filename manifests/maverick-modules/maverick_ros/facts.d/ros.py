#!/usr/bin/env python

import os,sys,re
import subprocess

# First we have to force source profile, in order to load ROS env
command = ['bash', '-c', 'source /etc/profile && env']
proc = subprocess.Popen(command, stdout = subprocess.PIPE)
for line in proc.stdout:
  (key, _, value) = line.partition("=")
  if re.search("^ROS", key):
      os.environ[key] = value.rstrip()
proc.communicate()

# Look for an arbitrary file to determine if ros is installed
try:
    rosroot = os.environ['ROS_ROOT']
    if not os.path.isfile(os.path.join(rosroot,"package.xml")):
        print "ros_installed=no"
        sys.exit(0)
except:
    print "ros_installed=no"
    sys.exit(0)

print "ros_installed=yes"
print "ros_distribution="+str(os.environ['ROS_DISTRO'])
print "ros_root="+str(os.environ['ROS_ROOT'])
print "ros_etc="+str(os.environ['ROS_ETC_DIR'])