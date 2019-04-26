#!/usr/bin/env python3

import os,sys,re
import subprocess

# First we have to force source profile, in order to load ROS env
command = ['bash', '-c', 'source /etc/profile && env']
proc = subprocess.Popen(command, stdout = subprocess.PIPE)
for line in proc.stdout:
  (key, _, value) = line.decode('utf-8').partition("=")
  if re.search("^ROS2", key):
    os.environ[key] = value.rstrip()
proc.communicate()

# Look for an arbitrary file to determine if ros is installed
try:
    rosroot = os.environ['ROS2_ROOT']
    if not os.path.isfile(os.path.join(rosroot,"package.xml")):
        print("ros2_installed=no")
        sys.exit(0)
except:
    print("ros2_installed=no")
    sys.exit(0)

print("ros2_installed=yes")
print("ros2_distribution="+str(os.environ['ROS_DISTRO']))
print("ros2_root="+str(os.environ['ROS_ROOT']))
print("ros2_etc="+str(os.environ['ROS_ETC_DIR']))