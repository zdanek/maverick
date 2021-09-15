#!/usr/bin/env python3

import os,sys,re
from os import path

if path.exists("/srv/maverick/software/ros2/current/ros2cli/bin/ros2"):
    print("ros2_installed=yes")
else:
    print("ros2_installed=no")
