#!/bin/bash

source /srv/maverick/software/ros/current/setup.bash
[ ! -r /srv/maverick/data/config/ros/rosmaster-$1.conf ] || . /srv/maverick/data/config/ros/rosmaster-$1.conf

/srv/maverick/software/ros/current/bin/roscore -p ${ROS_PORT}
