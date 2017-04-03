#!/bin/bash

source /srv/maverick/software/ros/current/setup.bash
[ ! -r /srv/maverick/data/config/ros/mavros-$1.conf ] || . /srv/maverick/data/config/ros/mavros-$1.conf

ROS_MASTER_URI="http://localhost:${ROS_PORT}" /srv/maverick/software/ros/current/bin/roslaunch mavros apm.launch fcu_url:=tcp://localhost:${MAVLINK_PORT}