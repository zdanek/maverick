#!/bin/bash

source /srv/maverick/software/ros/current/setup.bash
[ ! -r /srv/maverick/data/config/ros/mavros-$1.conf ] || . /srv/maverick/data/config/ros/mavros-$1.conf

echo "Waiting a few seconds for rosmaster to fully launch"
sleep 5

DT=$(date +"%Y-%m-%d-%H-%M")
ROS_MASTER_URI="http://localhost:${ROS_PORT}" /srv/maverick/software/ros/current/bin/roslaunch mavros apm.launch fcu_url:=tcp://localhost:${MAVLINK_PORT} >/srv/maverick/var/log/mavros/$1/mavros.DT.log 2>&1