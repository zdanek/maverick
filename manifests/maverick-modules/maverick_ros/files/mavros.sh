#!/bin/bash

source /srv/maverick/software/ros/current/setup.bash
[ ! -r /srv/maverick/config/ros/mavros-$1.conf ] || . /srv/maverick/config/ros/mavros-$1.conf
export ROS_LOG_DIR

echo "Waiting for rosmaster to launch"
while ! nc -z localhost $ROS_PORT </dev/null; do sleep 5; done
echo "rosmaster running, launching mavros"

DT=$(date +"%Y-%m-%d-%H-%M")
ROS_MASTER_URI="http://localhost:${ROS_PORT}" /srv/maverick/software/ros/current/bin/roslaunch mavros ${MAVROS_LAUNCHER} fcu_url:=tcp://localhost:${MAVLINK_PORT} >/srv/maverick/var/log/ros/$1/mavros.$DT.log 2>&1