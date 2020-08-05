#!/bin/bash

# Source ROS environment
source /srv/maverick/software/ros/current/setup.bash
. /etc/profile.d/31-maverick-ros-pythonpath.sh

# Source api config for this instance
[ ! -r /srv/maverick/config/web/api-env.$1.conf ] || . /srv/maverick/config/web/api-env.$1.conf
export API_PORT ROS_MASTER_URI

cd /srv/maverick/var/log/web/api/$1
/srv/maverick/software/python/bin/python3 /srv/maverick/software/maverick-api/maverick-api.py --config-file=/srv/maverick/config/web/maverick-api.$1.conf
