#!/bin/bash

# Set python module paths
export PYTHONPATH=/srv/maverick/software/gstreamer/lib/python2.7/site-packages:/srv/maverick/software/opencv/lib/python2.7/dist-packages:/opt/ros/current/lib/python2.7/dist-packages

# Source ROS environment
source /srv/maverick/software/ros/current/setup.bash

# Source api config for this instance
[ ! -r /srv/maverick/config/web/api-$1.conf ] || . /srv/maverick/config/web/api-$1.conf
export API_PORT ROS_MASTER_URI

cd /srv/maverick/var/log/web/api/$1
/srv/maverick/code/maverick-api/maverick-api --configuration /srv/maverick/config/web/api-$1.json >/srv/maverick/var/log/web/api/$1/log 2>&1