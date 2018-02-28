#!/bin/bash

# Set defaults, can be overriden in /srv/maverick/config/dev/px4sitl.conf
# More info here: https://dev.px4.io/en/simulation/
CONFIGURATION_TARGET=posix_sitl_default
VIEWER_MODEL_DEBUGGER=gazebo
SCREEN_NAME=px4sitl

[ ! -r /srv/maverick/config/dev/px4sitl.conf ] || . /srv/maverick/config/dev/px4sitl.conf

cd /srv/maverick/code/px4
/usr/bin/screen -c /srv/maverick/config/dev/px4sitl.screen.conf -S $SCREEN_NAME -D -m /usr/bin/make $CONFIGURATION_TARGET $VIEWER_MODEL_DEBUGGER
