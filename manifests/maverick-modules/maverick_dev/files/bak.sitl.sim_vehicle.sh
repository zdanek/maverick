#!/bin/bash

# Set defaults, can be overriden in /srv/maverick/data/config/sitl.conf
ENABLE=true
SCREEN_NAME=sitl
MAVPROXY_ARGS="--out=udpin:0.0.0.0:14560 --out=udpin:0.0.0.0:14561 --out=tcpin:0.0.0.0:5770"

[ ! -r /srv/maverick/data/config/sitl.conf ] || . /srv/maverick/data/config/sitl.conf

if [ "$ENABLE" == "false" ]; then
    echo "ENABLE flag is set to false, exiting"
    exit 0
fi

# Start mavproxy-sitl
# /usr/bin/screen -L -c /srv/maverick/data/config/sitl.screen.conf -S $SCREEN_NAME -d -m /srv/maverick/code/ardupilot/Tools/autotest/sim_vehicle.py -N -v $1 -m $MAVPROXY_ARGS
/usr/bin/screen -L -c /srv/maverick/data/config/sitl.screen.conf -S $SCREEN_NAME -d -m /srv/maverick/code/ardupilot/Tools/autotest/sim_vehicle.py -N --no-mavproxy -v $1
