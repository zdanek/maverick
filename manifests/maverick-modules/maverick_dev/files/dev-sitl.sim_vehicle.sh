#!/bin/bash

# Set defaults, can be overriden in /srv/maverick/data/config/dev-sitl.conf
ENABLE=true
SCREEN_NAME=dev-sitl

[ ! -r /srv/maverick/data/config/dev-sitl.conf ] || . /srv/maverick/data/config/dev-sitl.conf

if [ "$ENABLE" == "false" ]; then
    echo "ENABLE flag is set to false, exiting"
    exit 0
fi

# Start mavproxy-sitl
/usr/bin/screen -L -c /srv/maverick/data/config/dev-sitl.screen.conf -S $SCREEN_NAME -d -m /srv/maverick/code/ardupilot/Tools/autotest/sim_vehicle.py -N -v $1
