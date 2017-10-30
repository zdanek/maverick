#!/bin/bash

# Set defaults, can be overriden in /srv/maverick/data/config/mavlink/sitl.conf
FRAME=quad # Vehicle frame type - quad, heli, plane, quadplane, rover etc
LOCATION=CMAC # ~/code/ardupilot/Tools/autotest/locations.txt
CUSTOM_LOCATION= # Set to override LOCATION with a custom location
SPEEDUP=1 # Set simulation speedup (1 for wall clock time)
WIPE_EEPROM=false # Set to true to wipe eepom when SITL starts
GIMBAL=false # Set to true to enable MAVLink gimbal
TRACKER=false # Set to true to start an antenna tracker instance
TRACKER_LOCATION= # Set antenna tracker start location
VEHICLE_TYPE=ArduCopter # This should normally be overridden by sitl-vehicle.conf
SCREEN_NAME=sitl

[ ! -r /srv/maverick/data/config/mavlink/sitl.conf ] || . /srv/maverick/data/config/mavlink/sitl.conf
[ ! -r /srv/maverick/data/config/mavlink/sitl-vehicle.conf ] || . /srv/maverick/data/config/mavlink/sitl-vehicle.conf

cd /srv/maverick/data/mavlink/sitl
source /srv/maverick/.virtualenvs/sitl/bin/activate

# If jsbsim is installed, set the path
if [ -e /srv/maverick/software/jsbsim/bin ]; then
    export PATH=/srv/maverick/software/jsbsim/bin:$PATH
fi

# If custom location is specified then use that, otherwise location picked from list
if [ "x$CUSTOM_LOCATION" != "x" ]; then
    _LOCATION="--custom-location=$CUSTOM_LOCATION"
else
    _LOCATION="--location=$LOCATION"
fi

if $WIPE_EEPROM; then 
    _WIPE_EEPROM="--wipe-eeprom"
else
    _WIPE_EEPROM=""
fi

if $GIMBAL; then
    _GIMBAL="--mavlink-gimbal"
else
    _GIMBAL=""
fi

if $TRACKER; then
    _TRACKER="--tracker"
else
    _TRACKER=""
fi

if [ "x$TRACKER_LOCATION" != "x" ]; then
    _TRACKER_LOCATION="--tracker-location=$TRACKER_LOCATION"
else
    _TRACKER_LOCATION=""
fi

/usr/bin/screen -L sitl.log -S $SCREEN_NAME -d -m /srv/maverick/code/ardupilot/Tools/autotest/sim_vehicle.py --no-rebuild --no-mavproxy --vehicle=$VEHICLE_TYPE --use-dir=/srv/maverick/data/mavlink/sitl --frame=$FRAME --speedup=$SPEEDUP $_LOCATION $_WIPE_EEPROM $_GIMBAL $_TRACKER $_TRACKER_LOCATION  
