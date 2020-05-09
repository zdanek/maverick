#!/bin/bash

if [[ $# -eq 0 ]] ; then
    echo 'Error: apsitl.sh must be called with an argument: <instance-name>'
    exit 0
fi

# Set defaults, can be overriden in /srv/maverick/config/mavlink/sitl.conf
FRAME=quad # Vehicle frame type - quad, heli, plane, quadplane, rover etc
LOCATION=CMAC # ~/software/ardupilot/Tools/autotest/locations.txt
CUSTOM_LOCATION= # Set to override LOCATION with a custom location
SPEEDUP=1 # Set simulation speedup (1 for wall clock time)
WIPE_EEPROM=false # Set to true to wipe eepom when SITL starts
GIMBAL=false # Set to true to enable MAVLink gimbal
TRACKER=false # Set to true to start an antenna tracker instance
TRACKER_LOCATION= # Set antenna tracker start location
VEHICLE_TYPE=ArduCopter
SCREEN_NAME=$1
RCIN_PORT=5500
SITL_PORT=5600
INSTANCE=0
DEFAULT_PARAMS= # Default set of parameters to load if not already present

[ ! -r /srv/maverick/config/dev/apsitl_$1.conf ] || . /srv/maverick/config/dev/apsitl_$1.conf

cd /srv/maverick/data/dev/mavlink/apsitl_$1

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

if [ "x$DEFAULT_PARAMS" != "x" ]; then
    _DEFAULT_PARAMS="--add-param-file=$DEFAULT_PARAMS"
else
    _DEFAULT_PARAMS=""
fi

/usr/bin/screen -c /srv/maverick/config/dev/apsitl_$1.screen.conf -S $SCREEN_NAME -D -m /srv/maverick/software/python/bin/python3 /srv/maverick/software/ardupilot/Tools/autotest/sim_vehicle.py -A "--base-port=$SITL_PORT --rc-in-port=$RCIN_PORT" -I $INSTANCE --no-rebuild --no-mavproxy --vehicle=$VEHICLE_TYPE --use-dir=/srv/maverick/data/dev/mavlink/apsitl_$1 --frame=$FRAME --speedup=$SPEEDUP $_LOCATION $_WIPE_EEPROM $_GIMBAL $_TRACKER $_TRACKER_LOCATION $_DEFAULT_PARAMS  
