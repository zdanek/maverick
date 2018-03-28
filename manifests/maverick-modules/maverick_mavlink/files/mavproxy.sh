#!/bin/bash

# Set defaults, can be overriden in /srv/maverick/config/mavlink/mavproxy-<instance>.service.conf
ENABLE=true
SCREEN_NAME=mavlink-$1
MAVPROXY_PORT=/dev/ttyUSB0
MAVPROXY_BAUD=115200
MAVPROXY_ARGS="--out=udpin:0.0.0.0:14561 --out=tcpin:0.0.0.0:5761"

[ ! -r /srv/maverick/config/mavlink/mavproxy-$1.service.conf ] || . /srv/maverick/config/mavlink/mavproxy-$1.service.conf

if [ "$ENABLE" == "false" ]; then
    echo "ENABLE flag is set to false, exiting"
    exit 0
fi

/usr/bin/screen -c /srv/maverick/config/mavlink/mavproxy-$1.screen.conf -S $SCREEN_NAME -D -m /usr/local/bin/mavproxy.py --master $MAVPROXY_PORT --baud $MAVPROXY_BAUD $MAVPROXY_FLOW --state-basedir=/srv/maverick/var/log/mavlink-$1 $MAVPROXY_ARGS
