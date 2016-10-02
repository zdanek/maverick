#!/bin/bash

# Set defaults, can be overriden in /srv/maverick/data/config/mavproxy-fc.conf
ENABLE=true
SCREEN_NAME=mavproxy-fc
MAVPROXY_PORT=/dev/ttyUSB0
MAVPROXY_BAUD=115200

[ ! -r /srv/maverick/data/config/mavproxy-fc.conf ] || . /srv/maverick/data/config/mavproxy-fc.conf

if [ "$ENABLE" == "false" ]; then
    echo "ENABLE flag is set to false, exiting"
    exit 0
fi

/usr/bin/screen -L -c /srv/maverick/data/config/mavproxy-fc.screen.conf -S $SCREEN_NAME -d -m /srv/maverick/.virtualenvs/dronekit-fc/bin/python /srv/maverick/.virtualenvs/dronekit-fc/bin/mavproxy.py --master $MAVPROXY_PORT --baud $MAVPROXY_BAUD --out=udpin:0.0.0.0:14550 --out=udpin:0.0.0.0:14551 --out=udp:127.0.0.1:14552 --out=udp:127.0.0.1:14553 --state-basedir=/srv/maverick/var/log/mavproxy-fc
