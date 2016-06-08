#!/bin/sh

# Set defaults, can be overriden in /srv/maverick/data/config/mavproxy-fc.conf
SCREEN_NAME=mavproxy-fc
MAVPROXY_PORT=/dev/ttyUSB0
MAVPROXY_BAUD=115200

[ ! -r /srv/maverick/data/config/mavproxy-fc.conf ] || . /srv/maverick/data/config/mavproxy-fc.conf

/usr/bin/screen -S $SCREEN_NAME -d -m /srv/maverick/.virtualenvs/dronekit-fc/bin/python /srv/maverick/.virtualenvs/dronekit-fc/bin/mavproxy.py --master $MAVPROXY_PORT --baud $MAVPROXY_BAUD --out=udpin:0.0.0.0:14550 --out=udpin:0.0.0.0:14551 --out=udp:127.0.0.1:14552 --out=udp:127.0.0.1:14553 --state-basedir=/srv/maverick/data/logs/mavproxy-fc
