#!/bin/bash

# Set defaults, can be overriden in /srv/maverick/data/config/mavproxy-sitl.conf
ENABLE=true
SCREEN_NAME=mavproxy-sitl

[ ! -r /srv/maverick/data/config/mavproxy-sitl.conf ] || . /srv/maverick/data/config/mavproxy-sitl.conf

if [ "$ENABLE" == "false" ]; then
    echo "ENABLE flag is set to false, exiting"
    exit 0
fi

# Wait 5 seconds to ensure dev-sitl has fully started
sleep 5

# Start mavproxy-sitl
/usr/bin/screen -S $SCREEN_NAME -d -m /srv/maverick/.virtualenvs/dronekit-sitl/bin/python /srv/maverick/.virtualenvs/dronekit-sitl/bin/mavproxy.py --master tcp:127.0.0.1:5770 --sitl 127.0.0.1:5501 --out=udpin:0.0.0.0:14560 --out=udpin:0.0.0.0:14561 --out=udp:127.0.0.1:14562 --out=udp:127.0.0.1:14563 --state-basedir=/srv/maverick/var/log/mavproxy-sitl --cmd="arm uncheck all;"
#/srv/maverick/.virtualenvs/dronekit-sitl/bin/python /srv/maverick/.virtualenvs/dronekit-sitl/bin/mavproxy.py --master tcp:127.0.0.1:5770 --sitl 127.0.0.1:5501 --out=udpin:0.0.0.0:14560 --out=udpin:0.0.0.0:14561 --out=udp:127.0.0.1:14562 --out=udp:127.0.0.1:14563 --state-basedir=/srv/maverick/var/log/mavproxy-sitl --cmd="arm uncheck all;" >/var/tmp/mavproxy-sitl.out 2>&1