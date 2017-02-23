#!/bin/bash

# Set defaults, can be overriden in /srv/maverick/data/config/mavlinkrouter-<instance>.service.conf
ENABLE=true
SCREEN_NAME=mavlink-$1
SOURCE=/dev/ttyAMA0

[ ! -r /srv/maverick/data/config/mavlinkrouter-$1.service.conf ] || . /srv/maverick/data/config/mavlinkrouter-$1.service.conf

if [ "$ENABLE" == "false" ]; then
    echo "ENABLE flag is set to false, exiting"
    exit 0
fi

/srv/maverick/software/mavlinkrouter/bin/mavlink-router -c /srv/maverick/data/config/mavlinkrouter-$1.conf $SOURCE
